#!/usr/bin/env python3
"""
can_relay.py - runs ON THE RASPBERRY PI next to the gimbal. Python standard library only.

It relays CAN frames between can0 and ONE laptop (app.py --pi <this-pi>) over TCP, using can-utils
(`candump` / `cansend`: proven on this Pi, python-can was not). It knows nothing about angles:
soft limits etc. are enforced in the app.

    python3 can_relay.py --allow <laptop-ip>          # serve on port 7070
    python3 can_relay.py --selftest                   # read 0x0043 (node id) from node 1 and node 2, exit
    python3 can_relay.py --sim --port 7071            # laptop testing only: simulated drives (sim_hw.py)

Protocol: newline-delimited JSON over TCP (default port 7070).
    laptop -> Pi   {"op":"tx","id":"641","data":"0083"}     send one frame
                   {"op":"hb"}                               heartbeat (the app sends one every 0.5 s)
    Pi -> laptop   {"op":"rx","id":"5C1","data":"0000123E3E","t":<pi time>}   every frame from 0x5C1/0x5C2
                   {"op":"hello",...}  on connect          {"op":"err","msg":...}  refused / failed

Rules enforced here:
  * TX only to 0x641 (node 1, X) and 0x642 (node 2, Y).
  * Writes to 0x0050 / 0x0052 / 0x0053 / 0x00E8 (drive flash / limits) are refused.
  * One client at a time (--allow restricts which IPs may connect).
  * WATCHDOG (does not depend on the network): if the client sends nothing for 2.0 s, or disconnects,
    the relay itself sends 0x84 STOP to node 1 and node 2, logs it and keeps running.
"""
import argparse
import json
import logging
import os
import queue
import re
import shutil
import socket
import subprocess
import sys
import threading
import time

VERSION = 1
DEFAULT_PORT = 7070
WATCHDOG_S = 2.0
ALLOWED_TX = frozenset({0x641, 0x642})
FORWARD_RX = frozenset({0x5C1, 0x5C2})
NEVER_WRITE = frozenset({0x0050, 0x0052, 0x0053, 0x00E8})      # drive flash / limit registers
STOP_FRAMES = ((0x641, bytes.fromhex("0084")), (0x642, bytes.fromhex("0084")))
IP_LINK_UP = "sudo ip link set can0 up type can bitrate 1000000 restart-ms 100"
_CANDUMP = re.compile(r"^\(\s*([\d.]+)\)\s+(\S+)\s+([0-9A-Fa-f]{3,8})#([0-9A-Fa-f]*)\s*$")

log = logging.getLogger("can_relay")


def parse_candump(line: str):
    """'(1699999999.123456) can0 5C1#0000123E' -> (t, 0x5C1, bytes). Remote/FD frames -> None."""
    m = _CANDUMP.match(line.strip())
    if not m or len(m.group(4)) % 2:
        return None
    return float(m.group(1)), int(m.group(3), 16), bytes.fromhex(m.group(4))


def check_frame(can_id: int, data: bytes):
    """None if the laptop may send this frame, else the reason it is refused."""
    if can_id not in ALLOWED_TX:
        return f"id 0x{can_id:03X} not allowed (only 0x641 / 0x642)"
    if len(data) > 8:
        return "more than 8 data bytes"
    if len(data) >= 2:
        index = int.from_bytes(data[:2], "big")
        if index in NEVER_WRITE and len(data) > 2:
            return f"write to 0x{index:04X} refused (drive flash / limit register)"
    return None


# ------------------------------------------------------------------------------------------- CAN backends
class CanUtilsBus:
    """candump -L (reader thread) + cansend per frame."""

    def __init__(self, channel="can0"):
        self.channel = channel
        self.name = f"can-utils {channel}"
        for tool in ("candump", "cansend"):
            if shutil.which(tool) is None:
                raise RuntimeError(f"'{tool}' not found. Install can-utils:  sudo apt install can-utils")
        state = self.link_state()
        if state is not None and state not in ("up", "unknown"):
            raise RuntimeError(f"{channel} is {state}. Bring it up first:\n    {IP_LINK_UP}")
        self.on_frame = None
        self.proc = subprocess.Popen(["candump", "-L", f"{channel},5C1:7FF,5C2:7FF"], stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, text=True, bufsize=1)
        time.sleep(0.3)
        if self.proc.poll() is not None:
            err = (self.proc.stderr.read() or "").strip()
            raise RuntimeError(f"candump could not open {channel}: {err or 'exited'}\n"
                               f"Bring the CAN bus up first:\n    {IP_LINK_UP}")
        threading.Thread(target=self._read, name="candump", daemon=True).start()

    def link_state(self):
        try:
            with open(f"/sys/class/net/{self.channel}/operstate") as f:
                return f.read().strip()
        except OSError:
            return None if not sys.platform.startswith("linux") else "missing"

    def _read(self):
        for line in self.proc.stdout:
            f = parse_candump(line)
            if f is not None and self.on_frame:
                self.on_frame(*f)
        log.error("candump stopped (is %s up?  %s)", self.channel, IP_LINK_UP)

    def alive(self):
        return self.proc.poll() is None

    def send(self, can_id, data):
        frame = f"{can_id:03X}#{bytes(data).hex().upper()}"
        r = subprocess.run(["cansend", self.channel, frame], capture_output=True, text=True, timeout=2)
        if r.returncode != 0:
            raise OSError(f"cansend {frame} failed: {(r.stderr or '').strip()} (is {self.channel} up? {IP_LINK_UP})")

    def close(self):
        if self.proc.poll() is None:
            self.proc.terminate()


class SimBus:
    """Laptop testing only: simulated eRob drives from sim_hw.py (same frames as the real bus)."""

    def __init__(self):
        from sim_hw import SimCanLink
        self.link = SimCanLink()
        self.name = "SIMULATED can0 (sim_hw)"
        self.on_frame = None
        self.sent = []                     # (t, id, data) for tests
        self._stop = threading.Event()
        threading.Thread(target=self._read, name="simbus", daemon=True).start()

    def _read(self):
        while not self._stop.is_set():
            f = self.link.recv(0.01)
            if f is not None and self.on_frame:
                self.on_frame(time.time(), f[0], f[1])

    def alive(self):
        return True

    def send(self, can_id, data):
        self.sent.append((time.time(), can_id, bytes(data)))
        self.link.send(can_id, data)

    def close(self):
        self._stop.set()


# ------------------------------------------------------------------------------------------- relay
class Relay:
    def __init__(self, bus, host="0.0.0.0", port=DEFAULT_PORT, allow=(), watchdog_s=WATCHDOG_S):
        self.bus, self.host, self.port = bus, host, port
        self.allow = set(allow or ())
        self.watchdog_s = watchdog_s
        self.client = None                 # (socket, addr)
        self.last_rx = 0.0                 # monotonic time of the last line from the client
        self.armed = False                 # watchdog armed (a client has spoken since the last STOP)
        self.stops = 0                     # watchdog STOPs sent (tests read this)
        self._wlock = threading.Lock()     # socket writes
        self._clock = threading.Lock()     # client changes
        self._stop = threading.Event()
        self._srv = None
        self._tx_q = queue.Queue()
        bus.on_frame = self._on_frame

    # ---- CAN side
    def _on_frame(self, t, can_id, data):
        if can_id not in FORWARD_RX:
            return
        self._send_client({"op": "rx", "id": f"{can_id:03X}", "data": data.hex().upper(), "t": t})

    def stop_both(self, why):
        """0x84 STOP to node 1 and node 2. Never raises."""
        self.stops += 1
        msg = f"WATCHDOG: {why} -> 0x84 STOP to node 1 (X) and node 2 (Y)"
        log.warning(msg)
        print(time.strftime("%H:%M:%S"), msg, flush=True)
        for can_id, data in STOP_FRAMES:
            try:
                self.bus.send(can_id, data)
            except Exception as e:                       # keep going: try the other node too
                log.error("STOP to 0x%03X failed: %s", can_id, e)

    # ---- network side
    def _send_client(self, obj, sock=None):
        line = (json.dumps(obj, separators=(",", ":")) + "\n").encode()
        with self._clock:
            c = self.client
        s = sock or (c[0] if c else None)
        if s is None:
            return
        try:
            with self._wlock:
                s.sendall(line)
        except OSError:
            pass                                         # the reader notices the dead socket

    def serve_forever(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(2)
        srv.settimeout(0.5)
        self._srv = srv
        threading.Thread(target=self._watchdog, name="watchdog", daemon=True).start()
        log.info("relay on %s:%d, CAN %s, allow %s, watchdog %.1f s", self.host, self.port, self.bus.name,
                 ", ".join(sorted(self.allow)) or "any", self.watchdog_s)
        while not self._stop.is_set():
            try:
                s, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            ip = addr[0]
            if self.allow and ip not in self.allow:
                log.warning("refused %s (not in --allow)", ip)
                self._send_client({"op": "err", "msg": f"{ip} not allowed"}, s)
                s.close()
                continue
            with self._clock:
                old = self.client
            if old is not None:
                if old[1][0] != ip:
                    log.warning("refused %s: %s is connected (one client at a time)", ip, old[1][0])
                    self._send_client({"op": "err", "msg": f"busy: {old[1][0]} is connected"}, s)
                    s.close()
                    continue
                log.warning("%s reconnected: dropping its old connection", ip)
                self._drop(old, "replaced by a new connection")
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(0.2)
            c = (s, addr)
            with self._clock:
                self.client = c
                self.last_rx = time.monotonic()
                self.armed = True
            log.info("client %s:%d connected", *addr)
            self._send_client({"op": "hello", "relay": "can_relay", "version": VERSION, "can": self.bus.name,
                               "watchdog_s": self.watchdog_s})
            threading.Thread(target=self._client_loop, args=(c,), name="client", daemon=True).start()

    def _drop(self, c, why):
        with self._clock:
            if self.client is not c:
                return
            self.client = None
            fire = self.armed
            self.armed = False
        try:
            c[0].close()
        except OSError:
            pass
        log.warning("client %s disconnected (%s)", c[1][0], why)
        if fire:
            self.stop_both(f"client {c[1][0]} disconnected ({why})")

    def _client_loop(self, c):
        s, buf = c[0], b""
        while not self._stop.is_set():
            with self._clock:
                if self.client is not c:
                    return
            try:
                chunk = s.recv(4096)
            except socket.timeout:
                continue
            except OSError as e:
                return self._drop(c, str(e))
            if not chunk:
                return self._drop(c, "closed")
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if line.strip():
                    self._handle(line)

    def _handle(self, line: bytes):
        with self._clock:
            self.last_rx = time.monotonic()
            if not self.armed:
                log.info("client active again: watchdog re-armed")
            self.armed = True
        try:
            m = json.loads(line)
            op = m.get("op")
        except (ValueError, AttributeError):
            return self._send_client({"op": "err", "msg": "bad JSON"})
        if op == "hb":
            return
        if op != "tx":
            return self._send_client({"op": "err", "msg": f"unknown op {op!r}"})
        try:
            can_id = int(str(m["id"]), 16)
            data = bytes.fromhex(str(m.get("data", "")))
        except (KeyError, ValueError):
            return self._send_client({"op": "err", "msg": "bad id/data", "id": m.get("id"), "data": m.get("data")})
        why = check_frame(can_id, data)
        if why:
            log.warning("refused frame %03X#%s: %s", can_id, data.hex().upper(), why)
            return self._send_client({"op": "err", "msg": why, "id": m.get("id"), "data": m.get("data")})
        try:
            self.bus.send(can_id, data)
        except Exception as e:
            log.error("%s", e)
            self._send_client({"op": "err", "msg": str(e), "id": m.get("id"), "data": m.get("data")})

    def _watchdog(self):
        """Own thread: fires even if the network or the client thread hangs."""
        while not self._stop.is_set():
            time.sleep(0.1)
            with self._clock:
                c, armed, age = self.client, self.armed, time.monotonic() - self.last_rx
                fire = c is not None and armed and age > self.watchdog_s
                if fire:
                    self.armed = False                   # once per silence; re-armed by the next line
            if fire:
                self.stop_both(f"nothing from {c[1][0]} for {age:.1f} s")
            if not self.bus.alive():
                log.error("CAN backend stopped. Bring can0 up:  %s", IP_LINK_UP)
                time.sleep(5)

    def shutdown(self):
        self._stop.set()
        with self._clock:
            c = self.client
        if c:
            self._drop(c, "relay shutting down")
        if self._srv:
            self._srv.close()


# ------------------------------------------------------------------------------------------- self-test
def selftest(bus) -> bool:
    """Read 0x0043 (node id) from node 1 and node 2. Moves nothing."""
    q = queue.Queue()
    bus.on_frame = lambda t, i, d: q.put((i, d))
    ok = True
    for node in (1, 2):
        while not q.empty():
            q.get_nowait()
        bus.send(0x640 + node, bytes.fromhex("0043"))
        end, got = time.monotonic() + 0.5, None
        while time.monotonic() < end and got is None:
            try:
                i, d = q.get(timeout=0.05)
            except queue.Empty:
                continue
            if i == 0x5C0 + node and len(d) == 5 and d[-1] == 0x3E:
                got = int.from_bytes(d[:4], "big", signed=True)
        good = got == node
        ok &= good
        print(f"  {'PASS' if good else 'FAIL'}  node {node} ({'X' if node == 1 else 'Y'}): 0x0043 = "
              f"{got if got is not None else 'no reply'}", flush=True)
    print("SELFTEST:", "PASS" if ok else "FAIL")
    if not ok:
        print(f"  check 48 V, the CAN cable/termination, and that can0 is up:\n    {IP_LINK_UP}")
    return ok


def main():
    ap = argparse.ArgumentParser(description="CAN relay for the gimbal ground station (runs on the Pi)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--allow", action="append", default=[], metavar="IP", help="only accept this laptop IP (repeatable)")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--watchdog", type=float, default=WATCHDOG_S, help="seconds of silence before 0x84 STOP (2.0)")
    ap.add_argument("--selftest", action="store_true", help="read 0x0043 from node 1 and 2, then exit")
    ap.add_argument("--sim", action="store_true", help="simulated drives (laptop testing, needs sim_hw.py)")
    ap.add_argument("--log", help="also log to this file")
    a = ap.parse_args()
    handlers = [logging.StreamHandler(sys.stdout)]
    if a.log:
        handlers.append(logging.FileHandler(a.log))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(message)s", handlers=handlers)
    try:
        bus = SimBus() if a.sim else CanUtilsBus(a.channel)
    except RuntimeError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(2)
    if a.selftest:
        ok = selftest(bus)
        bus.close()
        sys.exit(0 if ok else 1)
    relay = Relay(bus, a.host, a.port, a.allow, a.watchdog)
    try:
        relay.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        relay.shutdown()
        bus.close()
        log.info("relay stopped")


if __name__ == "__main__":
    os.environ.setdefault("PYTHONUNBUFFERED", "1")
    main()
