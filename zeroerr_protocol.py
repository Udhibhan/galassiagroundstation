"""
zeroerr_protocol.py - ZeroErr eRob CAN-custom protocol: link adapters + one drive (node).

Bench-proven 2026-10-20 (both motors, Pi CAN HAT and Waveshare USB-CAN-A):
  command  0x640+N : [index 2B big-endian] [value 4B big-endian, optional]
  reply    0x5C0+N : read  -> [value 4B][3E]
                     write -> [3E]                     OK
                              [error code 2B][3E]      refused (e.g. 00 27 3E = motor OFF)
A refused write is not 0x80. Any write reply other than exactly 3E is an error.

Bench rules:
  * 1 Mbps, fixed.
  * Gap of at least 20 ms before every request; don't burst-poll right after enable.
  * One thread owns the bus (gimbal.Gimbal), nothing else calls send/recv.

Links (each has send(id, data), recv(timeout) -> (id, bytes) | None, close(), name):
  RemoteCanLink  TCP to can_relay.py on the Pi (default: app on the laptop, Pi only relays CAN)
  CanUtilsLink   can-utils candump / cansend subprocesses (app running on the Pi itself)
  SocketCanLink  python-can on SocketCAN
  WaveshareLink  Waveshare USB-CAN-A (serial)
  SimCanLink     sim_hw.py, no hardware
"""
import glob
import os
import queue
import re
import shutil
import struct
import subprocess
import threading
import time
from collections import deque

import config

ERRORS = {
    0x0D: "register not supported", 0x15: "not valid during motion", 0x16: "not valid with motor ON",
    0x1F: "commutation not found yet (wait 8 s after power-on)", 0x27: "motor is OFF",
    0x30: "invalid motion mode", 0x32: "command needs a parameter", 0x33: "command takes no parameter",
    0x56: "relay not active (wait 8 s after power-on)", 0x14A: "brake control not used",
    0xC26: "stored code seen after power-up (harmless, clears on enable)",
}
STATES = {0: "OFF", 1: "on, idle", 2: "moving", 3: "settling", 4: "target reached"}

# Registers: the ones the proven sequence uses, plus read-only status
REG_POSITION = 0x0002     # read, counts
REG_CURRENT = 0x0008      # read, motor current, signed mA
REG_ERROR = 0x001F        # read, stored error code
REG_BUS_MV = 0x0024       # read, bus voltage in mV
REG_TEMP = 0x0026         # read, drive temperature in deg C
REG_ID = 0x0043           # read, used for scanning
REG_OPMODE = 0x004E       # 3 = position control (write only while OFF)
REG_MOTION_MODE = 0x008D  # 1 = target-position mode (write only while OFF)
REG_LIMIT_ACC = 0x009E    # 1 = motion limited by accel/decel
REG_ACCEL = 0x0088
REG_DECEL = 0x0089
REG_SPEED = 0x008A
REG_START = 0x0083        # trigger, no value
REG_STOP = 0x0084         # trigger, no value
REG_REL_TARGET = 0x0087   # relative move, counts (set after enable)
REG_STATE = 0x010C        # read: 0 off, 1 idle, 2 moving, 3 settling, 4 reached
REG_ENABLE = 0x0100       # 1 enable, 0 disable (brake holds)
# Never written here: 0x0050 max speed, 0x0052/0x0053 drive limits, 0x00E8 save-to-flash.
NEVER_WRITE = frozenset({0x0050, 0x0052, 0x0053, 0x00E8})


def deg_to_counts(deg: float) -> int:
    return int(round(deg / 360.0 * config.COUNTS_PER_REV))


def counts_to_deg(c: int) -> float:
    return c * 360.0 / config.COUNTS_PER_REV


class DriveError(Exception):
    def __init__(self, node, index, reply: bytes):
        self.node, self.index = node, index
        self.code = int.from_bytes(reply[:-1], "big") if len(reply) >= 2 and reply[-1] == 0x3E else None
        why = (f"error 0x{self.code:X} ({ERRORS.get(self.code, 'see ZeroErr fault list')})"
               if self.code is not None else "abnormal reply")
        super().__init__(f"node {node}: 0x{index:04X} refused, reply {reply.hex(' ').upper()} = {why}")


# ---------------------------------------------------------------------------- adapters
IP_LINK_UP = "sudo ip link set can0 up type can bitrate 1000000 restart-ms 100"
_CANDUMP_LINE = re.compile(r"^\(\s*[\d.]+\)\s+(\S+)\s+([0-9A-Fa-f]{3,8})#([0-9A-Fa-f]*)\s*$")


def parse_candump_line(line: str):
    """'(1699999999.123456) can0 5C1#0000123E3E' gives (0x5C1, bytes 00 00 12 3E 3E), else None.
    Remote (#R) and CAN-FD (##) frames are ignored."""
    m = _CANDUMP_LINE.match(line.strip())
    if not m or len(m.group(3)) % 2:
        return None
    return int(m.group(2), 16), bytes.fromhex(m.group(3))


class CanUtilsLink:
    """can-utils backend: a `candump -L` subprocess feeds a reader thread, frames go out via
    `cansend`. Works on the Pi where python-can didn't. Bring the bus up first:
        sudo ip link set can0 up type can bitrate 1000000 restart-ms 100"""

    def __init__(self, channel="can0"):
        self.channel = channel
        self.name = f"can-utils {channel}"
        for tool in ("candump", "cansend"):
            if shutil.which(tool) is None:
                raise RuntimeError(f"'{tool}' not found. Install can-utils:  sudo apt install can-utils")
        self.q = queue.Queue()
        # only drive replies 0x5C0..0x5FF, otherwise our own 0x64x frames get looped back
        self.proc = subprocess.Popen(["candump", "-L", f"{channel},5C0:7C0"], stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, text=True, bufsize=1)
        time.sleep(0.3)
        if self.proc.poll() is not None:
            err = (self.proc.stderr.read() or "").strip()
            raise RuntimeError(f"candump could not open {channel}: {err or 'exited'}\n"
                               f"Bring the CAN bus up first:\n    {IP_LINK_UP}")
        self._reader = threading.Thread(target=self._read, name="candump", daemon=True)
        self._reader.start()

    def _read(self):
        for line in self.proc.stdout:
            f = parse_candump_line(line)
            if f is not None:
                self.q.put(f)

    def send(self, can_id, data):
        frame = f"{can_id:03X}#{bytes(data).hex().upper()}"
        r = subprocess.run(["cansend", self.channel, frame], capture_output=True, text=True, timeout=2)
        if r.returncode != 0:
            raise OSError(f"cansend {frame} failed: {(r.stderr or '').strip()} (is {self.channel} up? {IP_LINK_UP})")

    def recv(self, timeout):
        if self.proc.poll() is not None and self.q.empty():
            raise OSError(f"candump stopped (is {self.channel} up? {IP_LINK_UP})")
        try:
            return self.q.get(timeout=timeout) if timeout > 0 else self.q.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(2)
            except subprocess.TimeoutExpired:
                self.proc.kill()


class SocketCanLink:
    """Pi CAN HAT via python-can. Bring up first:  sudo ip link set can0 up type can bitrate 1000000 restart-ms 100"""

    def __init__(self, channel="can0"):
        import can
        self._can = can
        self.bus = can.Bus(interface="socketcan", channel=channel)
        self.name = f"SocketCAN {channel}"

    def send(self, can_id, data):
        self.bus.send(self._can.Message(arbitration_id=can_id, data=bytes(data), is_extended_id=False))

    def recv(self, timeout):
        m = self.bus.recv(timeout=max(timeout, 0))
        return (m.arbitration_id, bytes(m.data)) if m is not None else None

    def close(self):
        self.bus.shutdown()


class WaveshareLink:
    """Waveshare USB-CAN-A (CH340 serial, 2 Mbps, Waveshare variable-length frames). Not SocketCAN.
       config: AA 55 12 <speed> 01 00x8 00 00 00x4 <sum[2:19]>    frame: AA <C0|dlc> <ID LE 2B> <data> 55"""
    SPEED = {1_000_000: 0x01, 800_000: 0x02, 500_000: 0x03, 250_000: 0x05, 125_000: 0x07}

    def __init__(self, port, bitrate=1_000_000):
        import serial
        self.ser = serial.Serial(port, 2_000_000, timeout=0)
        cfg = bytearray([0xAA, 0x55, 0x12, self.SPEED[bitrate], 0x01] + [0] * 8 + [0x00, 0x00] + [0] * 4)
        cfg.append(sum(cfg[2:19]) & 0xFF)
        self.ser.write(cfg)
        self.ser.flush()
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.buf = bytearray()
        self.name = f"Waveshare USB-CAN-A {port}"

    def send(self, can_id, data):
        data = bytes(data)
        self.ser.write(bytes([0xAA, 0xC0 | len(data)]) + can_id.to_bytes(2, "little") + data + b"\x55")

    def _parse(self):
        while True:
            i = self.buf.find(0xAA)
            if i < 0:
                self.buf.clear()
                return None
            del self.buf[:i]
            if len(self.buf) < 2:
                return None
            t = self.buf[1]
            dlc, idlen = t & 0x0F, (4 if t & 0x20 else 2)
            if t & 0xC0 != 0xC0 or dlc > 8:
                del self.buf[0]
                continue
            n = 2 + idlen + dlc + 1
            if len(self.buf) < n:
                return None
            if self.buf[n - 1] != 0x55:
                del self.buf[0]
                continue
            fid = int.from_bytes(self.buf[2:2 + idlen], "little")
            data = bytes(self.buf[2 + idlen:2 + idlen + dlc])
            del self.buf[:n]
            return fid, data

    def recv(self, timeout):
        end = time.monotonic() + timeout
        while True:
            f = self._parse()
            if f:
                return f
            left = end - time.monotonic()
            if left <= 0:
                return None
            self.ser.timeout = min(left, 0.05)
            chunk = self.ser.read(max(1, self.ser.in_waiting))
            if chunk:
                self.buf += chunk

    def close(self):
        self.ser.close()


class RemoteCanLink:
    """CAN on the Pi through can_relay.py (TCP, newline-delimited JSON). Default in the split setup.
    A thread sends {"op":"hb"} every 0.5 s (the relay STOPs both nodes after 2 s of silence) and
    reconnects with backoff. While disconnected send/recv raise OSError, so the Gimbal goes
    'offline' and retries."""
    reply_timeout_s = config.RELAY_REPLY_TIMEOUT_S

    def __init__(self, pi: str, connect_timeout=3.0):
        import json
        import socket
        self._json, self._socket = json, socket
        host, _, port = (pi or "").partition(":")
        if not host:
            raise RuntimeError("remote CAN needs the Pi's address:  --pi <host>[:7070]")
        self.addr = (host, int(port or config.RELAY_PORT))
        self.name = f"Pi relay {self.addr[0]}:{self.addr[1]}"
        self.q = queue.Queue()
        self.connected = False
        self.hello = None
        self.last_error = None
        self.relay_errors = deque(maxlen=20)       # err messages from the relay
        self._sock = None
        self._wlock = threading.Lock()
        self._stop = threading.Event()
        self._backoff = 0.5
        self._connect_timeout = connect_timeout
        self._try_connect()
        threading.Thread(target=self._keeper, name="relay-link", daemon=True).start()

    # ---- connection
    def _try_connect(self):
        try:
            s = self._socket.create_connection(self.addr, timeout=self._connect_timeout)
            s.setsockopt(self._socket.IPPROTO_TCP, self._socket.TCP_NODELAY, 1)
            s.settimeout(None)
        except OSError as e:
            self.last_error = f"cannot reach the relay at {self.addr[0]}:{self.addr[1]}: {e}"
            return False
        with self._wlock:
            self._sock = s
        self.connected, self.last_error, self._backoff = True, None, 0.5
        threading.Thread(target=self._reader, args=(s,), name="relay-rx", daemon=True).start()
        return True

    def _lost(self, s, why):
        with self._wlock:
            if self._sock is not s:
                return
            self._sock = None
        self.connected = False
        self.last_error = f"relay connection lost: {why}"
        try:
            s.close()
        except OSError:
            pass

    def _reader(self, s):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = s.recv(4096)
            except OSError as e:
                return self._lost(s, e)
            if not chunk:
                return self._lost(s, "closed by the Pi")
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    m = self._json.loads(line)
                except ValueError:
                    continue
                op = m.get("op")
                if op == "rx":
                    try:
                        self.q.put((int(m["id"], 16), bytes.fromhex(m["data"])))
                    except (KeyError, ValueError):
                        pass
                elif op == "hello":
                    self.hello = m
                elif op == "err":
                    self.relay_errors.append(m)

    def _keeper(self):
        """Heartbeat every 0.5 s while connected; reconnect with backoff (0.5 .. 5 s) while not."""
        while not self._stop.is_set():
            if self.connected:
                try:
                    self._write({"op": "hb"})
                except OSError:
                    pass
                self._stop.wait(config.RELAY_HEARTBEAT_S)
            else:
                if not self._try_connect():
                    self._stop.wait(self._backoff)
                    self._backoff = min(self._backoff * 2, 5.0)

    def _write(self, obj):
        line = (self._json.dumps(obj, separators=(",", ":")) + "\n").encode()
        with self._wlock:
            s = self._sock
            if s is None:
                raise OSError(self.last_error or "relay not connected")
            try:
                s.sendall(line)
            except OSError as e:
                self._sock = None
                self.connected = False
                self.last_error = f"relay connection lost: {e}"
                raise OSError(self.last_error) from e

    # ---- link interface
    def send(self, can_id, data):
        self._write({"op": "tx", "id": f"{can_id:03X}", "data": bytes(data).hex().upper()})

    def recv(self, timeout):
        if not self.connected and self.q.empty():
            raise OSError(self.last_error or "relay not connected")
        try:
            return self.q.get(timeout=timeout) if timeout > 0 else self.q.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        self._stop.set()
        with self._wlock:
            s, self._sock = self._sock, None
        self.connected = False
        if s is not None:
            try:
                s.close()
            except OSError:
                pass


def _find_waveshare_port():
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            if p.vid == 0x1A86:                      # CH340 = Waveshare USB-CAN-A
                return p.device
    except ImportError:
        pass
    ports = sorted(glob.glob("/dev/ttyUSB*"))
    return ports[0] if ports else None


def open_link(cfg: config.CanBusConfig | None = None):
    """interface: remote (default, Pi relay) | canutils | socketcan | waveshare | sim (auto is for old scripts)."""
    cfg = cfg or config.CAN_BUS
    iface = cfg.interface
    if iface == "sim":
        from sim_hw import SimCanLink
        return SimCanLink()
    if iface == "remote":
        return RemoteCanLink(cfg.pi)
    if iface == "canutils":
        return CanUtilsLink(cfg.channel)
    if iface == "auto":
        iface = "socketcan" if os.path.exists(f"/sys/class/net/{cfg.channel}") else "waveshare"
    if iface == "socketcan":
        return SocketCanLink(cfg.channel)
    port = cfg.serial_port or _find_waveshare_port()
    if not port:
        raise RuntimeError("No CAN adapter: no can0 and no Waveshare (CH340) serial port. Check lsusb / dmesg.")
    return WaveshareLink(port, cfg.bitrate)


# ---------------------------------------------------------------------------- one drive
class Drive:
    """One eRob node. Not thread-safe on purpose: only the Gimbal thread calls it."""

    def __init__(self, link, node: int, gap_s: float = 0.02, timeout_s: float = 0.4):
        self.link, self.node = link, node
        self.tx, self.rx = 0x640 + node, 0x5C0 + node
        self.gap_s, self.timeout_s = gap_s, timeout_s

    def _xfer(self, payload: bytes, lengths: tuple, timeout=None) -> bytes:
        """Drop stale frames, send, wait for a reply from 0x5C0+N with a length in `lengths` ending 3E.
        A late reply to an earlier request (other length) is skipped. For reads, a 3-byte refusal is
        only returned (and becomes DriveError) if no 5-byte value arrives in time."""
        time.sleep(self.gap_s)
        while self.link.recv(0) is not None:          # drop stale frames (boot msgs, late replies)
            pass
        self.link.send(self.tx, payload)
        end = time.monotonic() + (timeout or self.timeout_s)
        other = None
        while (left := end - time.monotonic()) > 0:
            f = self.link.recv(left)
            if f is None or f[0] != self.rx:
                continue
            if len(f[1]) in lengths and f[1][-1] == 0x3E:
                return f[1]
            other = f[1]
        if other is not None:
            return other
        raise TimeoutError(f"node {self.node}: no reply to {payload.hex(' ').upper()}")

    def write(self, index: int, value: int | None = None) -> None:
        if index in NEVER_WRITE:
            raise PermissionError(f"0x{index:04X} is a drive flash/limit register: never written by this software")
        p = struct.pack(">H", index) + (struct.pack(">i", int(value)) if value is not None else b"")
        r = self._xfer(p, (1, 3))                     # 3E = ok, [code 2B][3E] = refused
        if r != b"\x3e":
            raise DriveError(self.node, index, r)

    def read(self, index: int, timeout=None) -> int:
        r = self._xfer(struct.pack(">H", index), (5,), timeout)    # [value 4B][3E]
        if len(r) != 5 or r[-1] != 0x3E:
            raise DriveError(self.node, index, r)
        return struct.unpack(">i", r[:4])[0]

    # --- the proven sequence, split into steps -------------------------------------
    def configure(self, speed_deg_s: float, accel_deg_s2: float | None = None, accel_counts: int | None = None) -> None:
        """Motor off, then position mode, target-position mode and profile, same order as on the bench:
        0100=0, 004E=3, 008D=1, 009E=1, 0088=accel, 0089=accel, 008A=speed.
        accel_counts (raw register value) wins over accel_deg_s2."""
        acc = int(accel_counts) if accel_counts is not None else deg_to_counts(accel_deg_s2)
        self.write(REG_ENABLE, 0)
        self.write(REG_OPMODE, 3)
        self.write(REG_MOTION_MODE, 1)
        self.write(REG_LIMIT_ACC, 1)
        self.write(REG_ACCEL, acc)
        self.write(REG_DECEL, acc)
        self.write(REG_SPEED, deg_to_counts(speed_deg_s))

    def enable(self, retries: int = 10) -> None:
        for _ in range(retries):
            try:
                self.write(REG_ENABLE, 1)
                return
            except DriveError as e:
                if e.code in (0x1F, 0x56):           # still booting
                    time.sleep(1.0)
                    continue
                raise
        raise RuntimeError(f"node {self.node}: would not enable")

    def move_relative(self, delta_deg: float) -> None:
        """0x87 relative target then 0x83 START back-to-back (motor must be enabled)."""
        self.write(REG_REL_TARGET, deg_to_counts(delta_deg))
        self.write(REG_START)

    def enable_and_move(self, delta_counts: int) -> None:
        """Test-stage move: 0100=1, 0087=<relative counts>, 0083 START, back-to-back.
        Don't enable and then wait: an enabled, idle drive switches its motor off (seen on the bench)."""
        self.write(REG_ENABLE, 1)
        self.write(REG_REL_TARGET, int(delta_counts))
        self.write(REG_START)

    def set_speed(self, speed_deg_s: float) -> None:
        self.write(REG_SPEED, deg_to_counts(speed_deg_s))

    def stop(self) -> None:
        self.write(REG_STOP)

    def disable(self) -> None:
        self.write(REG_ENABLE, 0)

    def position_counts(self) -> int:
        return self.read(REG_POSITION)

    def state(self) -> int:
        return self.read(REG_STATE)

    def error(self) -> int:
        return self.read(REG_ERROR)

    def bus_volts(self) -> float:
        return self.read(REG_BUS_MV) / 1000.0

    def current_a(self) -> float:
        return self.read(REG_CURRENT) / 1000.0          # signed: FF FF E5 FD = -6.66 A

    def temperature_c(self) -> int:
        return self.read(REG_TEMP)

    def node_id(self) -> int:
        return self.read(REG_ID)


def scan(link, ids=range(1, 33)) -> list[int]:
    found = []
    for n in ids:
        try:
            Drive(link, n, gap_s=0.005).read(REG_ID, timeout=0.05)
            found.append(n)
        except (TimeoutError, DriveError):
            pass
        except OSError:                                 # SocketCAN tx buffer full (no ACK on bus)
            time.sleep(0.1)
    return found
