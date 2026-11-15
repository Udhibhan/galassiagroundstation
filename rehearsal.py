"""
rehearsal.py - test the receiver from the web page without the radio.

    1. make a short fake downlink: test picture -> our transmitter simulator (tx_cli.py, file
       output only, nothing is transmitted) -> Received/test/<preset>.sc16
    2. replay that recording through the real receiver (same chain that decodes a satellite), ~20 s
    3. check the picture came out byte-identical

The SDR is never opened (the receiver reads the file), so this works with or without the radio plugged in.
"""
import hashlib
import logging
import os
import signal
import struct
import subprocess
import threading
import time
import zlib
from pathlib import Path

import clock
import config

log = logging.getLogger(__name__)
HERE = Path(__file__).resolve().parent
RATE_SPS = {"ops20": 40e6, "linkx2": 8e6, "field1": 2e6}          # samples per second of each preset
SECONDS = 12                                                       # length of the test signal for the built-in picture
MAX_BYTES = 400_000                                                # biggest picture accepted (it must fit in the test signal)
OK_EXT = (".png", ".jpg", ".jpeg", ".gif", ".webp", ".bmp")


def make_png(path: Path, w=192, h=128):
    """A small test picture (colour bars + gradient), written without any imaging library."""
    rows = []
    bars = [(235, 235, 235), (235, 235, 16), (16, 235, 235), (16, 235, 16), (235, 16, 235), (235, 16, 16), (16, 16, 235), (16, 16, 16)]
    for y in range(h):
        row = bytearray([0])
        for x in range(w):
            if y < h * 2 // 3:
                r, g, b = bars[x * len(bars) // w]
            else:
                v = int(255 * x / (w - 1))
                r, g, b = v, v, v
            if (x + y) % 41 == 0:
                r, g, b = 255, 140, 0
            row += bytes((r, g, b))
        rows.append(bytes(row))
    raw = zlib.compress(b"".join(rows), 9)

    def chunk(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) + chunk(b"IDAT", raw) + chunk(b"IEND", b""))


class Rehearsal:
    def __init__(self, demod):
        self.demod = demod
        self.thread = None
        self.cancel = threading.Event()
        self.s = self._fresh()
        self.outbox = HERE / config.DEMOD.out_root / "test" / "outbox"
        self.image = None                                  # {"name", "size"} of the picture the user uploaded (None = built-in test picture)

    def set_image(self, name: str, data: bytes):
        """Keep the user's picture as the thing to 'send'. Only a picture, only a safe file name, only small."""
        if self.busy():
            raise RuntimeError("a test is running: wait for it to finish")
        name = os.path.basename(str(name or "")).strip()
        name = "".join(ch if (ch.isalnum() or ch in "._- ") else "_" for ch in name).strip(" .") or "picture.png"
        if not name.lower().endswith(OK_EXT):
            raise ValueError("choose a picture (png, jpg, gif, webp or bmp)")
        if not data:
            raise ValueError("the file is empty")
        if len(data) > MAX_BYTES:
            raise ValueError(f"the picture is {len(data) / 1000:.0f} kB: pick one under {MAX_BYTES // 1000} kB (it has to fit in the test signal)")
        self.outbox.mkdir(parents=True, exist_ok=True)
        for f in self.outbox.iterdir():
            if f.is_file():
                f.unlink()
        (self.outbox / name).write_bytes(data)
        self.image = {"name": name, "size": len(data)}
        self.s = self._fresh()
        return dict(self.image)

    def clear_image(self):
        if self.busy():
            raise RuntimeError("a test is running: wait for it to finish")
        self.image = None
        self.s = self._fresh()

    def original_path(self):
        if self.image and (self.outbox / self.image["name"]).is_file():
            return self.outbox / self.image["name"]
        p = self.outbox / "test_image.png"
        return p if p.is_file() else None

    @staticmethod
    def _fresh():
        return {"state": "idle", "step": "", "msg": "", "ok": None, "details": {}, "t0": None, "t1": None}

    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def status(self):
        d = dict(self.s)
        d["busy"] = self.busy()
        ok, why = self.demod.available()
        d["installed"], d["why_not"] = ok, why
        d["image"], d["max_kb"] = self.image, MAX_BYTES // 1000
        return d

    def start(self, preset="field1", esn0=12.0):
        if self.busy():
            raise RuntimeError("the test is already running")
        ok, why = self.demod.available()
        if not ok:
            raise RuntimeError(f"the receiver software is not installed: {why}")
        if preset not in RATE_SPS:
            raise ValueError("preset must be ops20 / linkx2 / field1")
        try:
            esn0 = float(esn0)
        except (TypeError, ValueError):
            raise ValueError("signal strength must be a number (dB)")
        if not -6.0 <= esn0 <= 30.0:
            raise ValueError("signal strength (Es/N0) must be between -6 and 30 dB")
        self.cancel.clear()
        self.s = self._fresh()
        self.s.update(state="running", t0=time.time(), preset=preset, esn0=esn0)
        self.thread = threading.Thread(target=self._run, args=(preset, esn0), name="rehearsal", daemon=True)
        self.thread.start()
        return "started"

    def abort(self, why="Abort"):
        if self.busy():
            self.cancel.set()

    def _say(self, step, msg=""):
        self.s["step"], self.s["msg"] = step, msg
        log.info("Rehearsal: %s %s", step, msg)

    def _check(self):
        if self.cancel.is_set():
            raise RuntimeError("aborted")

    def _run(self, preset, esn0=12.0):
        try:
            if self.demod.busy():                                  # one receiver at a time: the test takes over from a radio session
                self._say("Stopping the receiver", "closing the radio session first")
                self.demod.stop("starting the receiver test")
                for _ in range(80):
                    if not self.demod.busy():
                        break
                    self._check()
                    time.sleep(0.5)
                if self.demod.busy():
                    raise RuntimeError("the receiver would not stop")
            folder = HERE / config.DEMOD.out_root / "test"
            outbox = self.outbox
            outbox.mkdir(parents=True, exist_ok=True)
            if self.image and (outbox / self.image["name"]).is_file():
                pic = outbox / self.image["name"]                          # the user's own picture
            else:
                for f in outbox.iterdir():
                    if f.is_file():
                        f.unlink()
                pic = outbox / "test_image.png"
                make_png(pic)
            want = hashlib.sha256(pic.read_bytes()).hexdigest()
            # long enough to lock and get the whole picture at least once (~44 kB/s at 1 MBd after overhead)
            dur = SECONDS if not self.image else int(min(120, max(SECONDS, 8 + 2.0 * pic.stat().st_size / (44000 * RATE_SPS[preset] / 2e6))))
            recv_s = dur + 10
            self.s["name"] = pic.name
            iq = folder / f"rehearsal_{preset}.sc16"
            for f in (iq, Path(str(iq) + ".json"), Path(str(iq) + ".start")):
                f.unlink(missing_ok=True)
            # 1. fake downlink (file output only: nothing is transmitted)
            self._say("Sending your picture" if self.image else "Making a test signal",
                      f"{pic.name} ({pic.stat().st_size / 1000:.0f} kB) as a {preset} DVB-S2 signal, {dur} s (a recording; nothing is transmitted)")
            need = int(RATE_SPS[preset] * 4 * dur) // 4 * 4
            argv = [str(HERE / config.DEMOD.env_script), "python3", "-u", str(HERE / "demod" / "unified" / "tx_cli.py"),
                    "--preset", preset, "--payload", "gsse", "--stack", "rev1.0", "--files", str(outbox),
                    "--output", "file", "--output-file", str(iq), "--duration", str(dur), "--esn0-db", f"{esn0:g}"]
            with open(folder / "make_signal.log", "w") as lg:
                p = subprocess.Popen(argv, stdout=lg, stderr=subprocess.STDOUT, cwd=str(HERE), start_new_session=True)
                t0 = time.monotonic()
                while True:
                    self._check()
                    size = iq.stat().st_size if iq.exists() else 0
                    if size >= need:
                        break
                    if p.poll() is not None and size < need:
                        if size < need * 0.9:
                            raise RuntimeError(f"the signal generator stopped early (see {folder / 'make_signal.log'})")
                        break
                    if time.monotonic() - t0 > 120:
                        raise RuntimeError("the signal generator took too long")
                    time.sleep(0.3)
                try:                                                   # it does not exit by itself: stop it politely
                    os.killpg(p.pid, signal.SIGINT)
                    p.wait(timeout=10)
                except Exception:
                    try:
                        os.killpg(p.pid, signal.SIGKILL)
                    except Exception:
                        pass
            with open(iq, "r+b") as fh:
                fh.truncate(need)
            # 2. through the real receiver (it reads the file, not the radio)
            self._say("Receiving it", f"the receiver decodes the signal ({recv_s} s)")
            self.demod.start({"mode": "live", "preset": preset, "record": False, "sim_input": str(iq)})
            t1 = time.monotonic()
            while time.monotonic() - t1 < recv_s and self.demod.busy():
                self._check()
                time.sleep(0.3)
            st = self.demod.status()
            self.demod.stop("test finished")
            for _ in range(60):
                if not self.demod.busy():
                    break
                time.sleep(0.5)
            # 3. is the picture there and identical?
            self._say("Checking the picture")
            st = self.demod.status()
            run_dir = HERE / config.DEMOD.out_root / (st.get("run") or "")
            got = run_dir / pic.name
            same = got.is_file() and hashlib.sha256(got.read_bytes()).hexdigest() == want
            det = {"locked": st.get("locked"), "snr_db": st.get("snr_db"), "frames": st.get("plframes"), "uncorrectable": st.get("bch_err"),
                   "packets": st.get("gsse_ok"), "picture_found": got.is_file(), "picture_identical": same, "run": st.get("run"), "name": pic.name, "esn0": esn0}
            self.s["details"] = det
            meas = st.get("snr_db")
            head = (f"set {esn0:g} dB, the receiver measured {meas:.1f} dB" if meas is not None else f"set {esn0:g} dB, the receiver could not measure it")
            counts = f"{st.get('plframes') or 0} frames, {st.get('bch_err') or 0} uncorrectable, {st.get('gsse_ok') or 0} packets"
            if same and st.get("locked"):
                self.s.update(state="done", ok=True, msg=f"PASS: {head}; {pic.name} came out identical ({counts})")
            else:
                why = ("the receiver never locked: the signal is too noisy" if not st.get("locked") else
                       "no picture came out: too much noise, frames were lost" if not got.is_file() else "the picture came out different from the original")
                self.s.update(state="failed", ok=False, msg=f"FAIL: {st.get('error') or why} ({head}; {counts})")
        except Exception as e:
            log.exception("rehearsal")
            try:
                self.demod.stop("test failed")
            except Exception:
                pass
            self.s.update(state="aborted" if str(e) == "aborted" else "failed", ok=False, msg=("stopped" if str(e) == "aborted" else f"FAIL: {e}"))
        finally:
            self.s["step"] = ""
            self.s["t1"] = time.time()
