"""
demod.py - the Galassia-5 demodulator run as a supervised subprocess.

    ./demod_env.sh python3 demod/unified/rx_cli.py ...
    (GNU Radio 3.10 + patched gr-dvbs2rx + the GSSE/GSUFTP image decoder)

Why a subprocess: a GNU Radio crash can't take the gimbal controller down, app.py needs no GNU
Radio, and the demod keeps its own conda env (~/gs_demod_env).

Modes (rx_cli.py --mode):
    file    decode an IQ recording (sc16 / fc32) as fast as the CPU allows, no SDR
    record  B206mini to full-rate sc16 recording + duty-cycled live demod (20 MBd passes)
    live    B206mini to full demod, recording optional (up to ~20 MS/s)

Receive only: this module only starts rx_cli.py, never tx_cli.py / tx_unified.py (which can open
a UHD sink). It always passes --antenna RX2 and refuses device args that mention tx.

The subprocess prints "[time] RX EVENT: ..." lines. A status line (every stats_every_s) looks like
    file 100.0% | REC 12s 0.0GB, 83GB free, lvl -23.1dBFS pk -12.0, O 3 | LOCK +2.00kHz 8.2dB | fr 283/199d | bch 256/17err | gsse 283/0bad files 1
and a finished image like
    FILE test_image.png: 182668 B sha256 1a94fb33adb66d17... PASS
Both are parsed here for the web page.
"""
import logging
import os
import re
import signal
import subprocess
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import config

log = logging.getLogger(__name__)
HERE = Path(__file__).resolve().parent
MODES = ("file", "record", "live")
PRESETS = ("ops20", "linkx2", "field1")
STACKS = ("auto", "rev1.0", "rev1.2")
IMG_EXT = {".png": "image/png", ".jpg": "image/jpeg", ".jpeg": "image/jpeg", ".gif": "image/gif",
           ".webp": "image/webp", ".bmp": "image/bmp", ".txt": "text/plain; charset=utf-8",
           ".json": "application/json"}
IQ_EXT = (".sc16", ".cs16", ".fc32", ".cfile", ".iq")

_RE_LOCK = re.compile(r"(LOCK|search) ([+-][\d.]+)kHz(?: ([\d.]+)dB)?")
_RE_FR = re.compile(r"fr (\d+)/(\d+)d")
_RE_BCH = re.compile(r"bch (\d+)/(\d+)err")
_RE_GSSE = re.compile(r"gsse (\d+)/(\d+)bad files (\d+)")
_RE_FILEPCT = re.compile(r"file\s+([\d.]+)%")
_RE_REC = re.compile(r"REC (\d+)s ([\d.]+)GB, (\d+)GB free(?:, lvl (-?[\d.]+)dBFS pk (-?[\d.]+))?(?:, O (\d+))?")
_RE_RECOFF = re.compile(r"rec off, (\d+)GB free")
_RE_LVL = re.compile(r"lvl (-?[\d.]+)dBFS pk (-?[\d.]+)")
_RE_GATE = re.compile(r"(duty|continuous|pass)\S* (\d+) win(?: (\d+) dropped)?")
_RE_FILE = re.compile(r"FILE (.+?): (\d+) B sha256 ([0-9a-f.]+)\s*(PASS|FAIL|not in hash list)?")
_RE_PLSYNC = re.compile(r"PL sync: locked=(True|False), carrier offset ([+-][\d.]+) kHz(?:, SNR ([\d.]+) dB)?")
_RE_PLF = re.compile(r"PLFRAMEs: \d+ SOFs, (\d+) data frames, (\d+) dummy")
_RE_BCHS = re.compile(r"BCH: (\d+) frames, (\d+) with uncorrectable")
_RE_TS = re.compile(r"^\[[\d:.]+\] (?:RX )?EVENT: ")


def _utc_stamp():
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _abs(p: str) -> Path:
    q = Path(os.path.expanduser(p))
    return q if q.is_absolute() else HERE / q


class DemodSession:
    """One demod run at a time. All public methods are thread-safe."""

    def __init__(self, cfg: config.DemodConfig = config.DEMOD, hold_sdr=None, release_sdr=None):
        self.cfg = cfg
        self.hold_sdr = hold_sdr or (lambda: None)         # called before an SDR run: free the B206mini (raises if it can't)
        self.release_sdr = release_sdr or (lambda: None)   # called after an SDR run: give it back
        self.lock = threading.RLock()
        self.proc = None
        self.thread = None
        self.logq = deque(maxlen=250)
        self.out_root = _abs(cfg.out_root)
        self._sdr_held = False
        self._stop_req = None
        self._rec_cache = (0.0, [])
        self.view_run = None                       # an earlier run shown on the page while nothing is running
        self.s = self._fresh()
        self._power = deque(maxlen=900)              # (monotonic s, input level dBFS): the power trace on the page

    # ------------------------------------------------------------------ state
    @staticmethod
    def _fresh():
        return {"state": "idle", "mode": None, "preset": None, "pid": None, "t0": None, "t1": None, "exit": None,
                "cmd": "", "msg": "", "run": None, "out_dir": None, "input": None, "rec_dir": None,
                "line": "", "progress": None, "locked": None, "cfo_khz": None, "snr_db": None,
                "plframes": 0, "dummy": 0, "bch_frames": 0, "bch_err": 0, "gsse_ok": 0, "gsse_bad": 0,
                "rec": None, "gate": None, "result": None, "error": None, "sdr": False, "stack": None,
                "level_dbfs": None, "peak_dbfs": None}

    def busy(self) -> bool:
        return self.thread is not None and self.thread.is_alive()

    def uses_sdr(self) -> bool:
        return self.busy() and self._sdr_held

    def available(self) -> tuple[bool, str]:
        """Is the demod installed? (env script, rx_cli.py, patched library)."""
        for p in (self.cfg.env_script, self.cfg.rx_cli):
            if not _abs(p).exists():
                return False, f"missing {p}"
        if not (HERE / "gr_dvbs2rx" / "lib").exists():
            return False, "gr_dvbs2rx/ (the patched library) is not built"
        return True, ""

    def status(self) -> dict:
        with self.lock:
            s = dict(self.s)
        s["busy"] = self.busy()
        now = time.monotonic()
        s["power"] = [[round(t - now, 1), v] for t, v in list(self._power) if now - t <= 240]      # seconds ago, dBFS
        ok, why = self.available()
        s["installed"], s["why_not"] = ok, why
        s["shown_run"] = s.get("run") or self.view_run
        s["files"] = self.files(s["shown_run"])
        s["log"] = list(self.logq)[-40:]
        s["recordings"] = self.recordings()
        s["defaults"] = {"preset": self.cfg.preset, "rec_dir": self.cfg.rec_dir, "centre_freq_hz": self.cfg.centre_freq_hz,
                         "rx_gain_db": self.cfg.rx_gain_db, "stack": self.cfg.stack, "presets": list(PRESETS),
                         "stacks": list(STACKS), "modes": list(MODES)}
        s["runs"] = self.runs()
        return s

    # ------------------------------------------------------------------ files
    def runs(self):
        """Earlier decode runs (newest first): [{run, n_files}]."""
        try:
            ds = sorted((d for d in self.out_root.iterdir() if d.is_dir() and d.name.startswith("run_")), key=lambda d: d.name, reverse=True)
        except OSError:
            return []
        return [{"run": d.name, "n": sum(1 for f in d.iterdir() if f.suffix.lower() in IMG_EXT and f.name not in ("MANIFEST.txt", "report.json"))}
                for d in ds[:12]]

    def view(self, run):
        """Show the images of an earlier run (or None = the current/last run)."""
        if run:
            if "/" in run or "\\" in run or run.startswith(".") or not (self.out_root / run).is_dir():
                raise ValueError("no such run")
        self.view_run = run or None
        return "ok"

    def files(self, run):
        if not run:
            return []
        d = self.out_root / run
        try:
            fs = [f for f in d.iterdir() if f.is_file() and f.name not in ("MANIFEST.txt", "report.json")]
        except OSError:
            return []
        verdicts = {}
        with self.lock:
            for f in self.s.get("found", []) if isinstance(self.s.get("found"), list) else []:
                verdicts[f["name"]] = f
        out = []
        for f in sorted(fs, key=lambda f: f.stat().st_mtime):
            v = verdicts.get(f.name, {})
            out.append({"name": f.name, "size": f.stat().st_size, "t": f.stat().st_mtime, "verdict": v.get("verdict", ""),
                        "sha256": v.get("sha256", ""), "image": f.suffix.lower() in (".png", ".jpg", ".jpeg", ".gif", ".webp", ".bmp"),
                        "url": f"/api/demod/file?run={run}&name={f.name}"})
        return out

    def file_path(self, run: str, name: str) -> tuple[Path, str]:
        """Safe lookup for the web server: only inside out_root/<run>/. Raises FileNotFoundError."""
        if "/" in run or "\\" in run or run.startswith(".") or "/" in name or "\\" in name or name.startswith("."):
            raise FileNotFoundError("bad name")
        p = (self.out_root / run / name).resolve()
        if not p.is_file() or self.out_root.resolve() not in p.parents:
            raise FileNotFoundError(name)
        return p, IMG_EXT.get(p.suffix.lower(), "application/octet-stream")

    def recordings(self):
        """IQ files in the recording folder (cached 3 s): [{name, path, size, t}]."""
        t = time.monotonic()
        if t - self._rec_cache[0] < 3.0:
            return self._rec_cache[1]
        out = []
        d = _abs(self.cfg.rec_dir)
        try:
            for f in d.iterdir():
                if f.is_file() and f.suffix.lower() in IQ_EXT:
                    st = f.stat()
                    out.append({"name": f.name, "path": str(f), "size": st.st_size, "t": st.st_mtime})
        except OSError:
            pass
        out.sort(key=lambda r: r["t"], reverse=True)
        self._rec_cache = (t, out[:30])
        return self._rec_cache[1]

    # ------------------------------------------------------------------ start / stop
    def build(self, o: dict) -> tuple[list, dict]:
        """Validate the request and return (argv, info). Raises ValueError with a message for the page."""
        cfg = self.cfg
        mode = o.get("mode", "file")
        if mode not in MODES:
            raise ValueError(f"mode must be one of {', '.join(MODES)}")
        preset = o.get("preset") or cfg.preset
        if preset not in PRESETS:
            raise ValueError(f"preset must be one of {', '.join(PRESETS)}")
        stack = o.get("stack") or cfg.stack
        if stack not in STACKS:
            raise ValueError(f"stack must be one of {', '.join(STACKS)}")
        run = "run_" + _utc_stamp()
        out_dir = self.out_root / run
        argv = [str(_abs(cfg.env_script)), "python3", "-u", str(_abs(cfg.rx_cli)),
                "--mode", mode, "--preset", preset, "--stack", stack,
                "--out-dir", str(out_dir), "--bbframe-file", "",            # the tool's defaults are paths from another machine
                "--report-json", str(out_dir / "report.json"),
                "--stats-every", f"{cfg.stats_every_s:g}", "--panels", "none"]
        modcod = str(o.get("modcod") or cfg.modcod or "")
        if modcod:
            argv += ["--modcod", modcod]
        info = {"mode": mode, "preset": preset, "run": run, "out_dir": str(out_dir), "stack": stack, "sdr": False,
                "input": None, "rec_dir": None}
        if mode == "file":
            path = os.path.expanduser(str(o.get("input_file") or ""))
            if not path or not os.path.isfile(path):
                raise ValueError("choose an IQ recording to decode (file not found)")
            argv += ["--input-file", path]
            info["input"] = path
            if o.get("conj"):
                argv += ["--conj", "1"]
        else:
            if str(o.get("device_args") or "").lower().find("tx") >= 0:
                raise ValueError("device args may not mention tx: the B206mini is receive only")
            rec_dir = str(_abs(o.get("rec_dir") or cfg.rec_dir))
            freq = float(o.get("centre_freq_hz") or cfg.centre_freq_hz)
            gain = float(o.get("rx_gain_db") if o.get("rx_gain_db") not in (None, "") else cfg.rx_gain_db)
            if not 0 <= gain <= 76:
                raise ValueError("RX gain must be 0..76 dB")
            if not 100e6 <= freq <= 6e9:
                raise ValueError("centre frequency must be 100 MHz .. 6 GHz")
            argv += ["--rec-dir", rec_dir, "--centre-freq", f"{freq:.0f}", "--rx-gain", f"{gain:g}",
                     "--antenna", "RX2", "--rf-freq", f"{float(o.get('rf_freq_hz') or cfg.rf_freq_hz):.0f}"]
            if o.get("device_args"):
                argv += ["--device-args", str(o["device_args"])]
            if o.get("record") in (True, False):
                argv += ["--record", "1" if o["record"] else "0"]
            if o.get("duration_s"):
                argv += ["--duration", f"{float(o['duration_s']):g}"]
            sim = o.get("sim_input")
            if sim:                                                          # rehearsal: a file stands in for the radio
                argv += ["--sim-input", os.path.expanduser(str(sim))]
            info.update(sdr=not sim, rec_dir=rec_dir)
        if o.get("tle"):
            tle = os.path.expanduser(str(o["tle"]))
            if not os.path.isfile(tle):
                raise ValueError("TLE file not found")
            argv += ["--tle", tle]
        if o.get("hashes"):
            h = os.path.expanduser(str(o["hashes"]))
            if not os.path.isfile(h):
                raise ValueError("hash file not found")
            argv += ["--verify-hashes", h]
        if "tx_cli" in " ".join(argv) or "tx_unified" in " ".join(argv):
            raise ValueError("refusing to start a transmitter")                # belt and braces
        return argv, info

    def start(self, o: dict) -> str:
        with self.lock:
            if self.busy():
                raise RuntimeError("the demodulator is already running: Stop it first")
            ok, why = self.available()
            if not ok:
                raise RuntimeError(f"demodulator not installed: {why}")
            argv, info = self.build(o)
            if info["sdr"]:
                self.hold_sdr()                                              # raises if the SDR can't be freed
                self._sdr_held = True
            else:
                self._sdr_held = False
            os.makedirs(info["out_dir"], exist_ok=True)
            self.logq.clear()
            self._power.clear()
            self.view_run = None
            self._stop_req = None
            self.s = self._fresh()
            self.s.update(info, state="starting", t0=time.time(), found=[],
                          cmd=" ".join(a if " " not in a else repr(a) for a in argv[1:]))
            try:
                self.proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                                             cwd=str(HERE), start_new_session=True)
            except OSError as e:
                self._give_back()
                self.s.update(state="failed", error=str(e), t1=time.time())
                raise RuntimeError(f"could not start the demodulator: {e}")
            self.s["pid"] = self.proc.pid
            self.thread = threading.Thread(target=self._reader, args=(self.proc,), name="demod", daemon=True)
            self.thread.start()
        log.info("Demod started (%s, %s): pid %s", info["mode"], info["preset"], self.proc.pid)
        return "started"

    def stop(self, why="Stop"):
        """SIGINT first (rx_cli flushes the decoder and writes its report), then SIGTERM, then SIGKILL."""
        with self.lock:
            p = self.proc
            if p is None or p.poll() is not None or not self.busy():
                return "not running"
            self._stop_req = why
            self.s["msg"] = f"stopping ({why})…"
        threading.Thread(target=self._killer, args=(p,), daemon=True).start()
        return "stopping"

    def _killer(self, p):
        for sig, wait in ((signal.SIGINT, self.cfg.stop_grace_s), (signal.SIGTERM, 8.0), (signal.SIGKILL, 3.0)):
            try:
                os.killpg(os.getpgid(p.pid), sig)
            except (ProcessLookupError, PermissionError, OSError):
                return
            end = time.monotonic() + wait
            while time.monotonic() < end:
                if p.poll() is not None:
                    return
                time.sleep(0.1)

    def _give_back(self):
        if self._sdr_held:
            self._sdr_held = False
            time.sleep(1.0)                                                  # let the USB device settle after the process exits
            try:
                self.release_sdr()
            except Exception:
                log.exception("re-opening the SDR power meter")

    # ------------------------------------------------------------------ output parsing
    def _reader(self, p):
        try:
            for raw in p.stdout:
                line = raw.rstrip("\n")
                self.logq.append(line[-300:])
                try:
                    self._parse(line)
                except Exception:
                    log.exception("demod output parse")
        finally:
            rc = p.wait()
            with self.lock:
                s = self.s
                s["exit"], s["t1"] = rc, time.time()
                if self._stop_req:
                    s["state"], s["msg"] = "stopped", f"stopped ({self._stop_req})"
                elif rc == 0:
                    s["state"] = "done"
                    s["msg"] = {"PASS": "finished: every file matches its hash", "FAIL": "finished: a hash check FAILED"}.get(
                        s.get("result"), f"finished, {len(self.files(s['run']))} file(s)")
                else:
                    s["state"] = "failed"
                    s["msg"] = s.get("error") or ("hash check failed (exit 1)" if rc == 1 and s.get("result") == "FAIL"
                                                  else f"exited with code {rc}")
                s["progress"] = 1.0 if s["state"] == "done" and s["mode"] == "file" else s["progress"]
            self._give_back()
            log.info("Demod ended: %s (%s)", self.s["state"], self.s["msg"])

    def _parse(self, line):
        with self.lock:
            s = self.s
            if s["state"] == "starting" and _RE_TS.match(line):
                s["state"] = "running"
            if "Traceback" in line:
                s["error"] = "the receiver crashed (see the log)"
            elif s.get("error") == "the receiver crashed (see the log)" and line and not line.startswith((" ", "[")):
                s["error"] = line[:200]                                      # the exception line after the traceback
            if "UHD" in line and ("Error" in line or "RuntimeError" in line):
                s["error"] = line[-200:]
            m = _RE_FILE.search(line)
            if m:
                s["found"].append({"name": os.path.basename(m.group(1)), "size": int(m.group(2)), "sha256": m.group(3),
                                   "verdict": m.group(4) or ""})
            m = _RE_PLSYNC.search(line)                                      # the end-of-run summary
            if m:
                s["locked"], s["cfo_khz"] = m.group(1) == "True", float(m.group(2))
                s["snr_db"] = float(m.group(3)) if m.group(3) else s["snr_db"]
            m = _RE_PLF.search(line)
            if m:
                s["plframes"], s["dummy"] = int(m.group(1)), int(m.group(2))
            m = _RE_BCHS.search(line)
            if m:
                s["bch_frames"], s["bch_err"] = int(m.group(1)), int(m.group(2))
            if line.strip().startswith("RESULT:"):
                s["result"] = line.split(":", 1)[1].strip()[:12]
            if "|" not in line and "gsse" not in line:
                return
            s["line"] = line.split("EVENT: ")[-1][:300]
            m = _RE_LOCK.search(line)
            if m:
                s["locked"], s["cfo_khz"] = m.group(1) == "LOCK", float(m.group(2))
                if m.group(3):
                    s["snr_db"] = float(m.group(3))
            m = _RE_FR.search(line)
            if m:
                s["plframes"], s["dummy"] = int(m.group(1)), int(m.group(2))
            m = _RE_BCH.search(line)
            if m:
                s["bch_frames"], s["bch_err"] = int(m.group(1)), int(m.group(2))
            m = _RE_GSSE.search(line)
            if m:
                s["gsse_ok"], s["gsse_bad"] = int(m.group(1)), int(m.group(2))
            m = _RE_FILEPCT.search(line)
            if m:
                s["progress"] = float(m.group(1)) / 100.0
            m = _RE_LVL.search(line)
            if m:
                s["level_dbfs"], s["peak_dbfs"] = float(m.group(1)), float(m.group(2))
                self._power.append((time.monotonic(), float(m.group(1))))
            m = _RE_REC.search(line)
            if m:
                s["rec"] = {"on": True, "seconds": int(m.group(1)), "gb": float(m.group(2)), "free_gb": int(m.group(3)),
                            "level_dbfs": float(m.group(4)) if m.group(4) else None,
                            "peak_dbfs": float(m.group(5)) if m.group(5) else None,
                            "overflows": int(m.group(6)) if m.group(6) else 0, "warning": "!!" in line}
            else:
                m = _RE_RECOFF.search(line)
                if m:
                    s["rec"] = {"on": False, "free_gb": int(m.group(1))}
            m = _RE_GATE.search(line)
            if m:
                s["gate"] = {"mode": m.group(1), "windows": int(m.group(2)), "dropped": int(m.group(3) or 0)}
