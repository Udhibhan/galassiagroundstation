"""
sdr_power.py - total-power meter on the LNB IF using the B206mini. RECEIVE ONLY.

The Sun is a strong Ku-band noise source, so the IF noise power rises several dB when the dish points at it.
~10 readings/s; each reading = mean |x|^2 over ~50 ms (200 000 samples at 4 Msps), kept LINEAR (dB is
only for display). Every reading carries a saturation flag: any |sample| > 0.7 FS -> "reduce gain" and the
measurement that used it is marked invalid.

SAFETY (B206mini is RECEIVE ONLY): this file only ever creates an RX streamer on RX2 (get_rx_stream,
set_rx_*). It never creates a TX streamer, never calls set_tx_* / get_tx_stream, never uses a USRP Sink,
uhd_siggen, tx_waveforms, tx_samples_from_file, benchmark_rate --tx_rate or any EEPROM/firmware tool.
The TX/RX port stays unconnected. Keep the SDR input at or below -15 dBm.
"""
import logging
import math
import threading
import time
from collections import deque

import clock
import config

log = logging.getLogger(__name__)


class PowerMeter:
    """Common part: a reader thread, a ring buffer of readings and averaging helpers.
    Subclasses implement _open() and _measure() -> (mean |x|^2 linear, peak |x|)."""
    name = "power meter"

    def __init__(self, cfg: config.SdrConfig = config.SDR):
        self.cfg = cfg
        self.gain_db = cfg.gain_db
        self.available = False
        self.error = "opening"
        self.readings = deque(maxlen=20000)          # ~30 min at 10/s
        self.on_reading = None                       # callback(reading) - the app logs every reading
        self.sat_total = 0
        self._stop = threading.Event()
        self._thread = None

    # ---- thread
    def start(self):
        self._thread = threading.Thread(target=self._run, name="sdr", daemon=True)
        self._thread.start()
        return self

    def _run(self):
        period = self.cfg.reading_period_s
        opened, fails = False, 0
        while not self._stop.is_set():
            if not opened:
                # keep retrying: the SDR may be plugged in later, or busy (sdr_plot.py / uhd_fft still open)
                try:
                    self._release()
                    self._open()
                    opened, fails = True, 0
                    self.available, self.error = True, None
                except Exception as e:               # no uhd module, no device, USB error, device busy...
                    self.available, self.error = False, str(e) or e.__class__.__name__
                    log.warning("SDR not available (retrying in 5 s): %s", self.error)
                    self._stop.wait(5.0)
                    continue
            t0 = time.monotonic()
            try:
                lin, peak = self._measure()
                fails = 0
            except Exception as e:
                fails += 1
                self.error = f"read failed: {e}"
                log.warning("SDR: %s", self.error)
                if fails >= 5:                       # unplugged / wedged -> reopen
                    opened = False
                self._stop.wait(1.0)
                continue
            tm = time.monotonic()
            mid = tm - min(0.025, tm - t0)                         # middle of the ~50 ms block
            sat = peak > self.cfg.saturation_fs
            if sat:
                self.sat_total += 1
            r = {"tm": mid, "t": clock.now().timestamp() - (tm - mid), "lin": lin,
                 "db": 10 * math.log10(lin + 1e-30), "peak": peak, "sat": sat}
            self.readings.append(r)
            cb = self.on_reading
            if cb:
                try:
                    cb(r)
                except Exception:
                    log.exception("power reading callback")
            self._stop.wait(max(0.0, period - (time.monotonic() - t0)))

    def close(self):
        self._stop.set()

    # ---- API
    def latest(self):
        return self.readings[-1] if self.readings else None

    def since(self, tm0, tm1=None):
        return [r for r in list(self.readings) if r["tm"] >= tm0 and (tm1 is None or r["tm"] <= tm1)]

    def stats(self, tm0, tm1=None) -> dict:
        rs = self.since(tm0, tm1)
        if not rs:
            return {"n": 0, "lin": None, "db": None, "sat": False}
        lin = sum(r["lin"] for r in rs) / len(rs)
        return {"n": len(rs), "lin": lin, "db": 10 * math.log10(lin + 1e-30), "sat": any(r["sat"] for r in rs),
                "peak": max(r["peak"] for r in rs)}

    def average(self, seconds, cancel=None) -> dict:
        """Collect readings for `seconds` from now and return stats() (n = 0 if cancelled / no SDR)."""
        tm0 = time.monotonic()
        while time.monotonic() - tm0 < seconds:
            if cancel and cancel():
                return {"n": 0, "lin": None, "db": None, "sat": False, "cancelled": True}
            time.sleep(0.05)
        return self.stats(tm0)

    def status(self) -> dict:
        r = self.latest()
        fresh = r is not None and time.monotonic() - r["tm"] < 2.0
        return {"available": bool(self.available and fresh), "error": self.error,
                "name": self.name, "gain_db": self.gain_db, "db": r["db"] if fresh else None,
                "sat": bool(r and fresh and r["sat"]), "sat_total": self.sat_total}

    # ---- subclass
    def _release(self):
        """Drop any half-open device before a (re)open. Subclasses with hardware override this."""

    def _open(self):
        raise NotImplementedError

    def _measure(self):
        raise NotImplementedError


class SdrPower(PowerMeter):
    name = "B206mini (RX2, receive only)"

    def _release(self):
        self._stream = self._usrp = None
        time.sleep(0.2)

    def _open(self):
        import numpy as np
        import uhd
        cfg = self.cfg
        self._np, self._uhd = np, uhd
        u = uhd.usrp.MultiUSRP(cfg.args)
        u.set_rx_rate(cfg.rate_hz, 0)
        u.set_rx_freq(uhd.types.TuneRequest(cfg.if_freq_hz), 0)
        u.set_rx_gain(cfg.gain_db, 0)
        u.set_rx_antenna(cfg.antenna, 0)               # RX2 only
        self._usrp = u
        self._stream = u.get_rx_stream(uhd.usrp.StreamArgs("fc32", "sc16"))      # RX streamer only
        self._buf = np.zeros((1, 50_000), dtype=np.complex64)
        self.gain_db = u.get_rx_gain(0)
        log.info("SDR: %s, RX2, IF %.3f GHz, %.1f Msps, gain %.1f dB", cfg.args, cfg.if_freq_hz / 1e9,
                 cfg.rate_hz / 1e6, self.gain_db)

    def _measure(self):
        """One burst of cfg.samples (+ settling), mean |x|^2 (linear, FS^2) and the largest |x|."""
        uhd, np = self._uhd, self._np
        skip = 2000
        want = self.cfg.samples + skip
        cmd = uhd.types.StreamCMD(uhd.types.StreamMode.num_done)
        cmd.num_samps = want
        cmd.stream_now = True
        self._stream.issue_stream_cmd(cmd)
        md = uhd.types.RXMetadata()
        got, acc, n, peak = 0, 0.0, 0, 0.0
        while got < want:
            k = self._stream.recv(self._buf, md, 1.0)
            if k == 0:
                if md.error_code != uhd.types.RXMetadataErrorCode.none:
                    raise RuntimeError(str(md.strerror()))
                break
            x = self._buf[0, :k]
            lo = max(0, skip - got)                       # drop the first samples (front-end settling)
            if lo < k:
                p = np.abs(x[lo:]) ** 2
                acc += float(p.sum())
                n += len(p)
                peak = max(peak, float(np.sqrt(p.max())))
            got += k
        if n == 0:
            raise RuntimeError("no samples")
        return acc / n, peak

    def close(self):
        super().close()
        time.sleep(0.15)
        self._stream = self._usrp = None
