"""Full-rate sc16 recorder for the B205mini stream (record + monitor mode).

Writes what `rx_samples_to_file --type short` writes (interleaved little-endian int16 I/Q),
plus two sidecars that the file mode / doppler_precomp read back:
  <file>.start  UTC of the first sample (ISO 8601), written as soon as it is known
  <file>.json   start_unix, sample rate, tuning, preset/MODCOD, and time_tags: one
                [file sample index, unix time] pair per UHD rx_time tag, i.e. after every
                overflow, so a decode can keep the Doppler model on time across sample gaps
Time comes from UHD's rx_time tags; rx_source sets the radio's clock to the host's (keep it
NTP-synced) before streaming.

Recording stops by itself, with a warning, once less than `min_free` bytes (2 GB) are left
on the destination disk; the flowgraph keeps running.
"""
from __future__ import annotations

import json
import os
import threading
import time

import numpy as np
from gnuradio import gr
import pmt

from . import config as C

RX_TIME = pmt.intern("rx_time")


def tag_time(value) -> float:
    """UHD rx_time tag value (uint64 secs, double frac) -> unix seconds."""
    return pmt.to_uint64(pmt.tuple_ref(value, 0)) + pmt.to_double(pmt.tuple_ref(value, 1))


class recorder(gr.sync_block):
    def __init__(self, samp_rate, rec_dir=".", prefix="rx", record=True, min_free=C.MIN_FREE_BYTES,
                 meta=None, log=C.log_event):
        gr.sync_block.__init__(self, "g5_recorder", in_sig=[(np.int16, 2)], out_sig=[])
        self.set_output_multiple(C.PY_CHUNK)
        self.fs = float(samp_rate)
        self.rec_dir = rec_dir or "."
        self.prefix = prefix or "rx"
        self.min_free = int(min_free)
        self.meta = dict(meta or {})
        self.log = log
        self._want = bool(record)
        self._lock = threading.Lock()
        self.f = None
        self.path = ""
        self.file_n = 0                 # samples in the current file
        self.files = []                 # finished recordings
        self.ref = None                 # (absolute sample index, unix time) of the last rx_time
        self.ref_from_uhd = False
        self.overflows = 0
        self.lost = 0
        self.status = "idle"
        self.warning = ""
        self.free = C.free_bytes(self.rec_dir)
        self._next_check = 0
        self._next_level = 0
        self.level_dbfs = None
        self.peak_dbfs = None
        self.n_total = 0
        self._file_tags = []

    # ------------------------------------------------------------------ control
    def set_recording(self, on):
        with self._lock:
            self._want = bool(on)
            if on:
                self.warning = ""

    def recording(self):
        return self.f is not None

    def seconds_left(self):
        """Recording time left before the min_free stop, at 4 B/sample."""
        return max(0.0, (self.free - self.min_free) / (4 * self.fs))

    def stats(self):
        return {"recording": self.recording(), "path": self.path,
                "seconds": self.file_n / self.fs if self.f else 0.0,
                "bytes": 4 * self.file_n if self.f else 0, "free": self.free,
                "seconds_left": self.seconds_left(), "overflows": self.overflows,
                "lost": self.lost, "status": self.status, "warning": self.warning,
                "level_dbfs": self.level_dbfs, "peak_dbfs": self.peak_dbfs,
                "files": list(self.files)}

    # ------------------------------------------------------------------ files
    def _time_of(self, n_abs):
        if self.ref is None:
            return None
        return self.ref[1] + (n_abs - self.ref[0]) / self.fs

    def _open(self, n_abs):
        os.makedirs(self.rec_dir, exist_ok=True)
        self.free = C.free_bytes(self.rec_dir)
        if self.free < self.min_free + 4 * self.fs:      # not even 1 s above the limit
            self._want = False
            self.warning = (f"NOT recording: only {self.free / 2**30:.1f} GB free on "
                            f"{self.rec_dir}")
            self.log(self.warning)
            return
        t0 = self._time_of(n_abs) or time.time()
        self.path = os.path.join(self.rec_dir, f"{self.prefix}_{C.utc_stamp(t0)}.sc16")
        self.f = open(self.path, "wb", buffering=16 << 20)
        self.file_n = 0
        self.start_unix = t0
        self.timed = self.ref_from_uhd
        self._file_tags = [[0, t0]]
        self.status = "recording"
        self._write_sidecars()
        self.log(f"recording -> {self.path} ({4 * self.fs / 1e6:.0f} MB/s, "
                 f"{self.free / 2**30:.0f} GB free = {self.seconds_left() / 60:.1f} min "
                 f"before the {self.min_free / 2**30:g} GB stop)")

    def _close(self, reason="stopped"):
        if self.f is None:
            return
        self.f.close()
        self.f = None
        self._write_sidecars(reason)
        self.files.append(self.path)
        self.status = reason
        self.log(f"recording closed ({reason}): {self.path}, {self.file_n / self.fs:.1f} s, "
                 f"{4 * self.file_n / 1e9:.2f} GB")

    def _write_sidecars(self, end_reason=None):
        with open(self.path + ".start", "w") as f:
            f.write(f"{C.iso_utc(self.start_unix)}\n# first sample of {os.path.basename(self.path)}"
                    f" ({'UHD rx_time' if self.timed else 'host clock, no rx_time tag'}), "
                    f"{self.fs:g} S/s sc16\n")
        j = dict(self.meta, start_unix=self.start_unix, start_utc=C.iso_utc(self.start_unix),
                 samp_rate=self.fs, format="sc16", n_samples=self.file_n,
                 duration_s=self.file_n / self.fs, time_tags=self._file_tags,
                 time_source="uhd rx_time" if self.timed else "host clock",
                 overflows_total=self.overflows, lost_samples_total=self.lost,
                 end_reason=end_reason)
        with open(self.path + ".json", "w") as f:
            json.dump(j, f, indent=1)

    # ------------------------------------------------------------------ stream
    def work(self, input_items, output_items):
        x = input_items[0]
        n = len(x)
        n0 = self.nitems_read(0)
        for t in self.get_tags_in_window(0, 0, n, RX_TIME):
            tt = tag_time(t.value)
            if self.ref is not None:
                exp = self._time_of(t.offset)
                gap = int(round((tt - exp) * self.fs))
                if gap > 0:
                    self.overflows += 1
                    self.lost += gap
                    if self.f is not None:
                        self._file_tags.append([self.file_n + int(t.offset - n0), tt])
            self.ref = (t.offset, tt)
            self.ref_from_uhd = True
        if self.ref is None:
            self.ref = (n0, time.time())         # no UHD tags (file test): host clock
        with self._lock:
            want = self._want
        if want and self.f is None:
            self._open(n0)
        elif not want and self.f is not None:
            self._close()
        if self.f is not None:
            self.f.write(memoryview(np.ascontiguousarray(x)).cast("B"))
            self.file_n += n
        self.n_total += n
        if self.n_total >= self._next_level:
            self._next_level = self.n_total + int(0.25 * self.fs)
            v = x[:65536].astype(np.float32)
            if v.size:
                self.level_dbfs = 10 * np.log10(max(np.mean(v * v) * 2, 1e-3) / 32768 ** 2)
                self.peak_dbfs = 20 * np.log10(max(np.abs(v).max(), 1) / 32768)
        if self.n_total >= self._next_check:
            self._next_check = self.n_total + int(0.5 * self.fs)
            self.free = C.free_bytes(self.rec_dir)
            if self.f is not None:
                if self.free < self.min_free:
                    self.warning = (f"RECORDING STOPPED: less than {self.min_free / 2**30:g} GB "
                                    f"left on {self.rec_dir}")
                    self.log("WARNING " + self.warning)
                    with self._lock:
                        self._want = False
                    self._close("disk full")
                else:
                    self._write_sidecars()   # keep n_samples current in case of a crash
        return n

    def stop(self):
        self._close("flowgraph stopped")
        return True
