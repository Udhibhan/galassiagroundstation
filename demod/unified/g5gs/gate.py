"""Duty gate: decouples the demodulator from the full-rate stream.

A GNU Radio block can't drop samples when its output is full (the scheduler simply stops
calling it, and the back-pressure reaches the radio as overflows). So the gate is a sink and
a separate source joined by a bounded queue, wrapped in one hier block:

  pass   direct connection (file mode: the demod sets the pace, nothing is lost)
  drop   every sample goes through while the demod keeps up; when the queue is full whole
         chunks are dropped (live decode at a rate the CPU almost manages)
  duty   `window` s out of every `period` s go through (record + monitor at 40 MS/s,
         where the demod runs at ~0.43x real time): a live lock / SNR check while the
         recording runs untouched

After every gap the source puts an rx_time tag on the first sample it outputs, so the
Doppler pre-comp downstream stays on time; the demod reacquires in each window (~0.05 s
at 20 MBd).
"""
from __future__ import annotations

import collections
import threading
import time

import numpy as np
from gnuradio import blocks, gr
import pmt

from . import config as C
from .recorder import RX_TIME, tag_time


class _Shared:
    def __init__(self, cap_items):
        self.q = collections.deque()
        self.items = 0
        self.cap = int(cap_items)
        self.cv = threading.Condition()
        self.closed = False
        self.dropped = 0
        self.passed = 0
        self.windows = 0
        self.last_window_unix = None


class _gate_sink(gr.sync_block):
    def __init__(self, sh, fs, mode, window, period):
        gr.sync_block.__init__(self, "g5_gate_sink", in_sig=[np.complex64], out_sig=[])
        self.set_output_multiple(C.PY_CHUNK)
        self.sh, self.fs, self.mode = sh, float(fs), mode
        self.win = int(window * fs)
        self.per = max(int(period * fs), self.win)
        self.ref = None                     # (abs index, unix) from rx_time tags
        self.gap = True                     # next queued chunk follows a gap

    def _t(self, n):
        if self.ref is None:
            return None
        return self.ref[1] + (n - self.ref[0]) / self.fs

    def _push(self, x, n_abs):
        sh = self.sh
        with sh.cv:
            if sh.items + len(x) > sh.cap:
                sh.dropped += len(x)
                self.gap = True
                return
            sh.q.append((np.array(x), self._t(n_abs) if self.gap else None, self.gap))
            if self.gap:
                sh.windows += 1
                sh.last_window_unix = self._t(n_abs) or time.time()
            self.gap = False
            sh.items += len(x)
            sh.passed += len(x)
            sh.cv.notify()

    def work(self, input_items, output_items):
        x = input_items[0]
        n = len(x)
        n0 = self.nitems_read(0)
        for t in self.get_tags_in_window(0, 0, n, RX_TIME):
            self.ref = (t.offset, tag_time(t.value))
            self.gap = True                 # the radio itself lost samples here
        if self.mode == "drop":
            self._push(x, n0)
            return n
        # duty: keep [k*per, k*per + win)
        i = 0
        while i < n:
            ph = (n0 + i) % self.per
            if ph < self.win:
                m = min(n - i, self.win - ph)
                self._push(x[i:i + m], n0 + i)
                i += m
            else:
                self.gap = True             # skipped by design: not counted as dropped
                i += min(n - i, self.per - ph)
        return n

    def stop(self):
        with self.sh.cv:
            self.sh.closed = True
            self.sh.cv.notify_all()
        return True


class _gate_source(gr.sync_block):
    def __init__(self, sh):
        gr.sync_block.__init__(self, "g5_gate_source", in_sig=[], out_sig=[np.complex64])
        self.set_min_output_buffer(4 * C.PY_CHUNK)
        self.set_min_noutput_items(C.PY_CHUNK)
        self.sh = sh
        self.cur = None
        self.pos = 0

    def work(self, input_items, output_items):
        out = output_items[0]
        sh = self.sh
        k = 0
        while k < len(out):
            if self.cur is None:
                with sh.cv:
                    if not sh.q:
                        if k:
                            break
                        if sh.closed:
                            return -1
                        sh.cv.wait(0.1)
                        if not sh.q:
                            return 0
                    self.cur = sh.q.popleft()
                    self.pos = 0
                    sh.items -= len(self.cur[0])
                x, t, gap = self.cur
                if gap and t is not None:
                    secs = int(t)
                    self.add_item_tag(0, self.nitems_written(0) + k, RX_TIME,
                                      pmt.make_tuple(pmt.from_uint64(secs),
                                                     pmt.from_double(t - secs)))
            x = self.cur[0]
            m = min(len(out) - k, len(x) - self.pos)
            out[k:k + m] = x[self.pos:self.pos + m]
            k += m
            self.pos += m
            if self.pos >= len(x):
                self.cur = None
        return k


class duty_gate(gr.hier_block2):
    def __init__(self, mode="pass", samp_rate=40e6, window=1.0, period=5.0, queue_s=0.5):
        gr.hier_block2.__init__(self, "g5_duty_gate",
                                gr.io_signature(1, 1, gr.sizeof_gr_complex),
                                gr.io_signature(1, 1, gr.sizeof_gr_complex))
        if mode not in ("pass", "drop", "duty"):
            raise ValueError(f"gate mode {mode!r}: pass, drop or duty")
        self.mode, self.window, self.period = mode, window, period
        if mode == "pass":
            self.sh = None
            self.connect(self, blocks.copy(gr.sizeof_gr_complex), self)   # pass-through
            return
        if mode == "drop" and samp_rate > C.LIVE_DECODE_MAX_SPS:
            C.log_event(f"WARNING live decode at {samp_rate / 1e6:g} MS/s: this laptop demodulates "
                        f"only ~{C.LIVE_DECODE_MAX_SPS / 1e6:g} MS/s live, so the demod will drop "
                        "samples (the recording is unaffected). Use record mode and decode the "
                        "file afterwards.")
        cap = int(max(queue_s, window * 1.05 if mode == "duty" else queue_s) * samp_rate)
        self.sh = _Shared(cap)
        self.snk = _gate_sink(self.sh, samp_rate, mode, window, period)
        self.src = _gate_source(self.sh)
        self.connect(self, self.snk)
        self.connect(self.src, self)

    def stats(self):
        if self.sh is None:
            return {"mode": "pass"}
        sh = self.sh
        return {"mode": self.mode, "window_s": self.window, "period_s": self.period,
                "windows": sh.windows, "passed": sh.passed, "dropped": sh.dropped,
                "queue_fill": sh.items / sh.cap if sh.cap else 0,
                "last_window_unix": sh.last_window_unix}


def auto_mode(rx_mode, samp_rate, duty_period, live_max=20e6):
    """Gate mode and period for an RX mode: file -> pass; live -> drop; record -> drop
    when the CPU can demodulate this rate live (<= ~20 MS/s), else duty."""
    if rx_mode == "file":
        return "pass", 0.0
    if rx_mode == "live":
        return "drop", 0.0
    if duty_period and duty_period > 0:
        return "duty", float(duty_period)
    return ("drop", 0.0) if samp_rate <= live_max else ("duty", 5.0)
