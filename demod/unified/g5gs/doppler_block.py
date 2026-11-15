"""TLE Doppler pre-compensation as a GRC block (optional: no TLE = straight pass-through).

Wraps doppler_precomp.Precomp (SGP4, light time, 1 ms range grid; field_test/doppler_precomp.py,
handoff_files/24102026_doppler_precomp.md). Time comes from rx_time tags on the stream:
UHD's (live; the radio clock is set to host UTC), rx_source's (file mode: the recording's
.start/.json), or the duty gate's (after each skipped stretch). Every tag re-anchors the
model, so sample gaps (overflows, duty windows) don't put it off time. Without any tag the
first sample is taken as start_utc, or "now".

The pass-information panel reads model / now_unix() / predicted() from this block.
"""
from __future__ import annotations

import time

import numpy as np
from gnuradio import blocks, gr

from . import config as C
from . import doppler_lib as dp
from .recorder import RX_TIME, tag_time


class _precomp_cc(gr.sync_block):
    def __init__(self, pc: dp.Precomp, fs, start_utc, log):
        gr.sync_block.__init__(self, "g5_doppler_precomp", in_sig=[np.complex64],
                               out_sig=[np.complex64])
        self.set_output_multiple(C.PY_CHUNK)
        self.pc, self.fs, self.log = pc, float(fs), log
        self.start_utc = start_utc
        self.ref = None                 # (abs index, unix) anchor
        self.t_last = None

    def _rotate(self, x, out, n_abs):
        t = self.ref[1] + (n_abs - self.ref[0]) / self.fs      # unix time of x[0]
        if self.pc.t0 is None:
            self.pc.set_start(t - self.pc.dt)
            g = self.pc.m.geometry([t])
            self.log(f"Doppler pre-comp: start {C.iso_utc(t)}, el {g['el_deg'][0]:.1f} deg, "
                     f"predicted {g['doppler_hz'][0] / 1e3:+.2f} kHz")
        # Precomp.cycles counts samples from its own start: convert our time to that index
        n_rel = (t + self.pc.dt - self.pc.t0) * self.fs
        ph = (2 * np.pi * self.pc.cycles(n_rel, len(x))).astype(np.float32)
        out[:] = x * (np.cos(ph) + 1j * np.sin(ph)).astype(np.complex64)
        self.t_last = t + len(x) / self.fs

    def work(self, input_items, output_items):
        x, out = input_items[0], output_items[0]
        n = len(x)
        n0 = self.nitems_read(0)
        cuts = []
        for tg in self.get_tags_in_window(0, 0, n, RX_TIME):
            cuts.append((int(tg.offset - n0), tag_time(tg.value)))
        if self.ref is None and (not cuts or cuts[0][0] > 0):
            t0 = dp.parse_utc(self.start_utc) if self.start_utc else time.time()
            if not self.start_utc:
                self.log("Doppler pre-comp: no rx_time tag and no start time: using 'now' "
                         "(host clock) for the first sample")
            self.ref = (n0, t0)
        i = 0
        for k, tt in cuts + [(n, None)]:
            if k > i:
                self._rotate(x[i:k], out[i:k], n0 + i)
            if tt is not None:
                self.ref = (n0 + k, tt)
            i = max(i, k)
        return n


class doppler_precomp(gr.hier_block2):
    def __init__(self, tle="", sat="", samp_rate=40e6, station="1.2977,103.7764,60",
                 rf_freq=dp.FC_KU, sign=1.0, dt=0.0, offset_hz=0.0, start_utc="",
                 log=C.log_event):
        gr.hier_block2.__init__(self, "g5_doppler_precomp",
                                gr.io_signature(1, 1, gr.sizeof_gr_complex),
                                gr.io_signature(1, 1, gr.sizeof_gr_complex))
        self.tle = C.path((tle or "").strip())
        self.model = self.blk = None
        self.sign, self.offset_hz, self.dt = float(sign), float(offset_hz), float(dt)
        if not self.tle:
            self.connect(self, blocks.copy(gr.sizeof_gr_complex), self)   # pass-through
            return
        name, s = dp.load_satrec(self.tle, sat or None)
        self.satrec = s
        self.model = dp.DopplerModel(s, dp.parse_station(station), rf_freq, name)
        pc = dp.Precomp(self.model, None, samp_rate, sign, offset_hz, dt)
        self.blk = _precomp_cc(pc, samp_rate, start_utc, log)
        self.connect(self, self.blk, self)
        age = (time.time() - dp.tle_epoch_unix(s)) / 86400
        log(f"Doppler pre-comp ON: {name} (NORAD {s.satnum}), TLE epoch "
            f"{C.iso_utc(dp.tle_epoch_unix(s))} ({age:+.1f} days from now), station {station}, "
            f"RF {rf_freq / 1e9:g} GHz" + (f", sign {sign:+g}" if sign != 1 else "")
            + (f", dt {dt:+g} s" if dt else "") + (f", offset {offset_hz:+g} Hz" if offset_hz
                                                    else ""))

    @property
    def active(self):
        return self.blk is not None

    def now_unix(self):
        """Time of the latest corrected sample (None before the first one / when off)."""
        return self.blk.t_last if self.blk is not None else None

    def predicted(self, t=None):
        """Geometry at t (default: the latest sample): dict of floats, or None."""
        if self.model is None:
            return None
        t = t if t is not None else self.now_unix()
        if t is None:
            return None
        g = self.model.geometry([t + self.dt])
        return {k: float(v[0]) for k, v in g.items()}

    def passes(self, t0=None, hours=24.0, min_el=5.0):
        if self.model is None:
            return []
        t0 = t0 if t0 is not None else (self.now_unix() or time.time())
        return self.model.passes(t0 - 900, t0 + hours * 3600, min_el)
