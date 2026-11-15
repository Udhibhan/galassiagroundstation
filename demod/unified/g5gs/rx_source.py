"""Receiver input: B205mini (record + monitor, or live decode) or an IQ file.

  record  B205mini to the sc16 recorder (full rate, sidecars) and to a complex out for monitoring
  live    same, recording optional (off by default); the demod is meant to run in full
  file    sc16 (rx_samples_to_file --type short / the recorder) or fc32, as fast as the CPU
          allows (or in real time with realtime=True). With need_time, the rx_time tags of
          the recording (its .json time_tags, else .start, else mtime - duration) are put
          back on the stream so the Doppler pre-comp stays on time.

Output is complex64 at the sample rate, after the optional conjugate (spectrally inverted
Ku chain). In live modes the radio's clock is set to the host's UTC before streaming, so
UHD's rx_time tags are UTC.
"""
from __future__ import annotations

import json
import os
import time

import numpy as np
from gnuradio import blocks, gr
import pmt

from . import config as C
from .recorder import RX_TIME, recorder


def _fmt(path, fmt):
    if fmt in ("", "auto"):
        return "sc16" if os.path.splitext(path)[1].lower() in (".sc16", ".cs16") else "fc32"
    return fmt


def file_time_tags(path, n_samples, samp_rate, start_utc="", log=C.log_event):
    """[(sample index, unix time)] for a recording, and where it came from."""
    from . import doppler_lib as dp
    if start_utc:
        return [(0, dp.parse_utc(start_utc))], "start_utc"
    if os.path.exists(path + ".json"):
        try:
            j = json.load(open(path + ".json"))
        except (OSError, ValueError):
            j = {}
        tags = j.get("time_tags")
        if tags:
            return [(int(i), float(t)) for i, t in tags], path + ".json (time_tags)"
    try:
        t0, src = dp.start_time(path, None, n_samples, samp_rate, log=log)
        return [(0, t0)], src
    except SystemExit:
        return [], "none"


class time_tagger(gr.sync_block):
    """Pass-through that puts rx_time tags at given sample indices."""

    def __init__(self, tags):
        gr.sync_block.__init__(self, "g5_time_tagger", in_sig=[np.complex64],
                               out_sig=[np.complex64])
        self.set_output_multiple(C.PY_CHUNK)
        self.tags = sorted(tags)
        self.k = 0

    def work(self, input_items, output_items):
        n = len(input_items[0])
        output_items[0][:] = input_items[0]
        n0 = self.nitems_written(0)
        while self.k < len(self.tags) and self.tags[self.k][0] < n0 + n:
            i, t = self.tags[self.k]
            secs = int(t)
            self.add_item_tag(0, max(i, n0), RX_TIME,
                              pmt.make_tuple(pmt.from_uint64(secs), pmt.from_double(t - secs)))
            self.k += 1
        return n


class rx_source(gr.hier_block2):
    def __init__(self, mode="record", samp_rate=40e6, sym_rate=20e6, rolloff=0.2,
                 centre_freq=1195e6, rx_gain=30, device_args="", lo_offset=-1, bandwidth=0,
                 input_file="", input_format="auto", realtime=False, need_time=False,
                 start_utc="", record=None, rec_dir=".", rec_prefix="rx",
                 min_free=C.MIN_FREE_BYTES, conj=False, meta=None, sim_input="",
                 antenna="RX2", log=C.log_event):
        gr.hier_block2.__init__(self, "g5_rx_source", gr.io_signature(0, 0, 0),
                                gr.io_signature(1, 1, gr.sizeof_gr_complex))
        input_file, rec_dir, sim_input = C.path(input_file), C.path(rec_dir), C.path(sim_input)
        if mode not in ("record", "live", "file"):
            raise ValueError(f"mode {mode!r}: record, live or file")
        self.mode, self.fs, self.log = mode, float(samp_rate), log
        self.usrp = self.rec = self.file_src = None
        self.n_file = 0
        self.time_tags, self.time_src = [], ""
        self.input_file = input_file
        self.antenna = C.antenna_name(antenna)
        if mode == "file":
            if not input_file:
                raise ValueError("file mode needs an input file")
            if not os.path.exists(input_file):
                raise ValueError(f"input file {input_file} not found")
            fmt = self.input_format = _fmt(input_file, input_format)
            isz = gr.sizeof_short * 2 if fmt == "sc16" else gr.sizeof_gr_complex
            self.n_file = os.path.getsize(input_file) // isz
            self.file_src = blocks.file_source(isz, input_file, False)
            chain = [self.file_src]
            if fmt == "sc16":
                chain.append(blocks.interleaved_short_to_complex(True, False, 32768.0))
            if realtime:
                chain.append(blocks.throttle(gr.sizeof_gr_complex, self.fs, True))
            if need_time:
                self.time_tags, self.time_src = file_time_tags(input_file, self.n_file,
                                                               self.fs, start_utc, log)
                if self.time_tags:
                    chain.append(time_tagger(self.time_tags))
                    log(f"input time: first sample {C.iso_utc(self.time_tags[0][1])} from "
                        f"{self.time_src}" + (f", {len(self.time_tags) - 1} gap re-sync tags"
                                              if len(self.time_tags) > 1 else ""))
            log(f"input file {input_file}: {fmt}, {self.n_file} samples = "
                f"{self.n_file / self.fs:.1f} s at {self.fs / 1e6:g} MS/s"
                + (" (real time)" if realtime else " (as fast as possible)"))
        elif sim_input:
            # rehearsal without a radio: an sc16 file looped in real time stands in for the
            # B205mini (no rx_time tags: the recorder uses the host clock)
            if not os.path.exists(sim_input):
                raise ValueError(f"rehearsal input {sim_input} not found")
            u = blocks.file_source(gr.sizeof_short * 2, sim_input, True)
            self.sim = [u, blocks.throttle(gr.sizeof_short * 2, self.fs, True)]
            self.connect(*self.sim)
            u = self.sim[-1]
            self.actual = {"samp_rate": self.fs, "rehearsal_input": sim_input}
            log(f"REHEARSAL: {sim_input} looped at {self.fs / 1e6:g} MS/s instead of the B205mini")
        else:
            from gnuradio import uhd
            dev = C.device_args(device_args, self.fs, "rx")
            self.usrp = uhd.usrp_source(dev, uhd.stream_args(cpu_format="sc16", args="",
                                                             channels=[0]))
            u = self.usrp
            u.set_samp_rate(self.fs)
            if lo_offset is None or lo_offset < 0:
                # move the LO spur out of the signal if the ADC band has room (2 MBd: yes;
                # 20 MBd at 40 MS/s, decimation 1: no, then the DC offset correction handles it)
                lo_offset = sym_rate * (1 + rolloff) / 2 + 1e5
                if lo_offset + sym_rate * (1 + rolloff) / 2 > 0.4 * u.get_clock_rate(0):
                    lo_offset = 0.0
            u.set_center_freq(uhd.tune_request(centre_freq, lo_offset), 0)
            u.set_antenna(self.antenna, 0)
            u.set_gain(rx_gain, 0)
            u.set_bandwidth(bandwidth or C.default_bw(self.fs), 0)
            u.set_auto_dc_offset(True, 0)
            u.set_auto_iq_balance(True, 0)
            u.set_time_now(uhd.time_spec(time.time()))
            self.actual = {"samp_rate": u.get_samp_rate(), "master_clock": u.get_clock_rate(0),
                           "centre_freq": u.get_center_freq(0), "lo_offset": lo_offset,
                           "bandwidth": u.get_bandwidth(0), "gain": u.get_gain(0),
                           "antenna": u.get_antenna(0), "dev_args": dev}
            log(f"B205mini: {self.actual['samp_rate'] / 1e6:g} MS/s, MCR "
                f"{self.actual['master_clock'] / 1e6:g} MHz, {centre_freq / 1e6:g} MHz "
                f"(LO offset {lo_offset / 1e6:g} MHz), BW {self.actual['bandwidth'] / 1e6:g} MHz,"
                f" gain {rx_gain:g} dB, port {self.actual['antenna']}, {dev}")
        if mode != "file":
            rec_on = (mode == "record") if record is None else bool(record)
            m = dict(meta or {}, centre_freq=centre_freq, rx_gain=rx_gain, device=self.actual)
            self.rec = recorder(self.fs, rec_dir, rec_prefix, rec_on, min_free, m, log)
            self.connect(u, self.rec)
            chain = [u, blocks.interleaved_short_to_complex(True, False, 32768.0)]
        if conj:
            chain.append(blocks.conjugate_cc())
        self.chain = chain          # keep Python blocks (time_tagger) referenced: GR doesn't
        for a, b in zip(chain, chain[1:]):
            self.connect(a, b)
        self.connect(chain[-1], self)
        self.started = time.time()

    # ------------------------------------------------------------------ controls
    def set_rx_gain(self, gain):
        if self.usrp is not None:
            self.usrp.set_gain(gain, 0)
            self.log(f"RX gain -> {gain:g} dB")

    def set_antenna(self, antenna):
        """Switch the receive port while running (B205mini: RX2 or TX/RX)."""
        antenna = C.antenna_name(antenna)
        if antenna == self.antenna:
            return
        self.antenna = antenna
        if self.usrp is not None:
            self.usrp.set_antenna(antenna, 0)
            self.log(f"RX port -> {self.usrp.get_antenna(0)}")

    def set_recording(self, on):
        if self.rec is not None:
            self.rec.set_recording(on)

    # ------------------------------------------------------------------ status
    def progress(self):
        """File mode: fraction of the file read (0..1), else None."""
        if self.file_src is None or not self.n_file:
            return None
        try:
            return min(1.0, self.file_src.nitems_written(0) / self.n_file)
        except Exception:                       # noqa: BLE001
            return None

    def stats(self):
        s = {"mode": self.mode, "antenna": self.antenna}
        if self.rec is not None:
            s.update(self.rec.stats())
        p = self.progress()
        if p is not None:
            s["progress"] = p
            s["file_seconds"] = self.n_file / self.fs
        return s
