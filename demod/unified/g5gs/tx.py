"""Transmitter blocks: payload, DVB-S2 (gr-dtv) + Link X dummy frames, impairments, output.

  tx_payload   bits for dvb_bbscrambler_bb (one bit per byte), from
                 gsse      a file or folder as the Link X / SX sends it: GSUFTP in GSSE
                           (rev1.0), GSUFTP/UDP/IPv4 in GSSE (rev1.2), or plain UDP
                           (udpraw); tx_simulator.py's encoder. Loops `repeat` times
                           (0 = forever), then `flush` empty BBFRAMEs so the receiver's LDPC
                           decoder releases its last frames
                 bbframes  a BBFRAME file (tx_simulator --out-file, or a capture), looped
                 gse_tap0  the old gr-dvbgse path: standard GSE from tap0 (field-test
                           send_dir/recv_dir transfers; run setup_tx_ifaces.sh first)
  dvbs2_tx     gr-dtv BBscrambler, BCH, LDPC, interleaver, modulator, PL framer (pilots on,
               gold code 0), then dummy PLFRAMEs (lead-in and after every data frame, as the
               Link X sends them when idle: RF ICD 1045318 sec. 3.1) and the RRC (roll-off
               0.2) at `sps`. A hier block because the gr-dtv GRC blocks take the MODCOD
               only as fixed enums.
  iq_select    generate (pass-through) or replay an IQ file (sc16/fc32) instead
  impairments  carrier on/off, level (dB), a transient schedule, CFO, TLE Doppler, AWGN
  tx_sink      B205mini (TX/RX port) or an IQ file (sc16/fc32 + .start/.json sidecars)
"""
from __future__ import annotations

import json
import os
import threading
import time

import numpy as np
from gnuradio import analog, blocks, dtv, gr
from gnuradio import filter as grfilter
from gnuradio.filter import firdes

from . import config as C
from . import gsrx  # noqa: F401
import bbframe  # noqa: E402
import make_linkx_iq as mli  # noqa: E402
import tx_simulator  # noqa: E402

RATES = {"1/4": dtv.C1_4, "1/3": dtv.C1_3, "2/5": dtv.C2_5, "1/2": dtv.C1_2, "3/5": dtv.C3_5,
         "2/3": dtv.C2_3, "3/4": dtv.C3_4, "4/5": dtv.C4_5, "5/6": dtv.C5_6, "8/9": dtv.C8_9,
         "9/10": dtv.C9_10}
MODS = {"QPSK": dtv.MOD_QPSK, "8PSK": dtv.MOD_8PSK}


def txlog(msg):
    C.log_event(msg, "TX")


# ---------------------------------------------------------------------------- payload
class _bbframe_bits(gr.sync_block):
    """BBFRAMEs (a list of bytes objects) -> bits, looped `repeat` times, then `flush` empty
    frames, then done."""

    def __init__(self, frames, flen, repeat=0, flush=100):
        gr.sync_block.__init__(self, "g5_bbframe_bits", in_sig=[], out_sig=[np.uint8])
        self.bits = np.unpackbits(np.frombuffer(b"".join(frames), np.uint8))
        self.flush_bits = np.unpackbits(np.frombuffer(bbframe.build_bbframe(b"", flen) * flush,
                                                      np.uint8)) if flush else np.zeros(0, np.uint8)
        self.flen_bits = flen * 8
        self.n_frames = len(frames)
        self.repeat = int(repeat)
        self.rep = 0
        self.pos = 0
        self.in_flush = False
        self.frames_out = 0
        self.set_output_multiple(self.flen_bits)

    def work(self, input_items, output_items):
        out = output_items[0]
        k = 0
        while k + self.flen_bits <= len(out):
            src = self.flush_bits if self.in_flush else self.bits
            if self.pos >= len(src):
                if self.in_flush:
                    break
                self.rep += 1
                self.pos = 0
                if self.repeat and self.rep >= self.repeat:
                    self.in_flush = True
                    if not len(self.flush_bits):
                        break
                continue
            m = min(len(out) - k, len(src) - self.pos)
            m -= m % self.flen_bits
            out[k:k + m] = src[self.pos:self.pos + m]
            self.pos += m
            k += m
            self.frames_out += m // self.flen_bits
        if k == 0 and self.in_flush and self.pos >= len(self.flush_bits):
            return -1
        return k


class tx_payload(gr.hier_block2):
    def __init__(self, mode="gsse", files="", stack="rev1.0", chunk_size=0, repeat=0, flush=100,
                 bbframe_file="", frame_len=2001, modcod="QPSK1/4", frame_size="normal",
                 isi=0, sis=False, enabled=True, gse_src_ip="44.0.0.3", gse_dst_ip="10.0.1.190"):
        gr.hier_block2.__init__(self, "g5_tx_payload", gr.io_signature(0, 0, 0),
                                gr.io_signature(1, 1, gr.sizeof_char))
        self.mode, self.src = mode, None
        files, bbframe_file = C.path(files), C.path(bbframe_file)
        self.flen = int(frame_len)
        if not enabled:                         # replay mode: no payload at all
            self.src = blocks.vector_source_b([], False)
            self.connect(self.src, self)
            self.text = "none (replaying an IQ file)"
            return
        if mode == "gsse":
            if not files or not os.path.exists(files):
                raise ValueError(f"payload files {files!r} not found")
            targets = []
            if os.path.isdir(files):
                for root, _, names in os.walk(files):
                    for nm in sorted(names):
                        full = os.path.join(root, nm)
                        targets.append((full, os.path.relpath(full, files)))
            else:
                targets.append((files, os.path.basename(files)))
            import argparse
            a = argparse.Namespace(stack=stack, repeat=1, src="10.128.0.4", dst="10.129.0.1",
                                   dport=5555, mtu=9000,
                                   chunk_size=chunk_size or {"rev1.0": 10_000_000,
                                                             "udpraw": 1024}.get(stack, 50_000))
            stream = tx_simulator.build_stream(targets, a)
            usable = self.flen - bbframe.BB_HEADER_LEN
            frames = [bbframe.build_bbframe(stream[o:o + usable], self.flen, isi=isi, sis=sis)
                      for o in range(0, len(stream), usable)]
            self.text = (f"{len(targets)} file(s) from {files}, stack {stack}, chunk "
                         f"{a.chunk_size} B: {len(stream)} B GSSE = {len(frames)} BBFRAMEs per pass")
        elif mode == "bbframes":
            data = open(bbframe_file, "rb").read()
            frames = [data[o:o + self.flen] for o in range(0, len(data) // self.flen * self.flen,
                                                           self.flen)]
            if not frames:
                raise ValueError(f"{bbframe_file}: no whole {self.flen}-byte BBFRAMEs")
            self.text = f"{len(frames)} BBFRAMEs from {bbframe_file}"
        elif mode == "gse_tap0":
            import dvbgse
            cons, rate = C.parse_modcod(modcod)
            gr_rate = getattr(dvbgse, "C" + rate.replace("/", "_"))
            fs = dvbgse.FECFRAME_NORMAL if frame_size == "normal" else dvbgse.FECFRAME_SHORT
            self.src = dvbgse.bbheader_source(
                dvbgse.STANDARD_DVBS2, fs, gr_rate, dvbgse.RO_0_20, dvbgse.INBAND_OFF, 168,
                4000000, dvbgse.PING_REPLY_OFF, dvbgse.IPADDR_SPOOF_OFF, gse_src_ip, gse_dst_ip,
                dvbgse.PADDING_PACKET_LENGTH_0)
            self.connect(self.src, self)
            self.text = "standard GSE from tap0 (gr-dvbgse)"
            txlog(f"payload: {self.text}")
            return
        else:
            raise ValueError(f"payload mode {mode!r}: gsse, bbframes or gse_tap0")
        self.src = _bbframe_bits(frames, self.flen, repeat, flush)
        self.connect(self.src, self)
        txlog(f"payload: {self.text}, " + (f"{repeat} pass(es) + {flush} flush frames"
                                           if repeat else "looped forever"))

    def describe_progress(self):
        s = self.src
        if isinstance(s, _bbframe_bits):
            return (f"{self.text}; pass {s.rep + (0 if s.in_flush else 1)}"
                    + (f"/{s.repeat}" if s.repeat else " (looping)")
                    + f", {s.frames_out} BBFRAMEs sent" + (", flushing" if s.in_flush else ""))
        return self.text


# ---------------------------------------------------------------------------- DVB-S2
class _pl_framer(gr.basic_block):
    """gr-dtv PLFRAMEs (2 samples/symbol, zero-stuffed) -> 1 sample/symbol, with dummy
    PLFRAMEs: `lead` before the first data frame and `gap` after every data frame."""

    def __init__(self, plen, lead=200, gap=0):
        gr.basic_block.__init__(self, "g5_pl_framer", in_sig=[np.complex64],
                                out_sig=[np.complex64])
        self.plen = int(plen)
        scr = mli.pl_scrambler(max(self.plen, 3240))
        self.dummy = mli.dummy_plframe(scr).astype(np.complex64)
        self.dl = len(self.dummy)
        self.lead_left = int(lead)
        self.gap = int(gap)
        self.gap_left = 0
        self.n_data = 0
        self.n_dummy = 0
        self.set_min_output_buffer(4 * (self.plen + self.dl))

    def forecast(self, noutput_items, ninputs):
        need = 0 if (self.lead_left or self.gap_left) else 2 * self.plen
        return [need] * ninputs

    def general_work(self, input_items, output_items):
        x, out = input_items[0], output_items[0]
        k = c = 0
        while True:
            if self.lead_left or self.gap_left:
                if len(out) - k < self.dl:
                    break
                out[k:k + self.dl] = self.dummy
                k += self.dl
                self.n_dummy += 1
                if self.lead_left:
                    self.lead_left -= 1
                else:
                    self.gap_left -= 1
                continue
            if len(x) - c >= 2 * self.plen and len(out) - k >= self.plen:
                out[k:k + self.plen] = x[c:c + 2 * self.plen:2]
                c += 2 * self.plen
                k += self.plen
                self.n_data += 1
                self.gap_left = self.gap
                continue
            break
        self.consume(0, c)
        return k

    def stats(self):
        return {"data": self.n_data, "dummy": self.n_dummy}


def rrc_taps(sps, rolloff=0.2, span=22):
    return firdes.root_raised_cosine(sps, sps, 1.0, rolloff, span * sps + 1)


def signal_power(source="generate", replay_file="", sps=2, rolloff=0.2):
    """Mean power of the impairments' input: the RRC output for unit-power symbols, or the
    first 1M samples of the replay file."""
    if source == "replay" and replay_file:
        p = C.path(replay_file)
        if p.lower().endswith((".sc16", ".cs16")):
            v = np.fromfile(p, "<i2", count=2 << 20).astype(np.float32) / 32768.0
            return float(np.mean(v * v) * 2) or 1.0
        x = np.fromfile(p, np.complex64, count=1 << 20)
        return float(np.mean(np.abs(x) ** 2)) or 1.0
    taps = rrc_taps(int(sps), rolloff)
    return float(np.sum(np.square(taps)) / sps)


def doppler_start(tle="", doppler_start_utc=""):
    """Unix time the Doppler simulation starts at (now if not given), None without a TLE."""
    if not tle:
        return None
    if doppler_start_utc:
        from . import doppler_lib as dp
        return dp.parse_utc(doppler_start_utc)
    return time.time()


class dvbs2_tx(gr.hier_block2):
    def __init__(self, modcod="QPSK1/4", frame_size="normal", sps=2, rolloff=0.2, lead=200,
                 gap=0, gold_code=0):
        gr.hier_block2.__init__(self, "g5_dvbs2_tx", gr.io_signature(1, 1, gr.sizeof_char),
                                gr.io_signature(1, 1, gr.sizeof_gr_complex))
        cons, rate = C.parse_modcod(modcod)
        fs = dtv.FECFRAME_NORMAL if frame_size == "normal" else dtv.FECFRAME_SHORT
        r, m = RATES[rate], MODS[cons]
        self.chain = [dtv.dvb_bbscrambler_bb(dtv.STANDARD_DVBS2, fs, r),
                      dtv.dvb_bch_bb(dtv.STANDARD_DVBS2, fs, r),
                      dtv.dvb_ldpc_bb(dtv.STANDARD_DVBS2, fs, r, dtv.MOD_OTHER),
                      dtv.dvbs2_interleaver_bb(fs, r, m),
                      dtv.dvbs2_modulator_bc(fs, r, m, dtv.INTERPOLATION_OFF),
                      dtv.dvbs2_physical_cc(fs, r, m, dtv.PILOTS_ON, gold_code)]
        self.framer = _pl_framer(C.plframe_syms(cons, frame_size), lead, gap)
        self.taps = rrc_taps(sps, rolloff)
        self.rrc = grfilter.interp_fir_filter_ccf(int(sps), self.taps)
        # mean output power for unit-power symbols (pilots, headers, dummies are unit too)
        self.power = float(np.sum(np.square(self.taps)) / sps)
        self.connect(self, *self.chain, self.framer, self.rrc, self)

    def stats(self):
        return self.framer.stats()


# ---------------------------------------------------------------------------- replay
class iq_select(gr.hier_block2):
    def __init__(self, mode="generate", replay_file="", replay_format="auto", replay_loop=True):
        gr.hier_block2.__init__(self, "g5_iq_select", gr.io_signature(1, 1, gr.sizeof_gr_complex),
                                gr.io_signature(1, 1, gr.sizeof_gr_complex))
        self.mode = mode
        replay_file = C.path(replay_file)
        if mode == "generate":
            self.connect(self, blocks.copy(gr.sizeof_gr_complex), self)
            return
        if mode != "replay":
            raise ValueError(f"source {mode!r}: generate or replay")
        if not os.path.exists(replay_file):
            raise ValueError(f"replay file {replay_file!r} not found")
        fmt = replay_format
        if fmt in ("", "auto"):
            fmt = "sc16" if replay_file.lower().endswith((".sc16", ".cs16")) else "fc32"
        self.connect(self, blocks.null_sink(gr.sizeof_gr_complex))
        if fmt == "sc16":
            self.src = blocks.file_source(gr.sizeof_short * 2, replay_file, replay_loop)
            self.connect(self.src, blocks.interleaved_short_to_complex(True, False, 32768.0), self)
        else:
            self.src = blocks.file_source(gr.sizeof_gr_complex, replay_file, replay_loop)
            self.connect(self.src, self)
        txlog(f"replaying {replay_file} ({fmt})" + (", looped" if replay_loop else ""))


# ---------------------------------------------------------------------------- impairments
def parse_schedule(text):
    """'5:off, 6:on, 10:-20dB, 12:0dB' -> [(5.0, 'off', None), ...] sorted by time."""
    out = []
    for item in str(text or "").replace(";", ",").split(","):
        item = item.strip()
        if not item:
            continue
        t, act = item.split(":", 1)
        act = act.strip().lower()
        if act in ("on", "off"):
            out.append((float(t), act, None))
        elif act.endswith("db"):
            out.append((float(t), "level", float(act[:-2])))
        else:
            raise ValueError(f"schedule item {item!r}: use t:on, t:off or t:<level>dB")
    return sorted(out)


class _impair_cc(gr.sync_block):
    """Gain (carrier on/off, level, schedule in signal time) and the TLE Doppler phase."""

    def __init__(self, fs, g0, level_db, schedule, precomp):
        gr.sync_block.__init__(self, "g5_tx_impair", in_sig=[np.complex64],
                               out_sig=[np.complex64])
        self.set_output_multiple(C.PY_CHUNK)
        self.fs, self.g0 = float(fs), float(g0)
        self.level_db, self.carrier = float(level_db), True
        self.sched = list(schedule)
        self.pc = precomp
        self.lock = threading.Lock()
        self.last_event = ""

    def gain(self):
        return self.g0 * 10 ** (self.level_db / 20) if self.carrier else 0.0

    def work(self, input_items, output_items):
        x, out = input_items[0], output_items[0]
        n = len(x)
        n0 = self.nitems_written(0)
        i = 0
        with self.lock:
            while i < n:
                t_now = (n0 + i) / self.fs
                while self.sched and self.sched[0][0] <= t_now:
                    t, act, val = self.sched.pop(0)
                    if act == "level":
                        self.level_db = val
                    else:
                        self.carrier = act == "on"
                    self.last_event = f"t={t:g}s {act}" + (f" {val:g} dB" if val is not None
                                                          else "")
                    txlog("schedule " + self.last_event)
                j = n if not self.sched else min(n, max(i + 1, int(np.ceil(
                    self.sched[0][0] * self.fs)) - n0))
                y = x[i:j] * np.complex64(self.gain())
                if self.pc is not None:
                    ph = (2 * np.pi * self.pc.cycles(n0 + i, j - i)).astype(np.float32)
                    y *= np.cos(ph) + 1j * np.sin(ph)
                out[i:j] = y
                i = j
        return n


class impairments(gr.hier_block2):
    def __init__(self, samp_rate=40e6, signal_power=1.0, rms_dbfs=-12.0, level_db=0.0,
                 cfo_hz=0.0, esn0_db="", sps=2, schedule="", tle="", sat="",
                 station="1.2977,103.7764,60", rf_freq=13.95e9, doppler_start_utc="",
                 doppler_dt=0.0, seed=0, carrier=True):
        gr.hier_block2.__init__(self, "g5_tx_impairments",
                                gr.io_signature(1, 1, gr.sizeof_gr_complex),
                                gr.io_signature(1, 1, gr.sizeof_gr_complex))
        self.fs = float(samp_rate)
        tle = C.path(tle)
        g0 = 10 ** (rms_dbfs / 20) / np.sqrt(signal_power)
        self.pc = None
        self.dop_text = ""
        self.start_unix = None
        if tle:
            from . import doppler_lib as dp
            name, s = dp.load_satrec(tle, sat or None)
            model = dp.DopplerModel(s, dp.parse_station(station), rf_freq, name)
            t0 = dp.parse_utc(doppler_start_utc) if doppler_start_utc not in ("", None) \
                else time.time()
            # sign -1: apply the Doppler (the receiver's pre-comp uses +1 to remove it)
            self.pc = dp.Precomp(model, t0, samp_rate, sign=-1.0, dt=doppler_dt)
            g = model.geometry([t0 + doppler_dt])
            self.start_unix = t0
            self.dop_text = (f"{name} from {C.iso_utc(t0)} (el {g['el_deg'][0]:.1f} deg, "
                             f"{g['doppler_hz'][0] / 1e3:+.1f} kHz at start)")
            txlog(f"Doppler simulation: {self.dop_text}")
        self.blk = _impair_cc(samp_rate, g0, level_db, parse_schedule(schedule), self.pc)
        self.blk.carrier = bool(carrier)
        self.cfo = float(cfo_hz)
        self.rot = blocks.rotator_cc(2 * np.pi * self.cfo / self.fs)
        self.connect(self, self.blk, self.rot)
        self.esn0 = None if esn0_db in ("", None) else float(esn0_db)
        if self.esn0 is None:
            self.connect(self.rot, self)
            return
        # Es/N0 after the receiver's matched filter: noise bandwidth = Rs = fs / sps
        p = (g0 ** 2) * signal_power
        sigma = np.sqrt(p * sps * 10 ** (-self.esn0 / 10))
        self.noise = analog.fastnoise_source_c(analog.GR_GAUSSIAN, sigma, seed, 1 << 22)
        self.add = blocks.add_cc()
        self.connect(self.rot, (self.add, 0))
        self.connect(self.noise, (self.add, 1))
        self.connect(self.add, self)

    def set_carrier(self, on):
        with self.blk.lock:
            self.blk.carrier = bool(on)
        txlog("carrier ON" if on else "carrier OFF")

    def set_level_db(self, db):
        with self.blk.lock:
            self.blk.level_db = float(db)
        txlog(f"level -> {db:g} dB")

    def set_cfo(self, hz):
        self.cfo = float(hz)
        self.rot.set_phase_inc(2 * np.pi * self.cfo / self.fs)
        txlog(f"CFO -> {hz:g} Hz")

    def stats(self):
        b = self.blk
        dop = None
        if self.pc is not None:
            t = b.nitems_written(0) / self.fs
            g = self.pc.m.geometry([self.pc.t0 + t])
            dop = (f"{g['doppler_hz'][0] / 1e3:+.2f} kHz, el {g['el_deg'][0]:.1f} deg at "
                   f"{C.iso_utc(self.pc.t0 + t)}")
        return {"carrier": b.carrier,
                "text": (("carrier ON" if b.carrier else "CARRIER OFF")
                         + f", level {b.level_db:+.1f} dB, CFO {self.cfo:g} Hz"
                         + (f", Es/N0 {self.esn0:g} dB" if self.esn0 is not None else "")
                         + (f", schedule: {len(b.sched)} left" + (f", last {b.last_event}"
                                                                 if b.last_event else "")
                            if b.sched or b.last_event else "")),
                "doppler": dop}


# ---------------------------------------------------------------------------- output
class tx_sink(gr.hier_block2):
    def __init__(self, mode="b205", samp_rate=40e6, sym_rate=20e6, rolloff=0.2,
                 centre_freq=1195e6, tx_gain=0, device_args="", lo_offset=-1,
                 output_file="", output_format="auto", duration=0.0, throttle=False,
                 start_unix=None, meta=None):
        gr.hier_block2.__init__(self, "g5_tx_sink", gr.io_signature(1, 1, gr.sizeof_gr_complex),
                                gr.io_signature(0, 0, 0))
        self.mode, self.fs = mode, float(samp_rate)
        output_file = C.path(output_file)
        self.usrp = None
        if mode == "b205":
            from gnuradio import uhd
            dev = C.device_args(device_args, self.fs, "tx")
            self.usrp = uhd.usrp_sink(dev, uhd.stream_args(cpu_format="fc32", channels=[0]), "")
            u = self.usrp
            u.set_samp_rate(self.fs)
            if lo_offset is None or lo_offset < 0:
                lo_offset = sym_rate * (1 + rolloff) / 2 + 1e5
                if lo_offset + sym_rate * (1 + rolloff) / 2 > 0.4 * u.get_clock_rate(0):
                    lo_offset = 0.0
            u.set_center_freq(uhd.tune_request(centre_freq, lo_offset), 0)
            u.set_antenna("TX/RX", 0)
            u.set_gain(tx_gain, 0)
            u.set_bandwidth(C.default_bw(self.fs), 0)
            self.text = (f"B205mini {centre_freq / 1e6:g} MHz, {u.get_samp_rate() / 1e6:g} MS/s, "
                         f"MCR {u.get_clock_rate(0) / 1e6:g} MHz, {dev}")
            self.gain = tx_gain
            self.connect(self, u)
        elif mode == "file":
            if not output_file:
                raise ValueError("file output needs an output file")
            os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)
            fmt = output_format
            if fmt in ("", "auto"):
                fmt = "sc16" if output_file.lower().endswith((".sc16", ".cs16")) else "fc32"
            chain = [self]
            if throttle:
                chain.append(blocks.throttle(gr.sizeof_gr_complex, self.fs, True))
            if duration:
                chain.append(blocks.head(gr.sizeof_gr_complex, int(duration * self.fs)))
            if fmt == "sc16":
                chain.append(blocks.complex_to_interleaved_short(True, 32767.0))
                self.fsink = blocks.file_sink(gr.sizeof_short * 2, output_file, False)
            else:
                self.fsink = blocks.file_sink(gr.sizeof_gr_complex, output_file, False)
            self.fsink.set_unbuffered(False)
            chain.append(self.fsink)
            self.connect(*chain)
            self.output_file, self.fmt = output_file, fmt
            t0 = start_unix if start_unix is not None else time.time()
            with open(output_file + ".start", "w") as f:
                f.write(f"{C.iso_utc(t0)}\n# first sample of {os.path.basename(output_file)} "
                        f"(tx_unified, {'Doppler-sim start' if start_unix else 'write time'})\n")
            with open(output_file + ".json", "w") as f:
                json.dump(dict(meta or {}, start_unix=t0, samp_rate=self.fs, format=fmt), f,
                          indent=1)
            self.text = (f"{output_file} ({fmt}" + (f", {duration:g} s" if duration else "")
                         + (", real time" if throttle else "") + ")")
        else:
            raise ValueError(f"output {mode!r}: b205 or file")
        txlog(f"output: {self.text}")

    def set_tx_gain(self, g):
        if self.usrp is not None:
            self.usrp.set_gain(g, 0)
            self.gain = g
            txlog(f"TX gain -> {g:g} dB")

    def describe(self):
        if self.mode == "file" and os.path.exists(self.output_file):
            n = os.path.getsize(self.output_file) // (4 if self.fmt == "sc16" else 8)
            return f"{self.text}: {n / self.fs:.1f} s written"
        return self.text + (f", gain {self.gain:g} dB" if self.usrp is not None else "")
