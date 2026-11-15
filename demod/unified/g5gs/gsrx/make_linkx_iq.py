#!/usr/bin/env python3
"""make_linkx_iq - turn a BBFRAME file into a Link X-like DVB-S2 IQ capture.

Used for test plan Step 3 / Test 3 (synthetic signal) and to check the whole laptop chain
before the radio is powered:

    tx_simulator.py        makes the BBFRAMEs
    make_linkx_iq.py       BBFRAMEs to .sc16
    newrx_v2.py --input-file X.sc16 --bbframe-file X.bbframes
    gs_rx.py --from-file X.bbframes --verify-hashes ...

Waveform per RF ICD 1045318 rev 1.0 sec. 3.1: roll-off 0.20, pilots on, PL scrambling Gold
code 0, DVB-S2 dummy PLFRAMEs while idle. Output is what `rx_samples_to_file --type short`
writes: interleaved little-endian int16 I/Q (sc16), or complex64 with --format fc32.

Data PLFRAMEs come from gr-dtv. gr-dtv cannot make dummy PLFRAMEs, so they are built
here: PLHEADER with PLS 0, then 36 slots of unmodulated (1+j)/sqrt(2) symbols,
PL-scrambled, no pilots (EN 302 307-1 sec. 5.5.1). The PLHEADER and scrambler code are
checked against gr-dtv's own frames at start-up.

  python3 make_linkx_iq.py --bbframes sim.bbframes --modcod 1 --out sim.sc16 \\
          --lead-dummies 200 --trail-dummies 200 --gap-dummies 2 --snr 10 --cfo 3000
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
from gnuradio import blocks, dtv, filter as grfilter, gr
from gnuradio.filter import firdes

import bbframe

SQ2 = np.sqrt(0.5)
SOF = 0x18D2E82                       # 26 bits
PLS_SCRAMBLE = 0x719D83C953422DFA     # 64 bits
RM_ROWS = (0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF, 0x0000FFFF, 0xFFFFFFFF)

RATES = {"1/4": dtv.C1_4, "1/3": dtv.C1_3, "2/5": dtv.C2_5, "1/2": dtv.C1_2,
         "3/5": dtv.C3_5, "2/3": dtv.C2_3, "3/4": dtv.C3_4, "4/5": dtv.C4_5,
         "5/6": dtv.C5_6, "8/9": dtv.C8_9, "9/10": dtv.C9_10}
MODS = {"QPSK": dtv.MOD_QPSK, "8PSK": dtv.MOD_8PSK}


def _bits(value: int, n: int) -> list[int]:
    return [(value >> (n - 1 - i)) & 1 for i in range(n)]


def plheader(pls: int) -> np.ndarray:
    """90 pi/2-BPSK PLHEADER symbols (SOF + PLS code) for a 7-bit PLS value."""
    b6, b7 = pls >> 1, pls & 1
    cw = 0
    for i, row in enumerate(RM_ROWS):
        if (b6 >> (5 - i)) & 1:
            cw ^= row
    code = []
    for bit in _bits(cw, 32):
        code += [bit, bit ^ b7]
    code = [c ^ s for c, s in zip(code, _bits(PLS_SCRAMBLE, 64))]
    bits = np.array(_bits(SOF, 26) + code)
    k = np.arange(90)
    rot = np.where(k % 2 == 0, 1 + 1j, -1 + 1j) * SQ2
    return (1 - 2 * bits) * rot


def pl_scrambler(n: int, gold: int = 0) -> np.ndarray:
    """R_n(i) in {0,1,2,3} for i < n (EN 302 307-1 sec. 5.5.4)."""
    period = (1 << 18) - 1
    x = np.zeros(period + 18, np.uint8)
    y = np.ones(period + 18, np.uint8)
    x[0] = 1
    for i in range(period):
        x[i + 18] = x[i + 7] ^ x[i]
        y[i + 18] = y[i + 10] ^ y[i + 7] ^ y[i + 5] ^ y[i]
    x = np.roll(x[:period], -gold)
    y = y[:period]
    z = x ^ y
    i = np.arange(n)
    return 2 * z[(i + 131072) % period] + z[i]


def dummy_plframe(scr: np.ndarray) -> np.ndarray:
    payload = np.full(3240, (1 + 1j) * SQ2) * (1j ** scr[:3240])
    return np.concatenate([plheader(0), payload])


def data_plframes(frames: bytes, flen: int, rate: str, mod: str, short: bool):
    """Run gr-dtv on concatenated BBFRAMEs; return (symbols, PLFRAME length)."""
    fs = dtv.FECFRAME_SHORT if short else dtv.FECFRAME_NORMAL
    tb = gr.top_block()
    src = blocks.vector_source_b(list(frames), False)
    unpack = blocks.packed_to_unpacked_bb(1, gr.GR_MSB_FIRST)
    chain = [dtv.dvb_bbscrambler_bb(dtv.STANDARD_DVBS2, fs, RATES[rate]),
             dtv.dvb_bch_bb(dtv.STANDARD_DVBS2, fs, RATES[rate]),
             dtv.dvb_ldpc_bb(dtv.STANDARD_DVBS2, fs, RATES[rate], dtv.MOD_OTHER),
             dtv.dvbs2_interleaver_bb(fs, RATES[rate], MODS[mod]),
             dtv.dvbs2_modulator_bc(fs, RATES[rate], MODS[mod], dtv.INTERPOLATION_OFF),
             dtv.dvbs2_physical_cc(fs, RATES[rate], MODS[mod], dtv.PILOTS_ON, 0)]
    sink = blocks.vector_sink_c()
    tb.connect(src, unpack, *chain, sink)
    tb.run()
    # dvbs2_physical_cc emits 2 samples/symbol, zero-stuffed: keep the symbols
    sym = np.array(sink.data(), np.complex64)[::2]
    n = len(frames) // flen
    if n == 0 or len(sym) % n:
        raise SystemExit(f"gr-dtv produced {len(sym)} symbols for {n} BBFRAMEs")
    return sym, len(sym) // n


def self_check(sym: np.ndarray, plen: int, pls: int, scr: np.ndarray):
    """Our PLHEADER and scrambler must reproduce gr-dtv's PLHEADER and first
    pilot block, otherwise the dummy frames would be wrong."""
    if not np.allclose(sym[:90], plheader(pls), atol=1e-4):
        raise SystemExit("self-check failed: PLHEADER differs from gr-dtv")
    if plen > 90 + 1440 + 36:
        pil = sym[90 + 1440:90 + 1476]
        want = (1 + 1j) * SQ2 * (1j ** scr[1440:1476])
        if not np.allclose(pil, want, atol=1e-4):
            raise SystemExit("self-check failed: PL scrambler differs from gr-dtv pilots")


def rrc_upsample(sym: np.ndarray, sps: int, rolloff: float) -> np.ndarray:
    taps = firdes.root_raised_cosine(sps, sps, 1.0, rolloff, 22 * sps + 1)
    tb = gr.top_block()
    src = blocks.vector_source_c(sym.tolist(), False)
    fir = grfilter.interp_fir_filter_ccf(sps, taps)
    sink = blocks.vector_sink_c()
    tb.connect(src, fir, sink)
    tb.run()
    return np.array(sink.data(), np.complex64)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bbframes", required=True, help="BBFRAME file (tx_simulator --out-file)")
    p.add_argument("--modcod", default="1",
                   help="Link X MODCOD number 1-17 or e.g. QPSK1/4 (QPSK/8PSK only) [1]")
    p.add_argument("--framesize", default="normal", choices=["normal", "short"])
    p.add_argument("--sps", type=int, default=4, help="samples per symbol [4 = 8 MS/s at 2 MBd]")
    p.add_argument("--rolloff", type=float, default=0.2)
    p.add_argument("--lead-dummies", type=int, default=200,
                   help="dummy PLFRAMEs before the data (receiver lock-up time)")
    p.add_argument("--trail-dummies", type=int, default=200)
    p.add_argument("--gap-dummies", type=int, default=0,
                   help="dummy PLFRAMEs inserted after every data PLFRAME")
    p.add_argument("--snr", type=float, default=None, help="Es/N0 in dB (default: no noise)")
    p.add_argument("--cfo", type=float, default=0.0, help="carrier frequency offset in Hz")
    p.add_argument("--sym-rate", type=float, default=2e6,
                   help="symbol rate in Bd, used for --cfo and the duration printout [2e6]")
    p.add_argument("--level-dbfs", type=float, default=-18.0,
                   help="RMS level in dBFS; -18 keeps peaks below the plan's 16384 limit [-18]")
    p.add_argument("--format", choices=["sc16", "fc32"], default="sc16")
    p.add_argument("--out", required=True)
    p.add_argument("--seed", type=int, default=1)
    a = p.parse_args(argv)

    s = a.modcod.strip().upper()
    if s.isdigit():
        modcod_num = int(s)
    else:
        modcod_num = next((n for n, (m, r) in bbframe.MODCODS.items() if m + r == s), 0)
    if not 1 <= modcod_num <= 17:
        sys.exit(f"--modcod {a.modcod}: only MODCOD 1-17 (QPSK, 8PSK); "
                 "gr-dvbs2rx cannot decode APSK")
    mod, rate = bbframe.MODCODS[modcod_num]
    short = a.framesize == "short"
    flen = bbframe.bbframe_len(a.framesize, rate)
    data = open(a.bbframes, "rb").read()
    data = data[:len(data) // flen * flen]
    if not data:
        sys.exit(f"{a.bbframes}: no whole {flen}-byte BBFRAMEs")
    pls = (modcod_num << 2) | (short << 1) | 1

    sym, plen = data_plframes(data, flen, rate, mod, short)
    scr = pl_scrambler(max(plen, 3240))
    self_check(sym, plen, pls, scr)
    dummy = dummy_plframe(scr)

    parts = [dummy] * a.lead_dummies
    for k in range(len(sym) // plen):
        parts.append(sym[k * plen:(k + 1) * plen])
        parts += [dummy] * a.gap_dummies
    parts += [dummy] * a.trail_dummies
    parts.append(plheader(0))          # receiver needs the next SOF to close the last frame
    stream = np.concatenate(parts).astype(np.complex64)
    n_dummy = a.lead_dummies + a.trail_dummies + a.gap_dummies * (len(sym) // plen)

    iq = rrc_upsample(stream, a.sps, a.rolloff)
    rng = np.random.default_rng(a.seed)
    if a.cfo:
        f = a.cfo / (a.sym_rate * a.sps)
        iq *= np.exp(2j * np.pi * f * np.arange(len(iq))).astype(np.complex64)
    if a.snr is not None:
        # Es/N0: after the receiver's matched filter the noise bandwidth is the
        # symbol rate, i.e. 1/sps of the sample rate, so scale the per-sample noise up.
        n0 = 10 ** (-a.snr / 10)
        sig_pow = np.mean(np.abs(iq) ** 2)
        noise_std = np.sqrt(sig_pow * a.sps * n0 / 2)
        iq += (rng.normal(0, noise_std, len(iq)) + 1j * rng.normal(0, noise_std, len(iq))
               ).astype(np.complex64)
    rms = np.sqrt(np.mean(np.abs(iq) ** 2))
    iq *= 10 ** (a.level_dbfs / 20) / rms

    if a.format == "sc16":
        out = np.empty(2 * len(iq), np.int16)
        scaled = np.clip(np.round(np.stack([iq.real, iq.imag], 1).ravel() * 32767),
                         -32768, 32767)
        out[:] = scaled
        out.astype("<i2").tofile(a.out)
        peak = int(np.abs(out).max())
    else:
        iq.astype(np.complex64).tofile(a.out)
        peak = float(np.abs(np.stack([iq.real, iq.imag])).max())

    print(f"{len(data) // flen} data PLFRAMEs (MODCOD {modcod_num}, PLS {pls}, "
          f"{plen} symbols each) + {n_dummy} dummy PLFRAMEs -> {len(iq)} samples "
          f"({len(iq) / (a.sym_rate * a.sps):.2f} s at {a.sym_rate * a.sps / 1e6:g} MS/s), "
          f"{a.format}, peak {peak}, wrote {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
