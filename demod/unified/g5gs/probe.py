"""Read the gr-dvbs2rx counters (lock, CFO, SNR, frames, BCH, LDPC) and the g5gs block stats.

Used by the status panel (twice a second) and by the CLI / end-of-run summary. The blocks
are found on the top block by their GRC ids, so the GRC graph keeps the standard dvbs2rx
blocks visible and editable.
"""
from __future__ import annotations

import threading

from . import config as C

# GRC ids used in rx_unified.grc (and by rx_cli.py)
IDS = {"src": "rx_source_0", "gate": "duty_gate_0", "precomp": "doppler_precomp_0",
       "agc": "front_agc_0", "plsync": "dvbs2rx_plsync_cc_0",
       "demap": "dvbs2rx_xfecframe_demapper_cb_0", "ldpc": "dvbs2rx_ldpc_decoder_bb_0",
       "bch": "dvbs2rx_bch_decoder_bb_0", "dec": "gs_decoder_0"}


def resolve(top, ids=None):
    ids = dict(IDS, **(ids or {}))
    return {k: getattr(top, v, None) for k, v in ids.items()}


def link_stats(b, sym_rate, dummy_frames=True):
    """Dict of the receiver counters; b = resolve(top)."""
    p, bch = b.get("plsync"), b.get("bch")
    s = {}
    if p is not None:
        s.update(locked=bool(p.get_locked()), cfo_hz=p.get_freq_offset() * sym_rate,
                 sof=p.get_sof_count(), plframes=p.get_frame_count(),
                 rejected=p.get_rejected_count(), dummy=p.get_dummy_count())
        if dummy_frames:
            # CCM refuses dummy PLS in the filter, so gr-dvbs2rx counts dummies as rejected
            s["dummy"] += s.pop("rejected")
            s["rejected"] = 0
    if bch is not None:
        s.update(bch_frames=bch.get_frame_count(), bch_errors=bch.get_error_count())
        ldpc = b.get("ldpc")
        if ldpc is not None and s["bch_frames"]:
            # get_average_trials() divides by the frame count: only after a frame
            s["ldpc_iter"] = ldpc.get_average_trials()
    d = b.get("demap")
    if d is not None and s.get("plframes"):
        try:
            s["snr_db"] = float(d.get_snr())
        except Exception:                       # noqa: BLE001
            pass
    return s


def summary_lines(top, sym_rate, dummy_frames=True):
    b = resolve(top)
    s = link_stats(b, sym_rate, dummy_frames)
    out = []
    if "locked" in s:
        out.append(f"PL sync: locked={s['locked']}, carrier offset {s['cfo_hz'] / 1e3:+.3f} kHz"
                   + (f", SNR {s['snr_db']:.1f} dB" if "snr_db" in s else ""))
        out.append(f"PLFRAMEs: {s['sof']} SOFs, {s['plframes']} data frames, {s['dummy']} dummy"
                   + (f", {s['rejected']} rejected by the PLS filter" if s["rejected"] else ""))
    if "bch_frames" in s:
        out.append(f"BCH: {s['bch_frames']} frames, {s['bch_errors']} with uncorrectable errors"
                   + (f"; LDPC average iterations {s['ldpc_iter']:.1f}" if "ldpc_iter" in s
                      else ""))
    src, gate = b.get("src"), b.get("gate")
    if src is not None:
        st = src.stats()
        if "overflows" in st:
            recs = st["files"] + ([st["path"]] if st.get("recording") else [])
            out.append(f"B205mini: {st['overflows']} overflow gaps, {st['lost']} samples lost; "
                       f"recordings: {', '.join(recs) or 'none'}")
        if st.get("progress") is not None:
            out.append(f"input file: {100 * st['progress']:.1f} % read")
    if gate is not None and gate.sh is not None:
        g = gate.stats()
        out.append(f"duty gate ({g['mode']}): {g['windows']} windows, {g['passed']} samples "
                   f"demodulated, {g['dropped']} dropped (demod behind)")
    return out


_printed = threading.Lock()


def print_summary(top, sym_rate, dummy_frames=True, log=C.log_event):
    """Print the link summary once per top block (CLI and GUI both call this at the end)."""
    with _printed:
        if getattr(top, "_g5_summary_done", False):
            return
        top._g5_summary_done = True
    for line in summary_lines(top, sym_rate, dummy_frames):
        log(line)


def status_line(top, sym_rate, dummy_frames=True):
    """One console line for the CLI: input, recording, gate, lock, frames, GSSE, files."""
    b = resolve(top)
    s = link_stats(b, sym_rate, dummy_frames)
    parts = []
    src = b.get("src")
    if src is not None:
        st = src.stats()
        if st.get("progress") is not None:
            parts.append(f"file {100 * st['progress']:5.1f}%")
        if "recording" in st:
            parts.append((f"REC {st['seconds']:.0f}s {st['bytes'] / 1e9:.1f}GB" if st["recording"]
                          else "rec off") + f", {st['free'] / 2**30:.0f}GB free"
                         + (f", lvl {st['level_dbfs']:.1f}dBFS pk {st['peak_dbfs']:.1f}"
                            if st.get("level_dbfs") is not None else "")
                         + (f", O {st['overflows']}" if st.get("overflows") else "")
                         + (f" !! {st['warning']}" if st.get("warning") else ""))
    gate = b.get("gate")
    if gate is not None and gate.sh is not None:
        g = gate.stats()
        parts.append(f"{g['mode']} {g['windows']} win" + (f" {g['dropped']} dropped"
                                                         if g["dropped"] else ""))
    if "locked" in s:
        parts.append(("LOCK" if s["locked"] else "search") + f" {s['cfo_hz'] / 1e3:+.2f}kHz"
                     + (f" {s['snr_db']:.1f}dB" if "snr_db" in s else ""))
        parts.append(f"fr {s['plframes']}/{s['dummy']}d")
    if "bch_frames" in s:
        parts.append(f"bch {s['bch_frames']}/{s['bch_errors']}err")
    dec = b.get("dec")
    if dec is not None:
        d = dec.stats()
        parts.append(f"gsse {d['gsse_ok']}/{d['gsse_crc_bad']}bad files {len(d['files'])}")
    pc = b.get("precomp")
    if pc is not None and pc.active and pc.now_unix():
        g = pc.predicted()
        parts.append(f"el {g['el_deg']:.1f} dop {g['doppler_hz'] / 1e3:+.1f}k")
    return " | ".join(parts)
