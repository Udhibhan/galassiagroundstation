#!/usr/bin/env python3
"""TLE-driven Doppler pre-compensation for the Ku downlink (13.95 GHz, CRISP ground station).

The satellite's orbit is propagated with SGP4 from a TLE (or OMM), giving the range R(t) from
the station; the downlink then arrives with carrier phase -fc R(t) / c (cycles), i.e. Doppler
-fc R'(t) / c. A fixed-LO downconverter passes that Doppler unchanged to the 1.195 GHz IF, so
the receiver undoes it by multiplying the baseband by exp(+j 2 pi fc (R(t) - R(t0)) / c),
sample by sample, before the AGC. What is left for the PL sync is only the prediction error
(TLE age, clock error) plus any fixed LO error. Phase-continuous: the correction is the
predicted carrier phase itself, not a stepped frequency.

  - SGP4 (`pip install --user sgp4`, or `sudo apt install python3-sgp4`), TEME -> ECEF by GMST
    (IAU-82; UT1 = UTC and no polar motion: < 1 Hz at Ku), WGS84 station, light time (the
    satellite position at emission time t - R/c)
  - R(t) is computed on a 1 ms grid in 30 s segments as needed (works for live streams and
    files of any length) and interpolated per sample: phase error < 1e-3 cycle
  - only the carrier Doppler is removed; the symbol-clock Doppler (<= 24 ppm) is left to
    symbol_sync, which handles it in every test (doppler_test/RESULTS.md)

TLE input: a text file with one or more element sets, 2-line or 3-line (name line + lines 1
and 2), as CelesTrak / Space-Track serve them; pick one with --sat (name substring or NORAD
catalogue number) when the file holds several. OMM in CelesTrak JSON (FORMAT=json) or CSV
(FORMAT=csv) also works (.json / .csv extension).

The time of the first sample is needed (see start_time()): --start-utc, else a sidecar
(<file>.start with an ISO time, or the .json from rx_capture.py / make_doppler_iq.py), else
the file's modification time minus its duration.

  python3 doppler_precomp.py passes --tle sat.tle --from 2026-10-25T00:00Z --hours 24
  python3 doppler_precomp.py profile --tle sat.tle --start-utc 2026-10-25T03:12:00Z \\
      --duration 700 --out pass.csv
  python3 doppler_precomp.py apply in.sc16 out.sc16 --tle sat.tle --samp-rate 40e6 \\
      [--start-utc ...]           # writes a pre-compensated copy (out may be a FIFO)
  python3 doppler_precomp.py stamp cap.sc16 --samp-rate 40e6   # right after recording
  newrx_v2.py --input-file cap.sc16 --tle sat.tle [--start-utc ...] ...   # built in
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime, timezone

import numpy as np

C_LIGHT = 299_792_458.0
RE = 6_378_137.0
F_WGS84 = 1 / 298.257223563
W_EARTH = 7.292115146706979e-5          # rad/s
CRISP = (1.2977, 103.7764, 60.0)        # lat deg, lon deg, height m (approximate)
FC_KU = 13.95e9
GRID_S = 1e-3                           # range grid spacing
SEG_S = 30.0                            # grid computed in segments of this length


# ---------------------------------------------------------------------------- time
def parse_utc(s) -> float:
    """ISO 8601 UTC ('2026-10-25T03:12:00.5Z', no zone = UTC) or unix seconds -> unix s."""
    if isinstance(s, (int, float)):
        return float(s)
    s = str(s).strip()
    try:
        return float(s)
    except ValueError:
        pass
    d = datetime.fromisoformat(s.replace("Z", "+00:00"))
    if d.tzinfo is None:
        d = d.replace(tzinfo=timezone.utc)
    return d.timestamp()


def fmt_utc(t: float) -> str:
    return datetime.fromtimestamp(t, timezone.utc).isoformat(timespec="milliseconds")[:-6] + "Z"


def _jd(t_unix):
    """unix s -> (jd whole, fraction) as sgp4_array wants (keeps full precision)."""
    t = np.asarray(t_unix, float)
    day = np.floor(t / 86400.0)
    return 2440587.5 + day, (t - day * 86400.0) / 86400.0


def gmst(jd, fr):
    """Greenwich mean sidereal time, IAU-82 (Vallado gstime), radians."""
    tut1 = ((jd - 2451545.0) + fr) / 36525.0
    sec = (-6.2e-6 * tut1 ** 3 + 0.093104 * tut1 ** 2
           + (876600.0 * 3600 + 8640184.812866) * tut1 + 67310.54841)
    return np.mod(np.radians(sec / 240.0), 2 * np.pi)


# ---------------------------------------------------------------------------- TLE / OMM
def _tle_checksum(line):
    return sum(int(c) if c.isdigit() else (c == "-") for c in line[:68]) % 10


def read_elements(path: str) -> list[tuple[str, object]]:
    """All element sets in a TLE / OMM file -> [(name, Satrec)]."""
    try:
        from sgp4.api import Satrec
        from sgp4 import omm
    except ImportError:
        raise SystemExit("needs the sgp4 package: pip install --user sgp4 "
                         "(or sudo apt install python3-sgp4)")
    ext = os.path.splitext(path)[1].lower()
    out = []
    if ext in (".json", ".csv", ".xml"):
        if ext == ".json":
            recs = json.load(open(path))
            recs = recs if isinstance(recs, list) else [recs]
        elif ext == ".csv":
            recs = list(omm.parse_csv(open(path)))
        else:
            recs = list(omm.parse_xml(path))
        for r in recs:
            s = Satrec()
            omm.initialize(s, r)
            out.append((r.get("OBJECT_NAME", str(r.get("NORAD_CAT_ID"))), s))
        return out
    lines = [ln.rstrip() for ln in open(path) if ln.strip()]
    i = 0
    while i < len(lines):
        if lines[i].startswith("1 ") and i + 1 < len(lines) and lines[i + 1].startswith("2 "):
            name = lines[i - 1].strip() if i and not lines[i - 1][:2] in ("1 ", "2 ") else ""
            l1, l2 = lines[i], lines[i + 1]
            for ln in (l1, l2):
                if len(ln) >= 69 and ln[68].isdigit() and int(ln[68]) != _tle_checksum(ln):
                    raise SystemExit(f"{path}: TLE checksum error in\n  {ln}")
            name = name[2:].strip() if name.startswith("0 ") else name
            out.append((name or l1[2:7].strip(), Satrec.twoline2rv(l1, l2)))
            i += 2
        else:
            i += 1
    if not out:
        raise SystemExit(f"{path}: no TLE / OMM element sets found")
    return out


def load_satrec(path: str, sat: str | None = None):
    """One element set from the file: --sat picks by name substring or NORAD number."""
    sets = read_elements(path)
    if sat:
        pick = [(n, s) for n, s in sets
                if sat.lower() in n.lower() or (sat.isdigit() and int(sat) == s.satnum)]
        if not pick:
            raise SystemExit(f"{path}: no element set matches {sat!r} "
                             f"(have {[n for n, _ in sets]})")
        sets = pick
    if len(sets) > 1:
        raise SystemExit(f"{path} holds {len(sets)} element sets "
                         f"({[n for n, _ in sets][:8]}...): choose one with --sat")
    return sets[0]


def tle_epoch_unix(s) -> float:
    return (s.jdsatepoch - 2440587.5 + s.jdsatepochF) * 86400.0


# ---------------------------------------------------------------------------- geometry
def station_ecef(lat_deg, lon_deg, h_m):
    lat, lon = np.radians(lat_deg), np.radians(lon_deg)
    e2 = F_WGS84 * (2 - F_WGS84)
    n = RE / np.sqrt(1 - e2 * np.sin(lat) ** 2)
    pos = np.array([(n + h_m) * np.cos(lat) * np.cos(lon), (n + h_m) * np.cos(lat) * np.sin(lon),
                    (n * (1 - e2) + h_m) * np.sin(lat)])
    up = np.array([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)])
    east = np.array([-np.sin(lon), np.cos(lon), 0.0])
    return pos, up, east, np.cross(up, east)


class DopplerModel:
    """Range, range rate, elevation and Doppler of one satellite seen from one station."""

    def __init__(self, satrec, station=CRISP, fc=FC_KU, name=""):
        self.sat, self.station, self.fc, self.name = satrec, tuple(station), fc, name
        self.gs, self.up, self.east, self.north = station_ecef(*station)

    def sat_ecef(self, t_unix):
        """Satellite position / velocity in ECEF (m, m/s) at unix times (array)."""
        jd, fr = _jd(t_unix)
        e, r, v = self.sat.sgp4_array(np.atleast_1d(jd), np.atleast_1d(fr))
        if np.any(e):
            raise RuntimeError(f"SGP4 error code {int(e[e != 0][0])} (decayed / bad TLE?)")
        r, v = r * 1e3, v * 1e3                       # TEME, m and m/s
        th = gmst(np.atleast_1d(jd), np.atleast_1d(fr))
        c, s = np.cos(th), np.sin(th)
        x, y = c * r[:, 0] + s * r[:, 1], -s * r[:, 0] + c * r[:, 1]
        vx = c * v[:, 0] + s * v[:, 1] + W_EARTH * y
        vy = -s * v[:, 0] + c * v[:, 1] - W_EARTH * x
        return np.stack([x, y, r[:, 2]], -1), np.stack([vx, vy, v[:, 2]], -1)

    def geometry(self, t_unix):
        """At receive times t: range (with light time), range rate, el, az, Doppler at RF."""
        t = np.atleast_1d(np.asarray(t_unix, float))
        p, _ = self.sat_ecef(t)
        tau = np.linalg.norm(p - self.gs, axis=-1) / C_LIGHT
        p, v = self.sat_ecef(t - tau)                 # position when the signal left
        d = p - self.gs
        rng = np.linalg.norm(d, axis=-1)
        rr = np.sum(d * v, -1) / rng
        return {"range_m": rng, "range_rate_mps": rr,
                "el_deg": np.degrees(np.arcsin(d @ self.up / rng)),
                "az_deg": np.degrees(np.arctan2(d @ self.east, d @ self.north)) % 360,
                "doppler_hz": -self.fc * rr / C_LIGHT}

    def passes(self, t0, t1, min_el=5.0):
        """Passes above min_el between unix t0 and t1 (10 s scan, refined to 0.1 s)."""
        ts = np.arange(t0, t1, 10.0)
        el = self.geometry(ts)["el_deg"]
        up = el >= min_el
        out = []
        edges = np.flatnonzero(np.diff(up.astype(int)))
        starts = [0] if up[0] else []
        starts += [i + 1 for i in edges if not up[i]]
        for s in starts:
            e = s
            while e + 1 < len(up) and up[e + 1]:
                e += 1
            fine = np.arange(ts[max(s - 1, 0)], ts[min(e + 1, len(ts) - 1)] + 10, 0.1)
            g = self.geometry(fine)
            vis = g["el_deg"] >= min_el
            if not vis.any():
                continue
            ft, fd = fine[vis], g["doppler_hz"][vis]
            rate = np.gradient(fd, ft)
            i = int(np.argmax(g["el_deg"]))
            out.append({"aos_utc": fmt_utc(ft[0]), "tca_utc": fmt_utc(fine[i]),
                        "los_utc": fmt_utc(ft[-1]), "aos_unix": float(ft[0]),
                        "tca_unix": float(fine[i]), "los_unix": float(ft[-1]),
                        "duration_s": round(float(ft[-1] - ft[0]), 1),
                        "max_el_deg": round(float(g["el_deg"][i]), 2),
                        "doppler_aos_hz": round(float(fd[0])),
                        "doppler_los_hz": round(float(fd[-1])),
                        "max_abs_rate_hz_s": round(float(np.abs(rate).max()), 1)})
        return out


class Precomp:
    """Correction phase (cycles) for sample n of a stream whose sample 0 is at start_unix.

    cycles(n) = sign * fc * (R(t) - R(t_start)) / c - offset_hz * t,  t = n / fs + dt
    and the corrected signal is x[n] * exp(j 2 pi cycles(n)).
    dt shifts the model in time (a known clock error, or a known along-track TLE error);
    offset_hz removes a fixed frequency error as well (e.g. a measured LO offset).
    """

    def __init__(self, model, start_unix, samp_rate, sign=1.0, offset_hz=0.0, dt=0.0):
        """start_unix None: a live stream; sample 0 is taken to be 'now' at the first call
        of rotate() (host clock: keep it NTP-synced; 1 s error ~ 4.6 kHz at worst)."""
        self.m, self.fs, self.dt = model, float(samp_rate), float(dt)
        self.sign, self.off = float(sign), float(offset_hz)
        self.t0 = None
        if start_unix is not None:
            self.set_start(start_unix)

    def set_start(self, start_unix):
        self.t0 = float(start_unix) + self.dt
        self.r0 = float(self.m.geometry([self.t0])["range_m"][0])
        self.seg = {}

    def _segment(self, k):
        if k not in self.seg:
            if len(self.seg) > 4:                     # keep a few recent segments
                self.seg.pop(min(self.seg))
            t = k * SEG_S + np.arange(int(SEG_S / GRID_S) + 2) * GRID_S
            r = self.m.geometry(self.t0 + t)["range_m"]
            self.seg[k] = (t, self.m.fc * (r - self.r0) / C_LIGHT)
        return self.seg[k]

    def cycles(self, n0, n):
        """Correction phase for samples n0 .. n0 + n - 1, cycles in [0, 1)."""
        t = (n0 + np.arange(n, dtype=np.float64)) / self.fs
        k0, k1 = int(np.floor(t[0] / SEG_S)), int(np.floor(t[-1] / SEG_S))
        if k0 == k1:
            gt, gc = self._segment(k0)
        else:
            parts = [self._segment(k) for k in range(k0, k1 + 1)]
            gt = np.concatenate([p[0] for p in parts])
            gc = np.concatenate([p[1] for p in parts])
        c = self.sign * np.interp(t, gt, gc) - self.off * t
        return np.mod(c, 1.0)

    def rotate(self, x, n0):
        if self.t0 is None:
            self.set_start(time.time() - (n0 + len(x)) / self.fs)
            g = self.m.geometry([self.t0])
            print(f"Doppler pre-comp: live start {fmt_utc(self.t0 - self.dt)}, el "
                  f"{g['el_deg'][0]:.1f} deg, predicted {g['doppler_hz'][0] / 1e3:+.2f} kHz",
                  flush=True)
        ph = (2 * np.pi * self.cycles(n0, len(x))).astype(np.float32)
        return x * (np.cos(ph) + 1j * np.sin(ph)).astype(np.complex64)

    def doppler_at(self, n):
        """Predicted Doppler (Hz, before sign) at sample n, for logging."""
        return float(self.m.geometry([self.t0 + n / self.fs])["doppler_hz"][0])


# ---------------------------------------------------------------------------- start time
def start_time(path: str | None, override=None, n_samples=None, samp_rate=None,
               log=print) -> tuple[float, str]:
    """Unix time of the first sample of a recording, and where it came from.

    Order: override (--start-utc) > <path>.start (ISO time on the first line) >
    <path>.json: 'start_unix' (make_doppler_iq.py) or the rx_capture.py run.json it points to
    ('first_sample_unix', else 'stream_start_unix') > file mtime - duration (the file is
    closed right after its last sample; rx_samples_to_file writes no time stamp).
    """
    if override is not None:
        return parse_utc(override), "--start-utc"
    if path:
        if os.path.exists(path + ".start"):
            return parse_utc(open(path + ".start").readline()), path + ".start"
        if os.path.exists(path + ".json"):
            j = json.load(open(path + ".json"))
            if j.get("start_unix") is not None:
                return float(j["start_unix"]), path + ".json"
            run = j.get("capture_run")
            if run and os.path.exists(run):
                r = json.load(open(run))
                for k in ("first_sample_unix", "stream_start_unix"):
                    if r.get(k) is not None:
                        return float(r[k]), f"{run} ({k})"
        if n_samples and samp_rate:
            t = os.path.getmtime(path) - n_samples / samp_rate
            log(f"Doppler pre-comp: no start time for {path}; using its modification time - "
                f"duration = {fmt_utc(t)}. Give --start-utc if the file was copied or touched.")
            return t, "mtime - duration"
    raise SystemExit("Doppler pre-comp needs the time of the first sample: --start-utc")


def make_block(precomp: Precomp, name="doppler_precomp"):
    """GNU Radio sync block (complex in / out) applying precomp sample by sample."""
    from gnuradio import gr

    class doppler_precomp_cc(gr.sync_block):
        def __init__(self):
            gr.sync_block.__init__(self, name, in_sig=[np.complex64], out_sig=[np.complex64])
            self.pc = precomp

        def work(self, input_items, output_items):
            x = input_items[0]
            n = len(x)
            output_items[0][:] = self.pc.rotate(x, self.nitems_written(0))
            return n
    return doppler_precomp_cc()


def from_args(o, samp_rate, path=None, n_samples=None, log=print):
    """Precomp from the common CLI options (add_args); None when --tle is not given.
    path None = live stream: --start-utc if given, else the moment samples start arriving."""
    if not getattr(o, "tle", None):
        return None
    name, s = load_satrec(o.tle, o.sat)
    model = DopplerModel(s, parse_station(o.station), o.rf_freq, name)
    if path is None and o.start_utc is None:
        t0, src = None, "live"
    else:
        t0, src = start_time(path, o.start_utc, n_samples, samp_rate, log)
    pc = Precomp(model, t0, samp_rate, o.precomp_sign, o.precomp_offset_hz, o.precomp_dt)
    tr = t0 if t0 is not None else time.time()
    g = model.geometry([tr + o.precomp_dt])
    age = (tr - tle_epoch_unix(s)) / 86400
    log(f"Doppler pre-comp: {name} (NORAD {s.satnum}), TLE epoch {fmt_utc(tle_epoch_unix(s))} "
        f"({age:+.2f} days), start "
        + (f"{fmt_utc(t0)} from {src}" if t0 is not None else "when streaming starts")
        + f", el {g['el_deg'][0]:.1f} deg, "
        f"predicted Doppler {g['doppler_hz'][0] / 1e3:+.2f} kHz at {o.rf_freq / 1e9:g} GHz"
        + (f", sign {o.precomp_sign:+g}" if o.precomp_sign != 1 else "")
        + (f", dt {o.precomp_dt:+g} s" if o.precomp_dt else "")
        + (f", offset {o.precomp_offset_hz:+g} Hz" if o.precomp_offset_hz else ""))
    if abs(age) > 3:
        log(f"Doppler pre-comp: WARNING TLE is {abs(age):.1f} days from the recording: "
            "expect km-level along-track error; fetch a newer one if you can")
    return pc


def parse_station(s):
    if isinstance(s, (tuple, list)):
        return tuple(float(v) for v in s)
    v = [float(x) for x in str(s).split(",")]
    return (v[0], v[1], v[2] if len(v) > 2 else 0.0)


def add_args(p, group_title="Doppler pre-compensation (off unless --tle is given)"):
    g = p.add_argument_group(group_title)
    g.add_argument("--tle", help="TLE file (2- or 3-line sets) or OMM .json/.csv: remove the "
                   "predicted carrier Doppler before the AGC")
    g.add_argument("--sat", help="element set to use when the file has several: name "
                   "substring or NORAD number")
    g.add_argument("--start-utc", help="UTC of the first sample, ISO 8601 (e.g. "
                   "2026-10-25T03:12:00.25Z) or unix s. Default for a file: <file>.start, its "
                   ".json, else mtime - duration; for the B205mini: when streaming starts")
    g.add_argument("--station", default=",".join(str(v) for v in CRISP),
                   help="lat,lon[,height m] of the antenna [CRISP %(default)s]")
    g.add_argument("--rf-freq", type=float, default=FC_KU,
                   help="downlink RF frequency, Hz (the Doppler is set by RF, not IF) [13.95e9]")
    g.add_argument("--precomp-dt", type=float, default=0.0,
                   help="evaluate the orbit this many s later (clock error, or a known "
                        "along-track TLE error) [0]")
    g.add_argument("--precomp-offset-hz", type=float, default=0.0,
                   help="also remove this fixed frequency offset, Hz (e.g. a measured LO "
                        "error) [0]")
    g.add_argument("--precomp-sign", type=float, default=1.0, choices=[1.0, -1.0],
                   help="-1 if the IF spectrum is inverted and NOT already undone with "
                        "--conj [1]")
    return g


# ---------------------------------------------------------------------------- CLI
def _cmd_passes(o):
    name, s = load_satrec(o.tle, o.sat)
    m = DopplerModel(s, parse_station(o.station), o.rf_freq, name)
    t0 = parse_utc(o.start) if o.start else time.time()
    ps = m.passes(t0, t0 + o.hours * 3600, o.min_el)
    print(f"{name} (NORAD {s.satnum}), TLE epoch {fmt_utc(tle_epoch_unix(s))}, station "
          f"{m.station}, {len(ps)} passes above {o.min_el:g} deg in {o.hours:g} h")
    for p in ps:
        print(f"  AOS {p['aos_utc']}  TCA {p['tca_utc']}  LOS {p['los_utc']}  "
              f"{p['duration_s']:5.0f} s  max el {p['max_el_deg']:5.1f}  Doppler "
              f"{p['doppler_aos_hz'] / 1e3:+6.1f} .. {p['doppler_los_hz'] / 1e3:+6.1f} kHz  "
              f"max rate {p['max_abs_rate_hz_s'] / 1e3:.2f} kHz/s")
    if o.json:
        json.dump(ps, open(o.json, "w"), indent=1)
    return 0


def _cmd_profile(o):
    name, s = load_satrec(o.tle, o.sat)
    m = DopplerModel(s, parse_station(o.station), o.rf_freq, name)
    t0 = parse_utc(o.start_utc)
    t = t0 + np.arange(0, o.duration, o.step)
    g = m.geometry(t)
    rate = np.gradient(g["doppler_hz"], t)
    with open(o.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["utc", "t_s", "el_deg", "az_deg", "range_km", "range_rate_mps",
                    "doppler_hz", "doppler_rate_hz_s"])
        for i in range(len(t)):
            w.writerow([fmt_utc(t[i]), round(t[i] - t0, 3), round(g["el_deg"][i], 3),
                        round(g["az_deg"][i], 3), round(g["range_m"][i] / 1e3, 4),
                        round(g["range_rate_mps"][i], 3), round(g["doppler_hz"][i], 2),
                        round(rate[i], 2)])
    print(f"wrote {o.out}: {len(t)} rows, Doppler {g['doppler_hz'].min() / 1e3:+.1f} .. "
          f"{g['doppler_hz'].max() / 1e3:+.1f} kHz, max |rate| {np.abs(rate).max():.0f} Hz/s")
    return 0


def _cmd_apply(o):
    fmt = o.format
    if fmt == "auto":
        fmt = "sc16" if os.path.splitext(o.input)[1].lower() in (".sc16", ".cs16") else "fc32"
    bps = 4 if fmt == "sc16" else 8
    n_total = os.path.getsize(o.input) // bps
    pc = from_args(o, o.samp_rate, o.input, n_total)
    chunk = 1 << 20
    n0 = 0
    t_start = time.time()
    with open(o.input, "rb") as fi, open(o.output, "wb") as fo:
        while True:
            b = fi.read(chunk * bps)
            if len(b) < bps:
                break
            b = b[:len(b) - len(b) % bps]
            if fmt == "sc16":
                v = np.frombuffer(b, "<i2").astype(np.float32)
                x = (v[0::2] + 1j * v[1::2]).astype(np.complex64)
            else:
                x = np.frombuffer(b, np.complex64)
            y = pc.rotate(x, n0)
            n0 += len(x)
            if fmt == "sc16":
                w = np.empty(2 * len(y), np.float32)
                w[0::2], w[1::2] = y.real, y.imag
                fo.write(np.clip(np.rint(w), -32768, 32767).astype("<i2").tobytes())
            else:
                fo.write(y.astype(np.complex64).tobytes())
    # the output is aligned with the input, so the same start time applies
    if not os.path.exists(o.output + ".start") and os.path.isfile(o.output):
        with open(o.output + ".start", "w") as f:
            f.write(fmt_utc(pc.t0 - o.precomp_dt) + "\n# pre-compensated copy of "
                    f"{os.path.abspath(o.input)}\n")
    print(f"wrote {o.output}: {n0} samples ({n0 / o.samp_rate:.1f} s) in "
          f"{time.time() - t_start:.0f} s")
    return 0


def _cmd_stamp(o):
    fmt = o.format
    if fmt == "auto":
        fmt = "sc16" if os.path.splitext(o.input)[1].lower() in (".sc16", ".cs16") else "fc32"
    n = os.path.getsize(o.input) // (4 if fmt == "sc16" else 8)
    if o.start_utc:
        t0, src = parse_utc(o.start_utc), "--start-utc"
    else:
        t0, src = os.path.getmtime(o.input) - n / o.samp_rate, "mtime - duration"
    with open(o.input + ".start", "w") as f:
        f.write(f"{fmt_utc(t0)}\n# first sample of {os.path.basename(o.input)} ({src}, "
                f"{n} samples at {o.samp_rate:g} S/s = {n / o.samp_rate:.3f} s), "
                f"written {fmt_utc(time.time())}\n")
    print(f"{o.input}.start: {fmt_utc(t0)} ({src})")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("passes", help="list passes over the station")
    add_args(p, "satellite and station")
    p.add_argument("--from", dest="start", help="start UTC [now]")
    p.add_argument("--hours", type=float, default=24.0)
    p.add_argument("--min-el", type=float, default=5.0)
    p.add_argument("--json", help="also write the passes here")
    p = sub.add_parser("profile", help="Doppler / elevation table (CSV) from --start-utc")
    add_args(p, "satellite and station")
    p.add_argument("--duration", type=float, default=900.0, help="s [900]")
    p.add_argument("--step", type=float, default=0.1, help="s [0.1]")
    p.add_argument("--out", required=True)
    p = sub.add_parser("apply", help="write a pre-compensated copy of an IQ file")
    p.add_argument("input")
    p.add_argument("output", help="output file (or FIFO), same format as the input")
    p.add_argument("--samp-rate", type=float, required=True)
    p.add_argument("--format", default="auto", choices=["auto", "sc16", "fc32"])
    add_args(p)
    p = sub.add_parser("stamp", help="write <file>.start (first-sample UTC) from the file's "
                       "modification time; run it right after recording, before copying")
    p.add_argument("input")
    p.add_argument("--samp-rate", type=float, required=True)
    p.add_argument("--format", default="auto", choices=["auto", "sc16", "fc32"])
    p.add_argument("--start-utc", help="write this time instead")
    o = ap.parse_args(argv)
    if o.cmd in ("passes", "profile") and not o.tle:
        ap.error("--tle is required")
    if o.cmd == "profile" and not o.start_utc:
        ap.error("profile needs --start-utc")
    return {"passes": _cmd_passes, "profile": _cmd_profile, "apply": _cmd_apply,
            "stamp": _cmd_stamp}[o.cmd](o)


if __name__ == "__main__":
    sys.exit(main())
