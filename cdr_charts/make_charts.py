#!/usr/bin/env python3
"""CDR charts from the gs_v3 field logs (2026-11-11). Run: ~/radioconda/bin/python cdr_charts/make_charts.py"""
import csv, json, os
from datetime import datetime, timezone
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "cdr_charts")
LOGS = os.path.join(ROOT, "logs")
REC = os.path.join(ROOT, "Received", "recordings")

C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.spines.top": False,
    "axes.spines.right": False, "font.size": 11, "axes.titlesize": 13, "axes.titleweight": "bold",
    "axes.titlecolor": INK, "lines.linewidth": 2, "legend.frameon": False, "figure.dpi": 150,
})
summary = []


def t(s):
    return datetime.fromisoformat(s)


def read_sun(stamp):
    rows = list(csv.DictReader(open(os.path.join(LOGS, f"sun_{stamp}.csv"))))
    for r in rows:
        r["t"] = t(r["time_utc"])
        for k in ("x_deg", "y_deg", "beam_az", "beam_el", "sun_az", "sun_el", "p_dbfs"):
            r[k] = float(r[k]) if r[k] else np.nan
    return rows


def sep(az1, el1, az2, el2):
    a1, e1, a2, e2 = map(np.radians, (az1, el1, az2, el2))
    c = np.sin(e1) * np.sin(e2) + np.cos(e1) * np.cos(e2) * np.cos(a1 - a2)
    return np.degrees(np.arccos(np.clip(c, -1, 1)))


def wrap(d):
    return (np.asarray(d) + 180) % 360 - 180


# ---------- SDR noise floor (parked at HOME, motors holding, 2026-11-11 03:10:33-03:13:03 UTC)
pw = [r for r in csv.DictReader(open(os.path.join(LOGS, "power_20261111.csv")))
      if "2026-11-11T03:10:35" <= r["time_utc"] <= "2026-11-11T03:13:02"]
p = np.array([float(r["p_dbfs"]) for r in pw])
sig01 = p.std()
sig1 = p[: len(p) // 10 * 10].reshape(-1, 10).mean(1).std()
summary.append(f"SDR noise floor parked at HOME (n={len(p)} x 0.1 s): mean {p.mean():.2f} dBFS, "
               f"sigma {sig01:.3f} dB per 0.1 s sample, {sig1:.3f} dB per 1 s average")

# ---------- 1. Ring scans: real (cloudy) vs simulated SDR
real = ["20261111_091047", "20261111_094100", "20261111_095654", "20261111_100154",
        "20261111_110037", "20261111_111900"]
fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.8), sharey=True, gridspec_kw={"wspace": 0.06})
for i, s in enumerate(real):
    rows = [r for r in read_sun(s) if r["step"].startswith("ring scan 1/3")]
    if len(rows) < 20:
        continue
    d = wrap(np.array([r["beam_az"] - r["sun_az"] for r in rows]))
    pp = np.array([r["p_dbfs"] for r in rows]); pp -= np.median(pp)
    o = np.argsort(d)
    a1.plot(d[o], pp[o], color=C[i], lw=1.4, label=f"{s[9:11]}:{s[11:13]} SGT+? run  (max rise {pp.max():+.2f} dB)".replace(" SGT+? run", ""))
    summary.append(f"Ring scan {s}: n={len(rows)}, max rise {pp.max():+.3f} dB, span {d.min():+.1f}..{d.max():+.1f} deg")
for a in (a1, a2):
    a.axhline(0.5, color=INK2, lw=1, ls="--")
    a.axhspan(-3 * sig01, 3 * sig01, color=GRID, alpha=0.6, lw=0)
    a.axvline(0, color=INK2, lw=0.8, ls=":")
a1.text(-11.5, 0.55, "0.5 dB detection threshold", color=INK2, fontsize=9)
a1.text(-11.5, -0.35, f"±3σ SDR noise ({3*sig01:.2f} dB)", color=INK2, fontsize=9)
a1.set_title("Real hardware, 11 Nov (cloudy): no Sun peak")
a1.set_xlabel("Beam azimuth minus predicted Sun azimuth (deg)")
a1.set_ylabel("Power rise over scan median (dB)")
a1.legend(fontsize=8.5, loc="upper right", title="Run start (SGT)", title_fontsize=8.5)
rows = [r for r in read_sun("20261111_103150") if r["step"].startswith("ring scan 1/3")]
d = wrap(np.array([r["beam_az"] - r["sun_az"] for r in rows])); pp = np.array([r["p_dbfs"] for r in rows]); pp -= np.median(pp)
o = np.argsort(d)
a2.plot(d[o], pp[o], color=C[6], lw=1.6)
a2.set_title("Same code, SIMULATED SDR (sim mode)")
a2.set_xlabel("Beam azimuth minus predicted Sun azimuth (deg)")
a2.annotate(f"+{pp.max():.1f} dB peak at {d[np.argmax(pp)]:+.2f}°\n→ north corrected", (d[np.argmax(pp)], pp.max()),
            xytext=(3, pp.max() - 1.5), fontsize=9, color=INK, arrowprops=dict(arrowstyle="-", color=INK2))
a1.set_ylim(-1, 9.5)
fig.savefig(os.path.join(OUT, "01_sun_ring_scans_real_vs_sim.png"), bbox_inches="tight"); plt.close(fig)
summary.append(f"Sim ring scan 20261111_103150: max rise {pp.max():+.2f} dB (SIMULATED SDR)")

# ---------- 2. Cloudy run timeline 03:19 UTC: power + axis angles, steps shaded
rows = read_sun("20261111_111900")
t0 = rows[0]["t"]
m = np.array([(r["t"] - t0).total_seconds() / 60 for r in rows])
steps, spans = [], []
for i, r in enumerate(rows):
    if not steps or steps[-1] != r["step"]:
        steps.append(r["step"]); spans.append([m[i], m[i]])
    spans[-1][1] = m[i]
fig, (b1, b2) = plt.subplots(2, 1, figsize=(13, 6.4), sharex=True, gridspec_kw={"hspace": 0.08, "height_ratios": [1.2, 1]})
short = {"Starting the motors": "start", "Going to HOME": "home", "GNSS heading average": "GNSS north (60 s avg)",
         "go to the predicted Sun": "slew", "ring scan 1/3: az ±12°, el +0.0°": "ring scan ±12°",
         "cold sky": "cold sky", "cross-scan X ±3°": "X cross-scan", "Pointing at the predicted Sun": "",
         "tracking the Sun": "track Sun (GNSS-only)", "moving 10° off the Sun": "off"}
for k, (st, (s0, s1)) in enumerate(zip(steps, spans)):
    for a in (b1, b2):
        if k % 2 == 0:
            a.axvspan(s0, s1, color=GRID, alpha=0.5, lw=0)
    b1.text((s0 + s1) / 2, 1.02, short.get(st, st), transform=b1.get_xaxis_transform(), ha="center", va="bottom",
            fontsize=8.5, color=INK2, rotation=0)
pp = np.array([r["p_dbfs"] for r in rows])
b1.plot(m, pp, color=C[0], lw=0.8)
b1.set_ylabel("SDR power (dBFS)")
b1.set_ylim(pp.min() - 0.1, pp.max() + 0.15)
b1.set_title("Sun auto-run 11 Nov 11:19 SGT (cloudy): peak-up found nothing → fell back to GNSS north and tracked", pad=22)
b2.plot(m, [r["x_deg"] for r in rows], color=C[1], label="X axis")
b2.plot(m, [r["y_deg"] for r in rows], color=C[2], label="Y axis")
b2.set_ylabel("Axis angle (deg)"); b2.set_xlabel("Minutes since start")
b2.legend(loc="lower left", ncol=2)
fig.savefig(os.path.join(OUT, "02_sun_cloudy_run_timeline.png"), bbox_inches="tight"); plt.close(fig)
summary.append(f"Cloudy run 03:19 UTC: total power range {pp.max()-pp.min():.2f} dB over whole run")

# ---------- 3. Sun tracking: encoder-derived beam vs Sun prediction
tr = [r for r in rows if r["step"] == "tracking the Sun"]
tm = np.array([(r["t"] - tr[0]["t"]).total_seconds() for r in tr])
e = sep(np.array([r["beam_az"] for r in tr]), np.array([r["beam_el"] for r in tr]),
        np.array([r["sun_az"] for r in tr]), np.array([r["sun_el"] for r in tr]))
fig, a = plt.subplots(figsize=(9, 3.6))
a.plot(tm, e, color=C[0])
a.set_xlabel("Seconds into Sun track"); a.set_ylabel("Beam − Sun (deg)")
a.set_title("Sun track servo error (encoder beam vs predicted Sun, same mount model)")
a.text(0.99, 0.92, f"mean {e.mean():.3f}°, 95th pct {np.percentile(e,95):.3f}°, max {e.max():.3f}°\n"
       "Measures servo following only, NOT true RF pointing (no Sun peak to check against)",
       transform=a.transAxes, ha="right", va="top", fontsize=9, color=INK2)
fig.savefig(os.path.join(OUT, "03_sun_track_servo_error.png"), bbox_inches="tight"); plt.close(fig)
summary.append(f"Sun track servo error: n={len(e)}, mean {e.mean():.3f}, p95 {np.percentile(e,95):.3f}, max {e.max():.3f} deg")

# ---------- 4. DS-EO pass sky plot + logged gimbal pointing
try:
    from skyfield.api import load, EarthSatellite, wgs84
    from sgp4.api import Satrec
    from sgp4 import omm
    ts = load.timescale()
    o = json.load(open(os.path.join(ROOT, "Received", "orbits", "52935.json")))[0]
    sr = Satrec(); omm.initialize(sr, o)
    sat = EarthSatellite.from_satrec(sr, ts)
    site = wgs84.latlon(1.29879, 103.77149, 60)
    T = ts.utc(2026, 11, 11, 3, 22, np.arange(0, 12.4 * 60, 5))
    alt, az, _ = (sat - site).at(T).altaz()
    sat_az, sat_el = az.degrees, alt.degrees
    secs = np.arange(0, 12.4 * 60, 5)
    def sat_at(dt):
        s = (dt - datetime(2026, 11, 11, 3, 22, tzinfo=timezone.utc)).total_seconds()
        return np.interp(s, secs, np.unwrap(np.radians(sat_az))) * 180 / np.pi % 360, np.interp(s, secs, sat_el)
    pl = [r for r in csv.DictReader(open(os.path.join(LOGS, "power_20261111.csv")))
          if "2026-11-11T03:22:00" <= r["time_utc"] <= "2026-11-11T03:34:30" and r["az"]]
    fig = plt.figure(figsize=(7.2, 7.2)); a = fig.add_subplot(projection="polar")
    a.set_theta_zero_location("N"); a.set_theta_direction(-1)
    a.set_rlim(90, 0); a.set_rticks([75, 60, 45, 30, 15]); a.set_rlabel_position(135)
    a.plot(np.radians(sat_az), sat_el, color=C[0], lw=2.5, label="DS-EO predicted pass (CelesTrak OMM)")
    for mm in range(0, 13, 2):
        i = mm * 12
        if i < len(sat_az):
            a.plot(np.radians(sat_az[i]), sat_el[i], "o", ms=5, color=C[0])
            a.text(np.radians(sat_az[i]), sat_el[i] - 4, f"03:{22+mm:02d}", fontsize=8, color=INK2)
    rec0, rec1 = datetime(2026, 11, 11, 3, 26, 46, tzinfo=timezone.utc), datetime(2026, 11, 11, 3, 29, 8, tzinfo=timezone.utc)
    w = [(sat_at(rec0 + (rec1 - rec0) * f)) for f in np.linspace(0, 1, 30)]
    a.plot(np.radians([x[0] for x in w]), [x[1] for x in w], color=C[3], lw=6, alpha=0.6, label="IQ recorded 03:26:46–03:29:08 (142 s, 0 overflows)")
    ga = np.array([float(r["az"]) for r in pl]); ge = np.array([float(r["el"]) for r in pl])
    a.scatter(np.radians(ga[::10]), ge[::10], s=10, color=C[1], label="Gimbal beam (encoder, 1 s) — logged while SDR free", zorder=3)
    a.plot(np.radians(339.4), 16.2, marker="*", ms=16, color=C[2], ls="none", label="Link-test pointing (ground TX), 03:35+")
    a.legend(loc="upper center", bbox_to_anchor=(0.5, -0.06), fontsize=8.5)
    a.set_title("DS-EO (NORAD 52935) pass, 11 Nov 03:22–03:34 UTC, max el 36.5°", pad=18)
    fig.savefig(os.path.join(OUT, "04_dseo_pass_skyplot.png"), bbox_inches="tight"); plt.close(fig)
    errs = []
    for r in pl:
        sa, se = sat_at(t(r["time_utc"]))
        errs.append((r["time_utc"][11:19], float(sep(float(r["az"]), float(r["el"]), sa, se))))
    summary.append("DS-EO logged gimbal-vs-predicted separation (sampled): " +
                   ", ".join(f"{a_}={b_:.1f}deg" for a_, b_ in errs[::60]))
except Exception as ex:
    summary.append(f"DS-EO skyplot skipped: {ex}")

# ---------- 5. IQ spectra: link test vs satellite pass (8 MS/s around 1150 MHz IF)
def psd(path, start_s, n=1 << 22, nfft=4096, fs=8e6):
    raw = np.fromfile(path, dtype=np.int16, count=2 * n, offset=int(start_s * fs) * 4)
    x = (raw[0::2] + 1j * raw[1::2]).astype(np.complex64) / 32768
    x = x[: len(x) // nfft * nfft].reshape(-1, nfft) * np.hanning(nfft)
    P = np.mean(np.abs(np.fft.fftshift(np.fft.fft(x, axis=1), axes=1)) ** 2, 0)
    f = np.fft.fftshift(np.fft.fftfreq(nfft, 1 / fs)) / 1e6
    return f, 10 * np.log10(P + 1e-20)
try:
    fig, a = plt.subplots(figsize=(10, 4.2))
    for path, st, lab, c in [(os.path.join(REC, "g5_20261111T033756Z.sc16"), 30, "Link test 03:37:56 (ground TX, 11 files decoded)", C[2]),
                             (os.path.join(REC, "g5_20261111T032646Z.sc16"), 70, "DS-EO pass 03:26:46 (mid-pass)", C[0])]:
        f, P = psd(path, st)
        a.plot(f, P - np.median(P), color=c, lw=1.2, label=lab)
        summary.append(f"PSD {os.path.basename(path)} @{st}s: in-band (|f|<1 MHz) minus edge (|f|>3 MHz) = "
                       f"{np.median(P[np.abs(f)<1]) - np.median(P[np.abs(f)>3]):.1f} dB")
    a.set_xlabel("Offset from 1150 MHz IF (MHz)"); a.set_ylabel("PSD rel. median (dB)")
    a.set_title("Received spectrum, B206mini 8 MS/s (DVB-S2 QPSK 2/3, 2 MBd)")
    a.legend(loc="upper left", fontsize=9)
    fig.savefig(os.path.join(OUT, "05_iq_spectrum_link_vs_pass.png"), bbox_inches="tight"); plt.close(fig)
except Exception as ex:
    summary.append(f"PSD skipped: {ex}")


# ---------- 6. Peak SDR power per Sun run and what the dish was doing at that moment
runs = sorted(f[4:-4] for f in os.listdir(LOGS) if f.startswith("sun_") and f.endswith(".csv"))
SIM = {"20261111_103009", "20261111_103142", "20261111_103150", "20261111_103322", "20261111_103805"}
lab, val, note, col = [], [], [], []
for s in runs:
    rows = read_sun(s)
    pp = np.array([r["p_dbfs"] for r in rows]); i = int(np.nanargmax(pp)); r = rows[i]
    rise = pp[i] - np.nanmedian(pp)
    off = sep(r["beam_az"], r["beam_el"], r["sun_az"], r["sun_el"])
    lab.append(f"{s[9:11]}:{s[11:13]} SGT" + ("  (SIM)" if s in SIM else ""))
    val.append(rise); col.append(C[6] if s in SIM else C[0])
    note.append(f"{r['step'][:34]} | X {r['x_deg']:+.1f}° Y {r['y_deg']:+.1f}° | {off:.1f}° from Sun")
    summary.append(f"Peak {s}{' SIM' if s in SIM else ''}: {pp[i]:.2f} dBFS, +{rise:.2f} dB over median, during '{r['step']}', "
                   f"X {r['x_deg']:+.2f} Y {r['y_deg']:+.2f}, beam {off:.1f} deg from Sun, sat={r['saturated']}")
fig, a = plt.subplots(figsize=(13, 0.48 * len(runs) + 1.4))
y = np.arange(len(runs))[::-1]
a.barh(y, val, color=col, height=0.6)
for yi, v, n in zip(y, val, note):
    a.text(max(v, 0) + 0.15, yi, n, va="center", fontsize=8.5, color=INK2)
a.set_yticks(y, lab); a.set_xlim(0, max(val) * 1.75)
a.axvline(0.5, color=INK2, lw=1, ls="--"); a.grid(axis="y", visible=False)
a.set_xlabel("Peak SDR power over run median (dB)")
a.set_title("Peak received power per Sun run (blue = real hardware, violet = simulated SDR) and dish state at the peak")
fig.savefig(os.path.join(OUT, "06_peak_power_per_run.png"), bbox_inches="tight"); plt.close(fig)

# ---------- 7. Demod run table
lines = ["| Run (UTC) | Source | BBFRAMEs | GSE OK / CRC bad | Files |", "|---|---|---|---|---|"]
for d in sorted(os.listdir(os.path.join(ROOT, "Received"))):
    if not d.startswith("run_"):
        continue
    rp = os.path.join(ROOT, "Received", d, "report.json")
    files = [f for f in os.listdir(os.path.join(ROOT, "Received", d)) if f not in ("report.json", "MANIFEST.txt")]
    if os.path.exists(rp):
        r = json.load(open(rp))
        lines.append(f"| {d[4:]} | {r['args'].get('input')} | {r['bbframe']['frames']} | {r['gsse']['ok']} / {r['gsse']['crc_bad']} | {len(files)} |")
    else:
        lines.append(f"| {d[4:]} | (no report.json) | ? | ? | {len(files)}: {sum(os.path.getsize(os.path.join(ROOT,'Received',d,f)) for f in files)} B |")
open(os.path.join(OUT, "demod_runs.md"), "w").write("\n".join(lines) + "\n")
open(os.path.join(OUT, "numbers.txt"), "w").write("\n".join(summary) + "\n")
print("\n".join(summary)); print("\n".join(lines))
