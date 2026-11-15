#!/usr/bin/env python3
"""Motor current charts from logs/motor_<date>.csv (written by gimbal.py on every drive poll).

  ~/radioconda/bin/python cdr_charts/motor_charts.py                 # all real-hardware rows in logs/
  ~/radioconda/bin/python cdr_charts/motor_charts.py --date 20261113  # one day
  ... --include-sim                                                  # also rows from the simulated CAN link
  ... --logs DIR --out DIR
Note: currents are polled at 1 Hz per axis (0.5 Hz while tracking), so short acceleration spikes can be missed.
"""
import argparse, csv, glob, os
from datetime import datetime
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
ap = argparse.ArgumentParser()
ap.add_argument("--logs", default=os.path.join(ROOT, "logs"))
ap.add_argument("--out", default=HERE)
ap.add_argument("--date", default="")
ap.add_argument("--include-sim", action="store_true")
A = ap.parse_args()

C = {"x": "#eb6834", "y": "#1baf7a"}
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
LIMIT = {"x": 15.0, "y": 11.0}  # config.X_AXIS / Y_AXIS current_limit_a
plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF, "axes.edgecolor": GRID,
    "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2, "axes.grid": True, "grid.color": GRID,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 11, "axes.titlesize": 13,
    "axes.titleweight": "bold", "axes.titlecolor": INK, "lines.linewidth": 2, "legend.frameon": False, "figure.dpi": 150,
})

rows = []
for f in sorted(glob.glob(os.path.join(A.logs, f"motor_{A.date}*.csv"))):
    for r in csv.DictReader(open(f)):
        if not r["current_a"] or not r["angle_deg"]:
            continue
        if ("SIMULATED" in r["link"] or "127.0.0.1" in r["link"]) and not A.include_sim:   # sim / selftest relay
            continue
        r["t"] = datetime.fromisoformat(r["time_utc"])
        r["i"], r["ang"] = float(r["current_a"]), float(r["angle_deg"])
        rows.append(r)
if not rows:
    raise SystemExit(f"no motor rows in {A.logs} (date '{A.date}', include_sim={A.include_sim}). "
                     "Run the app on hardware first: gimbal.py now writes logs/motor_<date>.csv.")
sim = any("SIMULATED" in r["link"] or "127.0.0.1" in r["link"] for r in rows)
tag = " [includes SIMULATED rows]" if sim else ""


def activity(r):
    if r["act"]:
        return {"goto": "goto (slew)", "track": "tracking"}.get(r["act"], r["act"])
    return {"moving": "jog (WASD)", "starting": "jog start", "stopping": "jog stop", "auto": "goto (slew)",
            "waking": "start-up", "homing": "homing"}.get(r["phase"], "holding" if r["state"] not in ("", "0") else "motor off")


for r in rows:
    r["activity"] = activity(r)

# ---- 1. timeline: current and angle, per axis, with activity bands
t0 = rows[0]["t"]
fig, (a1, a2) = plt.subplots(2, 1, figsize=(13, 6.4), sharex=True, gridspec_kw={"hspace": 0.08})
xs = [r for r in rows if r["axis"] == "x"]
acts, spans = [], []
for r in xs:
    m = (r["t"] - t0).total_seconds() / 60
    if not acts or acts[-1] != r["activity"]:
        acts.append(r["activity"]); spans.append([m, m])
    spans[-1][1] = m
for j, (ac, (s0, s1)) in enumerate(zip(acts, spans)):
    if ac not in ("holding", "motor off"):
        for a in (a1, a2):
            a.axvspan(s0, s1, color=GRID, alpha=0.7, lw=0)
        if s1 - s0 > 0.02 * max(1e-6, spans[-1][1]):
            a1.text((s0 + s1) / 2, 1.01, ac, transform=a1.get_xaxis_transform(), ha="center", va="bottom", fontsize=8, color=INK2)
for k in ("x", "y"):
    rr = [r for r in rows if r["axis"] == k]
    m = np.array([(r["t"] - t0).total_seconds() / 60 for r in rr])
    cur, ang = np.array([r["i"] for r in rr]), np.array([r["ang"] for r in rr])
    gap = np.where(np.diff(m) > 10 / 60)[0] + 1                 # break the line over logging gaps > 10 s
    m, cur, ang = (np.insert(v.astype(float), gap, np.nan) for v in (m, cur, ang))
    a1.plot(m, cur, "-o", ms=3, lw=1.2, color=C[k], label=f"{k.upper()} axis")
    a1.axhline(LIMIT[k], color=C[k], lw=1, ls="--"); a1.axhline(-LIMIT[k], color=C[k], lw=1, ls="--")
    a2.plot(m, ang, color=C[k], label=f"{k.upper()} axis")
a1.set_ylabel("Motor current (A)"); a2.set_ylabel("Axis angle (deg)"); a2.set_xlabel(f"Minutes since {t0:%Y-%m-%d %H:%M} UTC")
a1.legend(loc="upper right", ncol=2, fontsize=9); a1.text(0.005, 0.97, "dashed = software trip limit (X 15 A, Y 11 A)",
                                                          transform=a1.transAxes, va="top", fontsize=8.5, color=INK2)
a1.set_title("Motor current with motion" + tag, pad=20)
fig.savefig(os.path.join(A.out, "07_motor_current_timeline.png"), bbox_inches="tight"); plt.close(fig)

# ---- 2. max |current| per activity and axis
order = ["motor off", "start-up", "homing", "holding", "jog start", "jog (WASD)", "jog stop", "goto (slew)", "tracking"]
seen = [a for a in order if any(r["activity"] == a for r in rows)] + sorted({r["activity"] for r in rows} - set(order))
fig, a = plt.subplots(figsize=(11, 4.6))
w = 0.38
lines = ["activity,axis,n,max_abs_a,mean_abs_a,angle_at_max_deg,time_at_max_utc"]
for j, k in enumerate(("x", "y")):
    vals = []
    for ac in seen:
        rr = [r for r in rows if r["axis"] == k and r["activity"] == ac]
        if not rr:
            vals.append(0); continue
        b = max(rr, key=lambda r: abs(r["i"]))
        vals.append(abs(b["i"]))
        lines.append(f"{ac},{k},{len(rr)},{abs(b['i']):.3f},{np.mean([abs(r['i']) for r in rr]):.3f},{b['ang']:.2f},{b['time_utc']}")
    xpos = np.arange(len(seen)) + (j - 0.5) * w
    a.bar(xpos, vals, width=w - 0.04, color=C[k], label=f"{k.upper()} axis (limit {LIMIT[k]:.0f} A)")
    for xp, v in zip(xpos, vals):
        if v:
            a.text(xp, v + 0.1, f"{v:.1f}", ha="center", fontsize=8, color=INK2)
a.set_xticks(np.arange(len(seen)), seen); a.grid(axis="x", visible=False)
a.set_ylabel("Max |current| (A)")
a.set_title("Peak motor current by what the gimbal was doing" + tag)
a.legend(fontsize=9)
fig.savefig(os.path.join(A.out, "08_motor_current_max_by_activity.png"), bbox_inches="tight"); plt.close(fig)
open(os.path.join(A.out, "motor_current_summary.csv"), "w").write("\n".join(lines) + "\n")
print("\n".join(lines)); print(f"{len(rows)} rows, charts in {A.out}")
