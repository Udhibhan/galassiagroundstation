"""
current_char_analysis.py - shared numbers for the current characterisation (numpy only, no plotting).

current_char.py uses it to write summary.json (at the end of a run, or on abort for whatever finished);
current_char_report.py uses it for the figures, LaTeX table and dashboard. Everything is computed from
the run's samples.csv / points.csv / events.jsonl / plan.json.

Conventions
  ix_a, iy_a: drive register 0x0008, MOTOR current in A (signed), not the 48 V supply current.
  itotal_a = |I_X| + |I_Y|. Supply current only exists in the clamp-meter readings (events.jsonl).
  x_dir / y_dir: direction the axis moves in (+1 / -1), 0 = not moving.
  lifting = moving toward HOME (against gravity), dir * sign(angle) < 0. Lowering = away from HOME.
  % of rating = max(|I_X| / 25 A, |I_Y| / 18 A), the continuous ratings of the eRob110 (X) and eRob90 (Y).
"""
import csv
import json
import math
import os

import numpy as np

RATED_A = {"x": 25.0, "y": 18.0}
SOFT_STOP_A = {"x": 15.0, "y": 11.0}
NOV7 = {"friction_A": (1.65, 1.7), "G_pos_A": 7.6, "G_neg_A": 6.6}     # 7 Nov X-only fit (report Section 4.2)
CAD_TABLE = [(62.0, 90.0), (64.0, 89.98), (66.0, 89.01), (68.0, 87.08), (70.0, 83.14), (72.0, 82.05),
             (74.0, 79.68), (76.0, 77.31), (78.0, 74.91), (80.0, 72.47)]
NUM = ["t_rel_s", "x_cmd_deg", "y_cmd_deg", "x_deg", "y_deg", "x_state", "y_state", "ix_a", "iy_a", "itotal_a",
       "x_dir", "y_dir", "speed_deg_s", "accel_reg", "bus_v", "temp_x_c", "temp_y_c"]
SAMPLE_COLS = ["t_utc", "t_rel_s", "phase", "step_id", "moving_axes", "x_cmd_deg", "y_cmd_deg", "x_deg", "y_deg",
               "x_state", "y_state", "ix_a", "iy_a", "itotal_a", "x_dir", "y_dir", "speed_deg_s", "accel_reg",
               "bus_v", "temp_x_c", "temp_y_c", "note"]
POINT_STATS = ["mean", "max", "min", "std", "n"]
POINT_COLS = (["x_deg", "y_deg", "approach_x", "approach_y"]
              + [f"{q}_{s}" for q in ("ix", "iy", "itotal") for s in POINT_STATS]
              + ["skipped_reason", "x_meas_deg", "y_meas_deg"])


# ------------------------------------------------------------------ CAD collision limit
def cad_max_abs_y(abs_x: float, margin: float = 3.0) -> float:
    """Largest safe |Y| at this |X|: interpolate the CAD table, but take the lower neighbouring value (the
    one at the next table |X| up, the conservative side), then subtract the margin."""
    ax = abs(abs_x)
    if ax <= CAD_TABLE[0][0]:
        return CAD_TABLE[0][1] - margin
    for (x0, y0), (x1, y1) in zip(CAD_TABLE, CAD_TABLE[1:]):
        if x0 < ax <= x1:
            lin = y0 + (y1 - y0) * (ax - x0) / (x1 - x0)
            return min(lin, y0, y1) - margin
    return CAD_TABLE[-1][1] - margin if ax <= CAD_TABLE[-1][0] + 1e-9 else -1.0     # beyond the table: nothing is safe


def cad_ok(x: float, y: float, margin: float = 3.0, soft: float = 80.0) -> bool:
    return abs(x) <= soft + 1e-9 and abs(y) <= soft + 1e-9 and abs(y) <= cad_max_abs_y(abs(x), margin) + 1e-9


def cad_max_abs_x(abs_y: float, margin: float = 3.0, soft: float = 80.0) -> float:
    """Largest |X| (<= soft limit, 0.1 deg resolution) where |Y| = abs_y is still safe."""
    x = soft
    while x > 0 and cad_max_abs_y(x, margin) + 1e-9 < abs(abs_y):
        x = round(x - 0.1, 3)
    return max(x, 0.0)


# ------------------------------------------------------------------ loading
def _f(v):
    try:
        if isinstance(v, str) and v.lower().startswith("0x"):
            return float(int(v, 16))                    # accel_reg is logged as hex text
        return float(v) if v not in ("", None) else math.nan
    except ValueError:
        return math.nan


def load_samples(run_dir: str) -> dict:
    """samples.csv as a dict of numpy arrays (numeric columns) / lists (text columns). A torn last line is skipped."""
    path = os.path.join(run_dir, "samples.csv")
    cols = {c: [] for c in SAMPLE_COLS}
    if os.path.exists(path):
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                if row.get("note") is None and row.get("t_utc") is None:
                    continue
                if any(row.get(c) is None for c in SAMPLE_COLS[:14]):      # torn line after a crash
                    continue
                for c in SAMPLE_COLS:
                    cols[c].append(row.get(c) or "")
    out = {c: (np.array([_f(v) for v in cols[c]], float) if c in NUM else cols[c]) for c in SAMPLE_COLS}
    out["n"] = len(cols["t_utc"])
    return out


def load_points(run_dir: str) -> list[dict]:
    path = os.path.join(run_dir, "points.csv")
    pts = []
    if os.path.exists(path):
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                if row.get("x_deg") in (None, ""):
                    continue
                p = {k: (_f(v) if k not in ("approach_x", "approach_y", "skipped_reason") else (v or "")) for k, v in row.items()}
                pts.append(p)
    return pts


def load_events(run_dir: str) -> list[dict]:
    path = os.path.join(run_dir, "events.jsonl")
    ev = []
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                try:
                    ev.append(json.loads(line))
                except ValueError:
                    pass
    return ev


def load_plan(run_dir: str) -> dict:
    try:
        with open(os.path.join(run_dir, "plan.json")) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


# ------------------------------------------------------------------ helpers
def pct_rating(ix, iy) -> float:
    ix = 0.0 if ix is None or not math.isfinite(ix) else abs(ix)
    iy = 0.0 if iy is None or not math.isfinite(iy) else abs(iy)
    return 100.0 * max(ix / RATED_A["x"], iy / RATED_A["y"])


def lifting(angle, d):
    """Boolean array: moving toward HOME (against gravity)."""
    angle, d = np.asarray(angle, float), np.asarray(d, float)
    return (d != 0) & (np.sign(angle) * d < 0) & (np.abs(angle) > 0.5)


def lowering(angle, d):
    angle, d = np.asarray(angle, float), np.asarray(d, float)
    return (d != 0) & (np.sign(angle) * d > 0) & (np.abs(angle) > 0.5)


def _dir_word(d):
    return "+" if d > 0 else ("-" if d < 0 else "holding")


def _row(S, i, extra=None) -> dict:
    """One sample as a JSON-friendly dict."""
    ix, iy = S["ix_a"][i], S["iy_a"][i]
    x, y = S["x_deg"][i], S["y_deg"][i]
    xd, yd = S["x_dir"][i], S["y_dir"][i]
    rec = {"t_utc": S["t_utc"][i], "t_rel_s": _r(S["t_rel_s"][i], 1), "phase": S["phase"][i], "step_id": S["step_id"][i],
           "x_deg": _r(x, 2), "y_deg": _r(y, 2), "ix_a": _r(ix, 3), "iy_a": _r(iy, 3), "itotal_a": _r(S["itotal_a"][i], 3),
           "x_dir": _dir_word(xd), "y_dir": _dir_word(yd),
           "x_motion": ("lifting" if lifting([x], [xd])[0] else "lowering" if lowering([x], [xd])[0] else "holding"),
           "y_motion": ("lifting" if lifting([y], [yd])[0] else "lowering" if lowering([y], [yd])[0] else "holding"),
           "speed_deg_s": _r(S["speed_deg_s"][i], 2), "pct_rating": _r(pct_rating(ix, iy), 1)}
    if extra:
        rec.update(extra)
    return rec


def _r(v, n):
    return None if v is None or not math.isfinite(float(v)) else round(float(v), n)


def _argmax(vals, mask):
    vals = np.where(mask & np.isfinite(vals), vals, -np.inf)
    if not np.any(np.isfinite(vals)):
        return None
    i = int(np.argmax(vals))
    return i if math.isfinite(vals[i]) else None


# ------------------------------------------------------------------ model fit
def _lstsq(A, b):
    """Returns coef, standard errors, R^2, n."""
    n, p = A.shape
    if n <= p + 1:
        return None
    coef, *_ = np.linalg.lstsq(A, b, rcond=None)
    res = b - A @ coef
    dof = max(n - p, 1)
    s2 = float(res @ res) / dof
    try:
        cov = s2 * np.linalg.inv(A.T @ A)
        se = np.sqrt(np.clip(np.diag(cov), 0, None))
    except np.linalg.LinAlgError:
        se = np.full(p, math.nan)
    ss_tot = float(((b - b.mean()) ** 2).sum())
    r2 = 1.0 - float(res @ res) / ss_tot if ss_tot > 0 else math.nan
    return coef, se, r2, n, float(math.sqrt(s2))


def fit_axis(S, axis: str, phases=("3a", "3b", "3c")) -> dict | None:  # noqa: C901
    """Fit I = c0 + F*sign(motion) + G+ * (-sin a)[a>=0] + G- * (-sin a)[a<0] on samples where only this axis
    moves (sweeps). G is defined so it is POSITIVE when the motor holds the load against gravity with the
    sign convention I = F*dir - G*sin(angle) (the simulator's); on hardware the signs come out as measured.
    Also: coupling term C * (-sin a) * (1 - cos b), b = the other axis, and G per sweep line."""
    o = "y" if axis == "x" else "x"
    a = np.radians(S[f"{axis}_deg"])
    b = np.radians(S[f"{o}_deg"])
    I = S[f"i{axis}_a"]
    d = S[f"{axis}_dir"]
    od = S[f"{o}_dir"]
    ph = np.array([p for p in S["phase"]])
    sweep = np.array([s.endswith(("-up", "-down", "-out")) for s in S["step_id"]], bool)   # sweeps only, not repositioning
    m = np.isin(ph, list(phases)) & sweep & (d != 0) & (od == 0) & np.isfinite(I) & np.isfinite(a) & np.isfinite(b)
    if m.sum() < 8:
        return None
    a, b, I, d = a[m], b[m], I[m], d[m]
    s = -np.sin(a)
    pos, neg = (a >= 0).astype(float), (a < 0).astype(float)
    A = np.column_stack([np.ones_like(I), np.sign(d), s * pos, s * neg])
    base = _lstsq(A, I)
    if base is None:
        return None
    coef, se, r2, n, rms = base
    out = {"axis": axis, "n": int(n), "phases": list(phases), "r2": _r(r2, 4), "rms_resid_A": _r(rms, 3),
           "model": "I = c0 + F*sign(motion) - G+*sin(a)[a>=0] - G-*sin(a)[a<0]",
           "c0_A": [_r(coef[0], 3), _r(se[0], 3)], "F_A": [_r(coef[1], 3), _r(se[1], 3)],
           "G_pos_A": [_r(coef[2], 3), _r(se[2], 3)], "G_neg_A": [_r(coef[3], 3), _r(se[3], 3)]}
    # coupling: does the other axis' angle change this axis' gravity term?
    A2 = np.column_stack([A, s * (1 - np.cos(b))])
    cp = _lstsq(A2, I)
    if cp is not None and np.ptp(np.degrees(b)) > 20:
        c2, se2, r22, _, _ = cp
        out["coupling"] = {"model": f"... - C*sin({axis})*(1-cos({o}))", "C_A": [_r(c2[4], 3), _r(se2[4], 3)], "r2": _r(r22, 4),
                           "significant": bool(abs(c2[4]) > 2 * se2[4] and abs(c2[4]) > 0.05 * max(abs(coef[2]), abs(coef[3]), 1e-6))}
    # G per sweep line (the other axis fixed): one G (both sides) per line
    lines = []
    ob = np.round(np.degrees(b) / 5.0) * 5.0
    for v in sorted(set(ob.tolist())):
        mm = ob == v
        if mm.sum() < 8:
            continue
        r = _lstsq(np.column_stack([np.ones(mm.sum()), np.sign(d[mm]), s[mm]]), I[mm])
        if r is None:
            continue
        lines.append({f"{o}_deg": float(v), "G_A": [_r(r[0][2], 3), _r(r[1][2], 3)], "F_A": [_r(r[0][1], 3), _r(r[1][1], 3)],
                      "n": int(mm.sum()), "r2": _r(r[2], 4)})
    # centre-of-gravity model: gravity torque of a load whose CoG is rotated phi from the dish axis about this shaft,
    # I = F*sign(motion) + A*sin(a) + B*cos(a)  ->  G = hypot(A, B), balance (zero-torque) angle a0 = atan2(-B, A)
    cg = _lstsq(np.column_stack([np.sign(d), np.sin(a), np.cos(a)]), I)
    if cg is not None:
        c3, se3, r23, _, rms3 = cg
        out["cog"] = {"model": "I = F*sign(motion) + A*sin(a) + B*cos(a)",
                      "F_A": [_r(c3[0], 3), _r(se3[0], 3)], "A_A": [_r(c3[1], 3), _r(se3[1], 3)], "B_A": [_r(c3[2], 3), _r(se3[2], 3)],
                      "G_A": _r(float(math.hypot(c3[1], c3[2])), 3),
                      "balance_deg": _r(float(math.degrees(math.atan2(-c3[2], c3[1]))) if abs(c3[1]) > 1e-9 else 90.0, 1),
                      "r2": _r(r23, 4), "rms_resid_A": _r(rms3, 3)}
        bd = out["cog"]["balance_deg"]                     # atan2 branch: report the balance angle inside +/-90
        if bd is not None and abs(bd) > 90:
            out["cog"]["balance_deg"] = _r(bd - math.copysign(180.0, bd), 1)
    out["per_line"] = lines
    if len(lines) >= 2:
        g = np.array([l["G_A"][0] for l in lines])
        e = np.array([l["G_A"][1] or 0 for l in lines])
        out["G_spread_A"] = _r(float(g.max() - g.min()), 3)
        out["G_spread_significant"] = bool(g.max() - g.min() > 2 * math.sqrt(float((e ** 2).max()) * 2))
    return out


# ------------------------------------------------------------------ summary
def summarise(run_dir: str, aborted: str | None = None) -> dict:
    S = load_samples(run_dir)
    P = load_points(run_dir)
    E = load_events(run_dir)
    plan = load_plan(run_dir)
    n = S["n"]
    out = {"run_dir": os.path.basename(os.path.normpath(run_dir)), "simulated": bool(plan.get("simulated")),
           "label": "SIMULATED" if plan.get("simulated") else "HARDWARE",
           "notes": ("register 0x0008 is MOTOR current, not 48 V supply current. 'Total' = |I_X| + |I_Y|. "
                     "Supply current is only known from the clamp-meter readings."),
           "rated_A": RATED_A, "soft_stop_A": SOFT_STOP_A, "n_samples": n, "aborted": aborted}
    if n == 0:
        return out
    t = S["t_rel_s"]
    out["duration_s"] = _r(np.nanmax(t) - np.nanmin(t), 1) if n else 0
    out["t_first_utc"], out["t_last_utc"] = S["t_utc"][0], S["t_utc"][-1]
    ph = np.array(S["phase"])
    out["phases_logged"] = sorted(set(S["phase"]), key=lambda p: S["phase"].index(p))
    allm = np.ones(n, bool)
    ix, iy, it = np.abs(S["ix_a"]), np.abs(S["iy_a"]), S["itotal_a"]
    peaks = {}
    for key, vals in (("x", ix), ("y", iy), ("total", it)):
        i = _argmax(vals, allm)
        if i is not None:
            peaks[key] = _row(S, i)
    out["peak"] = peaks
    # holding grid
    meas = [p for p in P if not p.get("skipped_reason")]
    out["grid"] = {"n_cells": len(P), "n_measured": len(meas), "n_skipped": len(P) - len(meas),
                   "skipped_reasons": sorted(set(p["skipped_reason"] for p in P if p.get("skipped_reason")))}
    if meas:
        w = max(meas, key=lambda p: p["itotal_max"] if math.isfinite(p["itotal_max"]) else -1)
        out["worst_holding"] = {"x_deg": w["x_deg"], "y_deg": w["y_deg"], "itotal_max_A": _r(w["itotal_max"], 3),
                                "ix_at_A": _r(w["ix_max"] if abs(w["ix_max"]) >= abs(w["ix_min"]) else w["ix_min"], 3),
                                "iy_at_A": _r(w["iy_max"] if abs(w["iy_max"]) >= abs(w["iy_min"]) else w["iy_min"], 3),
                                "approach_x": w["approach_x"], "approach_y": w["approach_y"],
                                "pct_rating": _r(pct_rating(max(abs(w["ix_max"]), abs(w["ix_min"])),
                                                            max(abs(w["iy_max"]), abs(w["iy_min"]))), 1)}
    # cases for the report table
    cases = {}
    hold = (S["x_dir"] == 0) & (S["y_dir"] == 0)
    hm = hold & np.isin(ph, ["2"])
    i = _argmax(it, hm if hm.any() else hold)
    if i is not None:
        cases["peak_holding"] = _row(S, i)
    lx = lifting(S["x_deg"], S["x_dir"]) & (S["y_dir"] == 0)
    i = _argmax(ix, lx)
    if i is not None:
        cases["peak_lifting_x"] = _row(S, i)
    ly = lifting(S["y_deg"], S["y_dir"]) & (S["x_dir"] == 0)
    i = _argmax(iy, ly)
    if i is not None:
        cases["peak_lifting_y"] = _row(S, i)
    lb = lifting(S["x_deg"], S["x_dir"]) & lifting(S["y_deg"], S["y_dir"])
    i = _argmax(it, lb)
    if i is not None:
        cases["both_lifting"] = _row(S, i)
    # breakaway (4a): per axis, the moving axis' peak within the first 3 s of the step (first 1 s not polled: bench rule)
    brk = {}
    for k in ("x", "y"):
        m = np.array([s == f"4a-{k}-lift" for s in S["step_id"]])
        if m.any():
            t0 = np.nanmin(t[m])
            vals = ix if k == "x" else iy
            w3 = m & (t - t0 <= 3.0 + 1e-6)
            i = _argmax(vals, w3)
            hold_m = np.array([s == f"4a-{k}-hold" for s in S["step_id"]])
            hv = vals[hold_m]
            if i is not None:
                brk[k] = _row(S, i, {"first_sample_after_start_s": _r(t[m].min() - t0, 2),
                                     "hold_mean_before_A": _r(float(np.nanmean(hv)), 3) if hv.size else None,
                                     "note": "drive not polled in the first 1 s after START (bench rule): peak is from the 1-3 s window"})
    out["breakaway"] = brk
    if brk:
        k = max(brk, key=lambda kk: brk[kk]["itotal_a"] or 0)
        cases["breakaway"] = brk[k]
    # reversal (4b)
    rev = {}
    for sid in sorted(set(s for s in S["step_id"] if s.startswith("4b-"))):
        m = np.array([s == sid for s in S["step_id"]])
        i = _argmax(it, m)
        if i is not None:
            rev[sid] = _row(S, i)
    out["reversal"] = rev
    # thermal hold (4c)
    m = ph == "4c"
    if m.any():
        th = {"duration_s": _r(np.nanmax(t[m]) - np.nanmin(t[m]), 0)}
        for k in ("x", "y"):
            tc = S[f"temp_{k}_c"][m]
            tc = tc[np.isfinite(tc)]
            if tc.size:
                th[f"temp_{k}_start_c"], th[f"temp_{k}_end_c"] = float(tc[0]), float(tc[-1])
                th[f"temp_{k}_max_c"] = float(tc.max())
                th[f"rise_{k}_c"] = float(tc[-1] - tc[0])
        i = _argmax(it, m)
        if i is not None:
            th["peak"] = _row(S, i)
            th["pose"] = [_r(np.nanmedian(S["x_deg"][m]), 1), _r(np.nanmedian(S["y_deg"][m]), 1)]
            th["itotal_mean_A"] = _r(float(np.nanmean(it[m])), 3)
        out["thermal"] = th
        if "peak" in th:
            rise = max(th.get("rise_x_c", 0), th.get("rise_y_c", 0))
            cases["long_hold"] = dict(th["peak"], temp_rise_c=rise, duration_s=th["duration_s"])
    # tracking profile (4d)
    m = ph == "4d"
    if m.any():
        tr = {"n": int(m.sum()), "duration_s": _r(np.nanmax(t[m]) - np.nanmin(t[m]), 0)}
        for k, v in (("x", ix), ("y", iy), ("total", it)):
            vv = v[m][np.isfinite(v[m])]
            if vv.size:
                tr[f"peak_{k}_A"] = _r(float(vv.max()), 3)
                tr[f"rms_{k}_A"] = _r(float(np.sqrt(np.mean(vv ** 2))), 3)
        i = _argmax(it, m)
        if i is not None:
            tr["peak"] = _row(S, i)
            cases["tracking_peak"] = tr["peak"]
        out["tracking"] = tr
    # STOP test (5)
    stp = [e for e in E if e.get("type") == "stop_test"]
    if stp:
        out["stop_test"] = stp
    # accel test (6)
    m = ph == "6"
    if m.any():
        i = _argmax(ix, m & lifting(S["x_deg"], S["x_dir"]))
        out["accel_test"] = {"accel_reg": int(np.nanmax(S["accel_reg"][m])) if np.isfinite(np.nanmax(S["accel_reg"][m])) else None,
                             "peak_lifting_x": _row(S, i) if i is not None else None}
        base = np.array([s.startswith("3a-y+0") for s in S["step_id"]])
        j = _argmax(ix, base & lifting(S["x_deg"], S["x_dir"]))
        if j is not None:
            out["accel_test"]["baseline_peak_lifting_x_at_0x419"] = _row(S, j)
    out["cases"] = cases
    # clamp meter, trips, aborts
    out["clamp_meter"] = [e for e in E if e.get("type") == "clamp_meter"]
    out["current_stop_trips"] = [e for e in E if e.get("type") == "current_trip"]
    # phase 3e: diagonal reach (operator STOP angle per diagonal) + peak currents on each diagonal leg
    dr = {}
    for e in E:
        if e.get("type") == "diag_reach" and e.get("x_deg") is not None:
            d = {"x_deg": e["x_deg"], "y_deg": e["y_deg"], "how": e.get("how", ""), "step": e.get("step")}
            m = np.array([sid == e.get("step") for sid in S["step_id"]], dtype=bool) if n else np.zeros(0, bool)
            for k in ("ix", "iy", "itotal"):
                v = S[k + "_a"][m] if m.any() else np.array([])
                v = v[np.isfinite(v)] if len(v) else v
                d[k + "_max_a"] = round(float(np.max(np.abs(v))), 3) if len(v) else None
            dr[e["diagonal"]] = d
    for i, e in enumerate(E):                     # return leg lost: the next phase-level event after the reach is an abort
        if e.get("type") == "diag_reach" and e.get("diagonal") in dr:
            nxt = next((f for f in E[i + 1:] if f.get("type") in ("diag_reach", "phase_end", "abort", "session_abort")), None)
            dr[e["diagonal"]]["return_recorded"] = not (nxt is not None and nxt.get("type") in ("abort", "session_abort"))
    out["diag_reach"] = dr
    m3e = np.array([str(sid).startswith("3e-") for sid in S["step_id"]], dtype=bool) if n else np.zeros(0, bool)
    if m3e.any():
        tt = S["itotal_a"][m3e]
        ok = np.isfinite(tt)
        if ok.any():
            i = int(np.flatnonzero(m3e)[np.flatnonzero(ok)[np.argmax(tt[ok])]])
            out["diag_peak"] = {"itotal_a": round(float(S["itotal_a"][i]), 3), "x_deg": round(float(S["x_deg"][i]), 2),
                                "y_deg": round(float(S["y_deg"][i]), 2), "step_id": S["step_id"][i]}
    out["dropouts"] = [e for e in E if e.get("type") in ("dropout", "dropout_transient", "soak_dropout")]
    # sweeps and sampling
    out["sweeps"] = sorted(set(s.rsplit("-", 1)[0] for s in S["step_id"] if s[:2] in ("3a", "3b", "3c", "3d") or s.startswith("6-")))
    out["n_sweeps"] = len(out["sweeps"])
    rates = {}
    mv =np.array([bool(s) for s in S["moving_axes"]])
    for lab, mm in (("moving", mv), ("holding", ~mv)):
        tt = np.sort(t[mm])
        dt = np.diff(tt)
        dt = dt[(dt > 0) & (dt < 5)]
        rates[lab + "_hz_median"] = _r(1.0 / float(np.median(dt)), 2) if dt.size else None
    out["sample_rate"] = rates
    out["sample_rate_cfg"] = (plan.get("args") or {}).get("rates")
    # temperatures / bus over the run
    for k in ("x", "y"):
        tc = S[f"temp_{k}_c"][np.isfinite(S[f"temp_{k}_c"])]
        if tc.size:
            out[f"temp_{k}_c"] = {"first": float(tc[0]), "max": float(tc.max()), "last": float(tc[-1])}
    bv = S["bus_v"][np.isfinite(S["bus_v"])]
    if bv.size:
        out["bus_v"] = {"min": _r(bv.min(), 2), "max": _r(bv.max(), 2)}
    # near a software stop (within 20 %)
    near = []
    for k, v in (("x", ix), ("y", iy)):
        lim = SOFT_STOP_A[k]
        i = _argmax(v, allm)
        if i is not None:
            pk = float(v[i])
            near.append({"axis": k, "peak_A": _r(pk, 3), "soft_stop_A": lim, "pct_of_stop": _r(100 * pk / lim, 1),
                         "within_20pct": bool(pk >= 0.8 * lim)})
    out["vs_software_stop"] = near
    # model fits
    out["fit"] = {"x": fit_axis(S, "x"), "y": fit_axis(S, "y"), "nov7_reference": NOV7}
    return out


def write_summary(run_dir: str, aborted: str | None = None) -> dict:
    s = summarise(run_dir, aborted)
    tmp = os.path.join(run_dir, "summary.json.tmp")
    with open(tmp, "w") as f:
        json.dump(s, f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, os.path.join(run_dir, "summary.json"))
    return s
