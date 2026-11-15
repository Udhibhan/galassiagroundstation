#!/usr/bin/env python3
"""
current_char_report.py - figures, LaTeX table and dashboard from one current_char.py run.

    python3 current_char_report.py data/current_char/<run> --report-dir urex_report
    python3 current_char_report.py <run> --report-dir urex_report --fill-report urex_report/report.tex [--add-trace-figure]

Writes into the run folder (PDF vector + PNG 200 dpi each):
    current_map_total  current_map_axes  current_vs_angle  current_worst_trace  current_model_fit
    current_dashboard.html (self-contained, no internet)   current_summary.tex   report_fill.md
and copies into the report folder:
    figures/current_*.pdf     data/current_summary.tex
    data/current_char/{summary.json, points.csv, plan.json, events.jsonl, samples_thinned.csv (1 in 10)}
--fill-report writes the Section 4.3 / Appendix C text into that .tex file (only the placeholders named in
the task; numbers only from the run). A SIMULATED run is labelled SIMULATED everywhere.
"""
import argparse
import csv
import json
import os
import shutil
import sys
from datetime import datetime, timedelta, timezone

import numpy as np

import current_char_analysis as A

BLUE, ORANGE, GREY, INK, MUTED = "#2F5597", "#C55A11", "#7F7F7F", "#222222", "#666666"
RED = "#B00020"
SGT = timezone(timedelta(hours=8))
FIGS = ["current_map_total", "current_map_axes", "current_vs_angle", "current_worst_trace", "current_model_fit"]


# ================================================================================== matplotlib setup
def mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "STIXGeneral", "mathtext.fontset": "stix", "font.size": 9.5,
                         "axes.titlesize": 10, "axes.labelsize": 9.5, "legend.fontsize": 8, "xtick.labelsize": 8.5,
                         "ytick.labelsize": 8.5, "axes.spines.top": False, "axes.spines.right": False,
                         "axes.grid": True, "grid.color": "#e3e3e3", "grid.linewidth": 0.6, "axes.edgecolor": "#555555",
                         "axes.labelcolor": INK, "xtick.color": "#444444", "ytick.color": "#444444",
                         "savefig.bbox": "tight", "savefig.pad_inches": 0.04, "pdf.fonttype": 42, "hatch.linewidth": 0.5})
    return plt


def save(fig, run_dir, name, sim):
    if sim:
        fig.text(0.5, 0.5, "SIMULATED", fontsize=46, color="#C00000", alpha=0.10, ha="center", va="center",
                 rotation=25, zorder=100, transform=fig.transFigure, fontweight="bold")
    fig.savefig(os.path.join(run_dir, name + ".pdf"))
    fig.savefig(os.path.join(run_dir, name + ".png"), dpi=200)


def title(ax_or_fig, text, sim, fig=False):
    t = ("SIMULATED \u2014 " if sim else "") + text
    if fig:
        ax_or_fig.suptitle(t, fontsize=10.5, x=0.02, ha="left", color=INK)
    else:
        ax_or_fig.set_title(t, loc="left", color=INK)


def deg(v, n=0):
    return "?" if v is None else f"{v:+.{n}f}\u00b0"


# ================================================================================== grids
def grid_arrays(P):
    """-> xs, ys (sorted unique), and matrices [iy, ix] of peak |I_X|+|I_Y|, peak |I_X|, peak |I_Y|, and a skipped mask."""
    xs = sorted(set(p["x_deg"] for p in P))
    ys = sorted(set(p["y_deg"] for p in P))
    nan = np.full((len(ys), len(xs)), np.nan)
    tot, ix, iy = nan.copy(), nan.copy(), nan.copy()
    skip = np.zeros_like(nan, bool)
    for p in P:
        i, j = ys.index(p["y_deg"]), xs.index(p["x_deg"])
        if p.get("skipped_reason"):
            skip[i, j] = True
            continue
        tot[i, j] = p.get("itotal_max", np.nan)
        ix[i, j] = max(abs(p.get("ix_max", np.nan)), abs(p.get("ix_min", np.nan)))
        iy[i, j] = max(abs(p.get("iy_max", np.nan)), abs(p.get("iy_min", np.nan)))
    return np.array(xs), np.array(ys), tot, ix, iy, skip


def edges(v):
    v = np.asarray(v, float)
    if len(v) == 1:
        return np.array([v[0] - 5, v[0] + 5])
    d = np.diff(v) / 2
    return np.r_[v[0] - d[0], v[:-1] + d, v[-1] + d[-1]]


def draw_cad(ax, xs=None, ys=None, skip=None):
    """Hatch the cells that were not visited (outside the CAD limit minus 3 deg) and draw the CAD limit itself
    (linear interpolation of the CAD table, no margin)."""
    from matplotlib.patches import Rectangle
    if skip is not None and skip.any():
        ex, ey = edges(xs), edges(ys)
        for i, j in zip(*np.nonzero(skip)):
            ax.add_patch(Rectangle((ex[j], ey[i]), ex[j + 1] - ex[j], ey[i + 1] - ey[i], facecolor="none",
                                   edgecolor="#666666", hatch="////", linewidth=0.0, zorder=3))
    tx = np.array([0.0] + [p[0] for p in A.CAD_TABLE])
    ty = np.array([90.0] + [p[1] for p in A.CAD_TABLE])
    for side in (1, -1):
        xx = np.linspace(0, 80, 321)
        cad = np.interp(xx, tx, ty)
        m = cad <= 90.0
        for sy in (1, -1):
            ax.plot(side * xx[m], sy * np.minimum(cad[m], 89.5), color="#333333", lw=0.9, zorder=4)
    ax.plot([], [], color="#333333", lw=0.9, label="CAD collision limit")
    ax.add_patch(Rectangle((0, 0), 0, 0, facecolor="none", edgecolor="#666666", hatch="////",
                           label="not visited (outside limit \u2212 3\u00b0)"))


def heat(ax, xs, ys, Z, vmin, vmax, cmap="cividis", contours=True):
    m = ax.pcolormesh(edges(xs), edges(ys), np.ma.masked_invalid(Z), cmap=cmap, vmin=vmin, vmax=vmax, shading="flat",
                      edgecolors="white", linewidth=0.6, zorder=1)
    if contours and np.isfinite(Z).sum() >= 6 and len(xs) >= 3 and len(ys) >= 3:
        Zf = Z.copy()
        if np.isnan(Zf).any():                                    # fill holes for contouring only
            from numpy import nanmean
            Zf = np.where(np.isnan(Zf), nanmean(Zf), Zf)
        levels = np.unique(np.round(np.linspace(np.nanmin(Z), np.nanmax(Z), 7)[1:-1], 1))
        if len(levels) >= 2:
            cs = ax.contour(xs, ys, Zf, levels=levels, colors="white", linewidths=0.7, alpha=0.9, zorder=2)
            ax.clabel(cs, fmt="%.1f", fontsize=7, inline=True)
    ax.set_xlim(-90, 90)
    ax.set_ylim(-90, 90)
    ax.set_xticks(range(-80, 81, 20))
    ax.set_yticks(range(-80, 81, 20))
    ax.set_xlabel("X axis angle from HOME (deg)")
    ax.set_ylabel("Y axis angle from HOME (deg)")
    ax.set_aspect("equal")
    ax.grid(False)
    return m


def mark_worst(ax, xs, ys, Z, unit="A"):
    if not np.isfinite(Z).any():
        return None
    i, j = np.unravel_index(np.nanargmax(Z), Z.shape)
    x, y, v = xs[j], ys[i], Z[i, j]
    ax.plot(x, y, marker="o", ms=11, mfc="none", mec="#C00000", mew=1.6, zorder=6)
    dx = -1 if x > 0 else 1
    dy = -1 if y > 0 else 1
    ax.annotate(f"worst {v:.1f} {unit}\nX {_dg(x)}\u00b0, Y {_dg(y)}\u00b0", (x, y), xytext=(x + dx * 32, y + dy * 22),
                fontsize=7.5, color=INK, ha="center", zorder=7,
                bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#C00000", lw=0.6),
                arrowprops=dict(arrowstyle="-", color="#C00000", lw=0.8))
    return x, y, v


# ================================================================================== figures
def fig_map_total(plt, run_dir, P, sim, info):
    xs, ys, tot, ix, iy, skip = grid_arrays(P)
    fig, ax = plt.subplots(figsize=(16 / 2.54, 12 / 2.54))
    if not np.isfinite(tot).any():
        _path_map(plt, fig, ax, info["_S"], "itotal_a", "$|I_X|+|I_Y|$ along the measured paths (A)", sim, "Total")
        save(fig, run_dir, "current_map_total", sim)
        plt.close(fig)
        return
    m = heat(ax, xs, ys, tot, 0, np.nanmax(tot))
    draw_cad(ax, xs, ys, skip)
    w = mark_worst(ax, xs, ys, tot)
    cb = fig.colorbar(m, ax=ax, fraction=0.046, pad=0.03)
    cb.set_label("peak $|I_X|+|I_Y|$ while holding (A)")
    ax.legend(loc="lower left", fontsize=7, frameon=True, framealpha=0.9, edgecolor="#cccccc")
    pct = info.get("worst_holding", {}).get("pct_rating")
    title(ax, f"Peak holding current {w[2]:.1f} A at X {_dg(w[0])}\u00b0, Y {_dg(w[1])}\u00b0"
              + (f" ({pct:.0f}% of rating)" if pct is not None else ""), sim)
    save(fig, run_dir, "current_map_total", sim)
    plt.close(fig)


def _path_map(plt, fig, ax, S, col, cblab, sim, what, vmax=None, cbar=True):
    """No holding grid (fast plan): current along the swept paths, coloured at the pose it was measured."""
    v = np.abs(S[col])
    m = np.isfinite(v) & np.isfinite(S["x_deg"]) & np.isfinite(S["y_deg"])
    draw_cad(ax)
    ax.set_xlim(-90, 90)
    ax.set_ylim(-90, 90)
    ax.set_xticks(range(-80, 81, 20))
    ax.set_yticks(range(-80, 81, 20))
    ax.set_aspect("equal")
    ax.set_xlabel("X axis angle from HOME (deg)")
    ax.set_ylabel("Y axis angle from HOME (deg)")
    if not m.any():
        ax.text(0.5, 0.5, "not measured", ha="center", va="center", transform=ax.transAxes)
        return None
    o = np.argsort(v[m])
    sc = ax.scatter(S["x_deg"][m][o], S["y_deg"][m][o], c=v[m][o], cmap="cividis", vmin=0, vmax=vmax or v[m].max(),
                    s=7, lw=0, zorder=5)
    i = np.flatnonzero(m)[np.argmax(v[m])]
    ax.plot(S["x_deg"][i], S["y_deg"][i], marker="o", ms=11, mfc="none", mec="#C00000", mew=1.6, zorder=6)
    ax.annotate(f"peak {v[i]:.1f} A\nX {_dg(S['x_deg'][i])}\u00b0, Y {_dg(S['y_deg'][i])}\u00b0", (S["x_deg"][i], S["y_deg"][i]),
                xytext=(-30 if S["x_deg"][i] > 0 else 30, -30 if S["y_deg"][i] > 0 else 30), textcoords="offset points",
                fontsize=7.5, ha="center", bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#C00000", lw=0.6),
                arrowprops=dict(arrowstyle="-", color="#C00000", lw=0.8), zorder=7)
    if cbar:
        cb = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.03)
        cb.set_label(cblab)
    ax.legend(loc="lower left", fontsize=7, frameon=True, framealpha=0.9, edgecolor="#cccccc")
    if what:
        title(ax, f"{what} current {v[i]:.1f} A at X {_dg(S['x_deg'][i])}\u00b0, Y {_dg(S['y_deg'][i])}\u00b0 "
                  f"(along the sweep paths; no holding grid)", sim)
    return sc


def fig_map_axes(plt, run_dir, P, sim, S=None):
    xs, ys, tot, ix, iy, skip = grid_arrays(P)
    fig, axs = plt.subplots(1, 2, figsize=(17 / 2.54, 8.6 / 2.54), constrained_layout=True)
    if not np.isfinite(ix).any() and not np.isfinite(iy).any():
        vmax = float(np.nanmax(np.abs(np.r_[S["ix_a"], S["iy_a"]]))) if S is not None and S["n"] else 1.0
        sc = None
        for ax, col, lab in ((axs[0], "ix_a", "X"), (axs[1], "iy_a", "Y")):
            r = _path_map(plt, fig, ax, S, col, "", sim, None, vmax=vmax, cbar=False)
            sc = sc or r
            ax.set_title(f"({'a' if lab == 'X' else 'b'}) $|I_{lab}|$ along the measured paths", loc="left", fontsize=9)
        if sc is not None:
            fig.colorbar(sc, ax=axs, fraction=0.035, pad=0.02, shrink=0.85).set_label("|I| (A)")
        title(fig, "Current by axis along the sweep paths (no holding grid), one colour scale", sim, fig=True)
        save(fig, run_dir, "current_map_axes", sim)
        plt.close(fig)
        return
    vmax = np.nanmax([np.nanmax(ix) if np.isfinite(ix).any() else 0, np.nanmax(iy) if np.isfinite(iy).any() else 0]) or 1
    msgs = []
    for ax, Z, lab in ((axs[0], ix, "X"), (axs[1], iy, "Y")):
        if not np.isfinite(Z).any():
            ax.text(0.5, 0.5, "not measured", ha="center", va="center", transform=ax.transAxes)
            continue
        m = heat(ax, xs, ys, Z, 0, vmax, contours=True)
        draw_cad(ax, xs, ys, skip)
        w = mark_worst(ax, xs, ys, Z)
        ax.set_title(f"({'a' if lab == 'X' else 'b'}) $|I_{lab}|$ peak {w[2]:.1f} A at X {_dg(w[0])}\u00b0, Y {_dg(w[1])}\u00b0",
                     loc="left", fontsize=9)
        msgs.append(f"$|I_{lab}|$ peak {w[2]:.1f} A")
        if lab == "Y":
            ax.set_ylabel("")
    cb = fig.colorbar(m, ax=axs, fraction=0.035, pad=0.02, shrink=0.85)
    cb.set_label("peak |I| while holding (A)")
    title(fig, "Holding current by axis (one colour scale): " + ", ".join(msgs), sim, fig=True)
    save(fig, run_dir, "current_map_axes", sim)
    plt.close(fig)


def _runs(mask):
    """Index runs of consecutive True."""
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return []
    cuts = np.flatnonzero(np.diff(idx) > 1) + 1
    return np.split(idx, cuts)


def _line_colors(plt, n):
    cm = plt.get_cmap("viridis")
    return [cm(v) for v in np.linspace(0.08, 0.82, max(n, 1))]


def fig_vs_angle(plt, run_dir, S, sim, info):
    fig, axs = plt.subplots(1, 2, figsize=(17 / 2.54, 9.2 / 2.54), constrained_layout=True, sharey=True)
    steps = np.array(S["step_id"])
    head = []
    for ax, axis, other, tag, letter in ((axs[0], "x", "y", "3a", "a"), (axs[1], "y", "x", "3b", "b")):
        ang, oth = S[f"{axis}_deg"], S[f"{other}_deg"]
        I = np.abs(S[f"i{axis}_a"])
        d = S[f"{axis}_dir"]
        sweep = np.array([s.startswith(tag + "-") and s.endswith(("-up", "-down", "-out")) for s in steps])
        lines = sorted(set(round(v / 5) * 5 for v in oth[sweep & np.isfinite(oth)]))
        cols = _line_colors(plt, len(lines))
        best = (0, None)
        for c, ov in zip(cols, lines):
            sel = sweep & (np.abs(oth - ov) < 2.5) & (d != 0) & np.isfinite(I)
            for kind, ls in (("lift", "-"), ("lower", "--")):
                mk = sel & (A.lifting(ang, d) if kind == "lift" else A.lowering(ang, d))
                for run in _runs(mk):
                    if run.size < 2:
                        continue
                    o = np.argsort(ang[run])
                    ax.plot(ang[run][o], I[run][o], color=c, lw=1.3 if ls == "-" else 1.0, ls=ls)
                if kind == "lift" and mk.any():
                    k = np.flatnonzero(mk)[np.nanargmax(I[mk])]
                    if I[k] > best[0]:
                        best = (I[k], k)
            ax.plot([], [], color=c, lw=1.3, label=f"{other.upper()} = {_dg(ov)}\u00b0")
        rated, stop = A.RATED_A[axis], A.SOFT_STOP_A[axis]
        ax.axhline(stop, color=ORANGE, ls="--", lw=1)
        ax.text(-88, stop + 0.4, f"software stop {stop:g} A (60%)", color=ORANGE, fontsize=7.5)
        ax.axhline(rated, color=RED, lw=1)
        ax.text(-88, rated + 0.4, f"continuous rating {rated:g} A", color=RED, fontsize=7.5)
        ax.set_xlim(-90, 90)
        ax.set_xticks(range(-80, 81, 20))
        ax.set_ylim(0, 27.5)
        ax.set_xlabel(f"{axis.upper()} axis angle from HOME (deg)")
        if axis == "x":
            ax.set_ylabel("motor current |I| (A)")
        if not lines:
            ax.text(0.5, 0.5, f"phase {tag} not measured", ha="center", va="center", transform=ax.transAxes)
        else:
            ax.plot([], [], color=GREY, lw=1.3, ls="-", label="lifting (toward HOME)")
            ax.plot([], [], color=GREY, lw=1.0, ls="--", label="lowering")
            ax.legend(loc="upper center", ncol=3, fontsize=6.8, frameon=False, bbox_to_anchor=(0.5, -0.2))
        if best[1] is not None:
            k = best[1]
            ax.plot(ang[k], I[k], "o", ms=5, mfc="white", mec=INK, zorder=5)
            ax.annotate(f"{I[k]:.1f} A", (ang[k], I[k]), xytext=(0, 7), textcoords="offset points", ha="center", fontsize=7.5)
            head.append(f"{axis.upper()} {I[k]:.1f} A at {axis.upper()} {_dg(ang[k])}\u00b0 ({other.upper()} {_dg(oth[k])}\u00b0)")
        ax.set_title(f"({letter}) {axis.upper()} sweeps at 1\u00b0/s, one line per {other.upper()}", loc="left", fontsize=9)
    title(fig, "Peak lifting current: " + ("; ".join(head) if head else "sweeps not measured"), sim, fig=True)
    save(fig, run_dir, "current_vs_angle", sim)
    plt.close(fig)


def fig_worst_trace(plt, run_dir, S, sim, info):
    fig, axs = plt.subplots(1, 3, figsize=(17 / 2.54, 6.4 / 2.54), constrained_layout=True)
    t = S["t_rel_s"]
    steps = np.array(S["step_id"])
    head = []
    # (a) breakaway
    ax = axs[0]
    any_ = False
    for k, c in (("x", BLUE), ("y", ORANGE)):
        m_l = steps == f"4a-{k}-lift"
        m_h = steps == f"4a-{k}-hold"
        if not m_l.any():
            continue
        any_ = True
        t0 = np.nanmin(t[m_l]) - 1.0 if True else 0
        # START time is ~ 1 s (quiet) before the first logged sample of the lift step
        tl = t[m_l]
        t_start = tl.min() - info.get("quiet_s", 1.0)
        mm = (m_h & (t >= t_start - 6)) | (m_l & (t <= t_start + 6))
        I = np.abs(S[f"i{k}_a"])
        ax.plot(t[mm] - t_start, I[mm], color=c, lw=1.2, marker="o", ms=2.5, label=f"{k.upper()} lift 5\u00b0")
        b = info.get("breakaway", {}).get(k)
        if b:
            head.append(f"{k.upper()} {abs(b['i' + k + '_a']):.1f} A")
    ax.axvspan(0, info.get("quiet_s", 1.0), color="#bbbbbb", alpha=0.35, lw=0)
    ax.text(0.5 * info.get("quiet_s", 1.0), 0.04, "not polled\n(bench rule)", transform=ax.get_xaxis_transform(),
            ha="center", fontsize=6.3, color=MUTED)
    ax.axvline(0, color=INK, lw=0.7)
    ax.set_xlabel("time from START (s)")
    ax.set_ylabel("motor current |I| (A)")
    ax.set_title("(a) breakaway (4a)", loc="left", fontsize=9)
    if any_:
        ax.legend(fontsize=6.8, frameon=False, loc="upper left")
    else:
        ax.text(0.5, 0.5, "not measured", ha="center", va="center", transform=ax.transAxes)
    # (b) reversal
    ax = axs[1]
    sids = sorted(set(s.rsplit("-", 1)[0] for s in steps if s.startswith("4b-")))
    for sid in sids:
        axis = sid[3]
        m = np.array([s.startswith(sid + "-out") or s.startswith(sid + "-rev") for s in steps])
        if not m.any():
            continue
        r = np.flatnonzero(np.array([s.startswith(sid + "-rev") for s in steps]))
        t_rev = t[r].min() if r.size else np.nanmin(t[m])
        I = np.abs(S[f"i{axis}_a"])
        ls = "-" if "+" in sid else "--"
        ax.plot(t[m] - t_rev, I[m], color=BLUE if axis == "x" else ORANGE, ls=ls, lw=1.1,
                label=f"{axis.upper()} {sid[4:]}\u00b0")
    ax.axvline(0, color=INK, lw=0.7)
    ax.set_xlabel("time from reversal (s)")
    ax.set_title("(b) reversal at the extremes (4b)", loc="left", fontsize=9)
    if sids:
        ax.legend(fontsize=6.5, frameon=False, ncol=2, loc="upper left")
        rv = info.get("reversal") or {}
        if rv:
            kk = max(rv, key=lambda q: rv[q]["itotal_a"] or 0)
            head.append(f"reversal {rv[kk]['itotal_a']:.1f} A total")
    elif any(str(q).startswith("3e-") and str(q).endswith("-back") for q in steps):
        # no reversal test in this plan: (b) shows the diagonal returns to HOME instead (both axes lifting)
        names = {"pp": "from +X+Y", "mm": "from −X−Y", "pm": "from +X−Y", "mp": "from −X+Y"}
        for code, c in zip(("mm", "pm", "mp", "pp"), (BLUE, ORANGE, "#555555", "#8c6bb1")):
            m = steps == f"3e-{code}-back"
            if not m.any():
                continue
            o = np.argsort(np.abs(S["x_deg"][m]))
            ax.plot(np.abs(S["x_deg"][m])[o], S["itotal_a"][m][o], color=c, lw=1.1, label=names[code])
        ax.set_xlabel("|X| = |Y| along the diagonal (deg)")
        ax.set_title("(b) diagonal returns to HOME (3e)", loc="left", fontsize=9)
        ax.legend(fontsize=6.3, frameon=False, loc="upper left")
        dp = info.get("diag_peak")
        if dp:
            head.append(f"diagonal {dp['itotal_a']:.1f} A total")
    else:
        ax.text(0.5, 0.5, "not measured", ha="center", va="center", transform=ax.transAxes)
    # (c) thermal hold, temperature on a second axis (as requested)
    ax = axs[2]
    m = np.array(S["phase"]) == "4c"
    if m.any():
        tm = (t[m] - t[m].min()) / 60
        ax.plot(tm, np.abs(S["ix_a"][m]), color=BLUE, lw=1.2, label="$|I_X|$")
        ax.plot(tm, np.abs(S["iy_a"][m]), color=ORANGE, lw=1.2, label="$|I_Y|$")
        ax.set_xlabel("time in hold (min)")
        ax2 = ax.twinx()
        ax2.spines["right"].set_visible(True)
        ax2.plot(tm, S["temp_x_c"][m], color=BLUE, lw=0.9, ls=":", label="X drive temp.")
        ax2.plot(tm, S["temp_y_c"][m], color=ORANGE, lw=0.9, ls=":", label="Y drive temp.")
        ax2.set_ylabel("drive temperature (\u00b0C, dotted)")
        ax2.grid(False)
        lo = np.nanmin(np.r_[S["temp_x_c"][m], S["temp_y_c"][m]])
        hi = np.nanmax(np.r_[S["temp_x_c"][m], S["temp_y_c"][m]])
        ax2.set_ylim(lo - 2, hi + 4)
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=6.3, frameon=False, loc="lower right", ncol=2)
        th = info.get("thermal") or {}
        rise = max(th.get("rise_x_c", 0), th.get("rise_y_c", 0))
        head.append(f"hold +{rise:.0f}\u00b0C")
        ax.set_ylim(0, max(1.0, np.nanmax(np.abs(np.r_[S["ix_a"][m], S["iy_a"][m]])) * 1.35))
    else:
        ax.text(0.5, 0.5, "not measured", ha="center", va="center", transform=ax.transAxes)
    ax.set_title("(c) thermal hold (4c)", loc="left", fontsize=9)
    title(fig, "Worst-case traces: breakaway " + ", ".join(head), sim, fig=True)
    save(fig, run_dir, "current_worst_trace", sim)
    plt.close(fig)


def fig_model(plt, run_dir, S, sim, info):
    """Measured sweep currents with the centre-of-gravity model I = F sign(v) + A sin a + B cos a per axis.
    A third panel (G_X per Y line) only when the X sweeps covered two or more Y positions."""
    fit = info.get("fit") or {}
    pl = (fit.get("x") or {}).get("per_line") or []
    three = len(pl) >= 2
    fig, axs = plt.subplots(1, 3 if three else 2, figsize=(17 / 2.54, 6.4 / 2.54), constrained_layout=True,
                            gridspec_kw={"width_ratios": [1.15, 1.15, 0.9] if three else [1, 1]})
    head = []
    for ax, axis, letter in ((axs[0], "x", "a"), (axs[1], "y", "b")):
        f = fit.get(axis) or {}
        cg = f.get("cog")
        o = "y" if axis == "x" else "x"
        ph = np.array(S["phase"])
        d = S[f"{axis}_dir"]
        sweep = np.array([str(q).endswith(("-up", "-down", "-out")) for q in S["step_id"]], bool)
        m = np.isin(ph, ["3a", "3b", "3c"]) & sweep & (d != 0) & (S[f"{o}_dir"] == 0) & np.isfinite(S[f"i{axis}_a"])
        col, light = (BLUE, "#9aa7c7") if axis == "x" else (ORANGE, "#e3b597")
        if m.any():
            ax.scatter(S[f"{axis}_deg"][m], S[f"i{axis}_a"][m], s=2.5, color=light, alpha=0.7, lw=0, rasterized=True,
                       label="measured (sweeps)")
        if cg:
            a = np.linspace(-80, 80, 321)
            r = np.radians(a)
            for dirn, ls, lab in ((1, "-", "model, moving +"), (-1, "--", "model, moving \u2212")):
                ax.plot(a, cg["F_A"][0] * dirn + cg["A_A"][0] * np.sin(r) + cg["B_A"][0] * np.cos(r), color=col, ls=ls, lw=1.3, label=lab)
            ax.axvline(cg["balance_deg"], color=col, lw=0.7, ls=":")
            ax.text(cg["balance_deg"], 0.50, f"balance {cg['balance_deg']:+.0f}\u00b0 ", transform=ax.get_xaxis_transform(),
                    fontsize=6.3, color=col, va="center", ha="right", rotation=90)
            txt = (f"$F$ = {abs(cg['F_A'][0]):.2f} \u00b1 {cg['F_A'][1]:.2f} A (friction)\n"
                   f"$G$ = {cg['G_A']:.2f} A (gravity)\n"
                   f"$R^2$ = {cg['r2']:.3f}, rms {cg['rms_resid_A']:.2f} A, n = {f['n']}")
            ax.text(0.03, 0.03, txt, transform=ax.transAxes, fontsize=6.4, va="bottom",
                    bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#cccccc", lw=0.5, alpha=0.9))
            head.append(f"{axis.upper()}: friction {abs(cg['F_A'][0]):.2f} A, gravity {cg['G_A']:.1f} A, balance {cg['balance_deg']:+.0f}\u00b0")
        if axis == "x":
            a = np.linspace(-80, 80, 321)
            nov = np.where(a >= 0, 1.7 + 7.6 * np.abs(np.sin(np.radians(a))), 1.65 + 6.6 * np.abs(np.sin(np.radians(a))))
            lift_sign = 1.0
            if cg:   # lifting from +X = moving -, current sign of the model there
                lift_sign = np.sign(-cg["F_A"][0] + cg["A_A"][0] * np.sin(np.radians(60))) or 1.0
            sign = np.where(a >= 0, lift_sign, -lift_sign)
            for half in (a < 0, a >= 0):
                ax.plot(a[half], (sign * nov)[half], color=GREY, lw=1.0, ls=":",
                        label="7 Nov X-only fit (lifting)" if half[-1] else None)
        ax.axhline(0, color="#999999", lw=0.6)
        ax.set_xlim(-85, 85)
        ax.set_xticks(range(-80, 81, 40))
        ax.set_xlabel(f"{axis.upper()} angle from HOME (deg)")
        ax.set_ylabel(f"$I_{axis.upper()}$, signed (A)")
        ax.set_title(f"({letter}) {axis.upper()} axis ({'Y' if axis == 'x' else 'X'} at HOME)", loc="left", fontsize=8.5)
        ax.legend(fontsize=5.8, frameon=False, loc="upper right")
        if not cg:
            ax.text(0.5, 0.5, "not enough sweep data", ha="center", va="center", transform=ax.transAxes)
    if three:
        ax = axs[2]
        fx = fit.get("x") or {}
        yv = [q["y_deg"] for q in pl]
        gv = [abs(q["G_A"][0]) for q in pl]
        ge = [q["G_A"][1] or 0 for q in pl]
        ax.errorbar(yv, gv, yerr=[2 * e for e in ge], fmt="o", color=BLUE, ms=4, capsize=2, lw=1)
        ax.set_xlabel("Y position of the X sweep (deg)")
        ax.set_ylabel("$|G_X|$ fitted per line (A)")
        cp = fx.get("coupling") or {}
        ax.set_ylim(0, max(gv) * 1.3)
        ax.set_title("(c) $G_X$ vs Y (CoG coupling)", loc="left", fontsize=9)
        head.append("Y position changes X gravity" if cp.get("significant") else "no Y\u2192X coupling")
    title(fig, "Model $I = F\\,\\mathrm{sign}(v) + A\\sin\\theta + B\\cos\\theta$: " + ("; ".join(head) if head else "not fitted"), sim, fig=True)
    save(fig, run_dir, "current_model_fit", sim)
    plt.close(fig)


# ================================================================================== LaTeX table
def _a(v, dash="--"):
    """|v| to 2 decimals; an axis not read in that sample (breakaway: moving axis only) -> dash."""
    return dash if v is None else f"{abs(v):.2f}"


def _pose(c):
    return f"$({_dg(c['x_deg'])}, {_dg(c['y_deg'])})$"


def summary_tex(info, plan):
    sim = info.get("simulated")
    cases = info.get("cases") or {}
    date = _date(info)
    hold_min = (info.get("thermal") or {}).get("duration_s")
    rows = [("peak_holding", "Peak holding"), ("peak_lifting_x", "Peak lifting, X"), ("peak_lifting_y", "Peak lifting, Y"),
            ("both_lifting", "Both axes lifting"), ("breakaway", "Start from rest (breakaway)"),
            ("long_hold", None), ("tracking_peak", "Tracking profile peak (DS-EO)")]
    lines = []
    for key, lab in rows:
        c = cases.get(key)
        if key == "long_hold":
            if c:
                lab = (f"Long hold, {_dur(c['duration_s'])} (+{c['temp_rise_c']:.0f}\\,\\textdegree C)")
            else:
                lab = "Long hold (temperature rise)"
        if not c:
            lines.append(f"{lab} & \\multicolumn{{5}}{{c}}{{not measured}}\\\\")
            continue
        lines.append(f"{lab} & {_pose(c)} & {_a(c['ix_a'])} & {_a(c['iy_a'])} & {_a(c['itotal_a'])} & "
                     f"{c['pct_rating']:.0f}\\%\\\\")
    cap = (("SIMULATED DATA (pipeline test, not the gimbal). " if sim else "")
           + f"Worst cases from the current characterisation, {date}, plan \\texttt{{{plan.get('args', {}).get('plan', '?')}}}. "
           "Motor current (register 0x0008); total $=|I_X|+|I_Y|$; \\% of rating $=\\max(|I_X|/25\\,\\mathrm{A},\\,|I_Y|/18\\,\\mathrm{A})$."
           + (" Breakaway: the drive is not polled in the first 1\\,s after START (bench rule)." if cases.get("breakaway") else ""))
    tex = ["% generated by current_char_report.py from " + info.get("run_dir", "?") + (" -- SIMULATED" if sim else ""),
           "\\begin{table}[!htbp]\\centering\\small",
           f"\\caption{{{cap}}}\\label{{tab:worst}}",
           "\\begin{tabular}{@{}lccccc@{}}\\toprule",
           "Case & Pose $(X, Y)$ (\\textdegree) & $|I_X|$ (A) & $|I_Y|$ (A) & Total (A) & \\% of rating\\\\\\midrule"]
    tex += lines[:-1] + [lines[-1] + "\\bottomrule"]
    tex += ["\\end{tabular}\\end{table}", ""]
    return "\n".join(tex)


def _date(info):
    t = info.get("t_first_utc")
    try:
        d = datetime.fromisoformat(t).astimezone(SGT)
        return f"{d.day} {d.strftime('%B %Y')}"
    except (TypeError, ValueError):
        return "?"


# ================================================================================== dashboard
DASH = r"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Gimbal Current Dashboard</title>
<style>
:root{--bg:#fbfbfa;--panel:#ffffff;--ink:#1f1f1f;--muted:#6a6a6a;--line:#e4e4e2;--blue:#2F5597;--orange:#C55A11;--warn:#b00020}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#161616;--panel:#1f1f1f;--ink:#ececec;--muted:#a3a3a3;--line:#333;--blue:#7d9de0;--orange:#e8915a;--warn:#ff6b81}}
:root[data-theme="dark"]{--bg:#161616;--panel:#1f1f1f;--ink:#ececec;--muted:#a3a3a3;--line:#333;--blue:#7d9de0;--orange:#e8915a;--warn:#ff6b81}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.45 "STIX Two Text","Times New Roman",serif}
main{max-width:1180px;margin:0 auto;padding:16px}h1{font-size:22px;margin:4px 0 2px}h2{font-size:15px;margin:0 0 8px}
.sub{color:var(--muted);font-size:13px}.banner{background:var(--warn);color:#fff;font-weight:700;padding:6px 12px;border-radius:4px;margin:8px 0;letter-spacing:.04em}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:14px;margin-top:14px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:12px;min-width:0}
.wide{grid-column:1/-1}svg,canvas{display:block;width:100%;height:auto}
table{border-collapse:collapse;width:100%;font-size:13px}th,td{padding:5px 6px;border-bottom:1px solid var(--line);text-align:right}th:first-child,td:first-child{text-align:left}
th{color:var(--muted);font-weight:600}#tip{position:fixed;pointer-events:none;background:var(--panel);color:var(--ink);border:1px solid var(--line);
border-radius:6px;padding:5px 8px;font-size:12px;box-shadow:0 2px 8px rgba(0,0,0,.15);display:none;z-index:9;white-space:nowrap}
.legend{display:flex;flex-wrap:wrap;gap:10px;font-size:12px;color:var(--muted);margin-top:6px}.sw{display:inline-block;width:14px;height:3px;vertical-align:middle;margin-right:4px}
.note{font-size:12px;color:var(--muted);margin-top:6px}
</style></head><body><main>
<h1>Gimbal motor current</h1><div class="sub" id="sub"></div><div id="banner"></div>
<div class="grid">
<div class="card wide"><h2>Total current over X and Y while holding (3D, drag to rotate)</h2><canvas id="surf" width="1100" height="520"></canvas>
<div class="note">Height and colour: peak |I<sub>X</sub>|+|I<sub>Y</sub>| per grid cell (A). Cells outside the CAD limit minus 3&deg; were not visited.</div></div>
<div class="card"><h2>Peak |I<sub>X</sub>|+|I<sub>Y</sub>| holding (A)</h2><svg id="hmT"></svg></div>
<div class="card"><h2>Peak |I<sub>X</sub>| and |I<sub>Y</sub>| holding (A), shared scale</h2><svg id="hmX"></svg><svg id="hmY"></svg></div>
<div class="card wide"><h2>Current while moving (sweeps at 1&deg;/s)</h2><svg id="swX"></svg><svg id="swY"></svg>
<div class="legend">solid = lifting (toward HOME), dashed = lowering. Orange dashed = 60% software stop, red = continuous rating.</div></div>
<div class="card wide"><h2>Worst cases</h2><table id="tbl"></table><div class="note" id="tnote"></div></div>
</div></main><div id="tip"></div>
<script>
const D = __DATA__;
const CIV=[[0,32,77],[0,52,110],[57,71,108],[87,92,109],[112,113,115],[138,135,120],[166,157,117],[195,180,107],[225,205,90],[255,234,70]];
function cmap(t){t=Math.max(0,Math.min(1,t));const f=t*(CIV.length-1),i=Math.min(Math.floor(f),CIV.length-2),u=f-i,a=CIV[i],b=CIV[i+1];
 return `rgb(${a.map((v,k)=>Math.round(v+(b[k]-v)*u)).join(",")})`}
const tip=document.getElementById("tip");
function showTip(e,h){tip.innerHTML=h;tip.style.display="block";const w=tip.offsetWidth;let x=e.clientX+14;if(x+w>innerWidth-8)x=e.clientX-w-14;tip.style.left=x+"px";tip.style.top=(e.clientY+12)+"px"}
function hideTip(){tip.style.display="none"}
const css=v=>getComputedStyle(document.documentElement).getPropertyValue(v).trim();
document.getElementById("sub").textContent=D.subtitle;
if(D.simulated)document.getElementById("banner").innerHTML='<div class="banner">SIMULATED DATA - pipeline test, not the real gimbal</div>';
const NS="http://www.w3.org/2000/svg";function el(t,a,p){const e=document.createElementNS(NS,t);for(const k in a)e.setAttribute(k,a[k]);if(p)p.appendChild(e);return e}
// ---------- heat maps
function heat(id,Z,vmax,lab){const s=document.getElementById(id),W=420,H=380,L=44,B=34,T=8,R=58;s.setAttribute("viewBox",`0 0 ${W} ${H}`);
 const xs=D.grid.x,ys=D.grid.y,pw=W-L-R,ph=H-T-B,sx=v=>L+(v+90)/180*pw,sy=v=>T+(90-v)/180*ph,st=D.grid.step;
 for(let i=0;i<ys.length;i++)for(let j=0;j<xs.length;j++){const v=Z[i][j],x0=sx(xs[j]-st/2),y0=sy(ys[i]+st/2),w=sx(xs[j]+st/2)-x0-1.5,h=sy(ys[i]-st/2)-y0-1.5;
  const r=el("rect",{x:x0,y:y0,width:w,height:h,rx:2,fill:v==null?"none":cmap(v/vmax),stroke:v==null?css("--line"):"none","stroke-dasharray":v==null?"3 2":""},s);
  r.addEventListener("mousemove",e=>showTip(e,`X ${xs[j]>0?"+":""}${xs[j]}&deg;, Y ${ys[i]>0?"+":""}${ys[i]}&deg;<br>${lab}: <b>${v==null?"not visited":v.toFixed(2)+" A"}</b>`));r.addEventListener("mouseleave",hideTip)}
 for(let v=-80;v<=80;v+=40){el("text",{x:sx(v),y:H-B+16,"text-anchor":"middle","font-size":11,fill:css("--muted")},s).textContent=v;
  el("text",{x:L-6,y:sy(v)+4,"text-anchor":"end","font-size":11,fill:css("--muted")},s).textContent=v}
 el("text",{x:L+pw/2,y:H-4,"text-anchor":"middle","font-size":12,fill:css("--ink")},s).textContent="X (deg)";
 const g=el("text",{x:12,y:T+ph/2,"text-anchor":"middle","font-size":12,fill:css("--ink"),transform:`rotate(-90 12 ${T+ph/2})`},s);g.textContent="Y (deg)";
 for(let k=0;k<40;k++)el("rect",{x:W-R+14,y:T+ph-(k+1)*ph/40,width:12,height:ph/40+0.5,fill:cmap(k/39)},s);
 for(const f of [0,.5,1])el("text",{x:W-R+30,y:T+ph-f*ph+4,"font-size":10,fill:css("--muted")},s).textContent=(f*vmax).toFixed(1);
 el("text",{x:W-R+14,y:H-B+16,"font-size":10,fill:css("--muted")},s).textContent="A"}
if(D.grid.x.length){heat("hmT",D.grid.tot,D.grid.tmax,"|Ix|+|Iy|");heat("hmX",D.grid.ix,D.grid.amax,"|Ix|");heat("hmY",D.grid.iy,D.grid.amax,"|Iy|")}
// ---------- sweeps
function sweep(id,lines,axis,stop,rated){const s=document.getElementById(id),W=1100,H=300,L=48,B=34,T=14,R=150;s.setAttribute("viewBox",`0 0 ${W} ${H}`);
 const pw=W-L-R,ph=H-T-B,ymax=Math.max(rated+2,...lines.flatMap(l=>l.i)),sx=v=>L+(v+90)/180*pw,sy=v=>T+ph-v/ymax*ph;
 for(let v=0;v<=ymax;v+=5){el("line",{x1:L,x2:L+pw,y1:sy(v),y2:sy(v),stroke:css("--line")},s);el("text",{x:L-6,y:sy(v)+4,"text-anchor":"end","font-size":11,fill:css("--muted")},s).textContent=v}
 for(let v=-80;v<=80;v+=20)el("text",{x:sx(v),y:H-B+16,"text-anchor":"middle","font-size":11,fill:css("--muted")},s).textContent=v;
 el("text",{x:L+pw/2,y:H-3,"text-anchor":"middle","font-size":12,fill:css("--ink")},s).textContent=axis.toUpperCase()+" angle (deg)";
 el("text",{x:14,y:T+ph/2,"text-anchor":"middle","font-size":12,fill:css("--ink"),transform:`rotate(-90 14 ${T+ph/2})`},s).textContent="|I"+axis.toUpperCase()+"| (A)";
 el("line",{x1:L,x2:L+pw,y1:sy(stop),y2:sy(stop),stroke:css("--orange"),"stroke-dasharray":"5 4"},s);
 el("line",{x1:L,x2:L+pw,y1:sy(rated),y2:sy(rated),stroke:"#c0392b"},s);
 if(!lines.length){el("text",{x:L+pw/2,y:T+ph/2,"text-anchor":"middle",fill:css("--muted")},s).textContent="not measured";return}
 lines.forEach((l,n)=>{const c=l.color;for(const seg of l.segs){el("polyline",{points:seg.map(k=>sx(l.a[k])+","+sy(l.i[k])).join(" "),fill:"none",stroke:c,"stroke-width":2,
   "stroke-dasharray":l.kind[seg[0]]?"":"6 4","stroke-linejoin":"round"},s)}
   el("line",{x1:W-R+12,x2:W-R+34,y1:T+10+n*18,y2:T+10+n*18,stroke:c,"stroke-width":2},s);
   el("text",{x:W-R+40,y:T+14+n*18,"font-size":12,fill:css("--ink")},s).textContent=l.label});
 const cross=el("line",{y1:T,y2:T+ph,stroke:css("--muted"),"stroke-width":1,visibility:"hidden"},s);
 const ov=el("rect",{x:L,y:T,width:pw,height:ph,fill:"transparent"},s);
 ov.addEventListener("mousemove",e=>{const r=s.getBoundingClientRect(),xv=((e.clientX-r.left)/r.width*W-L)/pw*180-90;cross.setAttribute("x1",sx(xv));cross.setAttribute("x2",sx(xv));cross.setAttribute("visibility","visible");
  let h=`${axis.toUpperCase()} = ${xv.toFixed(1)}&deg;`;for(const l of lines){let best=null,bd=3;for(let k=0;k<l.a.length;k++){const d=Math.abs(l.a[k]-xv);if(d<bd&&(best==null||l.kind[k]==1||l.kind[best]!=1)){bd=d;best=k}}
   if(best!=null)h+=`<br><span style="color:${l.color}">&#9644;</span> ${l.label}: <b>${l.i[best].toFixed(2)} A</b> ${l.kind[best]?"lifting":"lowering"}`}showTip(e,h)});
 ov.addEventListener("mouseleave",()=>{hideTip();cross.setAttribute("visibility","hidden")})}
sweep("swX",D.sweeps.x,"x",D.stop.x,D.rated.x);sweep("swY",D.sweeps.y,"y",D.stop.y,D.rated.y);
// ---------- table
const tb=document.getElementById("tbl");tb.innerHTML="<tr><th>Case</th><th>Pose (X, Y)</th><th>|I<sub>X</sub>| (A)</th><th>|I<sub>Y</sub>| (A)</th><th>Total (A)</th><th>% of rating</th><th>Phase / step</th></tr>"+
 D.cases.map(c=>c.na?`<tr><td>${c.label}</td><td colspan="6" style="text-align:center;color:var(--muted)">not measured</td></tr>`:
 `<tr><td>${c.label}</td><td>(${c.x}, ${c.y})</td><td>${c.ix}</td><td>${c.iy}</td><td><b>${c.tot}</b></td><td>${c.pct}%</td><td>${c.where}</td></tr>`).join("");
document.getElementById("tnote").textContent=D.note;
// ---------- 3D surface (painter's algorithm)
(function(){const cv=document.getElementById("surf"),ctx=cv.getContext("2d");const xs=D.grid.x,ys=D.grid.y,Z=D.grid.tot;if(!xs.length){ctx.fillStyle=css("--muted");ctx.fillText("holding map not measured",20,30);return}
 let az=-0.75,el_=0.62,drag=null;const zmax=D.grid.tmax||1;
 function P(x,y,z){const X=x/90,Y=y/90,Zz=z/zmax*0.9;const ca=Math.cos(az),sa=Math.sin(az),ce=Math.cos(el_),se=Math.sin(el_);
  const x1=X*ca-Y*sa,y1=X*sa+Y*ca;const sxp=x1,syp=y1*se-Zz*ce,dep=y1*ce+Zz*se;return[cv.width/2+sxp*330,cv.height/2+40+syp*200,dep]}
 function draw(){ctx.clearRect(0,0,cv.width,cv.height);const q=[];
  for(let i=0;i<ys.length-1;i++)for(let j=0;j<xs.length-1;j++){const v=[Z[i][j],Z[i][j+1],Z[i+1][j+1],Z[i+1][j]];if(v.some(a=>a==null))continue;
   const pts=[P(xs[j],ys[i],v[0]),P(xs[j+1],ys[i],v[1]),P(xs[j+1],ys[i+1],v[2]),P(xs[j],ys[i+1],v[3])];q.push({pts,d:pts.reduce((s,p)=>s+p[2],0)/4,m:(v[0]+v[1]+v[2]+v[3])/4})}
  q.sort((a,b)=>a.d-b.d);for(const f of q){ctx.beginPath();f.pts.forEach((p,k)=>k?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.closePath();ctx.fillStyle=cmap(f.m/zmax);ctx.fill();ctx.strokeStyle="rgba(255,255,255,.55)";ctx.lineWidth=1;ctx.stroke()}
  ctx.strokeStyle=css("--muted");ctx.fillStyle=css("--muted");ctx.font="14px serif";
  for(const [a,b,t] of [[[-80,-80,0],[80,-80,0],"X"],[[-80,-80,0],[-80,80,0],"Y"],[[-80,-80,0],[-80,-80,zmax],"A"]]){const p=P(...a),r=P(...b);ctx.beginPath();ctx.moveTo(p[0],p[1]);ctx.lineTo(r[0],r[1]);ctx.stroke();ctx.fillText(t==="A"?zmax.toFixed(1)+" A":t+" +80",r[0]+4,r[1])}}
 cv.addEventListener("pointerdown",e=>{drag=[e.clientX,e.clientY,az,el_];cv.setPointerCapture(e.pointerId)});
 cv.addEventListener("pointermove",e=>{if(!drag)return;az=drag[2]+(e.clientX-drag[0])*0.01;el_=Math.max(0.05,Math.min(1.5,drag[3]+(e.clientY-drag[1])*0.01));draw()});
 cv.addEventListener("pointerup",()=>drag=null);draw()})();
</script></body></html>
"""


def dashboard(run_dir, S, P, info, plan):
    import matplotlib.cm as cm
    xs, ys, tot, ix, iy, skip = grid_arrays(P) if P else (np.array([]), np.array([]), np.zeros((0, 0)), None, None, None)
    tolist = (lambda Z: [[None if not np.isfinite(v) else round(float(v), 3) for v in row] for row in Z])
    grid = {"x": [float(v) for v in xs], "y": [float(v) for v in ys], "step": float(np.min(np.diff(xs))) if len(xs) > 1 else 10.0,
            "tot": tolist(tot) if len(xs) else [], "ix": tolist(ix) if len(xs) else [], "iy": tolist(iy) if len(xs) else [],
            "tmax": float(np.nanmax(tot)) if len(xs) and np.isfinite(tot).any() else 1.0,
            "amax": float(np.nanmax([np.nanmax(ix), np.nanmax(iy)])) if len(xs) and np.isfinite(ix).any() else 1.0}
    steps = np.array(S["step_id"])
    sweeps = {}
    for axis, other, tag in (("x", "y", "3a"), ("y", "x", "3b")):
        ang, oth = S[f"{axis}_deg"], S[f"{other}_deg"]
        I = np.abs(S[f"i{axis}_a"])
        d = S[f"{axis}_dir"]
        sw = np.array([s.startswith(tag + "-") and s.endswith(("-up", "-down", "-out")) for s in steps]) & (d != 0) & np.isfinite(I)
        vals = sorted(set(round(v / 5) * 5 for v in oth[sw & np.isfinite(oth)]))
        out = []
        for n, ov in enumerate(vals):
            m = sw & (np.abs(oth - ov) < 2.5)
            idx = np.flatnonzero(m)
            lift = A.lifting(ang[idx], d[idx])
            segs, cur = [], [0]
            for k in range(1, len(idx)):
                if idx[k] != idx[k - 1] + 1 or lift[k] != lift[k - 1]:
                    segs.append(cur)
                    cur = []
                cur.append(k)
            if cur:
                segs.append(cur)
            c = cm.viridis(0.08 + 0.74 * n / max(len(vals) - 1, 1))
            out.append({"label": f"{other.upper()} = {_dg(ov)}\u00b0", "a": [round(float(v), 2) for v in ang[idx]],
                        "i": [round(float(v), 3) for v in I[idx]], "kind": [int(v) for v in lift],
                        "segs": [s for s in segs if len(s) >= 2],
                        "color": "#%02x%02x%02x" % tuple(int(255 * v) for v in c[:3])})
        sweeps[axis] = out
    labels = [("peak_holding", "Peak holding"), ("peak_lifting_x", "Peak lifting, X"), ("peak_lifting_y", "Peak lifting, Y"),
              ("both_lifting", "Both axes lifting"), ("breakaway", "Start from rest (breakaway)"),
              ("long_hold", "Long hold"), ("tracking_peak", "Tracking profile peak")]
    cases = []
    for k, lab in labels:
        c = (info.get("cases") or {}).get(k)
        if not c:
            cases.append({"label": lab, "na": True})
            continue
        if k == "long_hold":
            lab = f"Long hold, {_dur(c['duration_s'])} (+{c['temp_rise_c']:.0f} \u00b0C)"
        cases.append({"label": lab, "x": f"{_dg(c['x_deg'])}", "y": f"{_dg(c['y_deg'])}", "ix": _a(c["ix_a"], "\u2013"),
                      "iy": _a(c["iy_a"], "\u2013"), "tot": _a(c["itotal_a"], "\u2013"), "pct": f"{c['pct_rating']:.0f}",
                      "where": f"{c['phase']} / {c['step_id']}"})
    data = {"simulated": bool(info.get("simulated")), "grid": grid, "sweeps": sweeps, "cases": cases,
            "stop": A.SOFT_STOP_A, "rated": A.RATED_A,
            "subtitle": (f"{'SIMULATED - ' if info.get('simulated') else ''}run {info.get('run_dir')}, {_date(info)}, plan "
                         f"{plan.get('args', {}).get('plan', '?')}, {info.get('n_samples')} samples. Register 0x0008 = motor current "
                         f"(not 48 V supply current); total = |I_X|+|I_Y|."),
            "note": "% of rating = max(|I_X|/25 A, |I_Y|/18 A). Breakaway: the drive is not polled in the first 1 s after START (bench rule)."}
    page = DASH.replace("__DATA__", json.dumps(data, separators=(",", ":")).replace("</", "<\\/"))
    with open(os.path.join(run_dir, "current_dashboard.html"), "w") as f:
        f.write(page)


# ================================================================================== report text
def _case_txt(c, axis=None):
    if not c:
        return None
    # return (f"{c['itotal_a'] if axis == 'total' else abs(c['i' + axis + '_a']):.1f}\\,A at "
    #         f"X\\,=\\,{c['x_deg']:+.0f}\\textdegree, Y\\,=\\,{c['y_deg']:+.0f}\\textdegree{{}}")
    return (f"{c['itotal_a'] if axis == 'total' else abs(c['i' + axis + '_a']):.1f}\\,A at "
            f"X\\,=\\,{_dg(c['x_deg'])}\\textdegree, Y\\,=\\,{_dg(c['y_deg'])}\\textdegree{{}}")


def _dg(v):
    """Signed whole degrees, but a plain 0 (no '-0')."""
    return "0" if round(v) == 0 else f"{v:+.0f}"


def _motion_words(c):
    parts = []
    for k in ("x", "y"):
        m = c.get(f"{k}_motion")
        if m and m != "holding":
            parts.append(f"{k.upper()} {m}")
    return ", ".join(parts) if parts else "holding"


def _phase_word(p):
    return {"soak": "soak test", "1": "baseline", "2": "holding map", "3a": "X sweep", "3b": "Y sweep", "3c": "speed sweep",
            "3d": "diagonal move", "3e": "diagonal reach", "4a": "breakaway test", "4b": "reversal test", "4c": "thermal hold", "4d": "tracking replay",
            "4e": "clamp-meter check", "5": "STOP test", "6": "accel test",
            "end": "final return to HOME", "abort-home": "return to HOME after the abort",
            "0": "setup"}.get(str(p).replace("fast_", ""), f"phase {p}")


def _merge_note(plan):
    m = plan.get("merged_from")
    if not m:
        return ""
    ses = m.get("sessions") or [{"run": os.path.basename(m.get("run_a", "").rstrip("/")), "phases": m.get("phases_from_a", [])},
                                {"run": os.path.basename(m.get("run_b", "").rstrip("/")), "phases": ["the rest"]}]
    txt = "; ".join(f"{', '.join(_phase_word(q) for q in x['phases'] if q not in ('soak', 'end', 'abort-home'))} from "
                    f"\\texttt{{{_tex(x['run'])}}}" for x in ses)
    return f" The data combine {len(ses)} sessions on the same set-up: {txt}."


def report_texts(info, plan):
    sim = info.get("simulated")
    simtag = "\\textbf{[SIMULATED DATA -- pipeline test, not the gimbal]} " if sim else ""
    date = _date(info)
    a = plan.get("args", {})
    notes = plan.get("operator_notes", {})
    cw = notes.get("counterweight") or "not recorded"
    g = info.get("grid", {})
    rates = info.get("sample_rate", {})
    rc = a.get("rates", {})
    ups = set(info.get("_steps", []))
    n3a = len([s for s in ups if s.startswith("3a-") and s.endswith("-up")])
    n3b = len([s for s in ups if s.startswith("3b-") and s.endswith("-up")])
    n3c = len([s for s in ups if s.startswith("3c-") and s.endswith("-up")])
    n3d = len([s for s in ups if s.startswith("3d-") and s.endswith("-fwd")])
    pk = info.get("peak", {})
    hd = []
    for k, lab in (("x", "X"), ("y", "Y"), ("total", "total $|I_X|+|I_Y|$")):
        c = pk.get(k)
        if c:
            hd.append(f"peak {lab} current was {_case_txt(c, k)} ({_motion_words(c)}, during the {_phase_word(c['phase'])}), "
                      f"{c['pct_rating']:.0f}\\% of the drive rating")
        else:
            hd.append(f"peak {lab} current was not measured")
    fast = a.get("plan") == "fast"
    v = plan.get("speed_deg_s", 1.0)
    gridtxt = ("No holding map was measured (fast plan); the run consisted of "
               if fast or not g.get("n_measured") else
               f"The holding map covered {g.get('n_measured', 0)} grid points at "
               f"{plan.get('grid', {}).get('step_deg', '?')}\\textdegree{{}} spacing ({g.get('n_skipped', 0)} outside the collision limit were not visited), "
               f"followed by ")
    ndr = len(info.get("diag_reach") or {})
    dtxt = []
    if n3c:
        dtxt.append(f"{_n(n3c, 'repeat sweep')} at other speeds")
    if n3d:
        dtxt.append(_n(n3d, "two-axis diagonal"))
    if ndr:
        dtxt.append(f"{_n(ndr, 'diagonal reach')} (both axes together toward $(\\pm80,\\pm80)$\\textdegree{{}} at "
                    f"{a.get('diag_speed', 1.0) if isinstance(a.get('diag_speed'), (int, float)) else 1}\\textdegree/s, stopped by the operator)")
    sweeps = [_n(n3a, 'X sweep'), _n(n3b, 'Y sweep')] + dtxt
    cwtxt = (f" with the counterweight setup recorded as ``{_tex(cw)}''" if cw != "not recorded" else "")
    p1 = (simtag +
          f"The characterisation was run on {date} (plan \\texttt{{{a.get('plan')}}}, sweeps at {v:g}\\textdegree/s){cwtxt}. " + gridtxt +
          (", ".join(sweeps[:-1]) + " and " + sweeps[-1] if len(sweeps) > 1 else sweeps[0]) + ". Both axes were read at "
          f"{rc.get('moving_hz', '?')}\\,Hz while moving and {rc.get('hold_hz', '?')}\\,Hz while holding "
          f"(achieved medians {rates.get('moving_hz_median', '?')} and {rates.get('holding_hz_median', '?')}\\,Hz), "
          f"and {format(info.get('n_samples', 0), ',').replace(',', '{,}')} samples were logged in total." +
          " The " + "; the ".join(hd) + ". Register 0x0008 is motor current, not 48\\,V supply current.")
    if fast or not g.get("n_measured"):
        cap_map = (("SIMULATED: " if sim else "") +
                   f"{date}, measured along the X and Y sweep paths at {v:g}\\textdegree/s"
                   + (" and the four diagonal reaches at 1\\textdegree/s" if info.get("diag_reach") else "")
                   + "; no holding grid was run")
    else:
        cap_map = (("SIMULATED: " if sim else "") +
                   f"{date}, while holding, grid {plan.get('grid', {}).get('step_deg', '?')}\\textdegree, serpentine at 1\\textdegree/s "
                   f"(approach direction alternating), 2\\,s settle and 5\\,s sample per point; peak of each point")
    cap_move = (("SIMULATED: " if sim else "") +
                f"{date}, sweeps at {v:g}\\textdegree/s across $\\pm$80\\textdegree{{}}"
                + ("" if fast else " (X clipped to the collision limit)") + "; "
                "solid lifting towards HOME, dashed lowering")
    # comparison paragraph
    fx = (info.get("fit") or {}).get("x")
    fy = (info.get("fit") or {}).get("y")
    parts = [simtag.strip()] if sim else []
    if fx:
        parts.append(f"The X sweeps fit $I = c_0 + F\\,\\mathrm{{sign}}(v) - G_\\pm\\sin\\theta$ with "
                     f"$F = {abs(fx['F_A'][0]):.2f} \\pm {_se(fx['F_A'][1])}$\\,A, $G_+ = {abs(fx['G_pos_A'][0]):.2f} \\pm {_se(fx['G_pos_A'][1])}$\\,A and "
                     f"$G_- = {abs(fx['G_neg_A'][0]):.2f} \\pm {_se(fx['G_neg_A'][1])}$\\,A ($R^2 = {fx['r2']:.3f}$), against "
                     f"1.65--1.7\\,A, 7.6\\,A and 6.6\\,A in the 7 November X-only fit.")
    else:
        parts.append("The X model could not be fitted (no X sweeps were measured).")
    if fy:
        parts.append(f"For Y, $F = {abs(fy['F_A'][0]):.2f} \\pm {_se(fy['F_A'][1])}$\\,A and $G_\\pm = {abs(fy['G_pos_A'][0]):.2f}/{abs(fy['G_neg_A'][0]):.2f}$\\,A "
                     f"($R^2 = {fy['r2']:.3f}$).")
    cp = (fx or {}).get("coupling")
    if cp:
        if cp.get("significant"):
            parts.append(f"The Y position changes the X gravity term (coupling $C = {cp['C_A'][0]:+.2f} \\pm {_se(cp['C_A'][1])}$\\,A, "
                         f"fitted $|G_X|$ ranging over {fx.get('G_spread_A')}\\,A between the five Y lines), so the centre of gravity is not on the Y axis.")
        else:
            parts.append(f"No measurable coupling of the Y position into the X current was found ($C = {cp['C_A'][0]:+.2f} \\pm {_se(cp['C_A'][1])}$\\,A).")
    else:
        parts.append("Coupling between Y position and X current was not tested (fewer than two Y lines).")
    near = info.get("vs_software_stop", [])
    trips = info.get("current_stop_trips", [])
    if trips:
        parts.append(f"The software current stop tripped {len(trips)} time(s): " +
                     "; ".join(_tex(t.get("msg", "")) for t in trips[:3]) + ".")
    close = [n for n in near if n.get("within_20pct")]
    if close:
        parts.append("Within 20\\% of a software stop: " + ", ".join(f"{n['axis'].upper()} reached {n['peak_A']:.1f}\\,A of {n['soft_stop_A']:g}\\,A ({n['pct_of_stop']:.0f}\\%)" for n in close) + ".")
    elif near:
        parts.append("No case came within 20\\% of a software stop (highest: " +
                     ", ".join(f"{n['axis'].upper()} {n['pct_of_stop']:.0f}\\% of {n['soft_stop_A']:g}\\,A" for n in near) + ").")
    cm_ = [c for c in info.get("clamp_meter", []) if c.get("supply_a") is not None]
    if cm_:
        parts.append("Clamp-meter readings of the 48\\,V supply current: " +
                     "; ".join(f"{c['supply_a']:.1f}\\,A ({_tex(c.get('where', ''))})" for c in cm_) + ".")
    else:
        parts.append("The 48\\,V supply current was not measured with a clamp meter.")
    dr = info.get("diag_reach") or {}
    if dr:
        parts.append("Moving both axes together along the diagonals, the dish was stopped by the operator at " +
                     "; ".join(f"$({v['x_deg']:+.1f}, {v['y_deg']:+.1f})$\\textdegree" +
                               (f" ({_tex(v['how'])})" if v.get("how") != "operator STOP" else "") for v in dr.values()) +
                     ", the practical diagonal limits of the mount" +
                     (f"; the largest total current on the diagonals was {info['diag_peak']['itotal_a']:.1f}\\,A at "
                      f"X\\,=\\,{_dg(info['diag_peak']['x_deg'])}\\textdegree, Y\\,=\\,{_dg(info['diag_peak']['y_deg'])}\\textdegree{{}}, "
                      "on the way back to HOME with both axes lifting." if info.get("diag_peak") else "."))
        lost = [n for n, v in dr.items() if v.get("return_recorded") is False]
        if lost:
            parts.append(f"The return leg from {', '.join(lost)} was not recorded: both drives stopped answering on the CAN bus "
                         "at the start of that leg and the session was aborted; the reach angle itself was logged before it.")
    p2 = " ".join(parts)
    # appendix C
    ph_run = [e for e in plan.get("phases_selected", [])]
    not_run = plan.get("phases_not_run", {})
    titles = {"soak": "2-minute soak test (slow X and Y moves at the chosen poll rate)", "1": "baseline at HOME, 60\\,s",
              "2": "holding map", "3a": "X sweeps at Y $\\in\\{-60,-30,0,+30,+60\\}$\\textdegree",
              "3b": "Y sweeps at X $\\in\\{-60,-30,0,+30,+60\\}$\\textdegree", "3c": "worst sweeps repeated at 0.5 and 1.5\\textdegree/s",
              "3d": "diagonals $(+60,+60)\\leftrightarrow(-60,-60)$ and $(+60,-60)\\leftrightarrow(-60,+60)$",
              "3e": "diagonal reach: straight diagonals toward $(\\pm80,\\pm80)$\\textdegree{} past the CAD limit, stopped by the operator; "
                    "the angle at the STOP key is taken as that diagonal's limit",
              "4a": "breakaway: 30\\,s hold then a 5\\textdegree{} lift at the worst holding pose, per axis",
              "4b": "reversal at X\\,=\\,$\\pm$80\\textdegree{} and Y\\,=\\,$\\pm$80\\textdegree",
              "4c": f"thermal hold at the worst pose, {plan.get('thermal_hold_s', 0) / 60:.0f}\\,min",
              "4d": "DS-EO 11 November pass replayed through the track mode", "4e": "clamp-meter readings",
              "5": "STOP (0x84) mid-move", "6": "X sweep at Y\\,=\\,0 with accel/decel 0x0B61"}
    lim = plan.get("abort_limits", {})
    evs = A.load_events(info.get("_run_path", "")) if info.get("_run_path") else []
    ended = {e.get("phase") for e in evs if e.get("type") == "phase_end"}
    started = {e.get("phase") for e in evs if e.get("type") == "phase_start"}
    items = []
    for p in ["soak"] + ph_run:
        st = "" if p in ended else (" (started, not completed: run aborted)" if p in started else " (not reached: run aborted)")
        items.append(f"\\item Phase {p}: {titles.get(p, p)}" + st)
    for p, why in not_run.items():
        items.append(f"\\item Phase {p}: not run ({_tex(why)})")
    appx = (simtag + "\n"
            f"\\noindent Run \\texttt{{{_tex(info.get('run_dir', ''))}}}, {date}, command \\texttt{{{_tex(plan.get('command', ''))}}}. "
            f"Grid {plan.get('grid', {}).get('step_deg')}\\textdegree{{}} from $-$80 to $+$80\\textdegree{{}} on both axes, serpentine order at "
            f"1\\textdegree/s, 2\\,s settle and 5\\,s sample per point; points outside the CAD collision limit minus 3\\textdegree{{}} "
            f"were skipped, and every path (not only its end point) was kept inside that limit, moving Y towards 0 first between "
            f"high-$|X|$ poses. Sweeps at 1\\textdegree/s; accel/decel register {plan.get('accel_reg', {}).get('proven', '0x0419')}. "
            f"Sampling: both axes {rc.get('moving_hz')}\\,Hz moving, {rc.get('hold_hz')}\\,Hz holding, {rc.get('fast_hz')}\\,Hz on the moving "
            f"axis for breakaway, 0.1\\,Hz in the thermal hold; at least {a.get('min_gap_s', 0.03) * 1000:.0f}\\,ms between requests, "
            f"no polling for {a.get('quiet_s', 1.0):g}\\,s after an enable or START "
            f"({a.get('quiet_track_s', 0.3):g}\\,s after a tracking step on a holding motor); temperature and bus voltage every 10\\,s. "
            f"Abort limits: Ctrl+C, a drive in state 0 that is still off 1\\,s later, drive temperature above {lim.get('temp_max_c'):g}\\,\\textdegree C "
            f"or a rise of more than {lim.get('temp_rise_c'):g}\\,\\textdegree C, bus voltage below {lim.get('min_bus_v'):g}\\,V, or loss of the relay link. "
            f"Software current stops X 15\\,A and Y 11\\,A were kept; a trip stops the axis, is logged as a worst case and the step is skipped."
            + (f" The run was aborted: {_tex(info['aborted'])}." if info.get("aborted") else "")
            + _merge_note(plan) +
            "\n\\begin{itemize}\n" + "\n".join(items) + "\n\\end{itemize}\n"
            "\\noindent Data files (\\texttt{data/current\\_char/}): \\texttt{plan.json} (parameters, HOME counts, configuration, "
            "operator notes), \\texttt{samples.csv} (every reading; 1 in 10 here as \\texttt{samples\\_thinned.csv}), "
            "\\texttt{points.csv} (one row per holding point), \\texttt{events.jsonl} (phases, trips, aborts, operator inputs) and "
            "\\texttt{summary.json} (worst cases and model fits).")
    trace_b = ("reversal at the $\\pm$80\\textdegree{} extremes" if info.get("reversal") or not info.get("diag_reach")
               else "total current on the diagonal returns to HOME, both axes lifting (no reversal test in this plan)")
    return {"p1": p1, "cap_map": cap_map, "cap_move": cap_move, "p2": p2, "appx": appx, "trace_b": trace_b}


def _dur(sec):
    return f"{sec / 60:.0f} min" if sec >= 120 else f"{sec:.0f} s"


def _n(n, word):
    return f"{n} {word}{'' if n == 1 else ('es' if word.endswith(('ch', 'sh', 's', 'x')) else 's')}"


def _se(v):
    return "?" if v is None else (f"{v:.2f}" if v >= 0.01 else f"{v:.1g}")


def _tex(s):
    s = str(s)
    for a, b in (("\\", "\\textbackslash{}"), ("&", "\\&"), ("%", "\\%"), ("$", "\\$"), ("#", "\\#"), ("_", "\\_"),
                 ("{", "\\{"), ("}", "\\}"), ("~", "\\textasciitilde{}"), ("^", "\\textasciicircum{}"),
                 ("\u00b0", "\\textdegree{}"), ("\u2212", "-")):
        s = s.replace(a, b)
    return s


PH = {
    "p1": "\\fillme{One paragraph from data/current\\_char/<run>/summary.json: date, counterweight configuration, number of holding points and sweeps, sample rate, total samples logged. Then state the three headline numbers: peak X current, peak Y current and peak total current, each with the pose and direction where it occurred and its percentage of the drive rating.}",
    "cap_map": "\\fillme{caption with date and conditions}",
    "cap_move": "\\fillme{caption}",
    "p2": "\\fillme{One paragraph: compare with the 7 Nov X-only model, say whether Y position changes X current (centre-of-gravity coupling), and whether any case came within 20\\% of a software stop.}",
    "appx": "\\fillme{Claude Code: insert the procedure actually run (grid, speeds, sweeps, worst-case tests, abort limits, sample rate), the data-file list and the command used, from data/current\\_char/<run>/plan.json.}",
}
TRACE_FIG = ("\n\\begin{figure}[!htbp]\\centering\n\\includegraphics[width=\\linewidth]{figures/current_worst_trace.pdf}\n"
             "\\caption{Worst-case traces: (a) breakaway from rest, (b) %s, "
             "(c) long hold with drive temperature%s.}\\label{fig:curtrace}\n\\end{figure}\n")


def fill_report(src_tex, out_tex, texts, add_trace=False, sim=False):
    with open(src_tex) as f:
        s = f.read()
    missing = [k for k, v in PH.items() if s.count(v) != 1]
    if missing:
        raise SystemExit(f"placeholders not found exactly once in {src_tex}: {missing}")
    for k, v in PH.items():
        new = texts[k]
        if k in ("cap_map", "cap_move"):
            new = "(" + new + ")"
        s = s.replace(v, new)
    if add_trace:
        anchor = "\\subsection{Range of motion}"
        # s = s.replace(anchor, (TRACE_FIG % (" -- SIMULATED" if sim else "")) + "\n" + anchor, 1)
        s = s.replace(anchor, (TRACE_FIG % (texts.get("trace_b", "reversal at the $\\pm$80\\textdegree{} extremes"),
                                            " -- SIMULATED" if sim else "")) + "\n" + anchor, 1)
    with open(out_tex, "w") as f:
        f.write(s)


# ================================================================================== main
def thin_samples(src, dst, every=10):
    with open(src, newline="") as fi, open(dst, "w", newline="") as fo:
        r, w = csv.reader(fi), csv.writer(fo)
        w.writerow(next(r))
        for i, row in enumerate(r):
            if i % every == 0:
                w.writerow(row)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Figures, LaTeX table and dashboard from a current_char.py run")
    ap.add_argument("run_dir")
    ap.add_argument("--report-dir", default=None, help="UREx report folder (figures/ and data/ are updated)")
    ap.add_argument("--fill-report", metavar="TEX", help="write the filled Section 4.3 / Appendix C text into this .tex "
                                                         "(read from <report-dir>/report.tex)")
    ap.add_argument("--add-trace-figure", action="store_true", help="with --fill-report: also add current_worst_trace.pdf to Section 4.3")
    a = ap.parse_args(argv)
    run = os.path.abspath(a.run_dir)
    plan = A.load_plan(run)
    info = A.write_summary(run, aborted=_prev_abort(run))
    info["_run_path"] = run
    info["_steps"] = sorted(set(A.load_samples(run)["step_id"]))
    info["quiet_s"] = plan.get("args", {}).get("quiet_s", 1.0)
    sim = bool(info.get("simulated"))
    S = A.load_samples(run)
    P = A.load_points(run)
    plt = mpl()
    info["_S"] = S
    fig_map_total(plt, run, P, sim, info)
    info["_S"] = S
    fig_map_axes(plt, run, P, sim, S)
    fig_vs_angle(plt, run, S, sim, info)
    fig_worst_trace(plt, run, S, sim, info)
    fig_model(plt, run, S, sim, info)
    dashboard(run, S, P, info, plan)
    tex = summary_tex(info, plan)
    with open(os.path.join(run, "current_summary.tex"), "w") as f:
        f.write(tex)
    texts = report_texts(info, plan)
    with open(os.path.join(run, "report_fill.md"), "w") as f:
        f.write(f"# Report text drafted from {info.get('run_dir')} ({'SIMULATED' if sim else 'HARDWARE'})\n\n")
        for k, v in texts.items():
            f.write(f"## {k}\n\n{v}\n\n")
    print(f"wrote {', '.join(n + '.pdf/.png' for n in FIGS)}, current_dashboard.html, current_summary.tex, report_fill.md in {run}")
    if a.report_dir:
        rd = os.path.abspath(a.report_dir)
        os.makedirs(os.path.join(rd, "figures"), exist_ok=True)
        dd = os.path.join(rd, "data", "current_char")
        os.makedirs(dd, exist_ok=True)
        for n in FIGS:
            shutil.copy2(os.path.join(run, n + ".pdf"), os.path.join(rd, "figures", n + ".pdf"))
        shutil.copy2(os.path.join(run, "current_summary.tex"), os.path.join(rd, "data", "current_summary.tex"))
        for n in ("summary.json", "points.csv", "plan.json", "events.jsonl"):
            if os.path.exists(os.path.join(run, n)):
                shutil.copy2(os.path.join(run, n), os.path.join(dd, n))
        thin_samples(os.path.join(run, "samples.csv"), os.path.join(dd, "samples_thinned.csv"))
        size = sum(os.path.getsize(os.path.join(dp, f)) for dp, _, fs in os.walk(rd) for f in fs)
        print(f"copied figures/current_*.pdf, data/current_summary.tex, data/current_char/* into {rd} "
              f"(report folder now {size / 1e6:.2f} MB{' - OVER 5 MB!' if size > 5e6 else ''})")
        if a.fill_report:
            fill_report(os.path.join(rd, "report.tex"), os.path.abspath(a.fill_report), texts, a.add_trace_figure, sim)
            print(f"filled placeholders written to {a.fill_report}")
    return 0


def _prev_abort(run):
    try:
        with open(os.path.join(run, "summary.json")) as f:
            return json.load(f).get("aborted")
    except (OSError, ValueError):
        return None


if __name__ == "__main__":
    sys.exit(main())
