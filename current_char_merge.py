#!/usr/bin/env python3
"""Merge two current-characterisation runs into one run folder for current_char_report.py.

For when a run aborted part-way and a later run did the rest. On 14 Nov 2026 the X sweep (3a) came from
run A (which aborted in 3b when a drive dropped out) and everything else from run B.

    python3 current_char_merge.py data/current_char/<A> data/current_char/<B> --from-a 3a
    -> data/current_char/<B>_merged/   (then: python3 current_char_report.py data/current_char/<B>_merged ...)

Rows of A whose phase is in --from-a are kept, plus all of B. B's times are shifted to follow A's so
t_rel_s keeps increasing. The originals are untouched. plan.json is B's with "merged_from" added.
"""
import argparse
import csv
import json
import os
import sys

import current_char_analysis as A


def _rows(path):
    with open(path, newline="") as f:
        r = csv.reader(f)
        head = next(r)
        return head, [row for row in r if row]


def _events(path):
    out = []
    try:
        with open(path) as f:
            for line in f:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    pass                                  # torn last line of a crashed run
    except OSError:
        pass
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("run_a", help="earlier run (supplies the --from-a phases)")
    ap.add_argument("run_b", help="later run (supplies everything else)")
    ap.add_argument("--from-a", default="3a", help="comma list of phases taken from run A (3a)")
    ap.add_argument("-o", "--out", default=None, help="output folder (default: <run_b>_merged)")
    a = ap.parse_args(argv)
    take = {p.strip() for p in a.from_a.split(",") if p.strip()}
    out = a.out or a.run_b.rstrip("/") + "_merged"
    os.makedirs(out, exist_ok=False)

    ha, ra = _rows(os.path.join(a.run_a, "samples.csv"))
    hb, rb = _rows(os.path.join(a.run_b, "samples.csv"))
    if ha != hb:
        raise SystemExit("samples.csv columns differ between the two runs")
    ip, it = ha.index("phase"), ha.index("t_rel_s")
    keep_a = [r for r in ra if r[ip] in take]
    if not keep_a:
        raise SystemExit(f"run A has no samples in phases {sorted(take)}")
    t_end_a = max(float(r[it]) for r in keep_a)
    t0_b = min((float(r[it]) for r in rb), default=0.0)
    shift = t_end_a + 60.0 - t0_b                         # B starts 60 s after A's last kept sample
    for r in rb:
        r[it] = f"{float(r[it]) + shift:.3f}"
    with open(os.path.join(out, "samples.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(ha)
        w.writerows(keep_a + rb)

    # ev = [e for e in _events(os.path.join(a.run_a, "events.jsonl")) if e.get("phase") in take]
    ev = []
    for e in _events(os.path.join(a.run_a, "events.jsonl")):
        if e.get("phase") in take:
            ev.append(e)
        elif e.get("type") in ("abort", "session_abort"):    # how run A's kept phases ended (e.g. a lost return leg)
            ev.append(dict(e, type="session_abort"))
    for e in _events(os.path.join(a.run_b, "events.jsonl")):
        if isinstance(e.get("t_rel_s"), (int, float)):
            e["t_rel_s"] = round(e["t_rel_s"] + shift, 2)
        ev.append(e)
    with open(os.path.join(out, "events.jsonl"), "w") as f:
        for e in ev:
            f.write(json.dumps(e) + "\n")

    pb = os.path.join(a.run_b, "points.csv")
    if os.path.exists(pb):
        with open(pb) as src, open(os.path.join(out, "points.csv"), "w") as dst:
            dst.write(src.read())

    plan_a, plan_b = A.load_plan(a.run_a), A.load_plan(a.run_b)
    plan = dict(plan_b)
    sel_b = list(plan_b.get("phases_selected", []))
    sel = [p for p in plan_a.get("phases_selected", []) if p in take] + [p for p in sel_b if p not in take]
    order = ["soak", "1", "2", "3a", "3b", "3c", "3d", "4a", "4b", "4c", "4d", "4e", "5", "6"]
    plan["phases_selected"] = sorted(dict.fromkeys(sel), key=lambda p: order.index(p) if p in order else 99)
    plan["phases_not_run"] = {k: v for k, v in plan_b.get("phases_not_run", {}).items() if k not in take}
    def _phases_in(rows):
        return sorted({r[ip] for r in rows})
    prev_sessions = (plan_a.get("merged_from") or {}).get("sessions")
    if prev_sessions:                                    # run A is itself a merge: keep its per-session split
        sessions = [dict(x, phases=[q for q in x["phases"] if q in take]) for x in prev_sessions]
        sessions = [x for x in sessions if x["phases"]]
    else:
        sessions = [{"run": os.path.basename(os.path.normpath(a.run_a)), "phases": _phases_in(keep_a)}]
    sessions.append({"run": os.path.basename(os.path.normpath(a.run_b)), "phases": _phases_in(rb)})
    plan["merged_from"] = {"sessions": sessions, "run_a": os.path.abspath(a.run_a), "phases_from_a": sorted(take),
                           "run_a_started_utc": plan_a.get("started_utc"),
                           "run_b": os.path.abspath(a.run_b), "run_b_started_utc": plan_b.get("started_utc"),
                           "time_shift_b_s": round(shift, 3)}
    with open(os.path.join(out, "plan.json"), "w") as f:
        json.dump(plan, f, indent=2)

    try:
        with open(os.path.join(a.run_b, "summary.json")) as f:
            aborted_b = json.load(f).get("aborted")
    except (OSError, ValueError):
        aborted_b = None
    A.write_summary(out, aborted=aborted_b)
    print(f"merged {len(keep_a)} samples of {sorted(take)} from {a.run_a} + {len(rb)} samples from {a.run_b}\n-> {out}\n"
          f"next: python3 current_char_report.py {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
