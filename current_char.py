#!/usr/bin/env python3
"""
current_char.py - current characterisation of the X-Y gimbal (motor current over the whole X and Y range,
moving and holding, with the worst cases). Headless: it starts the SAME Gimbal thread as app.py (the only
CAN sender) and gives it goto / track targets, like test_job.py and sun_job.py do.

    python3 current_char.py --sim --plan quick                      # simulator, 20x time (Phase 0: dry run)
    python3 current_char.py --pi <ip> --plan quick                  # hardware (Pi relay), ~1.5-2 h
    python3 current_char.py --pi <ip> --plan full --clamp-meter     # ~4 h, asks for DC clamp readings
    options: --phases 1,2,3,4,5   --allow-stop-test (phase 5)   --allow-accel-test (phase 6)

Before anything moves it prints the plan (phases, poses, angle range, estimated time, abort limits), asks
y/n for each phase, asks for operator notes, and waits for you to type RUN.

Data: data/current_char/<YYYYmmdd_HHMMSS>/  plan.json  samples.csv  points.csv  events.jsonl  summary.json
(never overwritten; every line flushed and fsync'ed). Then: python3 current_char_report.py <run_dir>.

IMPORTANT: register 0x0008 is MOTOR current, not the 48 V supply current. "Total" = |I_X| + |I_Y|.
Supply current is only known from the clamp-meter readings (--clamp-meter).

Bench rules followed (config.py / DEPLOY.md): only the Gimbal thread sends CAN; never enable without moving
(Start = wake moves); >= 1 s between motor-off/setup and enable (Gimbal); standalone moves >= 0.5 deg; gentle
polling (>= 30 ms between requests, nothing in the first 1 s after an enable or START, both axes read
together at 2 Hz moving / 1 Hz holding, state at most 1 Hz, temperature + bus voltage every 10 s); accel
0x0419 unless Phase 6; never writes 0x0050/0x0052/0x0053/0x00E8 (zeroerr_protocol refuses them);
soft limits +/-80 deg; CAD collision limit minus 3 deg on every PATH (not just end points); software
current stops X 15 A / Y 11 A kept (a trip = STOP, logged as a worst case, step skipped, run continues).
"""
import argparse
import csv
import json
import math
import os
import queue
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone

import config
from current_char_analysis import (POINT_COLS, RATED_A, SAMPLE_COLS, SOFT_STOP_A, cad_max_abs_x, cad_max_abs_y,
                                   cad_ok, write_summary)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_ROOT = os.path.join(HERE, "data", "current_char")
PROFILE_CSV = os.path.join(HERE, "urex_report", "data", "dseo_pass_reconstruction_2026-11-11.csv")
PROVEN_ACCEL = config.MANUAL_ACCEL_COUNTS          # 0x0419
TEST_ACCEL = 0x0B61                                # 2 deg/s^2 if the unit is counts/s^2
CAD_MARGIN = 3.0
SOFT = 80.0
MIN_MOVE = config.MIN_MOVE_DEG                     # 0.5 deg
# POS_TOL = 0.3                                   # a drive can settle a hair past its target (+80.004 on 14 Nov)
POS_TOL = 5.0                                     # operator choice 14 Nov: a measured angle up to 85 deg counts as AT 80 (no abort);
                                                  # targets stay within +/-80, the operator Ctrl+C's if it looks close


def clamp_soft(v, soft=None):
    """Treat a measured angle up to POS_TOL past the soft limit as AT the limit (encoder settle), so the
    planner and the CAD check do not see 'outside' for a dish that stopped 0.004 deg beyond +80."""
    soft = SOFT if soft is None else soft
    if soft < abs(v) <= soft + POS_TOL:
        return math.copysign(soft, v)
    return v

PHASES = [
    ("soak", "Soak test (mandatory first hardware phase): slow X and Y moves while polling at the chosen rate, 2 min"),
    ("1", "Baseline: HOME, both axes holding, 60 s"),
    ("2", "Holding map: X x Y grid, serpentine at 1 deg/s, settle 2 s + sample 5 s per cell"),
    ("3a", "X sweeps -80 -> +80 -> -80 at 1 deg/s for Y in {-60,-30,0,+30,+60} (X clipped to the CAD limit)"),
    ("3b", "Y sweeps -80 -> +80 -> -80 at 1 deg/s for X in {-60,-30,0,+30,+60}"),
    ("3c", "Speed: worst X sweep (from 3a) and worst Y sweep (from 3b) again at 0.5 and 1.5 deg/s"),
    ("3d", "Both axes together: diagonals (+60,+60)<->(-60,-60) and (+60,-60)<->(-60,+60) at 1 deg/s"),
    ("3e", "Diagonal reach: aim for (+80,+80) -> (-80,-80) -> HOME -> (+80,-80) -> (-80,+80) -> HOME; Enter/Space = STOP, "
           "that angle is the diagonal's max  [GOES PAST THE CAD LIMIT: operator-stopped]"),
    ("4a", "Breakaway: worst holding pose, hold 30 s, then 5 deg lifting toward HOME (each axis), fast sampling"),
    ("4b", "Reversal at the extremes: X at +80/-80 (worst allowed Y), Y at +80/-80 (worst allowed X)"),
    ("4c", "Thermal hold at the worst pose (full 10 min, quick 5 min), current + temperature every 10 s"),
    ("4d", "Tracking profile: DS-EO 11 Nov pass replayed through the real track mode (clipped to the limits)"),
    ("4e", "Clamp meter: 48 V supply current at HOME, at the worst pose and during the 3a sweep at Y = 0"),
    ("5", "STOP (0x84) mid-move at the worst pose: current spike + stopping distance  [NEVER TESTED ON THE LOAD]"),
    ("6", "3a at Y = 0 again with accel/decel 0x0B61 (2 deg/s^2), then 0x0419 restored"),
]
PHASE_TITLES = dict(PHASES)
FAST_TITLES = {
    "1": "Baseline: HOME, both axes holding, 10 s",
    "3a": "X sweep at Y = 0, 1.5 deg/s: 0 -> +80 -> -80 -> 0",
    "3b": "Y sweep at X = 0, 1.5 deg/s: 0 -> +80 -> -80 -> 0",
    "3d": "Both axes together: (+50,+50) -> (-50,-50) -> HOME at 1.5 deg/s",
    "4a": "Breakaway at the worst pose (+/-60, +/-60 from the sweeps): hold 5 s, lift 5 deg, each axis",
    "4c": "Hold 30 s at the worst pose (current + temperature)",
}
DIAG_LEGS = [  # (target, step, records a reach for this diagonal) - every leg is a straight diagonal (equal |dX|, |dY|)
    ((80.0, 80.0), "3e-pp-out", "+X+Y"),
    ((-80.0, -80.0), "3e-mm-out", "-X-Y"),
    ((0.0, 0.0), "3e-mm-back", None),
    ((80.0, -80.0), "3e-pm-out", "+X-Y"),
    ((-80.0, 80.0), "3e-mp-out", "-X+Y"),
    ((0.0, 0.0), "3e-mp-back", None),
]


DIAG_TARGETS = {"+X+Y": (80.0, 80.0, "pp"), "-X-Y": (-80.0, -80.0, "mm"), "+X-Y": (80.0, -80.0, "pm"), "-X+Y": (-80.0, 80.0, "mp")}


def diag_legs(spec):
    """--diag: None = the full sequence (DIAG_LEGS); else a list like '-X-Y,+X-Y' = each one HOME -> out -> HOME."""
    if not spec:
        return DIAG_LEGS
    legs = []
    for name in [t.strip().upper() for t in spec.split(",") if t.strip()]:
        if name not in DIAG_TARGETS:
            raise SystemExit(f"--diag: unknown diagonal {name!r} (use +X+Y, -X-Y, +X-Y, -X+Y)")
        x, y, code = DIAG_TARGETS[name]
        legs += [((x, y), f"3e-{code}-out", name), ((0.0, 0.0), f"3e-{code}-back", None)]
    return legs


class KeyWatch:
    """Enter / Space / s on the terminal = operator STOP during the diagonal reach (cbreak mode, Ctrl+C still works).
    Only on a real terminal; with --sim, --sim-diag-stop stands in for the key."""

    def __init__(self):
        self.on = sys.stdin.isatty()
        self.old = None

    def __enter__(self):
        if self.on:
            import termios
            import tty
            self.old = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            termios.tcflush(sys.stdin, termios.TCIFLUSH)      # a key typed before the leg does not count
        return self

    def pressed(self):
        if not self.on:
            return False
        import select
        hit = False
        while select.select([sys.stdin], [], [], 0)[0]:
            c = sys.stdin.read(1)
            if c in ("\n", "\r", " ", "s", "S"):
                hit = True
        return hit

    def __exit__(self, *exc):
        if self.on and self.old is not None:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old)
        return False


def title_of(pid, plan):
    return FAST_TITLES.get(pid, PHASE_TITLES[pid]) if plan == "fast" else PHASE_TITLES[pid]


class Abort(Exception):
    pass


class Dropout(Abort):
    pass


def sgn(v):
    return (v > 0) - (v < 0)


# ================================================================================== simulator time
class FastClock:
    """--sim only: everything that uses time.monotonic / time.sleep / clock.now runs `scale` x faster
    (gimbal thread, simulated drives, this runner). Never installed for hardware."""

    def __init__(self, scale):
        import clock
        self.scale = float(scale)
        self._mono, self._sleep = time.monotonic, time.sleep
        self.r0 = self._mono()
        self.utc0 = datetime.now(timezone.utc)
        time.monotonic = self.monotonic
        time.sleep = self.sleep
        clock.now = self.now

    def monotonic(self):
        return self.r0 + (self._mono() - self.r0) * self.scale

    def sleep(self, s):
        self._sleep(max(0.0, s) / self.scale)

    def now(self):
        from datetime import timedelta
        return self.utc0 + timedelta(seconds=self.monotonic() - self.r0)


# ================================================================================== path planning
def plan_path(p0, p1, margin=CAD_MARGIN):
    """Waypoints from p0 to p1 whose whole PATH stays inside the soft limits and the CAD limit (minus margin).
    The gimbal moves both axes at once with independent trapezoids, so a segment is safe when its whole
    bounding box is safe: |Y| <= cad(max |X| over the segment). Otherwise: bring Y toward 0 first (just
    enough), then move X, then Y."""
    # x0, y0 = p0
    x0, y0 = clamp_soft(p0[0]), clamp_soft(p0[1])
    x1, y1 = p1
    if not cad_ok(x1, y1, margin, SOFT):
        raise ValueError(f"target X {x1:+.1f}, Y {y1:+.1f} is outside the soft/CAD limit (margin {margin:g} deg)")
    mx = max(abs(x0), abs(x1))
    if max(abs(y0), abs(y1)) <= cad_max_abs_y(mx, margin) + 1e-9:
        return [(x1, y1)]
    lim = cad_max_abs_y(mx, margin)
    if lim < 0:
        raise ValueError(f"no safe Y for |X| = {mx:.2f} (start X {x0:+.2f}): outside the CAD table")
    ym = max(-lim, min(lim, y0))
    pts = []
    if abs(ym - y0) > 1e-9:
        pts.append((x0, ym))
    pts.append((x1, ym))
    if abs(y1 - ym) > 1e-9:
        pts.append((x1, y1))
    return pts


def seed_sweep_peaks(R, run_dir):
    """Fill R.sweep_peak from an earlier run's samples.csv (sweep steps 3a-/3b- only, peak |I| while that axis moved),
    for the steps this run does not repeat."""
    import csv
    seeded = set()
    toks = [t.strip() for t in str(R.args.phases or "").split(",")]
    rerun = {t for t in ("3a", "3b") if t in toks or "3" in toks}
    with open(os.path.join(run_dir, "samples.csv"), newline="") as f:
        for row in csv.DictReader(f):
            st = row.get("step_id") or ""
            if not st.startswith(("3a-", "3b-")) or st in R.sweep_peak:
                continue
            if st[:2] in rerun:                      # this run repeats that sweep: use only the new data
                continue
            sp = R._prev_peak.setdefault(st, {"x": 0.0, "y": 0.0})
            for k in ("x", "y"):
                try:
                    v = abs(float(row[f"i{k}_a"]))
                except (TypeError, ValueError, KeyError):
                    continue
                if k in (row.get("moving_axes") or ""):
                    sp[k] = max(sp[k], v)
            seeded.add(st)
    for st in seeded:
        R.sweep_peak.setdefault(st, R._prev_peak[st])
    return seeded


def move_duration(d, v, a):
    d = abs(d)
    if d <= 0:
        return 0.0
    if d >= v * v / a:
        return d / v + v / a
    return 2 * math.sqrt(d / a)


def counts_to_deg(c):
    return c * 360.0 / config.COUNTS_PER_REV


# ================================================================================== relay / process checks
def _tcp_established_to(ip: str, port: int) -> list[str]:
    """Local ESTABLISHED TCP connections to ip:port (from /proc/net/tcp*). [] if unknown."""
    hits = []
    try:
        packed = socket.inet_aton(ip)
        want = "".join(f"{b:02X}" for b in packed[::-1])
    except OSError:
        return hits
    for fn in ("/proc/net/tcp", "/proc/net/tcp6"):
        try:
            with open(fn) as f:
                next(f)
                for line in f:
                    parts = line.split()
                    rem, st = parts[2], parts[3]
                    if st != "01":                      # ESTABLISHED
                        continue
                    rip, rport = rem.split(":")
                    if int(rport, 16) == port and rip[-8:].upper() == want:
                        hits.append(f"{parts[1]} -> {ip}:{port}")
        except OSError:
            pass
    return hits


def _other_gs_processes() -> list[str]:
    out = []
    me = os.getpid()
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == me:
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
        except OSError:
            continue
        if any(s in cmd for s in ("app.py", "current_char.py", "track_satellite.py", "track_sun.py")) and "python" in cmd:
            out.append(f"pid {pid}: {cmd[:120]}")
    return out


def relay_preflight(pi: str):
    """Refuse to start if another client holds the relay. The relay REPLACES an existing connection from the
    same IP (and STOPs the drives), so this machine is checked BEFORE connecting; another machine is reported
    by the relay itself ('busy') after connecting."""
    host, _, port = pi.partition(":")
    port = int(port or config.RELAY_PORT)
    try:
        ip = socket.gethostbyname(host)
    except OSError as e:
        raise SystemExit(f"cannot resolve the Pi {host!r}: {e}")
    procs = _other_gs_processes()
    conns = _tcp_established_to(ip, port)
    if procs or conns:
        msg = ["REFUSED: another client already holds (or may hold) the CAN relay."]
        msg += [f"  process  {p}" for p in procs] + [f"  socket   {c}" for c in conns]
        msg.append("Close app.py / the other tool first (the relay accepts ONE client, and connecting from this "
                   "laptop would kick it off and STOP the drives).")
        raise SystemExit("\n".join(msg))


def check_relay_busy(link, wait_s=2.0):
    end = time.monotonic() + wait_s
    while time.monotonic() < end:
        errs = [e.get("msg", "") for e in list(getattr(link, "relay_errors", []))]
        bad = [m for m in errs if "busy" in m or "not allowed" in m]
        if bad:
            raise SystemExit(f"REFUSED by the relay: {bad[-1]} (another client holds it)")
        if link.connected and link.hello:
            return
        time.sleep(0.05)
    if not link.connected:
        raise SystemExit(f"cannot reach the relay: {link.last_error}")


# ================================================================================== logging
class RunLog:
    def __init__(self, run_dir, t0):
        self.dir, self.t0 = run_dir, t0
        self._sf = open(os.path.join(run_dir, "samples.csv"), "x", newline="")
        self._sw = csv.writer(self._sf)
        self._sw.writerow(SAMPLE_COLS)
        self._pf = open(os.path.join(run_dir, "points.csv"), "x", newline="")
        self._pw = csv.writer(self._pf)
        self._pw.writerow(POINT_COLS)
        self._ef = open(os.path.join(run_dir, "events.jsonl"), "x")
        for f in (self._sf, self._pf, self._ef):
            self._sync(f)
        self.n_samples = 0
        self.lock = threading.Lock()

    @staticmethod
    def _sync(f):
        f.flush()
        try:
            os.fsync(f.fileno())
        except OSError:
            pass

    def sample(self, row: list):
        with self.lock:
            self._sw.writerow(row)
            self._sync(self._sf)
            self.n_samples += 1

    def point(self, d: dict):
        with self.lock:
            self._pw.writerow([d.get(c, "") for c in POINT_COLS])
            self._sync(self._pf)

    def event(self, typ, echo=True, **kw):
        import clock
        rec = {"t_utc": clock.now().isoformat(timespec="milliseconds"),
               "t_rel_s": round(time.monotonic() - self.t0, 2), "type": typ}
        rec.update(kw)
        with self.lock:
            self._ef.write(json.dumps(rec, default=str) + "\n")
            self._sync(self._ef)
        if echo:
            msg = kw.get("msg") or ", ".join(f"{k}={v}" for k, v in kw.items())
            print(f"  [{time.strftime('%H:%M:%S')}] {typ}: {msg}", flush=True)

    def close(self):
        for f in (self._sf, self._pf, self._ef):
            try:
                self._sync(f)
                f.close()
            except (OSError, ValueError):
                pass


def write_json(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=2, default=str)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def fmt_s(s):
    s = int(round(s))
    return f"{s // 3600}h{(s % 3600) // 60:02d}m" if s >= 3600 else f"{s // 60}m{s % 60:02d}s"


def ask(prompt, default=""):
    try:
        v = input(prompt)
    except EOFError:
        return default
    return v.strip()


# ================================================================================== rigs
class DryRig:
    """Plan estimator: same calls as LiveRig, nothing moves; adds up the time."""
    dry = True

    def __init__(self, speed=1.0, accel_counts=PROVEN_ACCEL, pose=(0.0, 0.0)):
        self.t = 0.0
        self.pose = list(pose)
        self.speed = speed
        self.accel = accel_counts
        self.moves = 0
        self.prompts = 0
        self.points = []                     # (x, y, stats) never filled: data-dependent choices use defaults
        self.sweep_peak = {}
        self.res = {}

    def now(self):
        return self.t

    def event(self, *a, **k):
        pass

    def phase(self, pid):
        pass

    def set_speed(self, v):
        self.speed = v

    def set_accel(self, c):
        self.accel = c
        self.t += 8.0                         # setup + 1 s + wake moves

    def restart_motors(self, why=""):
        self.t += 8.0

    def move(self, x, y, step, collect=None, fast_axes=None, note=""):
        acc = counts_to_deg(self.accel)
        for wx, wy in plan_path(tuple(self.pose), (x, y)):
            d = max(move_duration(wx - self.pose[0], self.speed, acc), move_duration(wy - self.pose[1], self.speed, acc))
            if d > 0:
                self.t += d + 1.2
                self.moves += 1
            self.pose = [wx, wy]
        return "ok"

    def reach(self, tx, ty, step, watch=True):
        acc = counts_to_deg(self.accel)
        d = max(move_duration(tx - self.pose[0], self.speed, acc), move_duration(ty - self.pose[1], self.speed, acc))
        self.t += d + 1.2
        self.moves += 1
        self.pose = [tx, ty]
        return {"result": "arrived", "x_deg": tx, "y_deg": ty, "x_key_deg": None, "y_key_deg": None}

    def hold(self, seconds, step, collect=None, hold_hz=None):
        self.t += seconds
        return "ok"

    def track(self, fn, duration, step):
        self.t += duration
        p = fn(duration)
        self.pose = list(p)
        return "ok"

    def stop_test(self, axis, delta, after_s, step):
        acc = counts_to_deg(self.accel)
        self.t += after_s + self.speed / acc + 4.0
        self.pose[0 if axis == "x" else 1] += delta * 0.5
        return {}

    def clamp(self, where):
        self.prompts += 1
        return None

    def stats(self, key):
        return None


class LiveRig:
    """Drives the real (or simulated) Gimbal thread. Every wait goes through pump(): logs samples, sends the
    deadman heartbeat, and checks the abort conditions."""
    dry = False

    def __init__(self, g, link, log: RunLog, args, world=None):
        self.g, self.link, self.log, self.args, self.world = g, link, log, args, world
        self.q = queue.Queue()
        self.ctx = {"phase": "", "step": "", "cmd": (None, None), "collect": None, "note": ""}
        self.collect = {}
        self.rates = {"moving_hz": args.moving_hz, "hold_hz": args.hold_hz}
        self.temp0 = {}
        self.expect_on = False
        self.hold_trip = None
        self.sweep_peak = {}                 # step id -> {"x": max|ix| moving, "y": max|iy| moving}
        self._prev_peak = {}                 # --prev-run: peaks read from the earlier run
        self.points = []                     # measured grid cells: dict with stats
        self.res = {}
        self.remote = hasattr(link, "connected")
        self.sim_events = sorted(args.sim_event_parsed, key=lambda e: e[0])
        self.t0 = log.t0
        self._fast = None
        self.consec_trips = 0
        self.relaxed = False                 # return HOME after an abort: temperature / bus checks only logged

    # ---- sampler ------------------------------------------------------------------------------
    def sampler_cfg(self, **over):
        cfg = {"moving_hz": self.rates["moving_hz"], "hold_hz": self.rates["hold_hz"],
               "quiet_enable_s": self.args.quiet_s, "quiet_start_s": self.args.quiet_s,
               "quiet_track_s": self.args.quiet_track_s, "state_every_s": 1.0,
               "fast_hz": self.args.fast_hz, "on_sample": self._on_sample}
        cfg.update(over)
        return cfg

    def apply_sampler(self, **over):
        self.g.set_sampler(self.sampler_cfg(**over)).result(5)

    def _on_sample(self, s):                 # gimbal thread: keep quick
        cmd = self.ctx.get("cmd")
        if self.ctx.get("track"):
            try:
                cmd = self.g.act_target()
            except Exception:
                cmd = None
        self.q.put((self.ctx, cmd, s))

    def set_ctx(self, **kw):
        c = dict(self.ctx)
        c.update(kw)
        self.ctx = c                         # replaced, never mutated (the gimbal thread reads it)
        self.g.beat()                        # a prompt may have left the deadman stale: fresh beat before any goto

    def _drain(self):
        while True:
            try:
                ctx, cmd, s = self.q.get_nowait()
            except queue.Empty:
                return
            ax = s["axes"]
            x, y = ax["x"], ax["y"]
            ix = x.get("current") if x.get("read") else None
            iy = y.get("current") if y.get("read") else None
            tot = abs(ix) + abs(iy) if ix is not None and iy is not None else None
            moving = "".join(k for k in ("x", "y") if ax[k].get("moving"))
            cx, cy = (cmd or (None, None))[:2] if cmd else (None, None)
            g = self.g
            row = [s["t_utc"].isoformat(timespec="milliseconds"), f"{s['t_mono'] - self.t0:.3f}", ctx["phase"], ctx["step"],
                   moving, _f(cx, 3), _f(cy, 3), _f(x.get("angle"), 4), _f(y.get("angle"), 4),
                   _i(x.get("state")), _i(y.get("state")), _f(ix, 3), _f(iy, 3), _f(tot, 3),
                   x.get("dir", 0), y.get("dir", 0), _f(g.speed, 2), f"0x{s['accel_counts']:04X}",
                   _f(s.get("bus_v"), 2), _i(x.get("temp")), _i(y.get("temp")), ctx.get("note", "")]
            self.log.sample(row)
            if ctx.get("collect") is not None:
                self.collect.setdefault(ctx["collect"], []).append(
                    (s["t_mono"], ix, iy, tot, x.get("angle"), y.get("angle"), x.get("temp"), y.get("temp")))
            pre = ctx["step"]
            if pre:
                sp = self.sweep_peak.setdefault(pre, {"x": 0.0, "y": 0.0})
                if ix is not None and x.get("moving"):
                    sp["x"] = max(sp["x"], abs(ix))
                if iy is not None and y.get("moving"):
                    sp["y"] = max(sp["y"], abs(iy))
            # over the software stop while HOLDING: the gimbal only warns; the runner logs it and leaves the step
            if not moving and g.act is None:
                for k, v in (("x", ix), ("y", iy)):
                    if v is not None and abs(v) > SOFT_STOP_A[k] and self.hold_trip is None:
                        self.hold_trip = {"axis": k, "current": v}
                        self.log.event("current_trip", axis=k, current_a=round(v, 3), limit_a=SOFT_STOP_A[k], moving=False,
                                       x_deg=_r(x.get("angle")), y_deg=_r(y.get("angle")), phase=ctx["phase"], step=ctx["step"],
                                       msg=f"{k.upper()} {v:+.2f} A over the {SOFT_STOP_A[k]:.0f} A software stop while holding")

    def pump(self, check=True):
        self._drain()
        self.g.beat()
        if self.sim_events and self.world is not None:
            el = time.monotonic() - self.t0
            while self.sim_events and self.sim_events[0][0] <= el:
                _, axis, name, val = self.sim_events.pop(0)
                self.world.set_option(axis, name, val)
                self.log.event("sim_inject", axis=axis, name=name, value=val, msg=f"SIMULATOR: {axis}.{name} = {val}")
        if check:
            self.check_safety()

    def sleep(self, dt, check=True):
        end = time.monotonic() + dt
        while True:
            self.pump(check)
            left = end - time.monotonic()
            if left <= 0:
                return
            time.sleep(min(0.05, left))

    def now(self):
        return time.monotonic()

    def event(self, typ, **kw):
        self.log.event(typ, phase=self.ctx["phase"], **kw)

    def phase(self, pid):
        self.set_ctx(phase=pid, step="", collect=None, note="", track=False)

    # ---- abort conditions -----------------------------------------------------------------------
    def check_safety(self):
        g, a = self.g, self.args
        if self.remote and not self.link.connected:
            raise Abort(f"relay link lost: {self.link.last_error}")
        if g.mode == "offline":
            raise Abort("drives not answering (gimbal offline)")
        if g.mode == "fault":
            raise Abort(f"gimbal fault: {g.fault}")
        for k in ("x", "y"):
            t = g.run[k].temp
            if t is None:
                continue
            self.temp0.setdefault(k, t)
            if self.relaxed:
                continue
            if t > a.temp_max:
                raise Abort(f"{k.upper()} drive temperature {t} C above {a.temp_max:g} C")
            if t - self.temp0[k] > a.temp_rise:
                raise Abort(f"{k.upper()} drive temperature rose {t - self.temp0[k]:.0f} C (> {a.temp_rise:g} C) since the start")
        if g.bus_v is not None and g.bus_v < a.min_bus_v and not self.relaxed:
            raise Abort(f"bus voltage {g.bus_v:.1f} V below {a.min_bus_v:g} V")
        if self.expect_on:
            for k in ("x", "y"):
                if g.run[k].state == 0:
                    self._confirm_dropout(k)

    def _confirm_dropout(self, k):
        g = self.g
        err = g.run[k].err
        self.event("dropout_seen", axis=k, err=err, msg=f"{k.upper()} reports state 0 (motor OFF): re-reading in 1 s")
        self.sleep(1.0, check=False)
        try:
            st = g.call(lambda: g.axes[k].drive.state()).result(3)
            er = g.call(lambda: g.axes[k].drive.error()).result(3)
        except Exception as e:
            raise Abort(f"{k.upper()}: could not re-read state after a drop-out: {e}")
        g.run[k].state = st
        if st == 0:
            self.event("dropout", axis=k, err=f"0x{er:X}" if er is not None else None,
                       msg=f"{k.upper()} motor dropped out (state 0 again after 1 s, error 0x{er:X})")
            self.expect_on = False
            raise Dropout(f"{k.upper()} motor dropped out (error 0x{er:X})")
        self.event("dropout_transient", axis=k, state=st, msg=f"{k.upper()} state {st} on the re-read: continuing")

    # ---- motion ------------------------------------------------------------------------------------
    def begin(self):
        g = self.g
        self.event("start_motors", msg="Start: setup both (motors off) -> 1 s -> wake X then Y (+0.5 deg and back)")
        g.begin().result(5)
        end = time.monotonic() + 120
        while not (g.mode == "on" and g.started and g.holding):
            self.pump(check=False)
            if g.mode in ("fault", "offline"):
                raise Abort(f"Start failed: {g.fault or g.mode}")
            if time.monotonic() > end:
                raise Abort("Start did not finish in 120 s")
            time.sleep(0.1)
        self.expect_on = True
        self.event("motors_on", msg="both motors ON and holding")

    def restart_motors(self, why=""):
        self.expect_on = False
        self.begin()

    def set_speed(self, v):
        g = self.g
        if abs(g.speed - v) < 1e-9 and all(r.speed == v for r in g.run.values()):
            return
        r = g.set_speed(v).result(5)
        self.event("speed", speed=v, result=r, msg=f"profile speed {v:g} deg/s ({r})")
        if r == "needs start":
            ans = ask(f"\nThe drive refused a live speed change. Go HOME and press Start (motors off ~1 s) to use "
                      f"{v:g} deg/s? [y/N] ").lower()
            self.event("operator", question="live speed refused: HOME + Start?", answer=ans)
            if ans != "y":
                raise Abort("live speed change refused and the operator declined a restart")
            self.move(0.0, 0.0, f"{self.ctx['phase']}-home-for-speed")
            self.restart_motors("speed change")

    def set_accel(self, counts):
        g = self.g
        g.call(lambda: setattr(g, "accel_counts", int(counts))).result(5)
        self.event("accel", accel_reg=f"0x{counts:04X}", msg=f"accel/decel register -> 0x{counts:04X} (applied by Start)")
        self.restart_motors("accel change")

    def _wait_idle(self, timeout=8.0):
        end = time.monotonic() + timeout
        while any(r.phase in ("stopping", "starting", "moving") or r.mv is not None for r in self.g.run.values()):
            self.pump()
            if time.monotonic() > end:
                raise Abort("axes did not come to rest after a STOP")
            time.sleep(0.05)

    def move(self, x, y, step, collect=None, fast_axes=None, note=""):
        """Path-planned absolute move. -> "ok" | "trip" (software current stop: logged, caller skips the step)."""
        g = self.g
        p0 = g.angles()
        if None in p0:
            raise Abort("axis angle unknown")
        for wx, wy in plan_path(p0, (x, y)):
            r = self._goto(wx, wy, step, collect, fast_axes, note)
            if r != "ok":
                return r
        return "ok"

    def _goto(self, tx, ty, step, collect, fast_axes, note):
        g = self.g
        # cx, cy = g.angles()
        cx, cy = (clamp_soft(v) for v in g.angles())
        if abs(tx - cx) < MIN_MOVE:                  # never a standalone move < 0.5 deg: keep that axis
            tx = cx
        if abs(ty - cy) < MIN_MOVE:
            ty = cy
        if tx == cx and ty == cy:
            return "ok"
        if not cad_ok(tx, ty, CAD_MARGIN, SOFT):
            raise Abort(f"internal: goto X {tx:+.2f} Y {ty:+.2f} outside the CAD limit")
        self.set_ctx(step=step, cmd=(tx, ty), collect=collect, note=note, track=False)
        if fast_axes:
            self.apply_sampler(fast_until=time.monotonic() + self.args.quiet_s + 3.0, fast_axes=list(fast_axes))
            self._fast = True
        trip0 = g.last_trip
        act = g.goto(tx, ty, f"current char {step}").result(5)
        acc = counts_to_deg(g.accel_counts)
        dur = max(move_duration(tx - cx, g.speed, acc), move_duration(ty - cy, g.speed, acc))
        end = time.monotonic() + dur * 1.5 + 30
        while not act["event"].is_set():
            self.pump()
            if time.monotonic() > end:
                g.stop_act("move timed out").result(5)
                raise Abort(f"move to X {tx:+.1f} Y {ty:+.1f} did not finish in {dur * 1.5 + 30:.0f} s")
            time.sleep(0.05)
        if self._fast:
            self.apply_sampler()
            self._fast = None
        self.pump()
        if act["ok"]:
            self.consec_trips = 0
            return "ok"
        if g.last_trip is not None and g.last_trip is not trip0:
            tr = g.last_trip
            d = sgn((tx - cx) if tr["axis"] == "x" else (ty - cy))
            ang = tr.get("angle")
            motion = ("holding" if d == 0 else "lifting" if ang is not None and d * sgn(ang) < 0 else "lowering")
            self.event("current_trip", axis=tr["axis"], current_a=round(tr["current"], 3), limit_a=tr["limit"], moving=True,
                       x_deg=_r(g.axes["x"].angle), y_deg=_r(g.axes["y"].angle), direction=d, motion=motion, step=step,
                       msg=f"{tr['axis'].upper()} {tr['current']:+.2f} A over the {tr['limit']:.0f} A software stop while "
                           f"{motion}{'' if d == 0 else ' (+)' if d > 0 else ' (-)'} at X {g.axes['x'].angle:+.1f} Y {g.axes['y'].angle:+.1f}: "
                           f"STOPPED, step skipped (worst case)")
            self._wait_idle()
            self._count_trip()
            return "trip"
        self.check_safety()                          # a drop-out raises here
        raise Abort(f"move {step} failed: {act['why']}")

    def reach(self, tx, ty, step, watch=True):
        """Straight diagonal goto WITHOUT the CAD check (soft limits +/-80 only, current stops kept). Enter/Space
        (or --sim-diag-stop) = STOP both axes; the angle at the key press is that diagonal's max reach."""
        g = self.g
        cx, cy = (clamp_soft(v) for v in g.angles())
        if not (abs(tx) <= SOFT + 1e-9 and abs(ty) <= SOFT + 1e-9):
            raise Abort(f"internal: reach target X {tx:+.1f} Y {ty:+.1f} outside the soft limits")
        if abs(tx - cx) < MIN_MOVE and abs(ty - cy) < MIN_MOVE:
            return {"result": "arrived", "x_deg": _r(cx), "y_deg": _r(cy), "x_key_deg": None, "y_key_deg": None}
        self.set_ctx(step=step, cmd=(tx, ty), collect=step, note="diagonal reach", track=False)
        trip0 = g.last_trip
        act = g.goto(tx, ty, f"current char {step}").result(5)
        acc = counts_to_deg(g.accel_counts)
        dur = max(move_duration(tx - cx, g.speed, acc), move_duration(ty - cy, g.speed, acc))
        end = time.monotonic() + dur * 1.5 + 30
        simstop = getattr(self.args, "sim_diag_stop", None) if self.world is not None else None
        key = None
        with KeyWatch() as kw:
            while not act["event"].is_set():
                self.pump()
                hit = watch and kw.pressed()
                if watch and simstop is not None and tx * ty != 0:
                    x, y = g.angles()
                    hit = hit or (x is not None and min(abs(x), abs(y)) >= simstop and sgn(x) == sgn(tx) and sgn(y) == sgn(ty))
                if hit:
                    x, y = g.angles()
                    key = (x, y)
                    g.stop_act("operator STOP (diagonal reach)").result(5)
                    print(f"\n  >>> STOP at X {x:+.2f} Y {y:+.2f}")
                    break
                if time.monotonic() > end:
                    g.stop_act("move timed out").result(5)
                    raise Abort(f"move to X {tx:+.1f} Y {ty:+.1f} did not finish in {dur * 1.5 + 30:.0f} s")
                time.sleep(0.05)
        if key is not None:
            self._wait_idle()
            self.sleep(1.0)
            x, y = g.angles()
            self.event("diag_key_stop", step=step, x_key_deg=_r(key[0]), y_key_deg=_r(key[1]), x_deg=_r(x), y_deg=_r(y),
                       msg=f"operator STOP at X {key[0]:+.2f} Y {key[1]:+.2f}, came to rest at X {x:+.2f} Y {y:+.2f}")
            return {"result": "key", "x_deg": _r(x), "y_deg": _r(y), "x_key_deg": _r(key[0]), "y_key_deg": _r(key[1])}
        self.pump()
        x, y = g.angles()
        if act["ok"]:
            self.consec_trips = 0
            return {"result": "arrived", "x_deg": _r(x), "y_deg": _r(y), "x_key_deg": None, "y_key_deg": None}
        if g.last_trip is not None and g.last_trip is not trip0:
            tr = g.last_trip
            self.event("current_trip", axis=tr["axis"], current_a=round(tr["current"], 3), limit_a=tr["limit"], moving=True,
                       x_deg=_r(x), y_deg=_r(y), step=step, motion="diagonal reach",
                       msg=f"{tr['axis'].upper()} {tr['current']:+.2f} A over the {tr['limit']:.0f} A software stop on the "
                           f"diagonal at X {x:+.1f} Y {y:+.1f}: STOPPED (counts as this diagonal's limit)")
            self._wait_idle()
            self._count_trip()
            x, y = g.angles()
            return {"result": "trip", "x_deg": _r(x), "y_deg": _r(y), "x_key_deg": None, "y_key_deg": None}
        self.check_safety()
        raise Abort(f"move {step} failed: {act['why']}")

    def _count_trip(self):
        """A trip skips the step; three in a row means an axis is over its stop wherever the plan goes next
        (e.g. holding over the limit makes every goto trip): abort instead of creeping on."""
        self.consec_trips += 1
        if self.consec_trips >= 3:
            raise Abort(f"{self.consec_trips} software current stops in a row: an axis stays over its limit")

    def hold(self, seconds, step, collect=None, hold_hz=None):
        g = self.g
        x, y = g.angles()
        self.set_ctx(step=step, cmd=(x, y), collect=collect, note="", track=False)
        if hold_hz is not None:
            self.apply_sampler(hold_hz=hold_hz)
        self.hold_trip = None
        end = time.monotonic() + seconds
        try:
            while time.monotonic() < end:
                self.pump()
                if self.hold_trip is not None:
                    self._count_trip()
                    return "trip"
                time.sleep(0.05)
        finally:
            if hold_hz is not None:
                self.apply_sampler()
        return "ok"

    def track(self, fn, duration, step):
        """fn(seconds since start) -> (x, y). Runs the gimbal's real track mode for `duration` s."""
        import clock
        g = self.g
        t_start = clock.now()

        def target(t_utc):
            return fn(max(0.0, min(duration, (t_utc - t_start).total_seconds())))

        self.set_ctx(step=step, cmd=None, collect=None, note="", track=True)
        trip0 = g.last_trip
        act = g.track(target, f"current char {step}").result(5)
        end = time.monotonic() + duration
        result = "ok"
        while time.monotonic() < end:
            self.pump()
            if act["event"].is_set():
                if g.last_trip is not None and g.last_trip is not trip0:
                    tr = g.last_trip
                    self.event("current_trip", axis=tr["axis"], current_a=round(tr["current"], 3), limit_a=tr["limit"],
                               moving=True, x_deg=_r(g.axes["x"].angle), y_deg=_r(g.axes["y"].angle), step=step,
                               msg=f"{tr['axis'].upper()} {tr['current']:+.2f} A over the software stop while tracking: STOPPED, rest of the profile skipped")
                    result = "trip"
                    break
                self.check_safety()
                raise Abort(f"tracking stopped: {act['why']}")
            time.sleep(0.05)
        if g.act is not None:
            g.stop_act("tracking profile done").result(5)
        self._wait_idle()
        self.set_ctx(track=False)
        return result

    def stop_test(self, axis, delta, after_s, step):
        """Start a move of `delta` deg on `axis`, STOP (0x84) after `after_s` s, measure spike + distance."""
        import clock
        g = self.g
        # x, y = g.angles()
        x, y = (clamp_soft(v) for v in g.angles())
        tx, ty = (x + delta, y) if axis == "x" else (x, y + delta)
        if not cad_ok(tx, ty, CAD_MARGIN, SOFT) or plan_path((x, y), (tx, ty)) != [(tx, ty)]:
            self.event("stop_test_skipped", axis=axis, msg="move would leave the limits")
            return {}
        self.set_ctx(step=step, cmd=(tx, ty), collect=step, note="", track=False)
        self.apply_sampler(fast_until=time.monotonic() + after_s + 4.0, fast_axes=[axis])
        act = g.goto(tx, ty, f"current char {step}").result(5)
        self.sleep(after_s)
        tm = time.monotonic()
        pose = g.pose_at(tm)
        a_stop = pose[0 if axis == "x" else 1] if pose else None
        t_stop_utc = clock.now().isoformat(timespec="milliseconds")
        self.set_ctx(note="STOP 0x84 sent")
        g.stop_act("STOP test (phase 5)").result(5)
        self._wait_idle(15)
        self.sleep(3.0)
        self.set_ctx(note="")
        self.apply_sampler()
        final = g.angles()[0 if axis == "x" else 1]
        rows = [c for c in self.collect.get(step, []) if c[0] >= tm]
        k = 1 if axis == "x" else 2
        spike = max((abs(c[k]) for c in rows if c[k] is not None), default=None)
        res = {"axis": axis, "commanded_delta_deg": delta, "stop_after_s": after_s, "t_stop_utc": t_stop_utc,
               "angle_at_stop_deg": _r(a_stop), "final_angle_deg": _r(final),
               "stopping_distance_deg": _r(abs(final - a_stop)) if a_stop is not None and final is not None else None,
               "peak_current_after_stop_A": _r(spike), "act_why": act.get("why")}
        self.event("stop_test", **res, msg=f"{axis.upper()} STOP: distance {res['stopping_distance_deg']} deg, "
                                           f"peak {res['peak_current_after_stop_A']} A in the 3 s after STOP")
        return res

    def clamp(self, where):
        v = ask(f"\n>>> CLAMP METER ({where}): type the 48 V supply current in A (blank = no reading): ")
        val = None
        try:
            val = float(v) if v else None
        except ValueError:
            val = None
        x, y = self.g.angles()
        self.event("clamp_meter", where=where, supply_a=val, raw=v, x_deg=_r(x), y_deg=_r(y),
                   ix_a=_r(self.g.run["x"].current), iy_a=_r(self.g.run["y"].current),
                   msg=f"{where}: {v or 'no reading'}")
        return val

    def stats(self, key):
        rows = self.collect.get(key) or []
        return rows


def _f(v, n):
    return "" if v is None or (isinstance(v, float) and not math.isfinite(v)) else f"{float(v):.{n}f}"


def _i(v):
    return "" if v is None else str(int(v))


def _r(v, n=3):
    return None if v is None else round(float(v), n)


def cell_stats(rows):
    import statistics as st
    out = {}
    for name, k in (("ix", 1), ("iy", 2), ("itotal", 3)):
        v = [r[k] for r in rows if r[k] is not None]
        if v:
            out.update({f"{name}_mean": round(st.fmean(v), 4), f"{name}_max": round(max(v), 4), f"{name}_min": round(min(v), 4),
                        f"{name}_std": round(st.pstdev(v), 4), f"{name}_n": len(v)})
        else:
            out.update({f"{name}_n": 0})
    xs = [r[4] for r in rows if r[4] is not None]
    ys = [r[5] for r in rows if r[5] is not None]
    if xs:
        out["x_meas_deg"] = round(sum(xs) / len(xs), 3)
    if ys:
        out["y_meas_deg"] = round(sum(ys) / len(ys), 3)
    return out


# ================================================================================== the plan
class Plan:
    def __init__(self, args):
        self.a = args
        self.full = args.plan == "full"
        self.fast = args.plan == "fast"
        self.diag_spec = getattr(args, "diag", None)          # phase 3e subset (estimate uses it too)
        self.v = 1.5 if self.fast else 1.0     # fast plan: 1.5 deg/s (the fastest allowed speed)
        self.grid_step = 10 if self.full else 20
        self.thermal_s = 600 if self.full else 300
        self.log = None
        self.dir = {"x": 0, "y": 0}          # approach direction (last motion) per axis

    # ---- helpers --------------------------------------------------------------------------------
    def _mv(self, R, x, y, step, **kw):
        """Move and remember the approach direction of each axis."""
        p0 = R.pose if R.dry else R.g.angles()
        r = R.move(x, y, step, **kw)
        for k, a, b in (("x", p0[0], x), ("y", p0[1], y)):
            if abs(b - a) >= MIN_MOVE:
                self.dir[k] = sgn(b - a)
        return r

    def worst_hold(self, R):
        pts = [p for p in R.points if p.get("itotal_max") is not None]
        if pts:
            p = max(pts, key=lambda q: q["itotal_max"])
            return p["x_deg"], p["y_deg"], "phase 2 grid"
        return 60.0, 60.0, "default (phase 2 not run)"

    def worst_y_for_x_extreme(self, R, xe):
        pts = [p for p in R.points if p["x_deg"] == xe and p.get("ix_max") is not None]
        if pts:
            p = max(pts, key=lambda q: max(abs(q["ix_max"]), abs(q["ix_min"])))
            return p["y_deg"], "phase 2 grid"
        return 0.0, "default (no grid data at this X)"

    def worst_x_for_y_extreme(self, R, ye):
        pts = [p for p in R.points if p["y_deg"] == ye and p.get("iy_max") is not None]
        if pts:
            p = max(pts, key=lambda q: max(abs(q["iy_max"]), abs(q["iy_min"])))
            return p["x_deg"], "phase 2 grid"
        return 0.0, "default (no grid data at this Y)"

    # ---- phases ----------------------------------------------------------------------------------
    def ph_soak(self, R):
        R.set_speed(1.0)
        self._mv(R, 0.0, 0.0, "soak-home")
        for attempt in range(3):
            try:
                R.set_speed(0.5)
                t_end = R.now() + 120.0
                i = 0
                while R.now() < t_end or i == 0:
                    for wp in ((10.0, 0.0), (0.0, 0.0), (0.0, 10.0), (0.0, 0.0)):
                        self._mv(R, *wp, f"soak-{i}")
                        i += 1
                        if R.now() >= t_end:
                            break
                R.set_speed(1.0)
                R.event("soak_ok", moving_hz=R.rates["moving_hz"] if not R.dry else None,
                        msg="soak test passed: no drop-outs at this poll rate")
                return
            except Dropout as e:
                if R.dry:
                    raise
                old = dict(R.rates)
                R.rates = {k: v / 2.0 for k, v in R.rates.items()}
                R.event("soak_dropout", old_rates=old, new_rates=R.rates,
                        msg=f"DROP-OUT in the soak test ({e}). Poll rate halved: moving {R.rates['moving_hz']:g} Hz, "
                            f"holding {R.rates['hold_hz']:g} Hz")
                print(f"\n!!! A drive dropped out during the soak test. The poll rate is now HALVED "
                      f"(moving {R.rates['moving_hz']:g} Hz, holding {R.rates['hold_hz']:g} Hz).")
                ans = ask("Support the load if needed. Restart both motors (Start) and repeat the soak at the new rate? [y/N] ").lower()
                R.event("operator", question="restart after soak drop-out?", answer=ans)
                if ans != "y":
                    raise Abort("drop-out in the soak test; operator stopped the run")
                R.restart_motors("soak drop-out")
                R.apply_sampler()
        raise Abort("drives keep dropping out in the soak test even at a reduced poll rate")

    def ph_1(self, R):
        self._mv(R, 0.0, 0.0, "1-home")
        R.hold(60.0, "1-baseline", collect="baseline")

    def ph_2(self, R):
        st = self.grid_step
        vals = [float(v) for v in range(-80, 81, st)]
        R.set_speed(1.0)
        for i, x in enumerate(vals):
            ys = vals if i % 2 == 0 else list(reversed(vals))
            for y in ys:
                key = f"2-x{x:+.0f}-y{y:+.0f}"
                rec = {"x_deg": x, "y_deg": y}
                if not cad_ok(x, y, CAD_MARGIN, SOFT):
                    rec["skipped_reason"] = f"outside CAD limit minus {CAD_MARGIN:g} deg (max |Y| {cad_max_abs_y(abs(x)):.1f} at |X| {abs(x):g})"
                    if not R.dry:
                        self.log.point(rec)
                    continue
                r = self._mv(R, x, y, key + "-move")
                rec["approach_x"] = "+" if self.dir["x"] > 0 else ("-" if self.dir["x"] < 0 else "0")
                rec["approach_y"] = "+" if self.dir["y"] > 0 else ("-" if self.dir["y"] < 0 else "0")
                if r == "trip":
                    rec["skipped_reason"] = "software current stop on the way (events.jsonl)"
                    if not R.dry:
                        self.log.point(rec)
                    continue
                R.hold(2.0, key + "-settle")
                r = R.hold(5.0, key + "-hold", collect=key)
                if R.dry:
                    continue
                rec.update(cell_stats(R.stats(key)))
                if r == "trip":
                    rec["skipped_reason"] = "over the software current stop while holding (events.jsonl)"
                self.log.point(rec)
                if not rec.get("skipped_reason"):
                    R.points.append(rec)

    def _x_sweep(self, R, y, tag, clamp=False):
        xm = min(SOFT, cad_max_abs_x(abs(y)))
        pre = f"{tag}-y{y:+.0f}"
        self._mv(R, -xm, y, pre + "-approach")
        if clamp:
            r = self._mv(R, 60.0, y, pre + "-up")
            if r == "ok":
                R.hold(3.0, pre + "-clamp-hold")
                R.clamp(f"X sweep at Y {y:+.0f}, paused holding at X +60")
        self._mv(R, xm, y, pre + "-up")
        self._mv(R, -xm, y, pre + "-down")
        if clamp:
            R.clamp(f"X sweep at Y {y:+.0f}: highest reading seen WHILE MOVING")

    def _y_sweep(self, R, x, tag):
        ym = min(SOFT, cad_max_abs_y(abs(x)))
        pre = f"{tag}-x{x:+.0f}"
        self._mv(R, x, -ym, pre + "-approach")
        self._mv(R, x, ym, pre + "-up")
        self._mv(R, x, -ym, pre + "-down")

    def ph_3a(self, R):
        R.set_speed(1.0)
        for y in (-60.0, -30.0, 0.0, 30.0, 60.0):
            self._x_sweep(R, y, "3a", clamp=(self.a.clamp_meter and "4e" in self.selected and y == 0.0))

    def ph_3b(self, R):
        R.set_speed(1.0)
        for x in (-60.0, -30.0, 0.0, 30.0, 60.0):
            self._y_sweep(R, x, "3b")

    def _worst_line(self, R, tag, axis, default):
        best, val = default, -1.0
        for v in (-60.0, -30.0, 0.0, 30.0, 60.0):
            pk = 0.0
            for d in ("up", "down"):
                pk = max(pk, R.sweep_peak.get(f"{tag}-{'y' if axis == 'x' else 'x'}{v:+.0f}-{d}", {}).get(axis, 0.0))
            if pk > val and pk > 0:
                best, val = v, pk
        return best, (val if val > 0 else None)

    def ph_3c(self, R):
        y, pk = self._worst_line(R, "3a", "x", 0.0)
        R.event("worst_line", axis="x", y_deg=y, peak_a=pk, msg=f"worst X sweep from 3a: Y {y:+.0f} ({pk} A)")
        for v in (0.5, 1.5):
            R.set_speed(v)
            self._x_sweep(R, y, f"3c-v{v:g}")
        x, pk = self._worst_line(R, "3b", "y", 0.0)
        R.event("worst_line", axis="y", x_deg=x, peak_a=pk, msg=f"worst Y sweep from 3b: X {x:+.0f} ({pk} A)")
        for v in (0.5, 1.5):
            R.set_speed(v)
            self._y_sweep(R, x, f"3c-v{v:g}")
        R.set_speed(1.0)

    def ph_3d(self, R):
        R.set_speed(1.0)
        for a, b, tag in (((60.0, 60.0), (-60.0, -60.0), "3d-diag1"), ((60.0, -60.0), (-60.0, 60.0), "3d-diag2")):
            self._mv(R, *a, tag + "-approach")
            self._mv(R, *b, tag + "-fwd")
            self._mv(R, *a, tag + "-back")

    def ph_4a(self, R):
        R.set_speed(1.0)
        x, y, src = self.worst_hold(R)
        R.event("worst_pose", x_deg=x, y_deg=y, source=src, msg=f"breakaway at the worst holding pose X {x:+.0f} Y {y:+.0f} ({src})")
        for k in ("x", "y"):
            a = x if k == "x" else y
            if abs(a) < 5.0 + MIN_MOVE:
                R.event("breakaway_skipped", axis=k, msg=f"{k.upper()} is {a:+.0f} deg: too close to HOME for a 5 deg lift")
                continue
            self._mv(R, x, y, f"4a-{k}-approach")
            R.hold(30.0, f"4a-{k}-hold")
            tx, ty = (x - 5.0 * sgn(x), y) if k == "x" else (x, y - 5.0 * sgn(y))
            self._mv(R, tx, ty, f"4a-{k}-lift", fast_axes=[k])
        self._mv(R, x, y, "4a-return")

    def ph_4b(self, R):
        R.set_speed(1.0)
        for xe in (80.0, -80.0):
            y, src = self.worst_y_for_x_extreme(R, xe)
            y = max(-cad_max_abs_y(80.0), min(cad_max_abs_y(80.0), y))
            s = sgn(xe)
            tag = f"4b-x{xe:+.0f}"
            R.event("reversal_pose", axis="x", x_deg=xe, y_deg=y, source=src, msg=f"X reversal at {xe:+.0f}, Y {y:+.0f} ({src})")
            self._mv(R, xe - 10.0 * s, y, tag + "-approach")
            self._mv(R, xe, y, tag + "-out")
            self._mv(R, xe - 10.0 * s, y, tag + "-rev")
        for ye in (80.0, -80.0):
            x, src = self.worst_x_for_y_extreme(R, ye)
            xm = cad_max_abs_x(80.0)
            x = max(-xm, min(xm, x))
            s = sgn(ye)
            tag = f"4b-y{ye:+.0f}"
            R.event("reversal_pose", axis="y", x_deg=x, y_deg=ye, source=src, msg=f"Y reversal at {ye:+.0f}, X {x:+.0f} ({src})")
            self._mv(R, x, ye - 10.0 * s, tag + "-approach")
            self._mv(R, x, ye, tag + "-out")
            self._mv(R, x, ye - 10.0 * s, tag + "-rev")

    def ph_4c(self, R):
        R.set_speed(1.0)
        x, y, src = self.worst_hold(R)
        R.event("worst_pose", x_deg=x, y_deg=y, source=src, msg=f"thermal hold at X {x:+.0f} Y {y:+.0f} for {self.thermal_s / 60:.0f} min ({src})")
        self._mv(R, x, y, "4c-approach")
        R.hold(self.thermal_s, "4c-hold", hold_hz=0.1)

    def _profile(self):
        t, xs, ys = [], [], []
        with open(PROFILE_CSV) as f:
            for row in csv.DictReader(f):
                t.append(float(row[next(iter(row))]))
                xs.append(float(row["x_target_deg"]))
                ys.append(float(row["y_target_deg"]))
        t0 = t[0]
        t = [v - t0 for v in t]
        cx = [max(-SOFT, min(SOFT, v)) for v in xs]
        cy = []
        for xv, yv in zip(cx, ys):
            lim = min(SOFT, cad_max_abs_y(abs(xv)))
            cy.append(max(-lim, min(lim, yv)))
        n_clip = sum(1 for a, b, c, d in zip(xs, cx, ys, cy) if a != b or c != d)

        def fn(s):
            if s <= t[0]:
                return cx[0], cy[0]
            if s >= t[-1]:
                return cx[-1], cy[-1]
            lo, hi = 0, len(t) - 1
            while hi - lo > 1:
                mid = (lo + hi) // 2
                if t[mid] <= s:
                    lo = mid
                else:
                    hi = mid
            w = (s - t[lo]) / (t[hi] - t[lo])
            return cx[lo] + w * (cx[hi] - cx[lo]), cy[lo] + w * (cy[hi] - cy[lo])

        return fn, t[-1], n_clip, len(t)

    def ph_4d(self, R):
        fn, dur, n_clip, n = self._profile()
        R.set_speed(self.a.track_speed)
        R.event("profile", file=os.path.basename(PROFILE_CSV), duration_s=dur, points=n, clipped_points=n_clip,
                msg=f"DS-EO profile {dur:.0f} s, {n} points, {n_clip} clipped to the soft/CAD limits, track speed {self.a.track_speed:g} deg/s")
        x0, y0 = fn(0.0)
        self._mv(R, x0, y0, "4d-approach")
        R.track(fn, dur, "4d-track")
        R.set_speed(1.0)

    def ph_4e(self, R):
        R.set_speed(1.0)
        self._mv(R, 0.0, 0.0, "4e-home")
        R.hold(10.0, "4e-home-hold")
        R.clamp("HOME, both holding")
        x, y, src = self.worst_hold(R)
        self._mv(R, x, y, "4e-worst")
        R.hold(10.0, "4e-worst-hold")
        R.clamp(f"worst holding pose X {x:+.0f} Y {y:+.0f} ({src})")

    def ph_3e(self, R):
        if not R.dry:
            print("\n!!! PHASE 3e: DIAGONAL REACH. The dish aims for (+/-80, +/-80) along straight diagonals - PAST the CAD "
                  "collision limit (at X 80 the CAD allows |Y| 72.5). YOU are the limit: press ENTER or SPACE to STOP both "
                  "axes; that angle is recorded as the diagonal's max and the dish then heads the opposite way. "
                  f"Speed {R.args.diag_speed:g} deg/s: it coasts ~{R.args.diag_speed ** 2 / 2 / counts_to_deg(PROVEN_ACCEL):.1f} deg after the STOP, "
                  "so press early. Ctrl+C = abort everything. The software current stops (X 15 A, Y 11 A) still apply.")
            if not sys.stdin.isatty() and R.world is None:
                raise Abort("phase 3e needs a real terminal for the STOP key")
            ans = ask("Hand on the ePower switch, eyes on the dish. Type DIAG to run it (anything else skips): ")
            R.event("operator", question="phase 3e diagonal reach confirm", answer=ans)
            if ans != "DIAG":
                R.event("phase_skipped", msg="phase 3e skipped by the operator")
                return
        R.set_speed(R.args.diag_speed if not R.dry else 1.0)
        self._mv(R, 0.0, 0.0, "3e-home")
        res = {}
        # for (tx, ty), step, name in DIAG_LEGS:
        for (tx, ty), step, name in diag_legs(getattr(R.args, "diag", None) if not R.dry else self.diag_spec):
            r = R.reach(tx, ty, step)
            if name:
                if r["result"] == "key":
                    lim = (r["x_key_deg"], r["y_key_deg"])
                    how = "operator STOP"
                elif r["result"] == "trip":
                    lim = (r["x_deg"], r["y_deg"])
                    how = "software current stop"
                else:
                    lim = (r["x_deg"], r["y_deg"])
                    how = "reached the soft limit (no STOP)"
                res[name] = {"x_deg": lim[0], "y_deg": lim[1], "rest_x_deg": r["x_deg"], "rest_y_deg": r["y_deg"], "how": how}
                R.event("diag_reach", diagonal=name, x_deg=lim[0], y_deg=lim[1], how=how, step=step,
                        msg=f"diagonal {name}: max X {lim[0]:+.2f} Y {lim[1]:+.2f} ({how})" if lim[0] is not None else f"diagonal {name}: {how}")
        if not R.dry:
            R.res["diag_reach"] = res
            print("\n  Diagonal reach (angle at the STOP key):")
            for n, v in res.items():
                print(f"    {n}:  X {v['x_deg']:+.2f}  Y {v['y_deg']:+.2f}   ({v['how']})")

    def ph_5(self, R):
        if not R.dry:
            print("\n!!! PHASE 5: STOP (0x84) mid-move has NEVER been tested on the loaded dish. The axis will "
                  "decelerate at the accel register rate; the gimbal switches a motor OFF if the STOP is not obeyed.")
            ans = ask("Hand on the ePower switch, area clear. Type STOP to run it (anything else skips): ")
            R.event("operator", question="phase 5 STOP test confirm", answer=ans)
            if ans != "STOP":
                R.event("phase_skipped", msg="phase 5 skipped by the operator")
                return
        R.set_speed(1.0)
        x, y, src = self.worst_hold(R)
        for k in ("x", "y"):
            a = x if k == "x" else y
            if abs(a) < 10.0 + MIN_MOVE:
                continue
            self._mv(R, x, y, f"5-{k}-approach")
            R.hold(5.0, f"5-{k}-hold")
            R.stop_test(k, -10.0 * sgn(a), 4.0, f"5-{k}-stop")
        self._mv(R, x, y, "5-return")

    def ph_6(self, R):
        if not R.dry:
            print("\n!!! PHASE 6: the motors are switched OFF for ~1 s at HOME to load accel/decel 0x0B61, "
                  "then woken. 0x0419 is restored the same way afterwards.")
            ans = ask("Type ACCEL to run it (anything else skips): ")
            R.event("operator", question="phase 6 accel confirm", answer=ans)
            if ans != "ACCEL":
                R.event("phase_skipped", msg="phase 6 skipped by the operator")
                return
        R.set_speed(1.0)
        self._mv(R, 0.0, 0.0, "6-home")
        R.set_accel(TEST_ACCEL)
        try:
            self._x_sweep(R, 0.0, "6-acc0B61")
        finally:
            if not R.dry:
                try:
                    self._mv(R, 0.0, 0.0, "6-home-restore")
                    R.set_accel(PROVEN_ACCEL)
                except Abort:
                    raise
        if R.dry:
            self._mv(R, 0.0, 0.0, "6-home-restore")
            R.set_accel(PROVEN_ACCEL)

    # ---- driver -------------------------------------------------------------------------------------
    def run_phase(self, R, pid):
        if self.fast and hasattr(self, "fast_" + pid):
            return getattr(self, "fast_" + pid)(R)
        getattr(self, "ph_" + pid)(R)

    # ---- fast plan (~15 min, 1.5 deg/s): every sweep starts and ends at HOME (0 -> +80 -> -80 -> 0) --------
    def fast_1(self, R):
        R.set_speed(self.v)
        self._mv(R, 0.0, 0.0, "1-home")
        R.hold(10.0, "1-baseline", collect="baseline")

    def fast_3a(self, R):
        R.set_speed(self.v)
        self._mv(R, 0.0, 0.0, "3a-y+0-approach")
        self._mv(R, 80.0, 0.0, "3a-y+0-out")
        self._mv(R, -80.0, 0.0, "3a-y+0-up")
        self._mv(R, 0.0, 0.0, "3a-y+0-down")

    def fast_3b(self, R):
        R.set_speed(self.v)
        self._mv(R, 0.0, 0.0, "3b-x+0-approach")
        self._mv(R, 0.0, 80.0, "3b-x+0-out")
        self._mv(R, 0.0, -80.0, "3b-x+0-up")
        self._mv(R, 0.0, 0.0, "3b-x+0-down")

    def fast_3d(self, R):
        R.set_speed(self.v)
        self._mv(R, 50.0, 50.0, "3d-diag1-approach")
        self._mv(R, -50.0, -50.0, "3d-diag1-fwd")
        self._mv(R, 0.0, 0.0, "3d-diag1-back")

    def _fast_worst(self, R):
        """No holding map in the fast plan: the worst pose is (+/-60, +/-60), on the side where each axis' sweep
        needed the most lifting current ('-up' lifts the + side, '-down' lifts the - side). Default (+60, +60)."""
        side = {}
        for k, tag in (("x", "3a-y+0"), ("y", "3b-x+0")):
            up = R.sweep_peak.get(tag + "-up", {}).get(k, 0.0)
            dn = R.sweep_peak.get(tag + "-down", {}).get(k, 0.0)
            side[k] = -1.0 if dn > up else 1.0
        return 60.0 * side["x"], 60.0 * side["y"]

    def fast_4a(self, R):
        """Breakaway at the worst pose: hold 10 s, lift 5 deg toward HOME, per axis."""
        R.set_speed(self.v)
        x, y = self._fast_worst(R)
        R.event("worst_pose", x_deg=x, y_deg=y, source="fast plan: sweep sides", msg=f"worst pose X {x:+.0f} Y {y:+.0f}")
        self._mv(R, x, y, "4a-x-approach")
        R.hold(5.0, "4a-x-hold")
        self._mv(R, x - 5.0 * sgn(x), y, "4a-x-lift", fast_axes=["x"])
        self._mv(R, x, y, "4a-y-approach")
        R.hold(5.0, "4a-y-hold")
        self._mv(R, x, y - 5.0 * sgn(y), "4a-y-lift", fast_axes=["y"])

    def fast_4c(self, R):
        """30 s hold at the worst pose (the fast plan's hold check)."""
        R.set_speed(self.v)
        x, y = self._fast_worst(R)
        self._mv(R, x, y, "4c-approach")
        R.hold(30.0, "4c-hold", hold_hz=0.2)

    def estimate(self, selected):
        R = DryRig()
        per = {}
        moves = {}
        for pid in ["soak"] + [p for p, _ in PHASES[1:] if p in selected]:
            t0, m0 = R.t, R.moves
            self.selected = selected
            self.run_phase(R, pid)
            per[pid] = R.t - t0
            moves[pid] = R.moves - m0
        t0 = R.t
        R.move(0.0, 0.0, "end-home")
        per["end"] = R.t - t0
        return per, moves, R.prompts

    def grid_counts(self):
        vals = list(range(-80, 81, self.grid_step))
        n = len(vals) ** 2
        ok = sum(1 for x in vals for y in vals if cad_ok(x, y, CAD_MARGIN, SOFT))
        return n, ok


# ================================================================================== main
def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="Gimbal motor-current characterisation (headless, through the Gimbal thread)")
    ap.add_argument("--pi", default=config.CAN_BUS.pi, help="Pi running can_relay.py: <host>[:7070] (hardware)")
    ap.add_argument("--plan", choices=("fast", "quick", "full"), default="quick",
                    help="fast = ~15 min at 1.5 deg/s: soak, 20 s baseline, one X and one Y sweep, one diagonal, "
                         "breakaway + 1 min hold at the worst pose (no holding map)")
    ap.add_argument("--phases", default=None, help="e.g. 1,2,3,4,5 (3 = 3a-3d, 4 = 4a-4e); the soak test always runs. "
                                                   "Default: all (fast: 1,3a,3b,3d,4a,4c)")
    ap.add_argument("--sim", action="store_true", help="simulated drives (loaded-dish current model), no hardware")
    ap.add_argument("--time-scale", type=float, default=20.0, help="--sim only: run this many times faster (20)")
    ap.add_argument("--clamp-meter", action="store_true", help="phase 4e: ask for DC clamp-meter readings of the 48 V supply")
    ap.add_argument("--allow-stop-test", action="store_true", help="enable phase 5 (STOP mid-move, never tested on the load)")
    ap.add_argument("--allow-diag-reach", action="store_true",
                    help="enable phase 3e: diagonals toward (+/-80,+/-80) PAST the CAD limit, stopped by the operator's key")
    ap.add_argument("--diag", default=None, metavar="LIST",
                    help="phase 3e: only these diagonals, each HOME -> out -> HOME, e.g. -X-Y,+X-Y,-X+Y (default: all four "
                         "in the reversing sequence)")
    ap.add_argument("--diag-speed", type=float, default=1.0, help="phase 3e speed, deg/s (1.0: coasts ~0.7 deg after STOP)")
    ap.add_argument("--sim-diag-stop", type=float, default=None, metavar="DEG",
                    help="--sim only: press the 3e STOP key automatically when both |X| and |Y| reach DEG")
    ap.add_argument("--allow-accel-test", action="store_true", help="enable phase 6 (accel 0x0B61, then 0x0419 restored)")
    ap.add_argument("--moving-hz", type=float, default=2.0, help="both axes read at this rate while moving (2)")
    ap.add_argument("--hold-hz", type=float, default=1.0, help="... and at this rate while holding (1)")
    ap.add_argument("--fast-hz", type=float, default=4.0, help="breakaway / STOP test: moving axis only, this rate (4)")
    ap.add_argument("--quiet-s", type=float, default=1.0, help="no polling this long after an enable or a START (1.0)")
    ap.add_argument("--quiet-track-s", type=float, default=0.3,
                    help="phase 4d only: no polling this long after a tracking step's START on a HOLDING motor (0.3)")
    ap.add_argument("--min-gap-s", type=float, default=0.03, help="at least this long between CAN requests (0.03)")
    ap.add_argument("--track-speed", type=float, choices=config.MANUAL_SPEEDS_DEG_S, default=1.0,
                    help="profile speed for the tracking replay (the DS-EO profile peaks at 0.95 deg/s)")
    ap.add_argument("--temp-max", type=float, default=60.0, help="abort above this drive temperature, C (60)")
    ap.add_argument("--temp-rise", type=float, default=15.0, help="abort on a rise of more than this since the start, C (15)")
    ap.add_argument("--min-bus-v", type=float, default=44.0, help="abort below this bus voltage, V (44)")
    ap.add_argument("--out", default=OUT_ROOT, help="parent folder of the run folder")
    ap.add_argument("--prev-run", action="append", default=[], metavar="RUN_DIR",
                    help="earlier run whose sweeps (3a/3b) count for this one: their peaks pick the worst pose (4a/4c) "
                         "when a sweep is skipped here; merge the two runs with current_char_merge.py for the report")
    ap.add_argument("--sim-event", action="append", default=[], metavar="T:AXIS:NAME=VALUE",
                    help="--sim tests only: at T s (simulated) set a simulator option, e.g. 300:x:fake_temp_c=65, "
                         "200:y:gain_scale=4, 100:x:bus_v=43")
    a = ap.parse_args(argv)
    if a.phases is None:
        a.phases = "1,3a,3b,3d,4a,4c" if a.plan == "fast" else "1,2,3,4,5,6"
    if not a.sim and not a.pi:
        ap.error("--pi <ip> is required for hardware (or use --sim)")
    if a.sim_event and not a.sim:
        ap.error("--sim-event only with --sim")
    a.sim_event_parsed = []
    for s in a.sim_event:
        try:
            t, axis, kv = s.split(":", 2)
            name, val = kv.split("=", 1)
            v = None if val.lower() == "none" else (float(val) if val.replace(".", "", 1).replace("-", "", 1).isdigit() else val)
            a.sim_event_parsed.append((float(t), axis, name, v))
        except ValueError:
            ap.error(f"bad --sim-event {s!r}")
    return a


def select_phases(a):
    want = set()
    for tok in a.phases.split(","):
        tok = tok.strip()
        if not tok:
            continue
        if tok == "3":
            want |= {"3a", "3b", "3c", "3d"}
        elif tok == "4":
            want |= {"4a", "4b", "4c", "4d", "4e"}
        elif tok in PHASE_TITLES and tok != "soak":
            want.add(tok)
        elif tok in ("0", "soak"):
            pass
        else:
            raise SystemExit(f"unknown phase {tok!r} (use 1,2,3,3a..3e,4,4a..4e,5,6)")
    notes = {}
    if "4e" in want and not a.clamp_meter:
        want.discard("4e")
        notes["4e"] = "needs --clamp-meter"
    if "3e" in want and not a.allow_diag_reach:
        want.discard("3e")
        notes["3e"] = "needs --allow-diag-reach"
    if "5" in want and not a.allow_stop_test:
        want.discard("5")
        notes["5"] = "needs --allow-stop-test"
    if "6" in want and not a.allow_accel_test:
        want.discard("6")
        notes["6"] = "needs --allow-accel-test"
    return [p for p, _ in PHASES[1:] if p in want], notes


def git_head():
    try:
        r = subprocess.run(["git", "-C", HERE, "rev-parse", "HEAD"], capture_output=True, text=True, timeout=5)
        return r.stdout.strip() if r.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        return None


def config_snapshot():
    def ax(c):
        return {k: getattr(c, k) for k in ("name", "node_id", "model", "sign", "min_deg", "max_deg", "slew_speed_deg_s",
                                           "track_speed_deg_s", "accel_deg_s2", "current_limit_a")}
    cb = config.CAN_BUS
    return {"AXES": {k: ax(v) for k, v in config.AXES.items()}, "HOME_COUNTS": dict(config.HOME_COUNTS),
            "COUNTS_PER_REV": config.COUNTS_PER_REV, "MANUAL_ACCEL_COUNTS": f"0x{config.MANUAL_ACCEL_COUNTS:04X}",
            "MANUAL_SPEEDS_DEG_S": list(config.MANUAL_SPEEDS_DEG_S), "MIN_MOVE_DEG": config.MIN_MOVE_DEG,
            "SETUP_TO_ENABLE_S": config.SETUP_TO_ENABLE_S, "POLL_PERIOD_S": config.POLL_PERIOD_S,
            "SLOW_POLL_PERIOD_S": config.SLOW_POLL_PERIOD_S, "TRACK_STEP_PERIOD_S": config.TRACK_STEP_PERIOD_S,
            "GOTO_DONE_DEG": config.GOTO_DONE_DEG, "PAGE_DEADMAN_S": config.PAGE_DEADMAN_S,
            "CAN_BUS": {"interface": cb.interface, "bitrate": cb.bitrate, "gap_s": cb.gap_s, "reply_timeout_s": cb.reply_timeout_s},
            "RELAY_PORT": config.RELAY_PORT, "RELAY_WATCHDOG_S": config.RELAY_WATCHDOG_S,
            "BORESIGHT_OFFSET": dict(config.BORESIGHT_OFFSET)}


def new_run_dir(root, sim):
    os.makedirs(root, exist_ok=True)
    base = datetime.now().strftime("%Y%m%d_%H%M%S") + ("_SIM" if sim else "")
    d = os.path.join(root, base)
    i = 2
    while os.path.exists(d):
        d = os.path.join(root, f"{base}_{i}")
        i += 1
    os.makedirs(d)
    return d


def print_plan(a, plan, selected, notes, per, moves, prompts, status):
    n, ok = plan.grid_counts()
    print("\n" + "=" * 100)
    print(f"CURRENT CHARACTERISATION - plan '{a.plan}'" + ("   [SIMULATOR, x%g time]" % a.time_scale if a.sim else "   [HARDWARE]"))
    print("=" * 100)
    print(status)
    print(f"Angle range: X and Y -80..+80 deg from HOME (soft limits); CAD limit minus {CAD_MARGIN:g} deg on every path "
          f"(max |Y| {cad_max_abs_y(80):.2f} at |X| 80; |Y| 80 allowed up to |X| {cad_max_abs_x(80):.1f})")
    print(f"Grid ({'2'}): {plan.grid_step} deg steps -> {n} cells, {ok} inside the limit, {n - ok} skipped")
    print(f"Sampling: both axes {a.moving_hz:g} Hz moving / {a.hold_hz:g} Hz holding (state <= 1 Hz, temp + bus V every 10 s), "
          f">= {a.min_gap_s * 1000:.0f} ms between requests, quiet {a.quiet_s:g} s after enable/START "
          f"({a.quiet_track_s:g} s after a tracking step on a holding motor); breakaway/STOP: {a.fast_hz:g} Hz, moving axis only")
    print(f"Abort limits: Ctrl+C | drive state 0 still there 1 s later | temperature > {a.temp_max:g} C or rise > {a.temp_rise:g} C "
          f"| bus < {a.min_bus_v:g} V | relay link lost.  Software current stops kept: X {SOFT_STOP_A['x']:g} A, Y {SOFT_STOP_A['y']:g} A "
          f"(trip = STOP, logged, step skipped).")
    print(f"Accel/decel 0x{PROVEN_ACCEL:04X}" + (" (phase 6: 0x0B61, then restored)" if "6" in selected else "") + "; speeds 0.5/1.0/1.5 deg/s")
    print("-" * 100)
    total = 0.0
    print(f"{'phase':6} {'est.':>8} {'moves':>6}  description")
    for pid, title in PHASES:
        if pid in per:
            total += per[pid]
            print(f"{pid:6} {fmt_s(per[pid]):>8} {moves[pid]:>6}  {title_of(pid, a.plan)}")
        else:
            why = notes.get(pid, "not selected")
            print(f"{pid:6} {'-':>8} {'':>6}  {title}   [skipped: {why}]")
    total += per.get("end", 0)
    print(f"{'end':6} {fmt_s(per.get('end', 0)):>8} {'':>6}  return to HOME")
    print("-" * 100)
    print(f"ESTIMATED TOTAL {fmt_s(total)}  (+ prompts{', ' + str(prompts) + ' clamp readings' if prompts else ''}; "
          f"phase 2 and the worst-case poses use the data as it comes in)")
    print("NOTE: register 0x0008 is MOTOR current, not the 48 V supply current; 'total' = |I_X| + |I_Y|.")
    print("=" * 100)
    return total


def main(argv=None):
    a = parse_args(argv)
    import signal
    signal.signal(signal.SIGINT, signal.default_int_handler)     # Ctrl+C always aborts (even if started with & / nohup)
    selected, notes = select_phases(a)
    clk = None
    if a.sim:
        clk = FastClock(a.time_scale)
        # the gimbal's 2 s deadman counts simulated time; keep it 2 s of REAL time so a busy laptop does not trip it
        config.PAGE_DEADMAN_S = config.PAGE_DEADMAN_S * a.time_scale
    else:
        relay_preflight(a.pi)

    import clock
    from gimbal import Gimbal
    world = None
    if a.sim:
        from sim_hw import SimCanLink, SimWorld
        world = SimWorld.reset()
        world.set_option("x", "current_model", "load")
        for _k in ("x", "y"):                         # real drives stop a hair past the target (14 Nov: +80.004)
            world.set_option(_k, "settle_err_deg", 0.004)
        link = SimCanLink()
    else:
        from zeroerr_protocol import RemoteCanLink
        link = RemoteCanLink(a.pi)
        check_relay_busy(link)
    g = Gimbal(link, home=config.HOME_COUNTS)
    for ax in g.axes.values():
        ax.drive.gap_s = max(ax.drive.gap_s, a.min_gap_s)
    g.start()
    t_conn = time.monotonic() + 20
    while g.mode == "offline" and time.monotonic() < t_conn:
        time.sleep(0.1)
    if g.mode == "offline":
        g.shutdown()
        link.close()
        raise SystemExit("drives not answering (check 48 V, the CAN cable and the relay)")
    time.sleep(0.5)
    s = g.snapshot()
    ax = s["axes"]
    status = (f"Link: {s['link']}   bus {s['bus_v'] or '?'} V   "
              f"X {ax['x']['angle']:+.2f} deg (state {ax['x']['state_name']}, {ax['x']['temp']} C)   "
              f"Y {ax['y']['angle']:+.2f} deg (state {ax['y']['state_name']}, {ax['y']['temp']} C)   "
              f"HOME counts X {config.HOME_COUNTS['x']} Y {config.HOME_COUNTS['y']}")
    plan = Plan(a)
    plan.selected = selected
    per, moves, prompts = plan.estimate(selected)
    print_plan(a, plan, selected, notes, per, moves, prompts, status)
    start_pose = (ax["x"]["angle"], ax["y"]["angle"])
    if max(abs(start_pose[0]), abs(start_pose[1])) > 5:
        print(f"!!! The dish is at X {start_pose[0]:+.1f}, Y {start_pose[1]:+.1f}: Start switches both motors OFF for ~1 s "
              f"there (setup), then wakes them. Support the load or bring it near HOME first.")

    # ---- per-phase y/n
    print("\nPhase selection (Enter = yes):")
    run_list = []
    for pid in selected:
        ans = ask(f"  run phase {pid}: {title_of(pid, a.plan)[:80]}? [Y/n] ", "y").lower()
        if ans in ("", "y", "yes"):
            run_list.append(pid)
        else:
            notes[pid] = "skipped by the operator"
    print("  soak: mandatory (cannot be skipped)")
    if run_list != selected:
        per, moves, prompts = plan.estimate(run_list)
        plan.selected = run_list
        print_plan(a, plan, run_list, notes, per, moves, prompts, status)
    plan.selected = run_list

    # ---- operator notes
    print("\nOperator notes (saved in plan.json):")
    op = {"counterweight": ask("  counterweight setup: "), "cable_routing": ask("  cable routing: "),
          "wind_weather": ask("  wind / weather: "), "attached_to_dish": ask("  anything attached to the dish: "),
          "other": ask("  other notes: ")}
    if not a.sim:
        print("\nBEFORE ANYTHING MOVES: the area around the dish is clear and you have a hand on the ePower switch.")
    if ask("Type RUN to start: ") != "RUN":
        print("not started (nothing moved)")
        g.shutdown()
        link.close()
        return 1

    run_dir = new_run_dir(a.out, a.sim)
    t0 = time.monotonic()
    log = RunLog(run_dir, t0)
    plan.log = log
    cmdline = "python3 " + " ".join([os.path.basename(sys.argv[0])] + (argv if argv is not None else sys.argv[1:]))
    pj = {"simulated": a.sim, "label": "SIMULATED" if a.sim else "HARDWARE",
          "started_utc": clock.now().isoformat(timespec="seconds"), "command": cmdline,
          "prev_run": [os.path.abspath(p) for p in a.prev_run],
          "args": {"plan": a.plan, "phases_arg": a.phases, "pi": a.pi, "sim": a.sim,
                   "time_scale": a.time_scale if a.sim else 1.0,
                   "clamp_meter": a.clamp_meter, "allow_stop_test": a.allow_stop_test, "allow_accel_test": a.allow_accel_test,
                   "rates": {"moving_hz": a.moving_hz, "hold_hz": a.hold_hz, "fast_hz": a.fast_hz, "thermal_hold_hz": 0.1},
                   "quiet_s": a.quiet_s, "quiet_track_s": a.quiet_track_s, "min_gap_s": a.min_gap_s,
                   "track_speed": a.track_speed, "sim_events": a.sim_event},
          "abort_limits": {"temp_max_c": a.temp_max, "temp_rise_c": a.temp_rise, "min_bus_v": a.min_bus_v,
                           "dropout": "state 0 still there after a 1 s re-read", "relay_link_lost": True, "ctrl_c": True},
          "software_current_stops_A": SOFT_STOP_A, "rated_continuous_A": RATED_A,
          "rated_peak": {"x": "30 A for 4 s", "y": "27 A for 4 s"},
          "cad": {"margin_deg": CAD_MARGIN, "rule": "linear interpolation, floored to the lower neighbouring table value, minus margin",
                  "max_abs_y_at_abs_x_80": cad_max_abs_y(80), "max_abs_x_for_abs_y_80": cad_max_abs_x(80)},
          "grid": {"step_deg": plan.grid_step, "range_deg": [-80, 80], "settle_s": 2.0, "sample_s": 5.0,
                   "order": "serpentine: X columns, Y alternating direction", "speed_deg_s": 1.0},
          "sweeps": {"3a_y": [-60, -30, 0, 30, 60], "3b_x": [-60, -30, 0, 30, 60], "speed_deg_s": 1.0, "3c_speeds": [0.5, 1.5],
                     "3d": ["(+60,+60)<->(-60,-60)", "(+60,-60)<->(-60,+60)"]},
          "thermal_hold_s": 30 if plan.fast else plan.thermal_s, "speed_deg_s": plan.v,
          "fast_plan": ({"sweeps": "X at Y=0 and Y at X=0: 0 -> +80 -> -80 -> 0", "diagonal": "(+50,+50) -> (-50,-50) -> HOME",
                         "worst_pose": "(+/-60, +/-60) on the side each sweep lifted most", "baseline_s": 10,
                         "breakaway_hold_s": 5, "hold_s": 30, "holding_map": "not run"} if plan.fast else None), "accel_reg": {"proven": f"0x{PROVEN_ACCEL:04X}", "phase6": f"0x{TEST_ACCEL:04X}"},
          "phases_selected": run_list, "phases_not_run": notes, "estimate_s": per,
          "home_counts": dict(config.HOME_COUNTS), "start_pose_deg": start_pose, "start_status": status,
          "operator_notes": op, "config": config_snapshot(), "git_head": git_head(),
          "files": ["plan.json", "samples.csv", "points.csv", "events.jsonl", "summary.json"],
          "columns": {"samples.csv": SAMPLE_COLS, "points.csv": POINT_COLS},
          "column_notes": {"ix_a/iy_a": "register 0x0008, motor current, signed A", "itotal_a": "|ix|+|iy|",
                           "x_dir/y_dir": "+1/-1 = direction the axis is moving, 0 = not moving",
                           "approach_x/approach_y": "direction of the last move of that axis before the cell",
                           "x_cmd_deg/y_cmd_deg": "goto target (track: target at that time)"}}
    write_json(os.path.join(run_dir, "plan.json"), pj)
    print(f"\nRun folder: {run_dir}")
    R = LiveRig(g, link, log, a, world)
    for pr in a.prev_run:
        seeded = seed_sweep_peaks(R, pr)
        log.event("prev_run", run=os.path.abspath(pr), steps=sorted(seeded),
                  msg=f"sweep peaks from {pr}: {', '.join(sorted(seeded)) or 'none found'}")
    log.event("run_start", plan=a.plan, phases=run_list, msg=f"run started, phases {['soak'] + run_list}")
    aborted = None
    try:
        R.begin()
        R.apply_sampler()
        for pid in ["soak"] + run_list:
            R.phase(pid)
            log.event("phase_start", phase=pid, msg=f"phase {pid}: {title_of(pid, a.plan)}")
            tp = time.monotonic()
            plan.run_phase(R, pid)
            log.event("phase_end", phase=pid, seconds=round(time.monotonic() - tp, 1), msg=f"phase {pid} done in {fmt_s(time.monotonic() - tp)}")
            write_summary(run_dir)
        R.phase("end")
        plan._mv(R, 0.0, 0.0, "end-home")
        log.event("run_end", msg="all phases done, back at HOME")
    except (Abort, KeyboardInterrupt) as e:
        aborted = "Ctrl+C" if isinstance(e, KeyboardInterrupt) else str(e)
        _abort(R, g, log, aborted, a, plan)
    except Exception as e:
        aborted = f"error: {e!r}"
        import traceback
        traceback.print_exc()
        _abort(R, g, log, aborted, a, plan)
    finally:
        try:
            g.set_sampler(None).result(5)
        except Exception:
            pass
        try:
            summ = write_summary(run_dir, aborted)
        except Exception as e:
            summ = {}
            print(f"summary failed: {e!r}")
        print(g.shutdown())
        link.close()
        log.close()
    _print_summary(summ, run_dir)
    return 0 if aborted is None else 2


def _abort(R, g, log, why, a, plan):
    print(f"\n!!! ABORT: {why}")
    # try:
    #     stopped = g.stop_all(f"abort: {why}").result(5)
    # except Exception as e:
    #     stopped = f"stop_all failed: {e}"
    import signal
    old_int = signal.signal(signal.SIGINT, signal.SIG_IGN)   # a second Ctrl+C must not interrupt the STOP itself
    try:
        stopped = g.stop_all(f"abort: {why}").result(5)
    except Exception as e:
        stopped = f"stop_all failed: {e}"
    finally:
        signal.signal(signal.SIGINT, old_int)
    log.event("abort", reason=why, stopped=stopped, x_deg=_r(g.axes["x"].angle), y_deg=_r(g.axes["y"].angle),
              msg=f"{why} -> STOP both axes ({stopped})")
    end = time.monotonic() + 8
    while any(r.phase == "stopping" for r in g.run.values()) and time.monotonic() < end:
        try:
            R._drain()
        except Exception:
            pass
        time.sleep(0.05)
    try:
        R._drain()
    except Exception:
        pass
    x, y = g.angles()
    # print(f"Both axes stopped at X {x if x is None else round(x, 2)}, Y {y if y is None else round(y, 2)} (motors stay ON and holding).")
    on = g.mode == "on"                                  # "ready" = a motor is OFF (drop-out), "fault"/"offline" likewise
    print(f"Both axes stopped at X {x if x is None else round(x, 2)}, Y {y if y is None else round(y, 2)} "
          + ("(motors stay ON and holding)." if on else "(a motor is OFF: it is NOT holding - support the load)."))
    if g.mode in ("fault", "offline") or (R.remote and not R.link.connected):
        print("Cannot return HOME automatically (gimbal in fault/offline or relay lost). Use app.py when it is safe.")
        log.event("abort_no_home", mode=g.mode)
        return
    try:
        ans = ask("Return HOME now (path-planned, 1 deg/s)? [y/N] ").lower()
    except KeyboardInterrupt:
        ans = "n"
    log.event("operator", question="return HOME after abort?", answer=ans)
    if ans != "y":
        return
    try:
        R.phase("abort-home")
        R.expect_on = all(r.on_expected for r in g.run.values())
        R.relaxed = True                     # moving HOME lowers the load: temperature / bus no longer block it
        R.consec_trips = 0
        if g.speed != 1.0:
            R.set_speed(1.0)
        px, py = (clamp_soft(v) for v in g.angles())
        if not cad_ok(px, py, CAD_MARGIN, SOFT):        # stopped out on a 3e diagonal: back the way it came
            rr = R.reach(0.0, 0.0, "abort-home", watch=False)
            r = "ok" if rr["result"] == "arrived" else rr["result"]
        else:
            r = plan._mv(R, 0.0, 0.0, "abort-home")
        if r != "ok":
            log.event("abort_home_failed", reason=r, msg="return HOME stopped by a software current stop: use app.py when it is safe")
            return
        log.event("abort_home", msg="back at HOME")
        if g.accel_counts != PROVEN_ACCEL:
            ans = ask(f"Accel is still 0x{g.accel_counts:04X}. Restore 0x{PROVEN_ACCEL:04X} now (Start at HOME)? [y/N] ").lower()
            if ans == "y":
                R.set_accel(PROVEN_ACCEL)
    except (Abort, KeyboardInterrupt) as e:
        try:
            g.stop_all("abort during return HOME").result(5)
        except Exception:
            pass
        log.event("abort_home_failed", reason=str(e) or "Ctrl+C", msg=f"return HOME stopped: {e or 'Ctrl+C'}")


def _print_summary(s, run_dir):
    print("\n" + "=" * 100)
    print(f"SUMMARY ({s.get('label', '?')})  {run_dir}")
    if s.get("aborted"):
        print(f"ABORTED: {s['aborted']} (everything up to the abort is logged)")
    print(f"samples {s.get('n_samples')}, grid cells measured {s.get('grid', {}).get('n_measured')}, sweeps {s.get('n_sweeps')}")
    for k, lab in (("x", "|I_X|"), ("y", "|I_Y|"), ("total", "|I_X|+|I_Y|")):
        p = (s.get("peak") or {}).get(k)
        if p:
            print(f"  peak {lab:12} {p['itotal_a'] if k == 'total' else abs(p['i' + k + '_a']):6.2f} A at X {p['x_deg']:+.1f} Y {p['y_deg']:+.1f}, "
                  f"phase {p['phase']} step {p['step_id']}, X {p['x_motion']} Y {p['y_motion']}, {p['pct_rating']:.0f} % of rating")
    for t in s.get("current_stop_trips", []):
        print(f"  current stop: {t.get('msg')}")
    for n, v in (s.get("diag_reach") or {}).items():
        print(f"  diagonal {n:5} max X {v['x_deg']:+.2f} Y {v['y_deg']:+.2f} ({v['how']})")
    print("NOTE: register 0x0008 is MOTOR current, not 48 V supply current. 'Total' = |I_X| + |I_Y|. Supply current only "
          "from the clamp-meter readings.")
    print(f"Next: python3 current_char_report.py {run_dir} --report-dir urex_report")
    print("=" * 100)


if __name__ == "__main__":
    sys.exit(main())
