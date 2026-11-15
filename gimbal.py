"""
gimbal.py - the one thread that owns the CAN bus and both axes.

Test stage: manual latching WASD + Home + goto/track for the Sun / G-T steps. Nothing else sends CAN
frames. The web server and the Sun job queue commands here and read snapshot() / pose_at().

Modes:
  offline   drives not answering (retries every 2 s, also while the Pi relay is unreachable)
  ready     both nodes found, values shown, nothing moved. Press Start (or Home).
  starting  Start sequence: set up both (motor off), 1 s, wake X (+0.5 deg and back), wake Y
  homing    Home: both axes to 0/0 together (absolute: target = HOME - raw), at the current speed
  on        both motors on and holding; latching moves, goto/track allowed
  fault     a STOP was not obeyed (that motor was switched off) or Start/Home failed. Press Start again.

Bench rules (see config.py / DEPLOY.md):
  * Every move from motor-off is 0100=1, 0087=<relative counts>, 0083 back-to-back. Never enable and wait.
  * After a completed move the motor holds (state 4). It is only switched off automatically when
    a STOP (0x84) is not obeyed.
  * At least 1 s between a motor-off/setup and the next enable. Standalone moves at least 0.5 deg
    (tracking steps on a holding motor can be smaller).
  * Poll gently, about 6 reads/s in total, no bursts right after enable.

Per-axis phase: idle | starting (START in flight / being confirmed) | moving (state 2 confirmed)
                | stopping (0x84 sent, waiting for state != 2) | waking (Start) | homing (Home)
                | auto (goto/track: the activity controller steps this axis)

goto / track ("activity"): one relative step per axis at a time (START is refused while moving). Each
step is aimed at where the target will be when the step ends (duration from the trapezoid profile:
speed 0x8A, accel 0x88). After the planned end the position is read, then the next step may go
(at most every TRACK_STEP_PERIOD_S). Targets beyond the soft limits are clipped and reported.
"""
import logging
import math
import threading
import time
from collections import deque
from concurrent.futures import Future
from datetime import timedelta

import clock
import config
from erob_can_axis import Axis
from zeroerr_protocol import Drive, DriveError, ERRORS, STATES, counts_to_deg, deg_to_counts  # noqa: F401

log = logging.getLogger(__name__)

TICK_S = 0.1
CPR = config.COUNTS_PER_REV
STATE_NAMES = {0: "off", 1: "on (idle)", 2: "moving", 3: "settling", 4: "holding"}
COMM_ERRORS = (TimeoutError, DriveError, OSError)


def _err_text(code):
    if code is None:
        return "?"
    return f"0x{code:X} ({ERRORS.get(code, 'see ZeroErr fault list')})"


def wrap_counts(c: int) -> int:
    """Wrap a count difference to +/-262144 (half a revolution)."""
    return (int(c) + CPR // 2) % CPR - CPR // 2


def trapezoid(t, t0, p0, p1, v, a):
    """Position (deg) at time t of a move p0 -> p1 with profile speed v (deg/s) and accel a (deg/s^2).
    Same profile as the drive (and sim_hw)."""
    d, s = abs(p1 - p0), (1 if p1 >= p0 else -1)
    if d == 0 or t <= t0:
        return p0
    ta = v / a
    if a * ta * ta >= d:                         # triangle
        ta, tc = math.sqrt(d / a), 0.0
        v = a * ta
    else:
        tc = (d - a * ta * ta) / v
    dt = t - t0
    if dt >= 2 * ta + tc:
        return p1
    if dt < ta:
        x = 0.5 * a * dt * dt
    elif dt < ta + tc:
        x = 0.5 * a * ta * ta + v * (dt - ta)
    else:
        r = 2 * ta + tc - dt
        x = d - 0.5 * a * r * r
    return p0 + s * x


def move_duration(d, v, a):
    """Seconds for a move of d deg at profile speed v deg/s and accel a deg/s^2."""
    d = abs(d)
    if d <= 0:
        return 0.0
    if d >= v * v / a:
        return d / v + v / a
    return 2 * math.sqrt(d / a)


class _AxisRun:
    """Runtime state of one axis (only touched by the gimbal thread; read by snapshot()/pose_at())."""

    def __init__(self, key):
        self.key = key
        self.found = False
        self.raw = None
        self.current = None
        self.state = None
        self.temp = None
        self.err = None
        self.on_expected = False
        self.phase = "idle"
        self.latch = 0              # direction being moved / started (+1 / -1)
        self.pending = 0            # direction to start once the current STOP has finished
        self.start_ok = False       # START accepted (so a STOP must be sent to cancel)
        self.start_retry_until = 0.0
        self.next_attempt = 0.0
        self.confirm_at = 0.0
        self.confirm_until = 0.0
        self.stop_t = 0.0
        self.stop_deadline = 0.0
        self.next_check = 0.0
        self.off_t = -10.0          # last time we switched it off / saw it off
        self.next_poll = 0.0
        self.next_slow = 0.0
        self.fails = 0
        self.msg, self.msg_level = "", "info"
        self.refusal = {"seq": 0, "dir": 0, "msg": ""}
        self.speed = None           # profile speed believed to be in the drive (deg/s)
        self.over_warned = 0.0
        # activity (goto/track) + pose history
        self.mv = None              # planned step in progress: [t0, p0, p1, v, a, t_end] (monotonic, axis deg)
        self.next_step = 0.0
        self.busy_since = None      # first 0x15 of a step (still moving)
        self.pos_t = 0.0            # monotonic time of the last position read
        self.hist = deque(maxlen=400)     # (monotonic, axis deg) from position reads
        self.moves = deque(maxlen=60)     # planned steps, for pose_at()
        # current characterisation (current_char.py): when the last START / enable went to this axis
        self.t_start = -1e9               # monotonic time of the last 0x83 START
        self.t_enable = -1e9              # monotonic time of the last 0x0100=1 enable
        self.start_kind = ""              # "enable" | "start" | "track"
        self.next_state_read = 0.0        # sampler: state is read at most this often


class Gimbal:
    def __init__(self, link, home: dict | None = None):
        home = config.HOME_COUNTS if home is None else home
        cb = config.CAN_BUS
        timeout = getattr(link, "reply_timeout_s", cb.reply_timeout_s)
        self.link = link
        self.axes = {k: Axis(c, Drive(link, c.node_id, cb.gap_s, timeout), home.get(k))
                     for k, c in config.AXES.items()}
        self.run = {k: _AxisRun(k) for k in self.axes}
        self.mode = "offline"
        self.fault = None
        self.started = False
        self.events = deque(maxlen=60)
        self.bus_v = None
        self.speed = config.MANUAL_DEFAULT_SPEED_DEG_S      # wanted profile speed
        self.speed_pending = None                           # needs Start (motor off) to apply
        self.live_speed_ok = None                           # None = untried, True/False after the first try
        self.accel_counts = config.MANUAL_ACCEL_COUNTS
        self.start_step = ""
        self.home_result = None                             # {"ok", "err_counts": {x, y}, "msg"}
        self.act = None                                     # goto / track activity (dict)
        self._act_seq = 0
        self._seq = None
        self._seq_kind = None                               # "start" | "home"
        self._seq_resume = 0.0
        self._beat = None                                   # last page heartbeat (monotonic), None = no page yet
        self._q = deque()
        self._stop = threading.Event()
        self._next_connect = 0.0
        self._next_volts = 0.0
        # current characterisation sampler (None = normal app polling, unchanged). See set_sampler().
        self.sampler = None
        self._next_sample = 0.0
        self.last_trip = None                               # {"axis", "current", "limit", "phase", "t"} of the last current stop
        self.thread = threading.Thread(target=self._run, name="gimbal", daemon=True)

    # ======================================================================= public API
    def set_sampler(self, cfg: dict | None) -> Future:
        """Current characterisation (current_char.py). cfg = None restores the normal polling. Otherwise
        both axes are read together (position + current; state at most every state_every_s) at
        moving_hz while an axis moves and hold_hz while both hold, never within quiet_enable_s of an
        enable, quiet_start_s of a standalone START or quiet_track_s of a tracking START.
        cfg["on_sample"](dict) is called on the gimbal thread (keep it quick, put it on a queue)."""
        return self.call(self._do_set_sampler, cfg)

    def _do_set_sampler(self, cfg):
        self.sampler = dict(cfg) if cfg else None
        self._next_sample = time.monotonic()
        for r in self.run.values():
            r.next_poll = time.monotonic()
            r.next_state_read = 0.0
        return "on" if cfg else "off"

    def start(self):
        self.thread.start()

    def call(self, fn, *args) -> Future:
        f = Future()
        self._q.append((f, fn, args))
        return f

    def beat(self):
        """Heartbeat from the web page (deadman)."""
        self._beat = time.monotonic()

    def press(self, axis: str, direction: int) -> Future:
        """Latching key: start / stop / queue a move of one axis."""
        self.beat()
        return self.call(self._do_key, axis, 1 if direction > 0 else -1)

    def stop_all(self, why: str = "STOP") -> Future:
        return self.call(self._do_stop_all, why)

    def begin(self) -> Future:
        """The Start button: setup both drives, wait, wake X then Y."""
        return self.call(self._do_begin)

    def home(self) -> Future:
        """The Home button: both axes to 0/0 together (absolute), at the current speed."""
        return self.call(self._do_home)

    def set_speed(self, v: float) -> Future:
        return self.call(self._do_speed, float(v))

    def goto(self, x: float, y: float, label: str = "goto", safety: bool = False) -> Future:
        """Absolute move of both axes (axis degrees). Result: the activity dict; wait with wait_act().
        safety=True (moving off the Sun): not stopped by the page deadman, only by STOP."""
        return self.call(self._do_act, "goto", (float(x), float(y)), label, safety)

    def track(self, fn, label: str = "track") -> Future:
        """fn(datetime_utc) -> (x_deg, y_deg) or None. Followed until stop_act()/STOP. Replaces any
        running goto/track without stopping the motors."""
        return self.call(self._do_act, "track", fn, label, False)

    def stop_act(self, why="activity stopped") -> Future:
        """End goto/track: STOP any step in progress (motors stay ON and holding)."""
        return self.call(self._do_stop_act, why)

    def wait_act(self, act: dict, timeout: float, cancel=None) -> bool:
        """Wait for a goto to finish. True = arrived; False = stopped / failed / timed out / cancelled."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if act["event"].wait(0.1):
                return bool(act["ok"])
            if cancel and cancel():
                return False
        return False

    def shutdown(self, timeout=6.0) -> str:
        """Stop both (motors stay ON and holding), then end the thread."""
        f = self.call(self._do_shutdown)
        try:
            res = f.result(timeout=timeout)
        except Exception as e:
            res = f"shutdown: {e}"
        self._stop.set()
        self.thread.join(2)
        return res

    def angles(self):
        return self.axes["x"].angle, self.axes["y"].angle

    @property
    def holding(self) -> bool:
        """Both motors ON, nothing moving, no sequence running: ready for goto/track (Sun steps)."""
        return (self.mode == "on" and self.started and self._seq is None and self.act is None
                and all(r.phase == "idle" and r.state in (1, 3, 4) for r in self.run.values()))

    def pose_at(self, tm: float | None = None):
        """Best estimate of (x, y) axis degrees at monotonic time tm: the planned step profile while a
        step runs, otherwise interpolated position reads. None if unknown."""
        tm = time.monotonic() if tm is None else tm
        out = []
        for k in ("x", "y"):
            r = self.run[k]
            val = None
            for m in reversed(list(r.moves)):
                if m[0] <= tm <= m[5]:
                    val = trapezoid(tm, *m[:5])
                    break
            if val is None:
                h = list(r.hist)
                if not h:
                    val = self.axes[k].angle
                elif tm >= h[-1][0]:
                    val = h[-1][1]
                    m = r.moves[-1] if r.moves else None
                    if m and tm > m[5] >= h[-1][0]:
                        val = m[2]                                 # finished step, end not read yet
                else:
                    val = h[0][1]
                    for (t0, a0), (t1, a1) in zip(h, h[1:]):
                        if t0 <= tm <= t1:
                            val = a0 + (a1 - a0) * (tm - t0) / max(t1 - t0, 1e-6)
                            break
            if val is None:
                return None
            out.append(val)
        return tuple(out)

    def act_target(self, t=None):
        """Current goto/track target (x, y) or None."""
        a = self.act
        if a is None:
            return None
        if a["kind"] == "goto":
            return a["target"]
        try:
            return a["fn"](t or clock.now())
        except Exception:
            return None

    def settled(self, tol=0.05) -> bool:
        """Activity running, no step in progress, both axes within tol of the target now."""
        a = self.act
        if a is None or any(r.mv is not None for r in self.run.values()):
            return False
        tgt = self.act_target()
        if tgt is None:
            return False
        lims = [self.axes[k].limits for k in ("x", "y")]
        tgt = [max(lo, min(hi, v)) for v, (lo, hi) in zip(tgt, lims)]
        return all(self.axes[k].angle is not None and abs(self.axes[k].angle - v) <= tol
                   for k, v in zip(("x", "y"), tgt))

    def snapshot(self) -> dict:
        now = time.monotonic()
        ax = {}
        for k, a in self.axes.items():
            r = self.run[k]
            lo, hi = a.limits
            ax[k] = {"node": a.cfg.node_id, "model": a.cfg.model, "found": r.found,
                     "angle": a.angle, "raw": r.raw, "current": r.current,
                     "state": r.state, "state_name": STATE_NAMES.get(r.state, "?" if r.state is None else str(r.state)),
                     "temp": r.temp, "err": r.err, "err_text": _err_text(r.err) if r.err else "",
                     "phase": r.phase, "latch": r.latch, "pending": r.pending, "on_expected": r.on_expected,
                     "msg": r.msg, "msg_level": r.msg_level, "refusal": dict(r.refusal),
                     "min": lo, "max": hi, "home": a.home_counts, "speed": r.speed,
                     "current_limit": a.cfg.current_limit_a, "stepping": r.mv is not None}
        act = self.act
        act_s = None
        if act is not None:
            tgt = self.act_target()
            act_s = {"kind": act["kind"], "label": act["label"], "msg": act["msg"], "clipped": act["clipped"],
                     "safety": act["safety"], "target": list(tgt) if tgt else None}
        return {"mode": self.mode, "fault": self.fault, "started": self.started, "start_step": self.start_step,
                "link": getattr(self.link, "name", "?"), "bus_v": self.bus_v, "axes": ax,
                "speed": self.speed, "speed_pending": self.speed_pending, "live_speed_ok": self.live_speed_ok,
                "speeds": list(config.MANUAL_SPEEDS_DEG_S), "holding": self.holding,
                "homing": self._seq_kind == "home" and self._seq is not None, "home_result": self.home_result,
                "act": act_s,
                "heartbeat_age": None if self._beat is None else round(now - self._beat, 2),
                "deadman_s": config.PAGE_DEADMAN_S}

    # ======================================================================= internals
    def _event(self, msg, level="info"):
        self.events.append({"t": clock.now().isoformat(timespec="seconds"), "level": level, "msg": msg})
        (log.info if level == "info" else log.warning)(msg)

    def _axis_msg(self, k, msg, level="info", event=True):
        r = self.run[k]
        r.msg, r.msg_level = msg, level
        if event and msg:
            self._event(msg, level)

    def _run(self):
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                self._step()
            except Exception as e:                           # keep the owner thread alive
                log.exception("gimbal loop")
                self._event(f"controller error: {e}", "error")
            time.sleep(max(0.0, TICK_S - (time.monotonic() - t0)))

    def _step(self):
        while self._q:
            f, fn, args = self._q.popleft()
            try:
                f.set_result(fn(*args))
            except Exception as e:
                f.set_exception(e)
        now = time.monotonic()
        if self.mode == "offline":
            if now >= self._next_connect:
                self._next_connect = now + 2.0
                self._try_connect()
            return
        if self._seq is not None and now >= self._seq_resume:
            self._seq_step()
        self._deadman(now)
        if self.act is not None and self.mode != "offline":
            try:
                self._act_tick(time.monotonic())
            except COMM_ERRORS as e:
                self._act_fail(f"no reply from a drive during {self.act['label'] if self.act else 'activity'}: {e}")
        for k in self.axes:
            if self.mode == "offline":
                return
            self._axis_step(k, time.monotonic())
        if self.sampler is not None and self.mode != "offline":
            self._sample_tick(time.monotonic())
        if now >= self._next_volts:
            self._next_volts = now + config.SLOW_POLL_PERIOD_S
            try:
                self.bus_v = self.axes["x"].drive.bus_volts()
            except COMM_ERRORS:
                pass

    # ---- connection ---------------------------------------------------------------------
    def _try_connect(self):
        """Find both nodes (0x0043), read position/state/error/voltage/temperature. Doesn't move anything."""
        found = []
        for k, a in self.axes.items():
            r = self.run[k]
            try:
                nid = a.drive.node_id()
                r.found = nid == a.cfg.node_id
                if not r.found:
                    self._axis_msg(k, f"node {a.cfg.node_id} answered with id {nid}", "warn")
                    continue
                self._read_axis(k, full=True)
                r.err = a.drive.error()
                r.on_expected = r.state not in (None, 0)
                r.fails, r.phase, r.latch, r.pending = 0, "idle", 0, 0
                found.append(k)
            except COMM_ERRORS as e:
                r.found = False
                self._axis_msg(k, f"node {a.cfg.node_id} ({k.upper()}) not answering: {e}", "warn", event=False)
        if len(found) == len(self.axes):
            try:
                self.bus_v = self.axes["x"].drive.bus_volts()
            except COMM_ERRORS:
                pass
            self._next_volts = time.monotonic() + config.SLOW_POLL_PERIOD_S
            self.mode = "fault" if self.fault else "ready"
            self.started = False
            for i, k in enumerate(self.axes):
                self.run[k].next_poll = time.monotonic() + 0.5 * i
                self.run[k].next_slow = time.monotonic() + config.SLOW_POLL_PERIOD_S + 0.5 * i
                st, err = self.run[k].state, self.run[k].err
                self._axis_msg(k, f"{k.upper()} found: state {STATE_NAMES.get(st, st)}, error {_err_text(err) if err else 'none'}", event=False)
            v = f"{self.bus_v:.1f} V" if self.bus_v else "? V"
            self._event(f"drives answering on {self.link.name}: node 1 (X) + node 2 (Y), bus {v}. Press Start.")

    def _read_pos(self, k):
        a, r = self.axes[k], self.run[k]
        a.read()
        r.raw = a.raw_counts
        r.pos_t = time.monotonic()
        r.hist.append((r.pos_t, a.angle))

    def _read_axis(self, k, full=False, pos=True):
        a, r = self.axes[k], self.run[k]
        if pos:
            self._read_pos(k)
        r.current = a.drive.current_a()
        r.state = a.drive.state()
        if full:
            r.temp = a.drive.temperature_c()
        self._log_motor(k)

    def _log_motor(self, k):
        """Append the values just read to logs/motor_<UTC date>.csv (no extra CAN traffic, doesn't raise)."""
        try:
            import os
            a, r = self.axes[k], self.run[k]
            t = clock.now()
            os.makedirs(config.LOG_DIR, exist_ok=True)
            path = os.path.join(config.LOG_DIR, f"motor_{t:%Y%m%d}.csv")
            new = not os.path.exists(path)
            act = self.act["kind"] if self.act is not None else ""
            link = str(getattr(self.link, "name", "?")).replace(",", " ")
            with open(path, "a") as f:
                if new:
                    f.write("time_utc,axis,angle_deg,current_a,state,temp_c,phase,mode,act,link\n")
                ang = "" if a.angle is None else f"{a.angle:.4f}"
                cur = "" if r.current is None else f"{r.current:.3f}"
                f.write(f"{t.isoformat(timespec='milliseconds')},{k},{ang},{cur},{'' if r.state is None else r.state},"
                        f"{'' if r.temp is None else r.temp},{r.phase},{self.mode},{act},{link}\n")
        except Exception:
            pass

    def _comm_fail(self, k, e):
        r = self.run[k]
        r.fails += 1
        if r.fails >= 3:
            self._event(f"node {self.axes[k].cfg.node_id} ({k.upper()}) stopped answering: {e}", "error")
            if self.act is not None:
                self._end_act(False, f"drive {k.upper()} stopped answering")
            for kk in self.axes:
                rr = self.run[kk]
                if rr.phase in ("starting", "moving", "auto") and (rr.start_ok or rr.mv):
                    try:
                        self.axes[kk].drive.stop()
                    except COMM_ERRORS:
                        pass
                rr.phase, rr.latch, rr.pending, rr.start_ok, rr.mv = "idle", 0, 0, False, None
            self._seq, self._seq_kind = None, None
            self.mode, self.started = "offline", False
            self._next_connect = time.monotonic() + 2.0

    # ---- per-axis state machine ------------------------------------------------------------
    def _axis_step(self, k, now):
        a, r = self.axes[k], self.run[k]
        if r.phase in ("waking", "homing"):
            return                                             # the Start / Home sequence reads this axis itself
        try:
            if r.phase == "starting":
                self._starting_step(k, now)
            elif r.phase == "stopping":
                self._stopping_step(k, now)
            # if now >= r.next_poll:
            if now >= r.next_poll and self.sampler is None:    # the sampler replaces this poll (_sample_tick)
                if r.phase == "auto":                          # positions come from the step reads
                    r.next_poll = now + config.TRACK_POLL_PERIOD_S
                    self._read_axis(k, pos=now - r.pos_t > 1.5 and r.mv is None)
                else:
                    r.next_poll = now + config.POLL_PERIOD_S
                    self._read_axis(k)
                r.fails = 0
                self._check_axis(k)
            if now >= r.next_slow:
                r.next_slow = now + config.SLOW_POLL_PERIOD_S
                r.temp = a.drive.temperature_c()
        except COMM_ERRORS as e:
            self._comm_fail(k, e)

    def _check_axis(self, k):
        """After a poll: drop-out, current limit, end of a latched move (soft limit)."""
        a, r = self.axes[k], self.run[k]
        if r.state == 0 and r.on_expected:
            self._dropout(k)
            return
        if r.state not in (None, 0) and not r.on_expected and self.started:
            r.on_expected = True
        lim = a.cfg.current_limit_a
        if r.current is not None and abs(r.current) > lim:
            if r.phase in ("starting", "moving", "auto"):
                self.last_trip = {"axis": k, "current": r.current, "limit": lim, "phase": r.phase,
                                  "angle": a.angle, "t": time.monotonic()}
            if r.phase in ("starting", "moving"):
                self._stop_axis(k, f"{k.upper()} current {r.current:+.2f} A is over the {lim:.0f} A limit - stopped", "error")
            elif r.phase == "auto":
                self._act_fail(f"{k.upper()} current {r.current:+.2f} A is over the {lim:.0f} A limit - stopped")
            elif time.monotonic() - r.over_warned > 10:
                r.over_warned = time.monotonic()
                self._axis_msg(k, f"{k.upper()} current {r.current:+.2f} A is over the {lim:.0f} A limit while holding", "warn")
        if r.phase == "moving" and r.state in (1, 3, 4):
            self._finished(k)

    def _mark_start(self, k, kind):
        """Remember when a START (and, for kind 'enable', an enable) went to axis k (sampler quiet time)."""
        r = self.run[k]
        r.t_start = time.monotonic()
        r.start_kind = kind
        if kind == "enable":
            r.t_enable = r.t_start

    def _axis_moving(self, k):
        r = self.run[k]
        return (r.mv is not None or r.state == 2 or r.phase in ("starting", "moving", "stopping", "waking", "homing"))

    def _sample_tick(self, now):
        """Current characterisation: read both axes together (see set_sampler). Doesn't raise."""
        s = self.sampler
        if s is None or now < self._next_sample:
            return
        if self._seq is not None:                              # Start / Home read the axes themselves
            return
        quiet = {"enable": s.get("quiet_enable_s", 1.0), "start": s.get("quiet_start_s", 1.0),
                 "track": s.get("quiet_track_s", 0.3)}
        for r in self.run.values():
            if now - r.t_enable < quiet["enable"] or now - r.t_start < quiet.get(r.start_kind, quiet["start"]):
                return                                         # bench rule: no polling right after enable / START
        moving = [k for k in self.axes if self._axis_moving(k)]
        fast = s.get("fast_until", 0.0) > now
        hz = s.get("fast_hz", 4.0) if fast else (s.get("moving_hz", 2.0) if moving else s.get("hold_hz", 1.0))
        self._next_sample = now + 1.0 / max(hz, 1e-3)
        only = s.get("fast_axes") if fast else None
        out = {"t_mono": now, "t_utc": clock.now(), "axes": {}}
        for k, a in self.axes.items():
            r = self.run[k]
            if only and k not in only:
                out["axes"][k] = {"read": False}
                continue
            try:
                self._read_pos(k)
                r.current = a.drive.current_a()
                if now >= r.next_state_read or r.phase in ("starting", "stopping"):
                    r.next_state_read = now + s.get("state_every_s", 1.0)
                    r.state = a.drive.state()
                r.fails = 0
                self._log_motor(k)
                self._check_axis(k)
            except COMM_ERRORS as e:
                self._comm_fail(k, e)
                if self.mode == "offline":
                    return
            out["axes"][k] = {"read": True}
        for k, a in self.axes.items():
            r = self.run[k]
            mv = r.mv or (r.moves[-1] if r.moves and r.moves[-1][5] > now else None)
            d = 0
            if mv is not None and self._axis_moving(k):
                d = 1 if mv[2] > mv[1] else (-1 if mv[2] < mv[1] else 0)
            elif r.phase in ("moving", "starting", "stopping") and r.latch:
                d = r.latch
            out["axes"][k].update({"angle": a.angle, "current": r.current, "state": r.state, "temp": r.temp,
                                   "phase": r.phase, "moving": self._axis_moving(k), "dir": d, "speed": r.speed})
        out.update(bus_v=self.bus_v, accel_counts=self.accel_counts, mode=self.mode,
                   act=(self.act["kind"] if self.act is not None else ""))
        try:
            s["on_sample"](out)
        except Exception:
            log.exception("sampler callback")

    def _finished(self, k):
        a, r = self.axes[k], self.run[k]
        lo, hi = a.limits
        d = r.latch
        r.phase, r.latch, r.start_ok = "idle", 0, False
        lim = hi if d > 0 else lo
        if a.angle is not None and abs(a.angle - lim) < 1.0:
            self._axis_msg(k, f"{k.upper()} at {lim:+.0f}\N{DEGREE SIGN} limit", "warn")
        else:
            self._axis_msg(k, f"{k.upper()} stopped by itself at {a.angle:+.3f}\N{DEGREE SIGN} "
                              f"({STATE_NAMES.get(r.state, r.state)})", "warn")

    def _dropout(self, k):
        a, r = self.axes[k], self.run[k]
        try:
            r.err = a.drive.error()
        except COMM_ERRORS:
            r.err = None
        was_auto = r.phase == "auto"
        r.on_expected, r.phase, r.latch, r.pending, r.start_ok, r.mv = False, "idle", 0, 0, False, None
        r.off_t = time.monotonic()
        self._axis_msg(k, f"{k.upper()} switched its motor OFF (error {_err_text(r.err)}). "
                          f"No automatic retry: press a key to move it again, or Start.", "error")
        if was_auto or self.act is not None:
            self._act_fail(f"{k.upper()} motor dropped out (error {_err_text(r.err)})")

    # ---- latching keys -----------------------------------------------------------------------
    def _do_key(self, k, d):
        if k not in self.axes:
            raise ValueError(f"unknown axis {k!r}")
        a, r = self.axes[k], self.run[k]
        if self.mode == "offline":
            raise RuntimeError("Drives not answering")
        if self.mode == "starting":
            raise RuntimeError("Start sequence still running")
        if self._seq_kind == "home" and self._seq is not None:
            raise RuntimeError("Homing: press STOP first")
        if self.act is not None:
            raise RuntimeError(f"{self.act['label']} is running: Abort / STOP first")
        if not self.started:
            raise RuntimeError("Press Start first (sets up and wakes both motors)")
        if r.phase in ("starting", "moving"):
            same = d == r.latch
            self._stop_axis(k, f"{k.upper()} stop ({'same key' if same else 'opposite key: press it again to reverse'})")
            return "stopping"
        if r.phase == "stopping":
            r.pending = 0 if r.pending == d else d
            return "queued" if r.pending else "unqueued"
        lo, hi = a.limits
        if a.angle is not None and ((d > 0 and a.angle >= hi - config.MIN_MOVE_DEG) or
                                    (d < 0 and a.angle <= lo + config.MIN_MOVE_DEG)):
            self._refuse(k, d, f"{k.upper()} at {(hi if d > 0 else lo):+.0f}\N{DEGREE SIGN} limit: use the other key")
            return "at limit"
        self._begin_start(k, d)
        try:
            self._starting_step(k, time.monotonic())           # first attempt right away
        except COMM_ERRORS as e:
            if not r.start_ok:
                r.phase, r.latch = "idle", 0
            self._refuse(k, d, f"{k.upper()} {'+' if d > 0 else '-'}: no reply from the drive ({e})")
            raise
        return r.phase

    def _begin_start(self, k, d):
        r = self.run[k]
        r.phase, r.latch, r.pending, r.start_ok = "starting", d, 0, False
        r.start_retry_until = time.monotonic() + config.START_RETRY_S
        r.next_attempt = max(time.monotonic(), r.off_t + config.SETUP_TO_ENABLE_S)
        self._axis_msg(k, "", event=False)

    def _refuse(self, k, d, msg):
        r = self.run[k]
        r.refusal = {"seq": r.refusal["seq"] + 1, "dir": d, "msg": msg}
        self._axis_msg(k, msg, "error")

    def _starting_step(self, k, now):
        a, r = self.axes[k], self.run[k]
        if not r.start_ok:
            if now < r.next_attempt:
                return
            self._read_pos(k)
            lo, hi = a.limits
            target_deg = (hi if r.latch > 0 else lo) - a.angle          # relative, up to the soft limit
            if abs(target_deg) < config.MIN_MOVE_DEG:
                d = r.latch
                r.phase, r.latch = "idle", 0
                self._refuse(k, d, f"{k.upper()} at {(hi if d > 0 else lo):+.0f}\N{DEGREE SIGN} limit")
                return
            counts = deg_to_counts(a.cfg.sign * target_deg)
            try:
                a.drive.enable_and_move(counts)
                self._mark_start(k, "enable")
            except DriveError as e:
                if e.code == 0x15 and now < r.start_retry_until:      # still decelerating: retry
                    r.next_attempt = now + 0.25
                    return
                d = r.latch
                r.phase, r.latch = "idle", 0
                self._refuse(k, d, f"{k.upper()} {'+' if d > 0 else '-'} refused: {e}")
                return
            r.start_ok = True
            r.on_expected = True
            r.confirm_at = now + 0.3
            r.confirm_until = now + 2.5
            self._plan_move(k, now, a.angle, a.angle + target_deg)
            self._event(f"{k.upper()} {'+' if r.latch > 0 else '-'}: START accepted, "
                        f"{target_deg:+.2f}\N{DEGREE SIGN} to the soft limit ({counts:+d} counts)")
            return
        if now < r.confirm_at:
            return
        r.confirm_at = now + 0.3
        st = a.drive.state()
        r.state = st
        if st == 2:
            r.phase = "moving"
        elif st == 0:
            self._dropout(k)
        elif now > r.confirm_until:
            d = r.latch
            r.phase, r.latch, r.start_ok = "idle", 0, False
            self._refuse(k, d, f"{k.upper()}: START accepted but the drive reports '{STATE_NAMES.get(st, st)}', not moving")

    def _stop_axis(self, k, why, level="info"):
        a, r = self.axes[k], self.run[k]
        if r.phase == "starting" and not r.start_ok:              # never got going, just cancel
            r.phase, r.latch, r.pending = "idle", 0, 0
            self._axis_msg(k, why, level)
            return
        if r.phase not in ("starting", "moving"):
            return
        r.pending = 0
        try:
            a.drive.stop()
        except DriveError as e:
            self._event(f"{k.upper()}: STOP (0x84) refused: {e}", "error")
        except COMM_ERRORS as e:
            self._event(f"{k.upper()}: STOP (0x84) not answered: {e}", "error")
        self._cut_move(k)
        r.phase = "stopping"
        r.stop_t = time.monotonic()
        v = r.speed or self.speed
        decel_s = deg_to_counts(v) / max(self.accel_counts, 1)
        r.next_check = r.stop_t + config.STOP_CHECK_S
        # still state 2 after 0.6 s + 1.5 x decel time + 0.5 s means the STOP was not obeyed
        r.stop_deadline = r.stop_t + config.STOP_CHECK_S + 1.5 * decel_s + 0.5
        self._axis_msg(k, why, level)

    def _stopping_step(self, k, now):
        a, r = self.axes[k], self.run[k]
        if now < r.next_check:
            return
        r.next_check = now + 0.3
        st = a.drive.state()
        r.state = st
        if st != 2:
            r.phase, r.latch, r.start_ok = "idle", 0, False
            if st == 0 and r.on_expected:
                self._dropout(k)
                return
            self._read_pos(k)                                  # final position (the 1 Hz poll may be stale)
            note = f"{k.upper()} stopped ({STATE_NAMES.get(st, st)}) after {now - r.stop_t:.1f} s"
            if r.msg_level == "info":
                self._axis_msg(k, note)
            else:                                              # keep the warning (current limit, deadman) on screen
                self._event(note)
            if r.pending:
                d, r.pending = r.pending, 0
                self._begin_start(k, d)
            return
        if now >= r.stop_deadline:
            self._stop_not_obeyed(k, now - r.stop_t)

    def _stop_not_obeyed(self, k, waited):
        """STOP (0x84) did not stop the move: the only case where the software switches a motor off."""
        a, r = self.axes[k], self.run[k]
        try:
            a.drive.disable()
        except COMM_ERRORS as e:
            self._event(f"{k.upper()}: motor-off after unobeyed STOP failed: {e}", "error")
        r.phase, r.latch, r.pending, r.start_ok, r.on_expected = "idle", 0, 0, False, False
        r.off_t = time.monotonic()
        self.fault = (f"{k.upper()}: STOP (0x84) not obeyed - still moving {waited:.1f} s after STOP. "
                      f"{k.upper()} motor switched OFF. Support the load, then press Start.")
        self._axis_msg(k, self.fault, "error")
        for kk in self.axes:
            if kk != k:
                self._stop_axis(kk, f"{kk.upper()} stopped because of the {k.upper()} fault")
        self.mode, self.started = "fault", False

    def _do_stop_all(self, why="STOP"):
        stopped = []
        if self._seq is not None:
            kind = self._seq_kind
            self._seq.close()
            self._seq, self._seq_kind = None, None
            self.start_step = ""
            for k, r in self.run.items():
                if r.phase in ("waking", "homing"):
                    r.phase, r.start_ok = "moving", True       # STOP + stop check below
            if kind == "home":
                self.home_result = {"ok": False, "msg": f"Home stopped ({why})", "err_counts": None}
                self.started = all(r.on_expected for r in self.run.values())
                self.mode = "fault" if self.fault else ("on" if self.started else "ready")
                self._event(f"{why}: Home stopped", "warn")
            else:
                self.mode = "fault" if self.fault else "ready"
                self.started = False
                self._event(f"{why}: Start sequence aborted - press Start again", "warn")
        if self.act is not None:
            stopped += self._halt_act(f"{why}")
            self._end_act(False, why)
        for k, r in self.run.items():
            r.pending = 0
            if r.phase in ("starting", "moving"):
                self._stop_axis(k, f"{k.upper()} stop ({why})", "warn")
                if k.upper() not in stopped:
                    stopped.append(k.upper())
        if stopped:
            self._event(f"{why}: stopping {' + '.join(stopped)}", "warn")
        return stopped

    def _deadman(self, now):
        if self._beat is None or now - self._beat <= config.PAGE_DEADMAN_S:
            return
        act_busy = self.act is not None and not self.act["safety"]
        busy = (self._seq is not None or act_busy or
                any(r.phase in ("starting", "moving") or r.pending for r in self.run.values()))
        if busy:
            self._do_stop_all(f"no heartbeat from the web page for {config.PAGE_DEADMAN_S:.0f} s")

    # ---- speed ------------------------------------------------------------------------------------
    def _do_speed(self, v):
        if v not in config.MANUAL_SPEEDS_DEG_S:
            raise ValueError(f"speed must be one of {config.MANUAL_SPEEDS_DEG_S}")
        if self._seq is not None or self.act is not None or any(r.phase != "idle" for r in self.run.values()):
            raise RuntimeError("Stop both axes before changing the speed")
        self.speed = v
        if not self.started:
            self.speed_pending = None
            self._event(f"speed {v:.1f} \N{DEGREE SIGN}/s (applied at Start / Home)")
            return "at start"
        refused = []
        for k, a in self.axes.items():
            if self.run[k].speed == v:
                continue
            try:
                a.drive.set_speed(v)
                self.run[k].speed = v
            except DriveError as e:
                if e.code not in (0x15, 0x16):
                    raise
                refused.append(f"{k.upper()} {_err_text(e.code)}")
        if refused:
            self.live_speed_ok = False
            self.speed_pending = v
            self._event(f"drive refused a live speed change ({', '.join(refused)}). To use {v:.1f} \N{DEGREE SIGN}/s: "
                        f"support the load, then press Start (motors switch off for a moment).", "warn")
            return "needs start"
        self.live_speed_ok = True
        self.speed_pending = None
        self._event(f"speed {v:.1f} \N{DEGREE SIGN}/s (live 0x8A write accepted)")
        return "live"

    # ---- sequences (Start, Home) ------------------------------------------------------------------
    def _do_begin(self):
        if self.mode == "offline":
            raise RuntimeError("Drives not answering")
        if self._seq is not None:
            raise RuntimeError("Start/Home already running")
        if self.act is not None or any(r.phase != "idle" for r in self.run.values()):
            raise RuntimeError("Stop both axes first")
        self.fault = None
        self.mode, self.started = "starting", False
        self._run_seq("start", self._start_seq())
        return "starting"

    def _do_home(self):
        if self.mode == "offline":
            raise RuntimeError("Drives not answering")
        if self._seq is not None:
            raise RuntimeError("Start/Home already running")
        if self.act is not None or any(r.phase != "idle" for r in self.run.values()):
            raise RuntimeError("Stop both axes first")
        if not all(a.homed for a in self.axes.values()):
            raise RuntimeError("HOME_COUNTS not set in config.py")
        self.home_result = None
        self.mode = "homing"
        self._run_seq("home", self._home_seq())
        return "homing"

    def _run_seq(self, kind, gen):
        self._seq, self._seq_kind = gen, kind
        self._seq_resume = time.monotonic()
        self._seq_step()

    def _seq_step(self):
        kind = self._seq_kind
        try:
            delay = next(self._seq)
            self._seq_resume = time.monotonic() + delay
        except StopIteration:
            self._seq, self._seq_kind = None, None
        except Exception as e:
            self._seq, self._seq_kind = None, None
            for k, r in self.run.items():
                if r.phase in ("waking", "homing"):
                    try:
                        self.axes[k].drive.stop()
                    except COMM_ERRORS:
                        pass
                    r.phase = "idle"
            self.fault = f"{'Home' if kind == 'home' else 'Start'} failed: {e}"
            if kind == "home":
                self.home_result = {"ok": False, "msg": self.fault, "err_counts": None}
            self.mode, self.started, self.start_step = "fault", False, ""
            self._event(self.fault, "error")

    def _wait_state(self, k, end, what):
        """Generator helper: poll 0x010C every 0.3 s until the move is over (state != 2)."""
        r = self.run[k]
        a = self.axes[k]
        while True:
            yield 0.3
            st = a.drive.state()
            r.state = st
            if st == 4:                                        # completed move: holding
                return
            if st == 0:
                r.on_expected = False
                r.err = a.drive.error()
                raise RuntimeError(f"{k.upper()} switched its motor OFF during {what} (error {_err_text(r.err)})")
            if time.monotonic() > end:
                raise RuntimeError(f"{k.upper()} {what} did not finish in time (state {st})")

    def _start_seq(self):
        """Generator: yields the seconds to wait before the next step (the loop keeps serving STOP)."""
        v = self.speed
        self.start_step = "setup (motors off)"
        for k, a in self.axes.items():
            a.drive.configure(v, accel_counts=self.accel_counts)
            r = self.run[k]
            r.on_expected, r.speed, r.off_t = False, v, time.monotonic()
        self.speed_pending = None
        self._event(f"setup both drives: {v:.1f} \N{DEGREE SIGN}/s, accel 0x{self.accel_counts:X} - motors off")
        yield config.SETUP_TO_ENABLE_S
        for k, a in self.axes.items():
            r = self.run[k]
            self._read_pos(k)
            p0 = a.raw_counts
            wake = deg_to_counts(config.WAKE_STEP_DEG) * a.cfg.sign
            for label in ("+", "back"):
                self._read_pos(k)
                delta = wake if label == "+" else p0 - a.raw_counts
                self.start_step = f"wake {k.upper()} ({'+%.1f' % config.WAKE_STEP_DEG if label == '+' else 'back'})"
                r.phase = "waking"
                a.drive.enable_and_move(delta)
                self._mark_start(k, "enable")
                r.on_expected = True
                self._plan_move(k, time.monotonic(), a.angle, a.angle + a.cfg.sign * counts_to_deg(delta))
                yield from self._wait_state(k, time.monotonic() + config.WAKE_TIMEOUT_S, "wake")
                self._read_pos(k)
            r.err = a.drive.error()                            # the power-up code clears on enable
            r.phase = "idle"
            r.next_poll = time.monotonic() + 0.5
            self._axis_msg(k, f"{k.upper()} awake: motor ON and holding at {a.angle:+.3f}\N{DEGREE SIGN}")
        self.mode, self.started, self.start_step = "on", True, ""
        self._event("Start done: both motors ON and holding")

    def _home_seq(self):
        """Both axes to HOME (0/0) together: target = HOME - raw (counts, absolute). A motor that is ON
        and holding is not switched off: just 0100=1 (harmless), target, START. A motor that is OFF:
        setup, 1 s, move. Standalone moves under 0.5 deg from OFF go +1 deg beyond and back."""
        v = self.speed
        off = [k for k, r in self.run.items() if r.state in (None, 0) or not r.on_expected]
        if off:
            self.start_step = f"home: setup {' + '.join(k.upper() for k in off)} (motor off)"
            for k in off:
                self.axes[k].drive.configure(v, accel_counts=self.accel_counts)
                r = self.run[k]
                r.on_expected, r.speed, r.off_t = False, v, time.monotonic()
            self._event(f"Home: setup {' + '.join(k.upper() for k in off)} at {v:.1f} \N{DEGREE SIGN}/s - motor off, 1 s pause")
            yield config.SETUP_TO_ENABLE_S
        steps = {}
        min_c = deg_to_counts(config.MIN_MOVE_DEG)
        for k, a in self.axes.items():
            self._read_pos(k)
            delta = wrap_counts(a.home_counts - a.raw_counts)          # raw counts to go
            if delta == 0:
                continue
            if k in off and abs(delta) < min_c:                        # no move under 0.5 deg from OFF
                extra = deg_to_counts(1.0) * (1 if delta > 0 else -1)
                steps[k] = [delta + extra, -extra]
            else:
                steps[k] = [delta]
        self.start_step = "homing"
        end = time.monotonic() + config.HOME_TIMEOUT_S
        moving = {}
        while any(steps.values()) or moving:
            for k in list(steps):
                if steps[k] and k not in moving:
                    a, r = self.axes[k], self.run[k]
                    c = steps[k].pop(0)
                    r.phase = "homing"
                    a.drive.enable_and_move(c)                 # 0100=1 (harmless when holding), 0087, 0083
                    self._mark_start(k, "enable")
                    r.on_expected = True
                    self._plan_move(k, time.monotonic(), a.angle, a.angle + a.cfg.sign * counts_to_deg(c))
                    moving[k] = True
            yield 0.3
            for k in list(moving):
                a, r = self.axes[k], self.run[k]
                st = a.drive.state()
                r.state = st
                if st == 0:
                    r.on_expected = False
                    r.err = a.drive.error()
                    raise RuntimeError(f"{k.upper()} switched its motor OFF while homing (error {_err_text(r.err)})")
                if st == 4:                                    # completed move: holding
                    self._read_pos(k)
                    del moving[k]
            if time.monotonic() > end:
                raise RuntimeError(f"Home did not finish in {config.HOME_TIMEOUT_S:.0f} s")
        err = {}
        for k, a in self.axes.items():
            self._read_pos(k)
            err[k] = wrap_counts(a.home_counts - a.raw_counts)
            r = self.run[k]
            r.phase = "idle"
            r.next_poll = time.monotonic() + 0.5
        self.started = all(r.on_expected for r in self.run.values())
        self.mode, self.start_step = ("on" if self.started else "ready"), ""
        ok = all(abs(e) <= 50 for e in err.values())
        self.home_result = {"ok": ok, "err_counts": err,
                            "msg": f"Home done: error X {err['x']:+d}, Y {err['y']:+d} counts"}
        self._event(self.home_result["msg"] + ("" if ok else " (more than 50 counts!)"), "info" if ok else "warn")

    # ---- goto / track ----------------------------------------------------------------------------
    def _do_act(self, kind, target, label, safety):
        if self.mode != "on" or not self.started:
            raise RuntimeError("Motors must be ON and holding (press Start)")
        if self._seq is not None:
            raise RuntimeError("Start/Home running")
        if any(r.phase not in ("idle", "auto") for r in self.run.values()):
            raise RuntimeError("Stop the manual moves first")
        if kind == "goto":
            x, y = target
            for k, v in (("x", x), ("y", y)):
                lo, hi = self.axes[k].limits
                if not lo - 1e-9 <= v <= hi + 1e-9:
                    raise ValueError(f"{label}: {k.upper()} target {v:+.2f}\N{DEGREE SIGN} is beyond the "
                                     f"{lo:+.0f}..{hi:+.0f}\N{DEGREE SIGN} soft limit - refused")
        self._act_seq += 1
        old = self.act
        act = {"id": self._act_seq, "kind": kind, "label": label, "safety": safety, "clipped": False, "msg": "",
               "target": target if kind == "goto" else None, "fn": target if kind == "track" else None,
               "event": threading.Event(), "ok": None, "why": None, "t0": time.monotonic()}
        if old is not None:
            old["ok"], old["why"] = True, "replaced"
            old["event"].set()
        self.act = act
        for k, r in self.run.items():
            if r.phase == "idle":
                r.phase = "auto"
                r.next_step = time.monotonic()
                r.busy_since = None
        return act

    def _do_stop_act(self, why):
        if self.act is None:
            return []
        stopped = self._halt_act(why)
        self._end_act(False, why)
        return stopped

    def _halt_act(self, why):
        """STOP any step in progress (with the STOP-not-obeyed check); other axes back to idle."""
        stopped = []
        for k, r in self.run.items():
            if r.phase != "auto":
                continue
            if r.mv is not None:
                r.phase, r.start_ok, r.latch = "moving", True, (1 if r.mv[2] >= r.mv[1] else -1)
                self._stop_axis(k, f"{k.upper()} stop ({why})", "warn")
                stopped.append(k.upper())
            else:
                r.phase = "idle"
            r.mv = None
        return stopped

    def _end_act(self, ok, why):
        a = self.act
        if a is None:
            return
        self.act = None
        a["ok"], a["why"] = ok, why
        a["event"].set()
        for r in self.run.values():
            if r.phase == "auto":
                r.phase, r.mv = "idle", None
                r.next_poll = time.monotonic()                 # fresh state/current right away
        if not ok:
            self._event(f"{a['label']}: stopped ({why})", "warn")

    def _act_fail(self, why):
        if self.act is None:
            return
        self._halt_act(why)
        self._end_act(False, why)
        self._event(why, "error")

    def _plan_move(self, k, t0, p0, p1):
        r = self.run[k]
        v = r.speed or self.speed
        acc = counts_to_deg(self.accel_counts)
        t_end = t0 + move_duration(p1 - p0, v, acc)
        r.mv = [t0, p0, p1, v, acc, t_end] if r.phase == "auto" else None
        r.moves.append((t0, p0, p1, v, acc, t_end))
        return t_end

    def _cut_move(self, k):
        """A step was STOPped: its planned profile is no longer valid after now."""
        r = self.run[k]
        if r.moves:
            m = r.moves[-1]
            now = time.monotonic()
            if m[5] > now:
                r.moves[-1] = (m[0], m[1], trapezoid(now, *m[:5]), m[3], m[4], now)
        r.mv = None

    def _act_tick(self, now):
        act = self.act
        t = clock.now()
        v_def = self.speed
        acc = counts_to_deg(self.accel_counts)
        done = True
        for k in ("x", "y"):
            a, r = self.axes[k], self.run[k]
            if r.phase != "auto":
                done = False
                continue
            if r.mv is not None:
                if now < r.mv[5] + config.TRACK_SETTLE_S:
                    done = False
                    continue
                self._read_pos(k)                              # step should be over: where are we?
                r.mv = None
            if act["kind"] == "goto":
                want = act["target"][0 if k == "x" else 1]
            else:
                lead, want = 0.1, None
                for _ in range(3):                             # aim where the target will be at the step's end
                    p = act["fn"](t + timedelta(seconds=lead))
                    if p is None:
                        break
                    want = p[0 if k == "x" else 1]
                    lead = move_duration(want - a.angle, r.speed or v_def, acc) + 0.05
                if want is None:
                    continue
            lo, hi = a.limits
            if not lo <= want <= hi:
                if not act["clipped"]:
                    self._event(f"{act['label']}: {k.upper()} target {want:+.2f}\N{DEGREE SIGN} beyond the "
                                f"\N{PLUS-MINUS SIGN}{hi:.0f}\N{DEGREE SIGN} soft limit - clipped", "warn")
                act["clipped"] = True
                act["msg"] = f"{k.upper()} target beyond \N{PLUS-MINUS SIGN}{hi:.0f}\N{DEGREE SIGN}: clipped at the limit"
                want = max(lo, min(hi, want))
            err = want - a.angle
            tol = config.GOTO_DONE_DEG if act["kind"] == "goto" else config.TRACK_DEADBAND_DEG
            if abs(err) < tol or now < r.next_step:
                if abs(err) >= tol:
                    done = False
                continue
            done = False
            counts = deg_to_counts(a.cfg.sign * err)
            if counts == 0:
                continue
            try:
                a.drive.write(0x0087, counts)                  # REG_REL_TARGET then START: motor is holding
                a.drive.write(0x0083)
                self._mark_start(k, "track" if act["kind"] == "track" else "start")
            except DriveError as e:
                if e.code == 0x15:                             # still moving (slower than planned): retry
                    r.busy_since = r.busy_since or now
                    r.next_step = now + 0.1
                    if now - r.busy_since > 5.0:
                        self._act_fail(f"{act['label']}: {k.upper()} still busy (0x15) for 5 s")
                        return
                    continue
                if e.code == 0x27:
                    self._dropout(k)
                    return
                self._act_fail(f"{act['label']}: {k.upper()} step refused: {e}")
                return
            r.busy_since = None
            r.on_expected = True
            self._plan_move(k, now, a.angle, want)
            r.next_step = now + config.TRACK_STEP_PERIOD_S
        if act["kind"] == "goto" and done and self.act is act:
            self._end_act(True, "arrived")
            self._event(f"{act['label']}: arrived at X {self.axes['x'].angle:+.3f}\N{DEGREE SIGN}, "
                        f"Y {self.axes['y'].angle:+.3f}\N{DEGREE SIGN}")

    # ---- exit -------------------------------------------------------------------------------------
    def _do_shutdown(self):
        """STOP both; motors stay ON and holding (top-heavy load, brake unverified)."""
        stopped = self._do_stop_all("app exit")
        end = time.monotonic() + 4.0
        while any(r.phase == "stopping" for r in self.run.values()) and time.monotonic() < end:
            time.sleep(0.1)
            for k, r in self.run.items():
                if r.phase == "stopping":
                    try:
                        self._stopping_step(k, time.monotonic())
                    except COMM_ERRORS as e:
                        self._event(f"{k.upper()}: no reply while stopping at exit: {e}", "error")
                        r.phase = "idle"
        return "stopped " + ("+".join(stopped) if stopped else "nothing (not moving)")
