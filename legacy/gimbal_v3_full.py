# Old version from before the test stage, taken out of gimbal.py. Not imported anywhere.
# Full v3 gimbal controller (enable/disable, hold-to-jog, goto, track). Used by calibration and auto-tracking.
# To go back to it, copy this over the live code.

"""
gimbal.py - the ONE thread that owns the CAN bus and both axes.

Everything else (UI, calibration, auto-tracker) asks this object for motion; nothing else may
send CAN frames. Modes:
  offline   no reply from the drives (retries every 2 s)
  disabled  motors off, brake holding, positions still read
  hold      enabled, not moving
  jog       Test mode: one slow move toward the soft limit per held key; 0x84 STOP on release
  goto      point-to-point at slew speed (0x87 + 0x83, the proven move)
  track     every 0.5 s: relative step to where the target will be; position read once per axis
  fault     a drive switched off / stopped answering -> everything disabled, needs Reset

CAN budget (keeps the drives calm, see erob-can-bringup lessons):
  per axis: 1 position read per tick (4/s) + <= 2 writes per 0.5 s while tracking
            + state/error read every 2 s. ~15 frames/s total. v2 did ~180/s.
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
from erob_can_axis import Axis, LimitError
from zeroerr_protocol import Drive, DriveError, ERRORS, STATES

log = logging.getLogger(__name__)


class Gimbal:
    def __init__(self, link, home: dict | None = None):
        home = home or {}
        cb = config.CAN_BUS
        self.link = link
        self.axes = {k: Axis(c, Drive(link, c.node_id, cb.gap_s, cb.reply_timeout_s), home.get(k))
                     for k, c in config.AXES.items()}
        self.mode = "offline"
        self.fault = None
        self.events = deque(maxlen=60)
        self.jog_speed = config.JOG_SPEEDS_DEG_S[1]
        self.bus_v = None
        self.target = {"x": None, "y": None}
        self.track_label, self.track_err = "", None
        self._jog_dir = {"x": 0, "y": 0}
        self._jog_beat = 0.0
        self._jog = {k: {"dir": 0, "stopping": None} for k in self.axes}
        self._goto = {}
        self._goto_t0 = 0.0
        self._track_fn = None
        self._track_next = 0.0
        self.adaptive_speed = config.TRACK_ADAPTIVE_SPEED
        self._state = {k: None for k in self.axes}
        self._fails = {k: 0 for k in self.axes}
        self._q = deque()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._quiet_until = 0.0
        self._tick = 0
        self.thread = threading.Thread(target=self._run, name="gimbal", daemon=True)

    # ======================================================================= public API
    def start(self):
        self.thread.start()

    def shutdown(self):
        f = self._call(self._do_disable)
        try:
            f.result(timeout=2)
        except Exception:
            pass
        self._stop.set()

    def enable(self) -> Future:
        return self._call(self._do_enable)

    def disable(self) -> Future:
        return self._call(self._do_disable)

    def estop(self) -> Future:
        return self._call(self._do_estop)

    def reset_fault(self) -> Future:
        return self._call(self._do_reset)

    def set_home(self) -> Future:
        """Current position of both axes becomes 0/0 (dish vertical). Result: {'x': counts, 'y': counts}."""
        return self._call(self._do_set_home)

    def jog(self, dx: int, dy: int):
        """Held-key state from the UI; must be refreshed (heartbeat) or it stops by itself."""
        with self._lock:
            self._jog_dir = {"x": int(max(-1, min(1, dx))), "y": int(max(-1, min(1, dy)))}
            self._jog_beat = time.monotonic()
        if (dx or dy) and self.mode == "hold":
            self._call(self._set_mode, "jog")

    def set_jog_speed(self, v: float):
        self.jog_speed = float(v)

    def goto(self, x: float, y: float) -> Future:
        return self._call(self._do_goto, x, y)

    def track(self, fn, label: str = "") -> Future:
        """fn(datetime_utc) -> (x_deg, y_deg) or None. Called by this thread every 0.5 s."""
        return self._call(self._do_track, fn, label)

    def hold(self) -> Future:
        return self._call(self._do_hold)

    def wait_until_still(self, timeout: float, cancel=None) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cancel and cancel():
                return False
            if self.mode in ("hold", "disabled", "fault", "offline"):
                return self.mode == "hold"
            time.sleep(0.1)
        return False

    def angles(self):
        return self.axes["x"].angle, self.axes["y"].angle

    @property
    def homed(self) -> bool:
        return all(a.homed for a in self.axes.values())

    def snapshot(self) -> dict:
        ax = {}
        for k, a in self.axes.items():
            lo, hi = a.limits
            ax[k] = {"angle": a.angle, "enabled": a.enabled, "homed": a.homed, "node": a.cfg.node_id,
                     "model": a.cfg.model, "min": lo, "max": hi, "target": self.target[k],
                     "state": STATES.get(self._state[k], self._state[k]), "jog": self._jog[k]["dir"]}
        return {"mode": self.mode, "fault": self.fault, "axes": ax, "jog_speed": self.jog_speed,
                "bus_v": self.bus_v, "link": getattr(self.link, "name", "?"), "track_label": self.track_label,
                "track_err": self.track_err}

    # ======================================================================= internals
    def _event(self, msg, level="info"):
        self.events.append({"t": clock.now().isoformat(timespec="seconds"), "level": level, "msg": msg})
        getattr(log, "warning" if level != "info" else "info")(msg)

    def _call(self, fn, *args) -> Future:
        f = Future()
        self._q.append((f, fn, args))
        return f

    def _set_mode(self, m):
        if self.mode != m:
            self.mode = m

    def _run(self):
        next_t = time.monotonic()
        while not self._stop.is_set():
            next_t += config.TICK_S
            try:
                self._step()
            except Exception as e:                           # never let the owner thread die
                log.exception("gimbal loop")
                self._go_fault(f"controller error: {e}")
            self._tick += 1
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()                      # fixed rate, no drift build-up

    def _step(self):
        while self._q:
            f, fn, args = self._q.popleft()
            try:
                f.set_result(fn(*args))
            except Exception as e:
                self._event(str(e), "error")
                f.set_exception(e)
        if self.mode == "offline":
            if self._tick % 8 == 0:
                self._try_connect()
            return
        if time.monotonic() < self._quiet_until:              # stay silent just after enable
            return
        if not self._read_positions():
            return
        if self._tick % config.STATUS_EVERY_TICKS == 0:
            self._read_status()
        if self.mode == "jog":
            self._jog_step()
        elif self.mode == "goto":
            self._goto_step()
        elif self.mode == "track":
            self._track_step()

    # ---- connection / status ------------------------------------------------------------
    def _try_connect(self):
        try:
            for a in self.axes.values():
                a.read()
            self.bus_v = self.axes["x"].drive.bus_volts()
            for a in self.axes.values():
                a.enabled = a.drive.state() != 0
            if any(a.enabled for a in self.axes.values()):
                self._do_disable()                        # start from a known state: brake on
            self.mode = "disabled"
            self._event(f"drives answering on {self.link.name}: node 1 + node 2, bus {self.bus_v:.1f} V")
        except (TimeoutError, DriveError, OSError) as e:
            if self._tick % 40 == 0:
                self._event(f"waiting for drives ({e})", "warn")

    def _read_positions(self) -> bool:
        ok = True
        for k, a in self.axes.items():
            try:
                a.read()
                self._fails[k] = 0
            except (TimeoutError, DriveError, OSError) as e:
                self._fails[k] += 1
                ok = False
                if self._fails[k] >= 3:
                    self._go_fault(f"node {a.cfg.node_id} ({k}) stopped answering: {e}")
                    return False
        return ok

    def _read_status(self):
        for k, a in self.axes.items():
            try:
                st = a.drive.state()
                self._state[k] = st
                if a.enabled and st == 0:
                    err = a.drive.error()
                    self._go_fault(f"node {a.cfg.node_id} ({k}) switched its motor OFF, error 0x{err:X} "
                                   f"({ERRORS.get(err, 'see ZeroErr fault list')})")
                    return
            except (TimeoutError, DriveError, OSError):
                pass
        if self._tick % (config.STATUS_EVERY_TICKS * 5) == 0:
            try:
                self.bus_v = self.axes["x"].drive.bus_volts()
            except Exception:
                pass

    def _go_fault(self, why):
        self.fault = why
        self._event(why, "error")
        for a in self.axes.values():
            try:
                a.stop()
            except Exception:
                pass
            try:
                a.disable()
            except Exception:
                a.enabled = False
        self._clear_motion()
        self.mode = "fault"

    def _clear_motion(self):
        self._jog = {k: {"dir": 0, "stopping": None} for k in self.axes}
        self._goto, self._track_fn = {}, None
        self.target = {"x": None, "y": None}
        self.track_label, self.track_err = "", None

    def _halt(self):
        """Stop any move in progress (keeps motors enabled)."""
        for k, a in self.axes.items():
            if a.enabled and (self._jog[k]["dir"] or self._goto or self._track_fn):
                try:
                    a.stop()
                except Exception:
                    pass
        self._clear_motion()

    # ---- commands (run in this thread) ------------------------------------------------------
    def _do_enable(self):
        if self.mode == "fault":
            raise RuntimeError("Reset the fault first")
        if self.mode == "offline":
            raise RuntimeError("Drives not answering")
        for a in self.axes.values():
            a.prepare(a.cfg.track_speed_deg_s)              # motor OFF + proven configuration
        for a in self.axes.values():
            a.enable()
        self._quiet_until = time.monotonic() + 1.0
        self.mode = "hold"
        self._event("motors enabled")

    def _do_disable(self):
        self._halt()
        for a in self.axes.values():
            try:
                a.disable()
            except Exception as e:
                self._event(f"{a.name}: disable failed: {e}", "error")
        if self.mode not in ("offline", "fault"):
            self.mode = "disabled"
        self._event("motors disabled (brake holds)")

    def _do_estop(self):
        self._halt()
        self._do_disable()
        self._event("STOP pressed", "warn")

    def _do_reset(self):
        self.fault = None
        self.mode = "disabled"
        self._event("fault reset - enable motors to continue")

    def _do_set_home(self):
        if self.mode not in ("disabled", "hold"):
            raise RuntimeError("Stop moving before setting HOME")
        home = {k: a.set_home_here() for k, a in self.axes.items()}
        self._event(f"HOME set here (raw x {home['x']}, y {home['y']} counts)")
        return home

    def _do_hold(self):
        if self.mode in ("jog", "goto", "track"):
            self._halt()
            self.mode = "hold"

    def _need_enabled(self):
        if self.mode not in ("hold", "jog", "goto", "track"):
            raise RuntimeError("Enable the motors first")

    def _do_goto(self, x, y):
        self._need_enabled()
        tgt = {"x": x, "y": y}
        for k, a in self.axes.items():
            lo, hi = a.limits
            if not lo <= tgt[k] <= hi:
                raise LimitError(f"{k} target {tgt[k]:+.1f} outside {lo:+.0f}..{hi:+.0f}")
        self._halt()
        for a in self.axes.values():
            a.set_speed(a.cfg.slew_speed_deg_s)
        self._goto = dict(tgt)
        self.target = dict(tgt)
        self._goto_t0 = time.monotonic()
        self.mode = "goto"

    def _do_track(self, fn, label):
        self._need_enabled()
        self._halt()
        for a in self.axes.values():
            a.set_speed(a.cfg.track_speed_deg_s)
        self._track_fn, self.track_label = fn, label
        self._track_next = 0.0
        self.mode = "track"

    # ---- mode steps --------------------------------------------------------------------------
    def _jog_step(self):
        fresh = time.monotonic() - self._jog_beat < config.JOG_DEADMAN_S
        want = self._jog_dir if fresh else {"x": 0, "y": 0}
        for k, a in self.axes.items():
            j = self._jog[k]
            if j["stopping"] is not None:
                st = a.drive.state()
                if st != 2:
                    j["dir"], j["stopping"] = 0, None
                elif time.monotonic() - j["stopping"] > 1.5:     # 0x84 not honoured -> brake
                    self._go_fault(f"{k}: STOP (0x84) did not stop the move - motors disabled")
                    return
                continue
            if want[k] == j["dir"]:
                continue
            if j["dir"] != 0:
                a.stop()
                j["stopping"] = time.monotonic()
                continue
            lo, hi = a.limits
            delta = (hi if want[k] > 0 else lo) - a.angle
            if abs(delta) < 0.05:
                continue                                           # already at the soft limit
            a.set_speed(self.jog_speed)
            if a.move_by(delta):
                j["dir"] = want[k]
                self.target[k] = a.angle + delta
        if not fresh and all(j["dir"] == 0 and j["stopping"] is None for j in self._jog.values()):
            self.target = {"x": None, "y": None}
            self.mode = "hold"

    def _goto_step(self):
        for k in list(self._goto):
            a = self.axes[k]
            if self._goto[k] is None:
                continue
            delta = self._goto[k] - a.angle
            if abs(delta) < 0.01 or a.move_by(delta):
                self._goto[k] = None                              # move sent (or nothing to do)
        if any(v is not None for v in self._goto.values()):
            return
        states = [a.drive.state() for a in self.axes.values()]
        err = max(abs(self.target[k] - a.angle) for k, a in self.axes.items())
        if all(s != 2 for s in states) and err < 0.1:
            self._goto = {}
            self.mode = "hold"
        elif time.monotonic() - self._goto_t0 > 200:
            self._event(f"goto timed out, error {err:.2f} deg", "warn")
            self._goto = {}
            self.mode = "hold"

    def _track_step(self):
        now = time.monotonic()
        t = clock.now()
        cur = self._track_fn(t)
        if cur is not None:
            self.track_err = math.hypot(cur[0] - self.axes["x"].angle, cur[1] - self.axes["y"].angle)
        if now < self._track_next:
            return
        self._track_next = now + config.TRACK_PERIOD_S
        # adaptive: aim at the END of the step (the dish moves the whole period); fixed: aim mid-step
        lead_s = config.TRACK_PERIOD_S * (1.0 if self.adaptive_speed else 0.5)
        lead = self._track_fn(t + timedelta(seconds=lead_s))
        if lead is None:
            self.target = {"x": None, "y": None}
            return
        for k, a in zip(("x", "y"), (self.axes["x"], self.axes["y"])):
            want = a.clamp(lead[0] if k == "x" else lead[1])
            self.target[k] = want
            err = want - a.angle
            if abs(err) > config.TRACK_DEADBAND_DEG:
                if self.adaptive_speed:                           # finish the step in ~60 % of the period (+ramps) so the next START is never refused
                    v = min(a.cfg.track_speed_deg_s, max(0.05, abs(err) / (0.6 * config.TRACK_PERIOD_S)))
                    try:
                        a.set_speed(round(v, 3), live_only=True)
                    except DriveError as e:
                        self.adaptive_speed = False
                        self._event(f"drive refuses live speed changes ({e}); using fixed track speed", "warn")
                a.move_by(err)                                    # False = still moving, try next period
