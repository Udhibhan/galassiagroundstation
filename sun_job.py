"""
sun_job.py - Sun finding, peak-up, tracking and the G/T (Y-factor) measurement.

Runs in its own worker thread and never touches CAN. It only hands goto/track targets to the
gimbal thread (gimbal.goto / gimbal.track) and reads the SDR power meter (receive only).

Actions, one at a time:
  find (gnss)   average the GNSS heading (NAV-RELPOSNED, carrSoln = 2 only) for 60 s,
                mount yaw psi = heading - 90 deg (rover on the +Y side; on -Y use heading - 270),
                go to the predicted Sun (cal = {yaw: psi}), common peak-up, then track.
  find (blind)  elevation-ring scan: sweep azimuth at ~1 deg/s along el = el_sun(t) (yaw = 0),
                predicted az +/- 40 deg, using the gimbal's track mode, recording power.
                No peak >= ring_min_rise_db over the start of the cycle: widen to el +/- 1.2 deg,
                then +/- 80 deg. psi = predicted_az(t_peak) - az_at_peak (mount frame).
                Then peak-up, then track.
  peak-up       cold sky (10 deg off, same elevation, 10 s), cross-scan +/-3 deg in X then Y
                following the Sun (track + offset) in 0.2 deg steps, Gaussian fit in linear power
                above the cold floor, one refinement pass at +/-1.5 deg. Gives offsets {dX, dY},
                FWHM (sky degrees) and a refined yaw.
  track         follow the prediction + offsets (gimbal track mode, 0.5 s steps).
  gt            P_sun 10 s on peak, P_cold 10 s at 10 deg off, Y, then G/T (needs solar flux in sfu).
  off           move 10 deg off the Sun.

LNB heat: the dish focuses sunlight onto the LNB. Time spent within 1.5 deg of the Sun is limited to
120 s (config.SUN_JOB.heat_limit_s), then the dish moves 10 deg off by itself and says why.
Abort/STOP during any Sun step stops, then moves 10 deg off if near the Sun.

Readings, scans, fits, Y, G/T inputs/outputs, Sun az/el and gain go to logs/sun_<time>.json + .csv.
"""
import csv
import json
import logging
import math
import os
import threading
import time
from dataclasses import asdict, replace
from datetime import datetime, timedelta, timezone

import numpy as np

import clock
import config
import kinematics as K
from sun import sun_azel

log = logging.getLogger(__name__)
C_LIGHT = 299_792_458.0
K_BOLTZ = 1.380649e-23
ADVICE = "Best 08:30–10:30 or 15:00–17:00 local; Sun 20–75° up."


class Abort(Exception):
    pass


class HeatLimit(Exception):
    pass


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


# maths
def fit_gauss(x, y):
    """Least-squares fit of y = b + A exp(-4 ln2 (x - x0)^2 / w^2) in linear power.
    Levenberg-Marquardt with a numeric Jacobian. Returns dict(b, A, x0, w, rms, ok)."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    i = int(np.argmax(y))
    b0 = float(np.percentile(y, 20))
    A0 = float(y[i] - b0)
    above = x[y > b0 + A0 / 2]
    w0 = float(max(0.4, (above.max() - above.min()) if len(above) > 1 else 1.6))
    p = np.array([b0, max(A0, 1e-30), float(x[i]), w0])
    scale = max(abs(A0), 1e-30)

    def model(q):
        return q[0] + q[1] * np.exp(-4 * math.log(2) * (x - q[2]) ** 2 / max(q[3] ** 2, 1e-6))

    def res(q):
        return (model(q) - y) / scale

    lam = 1e-2
    r = res(p)
    for _ in range(100):
        J = np.empty((len(x), 4))
        for k in range(4):
            d = np.zeros(4)
            d[k] = 1e-6 * max(abs(p[k]), 1e-3 if k >= 2 else scale)
            J[:, k] = (res(p + d) - r) / d[k]
        A = J.T @ J
        g = J.T @ r
        try:
            step = -np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), g)
        except np.linalg.LinAlgError:
            break
        rn = res(p + step)
        if rn @ rn < r @ r:
            p, r, lam = p + step, rn, lam * 0.3
            if np.max(np.abs(step[2:])) < 1e-6:
                break
        else:
            lam *= 10
            if lam > 1e8:
                break
    b, A_, x0, w = (float(v) for v in p)
    w = abs(w)
    ok = A_ > 0 and 0.3 < w < 8 and x.min() - 0.5 <= x0 <= x.max() + 0.5
    return {"b": b, "A": A_, "x0": x0, "w": w, "rms": float(np.sqrt(np.mean((r * scale) ** 2))), "ok": bool(ok)}


def gt_from_y(y_lin, sfu, fwhm_deg, freq_hz=None):
    """G/T = 8 pi k (Y - 1) L / (lambda^2 S), S = sfu * 1e-22 W/m^2/Hz, L = 1 + 0.38 (0.53 deg / theta)^2."""
    freq_hz = freq_hz or config.SDR.sky_freq_hz
    lam = C_LIGHT / freq_hz
    L = 1 + 0.38 * (0.53 / fwhm_deg) ** 2 if fwhm_deg else None
    out = {"k": K_BOLTZ, "lambda_m": lam, "freq_hz": freq_hz, "L": L, "fwhm_deg": fwhm_deg, "sfu": sfu,
           "S": sfu * 1e-22 if sfu else None, "Y": y_lin, "Y_db": 10 * math.log10(y_lin) if y_lin and y_lin > 0 else None,
           "gt": None, "gt_db": None}
    if sfu and sfu > 0 and L and y_lin and y_lin > 1:
        gt = 8 * math.pi * K_BOLTZ * (y_lin - 1) * L / (lam ** 2 * sfu * 1e-22)
        out.update(gt=gt, gt_db=10 * math.log10(gt))
    return out


# session
class SunSession:
    def __init__(self, gimbal, meter_fn, gnss, site_fn):
        self.g = gimbal
        self.meter_fn = meter_fn             # returns PowerMeter or None
        self.gnss = gnss
        self.site_fn = site_fn               # returns (lat, lon, source)
        self.cfg = config.SUN_JOB
        self.thread = None
        self.cancel = threading.Event()
        self.cancel_why = None
        self.cal = None                      # {"yaw": psi} once known
        self.offs = (0.0, 0.0)               # peak-up pointing offsets (dX, dY), axis degrees
        self.result = None                   # last find/peak-up result
        self.gt = None                       # last G/T measurement
        self.flux_sfu = None
        self.floor_lin = None                # cold sky (or ring median) for the power-based heat test
        self.near_since = None
        self.rows = []                       # (reading, pose, step) collected while an action runs
        self.collect = False
        self.session = None                  # log file stem
        self.data = {}                       # everything for the JSON log
        self._act = None
        self._gnss_s = config.GNSS_HEADING_AVERAGE_S
        self._plot = {"title": "", "xlabel": "", "pts": [], "fit": []}
        self.prog = {"state": "idle", "action": None, "step": "", "pct": 0, "msg": "", "warnings": []}

    # public
    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def sun_now(self, t=None):
        lat, lon, _ = self.site_fn()
        return sun_azel(t or clock.now(), lat, lon)

    def checks(self, method=None) -> list:
        """Reasons a Sun step cannot start now (empty list = fine)."""
        why = []
        az, el = self.sun_now()
        if el < self.cfg.min_el:
            why.append(f"Sun elevation {el:.1f}° is below {self.cfg.min_el:.0f}°")
        if el > self.cfg.max_el:
            why.append(f"Sun elevation {el:.1f}° is above {self.cfg.max_el:.0f}° (near zenith yaw can't be measured)")
        if not self.g.holding:
            why.append("motors not ON and holding (press Start, wait for both to hold)")
        m = self.meter_fn()
        if m is None or not m.status()["available"]:
            why.append("SDR missing" + (f" ({m.error})" if m is not None and m.error else ""))
        if method == "gnss":
            r = self.gnss_reason()
            if r:
                why.append(r)
        return why

    def next_window(self):
        """First time in the next 24 h the Sun is inside the allowed elevation window, or None."""
        t = clock.now()
        for m in range(5, 24 * 60, 5):
            _, el = self.sun_now(t + timedelta(minutes=m))
            if self.cfg.min_el <= el <= self.cfg.max_el:
                return t + timedelta(minutes=m)
        return None

    def checks_auto(self) -> list:
        """Reasons the one-button Auto cannot start. It starts the motors itself and uses whatever is connected."""
        why = []
        az, el = self.sun_now()
        if el < self.cfg.min_el:
            nw = self.next_window()
            why.append(f"the Sun is {el:.0f}° up (needs at least {self.cfg.min_el:.0f}°)" +
                       (f"; usable from {nw.astimezone().strftime('%H:%M')}" if nw else ""))
        if el > self.cfg.max_el:
            why.append(f"the Sun is {el:.0f}° up (above {self.cfg.max_el:.0f}° north can't be measured)")
        st = self.gnss.status() if self.gnss else {"connected": False}
        m = self.meter_fn()
        if not st.get("connected") and not (m is not None and m.status()["available"]):
            why.append("neither the GNSS (to find north) nor the SDR (blind search) is connected")
        if self.g.mode in ("offline", "fault"):
            why.append(f"drives: {self.g.mode} (check 48 V, the CAN cable and the Pi relay)")
        return why

    def gnss_reason(self):
        st = self.gnss.status() if self.gnss else {"connected": False}
        if not st.get("connected"):
            return "no GNSS (heading kit not connected)"
        if st.get("carr") != 2:
            return "GNSS not RTK FIXED (carrSoln must be 2)"
        return None

    def start(self, action, method=None, opts=None):
        if self.busy():
            raise RuntimeError(f"a Sun step is already running ({self.prog['action']}): Abort it first")
        end = time.monotonic() + 3.0                           # a move just ended, let the state poll catch up
        while not self.g.holding and time.monotonic() < end and self.g.mode == "on":
            time.sleep(0.1)
        opts = dict(opts or {})
        self._gnss_s = opts.pop("gnss_avg_s", None) or config.GNSS_HEADING_AVERAGE_S
        self.cfg = replace(config.SUN_JOB, **opts) if opts else config.SUN_JOB
        if action == "auto":
            why = self.checks_auto()
            if why:
                raise RuntimeError("Can't start: " + "; ".join(why))
        elif action == "find":
            if method not in ("gnss", "blind"):
                raise ValueError("method must be gnss or blind")
            why = self.checks(method)
            if why:
                raise RuntimeError("Can't start: " + "; ".join(why))
        elif action in ("gt", "track"):
            if not (self.result and self.result.get("peak_ok")):
                raise RuntimeError("Run a successful Sun find + peak-up first")
            why = self.checks()
            if why:
                raise RuntimeError("Can't start: " + "; ".join(why))
        elif action == "off":
            if not self.g.holding and self.g.act is None:
                raise RuntimeError("motors not ON and holding")
        else:
            raise ValueError(f"unknown Sun action {action!r}")
        self.cancel.clear()
        self.cancel_why = None
        self.prog = {"state": "running", "action": action, "method": method, "step": "starting", "pct": 0,
                     "msg": "", "warnings": [], "log": [], "t0": clock.now().isoformat(timespec="seconds")}
        self.thread = threading.Thread(target=self._worker, args=(action, method), name=f"sun-{action}", daemon=True)
        self.thread.start()
        return "started"

    def abort(self, why="Abort"):
        if self.busy():
            self.cancel_why = why
            self.cancel.set()

    def set_flux(self, sfu):
        self.flux_sfu = float(sfu) if sfu not in (None, "") else None
        if self.gt:
            self.gt.update(gt_from_y(self.gt["Y"], self.flux_sfu, self.gt["fwhm_deg"]))
            self.data["gt"] = self.gt
            self._save()
        return self.gt

    def on_reading(self, r, pose):
        """App callback for every SDR reading (meter thread)."""
        if self.collect:
            self.rows.append((r, pose, self.prog.get("step", "")))

    def status(self) -> dict:
        lat, lon, src = self.site_fn()
        t = clock.now()
        az, el = sun_azel(t, lat, lon)
        cal = self.cal or {"yaw": 0.0}
        sun_axes = None
        try:
            sun_axes = K.azel_to_axes(az, el, cal)
        except ValueError:
            pass
        path = []
        for m in range(-180, 181, 15):                          # Sun path +/- 3 h for the sky plot
            a, e = sun_azel(t + timedelta(minutes=m), lat, lon)
            if e > 0:
                path.append([round(a, 2), round(e, 2)])
        res = dict(self.result) if self.result else None
        gt = dict(self.gt) if self.gt else None
        return {"az": az, "el": el, "site": {"lat": lat, "lon": lon, "source": src}, "advice": ADVICE,
                "path": path, "axes": sun_axes,
                "reachable_now": sun_axes is not None and K.within_limits(*sun_axes),
                "window": [self.cfg.min_el, self.cfg.max_el], "offset": config.BORESIGHT_OFFSET,
                "cal": self.cal, "offs": self.offs, "prog": dict(self.prog, plot=self._plot),
                "result": res, "gt": gt, "flux_sfu": self.flux_sfu, "log": self.session,
                "heat_limit_s": self.cfg.heat_limit_s, "gnss_reason": self.gnss_reason(),
                "checks": self.checks(), "checks_auto": self.checks_auto(), "busy": self.busy()}

    def reach_grid(self):
        return K.reach_grid(self.cal or {"yaw": 0.0})

    # worker
    def _note(self, text, level="info"):
        """One line of the running commentary on the page."""
        lg = self.prog.setdefault("log", [])
        lg.append({"t": clock.now().isoformat(timespec="seconds"), "msg": text, "level": level})
        del lg[:-120]

    def _stage(self, name):
        """Coarse stage for the progress diagram: motors, home, north, point, peak, track."""
        self.prog["stage"] = name

    def _say(self, step, pct=None, msg=None):
        self.prog["step"] = step
        if pct is not None:
            self.prog["pct"] = int(pct)
        if msg is not None:
            self.prog["msg"] = msg
        self._note(f"{step}: {msg}" if msg else step)
        log.info("Sun: %s %s", step, msg or "")

    def _warn(self, w):
        if w not in self.prog["warnings"]:
            self.prog["warnings"].append(w)
            self._note(w, "warn")
            log.warning("Sun: %s", w)

    def _worker(self, action, method):
        self.collect = True
        if self.session is None or action == "find":
            self.session = "sun_" + clock.now().astimezone().strftime("%Y%m%d_%H%M%S")
            self.rows, self.data = [], {}
        self.data.setdefault("actions", []).append({"action": action, "method": method,
                                                    "t": clock.now().isoformat(), "cfg": asdict(self.cfg)})
        try:
            if action == "auto":
                self._auto()
            elif action == "find":
                self._find(method)
            elif action == "gt":
                self._gt()
            elif action == "track":
                self._track_sun()
            elif action == "off":
                self._move_off("requested")
            self.prog["state"] = "done"
        except HeatLimit as e:
            self._cleanup(str(e), force_off=True)
            self.prog["state"] = "done" if action in ("track", "find", "auto") and self.result and self.result.get("peak_ok") else "failed"
            self.prog["msg"] = f"{e} - moved 10° off the Sun"
        except Abort as e:
            self._cleanup(str(e))
            self.prog["state"] = "aborted"
            self.prog["msg"] = f"aborted: {e}" + (f" - {self.prog.get('moved_off')}" if self.prog.get("moved_off") else "")
        except Exception as e:
            log.exception("Sun step failed")
            self._cleanup(str(e))
            self.prog["state"] = "failed"
            self.prog["msg"] = str(e) + (f" - {self.prog.get('moved_off')}" if self.prog.get("moved_off") else "")
        finally:
            self.collect = False
            self.prog["t1"] = clock.now().isoformat(timespec="seconds")
            self.data["prog"] = {k: v for k, v in self.prog.items() if k != "plot"}
            try:
                self._save()
            except Exception:
                log.exception("Sun log save failed")

    # waiting with checks
    def _meter(self):
        m = self.meter_fn()
        if m is None or not m.available:
            raise RuntimeError("SDR missing")
        return m

    def _check(self):
        if self.cancel.is_set():
            raise Abort(self.cancel_why or "Abort")
        a = self._act
        if a is not None and a["event"].is_set() and a["ok"] is False:
            raise Abort(f"gimbal stopped ({a['why']})")
        self._heat()

    def _sleep(self, dt):
        end = time.monotonic() + dt
        while True:
            self._check()
            left = end - time.monotonic()
            if left <= 0:
                return
            time.sleep(min(0.1, left))

    def _track(self, fn, label):
        def safe(t):
            try:
                return fn(t)
            except ValueError:
                return None
        self._act = self.g.track(safe, label).result(5)

    def _goto(self, x, y, label, safety=False, timeout=240):
        act = self.g.goto(x, y, label, safety).result(5)
        self._act = act
        end = time.monotonic() + timeout
        while not act["event"].wait(0.1):
            if not safety:                                  # the move-off itself is only stopped by STOP
                self._check()
            if time.monotonic() > end:
                raise RuntimeError(f"{label}: did not arrive in {timeout:.0f} s")
        if not act["ok"]:
            raise Abort(f"gimbal stopped ({act['why']})")

    def _wait_settled(self, timeout=20.0, tol=0.05):
        end = time.monotonic() + timeout
        while not self.g.settled(tol):
            self._check()
            if time.monotonic() > end:
                raise RuntimeError("the dish did not settle on the target")
            time.sleep(0.05)

    # geometry
    def _sun_axes(self, t, dX=0.0, dY=0.0, cal=None, daz=0.0, del_=0.0):
        az, el = self.sun_now(t)
        x, y = K.azel_to_axes(az + daz, el + del_, cal or self.cal)
        return x + dX, y + dY

    def _sep(self, pose):
        """Angle (deg) between the beam (cal + offsets) and the Sun now."""
        az, el = self.sun_now()
        x, y = pose
        v = K.body_to_sky(x - self.offs[0], y - self.offs[1], self.cal)
        return K.angle_between(v, K.azel_to_enu(az, el))

    def _near(self, radius, rise_db):
        pose = self.g.pose_at()
        if self.cal is not None and pose is not None:
            return self._sep(pose) < radius
        m = self.meter_fn()
        r = m.latest() if m else None
        if r and self.floor_lin:
            return 10 * math.log10(r["lin"] / self.floor_lin) > rise_db
        return False

    def _heat(self):
        near = self._near(self.cfg.heat_radius, 3.0)
        now = time.monotonic()
        if near:
            self.near_since = self.near_since or now
        else:
            self.near_since = None
        near_s = now - self.near_since if self.near_since else 0.0
        self.prog["heat"] = {"near": near, "near_s": round(near_s, 1), "limit_s": self.cfg.heat_limit_s}
        if near_s > self.cfg.heat_limit_s:
            self.near_since = None
            raise HeatLimit(f"LNB heat rule: {self.cfg.heat_limit_s:.0f} s within {self.cfg.heat_radius}° of the Sun")

    def _off_side(self):
        """+1/-1: azimuth side to go off to, away from where the Sun is moving."""
        t = clock.now()
        a0, _ = self.sun_now(t)
        a1, _ = self.sun_now(t + timedelta(minutes=2))
        return -1 if wrap180(a1 - a0) > 0 else 1

    def _cold_fn(self, side):
        """Track target 10 deg off the Sun on the sky, same elevation, on `side`."""
        def fn(t):
            az, el = self.sun_now(t)
            daz = side * self.cfg.cold_offset / max(math.cos(math.radians(el)), 0.2)
            x, y = K.azel_to_axes(az + daz, el, self.cal)
            return x + self.offs[0], y + self.offs[1]
        return fn

    def _cold_side(self):
        pref = self._off_side()
        for s in (pref, -pref):
            try:
                x, y = self._cold_fn(s)(clock.now())
            except ValueError:
                continue
            if K.within_limits(x, y, 0.5):
                return s
        raise RuntimeError("no reachable cold-sky point 10° off the Sun within ±80°")

    def _move_off(self, why):
        """Move 10 deg off the Sun from wherever the dish points. safety=True: not stopped by the deadman."""
        end = time.monotonic() + 5
        while any(r.phase == "stopping" for r in self.g.run.values()) and time.monotonic() < end:
            time.sleep(0.1)
        if self.g.act is not None:
            self.g.stop_act("moving off the Sun").result(5)
            end = time.monotonic() + 5
            while any(r.phase == "stopping" for r in self.g.run.values()) and time.monotonic() < end:
                time.sleep(0.1)
        if not (self.g.mode == "on" and self.g.started):
            self.prog["moved_off"] = "could NOT move off the Sun (motors not holding) - it drifts off by itself"
            log.warning("Sun: %s", self.prog["moved_off"])
            return False
        x, y = self.g.angles()
        cal = self.cal or {"yaw": 0.0}
        az_m, el_m = K.axes_to_azel(x - self.offs[0], y - self.offs[1], cal)
        pref = self._off_side()
        cands = [(az_m + s * self.cfg.cold_offset / max(math.cos(math.radians(el_m)), 0.2), el_m) for s in (pref, -pref)]
        cands.append((az_m, el_m - self.cfg.cold_offset))
        for az2, el2 in cands:
            p = K.reachable(az2, el2, cal, margin=0.5)
            if p is None:
                continue
            tx, ty = p[0] + self.offs[0], p[1] + self.offs[1]
            if not K.within_limits(tx, ty, 0.2):
                continue
            self._say("moving 10° off the Sun", msg=f"moving 10° off the Sun: {why}")
            try:
                self._goto(tx, ty, "move off the Sun", safety=True, timeout=120)
            except Abort as e:
                self.prog["moved_off"] = f"move-off stopped ({e})"
                return False
            self.prog["moved_off"] = f"moved 10° off the Sun ({why})"
            self.near_since = None
            return True
        self.prog["moved_off"] = "no reachable point 10° off the Sun - dish left where it is"
        return False

    def _cleanup(self, why, force_off=False):
        """After Abort / failure / heat limit: stop the activity, and if near the Sun move 10 deg off."""
        self._act = None
        try:
            near = force_off or self._near(self.cfg.abort_near, 1.0)
        except Exception:
            near = True
        if self.g.act is not None:
            try:
                self.g.stop_act(why).result(5)
            except Exception:
                pass
        if near:
            try:
                self._move_off(why)
            except Exception as e:
                self.prog["moved_off"] = f"move-off failed: {e}"
                log.exception("move off")

    # readings / plots
    def _rows_since(self, tm0):
        return [(r, p, s) for (r, p, s) in list(self.rows) if r["tm"] >= tm0]

    def _set_plot(self, title, xlabel, pts, fit=None):
        self._plot = {"title": title, "xlabel": xlabel, "pts": [[round(a, 4), round(b, 3)] for a, b in pts][-1500:],
                      "fit": [[round(a, 4), round(b, 3)] for a, b in (fit or [])]}

    def _avg(self, seconds, label):
        """Average linear power over `seconds` while following the current target."""
        m = self._meter()
        tm0 = time.monotonic()
        while time.monotonic() - tm0 < seconds:
            self._sleep(0.2)
            rs = m.since(tm0)
            self._set_plot(label, "time (s)", [(r["tm"] - tm0, r["db"]) for r in rs])
        st = m.stats(tm0)
        if st["n"] == 0:
            raise RuntimeError("no SDR readings")
        if st["sat"]:
            self._warn("SDR saturated (|x| > 0.7 FS): reduce gain - result INVALID")
        return st

    # find
    def _find(self, method):
        self.result = None
        self.cal, self.offs, self.floor_lin, self.near_since = None, (0.0, 0.0), None, None
        m = self._meter()
        az, el = self.sun_now()
        res = {"method": method, "t": clock.now().isoformat(), "sun_az": az, "sun_el": el, "site": self.site_fn(),
               "gain_db": m.gain_db, "offset": config.BORESIGHT_OFFSET, "peak_ok": False, "valid": True}
        self.data["find"] = res
        if method == "gnss":
            secs = self._gnss_s
            self._stage("north")
            self._say("GNSS heading average", 2, f"averaging the RTK FIXED heading for {secs:.0f} s")
            avg = self.gnss.average_baseline(secs, cancel=lambda: self.cancel.is_set())
            self._check()
            if avg is None:
                raise RuntimeError("too few RTK FIXED headings (carrSoln 2) during the average")
            bar = config.gnss_bar_deg()
            psi = wrap180(avg["heading"] - bar)
            res.update(psi=psi, psi_source="GNSS", gnss_heading=avg["heading"], gnss_n=avg["n"], bar_side=config.GNSS_BAR_ROVER_SIDE)
            self.cal = {"yaw": psi}
            self._stage("point")
            self._say("go to the predicted Sun", 12, f"yaw ψ = {psi:+.2f}° from GNSS heading {avg['heading']:.2f}°")
            x, y = self._sun_axes(clock.now())
            self._check_reach(x, y)
            self._track(lambda t: self._sun_axes(t), "Sun (predicted)")
            self._wait_settled(240)
            st = self._avg(2.0, "on the predicted Sun")
            res["predicted_db"] = st["db"]
            # GNSS north can be 5-10 deg off, the Sun's own radio power corrects it (3D view and sky plots follow)
            self._stage("peak")
            psi_g = psi
            res["gnss_psi"] = psi_g
            try:
                psi, rings = self._local_search(psi_g)
            except RuntimeError as e:
                # no clear Sun rise: don't wander off on a blind search, sit on the GNSS prediction
                return self._fallback_predicted(res, psi_g, str(e))
            res.update(psi=psi, psi_source="GNSS + Sun power", search=rings)
            self.cal = {"yaw": psi}
            self._say("North corrected", 40, f"the Sun's power peaked {wrap180(psi - psi_g):+.1f}° from the GNSS north → yaw ψ = {psi:+.2f}°")
        else:
            self._stage("north")
            psi, ring = self._ring_search()
            res.update(psi=psi, psi_source="ring scan", ring=ring)
            self.cal = {"yaw": psi}
            self._say("ring scan done", 40, f"yaw ψ ≈ {psi:+.2f}° from the ring scan")
        try:
            self._peakup(res)
        except RuntimeError as e:
            if not str(e).startswith("no Sun peak") or "gnss_psi" not in res:
                raise
            return self._fallback_predicted(res, res["gnss_psi"], str(e))
        self.result = res
        self.data["find"] = res
        self._save()
        self._track_sun()

    def _fallback_predicted(self, res, psi, why):
        """No Sun rise above threshold: point at and track the Sun predicted from the GNSS yaw, no offsets."""
        self._warn(f"{why}: pointing at the predicted Sun position instead (GNSS north, no peak-up)")
        self.cal, self.offs = {"yaw": psi}, (0.0, 0.0)
        res.update(psi=psi, psi_source="GNSS only (no Sun peak found)", peak_ok=False, fallback=why)
        self.result = res
        self.data["find"] = res
        self._save()
        x, y = self._sun_axes(clock.now())
        self._check_reach(x, y)
        self._stage("point")
        self._say("Pointing at the predicted Sun", 40, f"yaw ψ = {psi:+.2f}° → X {x:+.1f}°, Y {y:+.1f}°")
        self._track(lambda t: self._sun_axes(t), "Sun (predicted)")
        self._wait_settled(240)
        self._track_sun()

    def _auto(self):
        """One button: motors, GNSS north, Sun position, point, peak-up (if SDR), track. Narrated."""
        g = self.g
        lat, lon, src = self.site_fn()
        az, el = self.sun_now()
        self._say("Sun position", 2, f"site {lat:.5f}°, {lon:.5f}° ({src}); the Sun is now at azimuth {az:.1f}°, elevation {el:.1f}° "
                  "(computed on this laptop from date, time and site: no download needed)")
        # 1. motors
        self._stage("motors")
        if not (g.started and g.mode == "on"):
            self._say("Starting the motors", 4, "Start sequence: set up both drives, wake X then Y")
            g.begin().result(5)
            self._sleep(1.0)
        end = time.monotonic() + 150
        while not g.holding:
            self._check()
            if g.mode in ("fault", "offline"):
                raise RuntimeError(f"gimbal {g.mode}: {g.fault or ''}")
            if time.monotonic() > end:
                raise RuntimeError("the motors did not come up in 150 s")
            time.sleep(0.2)
        self._say("Motors holding", 8, "both axes ON and holding")
        # 1b. home (dish level, X 0 / Y 0) as the known starting pose
        self._stage("home")
        x0, y0 = g.angles()
        if x0 is None or y0 is None or max(abs(x0), abs(y0)) > 0.3:
            self._say("Going to HOME", 9, "moving both axes to 0° / 0°" + (f" (now X {x0:+.1f}°, Y {y0:+.1f}°)" if x0 is not None else ""))
            g.home().result(5)
            end = time.monotonic() + 300
            while g.mode == "homing" or not g.holding:
                self._check()
                if g.mode in ("fault", "offline"):
                    raise RuntimeError(f"gimbal {g.mode}: {g.fault or ''}")
                if time.monotonic() > end:
                    raise RuntimeError("homing did not finish in 300 s")
                time.sleep(0.2)
            self._note("at HOME")
        else:
            self._note("already at HOME")
        if not config.BORESIGHT_OFFSET.get("confirmed"):
            self._note(f"note: the {config.BORESIGHT_OFFSET['deg']}° beam offset (toward {config.BORESIGHT_OFFSET['toward']}) is not confirmed in config.py: "
                       "the pointing may be off until it is", "warn")
        # 2. what is connected
        m = self.meter_fn()
        have_sdr = m is not None and m.status()["available"]
        self._say("SDR", msg=(f"{m.name} found: will peak-up on the Sun's radio noise" if have_sdr else
                              "no SDR: will point by prediction only (no peak-up)"))
        self._stage("north")
        st = self.gnss.status() if self.gnss else {"connected": False}
        method = None
        if st.get("connected"):
            t0 = time.monotonic()
            last = 0.0
            self._say("Finding north", 10, "waiting for the GNSS heading kit to reach RTK FIXED")
            timeout = config.TEST.gnss_timeout_s
            while True:
                self._check()
                st = self.gnss.status()
                if st.get("carr") == 2 and st.get("heading_valid"):
                    method = "gnss"
                    break
                if time.monotonic() - t0 > timeout:
                    break
                if time.monotonic() - last > 5:
                    last = time.monotonic()
                    self._note(f"GNSS: {st.get('num_sv') or 0} satellites, "
                               f"{['no fix', 'no fix', '2D', '3D'][min(st.get('fix_type') or 0, 3)]}, "
                               f"{['no RTK', 'RTK FLOAT', 'RTK FIXED'][min(st.get('carr') or 0, 2)]} ({time.monotonic() - t0:.0f} s)")
                time.sleep(0.5)
            if method is None:
                self._warn(f"GNSS never reached RTK FIXED in {timeout:.0f} s")
        else:
            self._note("no GNSS connected", "warn")
        if method is None:
            if not have_sdr:
                raise RuntimeError("cannot find north: no RTK FIXED GNSS heading and no SDR for a blind search")
            self._note("falling back to the blind elevation-ring search with the SDR")
            return self._find("blind")
        if have_sdr:
            return self._find("gnss")
        # GNSS only: north from the heading, then follow the predicted Sun
        secs = self._gnss_s
        self._say("GNSS heading", 12, f"RTK FIXED: averaging the heading for {secs:.0f} s")
        avg = self.gnss.average_baseline(secs, cancel=lambda: self.cancel.is_set())
        self._check()
        if avg is None:
            raise RuntimeError("too few RTK FIXED headings during the average")
        bar = config.gnss_bar_deg()
        psi = wrap180(avg["heading"] - bar)
        self.cal, self.offs, self.floor_lin, self.near_since = {"yaw": psi}, (0.0, 0.0), None, None
        res = {"method": "gnss", "t": clock.now().isoformat(), "sun_az": az, "sun_el": el, "site": self.site_fn(),
               "psi": psi, "psi_source": "GNSS", "gnss_heading": avg["heading"], "gnss_n": avg["n"],
               "bar_side": config.GNSS_BAR_ROVER_SIDE, "peak_ok": False, "valid": True}
        self.result = res
        self.data["find"] = res
        self._say("North found", 20, f"baseline heading {avg['heading']:.2f}° true → mount yaw ψ = {psi:+.2f}°")
        x, y = self._sun_axes(clock.now())
        self._check_reach(x, y)
        self._stage("point")
        self._say("Pointing at the Sun", 30, f"moving to X {x:+.1f}°, Y {y:+.1f}°")
        self._track(lambda t: self._sun_axes(t), "Sun (predicted)")
        self._wait_settled(240)
        self._note("on the predicted Sun position (no SDR, so no peak-up: pointing accuracy = how well the mount is levelled)")
        self._track_sun()

    def _check_reach(self, x, y, margin=0.0):
        if not K.within_limits(x, y, margin):
            raise RuntimeError(f"the Sun is not reachable: needs X {x:+.1f}°, Y {y:+.1f}° "
                               f"(soft limits ±80°, boresight offset {config.BORESIGHT_OFFSET['deg']}° "
                               f"toward {config.BORESIGHT_OFFSET['toward']})")

    # method 2: elevation ring
    def _ring_search(self):
        cal0 = {"yaw": 0.0}
        rings = []
        plan = [(self.cfg.ring_half_width, d) for d in self.cfg.ring_el_steps]
        plan += [(self.cfg.ring_wide_half_width, d) for d in self.cfg.ring_el_steps]
        for i, (half, del_) in enumerate(plan):
            r = self._ring(cal0, half, del_, i, len(plan))
            rings.append({k: v for k, v in r.items() if k != "pts"} | {"pts": r.get("pts", [])[::2]})
            if r.get("found"):
                return r["psi"], rings
        raise RuntimeError(f"no Sun peak ≥ +{self.cfg.ring_min_rise_db:.2f} dB in any ring scan "
                           f"(±{self.cfg.ring_wide_half_width:.0f}°, el ±1.2°) - check the boresight offset direction")

    def _local_search(self, psi_g):
        """Ring scan around the predicted Sun using the GNSS yaw. Finds the power peak even if north is 5-10 deg off.
        Returns (corrected yaw, rings). In the scan frame (yaw = psi_g) the result is the residual, so total = psi_g + psi."""
        cal0 = {"yaw": psi_g}
        half = self.cfg.gnss_search_half
        rings = []
        plan = [(half, d) for d in self.cfg.ring_el_steps]
        self._say("Searching for the Sun's power", 14, f"GNSS north may be off by 5–10°: scanning ±{half:.0f}° around the prediction")
        for i, (h, del_) in enumerate(plan):
            r = self._ring(cal0, h, del_, i, len(plan))
            rings.append({k: v for k, v in r.items() if k != "pts"} | {"pts": r.get("pts", [])[::2]})
            if r.get("found"):
                return wrap180(psi_g + r["psi"]), rings
        raise RuntimeError(f"no Sun peak ≥ +{self.cfg.ring_min_rise_db:.2f} dB within ±{half:.0f}° of the GNSS prediction")

    def _ring(self, cal0, half, del_, idx, n):
        label = f"ring scan {idx + 1}/{n}: az ±{half:.0f}°, el {del_:+.1f}°"
        pct0 = 2 + 36 * idx / n
        az_s, el_s = self.sun_now()
        azs = np.arange(az_s - half, az_s + half + 1e-6, 0.5)
        ok = [K.reachable(a, el_s + del_, cal0, margin=0.5) is not None for a in azs]
        segs, cur = [], []
        for a, o in zip(azs, ok):
            if o:
                cur.append(a)
            elif cur:
                segs.append(cur)
                cur = []
        if cur:
            segs.append(cur)
        segs = [s for s in segs if s[-1] - s[0] >= 5.0]
        if not segs:
            self._warn(f"{label}: not reachable within ±80° - skipped")
            return {"half": half, "del": del_, "skipped": "not reachable"}
        seg = next((s for s in segs if s[0] <= az_s <= s[-1]), max(segs, key=lambda s: s[-1] - s[0]))
        a0, a1 = seg[0], seg[-1]
        if a0 > azs[0] + 0.01 or a1 < azs[-1] - 0.01:
            self._warn(f"{label}: clipped to az {a0:.0f}–{a1:.0f}° (±80° limits)")
        # start at the end nearer the current beam
        x, y = self.g.angles()
        az_now, _ = K.axes_to_azel(x, y, cal0)
        if abs(wrap180(az_now - a1)) < abs(wrap180(az_now - a0)):
            a0, a1 = a1, a0
        direction = 1 if a1 > a0 else -1
        rate = self.cfg.ring_rate
        self._say(label, pct0, f"to the ring start az {a0:.1f}° (yaw 0)")
        sx, sy = K.azel_to_axes(a0, self.sun_now()[1] + del_, cal0)
        self._goto(sx, sy, "ring start")
        # time table along the ring: az rate <= ring_rate, each axis <= ring_max_axis_rate
        el_r = self.sun_now()[1] + del_
        tab_az = list(np.arange(a0, a1 + direction * 1e-6, direction * 0.25))
        if abs(tab_az[-1] - a1) > 1e-6:
            tab_az.append(a1)
        tab_t, prev = [0.0], K.azel_to_axes(tab_az[0], el_r, cal0)
        for az_i, az_j in zip(tab_az, tab_az[1:]):
            cur = K.azel_to_axes(az_j, el_r, cal0)
            dt = max(abs(az_j - az_i) / rate, max(abs(cur[0] - prev[0]), abs(cur[1] - prev[1])) / self.cfg.ring_max_axis_rate)
            tab_t.append(tab_t[-1] + dt)
            prev = cur
        dur = tab_t[-1]
        t_s = clock.now()

        def fn(t):
            s = min(max((t - t_s).total_seconds(), 0.0), dur)
            az = float(np.interp(s, tab_t, tab_az))
            _, e = self.sun_now(t)
            return K.azel_to_axes(az, e + del_, cal0)

        tm0 = time.monotonic()
        self._track(fn, "ring scan")
        self._say(label, pct0, f"sweeping az {a0:.0f}→{a1:.0f}° in {dur:.0f} s (≤{rate:.1f}°/s az, "
                               f"≤{self.cfg.ring_max_axis_rate:.1f}°/s per axis)")
        end_xy = fn(t_s + timedelta(seconds=dur))
        while True:
            self._sleep(0.3)
            rows = self._rows_since(tm0)
            pts = [(K.axes_to_azel(p[0], p[1], cal0)[0], r["db"]) for r, p, _ in rows if p]
            self._set_plot(label, "mount azimuth (°, yaw 0)", pts)
            if len(rows) > 20:
                self.floor_lin = float(np.median([r["lin"] for r, _, _ in rows]))
            el_t = (clock.now() - t_s).total_seconds()
            self.prog["pct"] = int(pct0 + 36 / n * min(1.0, el_t / max(dur, 1)))
            if el_t > dur and self.g.settled(0.1):
                break
            if el_t > dur + 60:
                break
            if self.g.act is not None and self.g.act["clipped"]:
                self._warn(f"{label}: {self.g.act['msg']}")
        self.g.stop_act("ring done").result(5)
        self._act = None
        rows = [(r, p) for r, p, _ in self._rows_since(tm0) if p]
        out = {"half": half, "del": del_, "az0": a0, "az1": a1, "n": len(rows)}
        if len(rows) < 20:
            out["found"] = False
            return out
        lin = np.array([r["lin"] for r, _ in rows])
        azm = np.array([K.axes_to_azel(p[0], p[1], cal0)[0] for _, p in rows])
        med = float(np.median(lin))
        k = max(1, int(self.cfg.ring_ref_samples))
        ref = float(np.mean(lin[:k]))         # reference = power at the start of this ring cycle (not the median)
        i = int(np.argmax(lin))
        rise = 10 * math.log10(lin[i] / ref)
        out.update(median_db=10 * math.log10(med), ref_db=10 * math.log10(ref), peak_db=10 * math.log10(lin[i]), rise_db=rise,
                   pts=[[round(float(a), 3), round(10 * math.log10(v), 3)] for a, v in zip(azm, lin)])
        if any(r["sat"] for r, _ in rows):
            self._warn("SDR saturated during the ring scan: reduce gain")
        if rise < self.cfg.ring_min_rise_db:
            out["found"] = False
            self._say(label, msg=f"no peak (best {rise:+.2f} dB over the start of the cycle)")
            return out
        sel = np.abs(wrap180(azm - azm[i])) < 4.0
        az_rel = wrap180(azm[sel] - azm[i])
        f = fit_gauss(az_rel, lin[sel])
        az_pk = azm[i] + (f["x0"] if f["ok"] else 0.0)
        tm_pk = rows[i][0]["t"]
        sun_az_pk, sun_el_pk = self.sun_now(datetime.fromtimestamp(tm_pk, timezone.utc))
        psi = float(wrap180(sun_az_pk - az_pk))
        out.update(found=True, az_peak=float(az_pk), sun_az_at_peak=sun_az_pk, psi=psi, fit=f)
        fit_curve = [(float(azm[i] + d), 10 * math.log10(max(f["b"] + f["A"] * math.exp(-4 * math.log(2) * (d - f["x0"]) ** 2 / f["w"] ** 2), 1e-30)))
                     for d in np.linspace(-4, 4, 60)] if f["ok"] else []
        self._set_plot(label + " - peak found", "mount azimuth (°, yaw 0)", [(float(a), 10 * math.log10(v)) for a, v in zip(azm, lin)], fit_curve)
        self._say(label, msg=f"peak {rise:+.2f} dB (vs start of cycle) at mount az {az_pk:.2f}°, Sun az {sun_az_pk:.2f}° → ψ {psi:+.2f}°")
        return out

    # common peak-up
    def _peakup(self, res):
        cfg = self.cfg
        self._stage("peak")
        # reachability now and for the next 10 min, with the cut margins
        for mins in (0, 5, 10):
            x, y = self._sun_axes(clock.now() + timedelta(minutes=mins))
            self._check_reach(x, y, margin=cfg.cut_half)
        # 1. cold sky
        side = self._cold_side()
        self._say("cold sky", 42, f"10° off the Sun, same elevation, {cfg.cold_avg_s:.0f} s")
        self._track(self._cold_fn(side), "cold sky")
        self._wait_settled(240)
        st = self._avg(cfg.cold_avg_s, "cold sky")
        p_cold = st["lin"]
        self.floor_lin = p_cold
        res["p_cold"] = {"lin": p_cold, "db": st["db"], "n": st["n"], "sat": st["sat"], "side": side}
        if st["sat"]:
            res["valid"] = False
        cuts = []
        # 2. coarse cross-scan X then Y
        fx = self._cut("x", 0.0, 0.0, cfg.cut_half, 50, 62, p_cold, cuts)
        fy = self._cut("y", fx["x0"], 0.0, cfg.cut_half, 62, 74, p_cold, cuts)
        # 3. one refinement pass at +/-1.5 deg
        fx = self._cut("x", fy["x0"], fx["x0"], cfg.fine_half, 74, 84, p_cold, cuts)
        fy = self._cut("y", fx["x0"], fy["x0"], cfg.fine_half, 84, 94, p_cold, cuts)
        dX, dY = fx["x0"], fy["x0"]
        self.offs = (dX, dY)
        t = clock.now()
        sun_az, sun_el = self.sun_now(t)
        x_pk, y_pk = self._sun_axes(t, dX, dY)
        sx, sy = K.beam_scale(x_pk, y_pk, self.cal)
        fwhm_x, fwhm_y = fx["w"] * sx, fy["w"] * sy
        az_m, el_m = K.axes_to_azel(x_pk, y_pk, {"yaw": 0.0})
        psi_ref = wrap180(sun_az - az_m)
        rise = 10 * math.log10((0.5 * (fx["b"] + fx["A"] + fy["b"] + fy["A"])) / p_cold)
        res.update(dX=dX, dY=dY, fwhm_x=fwhm_x, fwhm_y=fwhm_y, fwhm=0.5 * (fwhm_x + fwhm_y),
                   fwhm_axis={"x": fx["w"], "y": fy["w"], "scale_x": sx, "scale_y": sy},
                   rise_db=rise, psi_ref=psi_ref, el_err=sun_el - el_m, peak_axes=[x_pk, y_pk],
                   sun_az_ref=sun_az, sun_el_ref=sun_el, cuts=cuts, peak_ok=True,
                   cal={"yaw": self.cal["yaw"], "dX": dX, "dY": dY})
        if any(c["sat"] for c in cuts):
            res["valid"] = False
            self._warn("SDR saturated during the cross-scans: reduce gain - peak-up INVALID")
        res["warnings"] = list(self.prog["warnings"])
        self._say("peak-up done", 95, f"dX {dX:+.3f}°, dY {dY:+.3f}°, FWHM {fwhm_x:.2f}/{fwhm_y:.2f}°, "
                                      f"rise +{rise:.1f} dB, refined ψ {psi_ref:+.2f}°")

    def _cut(self, axis, other, centre, half, pct0, pct1, p_cold, cuts):
        cfg = self.cfg
        offs = np.round(np.arange(centre - half, centre + half + 1e-9, cfg.cut_step), 4)
        label = f"cross-scan {axis.upper()} ±{half:g}°"
        self._say(label, pct0)
        pts = []
        sat = False
        for i, o in enumerate(offs):
            dX, dY = (o, other) if axis == "x" else (other, o)
            self._track(lambda t, dX=dX, dY=dY: self._sun_axes(t, dX, dY), label)
            self._wait_settled(30)
            tm0 = time.monotonic()
            self._sleep(cfg.dwell_s)
            st = self._meter().stats(tm0)
            if st["n"] == 0:
                continue
            sat |= st["sat"]
            x, y = self.g.angles()
            px, py = self._sun_axes(clock.now())
            actual = (x - px) if axis == "x" else (y - py)
            pts.append((actual, st["lin"]))
            self.prog["pct"] = int(pct0 + (pct1 - pct0) * (i + 1) / len(offs))
            self._set_plot(label, f"{axis.upper()} offset from the prediction (°)",
                           [(a, 10 * math.log10(v)) for a, v in pts])
        if sat:
            self._warn("SDR saturated (|x| > 0.7 FS): reduce gain - result INVALID")
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        f = fit_gauss(xs, ys)
        rise = 10 * math.log10(max(f["b"] + f["A"], 1e-30) / p_cold)
        curve = [(float(v), 10 * math.log10(max(f["b"] + f["A"] * math.exp(-4 * math.log(2) * (v - f["x0"]) ** 2 / f["w"] ** 2), 1e-30)))
                 for v in np.linspace(min(xs), max(xs), 80)]
        self._set_plot(label + " - fit", f"{axis.upper()} offset from the prediction (°)",
                       [(a, 10 * math.log10(v)) for a, v in pts], curve)
        cuts.append({"axis": axis, "half": half, "other": other, "pts": [[a, v] for a, v in pts], "fit": f,
                     "rise_db": rise, "sat": sat})
        if not f["ok"] or rise < cfg.min_rise_db:
            raise RuntimeError(f"no Sun peak in the {axis.upper()} cross-scan (rise {rise:+.1f} dB over cold sky)")
        self._say(label, msg=f"peak at {axis.upper()} {f['x0']:+.3f}°, width {f['w']:.2f}° (axis), +{rise:.1f} dB")
        return f

    # track
    def _track_sun(self):
        self._stage("track")
        self._say("tracking the Sun", 96, f"prediction + offsets; moves 10° off after {self.cfg.heat_limit_s:.0f} s (LNB heat)")
        self._track(lambda t: self._sun_axes(t, *self.offs), "Sun track")
        tm0 = time.monotonic()
        last_note = 0.0
        while True:
            self._sleep(0.3)
            if time.monotonic() - last_note > 15:
                last_note = time.monotonic()
                saz, sel_ = self.sun_now()
                bx, by = self.g.angles()
                self._note(f"tracking: Sun az {saz:.2f}° el {sel_:.2f}° → dish X {bx:+.2f}°, Y {by:+.2f}°" if bx is not None
                           else f"tracking: Sun az {saz:.2f}° el {sel_:.2f}°")
            a = self.g.act
            if a is not None and a["clipped"]:
                raise RuntimeError(f"Sun leaving the reachable sky: {a['msg']}")
            m = self.meter_fn()
            rs = m.since(tm0) if m else []
            self._set_plot("tracking the Sun", "time (s)", [(r["tm"] - tm0, r["db"]) for r in rs])

    # G/T
    def _gt(self):
        cfg = self.cfg
        fwhm = self.result["fwhm"]
        out = {"t": clock.now().isoformat(), "valid": bool(self.result.get("valid", True))}
        if self.near_since and time.monotonic() - self.near_since > cfg.heat_limit_s - (cfg.gt_avg_s + 10):
            self._say("LNB cool-down", 2, f"heat budget nearly used: {cfg.heat_cooldown_s:.0f} s off the Sun first")
            self._move_off("cool-down before G/T")
            self._sleep(cfg.heat_cooldown_s)
        self._say("G/T: on the Sun peak", 10, f"P_sun: {cfg.gt_avg_s:.0f} s average while tracking on peak")
        self._track(lambda t: self._sun_axes(t, *self.offs), "G/T: Sun")
        self._wait_settled(240)
        s_sun = self._avg(cfg.gt_avg_s, "G/T: P_sun")
        az, el = self.sun_now()
        side = self._cold_side()
        self._say("G/T: cold sky", 55, f"P_cold: 10° off, same elevation, {cfg.gt_avg_s:.0f} s")
        self._track(self._cold_fn(side), "G/T: cold sky")
        self._wait_settled(240)
        s_cold = self._avg(cfg.gt_avg_s, "G/T: P_cold")
        self.g.stop_act("G/T done").result(5)
        self._act = None
        y = s_sun["lin"] / s_cold["lin"]
        out.update(p_sun=s_sun, p_cold=s_cold, sun_az=az, sun_el=el, gain_db=self._meter().gain_db,
                   offs=list(self.offs), psi=self.cal["yaw"])
        if s_sun["sat"] or s_cold["sat"]:
            out["valid"] = False
            self._warn("SDR saturated during G/T: reduce gain - G/T INVALID")
        out.update(gt_from_y(y, self.flux_sfu, fwhm))
        out["warnings"] = list(self.prog["warnings"])
        self.gt = out
        self.data["gt"] = out
        msg = f"Y = {out['Y_db']:.2f} dB"
        msg += f", G/T = {out['gt_db']:.2f} dB/K" if out["gt_db"] is not None else " (enter the solar flux for G/T)"
        self._say("G/T done", 100, msg + " - dish left 10° off the Sun")

    # logs
    def _save(self):
        if not self.session:
            return
        os.makedirs(config.LOG_DIR, exist_ok=True)
        base = os.path.join(config.LOG_DIR, self.session)
        d = dict(self.data, flux_sfu=self.flux_sfu, site=self.site_fn(), offset=config.BORESIGHT_OFFSET,
                 cal=self.cal, offs=list(self.offs), sdr={"args": config.SDR.args, "if_hz": config.SDR.if_freq_hz,
                                                           "sky_hz": config.SDR.sky_freq_hz, "rate": config.SDR.rate_hz,
                                                           "gain_db": config.SDR.gain_db})
        with open(base + ".json", "w") as f:
            json.dump(d, f, indent=1, default=str)
        lat, lon, _ = self.site_fn()
        with open(base + ".csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_utc", "step", "x_deg", "y_deg", "beam_az", "beam_el", "sun_az", "sun_el",
                        "p_lin", "p_dbfs", "peak_fs", "saturated"])
            cal = self.cal or {"yaw": 0.0}
            for r, p, step in list(self.rows):
                t = datetime.fromtimestamp(r["t"], timezone.utc)
                saz, sel = sun_azel(t, lat, lon)
                if p:
                    baz, bel = K.axes_to_azel(p[0] - self.offs[0], p[1] - self.offs[1], cal)
                    w.writerow([t.isoformat(timespec="milliseconds"), step, f"{p[0]:.4f}", f"{p[1]:.4f}",
                                f"{baz:.3f}", f"{bel:.3f}", f"{saz:.3f}", f"{sel:.3f}", f"{r['lin']:.6e}",
                                f"{r['db']:.3f}", f"{r['peak']:.4f}", int(r["sat"])])
                else:
                    w.writerow([t.isoformat(timespec="milliseconds"), step, "", "", "", "", f"{saz:.3f}",
                                f"{sel:.3f}", f"{r['lin']:.6e}", f"{r['db']:.3f}", f"{r['peak']:.4f}", int(r["sat"])])
