"""
calibration.py - pointing calibration: HOME, GNSS dual-antenna poses, auto Sun scans.

Solves for yaw (north offset), tilt_n (mount roll) and x0, y0 (encoder zeros), see kinematics.py.
A mount tilt about East looks the same as an X zero offset, so x0 absorbs it.
  1. HOME      jog the dish vertical (spirit level on the dish back) to get the raw encoder zeros.
  2. GNSS      the gimbal visits 5 poses and the 1 m antenna bar's heading/pitch is averaged at each.
               Gives yaw to ~0.5 deg plus tilts, works in any weather. Status "coarse".
  3. Sun scans the dish tracks the Sun while cross-scanning. The Ku-band noise peak (LNB + SDR) gives
               the true boresight to ~0.05 deg. Needs 2+ scans at different times. Status "fine".
Every measurement goes into one weighted least-squares fit, re-run after each new one.

Don't park on the Sun, the dish focuses heat on the LNB. Each scan limits time near the Sun
(SUN_SCAN.max_on_sun_s) and ends pointing 10 deg away.
"""
import json
import logging
import math
import os
import threading
import time

import numpy as np

import clock
import config
import kinematics as K
from sun import sun_azel

log = logging.getLogger(__name__)

PRIOR_SIGMA = {"yaw": 30.0, "tilt_n": 3.0, "x0": 5.0, "y0": 5.0}
# tilt_e is not fitted: a mount tilt about East is the same rotation as an X encoder offset
# (Rx(tilt_e)·Rx(x - x0)), so the two can't be told apart and x0 absorbs it.
SOLVE = ("yaw", "tilt_n", "x0", "y0")
SUN_SIGMA_DEG = 0.05


# solver
def _residuals(p, gnss, suns, bar_body, gnss_sigma):
    cal = dict(K.ZERO_CAL, **dict(zip(SOLVE, p)))
    r = []
    for g in gnss:
        b = K.body_to_sky(g["x"], g["y"], cal, bar_body)
        m = K.azel_to_enu(g["heading"], g["pitch"])
        r.extend((b - m) / math.radians(gnss_sigma))
    for s in suns:
        b = K.body_to_sky(s["x"], s["y"], cal)
        m = K.azel_to_enu(s["az"], s["el"])
        r.extend((b - m) / math.radians(SUN_SIGMA_DEG))
    r.extend(np.asarray(p) / np.array([PRIOR_SIGMA[k] for k in SOLVE]))
    return np.array(r)


def solve(gnss: list, suns: list, start: dict | None = None,
          bar_body=config.GNSS.bar_body, gnss_sigma=config.GNSS.heading_sigma_deg) -> dict:
    """Levenberg-Marquardt with a numeric Jacobian. Returns params, 1-sigma, per-measurement residuals."""
    p = np.array([(start or K.ZERO_CAL).get(k, 0.0) for k in SOLVE], float)
    lam = 1e-3
    f = lambda q: _residuals(q, gnss, suns, bar_body, gnss_sigma)
    r = f(p)
    for _ in range(60):
        J = np.empty((len(r), len(p)))
        for i in range(len(p)):
            d = np.zeros_like(p)
            d[i] = 1e-4
            J[:, i] = (f(p + d) - r) / 1e-4
        A, g = J.T @ J, J.T @ r
        step = -np.linalg.solve(A + lam * np.diag(np.diag(A)), g)
        r_new = f(p + step)
        if r_new @ r_new < r @ r:
            p, r, lam = p + step, r_new, lam * 0.3
            if np.max(np.abs(step)) < 1e-6:
                break
        else:
            lam *= 10
    cov = np.linalg.inv(J.T @ J)
    cal = dict(K.ZERO_CAL, **dict(zip(SOLVE, (float(v) for v in p))))
    res_g = [K.angle_between(K.body_to_sky(g["x"], g["y"], cal, bar_body), K.azel_to_enu(g["heading"], g["pitch"]))
             for g in gnss]
    res_s = [K.angle_between(K.body_to_sky(s["x"], s["y"], cal), K.azel_to_enu(s["az"], s["el"])) for s in suns]
    return {"params": cal, "sigma": {k: float(math.sqrt(cov[i, i])) for i, k in enumerate(SOLVE)},
            "res_gnss": res_g, "res_sun": res_s,
            "rms_sun": float(np.sqrt(np.mean(np.square(res_s)))) if res_s else None,
            "rms_gnss": float(np.sqrt(np.mean(np.square(res_g)))) if res_g else None}


# store
class Calibration:
    def __init__(self, path=config.CALIBRATION_PATH):
        self.path = path
        self.lock = threading.Lock()
        self.data = {"home": None, "params": dict(K.ZERO_CAL), "sigma": None, "gnss_poses": [], "sun_points": [],
                     "site": None, "fit": None, "solved_at": None}
        if os.path.exists(path):
            try:
                with open(path) as fh:
                    self.data.update(json.load(fh))
            except Exception as e:
                log.warning("calibration file unreadable (%s) - starting empty", e)

    def save(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(self.data, fh, indent=2)
        os.replace(tmp, self.path)

    @property
    def params(self) -> dict:
        return self.data["params"]

    @property
    def status(self) -> str:
        d = self.data
        if not d["home"]:
            return "none"
        suns = d["sun_points"]
        azs = [s["az"] for s in suns]
        if len(suns) >= 3 or (len(suns) >= 2 and max(azs) - min(azs) > 30) or (len(suns) >= 1 and len(d["gnss_poses"]) >= 3):
            return "fine"
        if len(d["gnss_poses"]) >= 3 or suns:
            return "coarse"
        return "home"

    def set_home(self, home: dict):
        with self.lock:
            self.data.update(home=home, params=dict(K.ZERO_CAL), sigma=None, gnss_poses=[], sun_points=[],
                             fit=None, solved_at=None)
            self.save()

    def set_site(self, lat, lon, h):
        with self.lock:
            self.data["site"] = {"lat": lat, "lon": lon, "h": h}
            self.save()

    def add(self, kind: str, item: dict):
        with self.lock:
            self.data["gnss_poses" if kind == "gnss" else "sun_points"].append(item)
            self._resolve()
            self.save()

    def _resolve(self):
        d = self.data
        if not d["gnss_poses"] and not d["sun_points"]:
            return
        fit = solve(d["gnss_poses"], d["sun_points"], start=d["params"])
        d["params"], d["sigma"] = fit["params"], fit["sigma"]
        d["fit"] = {"rms_sun": fit["rms_sun"], "rms_gnss": fit["rms_gnss"], "res_sun": fit["res_sun"],
                    "res_gnss": fit["res_gnss"]}
        d["solved_at"] = clock.now().isoformat(timespec="seconds")

    def clear_measurements(self):
        with self.lock:
            self.data.update(params=dict(K.ZERO_CAL), sigma=None, gnss_poses=[], sun_points=[], fit=None)
            self.save()

    def summary(self) -> dict:
        d = self.data
        return {"status": self.status, "home": d["home"] is not None, "params": d["params"], "sigma": d["sigma"],
                "n_gnss": len(d["gnss_poses"]), "n_sun": len(d["sun_points"]), "fit": d["fit"],
                "solved_at": d["solved_at"], "site": d["site"],
                "sun_points": [{"az": s["az"], "el": s["el"], "t": s["t"], "rise_db": s.get("rise_db")}
                               for s in d["sun_points"]]}


# jobs
class Job:
    """A calibration routine running in its own thread. The UI polls .progress."""
    kind = "job"

    def __init__(self, gimbal, cal: Calibration, site):
        self.g, self.cal, self.site = gimbal, cal, site
        self.progress = {"kind": self.kind, "state": "running", "pct": 0, "msg": "starting", "plot": []}
        self._cancel = threading.Event()
        self.thread = threading.Thread(target=self._wrap, name=self.kind, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def cancel(self):
        self._cancel.set()

    @property
    def cancelled(self):
        return self._cancel.is_set()

    def say(self, msg, pct=None):
        self.progress["msg"] = msg
        if pct is not None:
            self.progress["pct"] = int(pct)
        log.info("%s: %s", self.kind, msg)

    def _wrap(self):
        try:
            self.run()
            self.progress["state"] = "cancelled" if self.cancelled else "done"
        except Exception as e:
            log.exception(self.kind)
            self.progress.update(state="failed", msg=str(e))
        finally:
            self.cleanup()
            if self.g.mode in ("track", "goto"):
                try:
                    self.g.hold().result(3)
                except Exception:
                    pass

    def cleanup(self):
        pass

    def _goto(self, x, y):
        self.g.goto(x, y).result(5)
        if not self.g.wait_until_still(120, lambda: self.cancelled):
            if self.cancelled:
                return False
            raise RuntimeError(f"could not reach ({x:+.1f}, {y:+.1f}): {self.g.fault or self.g.mode}")
        return True

    def _check_ready(self):
        if not self.g.homed:
            raise RuntimeError("Set HOME first (step 1)")
        if self.g.mode in ("jog", "goto", "track"):
            self.g.hold().result(3)
        if self.g.mode != "hold":
            raise RuntimeError("Enable the motors first")


class GnssPoseJob(Job):
    kind = "gnss"
    POSES = [(0, 0), (0, 25), (0, -25), (25, 0), (-25, 0), (20, 20)]

    def __init__(self, gimbal, cal, site, gnss):
        super().__init__(gimbal, cal, site)
        self.gnss = gnss

    def run(self):
        self._check_ready()
        st = self.gnss.status()
        if not st.get("heading_valid"):
            raise RuntimeError("GNSS heading not valid (need RTK FIXED: open sky, both antennas, base->rover link)")
        if st.get("lat") is not None:
            self.cal.set_site(st["lat"], st["lon"], st.get("h_msl", 0.0))
        n = len(self.POSES)
        for i, (x, y) in enumerate(self.POSES):
            if self.cancelled:
                return
            self.say(f"pose {i + 1}/{n}: moving to X {x:+.0f}, Y {y:+.0f}", 100 * i / n)
            if not self._goto(x, y):
                return
            time.sleep(config.GNSS.pose_settle_s)
            self.say(f"pose {i + 1}/{n}: averaging antenna heading {config.GNSS.pose_average_s:.0f} s", 100 * (i + 0.5) / n)
            b = self.gnss.average_baseline(config.GNSS.pose_average_s, lambda: self.cancelled)
            if b is None:
                if self.cancelled:
                    return
                self.say(f"pose {i + 1}: heading lost (not RTK FIXED) - skipped")
                continue
            xa, ya = self.g.angles()
            self.cal.add("gnss", {"x": xa, "y": ya, "heading": b["heading"], "pitch": b["pitch"],
                                  "t": clock.now().isoformat(timespec="seconds")})
            self.progress["plot"].append({"x": xa, "y": ya, "heading": b["heading"], "pitch": b["pitch"]})
        self._goto(0, 0)
        fit = self.cal.data["fit"] or {}
        self.say(f"done - yaw {self.cal.params['yaw']:+.2f} deg, GNSS fit RMS {fit.get('rms_gnss') or 0:.2f} deg", 100)
        if (fit.get("rms_gnss") or 0) > 3:
            self.say("GNSS fit RMS > 3 deg: check GNSS.bar_body direction and the axis signs (Test mode W/S, A/D)")


class SunScanJob(Job):
    kind = "sun"

    def __init__(self, gimbal, cal, site, sensor):
        super().__init__(gimbal, cal, site)
        self.sensor = sensor
        self.off = [0.0, 0.0]
        self.near_s = 0.0
        self.away = None

    def sun_axes(self, t):
        az, el = sun_azel(t, self.site().latitude_deg, self.site().longitude_deg)
        return K.azel_to_axes(az, el, self.cal.params)

    def _fn(self, t):
        x, y = self.sun_axes(t)
        return x + self.off[0], y + self.off[1]

    def _measure_at(self, dx, dy):
        self.off[:] = [dx, dy]
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:               # wait until the dish arrives
            time.sleep(0.1)
            if self.g.track_err is not None and self.g.track_err < 0.05 and time.monotonic() - t0 > 0.3:
                break
        time.sleep(config.SUN_SCAN.dwell_s)
        p = self.sensor.power_db()
        if math.hypot(dx, dy) < 1.5:                       # Sun inside about the main beam: LNB heats
            self.near_s += time.monotonic() - t0
            if self.near_s > config.SUN_SCAN.max_on_sun_s:
                raise RuntimeError("heat limit: too long near the Sun this scan - try again in 10 min")
        return p

    @staticmethod
    def fit_peak(offs, pdb, floor_db):
        """Gaussian fit on linear power above the cold-sky floor. Returns (peak, fwhm, rise_db) or None."""
        offs, pdb = np.asarray(offs), np.asarray(pdb)
        lin = 10 ** (pdb / 10) - 10 ** (floor_db / 10)
        top = lin.max()
        rise = 10 * math.log10(1 + top / 10 ** (floor_db / 10)) if top > 0 else 0.0
        sel = lin > 0.25 * top
        if top <= 0 or sel.sum() < 3:
            return None
        a, b, _ = np.polyfit(offs[sel], np.log(lin[sel]), 2)
        if a >= 0:
            return None
        peak = -b / (2 * a)
        if not offs.min() <= peak <= offs.max():
            return None
        return float(peak), float(math.sqrt(-4 * math.log(2) / a)), rise

    def _cut(self, axis, centre, half, step, floor, label, pct):
        offs = np.round(np.arange(-half, half + 1e-9, step), 3)
        pdb = []
        for i, o in enumerate(offs):
            if self.cancelled:
                return None
            dx, dy = (centre[0] + o, centre[1]) if axis == "x" else (centre[0], centre[1] + o)
            p = self._measure_at(dx, dy)
            pdb.append(p)
            self.progress["plot"].append({"cut": label, "axis": axis, "off": float(o), "db": round(p - floor, 3)})
            self.say(f"{label}: {axis.upper()} {o:+.2f} deg -> {p - floor:+.2f} dB", pct[0] + (pct[1] - pct[0]) * i / len(offs))
        r = self.fit_peak(offs, pdb, floor)
        if r is None:
            return None
        return centre[0 if axis == "x" else 1] + r[0], r[1], r[2]

    def run(self):
        S = config.SUN_SCAN
        self._check_ready()
        if not getattr(self.sensor, "available", False):
            raise RuntimeError(f"SDR not available: {getattr(self.sensor, 'error', '')}")
        site = self.site()
        az, el = sun_azel(clock.now(), site.latitude_deg, site.longitude_deg)
        if not S.min_elevation_deg <= el <= S.max_elevation_deg:
            raise RuntimeError(f"Sun at {el:.1f} deg elevation - scans need {S.min_elevation_deg:.0f}-"
                               f"{S.max_elevation_deg:.0f} deg (low morning/afternoon Sun is best)")
        sx, sy = self.sun_axes(clock.now())
        for a, v in (("x", sx), ("y", sy)):
            lo, hi = self.g.axes[a].limits
            if not lo + S.coarse_half_width_deg <= v <= hi - S.coarse_half_width_deg:
                raise RuntimeError(f"Sun is outside the {a.upper()} travel right now ({v:+.1f} deg)")

        # 1. cold sky reference, 10 deg off the Sun (Y side toward zenith)
        away = (0.0, -10.0 if sy > 0 else 10.0)
        self.away = away
        self.say("measuring cold sky 10 deg from the Sun", 2)
        self._goto(sx + away[0], sy + away[1])
        self.g.track(self._fn, "sun scan").result(5)
        self.off[:] = list(away)
        time.sleep(2.0)
        floor = float(np.median([self.sensor.power_db() for _ in range(7)]))
        self.progress["floor_db"] = floor

        # 2. coarse cross-scans (alternate X/Y until both peaks found), then fine
        centre = [0.0, 0.0]
        # 2. fine cross-scan straight away if GNSS/Sun already gave a decent prior, otherwise (or if the
        #    fine scan finds nothing) search with wider coarse cuts first. Fewer points means less LNB heating.
        centre = [0.0, 0.0]
        res = None
        if self.cal.status in ("coarse", "fine"):
            res = self._fine(centre, floor, quiet=True)
            if self.cancelled:
                return self._finish(away)
        if res is None:
            centre = [0.0, 0.0]
            found = {"x": False, "y": False}
            for it in range(3):
                for ax in ("x", "y"):
                    r = self._cut(ax, centre, S.coarse_half_width_deg, S.coarse_step_deg, floor,
                                  f"coarse {it + 1}", (5 + 20 * it, 15 + 20 * it) if ax == "x" else (15 + 20 * it, 25 + 20 * it))
                    if self.cancelled:
                        return self._finish(away)
                    if r and r[2] >= S.min_rise_db:
                        centre[0 if ax == "x" else 1], found[ax] = r[0], True
                if all(found.values()):
                    break
            if not all(found.values()):
                self._finish(away)
                raise RuntimeError("Sun not found: no noise rise. Check LNB power, dish clear of shade, "
                                   "and that GNSS/HOME are roughly right.")
            res = self._fine(centre, floor)
            if self.cancelled:
                return self._finish(away)
        t = clock.now()
        az, el = sun_azel(t, site.latitude_deg, site.longitude_deg)
        x, y = self.sun_axes(t)
        point = {"x": x + centre[0], "y": y + centre[1], "az": az, "el": el, "t": t.isoformat(timespec="seconds"),
                 "rise_db": round(max(res["x"][2], res["y"][2]), 2),
                 # 1 deg of X moves the beam only cos(y) deg on the sky, so scale the X-cut width
                 "fwhm_deg": round((res["x"][1] * math.cos(math.radians(y + centre[1])) + res["y"][1]) / 2, 3),
                 "offset": [round(c, 3) for c in centre]}
        self._finish(away)
        self.cal.add("sun", point)
        fit = self.cal.data["fit"]
        self.say(f"Sun found {centre[0]:+.2f}/{centre[1]:+.2f} deg from prediction, rise {point['rise_db']:.1f} dB, "
                 f"beam {point['fwhm_deg']:.2f} deg. Fit RMS {fit['rms_sun']:.3f} deg", 100)
        self.progress["result"] = point

    def _fine(self, centre, floor, quiet=False):
        S = config.SUN_SCAN
        res = {}
        for ax in ("x", "y"):
            r = self._cut(ax, centre, S.fine_half_width_deg, S.fine_step_deg, floor, "fine",
                          (65, 80) if ax == "x" else (80, 95))
            if self.cancelled:
                return None
            if not r or r[2] < S.min_rise_db:
                if quiet:
                    self.say("Sun not in the fine scan - widening the search")
                    return None
                self._finish(self.away)
                raise RuntimeError(f"fine {ax.upper()} cut: no clean peak (rise {r[2] if r else 0:.1f} dB)")
            centre[0 if ax == "x" else 1] = r[0]
            res[ax] = r
        return res

    def _finish(self, away):
        self.off[:] = list(away)                          # move off the Sun (LNB heat)
        time.sleep(1.5)
        self.g.hold().result(3)
        self.away = None

    def cleanup(self):                                    # failure/cancel path: never stay on the Sun
        if self.away is not None and self.g.mode == "track":
            self._finish(self.away)
