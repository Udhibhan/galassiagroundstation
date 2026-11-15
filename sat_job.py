"""
sat_job.py - satellite tracking, one button, narrated (the Satellite tab).

Pick a satellite (or "auto": the pass in progress or the next one) and press Track. The worker:
  1. picks the pass (rise / culmination / set from the orbital elements)
  2. gets fresh elements from CelesTrak (cached copy if offline; refuses elements older than
     TRACKING.tle_max_epoch_age_days)
  3. finds true north: reuses the mount yaw from an earlier Sun / Test run, else waits for GNSS RTK FIXED
     and averages the heading
  4. starts the motors if they are off
  5. precomputes the whole pass as X/Y axis angles and checks it against axis limits and speeds
     before anything moves
  6. slews to the rise point, waits for AOS, tracks until LOS, then goes back to HOME

Like the Sun and Test jobs it runs in its own thread and never touches CAN, it only gives the gimbal
thread goto / track targets. The page deadman (no heartbeat for 2 s) and STOP stop it like any other
goto/track. Receive only, nothing is transmitted.
"""
import logging
import threading
import time
from datetime import timedelta

import numpy as np
from skyfield.api import load, wgs84

import clock
import config
import kinematics as K
from pass_scheduler import find_passes
from tle_manager import TleManager
from trajectory import PassTable

log = logging.getLogger(__name__)


_ts = load.timescale()
MIN_SEGMENT_S = 60.0                         # a trackable stretch shorter than this is not worth a slew


class Abort(Exception):
    pass


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class SatSession:
    def __init__(self, gimbal, gnss, site_fn, mount_fn, sim=False, rx=None):
        self.g = gimbal
        self.gnss = gnss
        self.site_fn = site_fn              # returns (lat, lon, source)
        self.mount_fn = mount_fn            # returns mount yaw in degrees, or None if not known yet
        self.sim = sim
        self.rx = rx                          # .start(opts) / .stop(why) / .busy(), the receiver (None = no receiving)
        self.rx_opts = None                   # set per run from the page, None = track only
        self._rx_on = False
        self.tle = TleManager()
        self.targets = dict(config.TARGETS)
        self.passes = []
        self.pass_note = "predicting…"
        self._pred_at = 0.0
        self._pred_busy = False
        self.lock = threading.Lock()
        self.thread = None
        self.cancel = threading.Event()
        self.cancel_why = None
        self._act = None
        self.table = None
        self.cur = None
        self._feas_cache = {}
        self.prog = self._fresh()
        threading.Thread(target=self._predict_loop, name="sat-predict", daemon=True).start()

    # state
    @staticmethod
    def _fresh():
        return {"state": "idle", "step": "", "msg": "", "log": [], "sat": None, "t0": None, "t1": None}

    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def _note(self, text, level="info"):
        lg = self.prog.setdefault("log", [])
        lg.append({"t": clock.now().isoformat(timespec="seconds"), "msg": text, "level": level})
        del lg[:-120]
        log.info("Sat: %s", text)

    STAGE_OF = {"North": "north", "Finding north": "north", "Choosing the satellite": "pick", "Satellite": "pick",
                "Orbital elements": "elements", "Starting the motors": "motors", "Computing the path": "path",
                "Slewing to the rise point": "slew", "Waiting for the satellite to rise": "wait", "Tracking": "track",
                "Pass complete": "park"}

    def _say(self, step, msg=None):
        if step in self.STAGE_OF:
            self.prog["stage"] = self.STAGE_OF[step]
        self.prog["step"] = step
        self.prog["msg"] = msg or ""
        self._note(f"{step}: {msg}" if msg else step)

    def _site(self):
        lat, lon, _ = self.site_fn()
        return config.SiteConfig(lat, lon, config.SITE.elevation_m, "site")

    # predictions (background, network optional)
    def _predict_loop(self):
        while True:
            try:
                self.predict()
            except Exception as e:
                self.pass_note = f"prediction failed: {e}"
                log.warning("sat predict: %s", e)
            time.sleep(60)

    def predict(self, force=False):
        if self._pred_busy or (not force and self.passes and time.monotonic() - self._pred_at < 1800):
            return
        self._pred_busy = True
        try:
            errs = []
            try:
                errs = self.tle.refresh_all(list(self.targets))
            except Exception as e:
                errs = [str(e)]
            now, site, out = clock.now(), self._site(), []
            for catnr in self.targets:
                try:
                    sat = self.tle.get_satellite(catnr)
                    age = self.tle.epoch_age_days(sat)
                    for p in find_passes(sat, catnr, site, now - timedelta(minutes=20), config.TRACKING.lookahead_hours,
                                         config.TRACKING.min_pass_elevation_deg):
                        p.age_days = age
                        out.append(p)
                except Exception as e:
                    errs.append(f"{catnr}: {e}")
            out.sort(key=lambda p: p.rise)
            with self.lock:
                self.passes = out
            self._pred_at = time.monotonic()
            self.pass_note = ("" if not errs else "orbital elements: using the saved copy (" + "; ".join(errs[:2]) + ")")
        finally:
            self._pred_busy = False

    def add_target(self, catnr: int):
        catnr = int(catnr)
        self.tle.refresh(catnr, force=True)                  # raises if CelesTrak has nothing or is unreachable
        name = self.tle.get_satellite(catnr).name
        self.targets[catnr] = name
        self._pred_at = 0.0
        threading.Thread(target=self.predict, kwargs={"force": True}, daemon=True).start()
        return name

    def feasible(self, p, yaw):
        """Longest stretch of this pass the dish can follow (reachable sky, +/-80 deg travel, axis speeds).
        Returns {"ok", "why", "t0", "t1", "frac"}, t0/t1 are datetimes. Cached per pass + yaw."""
        key = (p.catnr, p.rise.isoformat(), round(yaw, 1))
        if key not in self._feas_cache:
            try:
                self._feas_cache[key] = self._feasible(p, yaw)
            except Exception as e:
                self._feas_cache[key] = {"ok": False, "why": str(e), "t0": None, "t1": None, "frac": 0.0}
            if len(self._feas_cache) > 400:
                self._feas_cache.clear()
        return self._feas_cache[key]

    def _feasible(self, p, yaw):
        sat, site, cal = self.tle.get_satellite(p.catnr), self._site(), {"yaw": yaw}
        start = max(p.rise, clock.now())
        if p.set <= start:
            return {"ok": False, "why": "already set", "t0": None, "t1": None, "frac": 0.0}
        step = 2.0
        n = int((p.set - start).total_seconds() / step) + 1
        t0 = _ts.from_datetime(start)
        times = _ts.tt_jd(t0.whole, t0.tt_fraction + np.arange(n) * step / 86400.0)
        topos = wgs84.latlon(site.latitude_deg, site.longitude_deg, elevation_m=site.elevation_m)
        alt, az, _ = (sat - topos).at(times).altaz()
        ok = np.zeros(n, bool)
        for i in range(n):
            try:
                x, y = K.azel_to_axes(float(az.degrees[i]), float(alt.degrees[i]), cal)
                ok[i] = K.within_limits(x, y, 0.5)
            except ValueError:
                pass
        best, i = (0, 0), 0
        while i < n:                                               # longest contiguous reachable run
            if ok[i]:
                j = i
                while j + 1 < n and ok[j + 1]:
                    j += 1
                if j - i > best[1] - best[0]:
                    best = (i, j)
                i = j + 1
            else:
                i += 1
        if not ok.any() or (best[1] - best[0]) * step < MIN_SEGMENT_S:
            return {"ok": False, "t0": None, "t1": None, "frac": 0.0,
                    "why": "the dish can't point at this part of the sky (the beam offset and the ±80° travel block it)"}
        sky = [[round(float(az.degrees[i]), 1), round(float(alt.degrees[i]), 1), int(ok[i])] for i in range(0, n, 5)]
        w0, w1 = start + timedelta(seconds=best[0] * step), start + timedelta(seconds=best[1] * step)
        tbl = PassTable(sat, site, w0, w1, cal, step_s=2.0)
        probs = tbl.problems()
        if probs:
            return {"ok": False, "t0": None, "t1": None, "frac": 0.0, "why": "; ".join(probs)}
        frac = (best[1] - best[0] + 1) / n
        return {"ok": True, "why": "" if frac > 0.98 else f"only {frac * 100:.0f}% of the pass is reachable",
                "t0": w0, "t1": w1, "frac": round(frac, 2), "sky": sky,
                "sky_t0": start.isoformat(), "sky_dt": 10}

    def upcoming(self, n=8):
        now = clock.now()
        yaw = self.mount_fn()
        approx = yaw is None
        with self.lock:
            ps = [p for p in self.passes if p.set > now][:n]
        out = []
        for p in ps:
            f = self.feasible(p, 0.0 if approx else yaw)
            out.append({"catnr": p.catnr, "name": p.name, "rise": p.rise.isoformat(), "set": p.set.isoformat(),
                        "culminate": p.culminate.isoformat(), "max_el": round(p.max_el, 1), "duration_s": round(p.duration_s),
                        "age_days": round(getattr(p, "age_days", 0), 1), "now": p.rise <= now,
                        "reachable": f["ok"], "why": f["why"], "frac": f["frac"], "approx": approx, "sky": f.get("sky"),
                        "sky_t0": f.get("sky_t0"), "sky_dt": f.get("sky_dt", 10),
                        "track_from": f["t0"].isoformat() if f["t0"] else None,
                        "track_to": f["t1"].isoformat() if f["t1"] else None})
        return out

    def candidates(self, catnr=None):
        now = clock.now()
        with self.lock:
            return [p for p in self.passes if p.set > now + timedelta(seconds=30) and (catnr is None or p.catnr == int(catnr))]

    # control
    def start(self, catnr=None, rx_opts=None):
        if self.busy():
            raise RuntimeError("satellite tracking is already running: press Stop first")
        if self.g.mode in ("offline", "fault"):
            raise RuntimeError(f"drives: {self.g.mode} (check 48 V, the CAN cable and the Pi relay)")
        st = self.gnss.status() if self.gnss else {"connected": False}
        if self.mount_fn() is None and not st.get("connected"):
            raise RuntimeError("true north is unknown: connect the GNSS heading kit (or run the Sun or Test north step first)")
        self.rx_opts = dict(rx_opts) if (rx_opts and self.rx is not None) else None
        self.cancel.clear()
        self.cancel_why = None
        self.table, self.cur = None, None
        self.prog = self._fresh()
        self.prog.update(state="running", t0=clock.now().isoformat(timespec="seconds"))
        self.thread = threading.Thread(target=self._worker, args=(catnr,), name="sat", daemon=True)
        self.thread.start()
        return "started"

    def abort(self, why="Stop"):
        if self.busy():
            self.cancel_why = why
            self.cancel.set()

    def _check(self):
        if self.cancel.is_set():
            raise Abort(self.cancel_why or "Stop")
        a = self._act
        if a is not None and a["event"].is_set() and a["ok"] is False:
            raise Abort(f"gimbal stopped ({a['why']})")

    def _sleep(self, dt):
        end = time.monotonic() + dt
        while time.monotonic() < end:
            self._check()
            time.sleep(min(0.1, max(0.0, end - time.monotonic())))

    # worker
    def _worker(self, catnr):
        try:
            self._run(catnr)
        except Abort as e:
            self.prog.update(state="aborted", msg=f"stopped: {e}")
            self._note(f"stopped: {e}", "warn")
            self._cleanup()
        except Exception as e:
            log.exception("Satellite tracking failed")
            self.prog.update(state="failed", msg=str(e))
            self._note(f"FAILED: {e}", "error")
            self._cleanup()
        finally:
            self.prog["t1"] = clock.now().isoformat(timespec="seconds")

    def _rx_begin(self, sat, p):
        """Start receiving as the satellite appears (the receiver needs a few seconds to open the radio and lock)."""
        if self._rx_on or self.rx_opts is None:
            return
        self._rx_on = True                                   # one attempt per pass
        o = dict(self.rx_opts)
        try:
            omm = (self.tle.cache.get(str(p.catnr)) or {}).get("omm")
            if omm and not o.get("tle"):                      # Doppler pre-compensation from the same elements
                import json, os
                d = os.path.join(config.DEMOD.out_root, "orbits"); os.makedirs(d, exist_ok=True)
                path = os.path.abspath(os.path.join(d, f"{p.catnr}.json"))
                with open(path, "w") as fh:
                    json.dump([omm], fh)
                o["tle"] = path
            self.rx.start(o)
            self._note(f"receiver started ({o.get('preset') or config.DEMOD.preset}): decoded files go to {config.DEMOD.out_root}/", "info")
        except Exception as e:
            self._note(f"receiver could not start: {e} (tracking continues)", "warn")

    def _rx_end(self):
        if self.rx is not None and self._rx_on:
            try:
                if self.rx.busy():
                    self.rx.stop("pass over")
                    self._note("receiver stopped (the pass is over)")
            except Exception as e:
                self._note(f"receiver stop: {e}", "warn")
        self._rx_on = False

    def _cleanup(self):
        self._rx_end()
        self._act = None
        try:
            if self.g.act is not None:
                self.g.stop_act("satellite tracking ended").result(3)
        except Exception:
            pass

    def _run(self, catnr):
        g, cfg = self.g, config.TRACKING
        # 1. north first: whether a pass can be followed depends on the mount yaw
        psi = self.mount_fn()
        if psi is not None:
            self._say("North", f"using mount yaw ψ = {psi:+.2f}° from the earlier Sun / Test run")
        else:
            psi = self._north_from_gnss()
        # 2. which pass: the soonest one the dish can actually follow
        self._say("Choosing the satellite", "waiting for the pass prediction" if not self.passes else None)
        end = time.monotonic() + 90
        while not self.passes and time.monotonic() < end:
            self._sleep(0.5)
        loc = lambda d: d.astimezone().strftime("%H:%M:%S")
        p, win = None, None
        for q in self.candidates(catnr):
            f = self.feasible(q, psi)
            if f["ok"]:
                p, win = q, f
                break
            self._note(f"skipping {q.name} {loc(q.rise)}–{loc(q.set)} (max {q.max_el:.0f}°): {f['why']}", "warn")
        if p is None:
            raise RuntimeError("no pass of " + (self.targets.get(int(catnr), str(catnr)) if catnr else "any target") +
                               f" in the next {cfg.lookahead_hours:.0f} h that the dish can follow (above {cfg.min_pass_elevation_deg:.0f}°)")
        self.cur = p
        self.prog["sat"] = p.name
        self._say("Satellite", f"{p.name} (NORAD {p.catnr}): rises {loc(p.rise)}, max elevation {p.max_el:.0f}°, sets {loc(p.set)} "
                  f"({p.duration_s / 60:.1f} min)" + ("  [auto-picked: " + ("in progress now" if p.rise <= clock.now() else "next trackable pass") + "]"
                                                      if catnr is None else ""))
        if win["frac"] < 0.98:
            self._note(f"the dish can follow {loc(win['t0'])}–{loc(win['t1'])} of it ({win['frac'] * 100:.0f}%): the rest is sky the "
                       "boresight offset / ±80° travel cannot reach", "warn")
        # 3. elements
        self._say("Orbital elements", "asking CelesTrak for the latest …")
        try:
            self.tle.refresh(p.catnr)
            self._note("CelesTrak answered (elements cached on disk)")
        except Exception as e:
            self._note(f"CelesTrak not reachable ({e}): using the saved copy", "warn")
        sat = self.tle.get_satellite(p.catnr)
        age = self.tle.epoch_age_days(sat)
        self._note(f"elements epoch {sat.epoch.utc_strftime('%Y-%m-%d %H:%M')} UTC, {age:.1f} days old")
        if not self.sim and age > cfg.tle_max_epoch_age_days:
            raise RuntimeError(f"elements are {age:.1f} days old (limit {cfg.tle_max_epoch_age_days:.0f}): refusing to track")
        # 4. motors
        if not (g.started and g.mode == "on"):
            self._say("Starting the motors", "Start sequence: set up both drives, wake X then Y")
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
        self._note("both motors ON and holding")
        # 5. the whole pass, checked before anything moves
        self._say("Computing the path", "az/el every 0.1 s → X/Y axis angles, checked against the ±80° limits and speeds")
        start = max(win["t0"], clock.now() + timedelta(seconds=5))
        stop_t = win["t1"]
        try:
            table = PassTable(sat, self._site(), start, stop_t, {"yaw": psi})
        except ValueError as e:
            raise RuntimeError(f"this pass can't be tracked: {e}")
        probs = table.problems()
        if probs:
            raise RuntimeError("this pass can't be tracked: " + "; ".join(probs))
        self.table = table
        x0, y0 = table.at(start)
        self._note(f"path OK: starts X {x0:+.1f}°, Y {y0:+.1f}°; elevation peaks at {max(table.el):.0f}°")
        # 6. slew, wait, track
        self._say("Slewing to the rise point", f"X {x0:+.1f}°, Y {y0:+.1f}°")
        act = g.goto(x0, y0, f"{p.name} rise").result(5)
        self._act = act
        while not act["event"].wait(0.2):
            self._check()
        if not act["ok"]:
            raise Abort(f"gimbal stopped ({act['why']})")
        last = 0.0
        while clock.now() < start:
            self._check()
            if self.rx_opts is not None and (start - clock.now()).total_seconds() <= 25:
                self._rx_begin(sat, p)
            if time.monotonic() - last > 10:
                last = time.monotonic()
                self._say("Waiting for the satellite to rise", f"AOS in {(start - clock.now()).total_seconds():.0f} s")
            time.sleep(0.2)
        self._rx_begin(sat, p)                                 # no-op if it already started during the wait
        self._say("Tracking", f"{p.name} until {loc(stop_t)}")
        self._act = g.track(lambda t: table.at(t), p.name).result(5)
        last = 0.0
        while clock.now() < stop_t:
            self._check()
            if time.monotonic() - last > 10:
                last = time.monotonic()
                ae = table.azel_at(clock.now())
                bx, by = g.angles()
                if ae:
                    self._note(f"{p.name}: az {ae[0]:.1f}° el {ae[1]:.1f}° → dish X {bx:+.2f}°, Y {by:+.2f}°; "
                               f"{'LOS' if stop_t >= p.set else 'end of reachable sky'} in {(stop_t - clock.now()).total_seconds():.0f} s")
            a = g.act
            if a is not None and a.get("clipped"):
                raise RuntimeError(f"left the reachable sky: {a.get('msg')}")
            time.sleep(0.25)
        # 7. the pass is over: stop receiving, park
        self._rx_end()
        self._say("Pass complete", "returning to HOME")
        g.stop_act("pass complete").result(5)
        act = g.goto(0.0, 0.0, "park").result(5)
        self._act = act
        while not act["event"].wait(0.2):
            self._check()
        self._act = None
        self.table = None
        self.prog.update(state="done", msg=f"{p.name}: pass tracked, dish back at HOME")
        self._note(self.prog["msg"])

    def _north_from_gnss(self):
        cfg = config.TEST
        st = self.gnss.status() if self.gnss else {"connected": False}
        if not st.get("connected"):
            raise RuntimeError("no GNSS and no earlier mount yaw: true north is unknown")
        self._say("Finding north", "waiting for the GNSS heading kit to reach RTK FIXED")
        t0, last = time.monotonic(), 0.0
        while True:
            self._check()
            st = self.gnss.status()
            if st.get("carr") == 2 and st.get("heading_valid"):
                break
            if time.monotonic() - t0 > cfg.gnss_timeout_s:
                raise RuntimeError(f"no RTK FIXED after {cfg.gnss_timeout_s:.0f} s: cannot find north")
            if time.monotonic() - last > 5:
                last = time.monotonic()
                self._note(f"GNSS: {st.get('num_sv') or 0} satellites, "
                           f"{['no RTK', 'RTK FLOAT', 'RTK FIXED'][min(st.get('carr') or 0, 2)]} ({time.monotonic() - t0:.0f} s)")
            time.sleep(0.5)
        self._note(f"RTK FIXED: averaging the heading for {cfg.gnss_heading_avg_s:.0f} s")
        avg = self.gnss.average_baseline(cfg.gnss_heading_avg_s, cancel=lambda: self.cancel.is_set())
        self._check()
        if avg is None:
            raise RuntimeError("RTK FIXED was lost during the heading average")
        bar = config.gnss_bar_deg()
        psi = wrap180(avg["heading"] - bar)
        self.found_yaw = psi
        self._note(f"baseline heading {avg['heading']:.2f}° true → mount yaw ψ = {psi:+.2f}°")
        return psi

    # for the page
    def status(self) -> dict:
        d = dict(self.prog)
        d["busy"] = self.busy()
        d["targets"] = [{"catnr": c, "name": n} for c, n in self.targets.items()]
        d["passes"] = self.upcoming()
        d["note"] = self.pass_note
        d["found_yaw"] = getattr(self, "found_yaw", None)
        cur, tbl = self.cur, self.table
        if cur is not None:
            d["pass"] = {"name": cur.name, "catnr": cur.catnr, "rise": cur.rise.isoformat(), "set": cur.set.isoformat(),
                         "max_el": round(cur.max_el, 1)}
        if tbl is not None:
            d["track"] = tbl.sky_track(10.0)
            d["track_t0"] = tbl.start.isoformat()
            ae = tbl.azel_at(clock.now())
            if ae:
                d["az"], d["el"] = round(ae[0], 2), round(ae[1], 2)
        return d
