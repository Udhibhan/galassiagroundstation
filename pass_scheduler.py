"""
pass_scheduler.py - pass prediction and the autonomous tracking daemon (Auto mode).

Fixes over v2: targets by catalog number, no sleeping for hours (the daemon answers Stop), the pass
is precomputed and checked before AOS, tracking goes through gimbal.Gimbal (one CAN owner, proven
moves) instead of a 10 Hz velocity loop, and stale elements are refused.
"""
import logging
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timedelta

from skyfield.api import load, wgs84

import clock
import config
from tle_manager import TleManager
from trajectory import PassTable

log = logging.getLogger(__name__)


@dataclass
class Pass:
    catnr: int
    name: str
    rise: datetime
    culminate: datetime
    set: datetime
    max_el: float

    @property
    def duration_s(self):
        return (self.set - self.rise).total_seconds()


def find_passes(sat, catnr, site, start: datetime, hours: float, min_el: float) -> list[Pass]:
    ts = load.timescale()
    topos = wgs84.latlon(site.latitude_deg, site.longitude_deg, elevation_m=site.elevation_m)
    t, ev = sat.find_events(topos, ts.from_datetime(start), ts.from_datetime(start + timedelta(hours=hours)),
                            altitude_degrees=min_el)
    out, rise, culm, mx = [], None, None, None
    for ti, e in zip(t, ev):
        if e == 0:
            rise, culm, mx = ti, None, None
        elif e == 1 and rise is not None:
            el = (sat - topos).at(ti).altaz()[0].degrees
            if mx is None or el > mx:
                culm, mx = ti, el
        elif e == 2 and rise is not None:
            out.append(Pass(catnr, sat.name, rise.utc_datetime(), (culm if culm is not None else rise).utc_datetime(), ti.utc_datetime(),
                            float(mx or min_el)))
            rise = None
    return out


class AutoTracker:
    def __init__(self, gimbal, cal, site_fn, sim=False):
        self.g, self.cal, self.site_fn, self.sim = gimbal, cal, site_fn, sim
        self.tle = TleManager()
        self.running = False
        self.state, self.msg = "stopped", "Auto mode is off"
        self.passes: list[Pass] = []
        self.table: PassTable | None = None
        self.current: Pass | None = None
        self.skipped = set()
        self._predicted_at = 0.0
        self._tle_at = 0.0
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, name="auto", daemon=True)
        self._thread.start()

    # control
    def start(self):
        self.running = True
        self.msg = "starting"

    def stop(self):
        self.running = False
        if self.g.mode in ("track", "goto"):
            self.g.hold()
        self.current, self.table = None, None
        self.state, self.msg = "stopped", "Auto mode is off"

    # predictions
    def predict(self, force=False):
        if not force and time.monotonic() - self._predicted_at < 1800 and self.passes:
            return
        if time.monotonic() - self._tle_at > config.TRACKING.tle_refresh_s or not self._tle_at:
            errs = self.tle.refresh_all(config.TARGETS)
            self._tle_at = time.monotonic()
            for e in errs:
                log.warning("element refresh: %s", e)
        now, site, out = clock.now(), self.site_fn(), []
        for catnr in config.TARGETS:
            try:
                sat = self.tle.get_satellite(catnr)
                age = self.tle.epoch_age_days(sat)
                for p in find_passes(sat, catnr, site, now - timedelta(minutes=20), config.TRACKING.lookahead_hours,
                                     config.TRACKING.min_pass_elevation_deg):
                    p.age_days = age
                    out.append(p)
            except Exception as e:
                log.warning("prediction failed for %s: %s", catnr, e)
        out.sort(key=lambda p: p.rise)
        with self._lock:
            self.passes = out
        self._predicted_at = time.monotonic()

    def upcoming(self, n=6) -> list[dict]:
        now = clock.now()
        with self._lock:
            ps = [p for p in self.passes if p.set > now][:n]
        res = []
        for p in ps:
            d = asdict(p)
            d.update(rise=p.rise.isoformat(), set=p.set.isoformat(), culminate=p.culminate.isoformat(),
                     duration_s=p.duration_s, age_days=round(getattr(p, "age_days", 0), 1),
                     skipped=(p.catnr, p.rise.isoformat()) in self.skipped)
            res.append(d)
        return res

    # daemon
    def _run(self):
        while True:
            try:
                self._loop_once()
            except Exception as e:
                log.exception("auto")
                self.state, self.msg = "error", str(e)
                if self.g.mode in ("track", "goto"):
                    self.g.hold()
                time.sleep(5)
            time.sleep(1.0)

    def _loop_once(self):
        if not self.passes or time.monotonic() - self._predicted_at > 1800:
            self.predict()
        if not self.running:
            return
        if self.g.mode == "fault":
            self.running = False
            self.state, self.msg = "error", f"stopped: {self.g.fault}"
            return
        if self.g.mode == "disabled":
            self.g.enable().result(5)
            time.sleep(1.5)
        now = clock.now()
        nxt = next((p for p in self.passes if p.set > now + timedelta(seconds=20)
                    and (p.catnr, p.rise.isoformat()) not in self.skipped), None)
        if nxt is None:
            self.state, self.msg = "waiting", "no passes in the next 24 h"
            return
        lead = (nxt.rise - now).total_seconds()
        if lead > config.TRACKING.preslew_s:
            self.state = "waiting"
            self.msg = f"next: {nxt.name} at {nxt.rise.astimezone().strftime('%H:%M:%S')}, max {nxt.max_el:.0f} deg"
            if self.g.mode not in ("hold", "disabled"):
                self.g.hold()
            return
        self._do_pass(nxt)

    def _do_pass(self, p: Pass):
        key = (p.catnr, p.rise.isoformat())
        if not self.sim and getattr(p, "age_days", 0) > config.TRACKING.tle_max_epoch_age_days:
            self.skipped.add(key)
            log.warning("skip %s: elements %.1f days old", p.name, p.age_days)
            return
        sat = self.tle.get_satellite(p.catnr)
        start = max(p.rise, clock.now())
        table = PassTable(sat, self.site_fn(), start, p.set, self.cal.params)
        probs = table.problems()
        if probs:
            self.skipped.add(key)
            self.msg = f"skipped {p.name}: {'; '.join(probs)}"
            log.warning(self.msg)
            return
        self.current, self.table = p, table
        self.state, self.msg = "slewing", f"moving to {p.name} rise point"
        x0, y0 = table.at(start)
        self.g.goto(x0, y0).result(5)
        self.g.wait_until_still(120, lambda: not self.running)
        if not self.running:
            return
        while self.running and clock.now() < p.rise:
            self.state, self.msg = "ready", f"{p.name}: waiting at rise point, AOS in {(p.rise - clock.now()).total_seconds():.0f} s"
            time.sleep(0.5)
        self.g.track(table.at, p.name).result(5)
        while self.running and clock.now() < p.set and self.g.mode == "track":
            self.state = "tracking"
            self.msg = f"tracking {p.name}, LOS in {(p.set - clock.now()).total_seconds():.0f} s"
            time.sleep(0.5)
        self.skipped.add(key)                         # done or aborted, don't retry this pass
        if self.running and self.g.mode == "track":
            self.state, self.msg = "parking", "pass complete, returning home"
            self.g.goto(0.0, 0.0).result(5)
            self.g.wait_until_still(120, lambda: not self.running)
        self.current, self.table = None, None

    def snapshot(self) -> dict:
        d = {"running": self.running, "state": self.state, "msg": self.msg, "passes": self.upcoming(),
             "current": None}
        p, tbl = self.current, self.table
        if not (p and tbl) and d["passes"]:                     # preview the next pass on the sky plot
            nxt = next((q for q in d["passes"] if not q["skipped"]), None)
            if nxt:
                key = (nxt["catnr"], nxt["rise"])
                if getattr(self, "_preview_key", None) != key:
                    try:
                        q = next(q for q in self.passes if (q.catnr, q.rise.isoformat()) == key)
                        t = PassTable(self.tle.get_satellite(q.catnr), self.site_fn(), q.rise, q.set,
                                      self.cal.params, step_s=5.0)
                        self._preview = {"name": q.name, "track": t.sky_track(5.0), "rise": nxt["rise"]}
                    except Exception:
                        self._preview = None
                    self._preview_key = key
                d["preview"] = self._preview
        if p and tbl:
            now = clock.now()
            ae = tbl.azel_at(now)
            d["current"] = {"name": p.name, "catnr": p.catnr, "rise": p.rise.isoformat(), "set": p.set.isoformat(),
                            "max_el": p.max_el, "track": tbl.sky_track(), "az": ae[0] if ae else None,
                            "el": ae[1] if ae else None}
        return d
