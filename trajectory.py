"""
trajectory.py - precomputed pass table: az/el/range-rate -> calibrated X/Y axis angles.

v2 called Skyfield 3x per control cycle with finite differences and clamped silently.
v3 computes the whole pass once (vectorised), checks it against the axis limits and speeds BEFORE
AOS, and the control loop just interpolates the table.
"""
from dataclasses import dataclass
from datetime import datetime

import numpy as np
from skyfield.api import EarthSatellite, load, wgs84

import config
import kinematics as K

_ts = load.timescale()


@dataclass
class XYSample:                      # kept for doppler.py compatibility
    time: datetime
    x_deg: float
    y_deg: float
    x_rate_deg_s: float
    y_rate_deg_s: float
    range_km: float
    range_rate_km_s: float


class PassTable:
    def __init__(self, sat: EarthSatellite, site, start: datetime, end: datetime, cal: dict,
                 step_s: float = config.TRACKING.table_step_s):
        self.name, self.start, self.end = sat.name, start, end
        n = int((end - start).total_seconds() / step_s) + 1
        self.t = np.arange(n) * step_s
        t0 = _ts.from_datetime(start)
        times = _ts.tt_jd(t0.whole, t0.tt_fraction + self.t / 86400.0)
        topos = wgs84.latlon(site.latitude_deg, site.longitude_deg, elevation_m=site.elevation_m)
        alt, az, dist = (sat - topos).at(times).altaz()
        self.az, self.el, self.range_km = az.degrees, alt.degrees, dist.km
        self.range_rate = np.gradient(self.range_km, step_s)
        self.x, self.y = K.azel_to_axes_vec(self.az, self.el, cal)
        self.x_rate, self.y_rate = np.gradient(self.x, step_s), np.gradient(self.y, step_s)

    def problems(self) -> list[str]:
        out = []
        for k, a, r in (("X", self.x, self.x_rate), ("Y", self.y, self.y_rate)):
            c = config.AXES[k.lower()]
            if a.min() < c.min_deg or a.max() > c.max_deg:
                out.append(f"{k} leaves travel ({a.min():+.1f}..{a.max():+.1f} deg)")
            if np.abs(r).max() > c.track_speed_deg_s:
                out.append(f"{k} needs {np.abs(r).max():.2f} deg/s (> {c.track_speed_deg_s})")
        return out

    def at(self, when: datetime):
        s = (when - self.start).total_seconds()
        if s < 0 or s > self.t[-1]:
            return None
        return float(np.interp(s, self.t, self.x)), float(np.interp(s, self.t, self.y))

    def azel_at(self, when: datetime):
        s = (when - self.start).total_seconds()
        if s < 0 or s > self.t[-1]:
            return None
        return float(np.interp(s, self.t, self.az)), float(np.interp(s, self.t, self.el))

    def sky_track(self, every_s=10.0):
        k = max(1, int(every_s / (self.t[1] - self.t[0])))
        return [[round(float(a), 2), round(float(e), 2)] for a, e in zip(self.az[::k], self.el[::k])]

    def sample(self, when: datetime) -> XYSample:
        s = (when - self.start).total_seconds()
        f = lambda arr: float(np.interp(s, self.t, arr))
        return XYSample(when, f(self.x), f(self.y), f(self.x_rate), f(self.y_rate), f(self.range_km), f(self.range_rate))
