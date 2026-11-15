"""
pass_sim.py - simulates a satellite pass for the X-Y gimbal (no hardware, no CAN traffic).

TLE sources:
    "celestrak" - CelesTrak NORAD GP data, by name (via tle_manager.TleManager)
    "spacex"    - CelesTrak SupGP Starlink file, built from SpaceX's own ephemerides.
                  CelesTrak allows one download per 2 hours, so it is cached to disk.

Zenith start: the virtual station is put at the satellite's sub-point when Track is pressed,
so the dish starts straight up (X = 0, Y = 0) and then follows the satellite for `duration_s`.

Needs: pip install skyfield requests numpy
"""

import logging
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import requests
from skyfield.api import EarthSatellite, load, wgs84

import config
import kinematics as K
from tle_manager import TleManager

logger = logging.getLogger(__name__)

COUNTS_PER_REV = config.COUNTS_PER_REV
HEADERS = {"User-Agent": "GroundStation-V2-Automated"}
SUPGP_URL = "https://celestrak.org/NORAD/elements/supplemental/sup-gp.php?FILE=starlink&FORMAT=TLE"
SUPGP_CACHE = Path("supgp_starlink_cache.txt")
SUPGP_MAX_AGE_S = 2 * 3600         # CelesTrak limit: one download per 2 hours
SEARCH_URL = "https://celestrak.org/NORAD/elements/gp.php?NAME={q}&FORMAT=TLE"

_ts = load.timescale()
_tle_manager = TleManager()


def _parse_tle_text(text: str) -> list[tuple[str, str, str]]:
    lines = [ln.rstrip() for ln in text.splitlines() if ln.strip()]
    out = []
    for i in range(len(lines) - 2):
        if lines[i + 1].startswith("1 ") and lines[i + 2].startswith("2 "):
            out.append((lines[i].strip(), lines[i + 1].strip(), lines[i + 2].strip()))
    return out


# -- TLE sources --

def _supgp_starlink() -> list[tuple[str, str, str]]:
    """SpaceX-derived Starlink TLEs, refreshed at most every 2 hours."""
    fresh = SUPGP_CACHE.exists() and time.time() - SUPGP_CACHE.stat().st_mtime < SUPGP_MAX_AGE_S
    if not fresh:
        try:
            resp = requests.get(SUPGP_URL, headers=HEADERS, timeout=30)
            resp.raise_for_status()
            if _parse_tle_text(resp.text):
                SUPGP_CACHE.write_text(resp.text)
            else:
                raise ValueError(resp.text.strip()[:200])
        except Exception as exc:
            if not SUPGP_CACHE.exists():
                raise RuntimeError(f"SpaceX SupGP download refused and no cache yet: {exc}") from exc
            logger.warning("SupGP refresh failed (%s); using cache", exc)
    return _parse_tle_text(SUPGP_CACHE.read_text())


def get_satellite(name: str, source: str = "celestrak") -> tuple[EarthSatellite, str, list[str]]:
    label = "CelesTrak GP (NORAD)"
    if source == "spacex":
        key = name.strip().upper()
        try:
            for n, l1, l2 in _supgp_starlink():
                if n.upper() == key or n.upper().startswith(key + " "):
                    return EarthSatellite(l1, l2, n, _ts), "CelesTrak SupGP (SpaceX ephemeris)", [n, l1, l2]
            label += f" - {name} not in SpaceX SupGP file"
        except RuntimeError as exc:
            logger.warning("%s; falling back to CelesTrak GP", exc)
            label += " - SpaceX SupGP unavailable (2 h download limit)"
    # look elements up by NORAD catalog number, names are ambiguous
    key = name.strip().upper()
    catnr = int(key) if key.isdigit() else next((c for c, n in config.TARGETS.items() if n.upper() == key), None)
    if catnr is None:
        raise ValueError(f"{name}: give a NORAD catalog number or one of {list(config.TARGETS.values())}")
    sat = _tle_manager.get_satellite(catnr)
    return sat, label, [sat.name, f"NORAD {catnr}", f"epoch {sat.epoch.utc_strftime('%Y-%m-%d %H:%M')}"]


def search(query: str, limit: int = 40) -> list[str]:
    resp = requests.get(SEARCH_URL.format(q=requests.utils.quote(query)), headers=HEADERS, timeout=15)
    resp.raise_for_status()
    return [n for n, _, _ in _parse_tle_text(resp.text)][:limit]


# -- Simulation --

def simulate(name: str, source: str = "celestrak", duration_s: float = 120.0,
             rate_hz: float = 10.0, start: datetime | None = None) -> dict:
    sat, source_label, tle = get_satellite(name, source)
    start = start or datetime.now(timezone.utc)
    t0 = _ts.from_datetime(start)

    # Station directly under the satellite at t0 so the dish starts at zenith
    sub = wgs84.subpoint_of(sat.at(t0))
    site = wgs84.latlon(sub.latitude.degrees, sub.longitude.degrees, elevation_m=0.0)

    dt = 1.0 / rate_hz
    secs = np.arange(0.0, duration_s + dt / 2, dt)
    t = _ts.tt_jd(t0.whole, t0.tt_fraction + secs / 86400.0)   # split JD keeps µs precision

    alt, az, dist = (sat - site).at(t).altaz()
    el_deg, az_deg, range_km = alt.degrees, az.degrees, dist.km

    x_deg, y_deg = K.azel_to_axes_vec(az_deg, el_deg)          # nominal mount (no calibration)
    x_rate, y_rate = np.gradient(x_deg, dt), np.gradient(y_deg, dt)
    x_acc, y_acc = np.gradient(x_rate, dt), np.gradient(y_rate, dt)
    range_rate = np.gradient(range_km, dt)

    geo = sat.at(t)
    lat, lon = wgs84.latlon_of(geo)
    height_km = wgs84.height_of(geo).km

    ax, ay = config.AXES["x"], config.AXES["y"]
    limits = {
        "x_min": ax.min_deg, "x_max": ax.max_deg, "x_speed": ax.track_speed_deg_s,
        "y_min": ay.min_deg, "y_max": ay.max_deg, "y_speed": ay.track_speed_deg_s,
    }
    violations = []
    if x_deg.min() < ax.min_deg or x_deg.max() > ax.max_deg:
        violations.append("X angle outside travel limits")
    if y_deg.min() < ay.min_deg or y_deg.max() > ay.max_deg:
        violations.append("Y angle outside travel limits")
    if np.abs(x_rate).max() > ax.track_speed_deg_s:
        violations.append("X speed above max_speed_deg_s")
    if np.abs(y_rate).max() > ay.track_speed_deg_s:
        violations.append("Y speed above max_speed_deg_s")
    if el_deg.min() < config.TRACKING.min_pass_elevation_deg:
        violations.append(f"elevation drops below {config.TRACKING.min_pass_elevation_deg:.0f} deg")

    epoch = sat.epoch.utc_datetime()

    def r(a, n=4):
        return np.round(a, n).tolist()

    return {
        "satellite": sat.name,
        "norad_id": int(sat.model.satnum),
        "source": source_label,
        "tle": tle,
        "tle_epoch": epoch.isoformat(),
        "tle_age_h": round((start - epoch).total_seconds() / 3600, 1),
        "start_utc": start.isoformat(),
        "duration_s": duration_s,
        "rate_hz": rate_hz,
        "counts_per_rev": COUNTS_PER_REV,
        "site": {"lat": round(float(sub.latitude.degrees), 5), "lon": round(float(sub.longitude.degrees), 5)},
        "limits": limits,
        "violations": violations,
        "t": r(secs, 2),
        "az": r(az_deg), "el": r(el_deg), "range_km": r(range_km, 2), "range_rate_km_s": r(range_rate),
        "x": r(x_deg), "y": r(y_deg),
        "x_rate": r(x_rate), "y_rate": r(y_rate),
        "x_acc": r(x_acc), "y_acc": r(y_acc),
        "lat": r(lat.degrees), "lon": r(lon.degrees), "height_km": r(height_km, 2),
    }


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    s = simulate("LUMELITE-4")
    i = len(s["t"]) - 1
    print(f"{s['satellite']} ({s['source']}), TLE age {s['tle_age_h']} h, station {s['site']}")
    print(f"  t=0   el {s['el'][0]:.2f}  X {s['x'][0]:+.3f}  Y {s['y'][0]:+.3f}")
    print(f"  t={s['t'][i]:.0f} el {s['el'][i]:.2f}  X {s['x'][i]:+.3f}  Y {s['y'][i]:+.3f}")
    print(f"  max |rate| X {max(map(abs, s['x_rate'])):.3f}  Y {max(map(abs, s['y_rate'])):.3f} deg/s")
    print(f"  violations: {s['violations'] or 'none'}")
