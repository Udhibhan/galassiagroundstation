"""
tle_manager.py - orbital elements by NORAD catalog number, OMM JSON first.

Changes from v2:
  * looks up by CATNR, not name ("ISS" matched UME, NORAD 8709, a 1976 satellite)
  * OMM JSON, since CelesTrak ran out of 5-digit catalog numbers 2026-07-11 and new objects are OMM-only
  * stale means element epoch age, not download time. Refreshes at most every 2 h (CelesTrak policy)
  * cache keyed by catalog number, old TLE-line entries still accepted
"""
import json
import logging
import os
import time

import requests
from skyfield.api import EarthSatellite, load

import clock
import config

log = logging.getLogger(__name__)
URL = "https://celestrak.org/NORAD/elements/gp.php?CATNR={catnr}&FORMAT=json"
HEADERS = {"User-Agent": "NUS-GroundStation/3"}


class TleManager:
    def __init__(self, path=config.TLE_CACHE_PATH):
        self.path = path
        self.ts = load.timescale()
        self.cache = {}
        if os.path.exists(path):
            try:
                with open(path) as fh:
                    self.cache = json.load(fh)
            except Exception as e:
                log.warning("element cache unreadable (%s)", e)

    def _save(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(self.cache, fh, indent=1)
        os.replace(tmp, self.path)

    def refresh(self, catnr: int, force=False) -> None:
        e = self.cache.get(str(catnr))
        if e and not force and time.time() - e.get("fetched_at", 0) < config.TRACKING.tle_refresh_s:
            return
        r = requests.get(URL.format(catnr=catnr), headers=HEADERS, timeout=15)
        r.raise_for_status()
        data = r.json()
        if not data:
            raise ValueError(f"CelesTrak has no elements for {catnr}")
        self.cache[str(catnr)] = {"omm": data[0], "fetched_at": time.time()}
        self._save()
        log.info("elements updated: %s (%s)", data[0].get("OBJECT_NAME"), catnr)

    def refresh_all(self, catnrs) -> list[str]:
        errors = []
        for c in catnrs:
            try:
                self.refresh(c)
            except Exception as e:
                errors.append(f"{c}: {e}")
        return errors

    def get_satellite(self, catnr: int) -> EarthSatellite:
        e = self.cache.get(str(catnr))
        if e is None:
            self.refresh(catnr)
            e = self.cache[str(catnr)]
        if "omm" in e:
            return EarthSatellite.from_omm(self.ts, e["omm"])
        return EarthSatellite(e["line1"], e["line2"], e.get("name", str(catnr)), self.ts)

    def epoch_age_days(self, sat: EarthSatellite) -> float:
        return (clock.now() - sat.epoch.utc_datetime()).total_seconds() / 86400.0


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    m = TleManager()
    for c, name in config.TARGETS.items():
        try:
            m.refresh(c)
            s = m.get_satellite(c)
            print(f"{c:>6} {s.name:<14} epoch {s.epoch.utc_strftime('%Y-%m-%d %H:%M')}  age {m.epoch_age_days(s):.1f} d")
        except Exception as e:
            print(f"{c:>6} {name:<14} FAILED: {e}")
