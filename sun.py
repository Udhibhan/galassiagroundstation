"""
sun.py - Sun azimuth/elevation without any download (Meeus / NOAA low-precision algorithm).
Accuracy ~0.01 deg (checked against Skyfield + DE421), plenty for a ~1.7 deg beam.
Replaces track_sun.py's 17 MB de421.bsp download, which also did NOT apply refraction
(.apparent().altaz() without temperature/pressure returns the airless position).
"""
import math
from datetime import datetime

D2R = math.pi / 180.0


def refraction_deg(el_deg: float, temp_c: float = 30.0, pressure_hpa: float = 1008.0) -> float:
    """Bennett (1982), scaled for Singapore air. Optical; close enough for Ku-band above ~10 deg."""
    if el_deg < -1.0:
        return 0.0
    r_arcmin = 1.02 / math.tan((el_deg + 10.3 / (el_deg + 5.11)) * D2R)
    return r_arcmin / 60.0 * (pressure_hpa / 1010.0) * (283.0 / (273.0 + temp_c))


def sun_azel(t: datetime, lat_deg: float, lon_deg: float, refraction: bool = True) -> tuple[float, float]:
    jd = t.timestamp() / 86400.0 + 2440587.5
    n = jd - 2451545.0
    T = n / 36525.0
    L0 = (280.46646 + 36000.76983 * T + 0.0003032 * T * T) % 360.0
    M = (357.52911 + 35999.05029 * T - 0.0001537 * T * T) * D2R
    C = ((1.914602 - 0.004817 * T - 0.000014 * T * T) * math.sin(M)
         + (0.019993 - 0.000101 * T) * math.sin(2 * M) + 0.000289 * math.sin(3 * M))
    omega = (125.04 - 1934.136 * T) * D2R
    lam = (L0 + C - 0.00569 - 0.00478 * math.sin(omega)) * D2R
    eps0 = 23.0 + (26.0 + (21.448 - T * (46.815 + T * (0.00059 - T * 0.001813))) / 60.0) / 60.0
    eps = (eps0 + 0.00256 * math.cos(omega)) * D2R
    ra = math.atan2(math.cos(eps) * math.sin(lam), math.cos(lam))
    dec = math.asin(math.sin(eps) * math.sin(lam))
    gmst = (280.46061837 + 360.98564736629 * n + 0.000387933 * T * T - T ** 3 / 38710000.0) % 360.0
    H = (gmst + lon_deg) * D2R - ra
    lat = lat_deg * D2R
    el = math.asin(math.sin(lat) * math.sin(dec) + math.cos(lat) * math.cos(dec) * math.cos(H))
    az = math.atan2(-math.cos(dec) * math.sin(H),
                    math.sin(dec) * math.cos(lat) - math.cos(dec) * math.cos(H) * math.sin(lat))
    el_deg = el / D2R
    if refraction:
        el_deg += refraction_deg(el_deg)
    return (az / D2R) % 360.0, el_deg
