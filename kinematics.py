"""
kinematics.py - the one copy of the X-Y gimbal geometry (used to be duplicated in trajectory.py,
track_sun.py, pass_sim.py and two HTML files).

Frames: ENU = (East, North, Up). Dish frame at HOME = ENU (dish face level).
  X axis: shaft along East, +X tips the dish toward South         = Rx(+x)
  Y axis: rides on the X frame, +Y tips the dish toward East      = Ry(+y)

Offset dish (config.BORESIGHT_OFFSET): the RF beam is not the dish face normal. In the dish frame
  boresight_body = R(offset) . [0,0,1]
     "+Y"/"-Y": R = Ry(+/-deg), exact: Rx(x) Ry(y) Ry(d) = Rx(x) Ry(y + d)
     "+X"/"-X": R = Rx(+/-deg), no closed form, so the inverse is solved numerically (Newton)
  boresight(x, y) = M . Rx(x - x0) . Ry(y - y0) . boresight_body
With deg = 0 this is the old model: x = atan2(-cos az, tan el), y = asin(sin az cos el).

Mount calibration:
  yaw     : mount rotated clockwise (seen from above) by yaw deg, Rz(-yaw), az_sky = az_mount + yaw
  tilt_e  : mount tilted about East (top toward South for +), Rx(tilt_e)
  tilt_n  : mount tilted about North (top toward East for +), Ry(tilt_n)
  x0, y0  : encoder offsets, true axis angle = axis_deg - x0
  sky_vector = M . Rx(x - x0) . Ry(y - y0) . body_vector,   M = Rz(-yaw) Rx(tilt_e) Ry(tilt_n)
"""
import math

import numpy as np

import config

D2R = math.pi / 180.0


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def azel_to_enu(az_deg, el_deg):
    az, el = az_deg * D2R, el_deg * D2R
    return np.array([math.cos(el) * math.sin(az), math.cos(el) * math.cos(az), math.sin(el)])


def enu_to_azel(v):
    v = np.asarray(v, float)
    v = v / np.linalg.norm(v)
    return math.degrees(math.atan2(v[0], v[1])) % 360.0, math.degrees(math.asin(max(-1.0, min(1.0, v[2]))))


PARAMS = ("yaw", "tilt_e", "tilt_n", "x0", "y0")
ZERO_CAL = {k: 0.0 for k in PARAMS}


# ------------------------------------------------------------------ boresight offset
def _offset(offset=None):
    """Returns (axis 'x'|'y'|None, signed degrees). offset=None reads config.BORESIGHT_OFFSET."""
    o = config.BORESIGHT_OFFSET if offset is None else offset
    deg = float(o.get("deg", 0.0) or 0.0)
    toward = str(o.get("toward", "+X")).strip().upper()
    if deg == 0.0:
        return None, 0.0
    if toward not in ("+X", "-X", "+Y", "-Y"):
        raise ValueError(f"BORESIGHT_OFFSET toward must be +X, -X, +Y or -Y, not {toward!r}")
    return toward[1].lower(), deg if toward[0] == "+" else -deg


def boresight_body(offset=None):
    ax, d = _offset(offset)
    z = np.array([0.0, 0.0, 1.0])
    if ax == "x":
        return rx(d * D2R) @ z
    if ax == "y":
        return ry(d * D2R) @ z
    return z


def mount_matrix(cal: dict):
    return rz(-cal.get("yaw", 0) * D2R) @ rx(cal.get("tilt_e", 0) * D2R) @ ry(cal.get("tilt_n", 0) * D2R)


def body_to_sky(x_deg, y_deg, cal: dict, body=None, offset=None):
    """Unit ENU vector of a dish-fixed direction for the given axis angles (default: RF boresight incl. offset)."""
    x = (x_deg - cal.get("x0", 0)) * D2R
    y = (y_deg - cal.get("y0", 0)) * D2R
    b = boresight_body(offset) if body is None else np.asarray(body, float)
    return mount_matrix(cal) @ rx(x) @ ry(y) @ b


def axes_to_azel(x_deg, y_deg, cal: dict = ZERO_CAL, offset=None):
    return enu_to_azel(body_to_sky(x_deg, y_deg, cal, offset=offset))


def _newton_x_offset(v, d_rad, guess=None):
    """Solve Rx(x) Ry(y) Rx(d) z = v for (x, y) in radians: Gauss-Newton on the 3-vector residual
    (2 unknowns). Returns (x, y, residual_norm)."""
    b = rx(d_rad) @ np.array([0.0, 0.0, 1.0])
    if guess is None:                                   # start from the no-offset closed form, minus d
        guess = (math.atan2(-v[1], v[2]) - d_rad, math.asin(max(-1.0, min(1.0, v[0]))))
    x, y = guess
    res = 1.0
    for _ in range(30):
        Rx, Ry = rx(x), ry(y)
        u = Rx @ Ry @ b
        r = u - v
        res = float(np.linalg.norm(r))
        if res < 1e-12:
            break
        dRx = np.array([[0, 0, 0], [0, -math.sin(x), -math.cos(x)], [0, math.cos(x), -math.sin(x)]])
        dRy = np.array([[-math.sin(y), 0, math.cos(y)], [0, 0, 0], [-math.cos(y), 0, -math.sin(y)]])
        J = np.column_stack([dRx @ Ry @ b, Rx @ dRy @ b])
        step, *_ = np.linalg.lstsq(J, -r, rcond=None)
        x, y = x + step[0], y + step[1]
    x = (x + math.pi) % (2 * math.pi) - math.pi
    y = (y + math.pi) % (2 * math.pi) - math.pi
    return x, y, res


def azel_to_axes(az_deg, el_deg, cal: dict = ZERO_CAL, offset=None):
    """Axis angles (x, y) in degrees that put the RF boresight on a sky direction. Inverse of body_to_sky.
    Raises ValueError if the boresight can't point there at all (some directions are unreachable
    with an X offset)."""
    v = mount_matrix(cal).T @ azel_to_enu(az_deg, el_deg)
    ax, d = _offset(offset)
    if ax == "x":
        x, y, res = _newton_x_offset(v, d * D2R)
        if res > 1e-6:
            raise ValueError(f"az {az_deg:.1f} el {el_deg:.1f}: beam can't point there (offset {d:+.1f} deg on X)")
        x, y = math.degrees(x), math.degrees(y)
    else:
        t = math.degrees(math.asin(max(-1.0, min(1.0, v[0]))))          # t = y + d (exact Y offset)
        y = t - (d if ax == "y" else 0.0)
        x = math.degrees(math.atan2(-v[1], v[2]))
        if ax == "y" and not within_limits(x + cal.get("x0", 0), y + cal.get("y0", 0)):
            # same beam direction, other pose: y + d = 180 - t, x + 180 (only matters when y + d passes 90)
            y2 = (180.0 - t) - d
            y2 = (y2 + 180.0) % 360.0 - 180.0
            x2 = (x + 360.0) % 360.0 - 180.0
            if within_limits(x2 + cal.get("x0", 0), y2 + cal.get("y0", 0)):
                x, y = x2, y2
    return x + cal.get("x0", 0), y + cal.get("y0", 0)


def azel_to_axes_vec(az_deg, el_deg, cal: dict = ZERO_CAL, offset=None):
    """Vectorised version for whole pass tables."""
    ax, d = _offset(offset)
    if ax == "x":                                       # numeric inverse: loop
        az, el = np.atleast_1d(az_deg), np.atleast_1d(el_deg)
        out = [azel_to_axes(a, e, cal, offset) for a, e in zip(az, el)]
        return np.array([o[0] for o in out]), np.array([o[1] for o in out])
    az, el = np.radians(az_deg), np.radians(el_deg)
    enu = np.stack([np.cos(el) * np.sin(az), np.cos(el) * np.cos(az), np.sin(el)])
    v = mount_matrix(cal).T @ enu
    y = np.degrees(np.arcsin(np.clip(v[0], -1, 1))) - (d if ax == "y" else 0.0) + cal.get("y0", 0)
    x = np.degrees(np.arctan2(-v[1], v[2])) + cal.get("x0", 0)
    return x, y


def within_limits(x, y, margin=0.0) -> bool:
    lx, ly = config.AXES["x"], config.AXES["y"]
    return (lx.min_deg + margin <= x <= lx.max_deg - margin) and (ly.min_deg + margin <= y <= ly.max_deg - margin)


def reachable(az_deg, el_deg, cal: dict = ZERO_CAL, offset=None, margin=0.0):
    """Returns (x, y) if the beam can point there within the soft limits, else None."""
    try:
        x, y = azel_to_axes(az_deg, el_deg, cal, offset)
    except ValueError:
        return None
    return (x, y) if within_limits(x, y, margin) else None


def reach_grid(cal: dict = ZERO_CAL, offset=None, az_step=5, el_step=5):
    """Reachable sky for the UI: rows of 0/1 per elevation band (el centre el_step/2 ...), az bins of az_step."""
    rows = []
    for el in np.arange(el_step / 2, 90, el_step):
        rows.append("".join("1" if reachable(az + az_step / 2, el, cal, offset) else "0"
                            for az in range(0, 360, az_step)))
    return {"az_step": az_step, "el_step": el_step, "rows": rows}


def beam_scale(x_deg, y_deg, cal: dict = ZERO_CAL, offset=None, h=0.05):
    """Sky degrees the beam moves per degree of X and of Y at this pose (no offset: cos(y) and 1)."""
    b0 = body_to_sky(x_deg, y_deg, cal, offset=offset)
    sx = angle_between(b0, body_to_sky(x_deg + h, y_deg, cal, offset=offset)) / h
    sy = angle_between(b0, body_to_sky(x_deg, y_deg + h, cal, offset=offset)) / h
    return sx, sy


def angle_between(u, v) -> float:
    u, v = np.asarray(u, float), np.asarray(v, float)
    c = float(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v)))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def compass(az: float) -> str:
    return ["N", "NE", "E", "SE", "S", "SW", "W", "NW"][int(((az % 360) + 22.5) // 45) % 8]
