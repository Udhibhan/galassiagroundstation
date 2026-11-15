"""
sim_hw.py - hardware simulator for `python app.py --sim`. No CAN, no serial, no SDR touched.

* SimCanLink speaks the real eRob CAN-custom protocol frames, so the real Drive/Gimbal code runs
  unchanged against it. It models what was learned on the bench (test stage, 2026-11):
    - an enabled drive that gets no START within ~0.5 s switches its motor OFF
    - START sent < 0.5 s after a motor-off is accepted, then the motor drops out
    - 0x4E / 0x8D written while ON -> refused 0x16
    - 0x83 START while still moving -> refused 0x15; START while OFF -> refused 0x27
    - 0x8A speed written while moving -> refused 0x15 (or 0x16 always, with refuse_live_speed)
    - after a completed move (or STOP) the motor stays ON and holds (state 4) indefinitely
    - 0x0008 current (signed mA): ~0.9 A holding, ~2.5 A moving; 0x0026 temperature; 0x0024 bus mV
* Test hooks (SimWorld.opts / SimDrive fields), reachable from app.py's sim-only "sim" command:
    fake_current_a  force |current| (A) while ON          refuse_start  next START refused with this code
    refuse_live_speed  0x8A refused 0x16 while ON          ignore_stop   0x84 accepted but not obeyed
    set_deg         teleport the axis to this angle from HOME
    current_model   "bench" (default, values above) | "load": loaded-dish model for current_char.py:
                    X: 1.7 A x sign(motion) + gravity 7.6 A (+ side) / 6.6 A (- side) x sin(x) x (1 - 0.25 (1 - cos y))
                    Y: 1.2 A x sign(motion) + gravity 3.0 A x sin(y) x cos(x); holding keeps +/-0.5 x friction
                    in the approach direction; noise 0.06 A; drive temperature follows I^2 (tau 600 s)
    gain_scale      multiply that axis' gravity term (current-stop trip tests)
    fake_temp_c     force the temperature read-back (temperature-abort tests)    bus_v  force the bus voltage
    (world-wide)    saturate: SDR peak 0.8 FS     gnss: "fixed" | "float" | "off"     sdr: "on" | "off"
* HIDDEN TRUTH for the Sun / GNSS simulation (the software must FIND these):
    mount yaw +4.0 deg, base tilt 0.5 deg, and the configured BORESIGHT_OFFSET (22 deg offset dish).
  SimSunSensor: cold-sky floor + Gaussian Sun beam (FWHM 1.6 deg, +9 dB) at the TRUE beam direction.
  SimGnss: NAV-RELPOSNED heading = bar direction + true yaw + 0.3 deg noise, carrSoln 2 (RTK FIXED).
"""
import math
import random
import struct
import threading
import time
from collections import deque

import clock
import config
import kinematics as K
from sdr_power import PowerMeter
from sun import sun_azel

# DISABLED (test stage): old hidden calibration for the GNSS-pose job
# TRUE_CAL = {"yaw": 3.2, "tilt_e": 0.45, "tilt_n": -0.35, "x0": 0.8, "y0": -0.6}
TRUE_YAW_DEG = 4.0          # hidden mount yaw (Sun methods 1 and 2 must find it within 0.5 deg)
TRUE_TILT = {"tilt_e": 0.30, "tilt_n": 0.40}     # hidden base tilt: 0.5 deg in total
SUN_FWHM_DEG = 1.6
SUN_RISE_DB = 9.0
FLOOR_DBFS = -42.0
CPR = config.COUNTS_PER_REV
IDLE_OFF_S = 0.5            # enabled with no START for this long -> motor OFF
REENABLE_MIN_S = 0.5        # START this soon after a motor-off -> drop out
SIM_ERR_IDLE = 0x0A01       # simulator's own error codes (the real codes are unknown)
SIM_ERR_DROP = 0x0A02


def _c(deg):
    return deg / 360.0 * CPR


class _SimDrive:
    def __init__(self, node, home_counts, start_deg):
        self.node = node
        self.home = home_counts
        self.pos = home_counts + _c(start_deg)
        self.on = False
        self.moved = False         # state 4 (holding) vs 1 (on, idle)
        self.t_on = 0.0
        self.t_off = -10.0
        self.armed = False         # enabled, waiting for START (idle-off timer running)
        self.drop_at = None
        self.regs = {0x004E: 0, 0x008D: 0, 0x009E: 0, 0x0088: 0, 0x0089: 0, 0x008A: 0, 0x0087: 0}
        self.move = None           # (t0, p0, p1, v, a)
        self.decel = None          # STOP ramp: (t0, p0, v0 signed counts/s, a)
        self.err = 0x0C26          # stored code after power-up (harmless)
        # test hooks
        self.fake_current_a = None
        self.refuse_start = None
        self.ignore_stop = False
        self.refuse_live_speed = False
        self.temp = 34.0 + node
        # loaded-dish model (SimWorld current_model = "load")
        self.world = None
        self.last_dir = 0          # direction of the last move (holding friction keeps this sign)
        self.gain_scale = 1.0
        self.fake_temp_c = None
        self.settle_err_deg = 0.0  # overshoot at the end of each move (the 14 Nov bench run stopped at +80.004)
        self.t_temp = time.monotonic()

    # trapezoidal profile ------------------------------------------------------
    def _profile(self, t):
        t0, p0, p1, v, a = self.move
        d, s = abs(p1 - p0), (1 if p1 >= p0 else -1)
        ta = v / a
        if a * ta * ta >= d:                         # triangle
            ta = math.sqrt(d / a)
            v = a * ta
            tc = 0.0
        else:
            tc = (d - a * ta * ta) / v
        dt = t - t0
        if dt >= 2 * ta + tc:
            return p1, True
        if dt < ta:
            x = 0.5 * a * dt * dt
        elif dt < ta + tc:
            x = 0.5 * a * ta * ta + v * (dt - ta)
        else:
            r = 2 * ta + tc - dt
            x = d - 0.5 * a * r * r
        return p0 + s * x, False

    def _decel_pos(self, t):
        t0, p0, v0, a = self.decel
        ts = abs(v0) / a
        dt = min(t - t0, ts)
        return p0 + v0 * dt - math.copysign(0.5 * a * dt * dt, v0), t - t0 >= ts

    def _motor_off(self, err=None):
        self.on, self.armed, self.move, self.drop_at, self.decel = False, False, None, None, None
        self.t_off = time.monotonic()
        if err is not None:
            self.err = err

    def update(self):
        now = time.monotonic()
        if self.world is not None and self.world.current_model == "load":
            dt = min(now - self.t_temp, 5.0)               # drive temperature: first order in I^2
            if dt > 0:
                i = abs(self._load_current_a(noise=False)) if self.on else 0.0
                t_amb = 34.0 + self.node
                self.temp += dt * (t_amb + 0.12 * i * i - self.temp) / 600.0
        self.t_temp = now
        if self.move:
            p, done = self._profile(now)
            self.pos = p
            if done:
                if self.settle_err_deg and self.move[2] != self.move[1]:   # real drives stop a hair past the target
                    self.pos = p + (1 if self.move[2] > self.move[1] else -1) * _c(self.settle_err_deg)
                self.move = None
        if self.decel:
            p, done = self._decel_pos(now)
            self.pos = p
            if done:
                self.decel = None
        if self.on and self.armed and now - self.t_on > IDLE_OFF_S:
            self._motor_off(SIM_ERR_IDLE)               # "enable and wait" -> drive switches OFF
        if self.drop_at is not None and now >= self.drop_at:
            self._motor_off(SIM_ERR_DROP)               # re-enabled too soon after motor-off

    def moving(self):
        self.update()
        return self.move is not None or self.decel is not None

    def set_deg(self, deg):
        self.update()
        self.move = self.decel = None
        self.pos = self.home + _c(deg)

    def _axis_deg(self, node):
        d = self.world.drives[node]
        sign = config.X_AXIS.sign if node == 1 else config.Y_AXIS.sign
        return sign * (((d.pos - d.home) * 360.0 / CPR + 180) % 360 - 180)

    def _load_current_a(self, noise=True):
        """Loaded-dish model (signed A): friction x sign(motion) - gravity (the motor holds the load against it)."""
        x, y = math.radians(self._axis_deg(1)), math.radians(self._axis_deg(2))
        moving = self.move is not None or self.decel is not None
        if self.move is not None:
            vdir = 1 if self.move[2] >= self.move[1] else -1
        elif self.decel is not None:
            vdir = 1 if self.decel[2] > 0 else -1
        else:
            vdir = 0
        if self.node == 1:
            fric, g = 1.7, (7.6 if x >= 0 else 6.6) * math.sin(x) * (1 - 0.25 * (1 - math.cos(y)))
        else:
            fric, g = 1.2, 3.0 * math.sin(y) * math.cos(x)
        f = fric * vdir if moving else 0.5 * fric * self.last_dir
        return f - self.gain_scale * g + (random.gauss(0, 0.06) if noise else 0.0)

    def _current_ma(self):
        if not self.on:
            return 0
        if self.fake_current_a is not None:
            return int(self.fake_current_a * 1000)
        if self.world is not None and self.world.current_model == "load":
            return int(self._load_current_a() * 1000)
        if self.move:
            t0, p0, p1, v, a = self.move
            return int(math.copysign(2500, p1 - p0) + random.gauss(0, 60))
        if self.decel:
            return int(math.copysign(1800, self.decel[2]) + random.gauss(0, 60))
        return int(-900 + random.gauss(0, 30))

    def handle(self, data: bytes) -> bytes:
        self.update()
        idx = struct.unpack(">H", data[:2])[0]
        val = struct.unpack(">i", data[2:6])[0] if len(data) >= 6 else None
        ok, ref = b"\x3e", (lambda code: code.to_bytes(2, "big") + b"\x3e")
        if val is None:                               # read or trigger
            if idx == 0x0083:
                if not self.on:
                    return ref(0x27)
                if self.moving():
                    return ref(0x15)
                if self.refuse_start is not None:
                    code, self.refuse_start = self.refuse_start, None
                    return ref(code)
                v, a = max(self.regs[0x008A], 1), max(self.regs[0x0088], 1)
                rel = self.regs[0x0087]
                self.move = (time.monotonic(), self.pos, self.pos + rel, v, a)
                if rel:
                    self.last_dir = 1 if rel > 0 else -1
                self.armed, self.moved = False, True
                if time.monotonic() - self.t_off < REENABLE_MIN_S:
                    self.drop_at = time.monotonic() + 0.1
                return ok
            if idx == 0x0084:
                if self.move and not self.ignore_stop:      # decelerate at 0x89 from the current speed
                    a = max(self.regs[0x0089], 1)
                    t = time.monotonic()
                    p_now, _ = self._profile(t)
                    p_dt, _ = self._profile(t + 0.01)
                    vel = (p_dt - p_now) / 0.01
                    self.move, self.pos = None, p_now
                    self.decel = (t, p_now, vel if abs(vel) > 1 else 1.0, a)
                return ok
            # reads = {0x0002: int(self.pos) % CPR, 0x0008: self._current_ma(), 0x001F: self.err,
            #          0x0024: int(47500 + random.gauss(0, 40)), 0x0026: int(round(self.temp)),
            bus_mv = self.world.bus_mv() if self.world is not None else int(47500 + random.gauss(0, 40))
            temp = self.temp if self.fake_temp_c is None else self.fake_temp_c
            reads = {0x0002: int(self.pos) % CPR, 0x0008: self._current_ma(), 0x001F: self.err,
                     0x0024: bus_mv, 0x0026: int(round(temp)),
                     0x0043: self.node,
                     0x010C: (0 if not self.on else 2 if self.moving() else 4 if self.moved else 1)}
            if idx in reads:
                return struct.pack(">i", reads[idx]) + b"\x3e"
            if idx in self.regs:
                return struct.pack(">i", self.regs[idx]) + b"\x3e"
            return ref(0x0D)
        if idx in (0x0050, 0x0052, 0x0053, 0x00E8):
            raise AssertionError(f"simulator: software wrote forbidden register 0x{idx:04X}")
        if idx == 0x0100:
            if val and not self.on:
                self.on, self.t_on, self.armed, self.moved = True, time.monotonic(), True, False
                self.err = 0                          # the power-up code clears on enable
            elif not val:
                self._motor_off()
            return ok
        if idx in (0x004E, 0x008D) and self.on:
            return ref(0x16)
        if idx == 0x008A and self.on:
            if self.moving():
                return ref(0x15)
            if self.refuse_live_speed:
                return ref(0x16)
        if idx in self.regs:
            self.regs[idx] = val
            return ok
        return ref(0x0D)

    def true_axis_deg(self, sign):
        """True angle from HOME (HOME = dish face level), as the kinematics define it."""
        self.update()
        d = (self.pos - self.home) * 360.0 / CPR
        return sign * ((d + 180) % 360 - 180)


class SimWorld:
    _inst = None

    def __init__(self):
        self.drives = {1: _SimDrive(1, config.HOME_COUNTS["x"], 0.30),
                       2: _SimDrive(2, config.HOME_COUNTS["y"], -0.20)}
        self.lock = threading.Lock()
        self.saturate = False
        self.gnss_mode = "fixed"
        self.gnss_t0 = time.monotonic()
        self.sdr_on = True
        self.current_model = "bench"      # "bench" | "load" (current_char.py)
        self.bus_v_override = None
        for d in self.drives.values():
            d.world = self

    def bus_mv(self):
        if self.bus_v_override is not None:
            return int(self.bus_v_override * 1000)
        return int(47500 + random.gauss(0, 40))

    def gnss_state(self):
        """"off" | "nofix" | "fix" | "float" | "fixed". Mode "acquire" walks through them in ~14 s (cold start)."""
        m = self.gnss_mode
        if m != "acquire":
            return m
        t = time.monotonic() - self.gnss_t0
        return "nofix" if t < 3 else "fix" if t < 7 else "float" if t < 11 else "fixed"

    @classmethod
    def get(cls):
        if cls._inst is None:
            cls._inst = SimWorld()
        return cls._inst

    @classmethod
    def reset(cls):
        cls._inst = None
        return cls.get()

    def drive(self, axis: str) -> _SimDrive:
        return self.drives[config.AXES[axis].node_id]

    def set_option(self, axis: str, name: str, value):
        if name == "saturate":
            self.saturate = bool(value)
            return
        if name == "gnss":
            if value not in ("fixed", "float", "off", "acquire"):
                raise ValueError("gnss must be fixed | float | off | acquire")
            self.gnss_mode = value
            self.gnss_t0 = time.monotonic()
            return
        if name == "sdr":
            self.sdr_on = value in (True, "on", 1)
            return
        if name == "current_model":
            if value not in ("bench", "load"):
                raise ValueError("current_model must be bench | load")
            self.current_model = value
            return
        if name == "bus_v":
            self.bus_v_override = None if value is None else float(value)
            return
        with self.lock:
            d = self.drive(axis)
            if name == "set_deg":
                d.set_deg(float(value))
            elif name == "drop_now":                   # the drive switches its motor OFF now (drop-out tests)
                d.update()
                d._motor_off(SIM_ERR_DROP)
            # elif name in ("fake_current_a", "refuse_start", "ignore_stop", "refuse_live_speed"):
            elif name in ("fake_current_a", "refuse_start", "ignore_stop", "refuse_live_speed",
                          "gain_scale", "fake_temp_c", "settle_err_deg"):
                setattr(d, name, value)
            else:
                raise ValueError(f"unknown sim option {name!r}")

    def true_axes(self):
        with self.lock:
            return (self.drives[1].true_axis_deg(config.X_AXIS.sign),
                    self.drives[2].true_axis_deg(config.Y_AXIS.sign))

    @staticmethod
    def true_cal():
        return dict(K.ZERO_CAL, yaw=TRUE_YAW_DEG, **TRUE_TILT)

    def true_vector(self):
        """TRUE beam direction (ENU): hidden yaw + tilt + the configured boresight offset."""
        x, y = self.true_axes()
        return K.body_to_sky(x, y, self.true_cal())


class SimCanLink:
    name = "SIMULATED CAN (no hardware)"

    def __init__(self):
        self.world = SimWorld.get()
        self.q = deque()

    def send(self, can_id, data):
        node = can_id - 0x640
        d = self.world.drives.get(node)
        if d is None:
            return
        with self.world.lock:
            reply = d.handle(bytes(data))
        self.q.append((0x5C0 + node, reply))

    def recv(self, timeout):
        if self.q:
            return self.q.popleft()
        if timeout > 0:
            time.sleep(min(timeout, 0.001))
        return None

    def close(self):
        pass


class SimGnss:
    """Same interface as gps_reader.GnssReader. The heading bar sits on the base (does not move with X/Y)."""
    name = "SIMULATED GNSS"

    def __init__(self):
        self.world = SimWorld.get()
        self.port = "sim"

    def start(self):
        pass

    def stop(self):
        pass

    def _heading(self):
        bar = config.gnss_bar_deg()
        return (bar + TRUE_YAW_DEG + random.gauss(0, 0.3)) % 360

    def status(self):
        m = self.world.gnss_state()
        if m == "off":
            return {"connected": False, "port": None, "error": "simulated: not connected"}
        if m == "nofix":
            t = time.monotonic() - self.world.gnss_t0
            return {"connected": True, "port": "sim", "error": None, "fix_type": 0, "num_sv": min(int(t * 3), 8)}
        carr = 2 if m == "fixed" else 1 if m == "float" else 0
        nsv = 31 if m in ("float", "fixed") else 14
        return {"connected": True, "port": "sim", "error": None, "fix_type": 3, "num_sv": nsv,
                "lat": config.SITE.latitude_deg, "lon": config.SITE.longitude_deg, "h_msl": config.SITE.elevation_m,
                "h_acc_m": 0.6, "clock_offset_s": 0.004, "carr": carr, "heading": self._heading(), "pitch": 0.2,
                "length_cm": 100.2, "heading_valid": carr == 2, "age_s": 0.2}

    def average_baseline(self, seconds, cancel=None):
        hs = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if cancel and cancel():
                return None
            if self.world.gnss_state() == "fixed":
                hs.append(self._heading())
            time.sleep(0.2)
        if len(hs) < max(3, seconds * 0.5):
            return None
        s = sum(math.sin(math.radians(h)) for h in hs)
        c = sum(math.cos(math.radians(h)) for h in hs)
        return {"heading": math.degrees(math.atan2(s, c)) % 360, "pitch": 0.2, "n": len(hs)}


class SimSunSensor(PowerMeter):
    """Fake B206mini power meter: cold-sky floor + a Gaussian Sun beam (FWHM 1.6 deg, +9 dB) at the
    TRUE beam direction (hidden yaw/tilt + boresight offset), from the simulated drive positions."""
    name = "SIMULATED SDR"

    def __init__(self, site=None, floor_db=FLOOR_DBFS, rise_db=SUN_RISE_DB, fwhm_deg=SUN_FWHM_DEG):
        super().__init__()
        self.world = SimWorld.get()
        self.site = site or (lambda: (config.SITE.latitude_deg, config.SITE.longitude_deg))
        self.floor, self.rise, self.fwhm = floor_db, rise_db, fwhm_deg

    def _open(self):
        pass

    def true_power(self, t=None):
        lat, lon = self.site()
        az, el = sun_azel(t or clock.now(), lat, lon)
        f = 10 ** (self.floor / 10)
        if el < 0:
            return f
        off = K.angle_between(self.world.true_vector(), K.azel_to_enu(az, el))
        return f * (1 + (10 ** (self.rise / 10) - 1) * math.exp(-4 * math.log(2) * (off / self.fwhm) ** 2))

    def _measure(self):
        if not self.world.sdr_on:
            raise RuntimeError("simulated SDR unplugged")
        time.sleep(0.05)                                       # the 50 ms block
        lin = self.true_power() * (1 + random.gauss(0, 0.004))
        peak = math.sqrt(lin * math.log(self.cfg.samples)) * (1 + abs(random.gauss(0, 0.05)))
        if self.world.saturate:
            peak = 0.8
        return lin, peak

    def status(self):
        st = super().status()
        if not self.world.sdr_on:
            st.update(available=False, error="simulated SDR unplugged")
        return st
