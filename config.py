"""Constants for the X-Y gimbal ground station. CAN numbers proven on the bench (2026-10-20, both motors) live here."""
from dataclasses import dataclass

# ---------------------------------------------------------------------------
# CAN bus (ZeroErr CAN-custom protocol, eRunner manual V1.7 ch. 7)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class CanBusConfig:
    interface: str = "remote"        # "remote" (default: Pi runs can_relay.py) | "canutils" | "socketcan" | "waveshare" | "sim"
    channel: str = "can0"            # SocketCAN (Pi CAN HAT)
    serial_port: str | None = None   # Waveshare USB-CAN-A (None = first CH340 found)
    bitrate: int = 1_000_000         # eRob CAN is fixed at 1 Mbps (register 0x44), not 500k
    gap_s: float = 0.02              # pause before every request (drives cut out under heavy polling)
    reply_timeout_s: float = 0.4     # max wait for a reply of the right length (local CAN)
    pi: str | None = None            # remote: "<host>[:7070]" of the Pi running can_relay.py


CAN_BUS = CanBusConfig()

# ---------------------------------------------------------------------------
# Pi CAN relay (can_relay.py on the Pi, RemoteCanLink in the app)
# ---------------------------------------------------------------------------
RELAY_PORT = 7070
RELAY_REPLY_TIMEOUT_S = 0.6          # network round trip + cansend
RELAY_HEARTBEAT_S = 0.5              # app to relay heartbeat
RELAY_WATCHDOG_S = 2.0               # relay sends 0x84 STOP to both nodes if the app is silent this long

COUNTS_PER_REV = 524_288             # output shaft, proven: 90 deg = 0x00020000 (not 131072)
BOOT_WAIT_S = 8.0                    # wait after drive power-on before enabling (errors 0x1F/0x56)

# ---------------------------------------------------------------------------
# HOME = dish vertical, set by hand on the bench (raw encoder counts, register 0x0002).
# Angle shown = wrap(raw - home) * 360 / 524288, wrap to +/-262144 counts.
# X: a previous alignment gave 97775 counts instead (1.84 deg away) - still being re-checked.
# ---------------------------------------------------------------------------
HOME_COUNTS = {"x": 100451, "y": 94947}


# ---------------------------------------------------------------------------
# Axes
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class AxisConfig:
    name: str
    node_id: int
    model: str
    sign: int                        # +1/-1: makes +deg match the kinematics (W/S, A/D check in Test mode)
    min_deg: float                   # software limits, relative to HOME (dish vertical)
    max_deg: float
    slew_speed_deg_s: float = 5.0    # proven bench speed (unloaded; not used in the test stage)
    track_speed_deg_s: float = 2.0   # profile speed for tracking steps (sats peak ~1.05 deg/s)
    accel_deg_s2: float = 2.0        # top-heavy load, so 2 deg/s^2 (0x00000419), not 10
    current_limit_a: float = 15.0    # software stop above this |current| (60 % of drive continuous)


# X shaft runs East-West; it tilts the dish North/South.   +X = toward SOUTH (see kinematics.py)
# Y shaft rides on the X frame; it tilts the dish East/West. +Y = toward EAST
# Drive limits read from the drives: X continuous 25 A (peak 30 A / 4 s), Y continuous 18 A (peak 27 A / 4 s).
# Soft limits +/-80 deg from HOME on both axes (enforced in the app; can_relay.py knows no angles).
X_AXIS = AxisConfig("x", node_id=1, model="eRob110H100I", sign=+1, min_deg=-80.0, max_deg=80.0, current_limit_a=15.0)
Y_AXIS = AxisConfig("y", node_id=2, model="eRob90I-100", sign=+1, min_deg=-80.0, max_deg=80.0, current_limit_a=11.0)
AXES = {"x": X_AXIS, "y": Y_AXIS}

# ---------------------------------------------------------------------------
# Offset dish: the RF beam is not the dish face normal. At HOME (face level) the beam
# leans `deg` toward `toward`, the way a positive/negative rotation of that axis would tilt it:
#   "+X" = toward South (like +X), "-X" = North, "+Y" = toward East (like +Y), "-Y" = West.
# boresight_body = R(offset) . [0,0,1] is used everywhere (kinematics.py). deg = 0 (wedge fitted) gives the old maths.
# TODO (user): set "toward" after checking the dish, then set "confirmed": True (the UI warns until then).
# ---------------------------------------------------------------------------
BORESIGHT_OFFSET = {"deg": 22.0, "toward": "+X", "confirmed": True}   # confirmed 2026-11-11: manual Sun point X+19.3 Y-34.7 fits GNSS yaw within 1.7 deg

# 3D view only (the web page picture, not the motors): flip an axis if the model turns the wrong way
# against the real dish. Motor direction for a positive encoder value is still to be confirmed.
VIEWER_FLIP = {"x": False, "y": False}

# Before HOME is set the encoder zero is unknown (raw reads ~236-240 deg on the bench), so jogging
# is limited to +/- this many degrees around wherever the axis was when the app started.
UNHOMED_TRAVEL_DEG = 30.0

# Disabled in the test stage: old hold-to-move jog
# JOG_SPEEDS_DEG_S = (0.2, 0.5, 1.0)   # Test mode "very slow" choices
# JOG_DEADMAN_S = 0.6                  # no key heartbeat for this long -> stop

# ---------------------------------------------------------------------------
# Test stage: manual latching WASD (gimbal.Gimbal)
# ---------------------------------------------------------------------------
# Accel/decel register value used on the bench, written raw. 0x419 = 1049 counts/s^2, which is
# 0.72 deg/s^2 if the unit is counts/s^2 like the 0x8A speed (1 deg/s = 0x5B0 = 1456). It was described as
# "2 deg/s^2" (that would be 0xB61). Kept at the proven value until the unit is confirmed.
MANUAL_ACCEL_COUNTS = 0x00000419
MANUAL_SPEEDS_DEG_S = (0.5, 1.0, 1.5)
MANUAL_DEFAULT_SPEED_DEG_S = 1.0     # 0x000005B0
WAKE_STEP_DEG = 0.5                  # Start: +0.5 deg then back (a 0.05 deg move made Y drop out)
MIN_MOVE_DEG = 0.5                   # no latched move shorter than this
SETUP_TO_ENABLE_S = 1.0              # at least 1 s between motor-off/setup and the next enable (Y dropped out at 0.3 s)
POLL_PERIOD_S = 1.0                  # per axis: position + current + state once a second (max 6 reads/s total)
SLOW_POLL_PERIOD_S = 10.0            # temperature / bus voltage
STOP_CHECK_S = 0.6                   # read 0x010C this long after 0x84 STOP (plus the decel time)
START_RETRY_S = 1.0                  # retry START refused with 0x15 (still decelerating) for this long
PAGE_DEADMAN_S = 2.0                 # no heartbeat from the web page for this long: STOP both
WAKE_TIMEOUT_S = 8.0                 # each wake move must reach state 4 within this
HOME_TIMEOUT_S = 240.0               # Home (up to 160 deg at 0.5 deg/s) must finish within this

# goto / track (Home, Sun steps). One relative step per axis at a time (START is refused while moving),
# aimed at where the target will be when the step ends. Tracking steps below MIN_MOVE_DEG are fine
# once the motor is holding (bench rule 4).
TRACK_STEP_PERIOD_S = 0.5            # at most one step per axis every 0.5 s
TRACK_DEADBAND_DEG = 0.02            # no step for errors below this
TRACK_SETTLE_S = 0.15                # after a step ends: read position, then the next step
GOTO_DONE_DEG = 0.02                 # goto finished when both axes are this close and still
TRACK_POLL_PERIOD_S = 2.0            # while tracking: current + state every 2 s per axis (positions come
                                     # from the step reads), about 6 reads/s total at most

# Control loop
TICK_S = 0.25                        # controller thread period
TRACK_PERIOD_S = 0.5                 # one relative step per axis every 0.5 s while tracking
TRACK_DEADBAND_DEG = 0.02
TRACK_ADAPTIVE_SPEED = True          # write 0x8A while on each step for smooth motion; falls back
                                     # to a fixed 2 deg/s step if the drive refuses (bench test T5)
STATUS_EVERY_TICKS = 8               # read state/error registers every 2 s (keeps CAN quiet)


# ---------------------------------------------------------------------------
# Site (overwritten by GNSS fix when available)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SiteConfig:
    latitude_deg: float = 1.2966
    longitude_deg: float = 103.7764
    elevation_m: float = 15.0
    name: str = "NUS terrace (default - GNSS overrides)"


SITE = SiteConfig()


# ---------------------------------------------------------------------------
# GNSS: ArduSimple simpleRTK2B heading kit (2x ZED-F9P, moving base)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class GnssConfig:
    rover_port: str | None = None    # heading board, POWER+GPS USB -> /dev/ttyACM* (None = auto)
    # Baseline (base antenna to rover antenna) direction in the dish frame at HOME:
    # (E, N, U) = (1, 0, 0) means the bar is parallel to the X shaft, rover antenna on the east end.
    bar_body: tuple = (0.0, 1.0, 0.0)    # rover on the -X side (see GNSS_BAR_ROVER_SIDE); was (1,0,0)
    pose_settle_s: float = 3.0
    pose_average_s: float = 12.0
    heading_sigma_deg: float = 0.5   # 1 m baseline


GNSS = GnssConfig()

# Sun method 1: the heading bar sits on the base (does not move with X/Y). GNSS_BAR_ROVER_SIDE is the mount
# direction the baseline (base antenna to rover antenna, as gnss_check.py prints it) points along, at yaw 0:
#   "+Y" = East (90)   "-Y" = West (270)   "+X" = South (180)   "-X" = North (0)
# psi = heading - that angle.
# A number is also accepted: the bar angle in degrees in that same frame (for a bar not parallel to an axis).
# 2026-11-11 field check: GNSS heading 251 deg, D (+Y) tilts the dish toward ~330 deg (iPhone compass),
# so psi ~ -120 deg and bar ~ 11 deg, i.e. "-X" (bar along the X shaft, rover antenna on the -X side).
GNSS_BAR_ROVER_SIDE = "-X"           # "+Y" | "-Y" | "+X" | "-X" | degrees   (was "+Y")
_BAR_DEG = {"+Y": 90.0, "-Y": 270.0, "+X": 180.0, "-X": 0.0}


def gnss_bar_deg() -> float:
    """Direction of the GNSS baseline in the mount frame (deg, 0 = -X/North-at-yaw-0, 90 = +Y/East): psi = heading - this."""
    v = GNSS_BAR_ROVER_SIDE
    return float(v) if isinstance(v, (int, float)) else _BAR_DEG[v]


GNSS_HEADING_AVERAGE_S = 60.0        # average NAV-RELPOSNED heading (carrSoln = 2 only) this long


# ---------------------------------------------------------------------------
# SDR power meter for sun scans (Ettus B206mini, receive only)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SdrConfig:
    args: str = "type=b200"
    if_freq_hz: float = 1.20e9       # LNB LO 12.8 GHz -> sky 14.0 GHz
    sky_freq_hz: float = 14.0e9      # used for lambda in G/T
    rate_hz: float = 4e6             # 4 Msps
    gain_db: float = 20.0            # start value; chain gives ~+40 dB noise rise; keep headroom
    antenna: str = "RX2"             # RX2 only, TX/RX port stays unconnected. Input max -15 dBm.
    samples: int = 200_000           # 50 ms at 4 Msps = one reading (mean |x|^2, kept linear)
    reading_period_s: float = 0.1    # ~10 readings/s
    saturation_fs: float = 0.7       # any |sample| above this means "reduce gain", result invalid


SDR = SdrConfig()


# ---------------------------------------------------------------------------
# Sun scan
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SunScanConfig:
    min_elevation_deg: float = 15.0
    max_elevation_deg: float = 80.0  # near zenith yaw is unobservable; low sun is best
    coarse_half_width_deg: float = 6.0
    coarse_step_deg: float = 0.5
    fine_half_width_deg: float = 2.5
    fine_step_deg: float = 0.2
    dwell_s: float = 0.35            # settle after each step before measuring
    min_rise_db: float = 1.0         # below this, "no sun found"
    max_on_sun_s: float = 90.0       # LNB heat: don't stay near the sun longer than this per scan
    beamwidth_deg: float = 1.7       # first guess (0.9 m dish @ 14 GHz); the fit measures it


SUN_SCAN = SunScanConfig()          # only used by calibration.py (disabled in the test stage)


# ---------------------------------------------------------------------------
# Sun finding + G/T (sun_job.py). Times in seconds, angles in degrees.
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SunJobConfig:
    min_el: float = 15.0             # refuse below (refraction, trees)
    max_el: float = 80.0             # refuse above (near zenith yaw can't be measured)
    # method 2 - elevation-ring scan (yaw unknown)
    ring_half_width: float = 40.0    # predicted az +/- this ...
    ring_wide_half_width: float = 80.0   # ... then widened to this
    ring_rate: float = 1.0           # az deg/s along the ring ...
    ring_max_axis_rate: float = 0.4  # ... but slower where X or Y would need more than this (deg/s): with accel
                                     # 0x419 the one-step-at-a-time tracker can't follow faster and would leave the ring
    ring_el_steps: tuple = (0.0, +1.2, -1.2)   # repeat at el +/- 1.2 deg (covers base tilt)
    gnss_search_half: float = 12.0   # GNSS north can be 5-10 deg off: search the Sun's power this far either side of the prediction
    ring_min_rise_db: float = 0.0    # no limit: the strongest point of the ring counts (was 3.0, then 0.15)
    ring_ref_samples: int = 5        # start-of-cycle reference = mean of this many first samples (1 = the very first value)
    # common peak-up
    cold_offset: float = 10.0        # cold sky: this far off the Sun (angle on the sky), same elevation
    cold_avg_s: float = 10.0
    cut_half: float = 3.0            # cross-scan +/- 3 deg in X, then in Y
    cut_step: float = 0.2
    fine_half: float = 1.5           # one refinement pass at +/- 1.5 deg
    dwell_s: float = 0.4             # power averaged this long at each settled step
    min_rise_db: float = 0.0         # no limit in the peak-up (was 1.0, then 0.15)
    # LNB heat rule: the dish focuses sunlight on the LNB
    heat_radius: float = 1.5         # "on the Sun" = beam within this of the Sun
    heat_limit_s: float = 120.0      # continuous time on the Sun, then move 10 deg off automatically
    heat_cooldown_s: float = 30.0    # G/T: if the budget is nearly used, cool down this long off the Sun first
    abort_near: float = 3.0          # Abort/STOP within this of the Sun -> move 10 deg off
    # G/T (Y-factor)
    gt_avg_s: float = 10.0           # P_sun and P_cold averages
    track_period_s: float = 0.5      # TRACK: gimbal track mode, 0.5 s steps


SUN_JOB = SunJobConfig()


# ---------------------------------------------------------------------------
# Test mode ("Run test": link, motors, SDR, GNSS, then point at true north)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class TestConfig:
    motor_move_deg: float = 5.0      # each axis goes HOME, +d, -d, HOME (small: the dish is top-heavy)
    motor_tol_deg: float = 0.15      # "reached" means within this of the target when the move ends
    move_timeout_s: float = 90.0
    sdr_seconds: float = 6.0         # how long the SDR readings are collected
    sdr_min_rate_hz: float = 4.0     # warn below this many readings/s (nominal ~10/s)
    gnss_timeout_s: float = 300.0    # wait this long for RTK FIXED before giving up
    gnss_heading_avg_s: float = 20.0 # average the RTK FIXED heading this long (Sun method 1 uses 60 s)
    north_az_deg: float = 0.0        # true north ...
    north_el_deg: float = 45.0       # ... at 45 deg elevation (the RF beam, not the dish face)
    sun_clear_deg: float = 15.0      # warn if the Sun is within this of the north target


TEST = TestConfig()


# ---------------------------------------------------------------------------
# Demodulator (Galassia-5 receiver: patched gr-dvbs2rx + GSSE/GSUFTP image decoder, run as a subprocess)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class DemodConfig:
    # How it is run: ./demod_env.sh python3 demod/unified/rx_cli.py ... (receive only, only rx_cli.py is ever started)
    env_script: str = "demod_env.sh"                 # relative to this folder
    rx_cli: str = "demod/unified/rx_cli.py"          # relative to this folder
    out_root: str = "Received"                       # decoded images / files: <out_root>/run_<UTC>/ (relative to this folder)
    rec_dir: str = "Received/recordings"             # raw IQ (sc16) recordings: 4 bytes x sample rate (20 MBd = 40 MS/s = 9.6 GB/min, 2 MBd = 1.9 GB/min)
    preset: str = "linkx2"                           # ops20 = 20 MBd, linkx2 = 2 MBd, field1 = 1 MBd (was ops20; linkx2 decoded images 2026-11-11)
    modcod: str = "6"                                # Link X MODCOD number (or e.g. "QPSK2/3"); "" = preset. 6 = what decoded on 2026-11-11
    # Radio (B206mini, RX2 only). 1195e6 is the validated IF of the Galassia-5 receiver.
    # Check: its default downlink RF is 13.95 GHz, i.e. an LNB LO of 12.755 GHz. config.SDR assumes LO 12.8 GHz and IF 1.2 GHz.
    centre_freq_hz: float = 1150e6                   # was 1195e6; 1150e6 decoded images in the GNU Radio GUI 2026-11-11 (LNB LO 12.8 GHz)
    rf_freq_hz: float = 13.95e9
    rx_gain_db: float = 30.0                         # the power meter uses SDR.gain_db; the demod wants more or less gain: tune on a pass
    antenna: str = "RX2"
    stack: str = "auto"                              # GSSE stack: auto | rev1.0 | rev1.2
    stats_every_s: float = 0.5                     # status line period: the power trace on the page updates about this often
    stop_grace_s: float = 25.0                       # SIGINT, flush, SIGTERM, kill


DEMOD = DemodConfig()
LOG_DIR = "logs"                     # sun_<time>.json/.csv and power_<date>.csv (relative to where app.py runs)


# ---------------------------------------------------------------------------
# Tracking / scheduling
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class TrackingConfig:
    min_pass_elevation_deg: float = 10.0
    lookahead_hours: float = 24.0
    preslew_s: float = 120.0
    table_step_s: float = 0.1        # precomputed pass table resolution
    tle_refresh_s: float = 2 * 3600  # CelesTrak policy: at most once per 2 h
    tle_max_epoch_age_days: float = 5.0   # refuse to track on elements older than this


TRACKING = TrackingConfig()

# Targets by NORAD catalog number (names are ambiguous, "ISS" once matched UME, NORAD 8709).
TARGETS = {
    56309: "LUMELITE-4",
    52935: "DS-EO",
    56310: "TELEOS-2",
    40057: "VELOX-I",
    # Galassia: add its catalog number here after launch.
}

# Files
CALIBRATION_PATH = "calibration.json"
TLE_CACHE_PATH = "omm_cache.json"
