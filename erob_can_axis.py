"""
erob_can_axis.py - one gimbal axis = one eRob drive + HOME zero + sign + software limits.

Angles used everywhere above this file are "axis degrees":
    axis_deg = sign * wrap(raw_deg - home_deg)        0 = dish vertical (HOME)
Before HOME is set, axis_deg is measured from wherever the axis was at start-up and travel is
limited to +/- UNHOMED_TRAVEL_DEG (raw encoder reads ~236-240 deg, so the old code's first
move_to() could swing ~240 deg into the cables).

Motion uses only the bench-proven primitives: configure (motor off) -> enable -> 0x87 relative + 0x83.
"""
import logging

import config
from zeroerr_protocol import Drive, DriveError, counts_to_deg, deg_to_counts

log = logging.getLogger(__name__)


def wrap180(d: float) -> float:
    return (d + 180.0) % 360.0 - 180.0


class LimitError(Exception):
    pass


class Axis:
    def __init__(self, cfg: config.AxisConfig, drive: Drive, home_counts: int | None = None):
        self.cfg, self.drive, self.name = cfg, drive, cfg.name
        self.home_counts = home_counts
        self._start_counts = None          # zero reference while un-homed
        self.enabled = False
        self.profile_speed = None
        self.angle = None                  # last read, axis degrees
        self.raw_counts = None

    # ---- geometry --------------------------------------------------------------
    @property
    def homed(self) -> bool:
        return self.home_counts is not None

    def _zero(self) -> int:
        return self.home_counts if self.homed else self._start_counts

    def counts_to_axis(self, raw: int) -> float:
        return self.cfg.sign * wrap180(counts_to_deg(raw - self._zero()))

    @property
    def limits(self) -> tuple[float, float]:
        if self.homed:
            return self.cfg.min_deg, self.cfg.max_deg
        return -config.UNHOMED_TRAVEL_DEG, config.UNHOMED_TRAVEL_DEG

    def clamp(self, axis_deg: float) -> float:
        lo, hi = self.limits
        return max(lo, min(hi, axis_deg))

    def set_home_here(self) -> int:
        """Current raw position becomes axis 0 (dish vertical). Returns the counts to store."""
        raw = self.drive.position_counts() % config.COUNTS_PER_REV
        self.home_counts = raw
        self.read()
        return raw

    # ---- CAN ---------------------------------------------------------------------
    def read(self) -> float:
        raw = self.drive.position_counts()
        if self._start_counts is None:
            self._start_counts = raw
        self.raw_counts = raw
        self.angle = self.counts_to_axis(raw)
        return self.angle

    def prepare(self, speed: float | None = None) -> None:
        """Motor OFF + configure (proven sequence). Leaves the motor disabled."""
        speed = speed or self.cfg.track_speed_deg_s
        self.drive.configure(speed, self.cfg.accel_deg_s2)
        self.enabled, self.profile_speed = False, speed

    def enable(self) -> None:
        self.drive.enable()
        self.enabled = True

    def disable(self) -> None:
        try:
            self.drive.disable()
        finally:
            self.enabled = False

    def set_speed(self, speed: float, live_only: bool = False) -> None:
        """Change profile speed. Tries a live write; if the drive refuses it while ON (0x15/0x16),
        falls back to the proven path: disable -> configure -> enable (brake holds meanwhile)."""
        if self.profile_speed is not None and abs(speed - self.profile_speed) < 1e-6:
            return
        try:
            self.drive.set_speed(speed)
        except DriveError as e:
            if e.code not in (0x15, 0x16) or live_only:
                raise
            log.info("%s: speed change refused while ON (0x%X) - re-configuring", self.name, e.code)
            was_on = self.enabled
            self.prepare(speed)
            if was_on:
                self.enable()
        self.profile_speed = speed

    def move_by(self, delta_axis_deg: float) -> bool:
        """Relative move in axis degrees, refusing anything that would leave the limits.
        Returns False if the drive said 'busy' (0x15: previous move still running)."""
        if self.angle is None:
            self.read()
        target = self.angle + delta_axis_deg
        lo, hi = self.limits
        if not lo - 1e-6 <= target <= hi + 1e-6:
            raise LimitError(f"{self.name}: target {target:+.2f} outside {lo:+.1f}..{hi:+.1f}")
        if abs(deg_to_counts(delta_axis_deg)) == 0:
            return True
        try:
            self.drive.move_relative(self.cfg.sign * delta_axis_deg)
            return True
        except DriveError as e:
            if e.code == 0x15:
                return False
            raise

    def stop(self) -> None:
        self.drive.stop()
