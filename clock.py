"""clock.py - one place that says what time it is (UTC). The simulator can shift it (GS_SIM_CLOCK)."""
import os
from datetime import datetime, timedelta, timezone

_offset = timedelta(0)


def set_sim_start(iso: str) -> None:
    """Make 'now' start at this ISO time (e.g. 2026-11-06T08:30:00+08:00) and run in real time."""
    global _offset
    t = datetime.fromisoformat(iso)
    if t.tzinfo is None:
        t = t.replace(tzinfo=timezone.utc)
    _offset = t - datetime.now(timezone.utc)


def now() -> datetime:
    return datetime.now(timezone.utc) + _offset


if os.environ.get("GS_SIM_CLOCK"):
    set_sim_start(os.environ["GS_SIM_CLOCK"])
