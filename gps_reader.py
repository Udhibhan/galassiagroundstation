"""
gps_reader.py - ZED-F9P heading-kit reader (UBX, not NMEA).

Replaces the v2 NMEA reader (pynmea2 on /dev/ttyAMA0 at 9600 baud). That one could never see the
ArduSimple kit: the heading rover talks UBX over native USB (/dev/ttyACM*), and NMEA has no
NAV-RELPOSNED (baseline heading/pitch) or reliable GPS time.

Reads from the rover board (simpleRTK2B bottom, POWER+GPS USB):
  NAV-PVT        position, fix, satellites, clock offset of this computer vs GNSS
  NAV-RELPOSNED  baseline heading (base antenna to rover antenna) and pitch
Enables those two messages in RAM only, nothing is written to flash. Receive only otherwise.

Config was done once on the bench with gnss_test.py config-base / config-rover.
Needs: pip install pyubx2==1.4.0 pyserial
"""
import logging
import math
import threading
import time
from datetime import datetime, timezone

import config

log = logging.getLogger(__name__)
UBLOX_VID = 0x1546


def find_rover_port():
    from serial.tools import list_ports
    for p in sorted(list_ports.comports(), key=lambda p: p.device):
        if p.vid == UBLOX_VID and ("ACM" in p.device or p.device.upper().startswith("COM")):   # Linux / Windows
            return p.device
    return None


class GnssReader:
    name = "ZED-F9P heading kit"

    def __init__(self, port: str | None = None):
        self.port = port or config.GNSS.rover_port
        self._pvt, self._rel, self._rel_t, self._pvt_t = None, None, 0.0, 0.0
        self._offsets = []
        self._samples = []                # recent valid FIXED samples (t, heading, pitch)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self.error = None

    # thread
    def start(self):
        self._thread = threading.Thread(target=self._run, name="gnss", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _run(self):
        while not self._stop.is_set():
            try:
                self._session()
            except Exception as e:                       # unplugged, permission, ModemManager...
                self.error = str(e)
                log.warning("GNSS: %s (retrying in 5 s)", e)
                self._stop.wait(5.0)

    def _session(self):
        import serial
        from pyubx2 import UBX_PROTOCOL, UBXMessage, UBXReader
        port = self.port or find_rover_port()
        if not port:
            raise RuntimeError("no u-blox /dev/ttyACM* found (rover = POWER+GPS USB)")
        with serial.Serial(port, 38400, timeout=0.5) as ser:
            ser.write(UBXMessage.config_set(1, 0, [("CFG_MSGOUT_UBX_NAV_PVT_USB", 1),
                                                   ("CFG_MSGOUT_UBX_NAV_RELPOSNED_USB", 1)]).serialize())
            rd = UBXReader(ser, protfilter=UBX_PROTOCOL, quitonerror=0)
            self.port, self.error = port, None
            log.info("GNSS: reading %s", port)
            while not self._stop.is_set():
                try:
                    _, m = rd.read()
                except Exception:
                    continue
                if m is None:
                    continue
                now = time.monotonic()
                if m.identity == "NAV-PVT":
                    with self._lock:
                        self._pvt, self._pvt_t = m, now
                        if m.validDate and m.validTime and m.fullyResolved:
                            t = datetime(m.year, m.month, m.day, m.hour, m.min, m.second,
                                         tzinfo=timezone.utc).timestamp() + m.nano * 1e-9
                            self._offsets = (self._offsets + [time.time() - t])[-30:]
                elif m.identity == "NAV-RELPOSNED":
                    horiz = math.hypot(m.relPosN, m.relPosE)
                    pitch = math.degrees(math.atan2(-m.relPosD, horiz)) if horiz > 0 else 0.0
                    with self._lock:
                        self._rel, self._rel_t = (m, pitch), now
                        if m.carrSoln == 2 and m.relPosHeadingValid:
                            self._samples = [s for s in self._samples if now - s[0] < 120] + [(now, m.relPosHeading, pitch)]

    # API
    def status(self) -> dict:
        with self._lock:
            pvt, rel, now = self._pvt, self._rel, time.monotonic()
            offs = sorted(self._offsets)
            st = {"connected": pvt is not None and now - self._pvt_t < 3, "port": self.port, "error": self.error}
            if pvt is not None:
                st.update(fix_type=pvt.fixType, num_sv=pvt.numSV, lat=pvt.lat, lon=pvt.lon,
                          h_msl=pvt.hMSL / 1000.0, h_acc_m=pvt.hAcc / 1000.0,
                          clock_offset_s=offs[len(offs) // 2] if offs else None)
            if rel is not None:
                m, pitch = rel
                st.update(carr=m.carrSoln, heading=m.relPosHeading, pitch=pitch, length_cm=m.relPosLength,
                          heading_valid=bool(m.relPosHeadingValid and m.carrSoln == 2), age_s=now - self._rel_t)
        return st

    def average_baseline(self, seconds: float, cancel=None) -> dict | None:
        """Mean heading/pitch over the next `seconds` of RTK-FIXED epochs, None if too few."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            if cancel and cancel():
                return None
            time.sleep(0.2)
        with self._lock:
            s = [x for x in self._samples if x[0] >= t0]
        if len(s) < max(3, seconds * 0.5):
            return None
        sn = sum(math.sin(math.radians(h)) for _, h, _ in s)
        cs = sum(math.cos(math.radians(h)) for _, h, _ in s)
        return {"heading": math.degrees(math.atan2(sn, cs)) % 360,
                "pitch": sum(p for _, _, p in s) / len(s), "n": len(s)}
