#!/usr/bin/env python3
"""
app.py - the ground station web UI. TEST STAGE: Manual (latching WASD + Home) and Sun / G/T.

SPLIT SETUP: runs on the UBUNTU LAPTOP (radioconda, UHD); the Raspberry Pi only relays CAN (can_relay.py).
    python3 app.py --pi <pi-ip>                 # DEFAULT: remote CAN via the Pi relay (port 7070)
    python3 app.py --sim                        # everything simulated (drives, SDR, GNSS), no hardware
    python3 app.py --sim --sim-clock 2026-11-09T09:30:00+08:00
    python3 app.py --iface canutils             # app on the Pi itself (candump/cansend), or socketcan / waveshare
    python3 app.py --pi <pi-ip> --gnss none     # without the heading kit (Sun method 2 only)
Then open http://localhost:8080 on the laptop.

Standard library HTTP + Server-Sent Events. numpy needed (kinematics); uhd / pyubx2 are optional.
"""
import argparse
import csv
import json
import logging
import os
import sys
import threading
import time
from urllib.parse import parse_qs, urlparse
from collections import deque
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import clock
import config
import kinematics as K
from gimbal import Gimbal
from sun_job import SunSession
from test_job import TestSession

from demod import DemodSession
from rehearsal import Rehearsal

# DISABLED (test stage): calibration (GNSS poses), auto satellite tracking, orbits
# from calibration import Calibration, GnssPoseJob, SunScanJob
# from pass_scheduler import AutoTracker

log = logging.getLogger("app")
UI_DIR = Path(__file__).resolve().parent / "ui"
UI = UI_DIR / "index.html"
STATIC_TYPES = {".js": "text/javascript", ".mjs": "text/javascript", ".glb": "model/gltf-binary", ".json": "application/json",
                ".css": "text/css", ".woff2": "font/woff2", ".svg": "image/svg+xml", ".png": "image/png", ".wasm": "application/wasm"}


def _finite(v):
    return v == v and abs(v) != float("inf")


class _NoGnss:
    name = "no GNSS"

    def start(self):
        pass

    def stop(self):
        pass

    def status(self):
        return {"connected": False, "port": None, "error": "disabled (--gnss none)"}

    def average_baseline(self, seconds, cancel=None):
        return None


class App:
    def __init__(self, sim: bool, iface: str, pi: str | None = None, gnss: str = "auto", sdr: bool = True):
        self.sim = sim
        self.events = deque(maxlen=60)
        self._plog, self._plog_day, self._plog_flush = None, None, 0.0     # before the SDR thread can call _on_reading
        if sim:
            from sim_hw import SimCanLink, SimGnss, SimSunSensor, SimWorld
            link = SimCanLink()
            self.world = SimWorld.get()
            self.gnss = SimGnss()
            self.meter = SimSunSensor(site=lambda: self.site()[:2])
        else:
            from zeroerr_protocol import open_link
            link = open_link(config.CanBusConfig(interface=iface, pi=pi))
            self.world = None
            if gnss == "none":
                self.gnss = _NoGnss()
            else:
                from gps_reader import GnssReader
                self.gnss = GnssReader(None if gnss == "auto" else gnss)
            self.meter = None
            if sdr:
                from sdr_power import SdrPower
                self.meter = SdrPower()
        self.link = link
        self.g = Gimbal(link, home=config.HOME_COUNTS)
        self.gnss.start()
        if self.meter is not None:
            self.meter.on_reading = self._on_reading
            self.meter.start()                                   # opens the USRP in its own thread
        self.sun = SunSession(self.g, lambda: self.meter, self.gnss, self.site)
        self.test = TestSession(self.g, lambda: self.meter, self.gnss, self.site, self.relay_status)
        self._sdr_enabled = sdr
        try:                                                     # skyfield is loaded only here (not at import time)
            from sat_job import SatSession
            from types import SimpleNamespace
            rx = SimpleNamespace(start=self.receive_start, stop=self.receive_stop, busy=lambda: self.demod.busy())
            self.sat = SatSession(self.g, self.gnss, self.site, self.mount_yaw, sim, rx=rx)
        except ImportError as e:                                 # skyfield missing: everything else still works
            log.warning("satellite tracking disabled: %s (pip install skyfield)", e)
            self.sat = None
        self._net = {"ok": None, "t": 0.0}
        threading.Thread(target=self._net_loop, name="net-check", daemon=True).start()
        self.demod = DemodSession(hold_sdr=self._hold_sdr, release_sdr=self._release_sdr)
        self.rehearsal = Rehearsal(self.demod)
        self._grid_key, self._grid = None, None
        self._plog, self._plog_day, self._plog_flush = None, None, 0.0
        self.g.start()

    # ------------------------------------------------------------------ helpers
    def site(self):
        st = self.gnss.status()
        if st.get("connected") and (st.get("fix_type") or 0) >= 3 and st.get("lat") is not None:
            return st["lat"], st["lon"], "GNSS"
        return config.SITE.latitude_deg, config.SITE.longitude_deg, "config default"

    # ------------------------------------------------------------------ SDR hand-over (one program opens the B206mini)
    def _new_meter(self):
        if self.sim:
            from sim_hw import SimSunSensor
            return SimSunSensor(site=lambda: self.site()[:2])
        from sdr_power import SdrPower
        return SdrPower()

    def _hold_sdr(self):
        """The demodulator is about to open the B206mini: stop the power meter and let go of the USB device."""
        if self.test.busy() or self.sun.busy():
            raise RuntimeError("The Test or a Sun step is using the SDR: stop it first")
        m, self.meter = self.meter, None
        if m is not None:
            m.close()
            t = getattr(m, "_thread", None)
            if t is not None:
                t.join(5.0)
            del m
            import gc
            gc.collect()
            time.sleep(1.0)
        self.event("SDR handed to the demodulator (power meter paused)")

    def _release_sdr(self):
        """The demodulator ended: take the SDR back for the power meter."""
        if self._sdr_enabled and self.meter is None:
            self.meter = self._new_meter()
            self.meter.on_reading = self._on_reading
            self.meter.start()
            self.event("SDR power meter resumed")

    # ------------------------------------------------------------------ receiving: the B206mini goes to the demodulator
    def receive_start(self, o=None):
        """Start taking in data: live demod (or record + duty demod at 20 MBd), decoded files -> Received/run_<UTC>/,
        raw IQ -> Received/recordings/. The power meter is paused meanwhile (one program opens the radio)."""
        o = dict(o or {})
        preset = o.get("preset") or config.DEMOD.preset
        if preset not in ("ops20", "linkx2", "field1"):
            raise ValueError("preset must be ops20 / linkx2 / field1")
        mode = o.get("mode") or ("record" if preset == "ops20" else "live")      # 20 MBd can't be demodulated live on this laptop
        opts = {"mode": mode, "preset": preset, "record": True if mode == "record" else bool(o.get("record", True))}
        if o.get("tle"):
            opts["tle"] = o["tle"]
        if self.sim:                                                             # the real radio is never opened with --sim
            opts["sim_input"] = o.get("sim_input") or os.environ.get("GS_SIM_IQ")
            if not opts["sim_input"]:                                            # no recording given: make a test signal and receive that
                self.event("Simulator: making a test signal and receiving it")
                return self.rehearsal.start("field1")
        r = self.demod.start(opts)
        self.event(f"Receiving started: {preset}, {mode}" + (", saving raw IQ" if opts["record"] else ""))
        return r

    def receive_stop(self, why="Stop"):
        return self.demod.stop(why)

    def cal(self):
        """Mount calibration for az/el: the Sun job's yaw if it has one, else the Test's / Satellite's GNSS-north yaw, else None."""
        if self.sun.cal:
            return {"cal": self.sun.cal, "offs": self.sun.offs, "source": "Sun"}
        n = self.test.north
        if n:
            return {"cal": {"yaw": n["yaw"]}, "offs": (0.0, 0.0), "source": "Test (GNSS north)"}
        fy = getattr(self.sat, "found_yaw", None) if self.sat else None
        if fy is not None:
            return {"cal": {"yaw": fy}, "offs": (0.0, 0.0), "source": "Satellite (GNSS north)"}
        try:                                                    # live GNSS heading (RTK FIXED) if nothing else has found north yet
            st = self.gnss.status() if self.gnss else {}
            if st.get("heading_valid") and st.get("heading") is not None:
                yaw = ((st["heading"] - config.gnss_bar_deg() + 180) % 360) - 180
                return {"cal": {"yaw": yaw}, "offs": (0.0, 0.0), "source": "GNSS heading (live)"}
        except Exception:
            pass
        return None

    def mount_yaw(self):
        c = self.cal()
        return None if c is None else c["cal"]["yaw"]

    # ------------------------------------------------------------------ what is plugged in
    def _net_loop(self):
        import socket
        while True:
            try:
                socket.create_connection(("celestrak.org", 443), timeout=3).close()
                self._net["ok"] = True
            except OSError:
                self._net["ok"] = False
            time.sleep(60)

    def components(self):
        """What is connected right now (each one decides which tests / modes are possible)."""
        rel = self.relay_status()
        snap = self.g.snapshot()
        drives = [k for k, a in snap["axes"].items() if a["found"]]
        gimbal_ok = bool(rel.get("connected")) and len(drives) == 2
        gn = self.gnss.status()
        sdr = self.meter.status() if self.meter else None
        sdr_ok = bool(sdr and sdr.get("available")) or self.demod.uses_sdr()
        if gimbal_ok:
            gd = "X and Y drives answer" + (f", bus {snap['bus_v']:.0f} V" if snap.get("bus_v") else "")
        elif not rel.get("connected"):
            gd = "gimbal link: " + (rel.get("error") or "not connected")
        else:
            gd = "link up but drive(s) not answering: " + (", ".join(k.upper() for k in ("x", "y") if k not in drives) or "?")
        return {
            "gimbal": {"found": gimbal_ok, "detail": gd, "tests": ["link", "motors", "north"]},
            "gnss": {"found": bool(gn.get("connected")), "tests": ["gnss", "north"],
                     "detail": (f"{gn.get('port')}" if gn.get("connected") else (gn.get("error") or "heading kit not found"))},
            "sdr": {"found": sdr_ok, "tests": ["sdr"],
                    "detail": "in use by the receiver" if self.demod.uses_sdr() else (((sdr or {}).get("name") or "B206mini") if sdr_ok else ((sdr or {}).get("error") or "SDR not found"))},
            "internet": {"found": bool(self._net["ok"]), "tests": [],
                         "detail": "CelesTrak reachable" if self._net["ok"] else "CelesTrak unreachable (saved orbital elements are used)"},
        }

    def north_estimate(self):
        """Axis angles for true north / 45 deg with mount yaw 0 (a rough idea for the confirm dialog)."""
        if not hasattr(self, "_north_est"):
            try:
                x, y = K.azel_to_axes(config.TEST.north_az_deg, config.TEST.north_el_deg, {"yaw": 0.0})
                self._north_est = {"x": x, "y": y, "az": config.TEST.north_az_deg, "el": config.TEST.north_el_deg}
            except ValueError:
                self._north_est = None
        return self._north_est

    def event(self, msg, level="info"):
        self.events.append({"t": clock.now().isoformat(timespec="seconds"), "level": level, "msg": msg})
        (log.info if level == "info" else log.warning)(msg)

    def _on_reading(self, r):
        """Every SDR reading: pose at that moment -> the Sun job + logs/power_<date>.csv."""
        pose = self.g.pose_at(r["tm"])
        self.sun.on_reading(r, pose)
        try:
            self._log_power(r, pose)
        except OSError:
            pass

    def _log_power(self, r, pose):
        t = datetime.fromtimestamp(r["t"], timezone.utc)
        day = t.strftime("%Y%m%d")
        if self._plog_day != day:
            if self._plog:
                self._plog[0].close()
            os.makedirs(config.LOG_DIR, exist_ok=True)
            path = os.path.join(config.LOG_DIR, f"power_{day}.csv")
            new = not os.path.exists(path)
            f = open(path, "a", newline="")
            w = csv.writer(f)
            if new:
                w.writerow(["time_utc", "x_deg", "y_deg", "az", "el", "cal", "p_lin", "p_dbfs", "peak_fs", "saturated", "gain_db"])
            self._plog, self._plog_day = (f, w), day
        f, w = self._plog
        cal = self.sun.cal
        if pose:
            dx, dy = self.sun.offs
            az, el = K.axes_to_azel(pose[0] - dx, pose[1] - dy, cal or {"yaw": 0.0})
            row = [f"{pose[0]:.4f}", f"{pose[1]:.4f}", f"{az:.3f}", f"{el:.3f}", "sun" if cal else "yaw0"]
        else:
            row = ["", "", "", "", ""]
        w.writerow([t.isoformat(timespec="milliseconds")] + row +
                   [f"{r['lin']:.6e}", f"{r['db']:.3f}", f"{r['peak']:.4f}", int(r["sat"]), self.meter.gain_db if self.meter else ""])
        if time.monotonic() - self._plog_flush > 2:
            f.flush()
            self._plog_flush = time.monotonic()

    def relay_status(self):
        if self.sim:
            return {"kind": "sim", "connected": True, "name": self.link.name, "error": None}
        if hasattr(self.link, "connected"):
            errs = list(getattr(self.link, "relay_errors", []))
            return {"kind": "remote", "connected": self.link.connected, "name": self.link.name,
                    "error": self.link.last_error, "relay_err": errs[-1]["msg"] if errs else None}
        return {"kind": "local", "connected": True, "name": self.link.name, "error": None}

    def gnss_status(self):
        st = self.gnss.status()
        if not st.get("connected"):
            chip = "not connected"
        elif st.get("carr") == 2 and st.get("heading_valid"):
            chip = "RTK FIXED"
        elif (st.get("fix_type") or 0) >= 3:
            chip = "RTK FLOAT" if st.get("carr") == 1 else "no RTK fix"
        else:
            chip = "no fix"
        return {"chip": chip, **{k: st.get(k) for k in ("connected", "port", "fix_type", "num_sv", "carr", "heading",
                                                         "heading_valid", "lat", "lon", "error")}}

    def grid(self):
        c = self.cal()
        key = round((c["cal"] if c else {"yaw": 0.0})["yaw"], 1)
        if key != self._grid_key:
            self._grid_key, self._grid = key, K.reach_grid(c["cal"] if c else {"yaw": 0.0})
        return self._grid

    # ------------------------------------------------------------------ state for the UI
    def state(self) -> dict:
        evs = sorted(list(self.events) + list(self.g.events), key=lambda e: e["t"])[-14:]
        sdr = self.meter.status() if self.meter else {
            "available": False, "error": "in use by the demodulator" if self.demod.uses_sdr() else "disabled (--no-sdr)"}
        pointing = None
        x, y = self.g.angles()
        c = self.cal()
        if x is not None and y is not None:
            dx, dy = c["offs"] if c else (0.0, 0.0)
            az, el = K.axes_to_azel(x - dx, y - dy, c["cal"] if c else {"yaw": 0.0})
            pointing = {"az": az, "el": el, "cal": c is not None, "target": None}
            tgt = self.g.act_target()
            if tgt is not None:
                try:
                    taz, tel = K.axes_to_azel(tgt[0] - dx, tgt[1] - dy, c["cal"] if c else {"yaw": 0.0})
                    pointing["target"] = {"az": taz, "el": tel, "x": tgt[0], "y": tgt[1]}
                except Exception:
                    pass
        return {"now": clock.now().isoformat(), "sim": self.sim, "gimbal": self.g.snapshot(), "events": evs,
                "pointing": pointing, "mount": ({"yaw": c["cal"]["yaw"], "source": c["source"]} if c else None),
                "test": dict(self.test.status(), estimate=self.north_estimate()),
                "components": self.components(),
                "sat": (self.sat.status() if self.sat else {"state": "unavailable", "msg": "skyfield is not installed", "busy": False,
                                                              "targets": [], "passes": [], "log": []}),
                "demod": self.demod.status(), "rehearsal": self.rehearsal.status(),
                "offset": config.BORESIGHT_OFFSET, "view": {"flip": config.VIEWER_FLIP},
                "relay": self.relay_status(), "sdr": sdr, "gnss": self.gnss_status(), "sun": self.sun.status(),
                "reach": self.grid(), "limits": [config.AXES["x"].min_deg, config.AXES["x"].max_deg]}

    # ------------------------------------------------------------------ commands
    def command(self, c: dict):
        cmd = c.get("cmd")
        g = self.g
        if cmd == "heartbeat":                                  # hot path: no logging
            g.beat()
            return None
        if cmd == "stop":
            why = c.get("why") or "STOP"
            f = g.stop_all(why)                                 # queued first: the job's move-off comes after it
            self.sun.abort(why)
            self.test.abort(why)
            if self.sat:
                self.sat.abort(why)
            return f.result(3)
        if cmd in ("key", "start", "home", "speed", "goto", "goto_azel", "sun_start", "sun_auto", "sun_gt", "sun_track", "sun_off",
                   "sat_start") and self.test.busy():
            raise RuntimeError("The test is running: press STOP or Abort test first")
        if cmd in ("key", "start", "home", "speed", "goto", "goto_azel") and self.sun.busy():
            raise RuntimeError(f"Sun step running ({self.sun.prog['step']}): Abort it first")
        if cmd in ("key", "start", "home", "speed", "goto", "goto_azel", "sun_start", "sun_auto", "sun_gt", "sun_track", "sun_off",
                   "test_start") and self.sat and self.sat.busy():
            raise RuntimeError("Satellite tracking is running: press Stop first")
        if cmd in ("sun_start", "sun_auto", "sun_gt", "sun_track", "test_start") and self.demod.uses_sdr():
            raise RuntimeError("The demodulator is using the SDR: stop it first")
        if cmd == "rx_start":
            if self.sun.busy() or self.test.busy() or (self.sat and self.sat.busy()):
                raise RuntimeError("Sun tracking, satellite tracking or a test is running: it needs the SDR")
            return self.receive_start(c.get("opts"))
        if cmd == "rehearsal_start":
            self.event("Receiver test started (test signal, no radio)")
            return self.rehearsal.start(c.get("preset") or "field1", c.get("esn0", 12.0))
        if cmd == "rehearsal_clear":
            self.rehearsal.clear_image()
            return "cleared"
        if cmd == "rehearsal_stop":
            self.rehearsal.abort()
            return self.receive_stop("test aborted")
        if cmd == "rx_stop":
            self.rehearsal.abort()
            return self.receive_stop(c.get("why") or "Stop button")
        if cmd == "demod_start":
            o = dict(c.get("opts") or {})
            if self.sim and o.get("mode", "file") != "file" and not o.get("sim_input"):
                raise RuntimeError("Simulator: record/live need a rehearsal file (sim_input), the real radio is never opened with --sim")
            r = self.demod.start(o)
            self.event(f"Demod started: {o.get('mode', 'file')} / {o.get('preset') or 'default preset'}")
            return r
        if cmd == "demod_view":
            return self.demod.view(c.get("run"))
        if cmd == "demod_stop":
            return self.demod.stop(c.get("why") or "Stop button")
        if cmd == "test_start":
            only = c.get("only") or None
            moves = only is None or any(i in only for i in ("motors", "north"))
            if moves and not c.get("confirm"):
                raise RuntimeError("This test moves the axes: confirm that the area is clear and the load is supported")
            if self.sun.busy():
                raise RuntimeError("A Sun step is running: Abort it first")
            self.event("Test started: " + (", ".join(only) if only else "all"))
            return self.test.start(only)
        if cmd == "test_abort":
            self.test.abort(c.get("why") or "Abort test")
            return "aborting"
        if cmd == "goto":
            x, y = float(c["x"]), float(c["y"])
            if not (_finite(x) and _finite(y)):
                raise ValueError("X and Y must be numbers")
            g.goto(x, y, "manual goto").result(3)
            self.event(f"Go to X {x:+.2f}°, Y {y:+.2f}°")
            return {"x": x, "y": y}
        if cmd == "goto_azel":
            az, el = float(c["az"]), float(c["el"])
            if not (_finite(az) and _finite(el) and 0 <= el <= 90):
                raise ValueError("azimuth 0-360 and elevation 0-90 required")
            cc = self.cal()
            cal = cc["cal"] if cc else {"yaw": 0.0}
            x, y = K.azel_to_axes(az % 360, el, cal)             # ValueError if the beam can't point there
            if cc:
                x, y = x + cc["offs"][0], y + cc["offs"][1]
            if not K.within_limits(x, y):
                raise ValueError(f"az {az:.1f}°, el {el:.1f}° needs X {x:+.1f}°, Y {y:+.1f}°: beyond the ±80° soft limits")
            g.goto(x, y, f"az {az:.1f} el {el:.1f}").result(3)
            self.event(f"Go to az {az:.2f}°, el {el:.2f}° (X {x:+.2f}°, Y {y:+.2f}°"
                       f"{'' if cc else ', mount yaw unknown: yaw 0 assumed'})")
            return {"x": x, "y": y, "yaw_known": cc is not None}
        if cmd == "key":
            return g.press(str(c["axis"]), int(c["dir"])).result(3)
        if cmd == "start":
            if not c.get("confirm"):
                raise RuntimeError("Start needs confirmation: motors switch off for a moment - support the load")
            return g.begin().result(3)
        if cmd == "home":
            return g.home().result(3)
        if cmd == "speed":
            return g.set_speed(float(c["value"])).result(3)
        if cmd == "sun_start":
            opts = c.get("opts") or None
            if opts and not self.sim:
                raise RuntimeError("sun_start opts only with --sim")
            res = self.sun.start("find", c.get("method"), opts)
            self.event(f"Sun find started: method {'1 (GNSS + peak-up)' if c.get('method') == 'gnss' else '2 (blind ring scan)'}")
            return res
        if cmd in ("sun_gt", "sun_track", "sun_off"):
            opts = c.get("opts") or None
            if opts and not self.sim:
                raise RuntimeError("opts only with --sim")
            return self.sun.start({"sun_gt": "gt", "sun_track": "track", "sun_off": "off"}[cmd], None, opts)
        if cmd == "sun_auto":
            if not c.get("confirm"):
                raise RuntimeError("Auto starts the motors and moves the dish: confirm that the area is clear and the load is supported")
            self.event("Sun Auto started")
            return self.sun.start("auto")
        if cmd == "sat_start":
            if not c.get("confirm"):
                raise RuntimeError("Tracking starts the motors and moves the dish: confirm that the area is clear and the load is supported")
            if self.sun.busy():
                raise RuntimeError("A Sun step is running: Abort it first")
            if not self.sat:
                raise RuntimeError("satellite tracking needs skyfield:  pip install skyfield")
            cat = c.get("catnr")
            self.event("Satellite tracking started: " + (f"NORAD {cat}" if cat else "auto-pick"))
            return self.sat.start(int(cat) if cat else None, c.get("rx") or None)
        if cmd == "sat_stop":
            if self.sat:
                self.sat.abort(c.get("why") or "Stop button")
            return "stopping"
        if cmd == "sat_add":
            if not self.sat:
                raise RuntimeError("satellite tracking needs skyfield:  pip install skyfield")
            return self.sat.add_target(int(c["catnr"]))
        if cmd == "sat_refresh":
            if self.sat:
                threading.Thread(target=self.sat.predict, kwargs={"force": True}, daemon=True).start()
            return "predicting"
        if cmd == "sun_abort":
            self.sun.abort(c.get("why") or "Abort button")
            return "aborting"
        if cmd == "flux":
            v = c.get("sfu")
            if v not in (None, "") and not (0 < float(v) < 100000):
                raise ValueError("solar flux must be a positive number of sfu")
            return self.sun.set_flux(v)
        if cmd == "sim":                                        # simulator test hooks only
            if not self.sim:
                raise RuntimeError("sim command only available with --sim")
            for k, v in c.items():
                if k in ("cmd", "axis"):
                    continue
                self.world.set_option(c.get("axis", "x"), k, v)
            return "ok"
        raise ValueError(f"unknown command {cmd!r}")

    def close(self):
        self.rehearsal.abort()
        self.demod.stop("app exit")
        t = self.demod.thread
        if t is not None:
            t.join(35)
        self.test.abort("app exit")
        self.sun.abort("app exit")
        if self.sat:
            self.sat.abort("app exit")
        res = self.g.shutdown()
        for x in (self.meter, self.link):
            try:
                x and x.close()
            except Exception:
                pass
        try:
            self.gnss.stop()
        except Exception:
            pass
        if self._plog:
            self._plog[0].close()
        return res


def make_handler(app: App):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body, default=str).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                return self._send(200, UI.read_bytes(), "text/html; charset=utf-8")
            if self.path == "/api/state":
                return self._send(200, app.state())
            if not self.path.startswith("/api/") and self.path not in ("/", "/index.html"):
                try:                                                # ui/model/*.glb, ui/vendor/*.js ... (inside ui/ only)
                    f = (UI_DIR / urlparse(self.path).path.lstrip("/")).resolve()
                    if UI_DIR.resolve() not in f.parents or f.suffix not in STATIC_TYPES or not f.is_file():
                        raise FileNotFoundError
                    data = f.read_bytes()
                except (FileNotFoundError, OSError, ValueError):
                    return self._send(404, {"error": "not found"})
                return self._send(200, data, STATIC_TYPES[f.suffix])
            if self.path == "/api/stream":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                try:
                    while True:
                        self.wfile.write(f"data: {json.dumps(app.state(), default=str)}\n\n".encode())
                        self.wfile.flush()
                        time.sleep(0.25)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    return
            if self.path.startswith("/api/rehearsal/original"):
                p = app.rehearsal.original_path()
                if p is None:
                    return self._send(404, {"error": "no picture"})
                return self._send(200, p.read_bytes(), {".png": "image/png", ".jpg": "image/jpeg", ".jpeg": "image/jpeg", ".gif": "image/gif",
                                                         ".webp": "image/webp", ".bmp": "image/bmp"}.get(p.suffix.lower(), "application/octet-stream"))
            if self.path.startswith("/api/demod/file"):
                q = parse_qs(urlparse(self.path).query)
                try:
                    p, ctype = app.demod.file_path((q.get("run") or [""])[0], (q.get("name") or [""])[0])
                    data = p.read_bytes()
                except (FileNotFoundError, OSError, ValueError):
                    return self._send(404, {"error": "not found"})
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Content-Security-Policy", "sandbox")
                self.send_header("Content-Disposition", ("inline" if ctype.startswith("image/") else "attachment") +
                                 f'; filename="{p.name}"')
                self.end_headers()
                self.wfile.write(data)
                return
            self._send(404, {"error": "not found"})

        def do_POST(self):
            if self.path == "/api/upload":                                  # a picture to send through the receiver test
                try:
                    from urllib.parse import unquote
                    n = int(self.headers.get("Content-Length", 0))
                    if n <= 0 or n > 600_000:
                        raise ValueError("the picture is too big (limit 400 kB)")
                    info = app.rehearsal.set_image(unquote(self.headers.get("X-Filename", "picture.png")), self.rfile.read(n))
                    app.event(f"Picture chosen for the receiver test: {info['name']} ({info['size'] / 1000:.0f} kB)")
                    return self._send(200, {"ok": True, "result": info})
                except Exception as e:
                    return self._send(400, {"ok": False, "error": str(e)})
            if self.path != "/api/cmd":
                return self._send(404, {"error": "not found"})
            c = {}
            try:
                c = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                res = app.command(c)
                self._send(200, {"ok": True, "result": res})
            except Exception as e:
                if c.get("cmd") != "heartbeat":
                    app.event(str(e), "error")
                self._send(400, {"ok": False, "error": str(e)})
    return H


OFF_HELP = """
  Motors are still ON and holding (they are never switched off automatically).
  To switch them off - SUPPORT THE LOAD FIRST (top-heavy, brake unverified), on the Pi:
      cansend can0 641#010000000000      # X (node 1): 0100 = 0
      cansend can0 642#010000000000      # Y (node 2): 0100 = 0
"""


def main():
    ap = argparse.ArgumentParser(description="Ground station UI (test stage: Manual + Sun / G/T)")
    ap.add_argument("--pi", default=config.CAN_BUS.pi, help="Pi running can_relay.py: <host>[:7070]")
    ap.add_argument("--iface", default=config.CAN_BUS.interface,
                    choices=["remote", "canutils", "socketcan", "waveshare", "sim"],
                    help="CAN backend (default remote = the Pi relay)")
    ap.add_argument("--sim", action="store_true", help="simulated drives, SDR and GNSS, no hardware")
    ap.add_argument("--sim-clock", help="pretend 'now' starts here, e.g. 2026-11-09T09:30:00+08:00")
    ap.add_argument("--gnss", default="auto", help="auto (find /dev/ttyACM* u-blox) | none | <port>")
    ap.add_argument("--no-sdr", action="store_true", help="don't open the B206mini")
    ap.add_argument("--host", default="127.0.0.1", help="127.0.0.1 (default) or 0.0.0.0 (trusted LAN only!)")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--no-browser", action="store_true", help="don't open the page in the browser automatically")
    a = ap.parse_args()
    for stream in (sys.stdout, sys.stderr):                     # Windows console: degree signs etc. in the log
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    if a.sim_clock:
        clock.set_sim_start(a.sim_clock)
    sim = a.sim or a.iface == "sim"
    if not sim and a.iface == "remote" and not a.pi:
        a.pi = os.environ.get("GS_PI") or "127.0.0.1"          # no Pi given: start anyway, the gimbal simply shows "not detected"
        if a.pi == "127.0.0.1":
            print("\n  No Pi address given (--pi <ip> or GS_PI=<ip>): starting WITHOUT the gimbal link."
                  "\n  GNSS, SDR, Sun / satellite prediction and the Test page still work.\n")
    if sim:
        os.environ.setdefault("GS_SIM", "1")
    app = App(sim, a.iface, a.pi, a.gnss, not a.no_sdr)
    srv = None
    for port in range(a.port, a.port + 11):                      # 8080 busy? take the next free one
        try:
            srv = ThreadingHTTPServer((a.host, port), make_handler(app))
            break
        except OSError:
            continue
    if srv is None:
        sys.exit(f"ports {a.port}-{a.port + 10} are all busy: use --port")
    srv.daemon_threads = True
    url = f"http://{'localhost' if a.host in ('127.0.0.1', '0.0.0.0') else a.host}:{port}"
    print(f"\n  Ground station UI: {url}   ({'SIMULATOR' if sim else 'HARDWARE: ' + app.link.name})\n")
    if not a.no_browser:
        import webbrowser
        threading.Timer(1.5, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down: STOP both (motors stay ON, holding)")
    finally:
        print(f"  {app.close()}")
        if not sim:
            print(OFF_HELP)


if __name__ == "__main__":
    main()


# ==================================================================================================
# DISABLED (test stage): the full app (Calibrate / Test / Auto, sky dome, GNSS, SDR, orbits).
# Needs the imports commented out at the top (numpy, skyfield, pyubx2, uhd behind them).
# ==================================================================================================
# class App:
#     def __init__(self, sim: bool, iface: str):
#         self.sim = sim
#         self.cal = Calibration()
#         self.events = deque(maxlen=60)
#         if sim:
#             from sim_hw import SimCanLink, SimGnss, SimSunSensor
#             link, self.gnss, self._sdr = SimCanLink(), SimGnss(), SimSunSensor()
#         else:
#             from gps_reader import GnssReader
#             from zeroerr_protocol import open_link
#             link = open_link(config.CanBusConfig(interface=iface))
#             self.gnss, self._sdr = GnssReader(), None
#         self.gnss.start()
#         self.g = Gimbal(link, home=self.cal.data["home"])
#         self.g.start()
#         self.auto = AutoTracker(self.g, self.cal, self.site, sim=sim)
#         self.job = None
#         self.ui_mode = "calibrate" if self.cal.status in ("none", "home") else "test"
#         if not sim:
#             threading.Thread(target=self.sdr, daemon=True).start()       # open the USRP in the background
#
#     # ------------------------------------------------------------------ helpers
#     def sdr(self):
#         if self._sdr is None:
#             from sdr_power import SdrPower
#             self._sdr = SdrPower()
#         return self._sdr
#
#     def site(self) -> config.SiteConfig:
#         s = self.cal.data.get("site")
#         if s:
#             return config.SiteConfig(s["lat"], s["lon"], s["h"], "GNSS (saved)")
#         st = self.gnss.status()
#         if st.get("fix_type", 0) >= 3:
#             return config.SiteConfig(st["lat"], st["lon"], st.get("h_msl", 0.0), "GNSS (live)")
#         return config.SITE
#
#     def event(self, msg, level="info"):
#         self.events.append({"t": clock.now().isoformat(timespec="seconds"), "level": level, "msg": msg})
#         log.info(msg)
#
#     def sun_path(self, site):
#         """Today's Sun track (local day) for the sky dome, cached per date."""
#         from datetime import datetime, timedelta, timezone
#         local = clock.now().astimezone(timezone(timedelta(hours=8)))
#         key = (local.date(), round(site.latitude_deg, 2), round(site.longitude_deg, 2))
#         if getattr(self, "_sun_key", None) != key:
#             start = datetime(local.year, local.month, local.day, tzinfo=local.tzinfo)
#             pts = []
#             for m in range(0, 24 * 60, 10):
#                 az, el = sun_azel(start + timedelta(minutes=m), site.latitude_deg, site.longitude_deg)
#                 if el > -2:
#                     pts.append([round(az, 2), round(el, 2), f"{m // 60:02d}:{m % 60:02d}"])
#             self._sun_key, self._sun_path = key, pts
#         return self._sun_path
#
#     def _job_running(self):
#         return self.job is not None and self.job.progress["state"] == "running"
#
#     # ------------------------------------------------------------------ state for the UI
#     def state(self) -> dict:
#         now = clock.now()
#         site = self.site()
#         g = self.g.snapshot()
#         x, y = self.g.angles()
#         pointing = None
#         if x is not None and y is not None:
#             az, el = K.axes_to_azel(x, y, self.cal.params)
#             pointing = {"az": az, "el": el, "compass": K.compass(az)}
#         saz, sel = sun_azel(now, site.latitude_deg, site.longitude_deg)
#         gs = self.gnss.status()
#         sdr = self._sdr
#         evs = sorted(list(self.events) + list(self.g.events), key=lambda e: e["t"])[-14:]
#         return {
#             "now": now.isoformat(), "sim": self.sim, "ui_mode": self.ui_mode, "gimbal": g, "pointing": pointing,
#             "sun": {"az": saz, "el": sel, "path": self.sun_path(site),
#                     "scan_ok": config.SUN_SCAN.min_elevation_deg <= sel <= config.SUN_SCAN.max_elevation_deg,
#                     "scan_window": [config.SUN_SCAN.min_elevation_deg, config.SUN_SCAN.max_elevation_deg]}, "site": {"lat": site.latitude_deg, "lon": site.longitude_deg,
#                                                     "source": site.name},
#             "cal": self.cal.summary(), "job": self.job.progress if self.job else None,
#             "auto": self.auto.snapshot(),
#             "gnss": {k: gs.get(k) for k in ("connected", "port", "fix_type", "num_sv", "carr", "heading",
#                                             "pitch", "heading_valid", "clock_offset_s", "error")},
#             "sdr": {"available": bool(sdr and sdr.available), "error": getattr(sdr, "error", None) if sdr else "opening"},
#             "jog_speeds": config.JOG_SPEEDS_DEG_S, "events": evs,
#         }
#
#     # ------------------------------------------------------------------ commands
#     def command(self, c: dict):
#         cmd = c.get("cmd")
#         g = self.g
#         if cmd == "jog":                               # hot path: no logging
#             if self.ui_mode in ("test", "calibrate") and not self._job_running() and not self.auto.running:
#                 g.jog(c.get("x", 0), c.get("y", 0))
#             return
#         if cmd == "mode":
#             m = c["mode"]
#             if m != self.ui_mode:
#                 self._stop_activity()
#                 self.ui_mode = m
#         elif cmd == "estop":
#             self._stop_activity()
#             g.estop().result(3)
#         elif cmd == "enable":
#             g.enable().result(5)
#         elif cmd == "disable":
#             self._stop_activity()
#             g.disable().result(3)
#         elif cmd == "reset":
#             g.reset_fault().result(3)
#         elif cmd == "jog_speed":
#             g.set_jog_speed(float(c["value"]))
#         elif cmd == "set_home":
#             home = g.set_home().result(3)
#             self.cal.set_home(home)
#             self.event("HOME saved - calibration measurements cleared (they were relative to the old HOME)")
#         elif cmd == "go_home":
#             g.goto(0.0, 0.0).result(3)
#         elif cmd in ("cal_gnss", "cal_sun"):
#             if self._job_running():
#                 raise RuntimeError("A calibration step is already running")
#             if self.auto.running:
#                 raise RuntimeError("Stop Auto mode first")
#             if cmd == "cal_gnss":
#                 self.job = GnssPoseJob(g, self.cal, self.site, self.gnss).start()
#             else:
#                 self.job = SunScanJob(g, self.cal, self.site, self.sdr()).start()
#         elif cmd == "cal_cancel":
#             if self.job:
#                 self.job.cancel()
#         elif cmd == "cal_clear":
#             self.cal.clear_measurements()
#             self.event("calibration measurements cleared (HOME kept)")
#         elif cmd == "auto_start":
#             if self.cal.status in ("none", "home") and not c.get("force"):
#                 raise RuntimeError("Calibrate first: HOME plus the GNSS or Sun step")
#             if self._job_running():
#                 raise RuntimeError("A calibration step is running")
#             self.auto.start()
#             self.event("Auto mode started")
#         elif cmd == "auto_stop":
#             self.auto.stop()
#             self.event("Auto mode stopped")
#         else:
#             raise ValueError(f"unknown command {cmd!r}")
#
#     def _stop_activity(self):
#         if self.job and self._job_running():
#             self.job.cancel()
#         if self.auto.running:
#             self.auto.stop()
#         self.g.jog(0, 0)
#         if self.g.mode in ("track", "goto", "jog"):
#             self.g.hold().result(3)
#
#
# def make_handler(app: App):
#     class H(BaseHTTPRequestHandler):
#         def log_message(self, *a):
#             pass
#
#         def _send(self, code, body, ctype="application/json"):
#             data = body if isinstance(body, bytes) else json.dumps(body, default=str).encode()
#             self.send_response(code)
#             self.send_header("Content-Type", ctype)
#             self.send_header("Content-Length", str(len(data)))
#             self.send_header("Cache-Control", "no-store")
#             self.end_headers()
#             self.wfile.write(data)
#
#         def do_GET(self):
#             if self.path in ("/", "/index.html"):
#                 return self._send(200, UI.read_bytes(), "text/html; charset=utf-8")
#             if self.path == "/api/state":
#                 return self._send(200, app.state())
#             if self.path == "/api/stream":
#                 self.send_response(200)
#                 self.send_header("Content-Type", "text/event-stream")
#                 self.send_header("Cache-Control", "no-cache")
#                 self.end_headers()
#                 try:
#                     while True:
#                         self.wfile.write(f"data: {json.dumps(app.state(), default=str)}\n\n".encode())
#                         self.wfile.flush()
#                         time.sleep(0.25)
#                 except (BrokenPipeError, ConnectionResetError):
#                     return
#             self._send(404, {"error": "not found"})
#
#         def do_POST(self):
#             if self.path != "/api/cmd":
#                 return self._send(404, {"error": "not found"})
#             try:
#                 c = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
#                 app.command(c)
#                 self._send(200, {"ok": True})
#             except Exception as e:
#                 if c.get("cmd") != "jog":
#                     app.event(str(e), "error")
#                 self._send(400, {"ok": False, "error": str(e)})
#     return H
#
#
# def main():
#     ap = argparse.ArgumentParser(description="Ground station UI")
#     ap.add_argument("--sim", action="store_true", help="simulated motors/GNSS/SDR")
#     ap.add_argument("--sim-clock", help="simulator start time, e.g. 2026-11-06T09:00:00+08:00")
#     ap.add_argument("--iface", default="auto", choices=["auto", "socketcan", "waveshare"])
#     ap.add_argument("--host", default="127.0.0.1")
#     ap.add_argument("--port", type=int, default=8080)
#     a = ap.parse_args()
#     logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
#     if a.sim_clock:
#         clock.set_sim_start(a.sim_clock)
#     if a.sim:
#         os.environ.setdefault("GS_SIM", "1")
#     app = App(a.sim, a.iface)
#     srv = ThreadingHTTPServer((a.host, a.port), make_handler(app))
#     srv.daemon_threads = True
#     print(f"\n  Ground station UI: http://{'localhost' if a.host in ('127.0.0.1', '0.0.0.0') else a.host}:{a.port}"
#           f"   ({'SIMULATOR' if a.sim else 'HARDWARE'})\n")
#     try:
#         srv.serve_forever()
#     except KeyboardInterrupt:
#         print("\nshutting down: motors off")
#     finally:
#         app.auto.stop()
#         app.g.shutdown()
#
#
# if __name__ == "__main__":
#     main()
