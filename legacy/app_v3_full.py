# Old version from before the test stage, taken out of app.py. Not imported anywhere.
# Full v3 app (Calibrate / Test / Auto, sky dome, GNSS, SDR, orbits).
# To go back to it, copy this over the live code.

class App:
    def __init__(self, sim: bool, iface: str):
        self.sim = sim
        self.cal = Calibration()
        self.events = deque(maxlen=60)
        if sim:
            from sim_hw import SimCanLink, SimGnss, SimSunSensor
            link, self.gnss, self._sdr = SimCanLink(), SimGnss(), SimSunSensor()
        else:
            from gps_reader import GnssReader
            from zeroerr_protocol import open_link
            link = open_link(config.CanBusConfig(interface=iface))
            self.gnss, self._sdr = GnssReader(), None
        self.gnss.start()
        self.g = Gimbal(link, home=self.cal.data["home"])
        self.g.start()
        self.auto = AutoTracker(self.g, self.cal, self.site, sim=sim)
        self.job = None
        self.ui_mode = "calibrate" if self.cal.status in ("none", "home") else "test"
        if not sim:
            threading.Thread(target=self.sdr, daemon=True).start()       # open the USRP in the background

    # ------------------------------------------------------------------ helpers
    def sdr(self):
        if self._sdr is None:
            from sdr_power import SdrPower
            self._sdr = SdrPower()
        return self._sdr

    def site(self) -> config.SiteConfig:
        s = self.cal.data.get("site")
        if s:
            return config.SiteConfig(s["lat"], s["lon"], s["h"], "GNSS (saved)")
        st = self.gnss.status()
        if st.get("fix_type", 0) >= 3:
            return config.SiteConfig(st["lat"], st["lon"], st.get("h_msl", 0.0), "GNSS (live)")
        return config.SITE

    def event(self, msg, level="info"):
        self.events.append({"t": clock.now().isoformat(timespec="seconds"), "level": level, "msg": msg})
        log.info(msg)

    def sun_path(self, site):
        """Today's Sun track (local day) for the sky dome, cached per date."""
        from datetime import datetime, timedelta, timezone
        local = clock.now().astimezone(timezone(timedelta(hours=8)))
        key = (local.date(), round(site.latitude_deg, 2), round(site.longitude_deg, 2))
        if getattr(self, "_sun_key", None) != key:
            start = datetime(local.year, local.month, local.day, tzinfo=local.tzinfo)
            pts = []
            for m in range(0, 24 * 60, 10):
                az, el = sun_azel(start + timedelta(minutes=m), site.latitude_deg, site.longitude_deg)
                if el > -2:
                    pts.append([round(az, 2), round(el, 2), f"{m // 60:02d}:{m % 60:02d}"])
            self._sun_key, self._sun_path = key, pts
        return self._sun_path

    def _job_running(self):
        return self.job is not None and self.job.progress["state"] == "running"

    # ------------------------------------------------------------------ state for the UI
    def state(self) -> dict:
        now = clock.now()
        site = self.site()
        g = self.g.snapshot()
        x, y = self.g.angles()
        pointing = None
        if x is not None and y is not None:
            az, el = K.axes_to_azel(x, y, self.cal.params)
            pointing = {"az": az, "el": el, "compass": K.compass(az)}
        saz, sel = sun_azel(now, site.latitude_deg, site.longitude_deg)
        gs = self.gnss.status()
        sdr = self._sdr
        evs = sorted(list(self.events) + list(self.g.events), key=lambda e: e["t"])[-14:]
        return {
            "now": now.isoformat(), "sim": self.sim, "ui_mode": self.ui_mode, "gimbal": g, "pointing": pointing,
            "sun": {"az": saz, "el": sel, "path": self.sun_path(site),
                    "scan_ok": config.SUN_SCAN.min_elevation_deg <= sel <= config.SUN_SCAN.max_elevation_deg,
                    "scan_window": [config.SUN_SCAN.min_elevation_deg, config.SUN_SCAN.max_elevation_deg]}, "site": {"lat": site.latitude_deg, "lon": site.longitude_deg,
                                                    "source": site.name},
            "cal": self.cal.summary(), "job": self.job.progress if self.job else None,
            "auto": self.auto.snapshot(),
            "gnss": {k: gs.get(k) for k in ("connected", "port", "fix_type", "num_sv", "carr", "heading",
                                            "pitch", "heading_valid", "clock_offset_s", "error")},
            "sdr": {"available": bool(sdr and sdr.available), "error": getattr(sdr, "error", None) if sdr else "opening"},
            "jog_speeds": config.JOG_SPEEDS_DEG_S, "events": evs,
        }

    # ------------------------------------------------------------------ commands
    def command(self, c: dict):
        cmd = c.get("cmd")
        g = self.g
        if cmd == "jog":                               # hot path: no logging
            if self.ui_mode in ("test", "calibrate") and not self._job_running() and not self.auto.running:
                g.jog(c.get("x", 0), c.get("y", 0))
            return
        if cmd == "mode":
            m = c["mode"]
            if m != self.ui_mode:
                self._stop_activity()
                self.ui_mode = m
        elif cmd == "estop":
            self._stop_activity()
            g.estop().result(3)
        elif cmd == "enable":
            g.enable().result(5)
        elif cmd == "disable":
            self._stop_activity()
            g.disable().result(3)
        elif cmd == "reset":
            g.reset_fault().result(3)
        elif cmd == "jog_speed":
            g.set_jog_speed(float(c["value"]))
        elif cmd == "set_home":
            home = g.set_home().result(3)
            self.cal.set_home(home)
            self.event("HOME saved - calibration measurements cleared (they were relative to the old HOME)")
        elif cmd == "go_home":
            g.goto(0.0, 0.0).result(3)
        elif cmd in ("cal_gnss", "cal_sun"):
            if self._job_running():
                raise RuntimeError("A calibration step is already running")
            if self.auto.running:
                raise RuntimeError("Stop Auto mode first")
            if cmd == "cal_gnss":
                self.job = GnssPoseJob(g, self.cal, self.site, self.gnss).start()
            else:
                self.job = SunScanJob(g, self.cal, self.site, self.sdr()).start()
        elif cmd == "cal_cancel":
            if self.job:
                self.job.cancel()
        elif cmd == "cal_clear":
            self.cal.clear_measurements()
            self.event("calibration measurements cleared (HOME kept)")
        elif cmd == "auto_start":
            if self.cal.status in ("none", "home") and not c.get("force"):
                raise RuntimeError("Calibrate first: HOME plus the GNSS or Sun step")
            if self._job_running():
                raise RuntimeError("A calibration step is running")
            self.auto.start()
            self.event("Auto mode started")
        elif cmd == "auto_stop":
            self.auto.stop()
            self.event("Auto mode stopped")
        else:
            raise ValueError(f"unknown command {cmd!r}")

    def _stop_activity(self):
        if self.job and self._job_running():
            self.job.cancel()
        if self.auto.running:
            self.auto.stop()
        self.g.jog(0, 0)
        if self.g.mode in ("track", "goto", "jog"):
            self.g.hold().result(3)


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
                except (BrokenPipeError, ConnectionResetError):
                    return
            self._send(404, {"error": "not found"})

        def do_POST(self):
            if self.path != "/api/cmd":
                return self._send(404, {"error": "not found"})
            try:
                c = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                app.command(c)
                self._send(200, {"ok": True})
            except Exception as e:
                if c.get("cmd") != "jog":
                    app.event(str(e), "error")
                self._send(400, {"ok": False, "error": str(e)})
    return H


def main():
    ap = argparse.ArgumentParser(description="Ground station UI")
    ap.add_argument("--sim", action="store_true", help="simulated motors/GNSS/SDR")
    ap.add_argument("--sim-clock", help="simulator start time, e.g. 2026-11-06T09:00:00+08:00")
    ap.add_argument("--iface", default="auto", choices=["auto", "socketcan", "waveshare"])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    if a.sim_clock:
        clock.set_sim_start(a.sim_clock)
    if a.sim:
        os.environ.setdefault("GS_SIM", "1")
    app = App(a.sim, a.iface)
    srv = ThreadingHTTPServer((a.host, a.port), make_handler(app))
    srv.daemon_threads = True
    print(f"\n  Ground station UI: http://{'localhost' if a.host in ('127.0.0.1', '0.0.0.0') else a.host}:{a.port}"
          f"   ({'SIMULATOR' if a.sim else 'HARDWARE'})\n")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down: motors off")
    finally:
        app.auto.stop()
        app.g.shutdown()


if __name__ == "__main__":
    main()
