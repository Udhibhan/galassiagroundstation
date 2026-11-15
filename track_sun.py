#!/usr/bin/env python3
"""
track_sun.py - headless: run ONE automatic Sun scan and update calibration.json (no UI).

    python3 track_sun.py           # needs HOME set, motors enable-able, SDR on the LNB chain
    python3 track_sun.py --sim --sim-clock 2026-11-06T09:00:00+08:00

v2 tracked the Sun open-loop and asked a person to rotate the base by eye: that measures nothing.
v3 cross-scans the Sun and finds the Ku-band noise peak with the B206mini (receive only), then
refits yaw / tilt / encoder zeros (calibration.py). Same code as the UI's "Scan the Sun".
"""
import argparse
import logging
import time

import clock


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--sim-clock")
    ap.add_argument("--iface", default="auto")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(message)s")
    if a.sim_clock:
        clock.set_sim_start(a.sim_clock)
    from app import App
    app = App(a.sim, a.iface)
    time.sleep(3)
    try:
        if not app.g.homed:
            raise SystemExit("HOME is not set: run app.py, Calibrate step 1.")
        app.g.enable().result(5)
        time.sleep(1.5)
        app.command({"cmd": "cal_sun"})
        while app.job.progress["state"] == "running":
            time.sleep(1)
        print(app.job.progress["state"].upper(), "-", app.job.progress["msg"])
        print("calibration:", app.cal.summary()["status"], {k: round(v, 3) for k, v in app.cal.params.items()})
    except KeyboardInterrupt:
        if app.job:
            app.job.cancel()
    finally:
        app.g.shutdown()


if __name__ == "__main__":
    main()
