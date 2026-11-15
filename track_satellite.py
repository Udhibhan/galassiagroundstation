#!/usr/bin/env python3
"""
track_satellite.py - headless: track the next pass of one satellite, no UI.

    python3 track_satellite.py 56309            # LUMELITE-4 by NORAD number
    python3 track_satellite.py 56309 --sim

v2's 10 Hz velocity loop (18 CAN transactions per cycle, mode switches while enabled, no encoder
zero) is gone: this now runs the same AutoTracker + Gimbal as Auto mode, limited to one target.
Ctrl+C stops and switches the motors off.
"""
import argparse
import logging
import time

import config


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("catnr", type=int)
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--iface", default="auto")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(message)s")
    config.TARGETS = {a.catnr: config.TARGETS.get(a.catnr, str(a.catnr))}
    from app import App
    app = App(a.sim, a.iface)
    time.sleep(3)
    try:
        app.command({"cmd": "auto_start"})
        last = ""
        while True:
            s = app.auto.snapshot()
            if s["msg"] != last:
                print(s["msg"])
                last = s["msg"]
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        app.auto.stop()
        app.g.shutdown()


if __name__ == "__main__":
    main()
