"""
gnss_check.py - live check of the ZED-F9P heading kit (receive only, uses gps_reader.GnssReader).

    ~/radioconda/bin/python gnss_check.py [--port /dev/ttyACM0] [--once]

Prints once a second: fix, satellites, RTK state, baseline heading with a compass direction, pitch,
baseline length, and a PASS/FAIL check of each. Ctrl-C to stop.
The direction shown is the baseline (base antenna toward rover antenna), not the dish.
"""
import argparse
import sys
import time

import gps_reader

FIX = {0: "no fix", 1: "dead reck", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "time only"}
CARR = {0: "none", 1: "RTK FLOAT", 2: "RTK FIXED"}
PTS = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]


def compass(deg):
    return PTS[int((deg % 360) / 22.5 + 0.5) % 16]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baseline-cm", type=float, default=None, help="optional: true antenna spacing; if omitted the length is not checked")
    ap.add_argument("--tol-cm", type=float, default=5.0, help="allowed length error (default 5)")
    ap.add_argument("--once", action="store_true", help="print one report after 15 s and exit")
    a = ap.parse_args()

    port = a.port or gps_reader.find_rover_port()
    if not port:
        sys.exit("No u-blox (VID 1546) /dev/ttyACM* found. Plug in the rover board (POWER+GPS USB) and check `ls /dev/ttyACM*`.\n"
                 "If it never appears: `sudo systemctl stop ModemManager` and make sure you are in the dialout group.")
    g = gps_reader.GnssReader(port)
    g.start()
    print(f"reading {port} ... (RTK FIXED needs both antennas with clear sky; can take a few minutes)\n")
    t0 = time.time()
    good = []          # headings from epochs that are RTK FIXED and the right length and level
    try:
        while True:
            time.sleep(1)
            s = g.status()
            if not s.get("connected"):
                print(f"[{time.time()-t0:4.0f}s] no data yet  {s.get('error') or ''}")
            else:
                fix, carr = s.get("fix_type", 0), s.get("carr")
                line = (f"[{time.time()-t0:4.0f}s] fix {FIX.get(fix, fix)}, {s.get('num_sv')} sv, hAcc {s.get('h_acc_m', 0):.2f} m"
                        f" | lat {s['lat']:.6f} lon {s['lon']:.6f}")
                if "heading" in s:
                    ok = (s["heading_valid"] and (a.baseline_cm is None or abs(s["length_cm"] - a.baseline_cm) <= a.tol_cm) and abs(s["pitch"]) < 10)
                    if ok:
                        good.append(s["heading"])
                    line += (f" | {CARR.get(carr, carr)} | HEADING {s['heading']:6.2f} deg ({compass(s['heading'])})"
                             f" pitch {s['pitch']:+.2f} len {s['length_cm']:.1f} cm"
                             f" {'GOOD' if ok else ('FALSE FIX (length/pitch off)' if s['heading_valid'] else 'not valid yet')}")
                else:
                    line += " | no RELPOSNED yet (is the rover receiving corrections from the base?)"
                print(line)
            if a.once and time.time() - t0 > 15:
                break
    except KeyboardInterrupt:
        pass
    s = g.status()
    g.stop()
    print("\n=== validation ===")
    checks = [
        ("USB data flowing", s.get("connected")),
        ("3D fix", s.get("fix_type") in (3, 4)),
        (">= 8 satellites", (s.get("num_sv") or 0) >= 8),
        ("RTK FIXED (carrSoln 2)", s.get("carr") == 2),
        ("heading valid", s.get("heading_valid")),
        ("baseline length (not checked)" if a.baseline_cm is None else f"baseline length within {a.tol_cm:.0f} cm of {a.baseline_cm:.0f} cm",
         a.baseline_cm is None or abs(s.get("length_cm", 1e9) - a.baseline_cm) <= a.tol_cm),
        ("baseline level (|pitch| < 10 deg)", abs(s.get("pitch", 99)) < 10),
    ]
    for name, ok in checks:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    if len(good) >= 5:
        import math
        r = [math.radians(h) for h in good]
        m = math.degrees(math.atan2(sum(map(math.sin, r)), sum(map(math.cos, r)))) % 360
        sd = (sum(((h - m + 180) % 360 - 180) ** 2 for h in good) / len(good)) ** 0.5
        print(f"\nBaseline points {m:.1f} deg true ({compass(m)}) base->rover, std {sd:.2f} deg over {len(good)} GOOD epochs.")
    else:
        print("\nNot enough GOOD epochs (RTK FIXED + correct length + level) for a trustworthy heading.")


if __name__ == "__main__":
    main()
