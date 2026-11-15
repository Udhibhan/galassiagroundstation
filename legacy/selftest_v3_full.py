# Old version from before the test stage, taken out of selftest.py. Not imported anywhere.
# v3 selftest (geometry, Sun, goto/jog/track, calibration solver).
# To go back to it, copy this over the live code.

def old_main():
    import config
    import kinematics as K
    from sun import sun_azel
    from datetime import datetime, timezone
    import calibration as C

    print("geometry")
    cal = {"yaw": 3.0, "tilt_e": 0.0, "tilt_n": -0.4, "x0": 0.7, "y0": -0.5}
    worst = 0.0
    for _ in range(500):
        az, el = random.uniform(0, 360), random.uniform(15, 89)
        x, y = K.azel_to_axes(az, el, cal)
        worst = max(worst, K.angle_between(K.body_to_sky(x, y, cal), K.azel_to_enu(az, el)))
    check("az/el -> X/Y -> az/el round trip", worst < 1e-6, f"worst {worst:.1e} deg")
    x, y = K.azel_to_axes(0, 45)
    check("north at 45 deg el = X -45 (W key side)", abs(x + 45) < 1e-9 and abs(y) < 1e-9, f"x {x:+.1f} y {y:+.1f}")
    x, y = K.azel_to_axes(90, 45)
    check("east at 45 deg el = Y +45 (D key side)", abs(y - 45) < 1e-9 and abs(x) < 1e-9, f"x {x:+.1f} y {y:+.1f}")

    print("sun")
    # reference: Skyfield + DE421, geometric (no refraction), Singapore 1.2966N 103.7764E
    az, el = sun_azel(datetime(2026, 11, 6, 1, 0, tzinfo=timezone.utc), 1.2966, 103.7764, refraction=False)
    check("Sun 2026-11-06 09:00 SGT", 30 < el < 40 and 105 < az < 125, f"az {az:.2f} el {el:.2f}")

    print("CAN framing")
    from zeroerr_protocol import DriveError, deg_to_counts
    check("90 deg = 0x00020000 counts", deg_to_counts(90) == 0x20000, hex(deg_to_counts(90)))
    e = DriveError(1, 0x83, bytes([0x00, 0x27, 0x3E]))
    check("refusal 00 27 3E decoded as 0x27", e.code == 0x27, str(e))

    print("controller on simulated drives")
    from gimbal import Gimbal
    from sim_hw import SimCanLink
    import clock
    g = Gimbal(SimCanLink())
    g.start()
    time.sleep(2.5)
    check("drives found", g.mode == "disabled", g.mode)
    g.set_home().result(3)
    g.enable().result(3)
    time.sleep(1.5)
    g.goto(15, -10).result(3)
    g.wait_until_still(30)
    x, y = g.angles()
    check("goto +15/-10", abs(x - 15) < 0.02 and abs(y + 10) < 0.02, f"{x:+.3f} {y:+.3f}")
    g.set_jog_speed(1.0)
    for _ in range(8):
        g.jog(0, 1)
        time.sleep(0.15)
    g.jog(0, 0)
    y_rel = g.angles()[1]
    time.sleep(1.0)
    y_after = g.angles()[1]
    check("jog moves and stops on release", y_rel > -10 + 0.4 and abs(y_after - y_rel) < 0.3 and g.mode == "hold",
          f"y {y_rel:+.2f} -> {y_after:+.2f}, mode {g.mode}")
    t0 = clock.now()
    g.track(lambda t: (15 + 0.8 * (t - t0).total_seconds(), y_after + 0.3 * (t - t0).total_seconds()), "ramp").result(3)
    errs = []
    for _ in range(40):
        time.sleep(0.25)
        errs.append(g.track_err or 0)
    mean = sum(errs[8:]) / len(errs[8:])
    check("track 0.85 deg/s ramp", mean < 0.3 and g.mode == "track", f"mean error {mean:.3f} deg")
    g.estop().result(3)
    check("STOP -> motors off", g.mode == "disabled" and not any(a.enabled for a in g.axes.values()), g.mode)
    g._stop.set()

    print("calibration solver")
    truth = {"yaw": 3.2, "tilt_e": 0.0, "tilt_n": -0.35, "x0": -0.8, "y0": 0.6}
    gnss = []
    for x, y in [(0, 0), (0, 25), (0, -25), (25, 0), (-25, 0), (20, 20)]:
        az, el = K.enu_to_azel(K.body_to_sky(x, y, truth, config.GNSS.bar_body))
        gnss.append({"x": x, "y": y, "heading": az + random.gauss(0, 0.2), "pitch": el + random.gauss(0, 0.3)})
    suns = []
    for h in (1.2, 8.3):
        t = datetime(2026, 11, 6, int(h), int(h % 1 * 60), tzinfo=timezone.utc)
        az, el = sun_azel(t, config.SITE.latitude_deg, config.SITE.longitude_deg)
        x, y = K.azel_to_axes(az, el, truth)
        suns.append({"x": x, "y": y, "az": az, "el": el})
    fit = C.solve(gnss, suns)
    err = max(abs(fit["params"][k] - truth[k]) for k in ("yaw", "x0"))
    check("recovers yaw and X zero", err < 0.2, {k: round(v, 2) for k, v in fit["params"].items()})
    pe = max(K.angle_between(K.body_to_sky(*K.azel_to_axes(a, e, fit["params"]), truth), K.azel_to_enu(a, e))
             for a in range(0, 360, 30) for e in (20, 45, 70))
    check("pointing error after calibration", pe < 0.3, f"worst {pe:.3f} deg over the sky")

    print("\nOVERALL:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)
