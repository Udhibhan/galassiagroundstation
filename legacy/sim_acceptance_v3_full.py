# Old version from before the test stage, taken out of sim_acceptance.py. Not imported anywhere.
# v3 acceptance run (jog, HOME, GNSS + Sun calibration, Auto tracking).
# To go back to it, copy this over the live code.

def old_state():
    return json.load(urllib.request.urlopen(BASE + "/api/state", timeout=5))


def old_cmd(**c):
    req = urllib.request.Request(BASE + "/api/cmd", json.dumps(c).encode(), {"Content-Type": "application/json"})
    try:
        return json.load(urllib.request.urlopen(req, timeout=10))
    except urllib.error.HTTPError as e:
        return json.load(e)


def old_check(name, ok, detail=""):
    results.append(ok)
    print(f"  {'PASS' if ok else 'FAIL'}  {name:<46} {detail}", flush=True)


def old_wait(pred, timeout, step=0.5):
    end = time.time() + timeout
    while time.time() < end:
        s = state()
        if pred(s):
            return s
        time.sleep(step)
    return state()


def old_job_done(s):
    return s["job"] and s["job"]["state"] != "running"


def old_main():
    work = tempfile.mkdtemp(prefix="gs_sim_")
    if len(sys.argv) > 1:                              # optional: an omm_cache.json to use offline
        shutil.copy(sys.argv[1], Path(work) / "omm_cache.json")
    print(f"simulator working folder: {work}")
    app = subprocess.Popen([sys.executable, str(HERE / "app.py"), "--sim", "--port", str(PORT),
                            "--sim-clock", "2026-11-06T09:00:00+08:00"], cwd=work,
                           stdout=open(Path(work) / "app.log", "w"), stderr=subprocess.STDOUT)
    try:
        for _ in range(40):
            try:
                state()
                break
            except Exception:
                time.sleep(0.5)
        print("\n1. start-up")
        s = wait(lambda s: s["gimbal"]["mode"] == "disabled", 15)
        check("simulated drives answer, motors off", s["gimbal"]["mode"] == "disabled", s["gimbal"]["mode"])
        r = cmd(cmd="auto_start")
        check("Auto refuses before calibration", not r["ok"], r.get("error", ""))

        print("\n2. jog before HOME")
        cmd(cmd="enable")
        s = wait(lambda s: s["gimbal"]["mode"] == "hold", 10)
        check("enable -> hold", s["gimbal"]["mode"] == "hold", s["gimbal"]["mode"])
        y0 = s["gimbal"]["axes"]["y"]["angle"]
        t_end = time.time() + 2.0
        while time.time() < t_end:
            cmd(cmd="jog", x=0, y=1)
            time.sleep(0.15)
        cmd(cmd="jog", x=0, y=0)
        s = wait(lambda s: s["gimbal"]["mode"] == "hold", 5)
        y1 = s["gimbal"]["axes"]["y"]["angle"]
        time.sleep(1.0)
        y2 = state()["gimbal"]["axes"]["y"]["angle"]
        check("hold D moves Y east, release stops", 0.5 < y1 - y0 < 2.0 and abs(y2 - y1) < 0.05,
              f"{y0:+.2f} -> {y1:+.2f} -> {y2:+.2f}")
        r = cmd(cmd="mode", mode="test")
        cmd(cmd="jog", x=1, y=0)                       # one heartbeat, then silence
        started = wait(lambda s: s["gimbal"]["mode"] == "jog", 3, 0.1)["gimbal"]["mode"] == "jog"
        s = wait(lambda s: s["gimbal"]["mode"] == "hold", 5)
        check("deadman: lost heartbeat -> stops", started and s["gimbal"]["mode"] == "hold",
              f"started={started}, now {s['gimbal']['mode']}")
        cmd(cmd="mode", mode="calibrate")
        s = state()
        lim = s["gimbal"]["axes"]["y"]["max"]
        check("un-homed travel limited to +/-30 deg", abs(lim - 30) < 1e-6, f"max {lim}")

        print("\n3. calibration")
        r = cmd(cmd="set_home")
        s = state()
        check("Set home", r["ok"] and s["cal"]["home"] and s["gimbal"]["axes"]["y"]["max"] == 85,
              f"status {s['cal']['status']}")
        cmd(cmd="cal_gnss")
        s = wait(job_done, 240, 2)
        yaw = s["cal"]["params"]["yaw"]
        check("GNSS step finds mount yaw (true +3.2)", s["job"]["state"] == "done" and abs(yaw - 3.2) < 0.6,
              f"{s['job']['msg']}")
        cmd(cmd="cal_sun")
        s = wait(job_done, 400, 2)
        fit = s["cal"]["fit"] or {}
        check("Sun scan finds the Sun", s["job"]["state"] == "done", s["job"]["msg"])
        check("status = Calibrated, Sun fit < 0.1 deg", s["cal"]["status"] == "fine" and (fit.get("rms_sun") or 9) < 0.1,
              f"{s['cal']['status']}, rms {fit.get('rms_sun')}")

        print("\n4. safety")
        cmd(cmd="go_home")
        wait(lambda s: s["gimbal"]["mode"] == "hold", 60)
        cmd(cmd="mode", mode="test")
        r = cmd(cmd="estop")
        s = state()
        check("STOP -> motors off", s["gimbal"]["mode"] == "disabled", s["gimbal"]["mode"])
        cmd(cmd="enable")
        wait(lambda s: s["gimbal"]["mode"] == "hold", 10)
        s = state()
        ax = s["gimbal"]["axes"]
        check("Return to home -> both axes 0", abs(ax["x"]["angle"]) < 0.05 and abs(ax["y"]["angle"]) < 0.05,
              f"x {ax['x']['angle']:+.3f}, y {ax['y']['angle']:+.3f} (true el {s['pointing']['el']:.1f}: home was set by eye)")

        print("\n5. auto mode")
        cmd(cmd="mode", mode="auto")
        r = cmd(cmd="auto_start")
        check("Auto starts after calibration", r["ok"], r.get("error", ""))
        s = wait(lambda s: s["auto"]["passes"], 90, 2)
        n = len(s["auto"]["passes"])
        if n:
            check("passes predicted", True, ", ".join(f"{p['name']} {p['rise'][11:16]}Z" for p in s["auto"]["passes"][:3]))
        else:
            print("  SKIP  passes predicted                               no orbit data: is the internet on?")
        s = wait(lambda s: s["auto"]["state"] in ("waiting", "slewing", "ready", "tracking"), 30)
        check("Auto running without error", s["auto"]["state"] != "error", f"{s['auto']['state']}: {s['auto']['msg']}")
        cmd(cmd="auto_stop")
        check("Auto stops", not state()["auto"]["running"])
    finally:
        app.terminate()
        app.wait(5)
    print(f"\nOVERALL: {'PASS' if all(results) else 'FAIL'}  ({sum(results)}/{len(results)})   log: {work}/app.log")
    sys.exit(0 if all(results) else 1)
