#!/usr/bin/env python3
"""
sim_acceptance.py - end-to-end test of the field-test station on the SIMULATOR, through the same web API
the page uses (app.py --sim --sim-clock 2026-11-09T09:30:00+08:00). No hardware is opened. Takes ~25 min
(the Sun steps run in real time at 1 deg/s).

    python3 sim_acceptance.py            # everything
    python3 sim_acceptance.py --quick    # relay + manual + Home only (~2 min)
Flow: relay watchdog (can_relay.py --sim as its own process, client killed) -> page -> Start/wake ->
W latch/stop -> Home -> Sun refusals -> Sun method 2 (blind ring scan, GNSS off) -> peak-up FWHM ->
G/T from a test flux -> LNB heat limit auto move-off -> saturation warning -> Sun method 1 (GNSS + peak-up)
-> Abort near the Sun moves off -> Home.
Hidden truth in the simulator: mount yaw +4.0 deg, base tilt 0.5 deg, boresight offset from config.py.
"""
import json
import math
import re
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
PORT = 8099
BASE = f"http://127.0.0.1:{PORT}"
SIM_CLOCK = "2026-11-09T09:30:00+08:00"
TRUE_YAW = 4.0
results = []


def state():
    return json.load(urllib.request.urlopen(BASE + "/api/state", timeout=10))


def cmd(**c):
    req = urllib.request.Request(BASE + "/api/cmd", json.dumps(c).encode(), {"Content-Type": "application/json"})
    try:
        return json.load(urllib.request.urlopen(req, timeout=15))
    except urllib.error.HTTPError as e:
        return json.load(e)


def check(name, ok, detail=""):
    results.append(bool(ok))
    print(f"  {'PASS' if ok else 'FAIL'}  {name:<52} {detail}", flush=True)


def wait(pred, timeout, step=0.3):
    end = time.time() + timeout
    while time.time() < end:
        s = state()
        if pred(s):
            return s
        time.sleep(step)
    return state()


def ax(s, k):
    return s["gimbal"]["axes"][k]


def sun_done(s):
    return not s["sun"]["busy"] and s["sun"]["prog"]["state"] != "running"


def follow(label, timeout):
    """Wait for the running Sun action, printing its steps."""
    last, end = None, time.time() + timeout
    while time.time() < end:
        s = state()
        p = s["sun"]["prog"]
        if p["step"] != last:
            last = p["step"]
            print(f"        {label}: {p['step']} - {p['msg']}", flush=True)
        if sun_done(s):
            return s
        time.sleep(1.0)
    return state()


def relay_watchdog_test():
    print("\n0. Pi relay watchdog (can_relay.py --sim in its own process)")
    rport = 7190
    log = tempfile.NamedTemporaryFile("w+", suffix="_relay.log", delete=False)
    rp = subprocess.Popen([sys.executable, "-u", str(HERE / "can_relay.py"), "--sim", "--host", "127.0.0.1", "--port", str(rport)],
                          cwd=str(HERE), stdout=log, stderr=subprocess.STDOUT)
    try:
        time.sleep(1.5)
        cli = subprocess.Popen([sys.executable, "-c", (
            "import socket,time\n"
            f"s=socket.create_connection(('127.0.0.1',{rport}))\n"
            "s.sendall(b'{\"op\":\"tx\",\"id\":\"641\",\"data\":\"0043\"}\\n')\n"
            "while True:\n s.sendall(b'{\"op\":\"hb\"}\\n'); time.sleep(0.5)\n")])
        time.sleep(2.5)
        log.flush()
        before = open(log.name).read()
        cli.kill()                                          # the laptop app dies
        cli.wait(5)
        t0 = time.time()
        while time.time() - t0 < 4 and "WATCHDOG" not in open(log.name).read():
            time.sleep(0.1)
        after = open(log.name).read()
        check("relay: heartbeating client -> no STOP", "WATCHDOG" not in before)
        check("relay: client process killed -> 0x84 STOP both nodes", "WATCHDOG" in after and "0x84 STOP to node 1 (X) and node 2 (Y)" in after,
              f"after {time.time() - t0:.1f} s")
        s = socket.create_connection(("127.0.0.1", rport))
        s.sendall(b'{"op":"tx","id":"642","data":"005300000001"}\n')
        s.settimeout(2)
        got = b""
        try:
            while b"err" not in got:
                got += s.recv(4096)
        except socket.timeout:
            pass
        check("relay: write to 0x0053 refused", b"0x0053" in got, got.decode(errors="replace").strip()[-80:])
        s.close()
    finally:
        rp.terminate()
        try:
            rp.wait(5)
        except subprocess.TimeoutExpired:
            rp.kill()


def main():
    quick = "--quick" in sys.argv
    relay_watchdog_test()
    work = tempfile.mkdtemp(prefix="gs_sim_")
    print(f"\nsimulator working folder: {work}")
    log = open(Path(work) / "app.log", "w")
    app = subprocess.Popen([sys.executable, str(HERE / "app.py"), "--sim", "--sim-clock", SIM_CLOCK, "--port", str(PORT)],
                           cwd=work, stdout=log, stderr=subprocess.STDOUT)
    beat = {"on": True, "run": True}

    def heartbeat():                                   # what the page does every 0.5 s
        while beat["run"]:
            if beat["on"]:
                try:
                    cmd(cmd="heartbeat")
                except Exception:
                    pass
            time.sleep(0.5)
    try:
        for _ in range(60):
            try:
                state()
                break
            except Exception:
                time.sleep(0.5)
        threading.Thread(target=heartbeat, daemon=True).start()

        print("\n1. page and start-up")
        page = urllib.request.urlopen(BASE + "/", timeout=5).read().decode()
        live = re.sub(r'<style media="not all".*?</style>', "", page, flags=re.S).split('<template id="disabled-test-stage-ui">')[0]
        check("page: header chips + STOP", all(f'id="{i}"' in live for i in ("c-pi", "c-mot", "c-sdr", "c-gnss", "c-volt", "stop")))
        check("page: Manual tab (WASD, Start, Home)", all(f'id="{i}"' in live for i in ("kW", "kA", "kS", "kD", "start", "home")))
        check("page: Sun / G/T tab (methods, plot, sky, flux, G/T)", all(f'id="{i}"' in live for i in ("sunstart", "sunabort", "plot", "sky", "flux", "gtbtn"))
              and 'value="gnss"' in live and 'value="blind"' in live)
        check("page: Calibrate/Auto/sky dome commented out", 'data-mode="calibrate"' not in live and 'id="dome"' not in live
              and 'id="disabled-test-stage-ui"' in page)
        s = wait(lambda s: s["gimbal"]["mode"] == "ready", 20)
        check("drives found, nothing moved; chips", s["gimbal"]["mode"] == "ready" and ax(s, "x")["state"] == 0 and s["relay"]["connected"]
              and s["sdr"]["available"] and s["gnss"]["chip"] == "RTK FIXED",
              f"relay {s['relay']['kind']}, SDR {s['sdr']['db']:.1f} dBFS, GNSS {s['gnss']['chip']}, {s['gimbal']['bus_v']:.1f} V")
        su = s["sun"]
        check("Sun at the sim clock (09:30 SGT)", 30 < su["el"] < 45 and 100 < su["az"] < 125 and su["site"]["source"] == "GNSS",
              f"az {su['az']:.2f} el {su['el']:.2f}, site {su['site']['lat']:.4f},{su['site']['lon']:.4f} ({su['site']['source']})")
        r = cmd(cmd="sun_start", method="blind")
        check("Sun refused while motors not holding", not r["ok"] and "holding" in r.get("error", ""), r.get("error", "")[:90])

        print("\n2. Start (setup + wake X, then Y)")
        r = cmd(cmd="start")
        check("Start needs the support-the-load confirmation", not r["ok"])
        cmd(cmd="start", confirm=True)
        s = wait(lambda s: s["gimbal"]["mode"] != "starting", 40)
        check("both motors ON and holding", s["gimbal"]["mode"] == "on" and ax(s, "x")["state"] == 4 and ax(s, "y")["state"] == 4
              and s["gimbal"]["holding"], f"{s['gimbal']['mode']} {s['gimbal']['fault'] or ''}")

        print("\n3. W latch / stop, Home")
        r = cmd(cmd="key", axis="x", dir=1)
        s = wait(lambda s: ax(s, "x")["phase"] == "moving", 4)
        check("W green: X moving + (drive state 2)", ax(s, "x")["phase"] == "moving" and ax(s, "x")["state"] == 2)
        time.sleep(2)
        cmd(cmd="key", axis="x", dir=1)
        s = wait(lambda s: ax(s, "x")["phase"] == "idle", 6)
        check("W again: X stops and holds (motor ON)", ax(s, "x")["phase"] == "idle" and ax(s, "x")["state"] == 4 and ax(s, "x")["angle"] > 1,
              f"X {ax(s, 'x')['angle']:+.3f}")
        cmd(cmd="key", axis="y", dir=-1)
        wait(lambda s: ax(s, "y")["phase"] == "moving", 4)
        time.sleep(1.5)
        cmd(cmd="stop", why="Space key")
        wait(lambda s: ax(s, "y")["phase"] == "idle", 6)
        r = cmd(cmd="home")
        s = state()
        check("Home: button green while homing", r["ok"] and s["gimbal"]["homing"])
        s = wait(lambda s: not s["gimbal"]["homing"] and s["gimbal"]["mode"] != "homing", 60)
        hr = s["gimbal"]["home_result"] or {}
        check("Home: both at 0/0, final error in counts", hr.get("ok") and abs(ax(s, "x")["angle"]) < 0.002 and abs(ax(s, "y")["angle"]) < 0.002,
              hr.get("msg", ""))
        if quick:
            return

        print("\n4. Sun method 2: blind elevation-ring scan (GNSS off)")
        cmd(cmd="sim", gnss="off")
        s = wait(lambda s: s["gnss"]["chip"] == "not connected", 5)
        check("method 1 disabled with a reason when no GNSS", s["sun"]["gnss_reason"] and "GNSS" in s["sun"]["gnss_reason"], s["sun"]["gnss_reason"])
        r = cmd(cmd="sun_start", method="gnss")
        check("method 1 refused without GNSS", not r["ok"], r.get("error", "")[:80])
        r = cmd(cmd="sun_start", method="blind", opts={"heat_limit_s": 120})
        check("method 2 started", r["ok"], r.get("error", ""))
        s = wait(lambda s: (s["sun"]["result"] or {}).get("peak_ok") or sun_done(s), 2400, 2)
        s = follow("method 2", 5) if not (s["sun"]["result"] or {}).get("peak_ok") else s
        res = s["sun"]["result"] or {}
        check("method 2: Sun found without GNSS (ring peak)", res.get("psi") is not None, f"ψ ring {res.get('psi', float('nan')):+.2f}°, "
              f"state {s['sun']['prog']['state']}: {s['sun']['prog']['msg']}")
        if res.get("psi") is None:
            return
        check("method 2: refined yaw = hidden +4.0 within 0.5", abs(res.get("psi_ref", 99) - TRUE_YAW) < 0.5,
              f"ψ refined {res.get('psi_ref', float('nan')):+.3f}° (ring {res['psi']:+.2f}°)")
        check("peak-up FWHM ≈ 1.6° (X and Y)", abs(res.get("fwhm_x", 0) - 1.6) < 0.15 and abs(res.get("fwhm_y", 0) - 1.6) < 0.15,
              f"X {res.get('fwhm_x', 0):.3f}° / Y {res.get('fwhm_y', 0):.3f}°, rise +{res.get('rise_db', 0):.2f} dB, "
              f"dX {res.get('dX', 0):+.3f} dY {res.get('dY', 0):+.3f}")
        check("peak rise ≈ +9 dB over cold sky", 8.0 < res.get("rise_db", 0) < 9.5, f"+{res.get('rise_db', 0):.2f} dB")
        s = wait(lambda s: s["sun"]["prog"]["step"] == "tracking the Sun" or sun_done(s), 30, 0.5)
        check("TRACK after peak-up", s["sun"]["prog"]["step"] == "tracking the Sun", s["sun"]["prog"]["step"])
        time.sleep(4)
        s = state()
        on = s["sun"]["prog"].get("heat") or {}
        check("tracking: on the Sun (heat timer running)", on.get("near") and s["sdr"]["db"] > -34, f"{s['sdr']['db']:.1f} dBFS, {on}")
        r = cmd(cmd="sun_abort", why="test: end track")
        s = wait(sun_done, 120, 1)
        x, y = ax(s, "x")["angle"], ax(s, "y")["angle"]
        check("Abort while on the Sun -> STOP + moved 10° off", s["sun"]["prog"]["state"] == "aborted" and "moved 10" in s["sun"]["prog"]["msg"]
              and s["sdr"]["db"] < -41.5, f"{s['sun']['prog']['msg']}, now {s['sdr']['db']:.1f} dBFS")

        print("\n5. G/T (Y-factor) with a test flux")
        cmd(cmd="flux", sfu=250)
        r = cmd(cmd="sun_gt")
        check("Measure G/T started (after a successful peak-up)", r["ok"], r.get("error", ""))
        s = follow("G/T", 300)
        gt = s["sun"]["gt"] or {}
        fw = (s["sun"]["result"] or {}).get("fwhm")
        if gt.get("Y"):
            lam = 299792458 / 14e9
            L = 1 + 0.38 * (0.53 / fw) ** 2
            exp_gt = 10 * math.log10(8 * math.pi * 1.380649e-23 * (gt["Y"] - 1) * L / (lam ** 2 * 250e-22))
        else:
            exp_gt = None
        check("Y ≈ 9 dB (sim Sun +9 dB)", gt.get("Y_db") and 8.3 < gt["Y_db"] < 9.3, f"Y {gt.get('Y_db')} dB")
        check("G/T computed from flux 250 sfu", gt.get("gt_db") is not None and exp_gt is not None and abs(gt["gt_db"] - exp_gt) < 1e-6,
              f"G/T {gt.get('gt_db')} dB/K, L {gt.get('L')}, θ {fw}")
        cmd(cmd="flux", sfu=200)
        s = state()
        g2 = s["sun"]["gt"] or {}
        check("G/T recomputed when the flux is edited", g2.get("gt_db") and abs(g2["gt_db"] - (gt["gt_db"] + 10 * math.log10(250 / 200))) < 1e-6,
              f"{g2.get('gt_db')} dB/K at 200 sfu")
        logn = s["sun"]["log"]
        jp = Path(work) / "logs" / f"{logn}.json"
        cp = Path(work) / "logs" / f"{logn}.csv"
        ok_log = jp.exists() and cp.exists()
        d = json.loads(jp.read_text()) if jp.exists() else {}
        check("logs/sun_<time>.json + .csv saved (scans, fits, Y, G/T)", ok_log and "find" in d and "gt" in d and d["gt"].get("gt_db")
              and len(cp.read_text().splitlines()) > 500 and d["find"].get("cuts"), f"{logn}, {len(cp.read_text().splitlines()) if ok_log else 0} rows")
        check("logs/power_<date>.csv (every reading)", any((Path(work) / "logs").glob("power_*.csv")))

        print("\n6. LNB heat rule + saturation")
        r = cmd(cmd="sun_track", opts={"heat_limit_s": 15})
        check("Track Sun (heat limit set to 15 s for the test)", r["ok"], r.get("error", ""))
        t0 = time.time()
        s = wait(lambda s: (s["sun"]["prog"].get("heat") or {}).get("near"), 90, 0.5)
        t_on = time.time()
        cmd(cmd="sim", saturate=True)
        s = follow("track", 120)
        cmd(cmd="sim", saturate=False)
        p = s["sun"]["prog"]
        check("heat limit: auto move 10° off + says why", "heat" in p["msg"].lower() and "moved 10" in p["msg"] and s["sdr"]["db"] < -41.5,
              f"after {time.time() - t_on:.0f} s on the Sun: {p['msg']}")
        cmd(cmd="sim", saturate=True)
        s = wait(lambda s: s["sdr"]["sat"], 5, 0.3)
        check("saturation: SDR chip / banner warn 'reduce gain'", s["sdr"]["sat"])
        r = cmd(cmd="sun_gt", opts={"gt_avg_s": 4})
        s = follow("G/T saturated", 240)
        cmd(cmd="sim", saturate=False)
        gt = s["sun"]["gt"] or {}
        check("saturation: G/T marked INVALID + 'reduce gain'", gt.get("valid") is False and any("reduce gain" in w for w in s["sun"]["prog"]["warnings"]),
              str(s["sun"]["prog"]["warnings"])[:100])

        print("\n7. Sun method 1: GNSS true north + peak-up")
        cmd(cmd="sim", gnss="fixed")
        s = wait(lambda s: s["gnss"]["chip"] == "RTK FIXED" and not s["sun"]["gnss_reason"], 5)
        r = cmd(cmd="sun_start", method="gnss", opts={"gnss_avg_s": 20})
        check("method 1 started (GNSS RTK FIXED)", r["ok"], r.get("error", ""))
        s = wait(lambda s: (s["sun"]["result"] or {}).get("peak_ok") or sun_done(s), 1200, 2)
        res = s["sun"]["result"] or {}
        check("method 1: GNSS yaw = hidden +4.0 within 0.5", res.get("psi") is not None and abs(res["psi"] - TRUE_YAW) < 0.5,
              f"ψ GNSS {res.get('psi', float('nan')):+.3f}° (heading {res.get('gnss_heading', float('nan')):.2f}°)")
        check("method 1: peak-up done, refined yaw within 0.5", res.get("peak_ok") and abs(res.get("psi_ref", 99) - TRUE_YAW) < 0.5,
              f"ψ refined {res.get('psi_ref', float('nan')):+.3f}°, FWHM {res.get('fwhm_x', 0):.2f}/{res.get('fwhm_y', 0):.2f}°, "
              f"dX {res.get('dX', 0):+.3f} dY {res.get('dY', 0):+.3f}")
        s = wait(lambda s: s["sun"]["prog"]["step"] == "tracking the Sun" or sun_done(s), 30, 0.5)
        time.sleep(3)
        cmd(cmd="stop", why="Space key")                         # global STOP during a Sun step
        s = wait(sun_done, 120, 1)
        check("STOP during TRACK -> stopped, then moved 10° off", s["sun"]["prog"]["state"] == "aborted" and "moved 10" in s["sun"]["prog"]["msg"],
              s["sun"]["prog"]["msg"])

        print("\n8. back home")
        cmd(cmd="home")
        s = wait(lambda s: not s["gimbal"]["homing"] and s["gimbal"]["mode"] != "homing", 200, 1)
        check("Home after the Sun steps", (s["gimbal"]["home_result"] or {}).get("ok"), (s["gimbal"]["home_result"] or {}).get("msg", ""))
        check("no motor switched off by the software", ax(s, "x")["state"] != 0 and ax(s, "y")["state"] != 0)
    finally:
        beat["run"] = False
        app.terminate()
        try:
            app.wait(10)
        except subprocess.TimeoutExpired:
            app.kill()
        print(f"\nOVERALL: {'PASS' if results and all(results) else 'FAIL'}  ({sum(results)}/{len(results)})   log: {work}/app.log")
        sys.exit(0 if results and all(results) else 1)


# DISABLED (test stage): the v3 acceptance run (jog, HOME, GNSS + Sun calibration, Auto tracking).
# def old_state():
#     return json.load(urllib.request.urlopen(BASE + "/api/state", timeout=5))
#
#
# def old_cmd(**c):
#     req = urllib.request.Request(BASE + "/api/cmd", json.dumps(c).encode(), {"Content-Type": "application/json"})
#     try:
#         return json.load(urllib.request.urlopen(req, timeout=10))
#     except urllib.error.HTTPError as e:
#         return json.load(e)
#
#
# def old_check(name, ok, detail=""):
#     results.append(ok)
#     print(f"  {'PASS' if ok else 'FAIL'}  {name:<46} {detail}", flush=True)
#
#
# def old_wait(pred, timeout, step=0.5):
#     end = time.time() + timeout
#     while time.time() < end:
#         s = state()
#         if pred(s):
#             return s
#         time.sleep(step)
#     return state()
#
#
# def old_job_done(s):
#     return s["job"] and s["job"]["state"] != "running"
#
#
# def old_main():
#     work = tempfile.mkdtemp(prefix="gs_sim_")
#     if len(sys.argv) > 1:                              # optional: an omm_cache.json to use offline
#         shutil.copy(sys.argv[1], Path(work) / "omm_cache.json")
#     print(f"simulator working folder: {work}")
#     app = subprocess.Popen([sys.executable, str(HERE / "app.py"), "--sim", "--port", str(PORT),
#                             "--sim-clock", "2026-11-06T09:00:00+08:00"], cwd=work,
#                            stdout=open(Path(work) / "app.log", "w"), stderr=subprocess.STDOUT)
#     try:
#         for _ in range(40):
#             try:
#                 state()
#                 break
#             except Exception:
#                 time.sleep(0.5)
#         print("\n1. start-up")
#         s = wait(lambda s: s["gimbal"]["mode"] == "disabled", 15)
#         check("simulated drives answer, motors off", s["gimbal"]["mode"] == "disabled", s["gimbal"]["mode"])
#         r = cmd(cmd="auto_start")
#         check("Auto refuses before calibration", not r["ok"], r.get("error", ""))
#
#         print("\n2. jog before HOME")
#         cmd(cmd="enable")
#         s = wait(lambda s: s["gimbal"]["mode"] == "hold", 10)
#         check("enable -> hold", s["gimbal"]["mode"] == "hold", s["gimbal"]["mode"])
#         y0 = s["gimbal"]["axes"]["y"]["angle"]
#         t_end = time.time() + 2.0
#         while time.time() < t_end:
#             cmd(cmd="jog", x=0, y=1)
#             time.sleep(0.15)
#         cmd(cmd="jog", x=0, y=0)
#         s = wait(lambda s: s["gimbal"]["mode"] == "hold", 5)
#         y1 = s["gimbal"]["axes"]["y"]["angle"]
#         time.sleep(1.0)
#         y2 = state()["gimbal"]["axes"]["y"]["angle"]
#         check("hold D moves Y east, release stops", 0.5 < y1 - y0 < 2.0 and abs(y2 - y1) < 0.05,
#               f"{y0:+.2f} -> {y1:+.2f} -> {y2:+.2f}")
#         r = cmd(cmd="mode", mode="test")
#         cmd(cmd="jog", x=1, y=0)                       # one heartbeat, then silence
#         started = wait(lambda s: s["gimbal"]["mode"] == "jog", 3, 0.1)["gimbal"]["mode"] == "jog"
#         s = wait(lambda s: s["gimbal"]["mode"] == "hold", 5)
#         check("deadman: lost heartbeat -> stops", started and s["gimbal"]["mode"] == "hold",
#               f"started={started}, now {s['gimbal']['mode']}")
#         cmd(cmd="mode", mode="calibrate")
#         s = state()
#         lim = s["gimbal"]["axes"]["y"]["max"]
#         check("un-homed travel limited to +/-30 deg", abs(lim - 30) < 1e-6, f"max {lim}")
#
#         print("\n3. calibration")
#         r = cmd(cmd="set_home")
#         s = state()
#         check("Set home", r["ok"] and s["cal"]["home"] and s["gimbal"]["axes"]["y"]["max"] == 85,
#               f"status {s['cal']['status']}")
#         cmd(cmd="cal_gnss")
#         s = wait(job_done, 240, 2)
#         yaw = s["cal"]["params"]["yaw"]
#         check("GNSS step finds mount yaw (true +3.2)", s["job"]["state"] == "done" and abs(yaw - 3.2) < 0.6,
#               f"{s['job']['msg']}")
#         cmd(cmd="cal_sun")
#         s = wait(job_done, 400, 2)
#         fit = s["cal"]["fit"] or {}
#         check("Sun scan finds the Sun", s["job"]["state"] == "done", s["job"]["msg"])
#         check("status = Calibrated, Sun fit < 0.1 deg", s["cal"]["status"] == "fine" and (fit.get("rms_sun") or 9) < 0.1,
#               f"{s['cal']['status']}, rms {fit.get('rms_sun')}")
#
#         print("\n4. safety")
#         cmd(cmd="go_home")
#         wait(lambda s: s["gimbal"]["mode"] == "hold", 60)
#         cmd(cmd="mode", mode="test")
#         r = cmd(cmd="estop")
#         s = state()
#         check("STOP -> motors off", s["gimbal"]["mode"] == "disabled", s["gimbal"]["mode"])
#         cmd(cmd="enable")
#         wait(lambda s: s["gimbal"]["mode"] == "hold", 10)
#         s = state()
#         ax = s["gimbal"]["axes"]
#         check("Return to home -> both axes 0", abs(ax["x"]["angle"]) < 0.05 and abs(ax["y"]["angle"]) < 0.05,
#               f"x {ax['x']['angle']:+.3f}, y {ax['y']['angle']:+.3f} (true el {s['pointing']['el']:.1f}: home was set by eye)")
#
#         print("\n5. auto mode")
#         cmd(cmd="mode", mode="auto")
#         r = cmd(cmd="auto_start")
#         check("Auto starts after calibration", r["ok"], r.get("error", ""))
#         s = wait(lambda s: s["auto"]["passes"], 90, 2)
#         n = len(s["auto"]["passes"])
#         if n:
#             check("passes predicted", True, ", ".join(f"{p['name']} {p['rise'][11:16]}Z" for p in s["auto"]["passes"][:3]))
#         else:
#             print("  SKIP  passes predicted                               no orbit data: is the internet on?")
#         s = wait(lambda s: s["auto"]["state"] in ("waiting", "slewing", "ready", "tracking"), 30)
#         check("Auto running without error", s["auto"]["state"] != "error", f"{s['auto']['state']}: {s['auto']['msg']}")
#         cmd(cmd="auto_stop")
#         check("Auto stops", not state()["auto"]["running"])
#     finally:
#         app.terminate()
#         app.wait(5)
#     print(f"\nOVERALL: {'PASS' if all(results) else 'FAIL'}  ({sum(results)}/{len(results)})   log: {work}/app.log")
#     sys.exit(0 if all(results) else 1)



if __name__ == "__main__":
    main()
