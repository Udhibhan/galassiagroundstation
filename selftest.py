#!/usr/bin/env python3
"""
selftest.py - offline checks, no hardware. Run before touching the real motors:
    python3 selftest.py

Covers CAN framing and can-utils parsing, the Pi relay (rules, watchdog STOP when the client is
killed or goes silent, Start/wake), offset-dish kinematics (Newton inverse vs closed form), Sun
position, Gaussian fit and G/T formula, the simulated drive's bench behaviour, and the controller
on the simulator: Start, wake, W latch, stop, D latch, STOP both, deadman, current limit, soft
limit (+/-80), refusal, STOP not obeyed, live speed change, Home (from OFF and from holding),
goto/track limits, exit.
The full Sun methods, peak-up, G/T, heat limit and saturation checks are in sim_acceptance.py.
"""
import json
import math
import random
import socket
import struct
import sys
import threading
import time

ok = True


def check(name, cond, detail=""):
    global ok
    ok &= bool(cond)
    print(f"  {'PASS' if cond else 'FAIL'}  {name:<48} {detail}", flush=True)


def wait(pred, timeout, step=0.05):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(step)
    return pred()


def main():
    import logging
    logging.basicConfig(level=logging.CRITICAL)      # keep the output to PASS/FAIL lines
    import config

    print("start-up imports")
    import app  # noqa: F401  (imports gimbal, sun_job, kinematics, zeroerr_protocol, erob_can_axis, clock, config)
    heavy = [m for m in ("numpy", "skyfield", "pyubx2", "uhd", "can", "serial") if m in sys.modules]
    check("app imports no skyfield/pyubx2/uhd at start-up", not [m for m in heavy if m in ("skyfield", "pyubx2", "uhd")],
          f"loaded: {heavy or 'none'}")
    import can_relay
    head = open(can_relay.__file__, encoding="utf-8").read().split("class SimBus")[0]
    imports = {l.split()[1].split(".")[0] for l in head.splitlines() if l.startswith(("import ", "from "))}
    check("can_relay.py: standard library only", imports <= set(sys.stdlib_module_names), str(sorted(imports)))
    import sdr_power
    body = open(sdr_power.__file__, encoding="utf-8").read().split('"""', 2)[2]
    code = "\n".join(l.split("#")[0] for l in body.splitlines())
    tx_words = ("get_tx_stream", "set_tx_", "tx_waveforms", "tx_samples_from_file", "uhd_siggen", "burn", "usrp_sink", "TX/RX")
    check("sdr_power.py: RX streamer on RX2 only, no TX calls", not any(w in code for w in tx_words)
          and "get_rx_stream" in code and '"RX2"' in open("config.py", encoding="utf-8").read())

    print("CAN framing")
    from zeroerr_protocol import Drive, DriveError, deg_to_counts, parse_candump_line
    check("90 deg = 0x00020000 counts", deg_to_counts(90) == 0x20000, hex(deg_to_counts(90)))
    check("1 deg/s speed = 0x000005B0", deg_to_counts(1.0) == 0x5B0, hex(deg_to_counts(1.0)))
    e = DriveError(1, 0x83, bytes([0x00, 0x27, 0x3E]))
    check("refusal 00 27 3E decoded as 0x27", e.code == 0x27, str(e))
    cur = struct.unpack(">i", bytes.fromhex("FFFFE5FD"))[0] / 1000
    check("current FF FF E5 FD = -6.66 A", abs(cur + 6.659) < 0.001, f"{cur:.3f} A")
    f = parse_candump_line("(1699999999.123456) can0 5C1#0000123E3E")
    check("candump -L line parsed", f == (0x5C1, bytes.fromhex("0000123E3E")), str(f))
    check("candump junk ignored", parse_candump_line("can0  5C1   [5]  00 00 12 3E 3E") is None)

    class FakeLink:                                 # a late write reply arrives before the read reply
        name = "fake"

        def __init__(self, frames):
            self.frames = list(frames)
            self.sent = []

        def send(self, i, d):
            self.sent.append((i, bytes(d)))

        def recv(self, timeout):
            if not self.sent:
                return None                         # nothing pending before the request is sent
            return self.frames.pop(0) if self.frames else None

    d = Drive(FakeLink([(0x5C1, b"\x3e"), (0x5C2, bytes.fromhex("000000023E")), (0x5C1, bytes.fromhex("0001885A3E"))]), 1)
    v = d.read(0x0002)
    check("read skips wrong-length / other-node frames", v == 0x1885A, hex(v))
    d = Drive(FakeLink([(0x5C1, bytes.fromhex("00273E"))]), 1)
    try:
        d.write(0x0083)
        got = None
    except DriveError as ex:
        got = ex.code
    check("write refusal [code][3E] raised", got == 0x27, f"code {got}")
    try:
        Drive(FakeLink([]), 1).write(0x00E8, 1)
        guarded = False
    except PermissionError:
        guarded = True
    check("flash/limit registers never written", guarded)

    print("Pi relay (can_relay.py) + RemoteCanLink")
    from can_relay import Relay, SimBus, check_frame, parse_candump
    check("relay: only 0x641/0x642 may be sent", check_frame(0x643, b"\x00\x43") and check_frame(0x600, b"\x00\x43")
          and check_frame(0x641, bytes.fromhex("0043")) is None)
    check("relay: writes to 0x50/52/53/E8 refused", all(check_frame(0x642, bytes.fromhex(f"{r:04X}00000001")) for r in (0x50, 0x52, 0x53, 0xE8))
          and check_frame(0x641, bytes.fromhex("010000000001")) is None)
    check("relay: candump -L line parsed", parse_candump("(1699999999.123456) can0 5C1#0000123E3E") == (1699999999.123456, 0x5C1, bytes.fromhex("0000123E3E")))
    from sim_hw import SimCanLink, SimWorld
    from zeroerr_protocol import RemoteCanLink
    SimWorld.reset()
    bus = SimBus()
    port = 7170 + random.randint(0, 50)
    relay = Relay(bus, "127.0.0.1", port)
    threading.Thread(target=relay.serve_forever, daemon=True).start()
    time.sleep(0.4)
    link = RemoteCanLink(f"127.0.0.1:{port}")
    rd1, rd2 = Drive(link, 1, timeout_s=0.6), Drive(link, 2, timeout_s=0.6)
    check("relay: node ids 1 and 2 read through TCP", rd1.node_id() == 1 and rd2.node_id() == 2, link.name)
    s0 = relay.stops
    time.sleep(2.6)
    check("relay: 0.5 s heartbeat keeps the watchdog quiet", relay.stops == s0)
    from gimbal import Gimbal
    gr = Gimbal(link)
    gr.start()
    wait(lambda: gr.mode == "ready", 8)
    gr.begin().result(3)
    wait(lambda: gr.mode != "starting", 40, 0.2)
    check("relay: Start / wake through the Pi relay", gr.mode == "on" and gr.started, f"mode {gr.mode} {gr.fault or ''}")
    gr.press("x", 1).result(3)
    wait(lambda: gr.run["x"].phase == "moving", 4)
    n0, s0 = len(bus.sent), relay.stops
    gr._stop.set()                                  # the app dies: its gimbal thread stops ...
    link.close()                                    # ... and the TCP connection drops (client killed)
    t0 = time.monotonic()
    got = wait(lambda: relay.stops > s0, 3)
    stops = [(i, d) for _, i, d in bus.sent[n0:] if d == bytes.fromhex("0084")]
    check("relay WATCHDOG: client killed -> 0x84 STOP both", got and {i for i, _ in stops} == {0x641, 0x642},
          f"after {time.monotonic() - t0:.2f} s: {[(hex(i), d.hex()) for i, d in stops]}")
    check("relay WATCHDOG: X really stopped, motor still ON", wait(lambda: not SimWorld.get().drive("x").moving(), 4)
          and SimWorld.get().drive("x").on)
    c = socket.create_connection(("127.0.0.1", port))      # a client that speaks, then goes silent
    c.sendall(b'{"op":"hb"}\n{"op":"tx","id":"641","data":"00E800000001"}\n')
    c.settimeout(1.0)
    buf = b""
    try:
        while buf.count(b"\n") < 2:
            buf += c.recv(4096)
    except socket.timeout:
        pass
    msgs = [json.loads(x) for x in buf.splitlines() if x.strip()]
    check("relay: forbidden write answered with op err", any(m.get("op") == "err" and "0x00E8" in m.get("msg", "") for m in msgs),
          str([m.get("msg") for m in msgs if m.get("op") == "err"]))
    s0, t0 = relay.stops, time.monotonic()
    got = wait(lambda: relay.stops > s0, 4)
    check("relay WATCHDOG: client silent 2.0 s -> STOP", got and 1.7 < time.monotonic() - t0 < 2.8, f"after {time.monotonic() - t0:.2f} s")
    c.close()
    relay.shutdown()
    bus.close()

    print("kinematics (offset dish) + Sun")
    import kinematics as K
    old = lambda az, el: (math.degrees(math.atan2(-math.cos(math.radians(az)), math.tan(math.radians(el)))),
                          math.degrees(math.asin(math.sin(math.radians(az)) * math.cos(math.radians(el)))))
    worst = max(max(abs(a - b) for a, b in zip(K.azel_to_axes(az, el, offset={"deg": 0}), old(az, el)))
                for az, el in ((random.uniform(0, 360), random.uniform(10, 89)) for _ in range(300)))
    check("offset 0 deg = the old formulas exactly", worst < 1e-9, f"worst {worst:.1e} deg")
    for tw in ("+X", "-X", "+Y", "-Y"):
        off = {"deg": 22.0, "toward": tw}
        w = 0.0
        for _ in range(400):
            cal = {"yaw": random.uniform(-10, 10), "tilt_n": 0.4, "tilt_e": 0.3}
            x, y = random.uniform(-80, 80), random.uniform(-80, 80)
            az, el = K.axes_to_azel(x, y, cal, off)
            x2, y2 = K.azel_to_axes(az, el, cal, off)
            w = max(w, K.angle_between(K.body_to_sky(x2, y2, cal, offset=off), K.azel_to_enu(az, el)))
        check(f"offset 22 deg {tw}: az/el -> X/Y -> az/el", w < 1e-5, f"worst {w:.1e} deg ({'Newton' if 'X' in tw else 'exact Y offset'})")
    off, d = {"deg": 22.0, "toward": "+X"}, math.radians(22.0)
    v = K.azel_to_enu(120.0, 40.0)
    yc = math.asin(v[0] / math.cos(d))                      # closed form, used only to check the Newton solver
    bx = math.atan2(-v[1], v[2]) - math.atan2(math.sin(d), math.cos(d) * math.cos(yc))
    xn, yn = K.azel_to_axes(120.0, 40.0, offset=off)
    check("+X offset: Newton = closed form", abs(xn - math.degrees(bx)) < 1e-7 and abs(yn - math.degrees(yc)) < 1e-7,
          f"x {xn:+.4f}/{math.degrees(bx):+.4f}, y {yn:+.4f}/{math.degrees(yc):+.4f}")
    xz, yz = K.azel_to_axes(0, 90, offset=off)
    check("+X offset 22: zenith needs X = -22", abs(xz + 22) < 1e-6 and abs(yz) < 1e-6, f"x {xz:+.3f} y {yz:+.3f}")
    check("unreachable beyond +/-80 refused", K.reachable(0, 3, {"yaw": 0}, {"deg": 0}) is None)
    from datetime import datetime, timezone
    from sun import sun_azel
    az, el = sun_azel(datetime(2026, 11, 6, 1, 0, tzinfo=timezone.utc), 1.2966, 103.7764, refraction=False)
    check("Sun 2026-11-06 09:00 SGT", 30 < el < 40 and 105 < az < 125, f"az {az:.2f} el {el:.2f}")

    print("Gaussian fit + G/T formula")
    from sun_job import fit_gauss, gt_from_y
    xs = [i * 0.2 - 3 for i in range(31)]
    ys = [1e-4 * (1 + 6.94 * math.exp(-4 * math.log(2) * (x - 0.37) ** 2 / 1.6 ** 2)) * (1 + random.gauss(0, 0.003)) for x in xs]
    f = fit_gauss(xs, ys)
    check("fit: peak 0.37, FWHM 1.60 (linear power)", abs(f["x0"] - 0.37) < 0.02 and abs(f["w"] - 1.6) < 0.03, f"{f['x0']:.3f} / {f['w']:.3f}")
    r = gt_from_y(10 ** 0.9, 250, 1.6)
    lam = 299792458 / 14e9
    exp_gt = 8 * math.pi * 1.380649e-23 * (10 ** 0.9 - 1) * (1 + 0.38 * (0.53 / 1.6) ** 2) / (lam ** 2 * 250e-22)
    check("G/T from Y 9 dB, test flux 250 sfu, FWHM 1.6", abs(r["gt"] - exp_gt) < 1e-9 * exp_gt and abs(r["gt_db"] - 23.40) < 0.01,
          f"{r['gt_db']:.2f} dB/K, L {r['L']:.4f}")
    check("G/T blank without flux (Y only)", gt_from_y(5.0, None, 1.6)["gt"] is None)
    from sim_hw import SimSunSensor
    SimWorld.reset()
    sens = SimSunSensor().start()
    time.sleep(0.6)
    SimWorld.get().set_option("x", "saturate", True)
    time.sleep(0.5)
    st = sens.status()
    SimWorld.get().set_option("x", "saturate", False)
    sens.close()
    check("SDR saturation (|x| > 0.7 FS) flagged", st["sat"] and sens.sat_total > 0, f"{sens.sat_total} saturated readings")

    print("simulated drive (bench behaviours)")
    SimWorld.reset()
    link = SimCanLink()
    d1 = Drive(link, 1, gap_s=0.0)
    d1.configure(1.0, accel_counts=config.MANUAL_ACCEL_COUNTS)
    time.sleep(0.6)
    d1.write(0x0100, 1)
    time.sleep(0.7)
    check("enabled + idle -> motor switches OFF", d1.state() == 0, f"state {d1.state()}")
    time.sleep(1.0)
    d1.enable_and_move(deg_to_counts(1.0))
    try:
        d1.write(0x004E, 3)
        r16 = None
    except DriveError as ex:
        r16 = ex.code
    check("0x4E while ON refused 0x16", r16 == 0x16, f"code {r16}")
    try:
        d1.write(0x0083)
        r15 = None
    except DriveError as ex:
        r15 = ex.code
    check("START while moving refused 0x15", r15 == 0x15, f"code {r15}")
    check("current while moving ~2.5 A", 2.0 < abs(d1.current_a()) < 3.0, f"{d1.current_a():+.2f} A")
    time.sleep(2.5)
    check("after a move: ON and holding (state 4)", d1.state() == 4, f"state {d1.state()}")
    check("holding current ~0.9 A, temp, 47.5 V", 0.7 < abs(d1.current_a()) < 1.1 and 20 < d1.temperature_c() < 60
          and 47 < d1.bus_volts() < 48, f"{d1.current_a():+.2f} A, {d1.temperature_c()} C, {d1.bus_volts():.1f} V")
    d1.write(0x0100, 0)
    time.sleep(0.2)
    d1.enable_and_move(deg_to_counts(1.0))
    time.sleep(0.4)
    check("START < 0.5 s after motor-off -> drops out", d1.state() == 0, f"state {d1.state()}")

    print("manual controller on simulated drives")
    SimWorld.reset()
    world = SimWorld.get()
    g = Gimbal(SimCanLink())
    g.start()
    beating = {"on": True}

    def beater():                                  # the web page's heartbeat
        while beating["on"] is not None:
            if beating["on"]:
                g.beat()
            time.sleep(0.5)
    threading.Thread(target=beater, daemon=True).start()

    wait(lambda: g.mode == "ready", 5)
    s = g.snapshot()["axes"]
    check("both nodes found, values read, nothing moved", g.mode == "ready" and s["x"]["state"] == 0 and s["x"]["temp"] is not None,
          f"x {s['x']['angle']:+.3f} ({s['x']['raw']}), y {s['y']['angle']:+.3f}, {g.bus_v} V")
    check("angle = wrap(raw - HOME)", abs(s["x"]["angle"] - 0.30) < 0.01 and abs(s["y"]["angle"] + 0.20) < 0.01,
          f"HOME {config.HOME_COUNTS}")
    try:
        g.press("x", 1).result(3)
        refused = False
    except RuntimeError:
        refused = True
    check("keys refused before Start", refused)
    x0, y0 = g.angles()
    g.begin().result(3)
    wait(lambda: g.mode != "starting", 30, 0.2)
    sx, sy = g.run["x"].state, g.run["y"].state
    check("Start: setup, wake X then Y -> both holding", g.mode == "on" and g.started and sx == 4 and sy == 4,
          f"mode {g.mode}, states {sx}/{sy}, {g.fault or ''}")
    x1, y1 = g.angles()
    check("wake returns to the start position", abs(x1 - x0) < 0.01 and abs(y1 - y0) < 0.01, f"x {x0:+.3f}->{x1:+.3f}")

    g.press("x", 1).result(3)
    check("W: in flight (starting)", g.run["x"].phase in ("starting", "moving"), g.run["x"].phase)
    check("W: drive reports state 2 -> moving (green)", wait(lambda: g.run["x"].phase == "moving", 3), g.run["x"].phase)
    time.sleep(1.5)
    g.press("x", 1).result(3)
    check("W again: STOP sent", g.run["x"].phase == "stopping", g.run["x"].phase)
    check("X stops, holds, motor stays ON", wait(lambda: g.run["x"].phase == "idle", 5) and g.run["x"].state in (3, 4),
          f"state {g.run['x'].state}, {g.angles()[0]:+.3f} deg")

    g.press("x", 1).result(3)                       # S while W green: stops, does not reverse
    wait(lambda: g.run["x"].phase == "moving", 3)
    g.press("x", -1).result(3)
    check("S while W green: X stops (no reverse)", g.run["x"].phase == "stopping" and g.run["x"].pending == 0)
    g.press("x", -1).result(3)                      # press S again while still stopping -> queued
    check("S again: reverses after the stop", wait(lambda: g.run["x"].phase == "moving" and g.run["x"].latch == -1, 8),
          f"{g.run['x'].phase} {g.run['x'].latch}")

    g.press("y", 1).result(3)
    check("D latch: X and Y both moving", wait(lambda: g.run["y"].phase == "moving", 3) and g.run["x"].phase == "moving")
    g.stop_all("selftest").result(3)
    check("STOP both", wait(lambda: all(r.phase == "idle" for r in g.run.values()), 6) and g.mode == "on",
          f"{[r.phase for r in g.run.values()]}, mode {g.mode}")

    g.press("y", -1).result(3)
    wait(lambda: g.run["y"].phase == "moving", 3)
    beating["on"] = False
    t0 = time.monotonic()
    stopped = wait(lambda: g.run["y"].phase in ("stopping", "idle"), 4)
    check("deadman: no heartbeat 2 s -> STOP", stopped and 1.3 < time.monotonic() - t0 < 3.0, f"after {time.monotonic() - t0:.1f} s")
    beating["on"] = True
    wait(lambda: g.run["y"].phase == "idle", 6)

    g.press("x", 1).result(3)
    wait(lambda: g.run["x"].phase == "moving", 3)
    world.set_option("x", "fake_current_a", 16.0)
    check("current limit 15 A on X -> stop + warning", wait(lambda: g.run["x"].phase != "moving", 3)
          and "limit" in g.run["x"].msg, g.run["x"].msg)
    world.set_option("x", "fake_current_a", None)
    wait(lambda: g.run["x"].phase == "idle", 6)

    world.set_option("y", "set_deg", 79.0)
    time.sleep(1.2)
    g.press("y", 1).result(3)
    check("soft limit: Y runs to +80 and un-latches", wait(lambda: g.run["y"].phase == "idle" and g.run["y"].latch == 0, 8)
          and "+80" in g.run["y"].msg and abs(g.angles()[1] - 80) < 0.01, f"{g.angles()[1]:+.3f}: {g.run['y'].msg}")
    seq = g.run["y"].refusal["seq"]
    g.press("y", 1).result(3)
    check("at the limit: same key refused (red)", g.run["y"].refusal["seq"] == seq + 1, g.run["y"].refusal["msg"])

    world.set_option("x", "refuse_start", 0x27)
    seq = g.run["x"].refusal["seq"]
    g.press("x", -1).result(3)
    r = g.run["x"].refusal
    check("refused START -> red + error code shown", r["seq"] == seq + 1 and r["dir"] == -1 and "0x27" in r["msg"]
          and g.run["x"].phase == "idle", r["msg"])

    res = g.set_speed(1.5).result(3)
    check("speed 1.5: live 0x8A write tried", res == "live" and g.run["x"].speed == 1.5, res)
    world.set_option("x", "refuse_live_speed", True)
    res = g.set_speed(0.5).result(3)
    check("live speed refused -> needs Start", res == "needs start" and g.speed_pending == 0.5, res)
    world.set_option("x", "refuse_live_speed", False)

    world.set_option("x", "ignore_stop", True)
    g.press("x", -1).result(3)
    wait(lambda: g.run["x"].phase == "moving", 3)
    g.press("x", -1).result(3)
    faulted = wait(lambda: g.mode == "fault", 8)
    check("STOP not obeyed -> X motor OFF + fault", faulted and world.drive("x").on is False and not g.started, g.fault or "")
    world.set_option("x", "ignore_stop", False)

    g.begin().result(3)
    wait(lambda: g.mode != "starting", 30, 0.2)
    check("Start again clears the fault", g.mode == "on" and g.fault is None, g.mode)

    print("Home, goto, track")
    g.home().result(3)
    check("Home: button state 'homing'", g.snapshot()["homing"])
    wait(lambda: g.mode != "homing", 200, 0.2)
    hr = g.home_result or {}
    check("Home from holding: 0/0, motors stay ON", hr.get("ok") and max(abs(v) for v in hr["err_counts"].values()) <= 2
          and world.drive("x").on and world.drive("y").on, hr.get("msg"))
    try:
        g.goto(81.0, 0.0, "test").result(3)
        refused = False
    except ValueError:
        refused = True
    check("goto beyond +/-80 refused", refused)
    act = g.goto(3.0, -2.0, "test goto").result(3)
    check("goto 3/-2", g.wait_act(act, 30) and abs(g.angles()[0] - 3) < 0.03 and abs(g.angles()[1] + 2) < 0.03, str(g.angles()))
    g.track(lambda t: (83.0, -2.0), "clip test").result(3)
    wait(lambda: g.act is None or g.act["clipped"], 3)
    clipped = g.act is not None and g.act["clipped"]
    g.stop_act("test").result(3)
    wait(lambda: all(r.phase == "idle" for r in g.run.values()), 8)
    check("track target beyond +/-80 clipped and reported", clipped)
    g.press("y", -1).result(3)
    wait(lambda: g.run["y"].phase == "moving", 3)
    res = g.shutdown()
    check("exit: STOP both, motors stay ON", world.drive("x").on and world.drive("y").on and not world.drive("y").moving(), res)
    beating["on"] = None

    SimWorld.reset()
    world = SimWorld.get()
    g = Gimbal(SimCanLink())
    g.start()
    wait(lambda: g.mode == "ready", 5)
    world.set_option("x", "set_deg", 6.0)
    world.set_option("y", "set_deg", 0.3)          # < 0.5 deg from motor OFF: +1 deg beyond, then back
    g.home().result(3)
    wait(lambda: g.mode != "homing", 60, 0.2)
    hr = g.home_result or {}
    check("Home from motors OFF: setup -> 1 s -> move", hr.get("ok") and g.mode == "on" and g.started
          and abs(g.angles()[0]) < 0.002 and abs(g.angles()[1]) < 0.002, hr.get("msg"))
    g.shutdown()

    # Disabled (test stage): geometry, Sun position, old controller (goto/jog/track) and
    # calibration-solver checks. They need numpy/kinematics/sun and the old Gimbal.
    # Old code is in legacy/selftest_v3_full.py.

    print("\nOVERALL:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
