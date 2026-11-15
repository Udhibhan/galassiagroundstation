"""
test_job.py - Test mode: press "Run test" and every external part is checked, one step after the other.

  1. link    Pi relay / CAN link answers, both drives found (no motion)
  2. motors  each axis HOME -> +d -> -d -> HOME, reports reached, error, current, time (moves the dish)
  3. sdr     B206mini readings for a few seconds: rate, mean/min/max dBFS, saturation (receive only)
  4. gnss    follows the receiver live (connected, 3D fix, RTK float, RTK FIXED, satellite count over
             time), then averages the RTK FIXED heading to get the mount yaw psi
  5. north   points the RF beam at true north, 45 deg elevation (psi from the GNSS heading) and checks it

Runs in its own worker thread and never touches CAN. It only gives the gimbal thread goto targets
(gimbal.goto) and reads the SDR power meter (receive only), like sun_job.SunSession.
The page deadman (no heartbeat for 2 s) and STOP stop it like any other goto.
"""
import copy
import logging
import math
import threading
import time

import clock
import config
import kinematics as K
from sun import sun_azel

log = logging.getLogger(__name__)

STEPS = [("link", "Link and drives"), ("motors", "Motors: move each axis and return to HOME"),
         ("sdr", "SDR readings"), ("gnss", "GNSS: find satellites, wait for RTK FIXED"),
         ("north", "Point the beam at true north, 45° up")]
GNSS_STAGES = ["no receiver", "no fix", "3D fix", "RTK float", "RTK FIXED"]


class Abort(Exception):
    pass


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class TestSession:
    def __init__(self, gimbal, meter_fn, gnss, site_fn, relay_fn):
        self.g = gimbal
        self.meter_fn = meter_fn             # returns PowerMeter or None
        self.gnss = gnss
        self.site_fn = site_fn               # returns (lat, lon, source)
        self.relay_fn = relay_fn             # returns {"connected": bool, "name": str, "error": str|None, ...}
        self.cfg = config.TEST
        self.thread = None
        self.cancel = threading.Event()
        self.cancel_why = None
        self.lock = threading.Lock()
        self.sel = [i for i, _ in STEPS]
        self.north = None                    # {"yaw", "heading", "x", "y", "az", "el"} after step 5
        self._psi = None
        self._act = None
        self.prog = self._fresh()

    # state
    @staticmethod
    def _fresh(sel=None):
        """sel = ids that will run, the others stay 'idle' (not selected)."""
        return {"state": "idle", "current": None, "msg": "", "t0": None, "t1": None, "selected": list(sel or []),
                "steps": [{"id": i, "title": t, "state": "pending" if (sel is not None and i in sel) else "idle",
                           "detail": "", "data": {}} for i, t in STEPS]}

    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def status(self) -> dict:
        with self.lock:
            return copy.deepcopy(dict(self.prog, north=self.north))

    def _step(self, sid):
        return next(s for s in self.prog["steps"] if s["id"] == sid)

    def _set(self, sid, state=None, detail=None, data=None, **kw):
        """Update a step. `data={...}` and plain keywords both go into the step's data dict."""
        with self.lock:
            s = self._step(sid)
            if state is not None:
                s["state"] = state
            if detail is not None:
                s["detail"] = detail
            s["data"].update(data or {})
            s["data"].update(kw)
        if state in ("pass", "warn", "fail", "skip"):
            log.info("Test %s: %s - %s", sid, state, s["detail"])

    # control
    # a test that needs another one's result pulls it in: north needs the GNSS heading
    DEPENDS = {"north": ["gnss"]}

    def start(self, only=None):
        """only = list of step ids to run (None = all). Each is independent, but 'north' also runs 'gnss'."""
        if self.busy():
            raise RuntimeError("a test is already running")
        ids = [i for i, _ in STEPS]
        only = list(only) if only else ids
        bad = [o for o in only if o not in ids]
        if bad:
            raise ValueError(f"unknown test {bad}")
        for o in list(only):
            for d in self.DEPENDS.get(o, []):
                if d not in only:
                    only.append(d)
        sel = [i for i in ids if i in only]                       # keep the natural order
        if any(i in sel for i in ("motors", "north")) and self.g.mode == "offline":
            raise RuntimeError("the drives are not answering (check 48 V, the CAN cable and the Pi relay)")
        self.sel = sel
        self.cancel.clear()
        self.cancel_why = None
        self.north = None
        self._psi = None
        self._ghist = []
        self._gwatch_stop = threading.Event()
        self.prog = self._fresh(sel)
        self.prog.update(state="running", t0=clock.now().isoformat(timespec="seconds"))
        self.thread = threading.Thread(target=self._worker, name="test", daemon=True)
        self.thread.start()
        # GNSS receiver is watched from the start so its search is already on the chart when its step is reached
        if "gnss" in sel:
            threading.Thread(target=self._gnss_watch, name="test-gnss", daemon=True).start()
        return "started"

    def abort(self, why="Abort"):
        if self.busy():
            self.cancel_why = why
            self.cancel.set()

    def _check(self):
        if self.cancel.is_set():
            raise Abort(self.cancel_why or "Abort")
        a = self._act
        if a is not None and a["event"].is_set() and a["ok"] is False:
            raise Abort(f"gimbal stopped ({a['why']})")

    def _sleep(self, dt):
        end = time.monotonic() + dt
        while time.monotonic() < end:
            self._check()
            time.sleep(min(0.1, max(0.0, end - time.monotonic())))

    # worker
    def _worker(self):
        order = [("link", self._link), ("motors", self._motors), ("sdr", self._sdr),
                 ("gnss", self._gnss), ("north", self._north)]
        try:
            for sid, fn in order:
                if sid not in self.sel:
                    continue
                self._check()
                self.prog["current"] = sid
                self._set(sid, "running")
                try:
                    fn()
                except Abort:
                    raise
                except Exception as e:
                    log.exception("Test step %s", sid)
                    self._set(sid, "fail", str(e) or e.__class__.__name__)
                if self._step(sid)["state"] == "running":
                    self._set(sid, "fail", "step ended without a result")
            states = [s["state"] for s in self.prog["steps"] if s["id"] in self.sel]
            self.prog["state"] = "failed" if "fail" in states else "done"
            self.prog["msg"] = ("FAILED: " + ", ".join(s["title"] for s in self.prog["steps"] if s["id"] in self.sel and s["state"] == "fail")
                                if "fail" in states else
                                "All checks passed" + (" (with warnings)" if "warn" in states else ""))
        except Abort as e:
            self._cleanup(str(e))
            self.prog["state"] = "aborted"
            self.prog["msg"] = f"aborted: {e}"
            for s in self.prog["steps"]:
                if s["id"] in self.sel and s["state"] in ("running", "pending"):
                    self._set(s["id"], "skip", "aborted" if s["state"] == "running" else "not run")
        except Exception as e:
            log.exception("Test failed")
            self._cleanup(str(e))
            self.prog["state"] = "failed"
            self.prog["msg"] = str(e)
        finally:
            self._gwatch_stop.set()
            self.prog["current"] = None
            self.prog["t1"] = clock.now().isoformat(timespec="seconds")

    def _cleanup(self, why):
        """Stop a goto in progress (motors stay on and holding)."""
        try:
            if self.g.act is not None:
                self.g.stop_act("test " + why).result(3)
        except Exception:
            pass

    def _skip_rest(self, sids, why):
        for sid in sids:
            self._set(sid, "skip", why)

    # 1. link
    def _link(self):
        rel = self.relay_fn()
        snap = self.g.snapshot()
        found = {k: bool(a["found"]) for k, a in snap["axes"].items()}
        data = {"relay": rel.get("name"), "connected": bool(rel.get("connected")), "bus_v": snap["bus_v"],
                "x_found": found.get("x"), "y_found": found.get("y"), "mode": snap["mode"]}
        self._set("link", data=data)
        if not rel.get("connected"):
            return self._set("link", "fail", f"CAN link/relay not connected: {rel.get('error') or ''}")
        if not all(found.values()):
            miss = ", ".join(k.upper() for k, v in found.items() if not v)
            return self._set("link", "fail", f"drive {miss} not answering")
        bus = snap["bus_v"]
        if bus is not None and bus < 40:
            return self._set("link", "warn", f"both drives answer, but bus voltage is only {bus:.1f} V (expected ~48 V)")
        self._set("link", "pass", f"{rel.get('name')}: X and Y answer" + (f", bus {bus:.1f} V" if bus else ""))

    # 2. motors
    def _wait_holding(self, timeout, what):
        end = time.monotonic() + timeout
        while not (self.g.mode == "on" and self.g.started and self.g.holding):
            self._check()
            if self.g.mode in ("fault", "offline"):
                raise RuntimeError(f"{what}: gimbal in {self.g.mode} ({self.g.fault or ''})")
            if time.monotonic() > end:
                raise RuntimeError(f"{what}: motors did not come up in {timeout:.0f} s")
            time.sleep(0.2)

    def _goto_wait(self, x, y, label, k):
        """goto + wait. Returns (ok, seconds, peak |current| A of axis k, final (x, y))."""
        cfg = self.cfg
        t0 = time.monotonic()
        act = self.g.goto(x, y, label).result(5)
        self._act = act
        peak = 0.0
        while not act["event"].wait(0.15):
            self._check()
            c = self.g.run[k].current
            if c is not None:
                peak = max(peak, abs(c))
            if time.monotonic() - t0 > cfg.move_timeout_s:
                self.g.stop_act("test timeout").result(3)
                return False, time.monotonic() - t0, peak, self.g.angles()
        self._act = None
        time.sleep(0.4)                                      # let the position read catch up
        return bool(act["ok"]), time.monotonic() - t0, peak, self.g.angles()

    def _ensure_motors(self, sid):
        g = self.g
        if not (g.started and g.mode == "on"):
            self._set(sid, detail="starting both motors (Start sequence)…")
            g.begin().result(5)
            self._sleep(1.0)
            self._wait_holding(150, "start")
        else:
            self._wait_holding(30, "motors")

    def _motors(self):
        cfg, g = self.cfg, self.g
        self._ensure_motors("motors")
        d = cfg.motor_move_deg
        rows, bad, warns = [], [], []
        for k in ("x", "y"):
            o = "y" if k == "x" else "x"
            lim = g.axes[k].limits
            lim_a = g.axes[k].cfg.current_limit_a
            for label, tgt in ((f"+{d:g}°", d), (f"−{d:g}°", -d), ("HOME", 0.0)):
                if not lim[0] <= tgt <= lim[1]:
                    raise RuntimeError(f"{k.upper()} test target {tgt:+.1f}° is outside the soft limit")
                hold = g.axes[o].angle if g.axes[o].angle is not None else 0.0
                tx, ty = (tgt, hold) if k == "x" else (hold, tgt)
                self._set("motors", detail=f"{k.upper()}: moving to {label} …", progress={"axis": k, "label": label})
                ok, secs, peak, (ax_, ay_) = self._goto_wait(tx, ty, f"test {k.upper()} {label}", k)
                got = ax_ if k == "x" else ay_
                err = None if got is None else got - tgt
                row = {"axis": k, "label": label, "target": tgt, "reached": got, "err": err,
                       "secs": round(secs, 1), "peak_a": round(peak, 2), "ok": ok}
                rows.append(row)
                self._set("motors", data={"rows": list(rows)})
                if not ok:
                    bad.append(f"{k.upper()} {label}: move did not complete")
                    break
                if err is None or abs(err) > cfg.motor_tol_deg:
                    bad.append(f"{k.upper()} {label}: ended {'?' if err is None else f'{err:+.2f}°'} from the target")
                if peak > lim_a:
                    bad.append(f"{k.upper()} {label}: current {peak:.1f} A above the {lim_a:.0f} A limit")
                elif peak > 0.8 * lim_a:
                    warns.append(f"{k.upper()} current reached {peak:.1f} A of {lim_a:.0f} A")
                err_code = g.run[k].err
                if err_code:
                    bad.append(f"{k.upper()} drive error {err_code:#06x}")
            if bad:
                break
        if not bad:                                          # both back at HOME
            ok, secs, _, (x, y) = self._goto_wait(0.0, 0.0, "test: return to HOME", "x")
            if not ok or x is None or y is None or max(abs(x), abs(y)) > cfg.motor_tol_deg:
                bad.append(f"return to HOME ended at X {x}, Y {y}")
            else:
                rows.append({"axis": "xy", "label": "both to HOME", "target": 0.0, "reached": max(abs(x), abs(y)),
                             "err": max(abs(x), abs(y)), "secs": round(secs, 1), "peak_a": None, "ok": True})
                self._set("motors", data={"rows": list(rows)})
        if bad:
            return self._set("motors", "fail", "; ".join(bad))
        self._set("motors", "warn" if warns else "pass",
                  ("; ".join(warns) if warns else f"X and Y moved ±{d:g}° and returned to HOME"))

    # 3. SDR
    def _sdr(self):
        cfg = self.cfg
        m = self.meter_fn()
        if m is None:
            return self._set("sdr", "fail", "SDR disabled (--no-sdr)")
        if not m.available:
            return self._set("sdr", "fail", f"SDR not available: {m.error}")
        self._set("sdr", detail=f"reading for {cfg.sdr_seconds:.0f} s …")
        t0 = time.monotonic()
        while time.monotonic() - t0 < cfg.sdr_seconds:
            self._check()
            rs = m.since(t0)
            if rs:
                db = [r["db"] for r in rs]
                self._set("sdr", detail=f"{len(rs)} readings, now {db[-1]:.1f} dBFS",
                          points=[round(r["db"], 2) for r in rs][-120:])
            time.sleep(0.25)
        rs = m.since(t0)
        if not rs:
            return self._set("sdr", "fail", "no readings arrived")
        db = [r["db"] for r in rs]
        mean_lin = sum(r["lin"] for r in rs) / len(rs)
        mean_db = 10 * math.log10(mean_lin + 1e-30)
        sd = math.sqrt(sum((v - sum(db) / len(db)) ** 2 for v in db) / len(db))
        rate = len(rs) / max(rs[-1]["tm"] - rs[0]["tm"], 0.1)
        sat = sum(1 for r in rs if r["sat"])
        peak = max(r["peak"] for r in rs)
        self._set("sdr", data={"n": len(rs), "rate_hz": round(rate, 1), "mean_db": round(mean_db, 2),
                               "min_db": round(min(db), 2), "max_db": round(max(db), 2), "std_db": round(sd, 2),
                               "peak": round(peak, 3), "sat": sat, "gain_db": m.gain_db, "name": m.name,
                               "points": [round(v, 2) for v in db][-120:]})
        summary = (f"{len(rs)} readings ({rate:.1f}/s), mean {mean_db:.1f} dBFS "
                   f"[{min(db):.1f} … {max(db):.1f}], gain {m.gain_db:.0f} dB")
        if sat:
            return self._set("sdr", "warn", f"{summary}; SATURATED in {sat} readings (peak {peak:.2f} FS): reduce gain")
        if rate < cfg.sdr_min_rate_hz:
            return self._set("sdr", "warn", f"{summary}; reading rate low (expected ~10/s)")
        if mean_db < -75:
            return self._set("sdr", "warn", f"{summary}; very low level: is the LNB bias tee on?")
        self._set("sdr", "pass", summary)

    # 4. GNSS
    @staticmethod
    def _gnss_stage(st):
        if not st.get("connected"):
            return 0
        if (st.get("fix_type") or 0) < 3:
            return 1
        if st.get("carr") == 2 and st.get("heading_valid"):
            return 4
        return 3 if st.get("carr") == 1 else 2

    def _gnss_watch(self):
        """Sample the receiver twice a second for the whole test (stage + satellite count) for the chart history."""
        t0 = time.monotonic()
        stop = self._gwatch_stop
        while not stop.is_set():
            st = self.gnss.status()
            stage = self._gnss_stage(st)
            self._ghist.append([round(time.monotonic() - t0, 1), st.get("num_sv") or 0, stage])
            del self._ghist[:-900]
            self._set("gnss", stage=stage, stage_name=GNSS_STAGES[stage], num_sv=st.get("num_sv"),
                      hist=list(self._ghist), port=st.get("port"), h_acc_m=st.get("h_acc_m"),
                      length_cm=st.get("length_cm"), lat=st.get("lat"), lon=st.get("lon"), error=st.get("error"))
            if self._step("gnss")["state"] == "pending":
                self._set("gnss", detail=f"watching in the background: {GNSS_STAGES[stage]}, {st.get('num_sv') or 0} satellites")
            stop.wait(0.5)

    def _gnss(self):
        cfg = self.cfg
        t0 = time.monotonic()
        stable = None
        while True:
            self._check()
            h = self._ghist[-1] if self._ghist else [0, 0, 0]
            stage, nsv, el = h[2], h[1], time.monotonic() - t0
            self._set("gnss", detail=f"{GNSS_STAGES[stage]} · {nsv} satellites · {el:.0f} s")
            if stage == 4:
                stable = stable or time.monotonic()
                if time.monotonic() - stable >= 2.0:
                    break
            else:
                stable = None
            if el > cfg.gnss_timeout_s:
                err = self._step("gnss")["data"].get("error")
                return self._set("gnss", "fail",
                                 f"no RTK FIXED after {cfg.gnss_timeout_s:.0f} s (stuck at: {GNSS_STAGES[stage]}, "
                                 f"{nsv} satellites){' - ' + err if err else ''}")
            time.sleep(0.5)
        hist = list(self._ghist)
        t_fix = next((h[0] for h in hist if h[2] == 4), hist[-1][0])        # first time RTK FIXED was seen
        secs = cfg.gnss_heading_avg_s
        self._set("gnss", detail=f"RTK FIXED: averaging the heading for {secs:.0f} s …")
        avg = self.gnss.average_baseline(secs, cancel=lambda: self.cancel.is_set())
        self._check()
        if avg is None:
            return self._set("gnss", "fail", "RTK FIXED was lost: too few valid headings during the average")
        bar = config.gnss_bar_deg()
        psi = wrap180(avg["heading"] - bar)
        self._psi = {"yaw": psi, "heading": avg["heading"], "n": avg["n"], "pitch": avg.get("pitch")}
        self._set("gnss", "pass",
                  f"RTK FIXED {t_fix:.0f} s after pressing Test · {hist[-1][1]} satellites · heading {avg['heading']:.2f}° "
                  f"→ mount yaw ψ = {psi:+.2f}°", heading=avg["heading"], psi=psi, n=avg["n"], pitch=avg.get("pitch"))

    # 5. north, 45 deg
    def _north(self):
        cfg, g = self.cfg, self.g
        if self._step("gnss")["state"] != "pass" or not self._psi:
            return self._set("north", "skip", "needs RTK FIXED heading (GNSS step)")
        if self._step("motors")["state"] == "fail":
            return self._set("north", "skip", "motors failed: not moving the dish")
        self._ensure_motors("north")
        psi = self._psi["yaw"]
        cal = {"yaw": psi}
        az, el = cfg.north_az_deg, cfg.north_el_deg
        try:
            x, y = K.azel_to_axes(az, el, cal)
        except ValueError as e:
            return self._set("north", "fail", str(e))
        if not K.within_limits(x, y):
            return self._set("north", "fail", f"needs X {x:+.1f}°, Y {y:+.1f}°: beyond the ±80° soft limits")
        warns = []
        lat, lon, _ = self.site_fn()
        saz, sel = sun_azel(clock.now(), lat, lon)
        if sel > 0:
            sep = math.degrees(math.acos(max(-1.0, min(1.0, float(K.azel_to_enu(az, el) @ K.azel_to_enu(saz, sel))))))
            if sep < cfg.sun_clear_deg:
                warns.append(f"the Sun is only {sep:.0f}° from the target (LNB heat!)")
        self._set("north", detail=f"ψ = {psi:+.2f}°: moving to X {x:+.1f}°, Y {y:+.1f}° …", target={"x": x, "y": y, "az": az, "el": el})
        ok, secs, _, (ax_, ay_) = self._goto_wait(x, y, "true north 45°", "x")
        if not ok:
            return self._set("north", "fail", "the move did not complete")
        self._sleep(1.0)
        ax_, ay_ = g.angles()
        baz, bel = K.axes_to_azel(ax_, ay_, cal)
        daz = wrap180(baz - az)
        err = math.hypot(daz * math.cos(math.radians(bel)), bel - el)
        self.north = {"yaw": psi, "heading": self._psi["heading"], "x": ax_, "y": ay_, "az": baz, "el": bel,
                      "t": clock.now().isoformat(timespec="seconds")}
        self._set("north", data={"x": ax_, "y": ay_, "az": baz, "el": bel, "err_deg": err, "secs": round(secs, 1)})
        msg = f"beam at az {(baz + 0.005) % 360 - 0.005:.2f}°, el {bel:.2f}° (X {ax_:+.2f}°, Y {ay_:+.2f}°), {err:.2f}° from the target"
        if not g.axes["x"].cfg.min_deg < ax_ < g.axes["x"].cfg.max_deg:
            warns.append("X near its limit")
        if config.BORESIGHT_OFFSET.get("deg") and not config.BORESIGHT_OFFSET.get("confirmed"):
            warns.append(f"boresight offset direction ({config.BORESIGHT_OFFSET['toward']}) is not confirmed in config.py: "
                         "the beam may not really be where this says")
        if err > 0.5:
            return self._set("north", "fail", msg + ": too far from the target")
        self._set("north", "warn" if warns else "pass", msg + ("; " + "; ".join(warns) if warns else ""))
