# Claude Code task: upgrade the ground-station repo from v2 to v3

Paste this file into Claude Code (VS Code terminal) from the repo root, with the `gs_v3/` bundle unzipped next to the repo.

---

## 0. Context (read first)

- **This repo** holds the v2 code: `config.py, zeroerr_protocol.py, erob_can_axis.py, tle_manager.py, trajectory.py, pass_scheduler.py, track_satellite.py, track_sun.py, gps_reader.py, manual_jog.py, doppler.py, server.py, pass_sim.py, sim_server.py, pass_sim.html, trackingdemo.html, tle_cache.json, README.md, requirements.txt`.
- **`../gs_v3/`** holds the reference v3 implementation. It passed `selftest.py` and a full simulated run: jog, home, GNSS calibration, Sun scans and Auto tracking.
- **Your job:** bring this repo to v3 *in place*.
  1. `diff` each repo file against its v3 version.
  2. If the repo file still matches the v2 original, replace it.
  3. If the user has changed it since v2, merge: keep their extra changes and apply every item in section 2 below.
- **Hardware:**
  - Raspberry Pi 5 + CAN HAT (`can0`), or a mini PC + Waveshare USB-CAN-A (CH340 serial, not SocketCAN).
  - Two ZeroErr eRob drives on the CAN-custom protocol: node 1 = X (eRob110), node 2 = Y (eRob90). 48 V from an ePower, star-wired with a fuse per motor.
  - ArduSimple simpleRTK2B heading kit; the rover is the native USB port, `/dev/ttyACM*`.
  - Ettus B206mini fed from the LNB through a bias tee.

## 1. Hard rules (never break these)

**CAN / eRob: use only the bench-proven method** (2026-10-20, both motors turned exactly 90°).
- Bus is **1 Mbps**: `sudo ip link set can0 up type can bitrate 1000000 restart-ms 100`.
- **524288 counts per output revolution** (90° = `0x00020000`).
- Command frames go to ID `0x640+N`, replies come from `0x5C0+N`, all big-endian.
- A write is accepted only if the reply is exactly `3E`.
- A refusal is `[2-byte code][3E]`, e.g. `00 27 3E` = motor OFF. It is never `0x80`.
- Sequence:
  1. Motor OFF: `0x0100=0`.
  2. Configure while OFF: `0x004E=3`, `0x008D=1`, `0x009E=1`, accel/decel `0x0088`/`0x0089`, speed `0x008A`.
  3. Enable: `0x0100=1`.
  4. Move: relative target `0x0087`, then START `0x0083`.
  5. Read back: `0x0002` position, `0x010C` state (4 = reached), `0x001F` error.
- **Never burst-poll.** Leave about 20 ms between requests, and stay quiet for 1 s after enable. Heavy polling right after enable made the drive switch the motor off after ~0.3 s.
- **Never write** `0x0050`, `0x0052`, `0x0053` or `0x00E8` (save to flash).
- **Never write** `0x004E`/`0x008D` while the motor is ON (refused with 0x16).
- Only `gimbal.Gimbal`'s thread may send CAN frames.

**B206mini: receive only.**
- Never run or call `usrp_burn_mb_eeprom`, `usrp_burn_db_eeprom`, `b2xx_fx3_utils`, `tx_waveforms`, `tx_samples_from_file`, `uhd_siggen`, `benchmark_rate --tx_rate`, a UHD USRP Sink, `set_tx_*` or `get_tx_stream`.
- The TX/RX port stays unconnected. Input must not exceed −15 dBm.

**People and hardware.** Before any step that moves a motor, stop and ask the user to confirm:
- the area around the dish is clear, and
- they have a hand on the ePower switch.
Never run hardware tests unattended.

## 2. File-by-file changes

| v2 file | Action | What must be true afterwards |
|---|---|---|
| `config.py` | replace/merge | `bitrate=1_000_000`. `COUNTS_PER_REV=524288`. `AXES` keyed `"x"`/`"y"` with `sign`, ±85° limits, slew 5°/s, track 2°/s, accel 10°/s². `UNHOMED_TRAVEL_DEG=30`. Jog speeds 0.2/0.5/1.0. `TICK_S=0.25`, `TRACK_PERIOD_S=0.5`, `TRACK_ADAPTIVE_SPEED=True`. `GNSS`, `SDR`, `SUN_SCAN` blocks. `TARGETS` keyed by NORAD number. `dry_run_enabled` removed (use `app.py --sim`). |
| `zeroerr_protocol.py` | replace | Adapters: `SocketCanLink`, `WaveshareLink` (2 Mbps serial, AA-55 config) and `open_link()` with auto-detect. `Drive` has a 20 ms gap, flushes stale frames, treats any write reply ≠ `3E` as `DriveError` with the 2-byte code, and wraps the proven sequence as `configure()`, `enable()` (retries 0x1F/0x56), `move_relative()`, `stop()`, `disable()`. `scan()` included. |
| `erob_can_axis.py` | replace | `Axis`: angle = sign × wrap(raw − home). Software limits (±30° before home). `move_by()` refuses targets outside the limits, and 0x15 means "busy, retry". `set_speed()` tries a live 0x8A write and otherwise reconfigures (disable → configure → enable). **Removed:** the 131072 CPR, the "continuous mode" velocity hack, the 0x50 max-speed write, mode writes while enabled, and the 4-read `get_state()`. |
| — | **new** `gimbal.py` | The single CAN-owner thread. Modes: offline / disabled / hold / jog / goto / track / fault. Jog = one move toward the soft limit, then 0x84 on release (fault + disable if it keeps moving), with a 0.6 s deadman. Track = one relative step per axis every 0.5 s, aimed at the target 0.5 s ahead (adaptive speed) or 0.25 s ahead (fixed). State/error read every 2 s; motor OFF unexpectedly → fault → both disabled. |
| — | **new** `kinematics.py` | The only copy of the X-Y maths, plus the calibration model `M·Rx(x−x0)·Ry(y−y0)`. Delete the duplicated `_az_el_to_xy` from `trajectory.py` and `track_sun.py`. |
| `trajectory.py` | replace | `PassTable`: a vectorised Skyfield pass at 0.1 s steps, mapped through the calibration. `problems()` checks limits and speeds before AOS; `at(t)` interpolates. `XYSample` is kept for `doppler.py`. |
| `tle_manager.py` | replace | Look-ups by **CATNR**, using **OMM JSON** (`EarthSatellite.from_omm`). Cache `omm_cache.json` keyed by catalog number; refresh at most every 2 h; report epoch age. Delete `tle_cache.json`: it maps "ISS" to UME, NORAD 8709. |
| `pass_scheduler.py` | replace | `find_passes()` + `AutoTracker` thread. No multi-hour `sleep()`. Skips passes with elements older than 5 days, or that leave the limits or exceed track speed. Pre-slews 120 s early, tracks via `gimbal.track(table.at)`, then parks at 0/0. |
| `track_satellite.py` | replace | Headless command-line front end to `AutoTracker` for one NORAD number. The old 10 Hz velocity loop is gone. |
| `track_sun.py` | replace | Headless command-line front end to `calibration.SunScanJob`. The de421 download and the "rotate the base by eye" loop are gone. |
| `gps_reader.py` | replace | UBX reader for the heading-kit rover (`/dev/ttyACM*`, u-blox VID 1546). Reads NAV-PVT and NAV-RELPOSNED, configured in RAM only. Provides `status()` and `average_baseline()`. pynmea2 dropped. |
| `manual_jog.py` | **delete** | Replaced by the UI's Test mode. Over SSH: `ssh -L 8080:localhost:8080 pi@…`. |
| `server.py` | **delete** | It is broken: it imports a non-existent `ZeroErrCANBus` and calls `TrajectoryModel()` with no satellite. Replaced by `app.py`. |
| — | **new** `app.py`, `ui/index.html` | Standard-library HTTP + SSE server and the single-page UI (Calibrate / Test / Auto). |
| — | **new** `calibration.py`, `sun.py`, `sdr_power.py`, `sim_hw.py`, `clock.py`, `selftest.py` | See README. |
| `doppler.py` | keep | Unchanged; still imports `XYSample`. |
| `pass_sim.py` | patch | Uses `kinematics.azel_to_axes_vec` and `config.AXES["x"/"y"]` with `track_speed_deg_s`. Satellites by name from `TARGETS` or by NORAD number. |
| `sim_server.py`, `pass_sim.html` | keep | — |
| `trackingdemo.html` | patch | CPR 524288. The CAN pane is labelled as an illustration only. |
| `README.md`, `requirements.txt` | replace | Add `pyubx2==1.4.0` and `numpy`; drop `pynmea2`. |

Also delete the `calibration.json` / `omm_cache.json` that the simulator created if they were committed. Add them to `.gitignore`.

## 3. Offline verification (no motors)

```bash
git checkout -b v3-proven-can
pip install -r requirements.txt
python3 selftest.py                       # must print OVERALL: PASS
python3 app.py --sim --sim-clock 2026-11-06T09:00:00+08:00
```

In the browser (http://localhost:8080):
1. Calibrate: Enable motors, hold D, then Set home here.
2. Run Measure with GNSS. Yaw should come out ≈ +3.2°.
3. Run Scan the Sun. Status should end as **Calibrated**.
4. Test: holding W moves the X needle toward the North end.
5. Auto: Start, and it slews to the next pass.

## 4. Hardware bring-up ladder (one step at a time; ask the user before each)

| # | Do | Pass |
|---|---|---|
| T0 | `python3 can_scan.py --expect 1 2` (existing bench tool) | Both nodes answer; the bus is clean after the scan. |
| T1 | `python3 app.py`, motors **off**, leave it 5 min | Header shows "Motors off · 48 V"; positions update; no faults in the log. |
| T2 | Enable motors (Test mode), idle 10 min | No "switched its motor OFF". **This is the polling soak:** 4 position reads/s per axis plus status every 2 s. If a drive drops, raise `TICK_S` to 0.5 and repeat. |
| T3 | Test mode, 0.2°/s: tap W, S, A, D for ~3 s each | W tilts north, S south, A west, D east. If one is reversed, flip that axis's `sign` in `config.py` (don't swap keys). |
| T4 | Hold D at 1°/s for 5 s, release | Stops within ~0.3 s. If the log says "STOP (0x84) did not stop the move", **report it**: release-to-stop would then need the disable fallback. |
| T5 | First track/goto: watch the log for "drive refuses live speed changes" | Record yes/no. "No" (no message) = smooth adaptive tracking. "Yes" = falls back to a fixed 2°/s step (±0.25° saw-tooth), which is acceptable. |
| T6 | Calibrate step 1: level the dish, Set home. Then Test: "Return to home". | Both axes read 0.00 ± 0.02°. |
| T7 | Track soak (`python3 -c` snippet below) for 5 min | No cut-outs; `track_err` mean < 0.3°. |
| T8 | Terrace, clear sky: Calibrate steps 2 and 3 (morning **and** afternoon Sun, 15–80° elevation) | Sun rise ≥ 3 dB, Sun fit RMS < 0.1°, status "Calibrated". |
| T9 | Auto mode for one real pass | It pre-slews, tracks and parks. Log the SDR power during the pass. |

T7 snippet:
```python
import math, time, clock
from zeroerr_protocol import open_link; from gimbal import Gimbal; from calibration import Calibration
g = Gimbal(open_link(), Calibration().data["home"]); g.start(); time.sleep(3)
g.enable().result(5); time.sleep(2); t0 = clock.now()
s = lambda t: (t - t0).total_seconds()
# both axes swing +/-15 deg, peak 0.79 deg/s (= a high satellite pass)
g.track(lambda t: (15*math.sin(2*math.pi*s(t)/120), 15*math.sin(2*math.pi*s(t)/120 + 1)), "soak").result(5)
for i in range(1200):
    time.sleep(0.25)
    if i % 20 == 0: print(f"{s(clock.now()):5.0f}s {g.mode} err={g.track_err}")
g.shutdown()
```

## 5. Report back to the user

- T2–T5 results: polling OK? 0x84 honoured? Live 0x8A accepted? Axis signs as shipped?
- Any `DriveError` codes seen, with node and register.
- The diff summary per file and any user changes you had to merge.
- Do not commit or push unless the user asks.

## 6. Known unknowns (decide from the bench results, not by guessing)

1. Whether START (0x83) mid-move is refused (0x15, assumed) or retargets. The code only sends START after the previous step should have finished, so both cases are safe.
2. Whether 0x84 STOP is honoured mid-move (T4).
3. Whether 0x8A can be written while enabled (T5).
4. Whether disable engages a brake, or just drops torque on these models. If the dish sags when disabled, tell the user before going further.
5. Node 2 failed to move when it was daisy-chained behind node 1 for power. Confirm power is star-wired before T2.
