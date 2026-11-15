# X-Y Gimbal Ground Station (v3)

Ku-band X-Y gimbal: ZeroErr eRob110 (X, node 1) and eRob90 (Y, node 2) on ZeroErr's CAN-custom protocol at 1 Mbps. Pointing calibration uses a dual ZED-F9P GNSS heading kit plus Sun scans through the LNB and a B206mini SDR (receive only).

## Run

```bash
./run.sh                      # real hardware: opens http://localhost:8080; whatever is plugged in is detected
./run.sh --pi 192.168.1.50    # also drive the gimbal through the Pi relay (or GS_PI=<ip> ./run.sh)
./run.sh --sim                # everything simulated, no hardware (add --sim-clock 2026-11-09T09:30:00+08:00)
python3 can_relay.py --allow <laptop-ip>   # on the Pi next to the gimbal (relays CAN only)
python3 selftest.py           # offline checks (~2 min)
```

`run.sh` uses `~/radioconda/bin/python` (or any python with numpy; override with `GS_PYTHON`). Needs `skyfield` and `requests` for
satellites (`pip install skyfield requests`) and `pyubx2`/`pyserial` for the GNSS. Nothing is required to be plugged in: the page shows what it found.

The page has a live 3D view of the gimbal on top (built from `XY Dish Assembly.STEP`; it follows the real X/Y angles) and five tabs:

| Tab | What it does |
|---|---|
| **Manual** | Start, WASD (latching), go to an angle (axis X/Y or sky az/el), Home |
| **Sun** | One button: start motors, find north (GNSS), Sun position, point, peak-up if the SDR is there, track. Every step is narrated. Step-by-step Sun find and G/T under "Advanced" |
| **Satellite** | Pick a satellite or "Auto" (the pass in progress / next one the dish can follow): CelesTrak elements, north, path check, slew, track, park |
| **Test** | One card per component (gimbal, SDR, GNSS), detected automatically; run any check on its own or "Run all detected" |
| **Demod** | The Galassia-5 DVB-S2 receiver (see `demod/unified/README.md`) |

The 3D model is `ui/model/gimbal.glb` (+ `gimbal_meta.json`); three.js is vendored in `ui/vendor/`, so the page needs no internet.

## DISABLED (test stage): the full v3 UI had one screen, three modes

| Mode | What it does |
|---|---|
| **Calibrate** | 1. Set home (jog level, spirit level on the dish back). 2. Measure with GNSS (6 poses, ~2 min). 3. Scan the Sun (~3 min, morning + afternoon). |
| **Test** | Hold **W/S** = X axis (tilt north/south), **A/D** = Y axis (tilt west/east), at 0.2 / 0.5 / 1 °/s. Release = stop. **Space/Esc = STOP** (motors off). |
| **Auto** | Predicts passes of `config.TARGETS` (by NORAD number), pre-slews 2 min early, tracks, returns home. Needs calibration. |

## Files

| File | Purpose |
|---|---|
| `config.py` | All constants: CAN 1 Mbps, 524288 counts/rev, axes/limits (±80°), BORESIGHT_OFFSET, relay, GNSS, SDR, Sun job |
| `can_relay.py` | Runs on the Pi: TCP ↔ can0 via candump/cansend, ID/register filter, watchdog STOP (+ `can-relay.service`) |
| `sun_job.py` | Sun methods 1/2, peak-up, track, LNB heat rule, G/T, logs |
| `zeroerr_protocol.py` | CAN adapters (RemoteCanLink = Pi relay, can-utils, SocketCAN, Waveshare, sim) and one eRob drive |
| `erob_can_axis.py` | One axis: home zero, sign, soft limits, relative moves |
| `gimbal.py` | The only thread that touches CAN: jog / goto / track / fault handling |
| `kinematics.py` | X-Y geometry and the mount-calibration model (the only copy) |
| `calibration.py` | GNSS poses + Sun cross-scans + least-squares fit → `calibration.json` |
| `sun.py` | Sun position (no download), with refraction |
| `gps_reader.py` | ZED-F9P UBX reader: position, clock offset, baseline heading/pitch |
| `sdr_power.py` | B206mini RX2 power meter (receive only) |
| `tle_manager.py` | OMM elements by NORAD number from CelesTrak, cached, at most every 2 h |
| `trajectory.py` | Precomputed, limit-checked pass tables |
| `pass_scheduler.py` | Pass prediction + Auto-mode daemon |
| `app.py` + `ui/index.html` | Web UI server (stdlib HTTP + SSE) |
| `sim_hw.py` | Simulated drives, GNSS and Sun noise with a hidden mount error |
| `track_satellite.py`, `track_sun.py` | Headless command-line versions of Auto / Sun scan |
| `doppler.py`, `pass_sim.py`, `sim_server.py`, `pass_sim.html` | Unchanged tools (pass_sim uses the new modules) |
| `clock.py` | The one source of "now" (UTC); the simulator can shift it |
| `sat_job.py` | Satellite tab: pick pass, north, path check, slew, track, park |
| `test_job.py` | Test tab: link / motors / SDR / GNSS checks |
| `demod.py` | Runs the Galassia-5 DVB-S2 receiver (`demod/unified/`) as a supervised subprocess |
| `rehearsal.py` | Receiver rehearsal with no radio: fake downlink file → receiver → decoded image |
| `demod_env.sh` | Runs a command in the demod env (GNU Radio 3.10 + the patched gr-dvbs2rx in `gr_dvbs2rx/`) |
| `gnss_check.py`, `sdr_plot.py` | Stand-alone live checks of the GNSS heading kit and the LNB IF power |
| `current_char.py` | Headless motor-current characterisation over the ±80° range (see `docs/notes/HANDOFF_current_char.md`) |
| `current_char_analysis.py`, `current_char_merge.py`, `current_char_report.py` | Analysis, run merging, figures/dashboard for the current characterisation |
| `cdr_charts/` | Chart scripts for the CDR (field logs, motor currents) and the generated charts |
| `selftest.py` | Offline checks |
| `sim_acceptance.py` | End-to-end test on the simulator through the web API (~25 min) |

## Repository layout

| Path | Contents |
|---|---|
| `ui/` | Web page, 3D viewer, vendored three.js and fonts, gimbal model (`ui/legacy/` = the v1 page) |
| `demod/unified/` | Galassia-5 DVB-S2 RX/TX flowgraphs, CLIs and the `g5gs` decoder package |
| `gr_dvbs2rx/` | Prebuilt patched gr-dvbs2rx (Python 3.14, x86_64), used by `demod_env.sh` |
| `data/current_char/` | The four hardware runs used for the final current result, and the merged `20261114_combined/` |
| `docs/` | UI screenshots; `docs/notes/` holds the current-characterisation handoff and the v2→v3 upgrade notes |

Not in git (see `.gitignore`): `calibration.json` and `omm_cache.json` (site-specific, written at runtime), `logs/`, `Received/` (IQ recordings), backups, zips and the UREx report.

## Safety

* Software limits are ±80° from HOME (enforced in the app; the relay knows no angles). Before HOME is set, each axis can move only ±30° from where it started.
* The software never writes drive flash or drive limits (0x50, 0x52, 0x53, 0xE8).
* Releasing a jog key sends STOP (0x84). If the drive keeps moving, the motors are disabled.
* Sun scans never park on the Sun: they cap time near it and finish 10° away, because the dish focuses heat on the LNB.
* The B206mini is receive-only: RX2 only, nothing connected to TX/RX.
