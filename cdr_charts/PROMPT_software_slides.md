# Prompt for the slides Claude: software additions (paste below the line)

---

This adds to the hardware/photo slides I've already asked for. It covers the **software** side of the ground station. Rules:

- **Add at most 3 new slides.** Wherever a photo slide already covers a topic (Sun tracking, satellite track, link test), put the chart **on that slide** and don't make a new one.
- **Be honest. This is a CDR.** Label every result as *verified on hardware*, *simulation only* or *not yet shown*. Never present a simulated result as a field result.
- Keep it short: at most 4 bullets per slide and at most about 12 words per bullet. One chart per slide, two at most.
- Charts are in `cdr_charts/` (PNG). All times are 11 Nov 2026, SGT = UTC+8.

## Software facts (current system, gs_v3)

- Python web app (`./run.sh`, localhost:8080) with a live 3D gimbal view and 5 tabs: **Manual, Sun, Satellite, Test, Demod**. It detects the hardware that's plugged in, and `--sim` simulates everything.
- Chain: laptop app → TCP → **Raspberry Pi CAN relay** (watchdog STOP, ID filter) → CAN 1 Mbps → **eRob110 (X) + eRob90 (Y)**. Only one thread ever talks to CAN.
- Sensors: **dual ZED-F9P RTK heading** (north), **B206mini SDR** (receive only) behind the Ku LNB (IF 1150 MHz) for the Sun power meter and the IQ capture.
- Demod: GNU Radio + gr-dvbs2rx DVB-S2 receiver (QPSK 2/3, 2 MBd) → BBFRAME → GSE → GSUFTP file reassembly, checked with SHA-256 manifests.
- Satellite mode: CelesTrak OMM (cached for 2 h) → pass prediction → limit-checked trajectory → pre-slew → track → park.
- Sun auto-sequence: start motors → home → GNSS north (60 s RTK-FIXED average) → slew to the predicted Sun → ±12° ring scan → cold sky → X/Y cross-scans → track. If no peak shows above threshold, it **falls back to GNSS-only pointing**.
- Safety in software: soft limits ±80°, a jog-release STOP, a motor-current trip (15 A on X, 11 A on Y), and an LNB heat rule (at most 120 s near the Sun, then move 10° off). It never writes drive flash, and the SDR's TX path is never used.

## New slides (max 3)

**1. Software architecture** (only if the deck has no system/data-flow diagram yet)
One block diagram: UI/app → gimbal thread → Pi relay → drives; GNSS → north; SDR → (a) power meter for Sun peak-up, (b) IQ recorder → DVB-S2 demod → files. Add a small side box listing the safety interlocks above.

**2. Field results: evidence** (merge into the photo slides if they already exist)
- Sun (chart `01_sun_ring_scans_real_vs_sim.png`, optional `02_sun_cloudy_run_timeline.png`): 6 real ring scans on a cloudy day all peaked at +0.03 to +0.19 dB. The threshold is 0.5 dB and the SDR noise is σ = 0.014 dB, so **no Sun detection on hardware**. The same code in simulation finds a +8.2 dB peak and corrects north. On hardware the fallback worked: the dish pointed and tracked the Sun from GNSS north alone. The servo following error during the track was 0.015° mean and 0.026° max (`03_sun_track_servo_error.png`). Note that this measures the motors following the command, not RF pointing accuracy.
- Satellite (chart `04_dseo_pass_skyplot.png`): DS-EO (NORAD 52935) pass from 11:22 to 11:34 SGT, max elevation 36.5°. The gimbal was on the predicted track around the IQ recording window (11:26:46 to 11:29:08, 142 s, 0 overflows). **[TEAM TO CONFIRM: logs show 0 DVB-S2 frames decoded from this recording, and its spectrum shows no carrier (chart 05, blue). State exactly what was received during the pass.]**
- Link test (chart `05_iq_spectrum_link_vs_pass.png`, green): a ground transmitter far away sent a DVB-S2 QPSK 2/3 2 MBd carrier, about 19 dB above the noise floor. The full demod chain received **11 files (10 JPEG + 1 text, 442 kB)** with SHA-256 manifests. Earlier offline replay rehearsals (QPSK 1/4 at Es/N0 1 dB) decoded about 608 BBFRAMEs per run with 0 or 1 bad GSE CRC.
- Optional chart `06_peak_power_per_run.png`: the strongest power on hardware all day was +3.9 dB, with the dish pointing low (az 246°, el 24°). That is not the Sun, but it shows the receive chain does respond to power changes.

**3. Verification status and open items** (table: Item | Status | Next step)
- Verified on hardware: CAN motion and safety stops, manual WASD and typed-angle goto, per-component Test mode, GNSS north, Sun tracking on GNSS-only pointing, satellite pass prediction and pre-slew, end-to-end file downlink over a ground link.
- Simulation only: Sun peak-up and north correction, the mount calibration fit.
- Not yet shown: a Sun peak on hardware (needs a clear sky, and the 22° boresight offset is not yet confirmed), a decoded satellite downlink, G/T measurement, motor-current data in the field (logging was added on 12 Nov; there's no hardware data yet). If `07_motor_current_timeline.png` / `08_motor_current_max_by_activity.png` exist, put them on the Test-mode slide: peak current by activity against the 15 A (X) / 11 A (Y) trip limits.
