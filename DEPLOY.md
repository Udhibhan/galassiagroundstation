# DEPLOY: field test (laptop runs the app, the Pi only relays CAN)

```
UBUNTU LAPTOP (radioconda, UHD 4.11)                         RASPBERRY PI (next to the gimbal)
  app.py + web page  http://localhost:8080                     can_relay.py ── can0 ── node 1 X (eRob110)
  B206mini (USB-C) = LNB IF power, RX2 only   ── Wi-Fi/Eth ──►  candump/cansend      └─ node 2 Y (eRob90)
  u-blox heading kit (USB, optional)            TCP 7070        + watchdog: 2 s silence → 0x84 STOP both
```

Safety rules that stay true the whole time:
- The relay only sends to 0x641/0x642 and refuses writes to 0x0050/0x0052/0x0053/0x00E8.
- If the laptop goes quiet for 2 s or disconnects, the **Pi itself** sends 0x84 STOP to both nodes.
- The page sends a heartbeat every 0.5 s; 2 s without it → STOP all.
- Motors are never switched off automatically, except when a STOP is not obeyed.
- The B206mini is **receive only**: RX2, TX/RX port unconnected, input ≤ −15 dBm.

---

## Part 1: Raspberry Pi

1. Copy the relay (it needs only Python 3, no packages):
   ```bash
   scp can_relay.py can-relay.service groundstationrpi@<pi-ip>:~/gs/
   ```
2. Bring up the CAN bus (1 Mbps, **not** 500k):
   ```bash
   sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
   ip -details link show can0          # state ERROR-ACTIVE, bitrate 1000000
   ```
3. Find the Pi's IP (you type it on the laptop):
   ```bash
   hostname -I
   ```
4. Self-test (only reads 0x0043 from node 1 and node 2, moves nothing):
   ```bash
   python3 ~/gs/can_relay.py --selftest          # expect: PASS node 1, PASS node 2
   ```
5. Run the relay, accepting only your laptop:
   ```bash
   python3 ~/gs/can_relay.py --allow <laptop-ip>
   ```
   Leave this terminal open. It logs connects/disconnects and every `WATCHDOG ... 0x84 STOP` line.
6. Optional: start it automatically with systemd. Edit `User`, the paths and `LAPTOP_IP_HERE` in
   `can-relay.service` first, then:
   ```bash
   sudo cp ~/gs/can-relay.service /etc/systemd/system/
   sudo systemctl daemon-reload && sudo systemctl enable --now can-relay
   journalctl -u can-relay -f
   ```

If can0 is down, the relay says so and prints the `ip link` command above.

## Part 2: Ubuntu laptop

1. Activate radioconda (UHD 4.11 already works there):
   ```bash
   conda activate base
   pip install pyubx2==1.4.0 pyserial        # numpy comes with radioconda
   ```
2. Plug in the B206mini (USB-C) and check it:
   ```bash
   uhd_find_devices                          # must list the B206mini
   ```
   Use RX2 only, with nothing on TX/RX. Do not run any TX tools.
3. GNSS heading kit (optional): it shows up as `/dev/ttyACM*`. Stop ModemManager grabbing it, once:
   ```bash
   echo 'ATTRS{idVendor}=="1546", ENV{ID_MM_DEVICE_IGNORE}="1"' | sudo tee /etc/udev/rules.d/99-ublox-mm-ignore.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   sudo usermod -aG dialout $USER            # then log out/in once
   ```
4. Run the app:
   ```bash
   python3 app.py --pi <pi-ip>               # add --gnss none if the kit is not plugged in
   ```
5. Open **http://localhost:8080**. The header shows the Pi relay, motors, SDR, GNSS, bus voltage and STOP.
6. Logs are written to `logs/` where you started the app:
   - `sun_<time>.json` + `.csv`: every Sun step, fit, Y and G/T
   - `power_<date>.csv`: every SDR reading with X, Y, az, el

Try it first without hardware: `python3 app.py --sim --sim-clock 2026-11-09T09:30:00+08:00`.

Before the Sun steps, set `BORESIGHT_OFFSET["toward"]` in `config.py` (offset dish: which way the beam
leans at HOME, as +X/−X/+Y/−Y), then `"confirmed": True`. The Sun panel warns until you do.

---

## Tomorrow's field test checklist (you do these, one at a time)

Before every step that moves the dish: **the area around the dish is clear and someone has a hand on
the ePower switch**. STOP = the red button, Space or Esc.

| # | Do | Pass when |
|---|----|-----------|
| 1 | **Relay self-test** on the Pi: `python3 can_relay.py --selftest` | `PASS node 1`, `PASS node 2` |
| 2 | **App connects**: start the relay (`--allow <laptop-ip>`), then `python3 app.py --pi <pi-ip>`, and open the page | Pi relay **connected**, Motors **off**, bus ≈ 47.5 V. Both axes show angle, raw counts, temperature. Nothing moved. |
| 2b | **Watchdog** (optional, motors still off): Ctrl-C the app | The Pi terminal prints `WATCHDOG ... 0x84 STOP`. Restart the app. |
| 3 | **Start**: confirm "support the load" | Setup → 1 s → X +0.5° and back → Y +0.5° and back. Motors **holding**, both states "holding". |
| 4 | **W, then W again**: the **first real STOP-mid-move test**. Press W, let X move ~2 s, press W again. | Key turns amber then green. After the second press X stops and holds, and the message says "X stopped (holding) after … s". If the banner says **"STOP (0x84) not obeyed"**, X was switched OFF: support the load and **report it** before going on. |
| 4b | Same for S, A, D. Try 0.5/1.0/1.5 °/s once. | Each key moves the right way: W = X+ (tilts South), D = Y+ (tilts East). Note whether "live change accepted" appears. |
| 5 | **Home** | Button green while moving. Both axes 0.000°, error shown in counts (expect within a few counts). |
| 6 | **SDR at cold sky**: Sun / G/T tab, dish pointing well away from the Sun | SDR chip "ready" with a steady dBFS value and **no "SATURATED"**. If saturated, lower `SDR.gain_db` in config.py. |
| 7 | **GNSS heading** (if the kit is fitted): wait for the GNSS chip to show **RTK FIXED** | Method 1 radio button becomes available. |
| 8 | **Sun method 1 or 2** (Sun 20–75° up; best 08:30–10:30 or 15:00–17:00). Pick a method and press Start Sun find. | Method 1: ψ from GNSS within a few degrees of what you expect. Method 2: the ring scan plot shows a clear peak (+3 dB or more) and gives ψ. |
| 9 | **Peak-up** (runs automatically) | Cold sky, then X and Y cross-scans with Gaussian fits. Result card: dX, dY, FWHM ≈ 1.5–1.8°, peak rise in dB, refined ψ. |
| 10 | **Track** (runs automatically, ≤ 120 s) | Power stays at the peak. At 120 s the dish moves 10° off by itself and says "LNB heat rule". |
| 11 | **G/T**: type today's ~15 GHz solar flux (sfu) and press Measure G/T | P_sun 10 s, then P_cold 10 s at 10° off. Y in dB, then G/T in dB/K. The dish ends 10° off the Sun. |
| 12 | **Home** | Back at 0/0. Copy the `logs/` folder. |

If anything looks wrong: STOP (Space). The motors stay ON and holding. To switch them off, support the
load first, then on the Pi: `cansend can0 641#010000000000` (X) and `cansend can0 642#010000000000` (Y).
