# Galassia-5 unified ground station (RX \+ TX, GNU Radio GUI)

*Built 2026-10-24. Combines the part 2 work in one tool:*

| Earlier work | What it contributes here |
| :---- | :---- |
| `gr-dvbs2rx` transient patch (private repo `gr-dvbs2rx-patched`) | the demodulator survives AGC overshoot / cold starts (must be installed) |
| field test (`field_test/`) | CCM PLS filter, dummy frames, AGC3 ref 1.37, `pl_freq_est_period` 17, gr-dvbgse/tap0 TX path, UDP out for dvb-gse |
| post-AGC (`19102026_postagc_results.md`) | second AGC before the PL sync: decodes down to Es/N0 \~2 dB |
| B205mini stress test | `num_recv_frames=512`, master clock choice; 20 MBd can't be demodulated live on this laptop, so record \+ decode afterwards |
| Link X test (`gsrx_v2`) | GSSE / GSUFTP decoding (rev 1.0 and rev 1.2, UDP-raw fallback) **inside** the flowgraph |
| Doppler test (`doppler_precomp.py`) | optional TLE pre-compensation (SGP4), pass prediction, and a TLE Doppler simulator in the TX |

Nothing in the older folders was changed; their code is copied into `g5gs/` (`g5gs/gsrx/` and `g5gs/doppler_lib.py` are byte-identical copies).

## Setup on a new laptop

`setup_grc.sh` does **not** build anything: the patched gr-dvbs2rx is built and installed by hand, once per laptop.

**1\. GNU Radio 3.10** (with gr-dtv and gr-uhd; Ubuntu 24.04: the GNU Radio PPA / apt packages). gr-dvbs2rx must be built against the GNU Radio version installed on *that* laptop (a copy built for 3.10.9 does not load on 3.10.12).

**2\. The patched gr-dvbs2rx** (private repo: `gh auth login` or a token first):

```sh
git clone --recursive https://github.com/Smolrdwarf/gr-dvbs2rx-patched.git
cd gr-dvbs2rx-patched            # default branch transient-fix = the patched one
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc) && ctest
sudo make install && sudo ldconfig
```

`--recursive` matters (the `cpu_features` submodule). If cmake complains, the repo README lists the build dependencies (pybind11 etc.). Do **not** use `part2/dvbs2rx_patch/install_patched.sh` on other laptops: it hard-codes Compewter's paths and old installs.

**3\. Python extras:** `pip install --user sgp4` (TLE features only), `pyqtgraph` for the Doppler plot (`sudo apt install python3-pyqtgraph` if missing), PyYAML (normally present). gr-dvbgse only if that laptop will use the old tap0 TX path.

**4\. This folder:** copy `unified/` over, then

```sh
cd unified
./setup_grc.sh       # makes `import g5gs` work, adds grc/ to GRC's block path, checks gr-dvbs2rx
```

No sudo. Restart GNU Radio Companion afterwards: the blocks are under **G5 Ground Station**.

**The gr-dvbs2rx check** (also on its own: `python3 tools/check_dvbs2rx.py`) is read-only. It looks at the library GNU Radio actually loads and reports:

- `from gnuradio import dvbs2rx` failing (not installed, or built for another GR version);  
- **NOT PATCHED**: the loaded library lacks the patch's "Input power" message;  
- more than one `gnuradio/dvbs2rx` Python package or library copy (a stale unpatched copy under `/usr` shadowed the `/usr/local` build on Compewter: remove it, after checking `dpkg -S <path>` says no package owns it);  
- a pip-installed numpy 2.x shadowing Ubuntu's (breaks `from gnuradio import gr`: `pip uninstall numpy`). `python3 tools/check_dvbs2rx.py --lib <file.so>` checks one library file for the patch.

## Starting it

| Way | Command |
| :---- | :---- |
| Launcher (preset dropdown, file pickers, shows the command) | `python3 g5_launch.py` |
| RX GUI directly | `python3 rx_unified.py --preset ops20 --mode record --rec-dir "/media/$USER/Extreme SSD/pass1"` |
| RX headless (same flowgraph, no window) | `python3 rx_cli.py --mode file --input-file X.sc16 --tle sat.tle` |
| TX GUI / headless | `python3 tx_unified.py ...` / `python3 tx_cli.py ...` |
| Edit the flowgraphs | `gnuradio-companion rx_unified.grc` (then Generate, or `grcc -o . rx_unified.grc`) |

`rx_unified.py --help` / `tx_unified.py --help` list every option (each one is a GRC `parameter` block). Start the launcher from a terminal: the GUIs log there.

**Radio settings (RX):** `--centre-freq` (IF, default 1195e6), `--rx-gain` (also a slider), `--antenna RX2|TX/RX` (also the **RX port** dropdown in the GUI, switchable while running; launcher: RX port), `--device-args serial=...` when more than one B205mini is plugged in (empty \= the first one UHD finds). The radio is opened and started automatically in record / live mode.

## Presets (`--preset`; any field can be overridden)

| Preset | Symbol rate | sps | Sample rate | Use |
| :---- | :---- | :---- | :---- | :---- |
| `ops20` (default) | 20 MBd | 2 | 40 MS/s | operations |
| `linkx2` | 2 MBd | 4 | 8 MS/s | the Link X test settings |
| `field1` | 1 MBd | 2 | 2 MS/s | the field-test settings |

All presets: QPSK 1/4, normal frames, pilots, roll-off 0.2, CCM, dummy frames on. Override with `--modcod 1..17` (or `QPSK3/4`, `8PSK2/3`), `--frame-size short`, `--sym-rate`, `--sps` (even). Receiver defaults are the validated ones (AGC3 ref 1.37, post-AGC 1.0 / 1e-3 / 1e-5, period 17, LDPC 25 iterations).

## Receiver modes (`--mode`)

| Mode | What runs | When |
| :---- | :---- | :---- |
| `record` (default) | B205mini \-\> **full-rate sc16 recording** \+ spectrum / level / overflows \+ a **duty-cycled live demod** (1 s every 5 s at 40 MS/s; continuous at \<= 20 MS/s) | passes at 20 MBd |
| `live` | B205mini \-\> full demod; samples are dropped if the CPU can't keep up (recording optional) | \<= \~20 MS/s (e.g. `linkx2`) |
| `file` | a recording / IQ file, as fast as the CPU allows (0.6x real time at 40 MS/s) | decoding after the pass |

Why: on this laptop the demod runs at \~0.4-0.6x real time at 40 MS/s (symbol sync is the bottleneck, stress test 2026-10-04), so at 20 MBd a pass is recorded and decoded afterwards. The duty gate decouples the demod from the radio so a slow demod never causes overflows.

### Recording

- Destination: `--rec-dir` (GUI: launcher field). Files are `<prefix>_<UTC>.sc16` (= `rx_samples_to_file --type short` format) plus `.start` (UTC of the first sample, from UHD's `rx_time`; the radio clock is set from the host, so keep NTP on) and `.json` (rate, tuning, preset, MODCOD, and a time tag after every overflow so a Doppler decode stays on time).  
- 40 MS/s \= 160 MB/s \= 9.6 GB/min. The status panel shows the free space and the minutes left.  
- **Recording stops by itself, with a warning, when less than 2 GB is left** on that disk; the monitor keeps running. "Record to disk" in the GUI starts / stops a new file at any time.  
- Tested: the SanDisk Extreme SSD (exFAT) writes 465 MB/s; recording held 1.000x real time at 40 MS/s while the duty demod ran (1.2-1.6 cores in total).

### Doppler pre-compensation (optional)

Leave `--tle` empty: no pre-comp, nothing else changes. With `--tle sat.tle` (2/3-line TLE, or CelesTrak OMM .json/.csv; `--sat` picks one set):

- the carrier Doppler predicted by SGP4 is removed sample by sample before the AGC (phase-continuous, light time included);  
- time comes from `rx_time` tags: UHD's when live, the recording's `.start` / `.json` in file mode (`--start-utc` overrides);  
- the **Doppler / pass** tab shows elevation / azimuth / range, the predicted Doppler, the residual the PL sync measures, AOS / TCA / LOS countdown and the next passes.  
- `--precomp-sign -1` only for an inverted IF that is not undone with `--conj`; `--precomp-dt` (time shift), `--precomp-offset-hz` (known LO error).  
- At 20 MBd the receiver survives the worst-case CRISP pass without it (Doppler test s2\_20M); at 2 MBd it is required. A wrong sign shows at once: the residual doubles.

### Decoded files (in-graph GSSE / GSUFTP)

- `--stack auto` (rev 1.0 Link X: GSUFTP in GSSE; rev 1.2 SX: GSUFTP / UDP / IPv4 in GSSE).  
- Files go to `--out-dir` (default: `<input>_out` in file mode, `<rec-dir>/downlink_<UTC>` live) with `MANIFEST.txt`. `--verify-hashes tv.sha256` checks them (CLI exit code 1 on FAIL).  
- Extras: `--bbframe-file` (raw BBFRAMEs, for `g5gs/gsrx/gs_rx.py --from-file`), `--udp-forward 127.0.0.1:2000` (dvb-gse / recv\_dir, field-test tun0 transfers), `--udp-raw-out/--udp-raw-ref` (Branch C1), `--report-json`.  
- Split files: `python3 g5gs/gsrx/gsuftp_combine.py <out_dir> <dest>`.

### GUI panels

Spectrum (input \+ after PL sync), constellation, status, Doppler / pass, files. The **Panels** checkboxes switch each one off at run time: its widgets hide and the Copy "valve" in front of its Qt sink stops passing samples, so a switched-off panel costs no CPU. `--panels none` (or a list, e.g. `--panels status,files`) sets the start state; the CLIs default to none.

## Transmitter

```
payload (gsse | bbframes | gse_tap0) -> DVB-S2 (gr-dtv) + dummy PLFRAMEs + RRC
   -> [generate | replay an IQ file] -> impairments -> B205mini | IQ file
```

- `--payload gsse --files <file or folder> --stack rev1.0|rev1.2|udpraw`: exactly what the Link X / SX sends (tx\_simulator's encoder), looped `--repeat` times (0 \= forever) then 100 empty BBFRAMEs so the receiver's LDPC releases its last frames.  
- `--payload gse_tap0`: the old gr-dvbgse path (run `field_test/setup_tx_ifaces.sh` first).  
- Dummy frames: `--lead-dummies 200`, `--gap-dummies N` after each data frame (Link X idle).  
- Impairments: carrier on/off checkbox, level slider (digital fades / steps), CFO slider, `--esn0-db` (AWGN, Es/N0 after the matched filter), `--schedule "5:off,6:on,10:-20dB,12:0dB"` (signal time, repeatable), `--tle sat.tle --doppler-start-utc 2026-10-25T02:59:00Z` (carrier Doppler of that pass; the file's `.start` is set to the same time, so the RX pre-comp lines up).  
- B205mini: TX/RX port, `--tx-gain`, master clock chosen for the rate, 512 send frames. The gr-dtv chain generates \~47 Msym/s, so 20 MBd runs live; with impairments the chain makes 40 MS/s at \~1.9x real time.  
- `--output file --output-file X.sc16 [--duration s] [--realtime 1]`.

## Operations runbook (a pass at 20 MBd)

1. **Before:** `timedatectl` shows NTP synchronised. SSD mounted with enough space (`df -h`): a 10-min pass is \~96 GB. Fresh TLE. Optional pass list: `python3 g5gs/doppler_lib.py passes --tle sat.tle --hours 24`.  
2. **Start** (a few minutes before AOS): launcher \-\> Receiver, preset `ops20`, mode Record \+ monitor, recording folder on the SSD, TLE (optional), Start GUI. Or `python3 rx_unified.py --mode record --rec-dir "/media/$USER/Extreme SSD/<pass>" --tle sat.tle`.  
3. **Watch:** input level (peaks below \-1 dBFS; reduce RX gain if CLIPPING), overflows 0, recording growing, disk time left, PL sync LOCKED in the monitor windows, carrier offset sensible (with a TLE: residual small and steady).  
4. **After LOS:** close the window (the recording closes, sidecars are final).  
5. **Decode:** launcher \-\> mode Decode a file \-\> pick the `.sc16` (the TLE is optional; time comes from the sidecar), or `python3 rx_cli.py --mode file --input-file <rec>.sc16 --tle sat.tle --verify-hashes x.sha256`. Files appear in `<rec>_out/` with SHA-256s.

**Bench loopback (two B205minis, 40 dB attenuation):** TX laptop `python3 tx_unified.py --preset linkx2 --files <outbox> --tx-gain 50`; RX laptop `python3 rx_unified.py --preset linkx2 --mode live --rx-gain 30`. **Rehearsal without a radio:** `--sim-input file.sc16` replays an sc16 file in real time in place of the B205mini (record / live modes), including the recorder and the duty demod.

## Testing

`python3 tools/selftest.py --work "/media/$USER/Extreme SSD/g5_selftest"` runs TX \-\> file \-\> RX loopbacks (all presets, rev 1.0 / 1.2 / UDP-raw, dummy frames, a dropout, 3 dB, a TCA pass with an aged TLE) plus a 40 MS/s record \+ monitor rehearsal and the decode of its recording. `python3 g5gs/gsrx/test_vectors.py` checks the GSSE/GSUFTP layer.

## Layout

```
g5_launch.py            launcher (presets, file pickers, command preview)
rx_unified.grc/.py      RX GUI (GRC source / generated)    rx_cli.py  headless twin
tx_unified.grc/.py      TX GUI                              tx_cli.py  headless twin
setup_grc.sh            one-time setup
g5gs/                   the blocks (pure Python)
  config.py             presets, MODCOD tables, B205mini settings
  rx_source.py          B205mini / file / rehearsal input, recorder hookup, time tags
  recorder.py           sc16 recorder, sidecars, 2 GB stop, overflow tracking
  gate.py               duty gate (pass / drop / duty)
  doppler_block.py      TLE pre-comp block       doppler_lib.py  = field_test/doppler_precomp.py
  agc.py                front AGC + post-AGC     gs_decoder.py   in-graph gs_rx
  tx.py                 payload, DVB-S2 TX + dummies, replay, impairments, sink
  panels.py, probe.py   Qt panels, panel switch, counters / summaries
  gsrx/                 = link_x_test/gsrx_v2 (v2.2)
grc/                    GRC block definitions (made by tools/make_blocks.py)
tools/                  make_blocks.py, make_grc.py (first version of the .grc files), selftest.py,
                        check_dvbs2rx.py (is the patched gr-dvbs2rx the one in use?)
```

The standard gr-dvbs2rx blocks are visible in `rx_unified.grc`; the gr-dtv TX chain is inside the `G5 DVB-S2 TX` block because gr-dtv's GRC blocks only take the MODCOD as fixed enums. `tools/make_grc.py` overwrites the .grc files: after editing them in GRC, don't rerun it. Gotchas found while building: GRC parameter ids must not be Mako words (`loop`); write dicts as `{'k': v}`, not `dict(k=v)`, in GRC expressions; Python blocks at tens of MS/s need big chunks (`set_output_multiple(65536)`), otherwise GIL hand-offs between block threads cost 6x.

- &nbsp;