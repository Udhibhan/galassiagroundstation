# Context handoff: X-Y gimbal current characterisation (14 Nov 2026)

You're picking up a project mid-way. Read this before looking at the attached files.

## The project
- A low-cost Ku-band LEO satellite ground station (NUS UREx project). An X-Y pedestal is driven by two ZeroErr eRob servo drives on a CAN bus: **X = eRob110** (rated 25 A continuous) and **Y = eRob90** (rated 18 A). A Raspberry Pi relays CAN over the network (`can_relay.py`) to a laptop running the Python control software (`gimbal.py`, `app.py`, `config.py`, `zeroerr_protocol.py`).
- My role covers electrical and software integration only. A teammate did the mechanical side.
- The software is single-threaded for CAN: only the `Gimbal` thread sends CAN frames. Everything else gives it goto/track targets.

## The test: `current_char.py`
This is a headless, crash-safe script that measures motor current over the whole ±80° range on both axes. It starts the same Gimbal thread as `app.py`.
- **Phases:** soak (2 min slow moves), 1 baseline at HOME, 2 holding grid, 3a X sweeps, 3b Y sweeps, 3c speed variants, 3d ±60 diagonals, **3e diagonal reach** (both axes together toward (±80, ±80) at 1°/s, past the CAD limit; the operator presses Enter/Space to STOP and that angle is recorded as the diagonal's limit; needs `--allow-diag-reach`, and `--diag=` picks which diagonals), 4a breakaway, 4b reversal, 4c thermal hold, 4d tracking replay.
- **Plans:** `quick`, `full`, `fast`. We used `fast` (sweeps at 1.5°/s, no holding grid).
- `--prev-run <dir>` (repeatable) reuses data from earlier runs instead of redoing phases.
- **Safety and bench rules:**
  - ±80° soft limits, and the CAD collision limit minus 3° is applied to every path.
  - Software current stops of X 15 A and Y 11 A.
  - Aborts on Ctrl+C, a drive dropping out, temperature above 60 °C or a rise of more than 15 °C, bus voltage below 44 V, or loss of the relay link.
  - Polling is gentle: at least 30 ms between requests, 2 Hz while moving, 1 Hz while holding.
  - The script never writes 0x0050/0x0052/0x0053/0x00E8.
  - Acceleration register 0x0419.
- **Important:** register 0x0008 is MOTOR current, not 48 V supply current. "Total" means |I_X| + |I_Y|. No clamp meter was used.
- **Output per run:** `data/current_char/<YYYYmmdd_HHMMSS>/` containing `plan.json`, `samples.csv`, `points.csv`, `events.jsonl` and `summary.json`.
- **Tooling:**
  - `current_char_analysis.py`: shared numbers and model fits.
  - `current_char_merge.py`: merges runs and keeps chosen phases from each.
  - `current_char_report.py`: makes figures (`current_map_total`, `current_map_axes`, `current_vs_angle`, `current_worst_trace`, `current_model_fit`), `current_summary.tex`, `report_fill.md` and an HTML dashboard.
  - `sim_hw.py`: hardware simulator (`--sim`).

## What happened on 14 Nov (all hardware runs, Pi at 172.20.10.2)
| Run | Command / phases | Outcome | Used? |
|---|---|---|---|
| 184758_SIM, 190035_SIM | simulator dry runs | fine | no |
| 191114, 191618 | `--plan fast` | Aborted by a planner bug: X settled at +80.004°, so the "goto +80, +1" was judged outside the CAD limit. Fixed by `clamp_soft` / `POS_TOL` (a measured angle up to 85° now counts as AT 80). | no |
| **192721** | `--plan fast` | 3a X sweep completed. Aborted in 3b at Y = +54.4° when **both drives dropped out together** (error 0x3F1; currents about 1 and 2.4 A, 33–35 °C, bus 47.4 V, so not an overload). | **3a (X sweep)** |
| **194244** | `--phases 1,3b,3d,4a,4c --prev-run 192721` | Baseline and 3b Y sweep good. I pressed Ctrl+C during 3d (±50 diagonal) because it was "too easy". 3d data discarded. | **1 + 3b (Y sweep)** |
| 200410 | 3e,4a,4c | Start failed: node 1 gave no reply to 00 02 | no |
| 200507 | 3e,4a,4c | Ctrl+C at (23, 23) | no |
| 200925 | 3e | Both drives stopped answering at (63.5, 63.0) | no |
| 201602 | 3e | Both drives stopped answering at (60.5, 60.3) | no |
| **201825** | 3e (+X+Y) | Operator STOP at **(+71.99, +72.00)**, which is the real +X+Y limit. Coasted to rest at about (73.1, 73.1). Then lost CAN from both drives 0.5 s into the return leg. | **3e +X+Y** |
| **202836** | `--phases 3e,4a,4c --allow-diag-reach --diag=-X-Y,+X-Y,-X+Y --prev-run 192721 --prev-run 194244` | Completed and returned to HOME | **3e ×3, 4a, 4c, soak** |

**Recurring fault (unresolved):** both drives drop off the CAN bus at the same moment. Every time, Y was high and positive (Y about 54–73°). Suspected causes are a cable or connector pulled at high +Y, a CAN bus-off, or the shared 48 V / ePower supply. It is not overcurrent or overheating.

## Final merged dataset: `data/current_char/20261114_combined/`
This is merged from 192721 [3a], 194244 [1, 3b], 201825 [3e +X+Y] and 202836 [3e ×3, 4a, 4c, soak]. It has 2,390 samples, and the figures, `summary.json` and `report_fill.md` were generated from it. Key results:
- **Peak X current:** 7.8 A at (+67, −67) during the diagonal reach, which is 31% of the X rating.
- **Peak Y current:** 5.2 A at Y = +43 during the Y sweep, which is 29% of the Y rating.
- **Peak total current:** 9.5 A.
- **Distance from the software stops:** no case came within 20% of a stop. The highest were X at 52% of 15 A and Y at 48% of 11 A.
- **Model** I = F·sgn(v) + A·sinθ + B·cosθ, giving R² ≥ 0.99 and residuals of 0.2–0.3 A:
  - **X:** F = 1.66 A, gravity amplitude 5.15 A, balanced at +0.5°, so the centre of gravity is on the dish axis. The 7 Nov X-only estimate is about 30% higher and is superseded by this one.
  - **Y:** F = 1.26 A, gravity amplitude 3.61 A, balanced at **−40°**. The centre of gravity is offset (offset feed arm and LNB), so Y holds 2.3 A even at HOME and its peak is at about +50°. Re-balancing would cut the Y peak by about 18%.
- **Diagonal limits (operator STOP):** (+72.0, +72.0), (−74.3, −74.4), (+73.9, −74.0), (−75.8, +75.8). Three of the four are within about 1° of the CAD prediction.
- **Not done:**
  - X/Y coupling: only one sweep line per axis.
  - 48 V supply current: no clamp meter.
  - Reversal test (4b).
  - The +X+Y return leg was not recorded.

## Report status
- The UREx report is `urex_report/report.tex`, built with tectonic. Section 4.3 (current characterisation) and Appendix C were **hand-finalised**, so don't re-run `current_char_report.py --fill-report`, because the placeholders are gone. Edit the .tex directly.
- Final PDFs are in `UREx_submission/` (report, executive summary, logbook).
- Items I'm deliberately leaving for later: the teammate's surname, extra acknowledgements, the repo link, and reference access dates.
- Future work listed in the report: (1) find the cause of the dual-drive drop-out by logging relay CAN traffic and the 48 V line during repeated high-Y moves; (2) confirm the acceleration unit and retest at 2°/s²; (3) re-balance Y and repeat the sweeps at several positions of the other axis, with a clamp meter.

## Attached
The log, plus the files from the session. These are the run folders and/or `20261114_combined/` (summary.json, report_fill.md, figures) and the scripts `current_char*.py`.

## What I want from you
<fill in>
