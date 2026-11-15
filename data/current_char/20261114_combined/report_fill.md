# Report text drafted from 20261114_combined (HARDWARE)

## p1

The characterisation was run on 14 November 2026 (plan \texttt{fast}, sweeps at 1.5\textdegree/s). No holding map was measured (fast plan); the run consisted of 1 X sweep, 1 Y sweep and 4 diagonal reaches (both axes together toward $(\pm80,\pm80)$\textdegree{} at 1\textdegree/s, stopped by the operator). Both axes were read at 2.0\,Hz while moving and 1.0\,Hz while holding (achieved medians 1.82 and 0.97\,Hz), and 2{,}390 samples were logged in total. The peak X current was 7.8\,A at X\,=\,+67\textdegree, Y\,=\,-67\textdegree{} (X lifting, Y lifting, during the diagonal reach), 31\% of the drive rating; the peak Y current was 5.2\,A at X\,=\,0\textdegree, Y\,=\,+43\textdegree{} (Y lifting, during the Y sweep), 29\% of the drive rating; the peak total $|I_X|+|I_Y|$ current was 9.5\,A at X\,=\,-47\textdegree, Y\,=\,0\textdegree{} (X lifting, during the X sweep), 24\% of the drive rating. Register 0x0008 is motor current, not 48\,V supply current.

## cap_map

14 November 2026, measured along the X and Y sweep paths at 1.5\textdegree/s and the four diagonal reaches at 1\textdegree/s; no holding grid was run

## cap_move

14 November 2026, sweeps at 1.5\textdegree/s across $\pm$80\textdegree{}; solid lifting towards HOME, dashed lowering

## p2

The X sweeps fit $I = c_0 + F\,\mathrm{sign}(v) - G_\pm\sin\theta$ with $F = 1.66 \pm 0.01$\,A, $G_+ = 5.17 \pm 0.05$\,A and $G_- = 5.12 \pm 0.05$\,A ($R^2 = 0.995$), against 1.65--1.7\,A, 7.6\,A and 6.6\,A in the 7 November X-only fit. For Y, $F = 1.26 \pm 0.01$\,A and $G_\pm = 0.71/4.86$\,A ($R^2 = 0.984$). Coupling between Y position and X current was not tested (fewer than two Y lines). No case came within 20\% of a software stop (highest: X 52\% of 15\,A, Y 48\% of 11\,A). The 48\,V supply current was not measured with a clamp meter. Moving both axes together along the diagonals, the dish was stopped by the operator at $(+72.0, +72.0)$\textdegree; $(-74.3, -74.4)$\textdegree; $(+73.9, -74.0)$\textdegree; $(-75.8, +75.8)$\textdegree, the practical diagonal limits of the mount; the largest total current on the diagonals was 9.5\,A at X\,=\,-65\textdegree, Y\,=\,-65\textdegree{}, on the way back to HOME with both axes lifting. The return leg from +X+Y was not recorded: both drives stopped answering on the CAN bus at the start of that leg and the session was aborted; the reach angle itself was logged before it.

## appx


\noindent Run \texttt{20261114\_combined}, 14 November 2026, command \texttt{python3 current\_char.py --pi 172.20.10.2 --plan fast --phases 3e,4a,4c --allow-diag-reach --diag=-X-Y,+X-Y,-X+Y --prev-run data/current\_char/20261114\_192721 --prev-run data/current\_char/20261114\_194244}. Grid 20\textdegree{} from $-$80 to $+$80\textdegree{} on both axes, serpentine order at 1\textdegree/s, 2\,s settle and 5\,s sample per point; points outside the CAD collision limit minus 3\textdegree{} were skipped, and every path (not only its end point) was kept inside that limit, moving Y towards 0 first between high-$|X|$ poses. Sweeps at 1\textdegree/s; accel/decel register 0x0419. Sampling: both axes 2.0\,Hz moving, 1.0\,Hz holding, 4.0\,Hz on the moving axis for breakaway, 0.1\,Hz in the thermal hold; at least 30\,ms between requests, no polling for 1\,s after an enable or START (0.3\,s after a tracking step on a holding motor); temperature and bus voltage every 10\,s. Abort limits: Ctrl+C, a drive in state 0 that is still off 1\,s later, drive temperature above 60\,\textdegree C or a rise of more than 15\,\textdegree C, bus voltage below 44\,V, or loss of the relay link. Software current stops X 15\,A and Y 11\,A were kept; a trip stops the axis, is logged as a worst case and the step is skipped. The data combine 4 sessions on the same set-up: X sweep from \texttt{20261114\_192721}; baseline, Y sweep from \texttt{20261114\_194244}; diagonal reach from \texttt{20261114\_201825}; diagonal reach, breakaway test, thermal hold from \texttt{20261114\_202836}.
\begin{itemize}
\item Phase soak: 2-minute soak test (slow X and Y moves at the chosen poll rate)
\item Phase 1: baseline at HOME, 60\,s
\item Phase 3a: X sweeps at Y $\in\{-60,-30,0,+30,+60\}$\textdegree
\item Phase 3b: Y sweeps at X $\in\{-60,-30,0,+30,+60\}$\textdegree
\item Phase 4a: breakaway: 30\,s hold then a 5\textdegree{} lift at the worst holding pose, per axis
\item Phase 4c: thermal hold at the worst pose, 0\,min
\item Phase 3e: diagonal reach: straight diagonals toward $(\pm80,\pm80)$\textdegree{} past the CAD limit, stopped by the operator; the angle at the STOP key is taken as that diagonal's limit
\end{itemize}
\noindent Data files (\texttt{data/current\_char/}): \texttt{plan.json} (parameters, HOME counts, configuration, operator notes), \texttt{samples.csv} (every reading; 1 in 10 here as \texttt{samples\_thinned.csv}), \texttt{points.csv} (one row per holding point), \texttt{events.jsonl} (phases, trips, aborts, operator inputs) and \texttt{summary.json} (worst cases and model fits).

## trace_b

total current on the diagonal returns to HOME, both axes lifting (no reversal test in this plan)

