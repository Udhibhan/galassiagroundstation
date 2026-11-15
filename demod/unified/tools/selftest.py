#!/usr/bin/env python3
"""Offline self-test of the unified RX/TX (no radio): TX -> IQ file -> RX loopbacks.

  python3 tools/selftest.py --work "/media/$USER/Extreme SSD/g5_selftest"   [--quick]

Each case makes an IQ file with tx_cli.py and decodes it with rx_cli.py; PASS = every file
hash matches (rx_cli exit code 0). Needs ~10 GB in --work (files are deleted unless --keep).
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TV = os.path.expanduser("~/g5/part2/link_x_test/test_vectors")
TLE = os.path.expanduser("~/g5/part2/doppler_test/tle")
PY = sys.executable


def run(argv, log):
    t = time.time()
    with open(log, "w") as f:
        rc = subprocess.call(argv, stdout=f, stderr=subprocess.STDOUT, cwd=ROOT)
    return rc, time.time() - t


def grep(log, *keys):
    out = []
    for line in open(log, errors="replace"):
        if any(k in line for k in keys):
            out.append(line.split("EVENT: ")[-1].strip())
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--work", required=True,
                    help="scratch folder (regular files only, so an exFAT SSD is fine)")
    ap.add_argument("--quick", action="store_true", help="skip the slower cases")
    ap.add_argument("--keep", action="store_true", help="keep the IQ files")
    a = ap.parse_args()
    w = os.path.abspath(os.path.expanduser(a.work))
    os.makedirs(w, exist_ok=True)
    outbox = os.path.join(w, "outbox")
    os.makedirs(outbox, exist_ok=True)
    for n in ("tv1_pattern_64k.bin", "tv3_blocks_1M.bin"):
        shutil.copy(os.path.join(TV, n), outbox)
    with open(os.path.join(w, "outbox.sha256"), "w") as f:
        subprocess.call(["sha256sum", "tv1_pattern_64k.bin", "tv3_blocks_1M.bin"], cwd=outbox,
                        stdout=f)
    tv1 = os.path.join(TV, "tv1_pattern_64k.bin")
    tv1h = os.path.join(TV, "tv1.sha256")

    cases = [
        # name, tx options, rx options, hashes
        ("field1 rev1.0 8 dB", ["--preset", "field1", "--files", tv1, "--repeat", "5",
                                "--cfo-hz", "2000", "--esn0-db", "8"], ["--preset", "field1"], tv1h),
        ("linkx2 rev1.0 dummies 8 dB", ["--preset", "linkx2", "--files", tv1, "--repeat", "10",
                                        "--gap-dummies", "1", "--cfo-hz", "3000", "--esn0-db", "8"],
         ["--preset", "linkx2"], tv1h),
        ("ops20 rev1.2 2 files, dropout, 6 dB",
         ["--preset", "ops20", "--files", outbox, "--stack", "rev1.2", "--repeat", "3",
          "--cfo-hz", "-4000", "--esn0-db", "6", "--schedule", "0.3:off,0.35:on"],
         ["--preset", "ops20"], os.path.join(w, "outbox.sha256")),
        ("linkx2 low SNR 3 dB (post-AGC)", ["--preset", "linkx2", "--files", tv1, "--repeat", "10",
                                            "--cfo-hz", "1000", "--esn0-db", "3"],
         ["--preset", "linkx2"], tv1h),
        ("linkx2 TCA pass + aged TLE pre-comp",
         ["--preset", "linkx2", "--files", tv1, "--repeat", "20", "--cfo-hz", "5000",
          "--esn0-db", "8", "--tle", os.path.join(TLE, "overhead.tle"),
          "--doppler-start-utc", "2026-10-25T02:59:50Z"],
         ["--preset", "linkx2", "--tle", os.path.join(TLE, "aged_5s_3km.tle")], tv1h),
    ]
    if not a.quick:
        cases.append(("ops20 udpraw (Branch C1)",
                      ["--preset", "ops20", "--files", tv1, "--stack", "udpraw", "--repeat", "3",
                       "--esn0-db", "10"],
                      ["--preset", "ops20", "--udp-raw-out", os.path.join(w, "c1.udpraw"),
                       "--udp-raw-ref", tv1], None))
    results = []
    for i, (name, txo, rxo, hashes) in enumerate(cases):
        iq = os.path.join(w, f"case{i}.sc16")
        rc_tx, t_tx = run([PY, "tx_cli.py", "--output", "file", "--output-file", iq] + txo,
                          os.path.join(w, f"case{i}_tx.log"))
        rx = [PY, "rx_cli.py", "--mode", "file", "--input-file", iq,
              "--out-dir", os.path.join(w, f"case{i}_out")] + rxo
        if hashes:
            rx += ["--verify-hashes", hashes]
        rlog = os.path.join(w, f"case{i}_rx.log")
        rc_rx, t_rx = run(rx, rlog) if rc_tx == 0 else (99, 0)
        ok = rc_tx == 0 and rc_rx == 0 and any("RESULT: PASS" in x for x in grep(rlog, "RESULT"))
        info = grep(rlog, "PL sync:", "BCH:")
        results.append((name, ok, f"tx {t_tx:.0f}s rx {t_rx:.0f}s; " + "; ".join(info)))
        print(f"{'PASS' if ok else 'FAIL'}  {name}  ({results[-1][2]})", flush=True)
        if not a.keep and os.path.exists(iq):
            os.remove(iq)

    # record + monitor rehearsal at 40 MS/s, then decode the recording with its sidecars
    src = os.path.join(w, "rehearse_src.sc16")
    run([PY, "tx_cli.py", "--output", "file", "--output-file", src, "--preset", "ops20",
         "--files", tv1, "--repeat", "40", "--cfo-hz", "3000", "--esn0-db", "8"],
        os.path.join(w, "rehearse_tx.log"))
    recdir = os.path.join(w, "rec")
    shutil.rmtree(recdir, ignore_errors=True)
    rlog = os.path.join(w, "rehearse_rx.log")
    rc, _ = run([PY, "rx_cli.py", "--mode", "record", "--sim-input", src, "--rec-dir", recdir,
                 "--duration", "15", "--out-dir", os.path.join(w, "rehearse_out")], rlog)
    recs = sorted(f for f in os.listdir(recdir) if f.endswith(".sc16")) if os.path.isdir(recdir) else []
    lock = any("LOCK " in x for x in grep(rlog, "LOCK"))
    ok1 = rc == 0 and len(recs) == 1 and lock
    print(f"{'PASS' if ok1 else 'FAIL'}  record + monitor rehearsal 15 s at 40 MS/s "
          f"(exit code {rc}, recording {recs}, live lock seen: {lock})", flush=True)
    results.append(("record + monitor rehearsal", ok1, ""))
    if recs:
        rec = os.path.join(recdir, recs[0])
        dlog = os.path.join(w, "rehearse_decode.log")
        rc, t = run([PY, "rx_cli.py", "--mode", "file", "--input-file", rec, "--verify-hashes",
                     tv1h], dlog)
        ok2 = rc == 0
        print(f"{'PASS' if ok2 else 'FAIL'}  decode the recording ({t:.0f}s; "
              f"{'; '.join(grep(dlog, 'PL sync:', 'BCH:'))})", flush=True)
        results.append(("decode the recording", ok2, ""))
        if not a.keep:
            os.remove(rec)
    if not a.keep and os.path.exists(src):
        os.remove(src)
    n_ok = sum(1 for r in results if r[1])
    print(f"\n{n_ok}/{len(results)} passed; logs in {w}")
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
