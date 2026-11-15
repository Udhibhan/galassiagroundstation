#!/usr/bin/env python3
"""Headless twin of the TX GUI: the same flowgraph (tx_unified.py, generated from
tx_unified.grc), the same options, no window.

  # TV-1 as rev 1.0 GSSE, 20 passes, Link X 2 MBd, to a file with 3 kHz CFO at 8 dB
  python3 tx_cli.py --preset linkx2 --files tv1_pattern_64k.bin --repeat 20 --output file \\
      --output-file /tmp/linkx.sc16 --cfo-hz 3000 --esn0-db 8 --gap-dummies 1
  # a simulated pass from a TLE, starting 60 s before TCA, on the B205mini
  python3 tx_cli.py --files ./outbox --tle sat.tle --doppler-start-utc 2026-10-25T02:59:00Z

Ends when the payload is finished (--repeat N, or --duration for a file), else at
--duration / Ctrl-C.
"""
import inspect
import os
import signal
import sys
import threading
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt5 import Qt  # noqa: E402

import g5gs  # noqa: E402
from g5gs import config as C  # noqa: E402
import tx_unified  # noqa: E402


def main():
    p = tx_unified.argument_parser()
    p.description = __doc__
    p.set_defaults(panels="none")
    p.add_argument("--run-for", type=float, default=0,
                   help="stop after this many wall-clock seconds (0 = until the payload ends "
                        "or Ctrl-C)")
    p.add_argument("--stats-every", type=float, default=5.0)
    o = p.parse_args()
    qapp = Qt.QApplication(sys.argv[:1])  # noqa: F841
    sig = inspect.signature(tx_unified.tx_unified.__init__)
    tb = tx_unified.tx_unified(**{k: getattr(o, k) for k in sig.parameters if k != "self"})
    g5gs.panels.apply_panels(tb, tb.panel_groups, o.panels)
    C.log_event(tb.cfg_text, "TX")
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    done = threading.Event()
    tb.start()
    t0 = time.time()
    threading.Thread(target=lambda: (tb.wait(), done.set()), daemon=True).start()
    nxt = t0 + o.stats_every
    while not done.is_set() and not stop.is_set():
        if o.run_for and time.time() - t0 >= o.run_for:
            break
        done.wait(0.2)
        if time.time() >= nxt:
            nxt += o.stats_every
            fr = tb.dvbs2_tx_0.stats()
            C.log_event(f"{tb.tx_payload_0.describe_progress()} | PLFRAMEs {fr['data']} data "
                        f"{fr['dummy']} dummy | {tb.tx_impairments_0.stats()['text']} | "
                        f"{tb.tx_sink_0.describe()}", "TX")
    if not done.is_set():
        # only the waiter thread calls tb.wait(): two concurrent wait() calls on one top block
        # can crash GNU Radio's teardown
        tb.stop()
        done.wait()
    fr = tb.dvbs2_tx_0.stats()
    C.log_event(f"done after {time.time() - t0:.1f} s: {fr['data']} data + {fr['dummy']} dummy "
                f"PLFRAMEs; {tb.tx_sink_0.describe()}", "TX")
    return 0


if __name__ == "__main__":
    sys.exit(main())
