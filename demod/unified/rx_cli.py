#!/usr/bin/env python3
"""Headless twin of the RX GUI: the same flowgraph (rx_unified.py, generated from
rx_unified.grc), the same options, no window.

  python3 rx_cli.py --mode file --preset linkx2 --input-file cap.sc16 --verify-hashes tv1.sha256
  python3 rx_cli.py --mode record --rec-dir "/media/$USER/Extreme SSD/pass1" --duration 900
  python3 rx_cli.py --mode file --input-file pass1/g5_20261025T030000Z.sc16 --tle sat.tle

File mode ends at the end of the file; record / live run until --duration or Ctrl-C.
The exit code is 1 if --verify-hashes / --udp-raw-ref failed, else 0.
Panels are off unless --panels is given (they cost CPU and nobody sees them).
"""
import inspect
import os
import signal
import sys
import threading
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")   # the top block is still a QWidget
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt5 import Qt  # noqa: E402

import g5gs  # noqa: E402
from g5gs import config as C, probe  # noqa: E402
import rx_unified  # noqa: E402


def main():
    p = rx_unified.argument_parser()
    p.description = __doc__
    p.set_defaults(panels="none")
    p.add_argument("--duration", type=float, default=0,
                   help="record / live: stop after this many seconds (0 = until Ctrl-C)")
    p.add_argument("--stats-every", type=float, default=5.0, help="status line period, s [5]")
    o = p.parse_args()
    qapp = Qt.QApplication(sys.argv[:1])  # noqa: F841  (Qt widgets need an application)
    sig = inspect.signature(rx_unified.rx_unified.__init__)
    tb = rx_unified.rx_unified(**{k: getattr(o, k) for k in sig.parameters if k != "self"})
    g5gs.panels.apply_panels(tb, tb.panel_groups, o.panels)
    dec = tb.gs_decoder_0
    dec.before_report.append(lambda: probe.print_summary(tb, tb.rs, bool(o.dummy_frames)))
    C.log_event(tb.cfg_text)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    done = threading.Event()
    tb.start()
    t0 = time.time()

    def waiter():
        tb.wait()
        done.set()
    threading.Thread(target=waiter, daemon=True).start()
    next_print = t0 + o.stats_every
    while not done.is_set() and not stop.is_set():
        if o.duration and o.mode != "file" and time.time() - t0 >= o.duration:
            C.log_event(f"--duration {o.duration:g} s reached")
            break
        done.wait(0.2)
        if time.time() >= next_print:
            next_print += o.stats_every
            C.log_event(probe.status_line(tb, tb.rs, bool(o.dummy_frames)))
    if not done.is_set():
        # only the waiter thread calls tb.wait(): two concurrent wait() calls on one top block
        # can crash GNU Radio's teardown
        tb.stop()
        done.wait()
    dec.finish()                # no-op if the flowgraph already ran it
    C.log_event(f"finished after {time.time() - t0:.1f} s")
    return 1 if dec.ok is False else 0


if __name__ == "__main__":
    sys.exit(main())
