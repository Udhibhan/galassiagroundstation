"""GSSE / GSUFTP decoder inside the flowgraph (gs_rx v2.2 Pipeline, vendored in g5gs/gsrx).

Input is the BBFRAME byte stream from bbdescrambler_bb (K_BCH/8 bytes per frame).
Steps: strip BBHEADER (with confirmed re-sync), GSSE deframe + CRC-32, then
  rev 1.0: GSUFTP directly
  rev 1.2: IPv4, UDP, GSUFTP
and write the files to out_dir (+ MANIFEST.txt, SHA-256/MD5).

Optional side outputs:
  bbframe_file  raw BBFRAMEs (re-run gs_rx.py --from-file later)
  udp_forward   host:port, one BBFRAME per datagram (dvb-gse "UDP complete" / recv_dir for
                the field-test tun0 transfers)
  udp_raw_out   Branch C1: plain UDP payloads instead of GSUFTP (+ udp_raw_ref check)
At the end (flowgraph stop / end of file): link report, hash check against verify_hashes,
JSON report. `finished` is then True and `ok` holds the hash / UDP-raw result.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import threading
import time

import numpy as np
from gnuradio import gr

from . import config as C
from . import gsrx  # noqa: F401  (puts the gsrx folder on sys.path)
import bbframe  # noqa: E402
import gs_rx  # noqa: E402


def read_hashes(path):
    want = {}
    if path and os.path.exists(path):
        for line in open(path):
            parts = line.split()
            if len(parts) >= 2:
                want[os.path.basename(parts[-1].lstrip("*"))] = parts[0].lower()
    return want


class gs_decoder(gr.sync_block):
    def __init__(self, frame_len=2001, stack="auto", out_dir="./downlink", verify_hashes="",
                 bbframe_file="", udp_forward="", udp_raw_out="", udp_raw_ref="",
                 report_json="", dump_payloads="", isi=-1, udp_port=0, log=C.log_event):
        gr.sync_block.__init__(self, "g5_gs_decoder", in_sig=[np.uint8], out_sig=[])
        out_dir = C.path(out_dir)
        verify_hashes = C.path(verify_hashes)
        bbframe_file = C.path(bbframe_file)
        udp_raw_out = C.path(udp_raw_out)
        udp_raw_ref = C.path(udp_raw_ref)
        report_json = C.path(report_json)
        dump_payloads = C.path(dump_payloads)
        self.log = log
        self.flen = int(frame_len)
        self.a = argparse.Namespace(
            bbframe_len=self.flen, isi=None if isi is None or isi < 0 else int(isi),
            no_crc8=False, max_gsse_frame=16 << 20, no_ip_checksum=False, no_udp_checksum=False,
            out=out_dir or "./downlink", quiet=True, stack=stack or "auto", dump_gsse=None,
            dump_payloads=dump_payloads or None, dump_max_bytes=4096,
            udp_raw_out=udp_raw_out or None, udp_raw_ref=udp_raw_ref or None,
            udp_port=udp_port or None, input="bbframe", verify_hashes=verify_hashes or None,
            report_json=report_json or None)
        os.makedirs(self.a.out, exist_ok=True)
        self.pipe = gs_rx.Pipeline(self.a)
        self.want = read_hashes(verify_hashes)
        self.bbf = open(bbframe_file, "wb", buffering=8 << 20) if bbframe_file else None
        self.bbframe_file = bbframe_file
        self.udp = None
        if udp_forward:
            host, port = udp_forward.rsplit(":", 1)
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_dst = (host, int(port))
            self._ubuf = b""
        self.before_report = []       # callables run at the end, before the link report
        self.files = []               # [(file_id, info, verdict)] in completion order
        self._seen = set()
        self.finished = False
        self.ok = None
        self.bytes_in = 0
        self._header_logged = False
        self._lock = threading.Lock()
        self.t0 = time.time()
        log(f"GSSE/GSUFTP decoder: {self.flen} B BBFRAMEs, stack {self.a.stack}, files -> "
            f"{os.path.abspath(self.a.out)}"
            + (f", hashes from {verify_hashes} ({len(self.want)})" if verify_hashes else "")
            + (f", BBFRAMEs -> {bbframe_file}" if bbframe_file else "")
            + (f", UDP -> {udp_forward}" if udp_forward else ""))

    def work(self, input_items, output_items):
        x = input_items[0]
        data = x.tobytes()
        self.bytes_in += len(data)
        if self.bbf:
            self.bbf.write(data)
        if self.udp is not None:
            buf = self._ubuf + data
            k = len(buf) // self.flen * self.flen
            for off in range(0, k, self.flen):
                self.udp.sendto(buf[off:off + self.flen], self.udp_dst)
            self._ubuf = buf[k:]
        with self._lock:
            self.pipe.feed(data)
        if not self._header_logged and self.pipe.bb.first_header:
            self._header_logged = True
            self.log("first BBHEADER: " + bbframe.describe(self.pipe.bb.first_header))
        if len(self.pipe.ftp.completed) != len(self._seen):
            for fid, info in self.pipe.ftp.completed.items():
                if fid in self._seen:
                    continue
                self._seen.add(fid)
                name = os.path.basename(fid)
                verdict = ("" if not self.want else "PASS" if self.want.get(name) == info["sha256"]
                           else "FAIL" if name in self.want else "not in hash list")
                self.files.append((fid, info, verdict))
                self.log(f"FILE {fid}: {info['size']} B sha256 {info['sha256'][:16]}..."
                         + (f" {verdict}" if verdict else ""))
        return len(x)

    # ------------------------------------------------------------------ status
    def stats(self):
        p = self.pipe
        with self._lock:
            return {"bbframes": p.bb.frames, "resyncs": p.bb.resync_events,
                    "partial": p.bb.partial_frames, "gsse_ok": p.df.frames_ok,
                    "gsse_crc_bad": p.df.frames_crc_bad, "gsse_abandoned": p.df.frames_abandoned,
                    "kinds": dict(p.kind), "udp": dict(p.udp),
                    "gsuftp": dict(p.ftp.stats),
                    "partial_files": {k: (len(v["chunks"]), v["total"])
                                      for k, v in p.ftp.partial.items()},
                    "udp_raw_bytes": p.udp_raw_bytes, "bytes_in": self.bytes_in,
                    "files": list(self.files), "want": dict(self.want),
                    "finished": self.finished, "ok": self.ok}

    def stop(self):
        if self.finished:
            return True
        with self._lock:
            self.finish()
        return True

    def finish(self):
        """Report, verify and close. Idempotent."""
        if self.finished:
            return
        self.finished = True
        for fn in self.before_report:
            try:
                fn()
            except Exception as e:              # noqa: BLE001
                print(f"end-of-run hook failed: {e}", flush=True)
        p = self.pipe
        if self.bbf:
            self.bbf.close()
        p.report()
        if p.udp_raw:
            p.udp_raw.close()
        ok = True
        if self.a.verify_hashes:
            ok = gs_rx.verify_hashes(self.a.verify_hashes, p.ftp.completed)
        if self.a.udp_raw_ref:
            ok = gs_rx.verify_udp_raw(self.a.udp_raw_out, self.a.udp_raw_ref) and ok
        self.ok = ok if (self.a.verify_hashes or self.a.udp_raw_ref) else None
        if self.a.report_json:
            with open(self.a.report_json, "w") as f:
                json.dump(p.summary(), f, indent=2, default=str)
            print(f"report written to {self.a.report_json}", flush=True)
