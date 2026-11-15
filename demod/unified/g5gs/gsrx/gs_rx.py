#!/usr/bin/env python3
"""
gs_rx - GomSpace NanoCom Link X / SX ground-side receiver (v2).

Takes BBFRAMEs from a DVB-S2 demodulator (e.g. gr-dvbs2rx over UDP, or a file)
and recovers GSUFTP files.

  BBFRAMEs -> header strip (bbframe.py) -> GSSE deframe + CRC-32 (gsse.py)
           -> [rev 1.2 only: IPv4 defrag + UDP (ipudp.py)] -> GSUFTP (gsuftp.py)

Protocol stack by RF ICD 1045318 revision (--stack):
  rev1.0  current Link X:  GSUFTP packets directly inside GSSE (no IP/UDP)
  rev1.2  later SX:        GSUFTP over UDP over IPv4 inside GSSE
  auto    (default) decide per GSSE payload: valid IPv4 header -> rev1.2 path,
          otherwise try GSUFTP directly -> rev1.0 path, otherwise "unknown".

Typical use:
  python3 gs_rx.py --from-file c2.bbframes --modcod 1 --out ./rx
  python3 gs_rx.py --from-file c2.bbframes --bbframe-len auto --out ./rx \\
          --verify-hashes tv_hashes.sha256 --dump-payloads ./payloads
  python3 gs_rx.py --listen 0.0.0.0:2000 --modcod 1 --out ./rx
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time

import bbframe
import gsse
import gsuftp
import ipudp


class Pipeline:
    """Wires the layers together and keeps the counters."""

    def __init__(self, args):
        self.args = args
        self.bb = bbframe.BBFrameStream(args.bbframe_len, isi=args.isi,
                                        check_crc8=not args.no_crc8)
        self.df = gsse.GsseDeframer(max_frame=args.max_gsse_frame)
        self.ip = ipudp.IPv4Reassembler(verify_checksum=not args.no_ip_checksum)
        self.ftp = gsuftp.GsuftpReceiver(
            args.out, verbose=not args.quiet,
            expected_chunk=gsuftp.CHUNK_SIZE_REV12 if args.stack == "rev1.2" else None)
        self.gsse_dump = open(args.dump_gsse, "wb") if args.dump_gsse else None
        self.payload_dir = args.dump_payloads
        if self.payload_dir:
            os.makedirs(self.payload_dir, exist_ok=True)
        self.n_payload = 0
        self.kind = {"ipv4": 0, "gsuftp_direct": 0, "unknown": 0}
        self.ip_seen = 0
        self.udp = {"ok": 0, "none": 0, "bad": 0, "filtered": 0, "to_gsuftp": 0}
        self.udp_ports = {}
        self.udp_raw = open(args.udp_raw_out, "wb") if args.udp_raw_out else None
        self.udp_raw_bytes = 0
        self.t0 = time.time()
        self.stage = args.input

    def feed(self, data: bytes):
        if self.stage == "bbframe":
            for field in self.bb.feed(data):
                if self.bb.frames == 1 and self.bb.first_header and not self.args.quiet:
                    print("first BBHEADER: " + bbframe.describe(self.bb.first_header))
                self._gsse(field)
        elif self.stage == "gsse":
            self._gsse(data)
        else:                                      # already IP packets
            self._ip(data)

    def _gsse(self, byte_stream: bytes):
        if self.gsse_dump:
            self.gsse_dump.write(byte_stream)
        for payload in self.df.feed(byte_stream):
            self._payload(payload)

    def _payload(self, payload: bytes):
        """One CRC-verified GSSE payload: route by protocol stack."""
        self.n_payload += 1
        stack = self.args.stack
        if stack == "rev1.2" or (stack == "auto" and ipudp.looks_like_ipv4(payload)):
            self.kind["ipv4"] += 1
            self._dump(payload, "ipv4")
            self._ip(payload)
            return
        before = self.ftp.stats["bad"]
        self.ftp.handle_packet(payload)
        if self.ftp.stats["bad"] == before:
            self.kind["gsuftp_direct"] += 1
            self._dump(payload, "gsuftp")
        else:
            self.kind["unknown"] += 1
            self._dump(payload, "unknown", force=True)

    def _dump(self, payload: bytes, kind: str, force: bool = False):
        """Save GSSE payloads for inspection. By default the first 20 of each
        kind plus every unknown one (capped), so the rev 1.0 header can be
        examined with xxd after the first capture."""
        if not self.payload_dir:
            return
        n_kind = self.kind[kind if kind != "gsuftp" else "gsuftp_direct"]
        if n_kind > 20 and not (force and n_kind <= 200):
            return
        path = os.path.join(self.payload_dir, f"{self.n_payload:06d}_{kind}.bin")
        with open(path, "wb") as f:
            f.write(payload[: self.args.dump_max_bytes])

    def _ip(self, packet: bytes):
        self.ip_seen += 1
        for src, dst, proto, payload in self.ip.feed(packet):
            if proto != ipudp.PROTO_UDP:
                continue
            parsed = ipudp.parse_udp(payload, src, dst, verify=not self.args.no_udp_checksum)
            if parsed is None:
                continue
            sport, dport, data, state = parsed
            self.udp[state] += 1
            self.udp_ports[dport] = self.udp_ports.get(dport, 0) + 1
            if state == "bad":
                continue
            if self.args.udp_port and dport != self.args.udp_port:
                self.udp["filtered"] += 1
                continue
            if self.udp_raw:                       # plain UDP test data, not GSUFTP
                self.udp_raw.write(data)
                self.udp_raw_bytes += len(data)
                continue
            self.udp["to_gsuftp"] += 1
            self.ftp.handle_packet(data)

    def summary(self) -> dict:
        return {
            "elapsed_s": round(time.time() - self.t0, 2),
            "args": {k: v for k, v in vars(self.args).items()},
            "bbframe": {"frames": self.bb.frames, "resync_events": self.bb.resync_events,
                        "bytes_resynced": self.bb.bytes_resynced,
                        "partial_frames": self.bb.partial_frames,
                        "wrong_isi": self.bb.frames_wrong_isi,
                        "first_header": self.bb.first_header,
                        "streams": {f"0x{m:02X}/isi{i}": c
                                    for (m, i), c in self.bb.stream_stats.items()}},
            "gsse": {"ok": self.df.frames_ok, "crc_bad": self.df.frames_crc_bad,
                     "abandoned": self.df.frames_abandoned,
                     "oversize": self.df.frames_oversize,
                     "bytes_discarded": self.df.bytes_discarded,
                     "max_payload": self.df.max_frame_seen},
            "payload_kinds": self.kind,
            "ip": dict(self.ip.stats, packets_in=self.ip_seen),
            "udp": dict(self.udp, dst_ports=self.udp_ports),
            "gsuftp": dict(self.ftp.stats, chunk_sizes=sorted(self.ftp.chunk_sizes)),
            "files": self.ftp.completed,
            "udp_raw_bytes": self.udp_raw_bytes,
        }

    def report(self):
        el = time.time() - self.t0
        print("\n--- link report (%.1f s) ---" % el)
        if self.stage == "bbframe":
            print(f"BBFRAMEs: {self.bb.frames} ok, {self.bb.resync_events} re-sync "
                  f"events, {self.bb.bytes_resynced} B slid over, "
                  f"{self.bb.partial_frames} partial (DFL<max), "
                  f"{self.bb.frames_wrong_isi} filtered by ISI")
            for line in self.bb.stream_report():
                print(line)
        if self.stage in ("bbframe", "gsse"):
            print(f"GSSE: {self.df.frames_ok} frames ok, {self.df.frames_crc_bad} CRC-32 "
                  f"fail, {self.df.frames_abandoned} abandoned (lost TAIL), "
                  f"{self.df.frames_oversize} oversize, {self.df.bytes_discarded} B "
                  f"outside frames, largest payload {self.df.max_frame_seen} B")
            print(f"GSSE payloads: {self.kind['ipv4']} IPv4 (rev1.2 path), "
                  f"{self.kind['gsuftp_direct']} GSUFTP direct (rev1.0 path), "
                  f"{self.kind['unknown']} unknown")
        if self.ip_seen:
            print(f"IP: {self.ip_seen} packets in, {self.ip.stats['reassembled']} "
                  f"defragmented, {self.ip.stats['expired']} expired, "
                  f"{self.ip.stats['bad_cksum']} bad header checksum, "
                  f"{self.ip.stats['not_ipv4']} not IPv4")
            print(f"UDP: {self.udp['ok']} cksum ok, {self.udp['none']} no cksum, "
                  f"{self.udp['bad']} BAD cksum, {self.udp['filtered']} filtered; "
                  f"dst ports {self.udp_ports}")
        if self.udp_raw:
            print(f"UDP raw payloads: {self.udp_raw_bytes} B written to {self.args.udp_raw_out}")
        else:
            print(self.ftp.report())


def verify_hashes(path: str, completed: dict) -> bool:
    """Compare completed files against a `sha256sum` file (test plan L-03)."""
    want = {}
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) >= 2:
                want[os.path.basename(parts[-1].lstrip("*"))] = parts[0].lower()
    got = {os.path.basename(k): v["sha256"] for k, v in completed.items()}
    ok_all = True
    print("\n--- hash verification ---")
    for name, h in sorted(want.items()):
        if name not in got:
            print(f"  MISSING  {name}")
            ok_all = False
        elif got[name] == h:
            print(f"  PASS     {name}")
        else:
            print(f"  FAIL     {name}  got {got[name]}")
            ok_all = False
    print("RESULT:", "PASS" if ok_all else "FAIL")
    return ok_all


def verify_udp_raw(stream_path: str, ref_path: str) -> bool:
    """Count intact copies of the reference file in the raw UDP payload stream
    (Branch C1: TV-1 sent as plain UDP over the rf1 TUN, no GSUFTP)."""
    with open(ref_path, "rb") as f:
        ref = f.read()
    with open(stream_path, "rb") as f:
        data = f.read()
    copies, pos = 0, data.find(ref)
    while pos >= 0:
        copies += 1
        pos = data.find(ref, pos + len(ref))
    name = os.path.basename(ref_path)
    print("\n--- UDP raw verification ---")
    print(f"  {copies} intact cop{'y' if copies == 1 else 'ies'} of {name} "
          f"({len(ref)} B) in {len(data)} B of UDP payload")
    print("RESULT:", "PASS" if copies else "FAIL")
    return copies > 0


def _read_head(path: str, n: int) -> bytes:
    with open(path, "rb") as f:
        return f.read(n)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--listen", default="0.0.0.0:2000",
                   help="UDP host:port to receive from GNU Radio (default 0.0.0.0:2000)")
    p.add_argument("--from-file", help="replay a captured stream instead of listening")
    p.add_argument("--input", choices=["bbframe", "gsse", "ip"], default="bbframe",
                   help="input bytes are DVB-S2 BBFRAMEs (default), a raw GSSE "
                        "byte stream, or IP packets one per datagram")
    p.add_argument("--stack", choices=["auto", "rev1.0", "rev1.2"], default="auto",
                   help="what GSSE carries: rev1.0 GSUFTP directly (current Link X), "
                        "rev1.2 IPv4/UDP/GSUFTP (later SX), auto = detect per payload")
    p.add_argument("--modcod", default="1",
                   help="Link X MODCOD number 1-28, 'QPSK1/4' style, or bare rate")
    p.add_argument("--framesize", default="normal", choices=["normal", "short"])
    p.add_argument("--bbframe-len",
                   help="BBFRAME size in bytes, or 'auto' to detect from --from-file data")
    p.add_argument("--isi", type=int, help="only accept this DVB-S2 stream ID")
    p.add_argument("--udp-port", type=int,
                   help="rev1.2: only accept GSUFTP on this destination UDP port")
    p.add_argument("--udp-raw-out", metavar="FILE",
                   help="write UDP payloads here in arrival order instead of parsing "
                        "them as GSUFTP (plain UDP test data, e.g. nc over rf1)")
    p.add_argument("--udp-raw-ref", metavar="FILE",
                   help="with --udp-raw-out: PASS only if the stream holds at least one "
                        "intact copy of this file")
    p.add_argument("--out", default="./downlink", help="output directory for files")
    p.add_argument("--dump-gsse", help="also write the recovered GSSE byte stream here")
    p.add_argument("--dump-payloads", metavar="DIR",
                   help="save GSSE payloads (first 20 per kind + unknowns) for xxd")
    p.add_argument("--dump-max-bytes", type=int, default=4096,
                   help="bytes saved per dumped payload (default 4096)")
    p.add_argument("--max-gsse-frame", type=int, default=16 << 20,
                   help="largest GSSE frame accepted (default 16 MiB; rev1.0 "
                        "GSUFTP packets are ~10 MB)")
    p.add_argument("--verify-hashes", metavar="SHA256FILE",
                   help="sha256sum file of the transmitted files; exit 0 only if all match")
    p.add_argument("--report-json", help="write all counters + file hashes here")
    p.add_argument("--no-crc8", action="store_true", help="do not require BBHEADER CRC-8")
    p.add_argument("--no-ip-checksum", action="store_true")
    p.add_argument("--no-udp-checksum", action="store_true")
    p.add_argument("--stats-every", type=float, default=10.0,
                   help="seconds between progress lines in --listen mode (0 = off)")
    p.add_argument("--quiet", action="store_true")
    args = p.parse_args(argv)
    if args.udp_raw_ref and not args.udp_raw_out:
        sys.exit("--udp-raw-ref needs --udp-raw-out")

    if args.bbframe_len == "auto":
        if not args.from_file:
            sys.exit("--bbframe-len auto needs --from-file")
        det = bbframe.detect_frame_len(_read_head(args.from_file, 256 * 1024))
        if not det:
            sys.exit("could not detect a BBFRAME length (no run of valid BBHEADERs "
                     "found) - is the input really BBFRAMEs?")
        args.bbframe_len = det[0]
        print(f"auto-detected BBFRAME length {det[0]} B ({det[1]})")
    elif args.bbframe_len:
        args.bbframe_len = int(args.bbframe_len)
    else:
        args.bbframe_len = bbframe.bbframe_len(args.framesize,
                                               bbframe.parse_modcod(args.modcod))

    os.makedirs(args.out, exist_ok=True)
    pipe = Pipeline(args)

    if args.input == "bbframe":
        print(f"BBFRAME size {args.bbframe_len} B - set the GNU Radio UDP sink "
              f"payload size to this for one frame per datagram")
    print(f"protocol stack: {args.stack}; writing completed files to "
          f"{os.path.abspath(args.out)}")

    ok = True
    try:
        if args.from_file:
            with open(args.from_file, "rb") as f:
                while True:
                    chunk = f.read(1 << 20)
                    if not chunk:
                        break
                    pipe.feed(chunk)
        else:
            host, port = args.listen.rsplit(":", 1)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
            sock.bind((host, int(port)))
            sock.settimeout(1.0)
            print(f"listening on {args.listen} (ctrl-C to stop)")
            last = time.time()
            while True:
                try:
                    data, _ = sock.recvfrom(65535)
                except socket.timeout:
                    data = None
                if data:
                    pipe.feed(data)
                if args.stats_every and time.time() - last > args.stats_every:
                    last = time.time()
                    print(f"  [bb {pipe.bb.frames} | gsse {pipe.df.frames_ok} ok "
                          f"/{pipe.df.frames_crc_bad} bad | payloads {pipe.kind} "
                          f"| files {pipe.ftp.stats['files']}]")
    except KeyboardInterrupt:
        pass
    finally:
        pipe.report()
        if pipe.udp_raw:
            pipe.udp_raw.close()
        if args.verify_hashes:
            ok = verify_hashes(args.verify_hashes, pipe.ftp.completed)
        if args.udp_raw_ref:
            ok = verify_udp_raw(args.udp_raw_out, args.udp_raw_ref) and ok
        if args.report_json:
            with open(args.report_json, "w") as f:
                json.dump(pipe.summary(), f, indent=2, default=str)
            print(f"report written to {args.report_json}")
        if pipe.gsse_dump:
            pipe.gsse_dump.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
