#!/usr/bin/env python3
"""tx_simulator - pretend to be the NanoCom Link X so the ground chain can be tested
without a radio. Produces BBFRAMEs (file or UDP), the same shape of data gr-dvbs2rx emits.

  --stack rev1.0  (current Link X)  GSUFTP packets in GSSE in BBFRAMEs
                  chunk size default 10 000 000 B (GSUFTP manual sec. 4 "~10 MB");
                  header layout assumed identical to rev 1.2 (undocumented).
  --stack rev1.2  (later SX)        GSUFTP 50 000 B chunks in UDP in IPv4
                  (fragmented to --mtu) in GSSE in BBFRAMEs

  python3 tx_simulator.py --stack rev1.0 --files ./testdata --out-file sim10.bbframes
  python3 gs_rx.py --from-file sim10.bbframes --bbframe-len auto --out ./rx

Failure injection: --drop-frames P, --corrupt P, --repeat N (looping like gsuftp-tx),
--junk N (pre-lock garbage bytes).
"""

from __future__ import annotations

import argparse
import os
import random
import socket
import sys
import time

import bbframe
import gsse
import gsuftp
import ipudp


def gsuftp_packets(path: str, rel: str, chunk_size: int):
    size = os.path.getsize(path)
    total = max(1, (size + chunk_size - 1) // chunk_size)
    with open(path, "rb") as f:
        for n in range(1, total + 1):
            yield gsuftp.build_packet(rel, total, n, f.read(chunk_size))


def raw_chunks(path: str, chunk_size: int):
    with open(path, "rb") as f:
        while chunk := f.read(chunk_size):
            yield chunk


def build_stream(targets, args) -> bytes:
    stream = bytearray()
    ident = 0
    for _ in range(args.repeat):
        for full, rel in targets:
            pkts = (raw_chunks(full, args.chunk_size) if args.stack == "udpraw"
                    else gsuftp_packets(full, rel, args.chunk_size))
            for pkt in pkts:
                if args.stack == "rev1.0":
                    stream += gsse.encode(pkt)
                elif args.stack == "udpraw":
                    ident = (ident + 1) & 0xFFFF
                    for ip_pkt in ipudp.build_udp_ip(args.src, args.dst, 40000,
                                                     args.dport, pkt, ident, args.mtu):
                        stream += gsse.encode(ip_pkt)
                else:
                    ident = (ident + 1) & 0xFFFF
                    for ip_pkt in ipudp.build_udp_ip(args.src, args.dst, 40000,
                                                     args.dport, pkt, ident, args.mtu):
                        stream += gsse.encode(ip_pkt)
    return bytes(stream)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--files", required=True, help="file or directory to 'downlink'")
    p.add_argument("--stack", choices=["rev1.0", "rev1.2", "udpraw"], default="rev1.0",
                   help="udpraw: files as plain UDP datagrams of --chunk-size B, no "
                        "GSUFTP (like busybox nc over the rf1 TUN)")
    p.add_argument("--chunk-size", type=int,
                   help="GSUFTP chunk size (default 10 000 000 rev1.0, 50 000 rev1.2)")
    p.add_argument("--out-file", help="write BBFRAMEs here")
    p.add_argument("--send", help="stream BBFRAMEs to host:port instead")
    p.add_argument("--rate", type=float, default=2000.0,
                   help="--send pacing in BBFRAMEs per second (default 2000)")
    p.add_argument("--modcod", default="1")
    p.add_argument("--framesize", default="normal", choices=["normal", "short"])
    p.add_argument("--isi", type=int, default=0)
    p.add_argument("--sis", action="store_true", help="single-input-stream BBHEADERs")
    p.add_argument("--mtu", type=int, default=9000, help="rev1.2 interface MTU")
    p.add_argument("--src", default="10.128.0.4")
    p.add_argument("--dst", default="10.129.0.1")
    p.add_argument("--dport", type=int, default=5555, help="rev1.2 GSUFTP UDP port")
    p.add_argument("--repeat", type=int, default=1)
    p.add_argument("--junk", type=int, default=0, help="random bytes before first frame")
    p.add_argument("--drop-frames", type=float, default=0.0)
    p.add_argument("--corrupt", type=float, default=0.0)
    p.add_argument("--seed", type=int, default=1)
    args = p.parse_args(argv)
    if args.chunk_size is None:
        args.chunk_size = {"rev1.0": 10_000_000, "udpraw": 1024}.get(
            args.stack, gsuftp.CHUNK_SIZE_REV12)

    random.seed(args.seed)
    flen = bbframe.bbframe_len(args.framesize, bbframe.parse_modcod(args.modcod))
    usable = flen - bbframe.BB_HEADER_LEN

    targets = []
    if os.path.isdir(args.files):
        for root, _, names in os.walk(args.files):
            for nm in sorted(names):
                full = os.path.join(root, nm)
                targets.append((full, os.path.relpath(full, args.files)))
    else:
        targets.append((args.files, os.path.basename(args.files)))
    if not targets:
        sys.exit("no files to send")

    stream = build_stream(targets, args)

    kept = bytearray(random.randbytes(args.junk)) if args.junk else bytearray()
    n_frames = dropped = corrupted = 0
    for off in range(0, len(stream), usable):
        fr = bbframe.build_bbframe(stream[off:off + usable], flen,
                                   isi=args.isi, sis=args.sis)
        n_frames += 1
        if random.random() < args.drop_frames:
            dropped += 1
            continue
        if random.random() < args.corrupt:
            fr = bytearray(fr)
            fr[random.randrange(bbframe.BB_HEADER_LEN, len(fr))] ^= 1 << random.randrange(8)
            fr = bytes(fr)
            corrupted += 1
        kept += fr

    print(f"{len(targets)} file(s) x{args.repeat}, stack {args.stack}, chunk "
          f"{args.chunk_size} B -> {len(stream)} B GSSE stream -> {n_frames} "
          f"BBFRAMEs of {flen} B ({dropped} dropped, {corrupted} corrupted)")

    if args.out_file:
        with open(args.out_file, "wb") as f:
            f.write(kept)
        print(f"wrote {args.out_file}")
    if args.send:
        host, port = args.send.rsplit(":", 1)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        gap = 1.0 / args.rate if args.rate > 0 else 0
        sent = 0
        for off in range(0, len(kept), flen):
            sock.sendto(bytes(kept[off:off + flen]), (host, int(port)))
            sent += 1
            if gap:
                time.sleep(gap)
        print(f"sent {sent} BBFRAMEs to {args.send}")
    if not args.out_file and not args.send:
        sys.stdout.buffer.write(bytes(kept))
    return 0


if __name__ == "__main__":
    sys.exit(main())
