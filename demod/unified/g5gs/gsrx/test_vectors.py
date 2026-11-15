#!/usr/bin/env python3
"""
Verifies the implementation against the worked examples in the GomSpace ICD,
fuzzes the fast paths against the v1 reference code, and runs end-to-end
simulations of both protocol stacks. Run before trusting anything else:

    python3 test_vectors.py
"""
import hashlib
import os
import random
import subprocess
import sys
import tempfile

import bbframe
import gsse
import gsuftp
import ipudp

FAILS = 0
HERE = os.path.dirname(os.path.abspath(__file__))


def check(name, got, want):
    global FAILS
    ok = got == want
    print(f"[{'PASS' if ok else 'FAIL'}] {name}")
    if not ok:
        FAILS += 1
        print(f"    got:  {got!r}"[:300])
        print(f"    want: {want!r}"[:300])


# ---------------------------------------------------------- ICD A.1-A.4 (GSSE)
PAYLOAD = bytes.fromhex("FFA1484541EB544149C1")
check("ICD A.1 CRC-32 append", gsse.append_crc(PAYLOAD).hex().upper(),
      "FFA1484541EB544149C1" "05BAF0B2")
check("ICD A.1 CRC self-check residue is zero", gsse.check_crc(gsse.append_crc(PAYLOAD)), True)
check("ICD A.2 byte stuffing", gsse.stuff(gsse.append_crc(PAYLOAD)).hex().upper(),
      "FFA148454100EB54414900C1" "05BAF0B2")
check("ICD A.2 de-stuffing is the exact inverse",
      gsse.destuff(gsse.stuff(gsse.append_crc(PAYLOAD))), gsse.append_crc(PAYLOAD))
check("ICD A.3 marker insertion (full frame)", gsse.encode(PAYLOAD).hex().upper(),
      "48454144" "FFA148454100EB54414900C1" "05BAF0B2" "5441494C")
check("ICD A.4 decode(encode(x)) == x", gsse.decode(gsse.encode(PAYLOAD)), PAYLOAD)

# Link X test plan Test 2 / Step 2 negative tests, on the ICD worked example
FRAMED = gsse.encode(PAYLOAD)
flipped = bytearray(FRAMED)
flipped[6] ^= 0x01                                     # one bit inside the stuffed body
df = gsse.GsseDeframer()
check("Test 2: one flipped bit -> CRC fails, frame discarded",
      (list(df.feed(bytes(flipped))), df.frames_crc_bad), ([], 1))
df = gsse.GsseDeframer()
idle = bytes(64) + bytes.fromhex("FF") * 64 + b"HEA" + b"TAI" + b"junk"
check("Test 2: idle/garbage between two frames -> both recovered",
      list(df.feed(FRAMED + idle + FRAMED)), [PAYLOAD, PAYLOAD])
split_ok = True
for cut in range(1, len(FRAMED)):
    df = gsse.GsseDeframer()
    got = list(df.feed(FRAMED[:cut])) + list(df.feed(FRAMED[cut:]))
    split_ok &= got == [PAYLOAD]
check(f"Test 2: frame split across two input chunks (all {len(FRAMED) - 1} cut points)",
      split_ok, True)

# --------------------------------------------------- rev 1.2 ICD B.2 (GSUFTP)
HDR = bytes.fromhex("7375626469722F7465737466696C652E62696E" "00" "3133" "00" "32" "00")
name, total, chunk, data = gsuftp.parse_packet(HDR + b"payload bytes")
check("ICD B.2 file identifier", name, "subdir/testfile.bin")
check("ICD B.2 total chunk number", total, 13)
check("ICD B.2 chunk number (1-indexed)", chunk, 2)
check("ICD B.2 file data is the remainder", data, b"payload bytes")
check("ICD B.2 header round-trip",
      gsuftp.build_packet("subdir/testfile.bin", 13, 2, b"payload bytes"), HDR + b"payload bytes")
check("ICD B.2 chunk 2 offset", gsuftp.chunk_offset(2), 50000)

# --------------------------------- v2 fast paths == v1 reference implementation
random.seed(0xC0FFEE)
tricky = [b"", b"\x00", bytes.fromhex("484541"), bytes.fromhex("48454100"),
          bytes.fromhex("484541484541484541"), bytes.fromhex("544149"),
          bytes.fromhex("54414900"), bytes.fromhex("48454144"), bytes.fromhex("5441494C"),
          bytes.fromhex("484541") * 40, bytes.fromhex("544149") * 40,
          bytes.fromhex("4845415441494845"), bytes.fromhex("4845410000"),
          bytes.fromhex("5441490000544149")]
alphabet = bytes([0x48, 0x45, 0x41, 0x44, 0x54, 0x49, 0x4C, 0x00, 0xFF])
for _ in range(6000):
    tricky.append(bytes(random.choice(alphabet) for _ in range(random.randrange(0, 200))))
for _ in range(300):
    tricky.append(os.urandom(random.randrange(0, 4096)))

mism = sum(gsse.stuff(p) != gsse.stuff_reference(p) for p in tricky)
check(f"fast stuff() == v1 reference on {len(tricky)} payloads", mism, 0)
# destuff must also agree on arbitrary (incl. corrupted / unstuffed) input
mism = sum(gsse.destuff(p) != gsse.destuff_reference(p) for p in tricky)
check("fast destuff() == v1 reference on arbitrary input", mism, 0)
mism = sum(gsse.crc32_mpeg2(p) != gsse.crc32_mpeg2_reference(p) for p in tricky)
check("fast CRC-32/MPEG-2 == v1 table reference", mism, 0)
bad = sum(gsse.decode(gsse.encode(p)) != p for p in tricky)
check("round-trip through full GSSE frame", bad, 0)

# ------------------------------------------------- streaming deframer, lossy
payloads = [os.urandom(random.randrange(20, 3000)) for _ in range(50)]
stream = bytearray(os.urandom(777))                    # pre-lock garbage
for p in payloads:
    stream += gsse.encode(p)
corrupt = bytearray(gsse.encode(os.urandom(500)))
corrupt[60] ^= 0x01                                    # bit flip -> CRC fail
stream += corrupt
stream += gsse.encode(os.urandom(400))[:200]          # truncated frame (lost TAIL)
tail_payloads = [os.urandom(random.randrange(20, 3000)) for _ in range(10)]
for p in tail_payloads:
    stream += gsse.encode(p)

df = gsse.GsseDeframer()
recovered = []
i = 0
while i < len(stream):
    n = random.randrange(1, 5000)
    recovered.extend(df.feed(bytes(stream[i:i + n])))
    i += n
check("deframer: all pre-corruption payloads recovered", recovered[:len(payloads)], payloads)
check("deframer: bit-flipped frame rejected by CRC", df.frames_crc_bad, 1)
check("deframer: truncated frame abandoned at next HEAD (v2 re-sync)", df.frames_abandoned, 1)
check("deframer: NO good frame lost after the truncated one (v1 lost one)",
      recovered[len(payloads):], tail_payloads)

# Large frame (rev 1.0 GSUFTP packets are ~10 MB)
big = os.urandom(10_000_100)
df = gsse.GsseDeframer()
enc = gsse.encode(big)
out = []
for k in range(0, len(enc), 65536):
    out.extend(df.feed(enc[k:k + 65536]))
check("deframer: 10 MB frame delivered intact (v1 dropped it as oversize)", out == [big], True)

# ------------------------------------------------------------ GSUFTP assembly
with tempfile.TemporaryDirectory() as td:
    rx = gsuftp.GsuftpReceiver(td, verbose=False)
    original = os.urandom(50000 * 3 + 1234)
    pkts = [gsuftp.build_packet("sub/dir/img.bin", 4, c + 1, original[c * 50000:(c + 1) * 50000])
            for c in range(4)]
    for idx in [3, 1, 3, 0, 2, 2]:                     # out of order + duplicates
        rx.handle_packet(pkts[idx])
    with open(os.path.join(td, "sub/dir/img.bin"), "rb") as f:
        check("GSUFTP reassembled bytes identical", f.read(), original)
    check("GSUFTP duplicates ignored", rx.stats["dup"], 2)
    check("GSUFTP sha256 recorded",
          rx.completed["sub/dir/img.bin"]["sha256"], hashlib.sha256(original).hexdigest())
    # a different total for an incomplete file id restarts it
    rx.handle_packet(gsuftp.build_packet("x.bin", 3, 1, b"a" * 10))
    rx.handle_packet(gsuftp.build_packet("x.bin", 2, 1, b"b" * 10))
    check("GSUFTP total-change restarts file", rx.stats["restarted"], 1)
    check("GSUFTP rejects non-GSUFTP payload",
          rx.handle_packet(bytes.fromhex("4500001C") + os.urandom(24)), None)

# ---------------------------------------------------------- IPv4 / UDP layer
frags = ipudp.build_udp_ip("10.128.0.4", "10.129.0.1", 40000, 5555, os.urandom(50050), 7, 9000)
re = ipudp.IPv4Reassembler()
got = [x for f in frags for x in re.feed(f)]
sport, dport, body, state = ipudp.parse_udp(got[0][3], got[0][0], got[0][1])
check("IPv4 fragments reassembled into one datagram", len(got), 1)
check("UDP checksum verifies", state, "ok")
# stale fragment from an older datagram with the same IP ID -> UDP checksum bad
old = ipudp.build_udp_ip("10.128.0.4", "10.129.0.1", 40000, 5555, os.urandom(50050), 9, 9000)
new = ipudp.build_udp_ip("10.128.0.4", "10.129.0.1", 40000, 5555, os.urandom(50050), 9, 9000)
re = ipudp.IPv4Reassembler()
mixed = [old[1]] + new[:1] + new[2:]                   # new datagram, frag 2 lost, old frag 2 present
got = [x for f in mixed for x in re.feed(f)]
st = ipudp.parse_udp(got[0][3], got[0][0], got[0][1])[3] if got else "none"
check("IP-ID collision mis-reassembly caught by UDP checksum", st, "bad")

# ----------------------------------------------------------- BBFRAME layer
check("table CRC-8 == bitwise CRC-8", all(bbframe.crc8(p[:9]) == bbframe._crc8_bitwise(p[:9])
                                          for p in tricky), True)
check("MODCOD 1 -> QPSK 1/4 -> 2001 B", bbframe.bbframe_len("normal", bbframe.parse_modcod("1")), 2001)
check("MODCOD 13 -> 8PSK 2/3 -> 5380 B", bbframe.bbframe_len("normal", bbframe.parse_modcod("13")), 5380)
fl = 2001
fields = [os.urandom(fl - 10) for _ in range(60)]
frames = b"".join(bbframe.build_bbframe(f, fl) for f in fields)
check("detect_frame_len finds 2001 (normal 1/4)", bbframe.detect_frame_len(os.urandom(333) + frames),
      (2001, "normal 1/4"))
# misaligned start + a dropped frame in the middle -> re-sync, no garbage out
data = os.urandom(1234) + frames[:fl * 20] + frames[fl * 21:]
bs = bbframe.BBFrameStream(fl)
outf = list(bs.feed(data))
check("BBFRAME re-sync after junk + dropped frame: only genuine fields",
      all(f in fields for f in outf) and len(outf) == 59, True)

# ----------------------------------------- end-to-end simulations (both stacks)
def run(cmd):
    return subprocess.run([sys.executable] + cmd, cwd=HERE, capture_output=True, text=True)


with tempfile.TemporaryDirectory() as td:
    src = os.path.join(td, "src")
    os.makedirs(src)
    with open(os.path.join(src, "tv1_pattern_64k.bin"), "wb") as f:
        f.write((b"G5 LINKX KU TEST HEAD TAIL HEA TAI 0123456789\n" * 2000)[:65536])
    with open(os.path.join(src, "tv2_random_1M.bin"), "wb") as f:
        f.write(random.Random(42).randbytes(1 << 20))   # deterministic
    with open(os.path.join(td, "h.sha256"), "w") as f:
        for nm in sorted(os.listdir(src)):
            f.write(hashlib.sha256(open(os.path.join(src, nm), "rb").read()).hexdigest()
                    + "  " + nm + "\n")

    cases = [("rev1.0", "clean", ["--chunk-size", "300000", "--repeat", "1"]),
             ("rev1.2", "clean", ["--repeat", "1"]),
             ("rev1.0", "lossy", ["--chunk-size", "300000", "--repeat", "6",
                                  "--drop-frames", "0.001", "--corrupt", "0.001"]),
             ("rev1.2", "lossy", ["--repeat", "6", "--drop-frames", "0.002",
                                  "--corrupt", "0.002"]),
             ("rev1.0", "short-frames 8PSK", ["--chunk-size", "300000", "--repeat", "1",
                                              "--modcod", "13", "--framesize", "short"])]
    for stack, label, extra in cases:
        bb = os.path.join(td, f"{stack}_{label[:5]}.bbframes")
        r = run(["tx_simulator.py", "--stack", stack, "--files", src, "--out-file", bb,
                 "--junk", "5000", "--seed", "3"] + extra)
        out = os.path.join(td, f"rx_{stack}_{label[:5]}")
        r = run(["gs_rx.py", "--from-file", bb, "--bbframe-len", "auto", "--out", out,
                 "--verify-hashes", os.path.join(td, "h.sha256"), "--quiet"])
        check(f"end-to-end {stack} {label} (auto stack + auto frame length): "
              f"files hash-verified", r.returncode, 0)
        if r.returncode:
            print(r.stdout[-2000:], r.stderr[-2000:])

    # plain UDP over rf1 (test plan Branch C1, Step 22C / Test 16C): no GSUFTP
    tv1 = os.path.join(src, "tv1_pattern_64k.bin")
    for label, extra, want in (("clean", [], 0), ("every frame corrupted", ["--corrupt", "1.0"], 1)):
        bb = os.path.join(td, f"udpraw_{label[:5]}.bbframes")
        run(["tx_simulator.py", "--stack", "udpraw", "--files", tv1, "--out-file", bb,
             "--repeat", "3", "--dport", "5000", "--junk", "500", "--seed", "3"] + extra)
        r = run(["gs_rx.py", "--from-file", bb, "--modcod", "1", "--out", os.path.join(td, "rx_u"),
                 "--udp-raw-out", os.path.join(td, f"u_{label[:5]}.raw"), "--udp-raw-ref", tv1, "--quiet"])
        check(f"end-to-end udpraw {label}: --udp-raw-ref exit code", r.returncode, want)

print()
if FAILS:
    print(f"{FAILS} FAILURE(S)")
    sys.exit(1)
print("all checks passed")
