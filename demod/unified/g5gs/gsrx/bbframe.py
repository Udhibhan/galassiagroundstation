"""DVB-S2 baseband frame handling: turns the BBFRAME stream from the receiver back into
the continuous byte stream that GSSE runs on.

Refs: ETSI EN 302 307-1 clause 5.1.6 (BBHEADER), GomSpace ICD 1045318 section 3.2
(same in rev 1.0 and rev 1.2).

A BBFRAME is a 10-byte BBHEADER, DFL/8 bytes of data field, then zero padding up to
KBCH/8 bytes. For a generic continuous stream the data field is just the next slice of an
endless byte stream, so: strip the header, keep DFL/8 bytes, drop the padding, concatenate.
The result is the GSSE byte stream.

v2 changes (see CHANGES.md):
  * Re-sync needs two consecutive valid headers (k and k + frame_len). With only the
    8-bit CRC and a DFL range check, v1 had about a 1 in 8000 false-lock chance per byte
    offset while sliding, roughly 20 % per lost frame at QPSK 1/4.
  * DFL = 0 is accepted (valid header, empty data field).
  * Link X MODCOD numbers 1-28 (Control ICD Table 3.5) are accepted.
  * detect_frame_len() finds the BBFRAME length from the data, so a wrong --modcod or UDP
    sink size gets diagnosed instead of silently re-syncing.
  * Per-stream statistics (MATYPE / ISI histogram) for the test record.
"""

from __future__ import annotations

from collections import Counter

BB_HEADER_LEN = 10

# KBCH in bits, [framesize][code rate]. KBCH/8 = BBFRAME size in bytes.
KBCH = {
    "normal": {
        "1/4": 16008, "1/3": 21408, "2/5": 25728, "1/2": 32208, "3/5": 38688,
        "2/3": 43040, "3/4": 48408, "4/5": 51648, "5/6": 53840, "8/9": 57472,
        "9/10": 58192,
    },
    "short": {
        "1/4": 3072, "1/3": 5232, "2/5": 6312, "1/2": 7032, "3/5": 9552,
        "2/3": 10632, "3/4": 11712, "4/5": 12432, "5/6": 13152, "8/9": 14232,
    },
}

# DVB-S2 MODCOD numbering as used by the Link X `modcod` parameter
# (Control ICD 1045207 Table 3.5; Datasheet Table 6-2).
MODCODS = {
    1: ("QPSK", "1/4"), 2: ("QPSK", "1/3"), 3: ("QPSK", "2/5"), 4: ("QPSK", "1/2"),
    5: ("QPSK", "3/5"), 6: ("QPSK", "2/3"), 7: ("QPSK", "3/4"), 8: ("QPSK", "4/5"),
    9: ("QPSK", "5/6"), 10: ("QPSK", "8/9"), 11: ("QPSK", "9/10"),
    12: ("8PSK", "3/5"), 13: ("8PSK", "2/3"), 14: ("8PSK", "3/4"),
    15: ("8PSK", "5/6"), 16: ("8PSK", "8/9"), 17: ("8PSK", "9/10"),
    18: ("16APSK", "2/3"), 19: ("16APSK", "3/4"), 20: ("16APSK", "4/5"),
    21: ("16APSK", "5/6"), 22: ("16APSK", "8/9"), 23: ("16APSK", "9/10"),
    24: ("32APSK", "3/4"), 25: ("32APSK", "4/5"), 26: ("32APSK", "5/6"),
    27: ("32APSK", "8/9"), 28: ("32APSK", "9/10"),
}

TS_GS = {0b00: "generic packetized", 0b01: "generic continuous",
         0b10: "GSE-HEM / reserved", 0b11: "transport stream"}
ROLLOFF = {0b00: 0.35, 0b01: 0.25, 0b10: 0.20, 0b11: "reserved/0.15"}


def parse_modcod(s) -> str:
    """'QPSK1/4', '1/4' or a Link X MODCOD number ('1'..'28') -> code rate."""
    s = str(s).strip().upper()
    if s.isdigit():
        n = int(s)
        if n not in MODCODS:
            raise SystemExit(f"MODCOD {n} not in 1..28")
        return MODCODS[n][1]
    for mod in ("8PSK", "16APSK", "32APSK", "QPSK"):
        if s.startswith(mod):
            return s[len(mod):]
    return s


def bbframe_len(framesize: str, rate: str) -> int:
    """BBFRAME length in bytes for a given FECFRAME size and code rate."""
    try:
        return KBCH[framesize][rate] // 8
    except KeyError:
        raise SystemExit(f"unknown MODCOD combination {framesize!r} {rate!r}; "
                         f"valid rates: {sorted(KBCH[framesize])}"
                         if framesize in KBCH else
                         f"framesize must be 'normal' or 'short', got {framesize!r}")


def _crc8_bitwise(data: bytes) -> int:
    reg = 0
    for byte in data:
        for j in range(8):
            bit = (byte >> (7 - j)) & 1
            feed = (reg >> 7) ^ bit
            reg = (reg << 1) & 0xFF
            if feed:
                reg ^= 0xD5
    return reg


_CRC8_TABLE = [_crc8_bitwise(bytes([i])) for i in range(256)]


def crc8(data: bytes) -> int:
    """DVB-S2 CRC-8, generator x^8+x^7+x^6+x^4+x^2+1 (0xD5), init 0, MSB
    first, over the first 9 bytes of the BBHEADER. (EN 302 307-1 V1.4.1 XORs
    the CRC with MODE; MODE = 0 in normal mode, so the value is unchanged.)
    Table-driven; equal to _crc8_bitwise (checked in test_vectors.py)."""
    reg = 0
    for byte in data:
        reg = _CRC8_TABLE[reg ^ byte]
    return reg


def parse_bbheader(hdr: bytes) -> dict:
    """Decode the 10-byte BBHEADER into a dict of fields."""
    if len(hdr) < BB_HEADER_LEN:
        raise ValueError("short BBHEADER")
    m1 = hdr[0]
    return {
        "matype1": m1,
        "ts_gs": (m1 >> 6) & 0b11,
        "sis": bool((m1 >> 5) & 1),        # True = single input stream
        "ccm": bool((m1 >> 4) & 1),        # True = CCM, False = ACM/VCM
        "issyi": bool((m1 >> 3) & 1),
        "npd": bool((m1 >> 2) & 1),
        "ro": m1 & 0b11,
        "isi": hdr[1],                     # only meaningful when sis is False
        "upl": int.from_bytes(hdr[2:4], "big"),
        "dfl": int.from_bytes(hdr[4:6], "big"),
        "sync": hdr[6],
        "syncd": int.from_bytes(hdr[7:9], "big"),
        "crc8": hdr[9],
        "crc8_ok": crc8(hdr[:9]) == hdr[9],
    }


def describe(h: dict) -> str:
    stream = "SIS (single stream)" if h["sis"] else f"MIS, ISI={h['isi']}"
    return (f"MATYPE-1=0x{h['matype1']:02X}: stream type={TS_GS.get(h['ts_gs'])}, "
            f"{stream}, {'CCM' if h['ccm'] else 'ACM/VCM'}, "
            f"rolloff={ROLLOFF.get(h['ro'])}, "
            f"dfl={h['dfl']} bits ({h['dfl'] // 8} bytes), "
            f"crc8={'ok' if h['crc8_ok'] else 'BAD'}")


def _header_ok(buf, off: int, frame_len: int, check_crc8: bool) -> bool:
    hdr = bytes(buf[off:off + BB_HEADER_LEN])
    if len(hdr) < BB_HEADER_LEN:
        return False
    dfl = int.from_bytes(hdr[4:6], "big")        # cheap test first
    if dfl % 8 or dfl > (frame_len - BB_HEADER_LEN) * 8:
        return False
    return not check_crc8 or crc8(hdr[:9]) == hdr[9]


def detect_frame_len(sample: bytes, framesizes=("normal", "short")):
    """Find the BBFRAME length from raw data: collect every offset holding a
    CRC-8-valid BBHEADER once, then pick the DVB-S2 KBCH/8 for which valid
    headers repeat at that spacing the most times (>= 3 in a row).
    Returns (length, label) or None."""
    max_dfl = (max(KBCH["normal"].values()) // 8 - BB_HEADER_LEN) * 8
    valid = set()
    for off in range(0, len(sample) - BB_HEADER_LEN + 1):
        dfl = (sample[off + 4] << 8) | sample[off + 5]
        if dfl % 8 or dfl > max_dfl:
            continue
        if crc8(sample[off:off + 9]) == sample[off + 9]:
            valid.add(off)
    best = None
    for fs in framesizes:
        for rate, kbch in KBCH[fs].items():
            n = kbch // 8
            for off in sorted(valid):
                run = 1
                while off + run * n in valid:
                    run += 1
                if run >= 3:
                    if best is None or run > best[0]:
                        best = (run, n, f"{fs} {rate}")
                    break
    return (best[1], best[2]) if best else None


class BBFrameStream:
    """Consumes whatever bytes arrive from the DVB-S2 receiver and yields
    data fields.

    Handles one BBFRAME per UDP datagram and frames split across datagrams or
    read from a file. Alignment is confirmed on every frame using the CRC-8.
    When lost, it slides byte by byte and only re-locks when a candidate
    header AND the header one frame later both validate.
    """

    def __init__(self, frame_len: int, isi: int | None = None,
                 check_crc8: bool = True):
        self.frame_len = frame_len
        self.isi = isi
        self.check_crc8 = check_crc8
        self.buf = bytearray()
        self.locked = False
        self.frames = 0
        self.frames_wrong_isi = 0
        self.resync_events = 0
        self.bytes_resynced = 0
        self.first_header = None
        self.stream_stats = Counter()    # (matype1, isi) -> frames
        self.partial_frames = 0          # DFL < max (buf_timeout flushes)

    # v1 name kept for gs_rx compatibility
    @property
    def frames_bad(self):
        return self.resync_events

    def feed(self, data: bytes):
        """Yield data-field byte strings, in order."""
        self.buf += data
        fl = self.frame_len
        while True:
            if not self.locked:
                if len(self.buf) < 2 * fl:
                    return
                if (_header_ok(self.buf, 0, fl, self.check_crc8)
                        and _header_ok(self.buf, fl, fl, self.check_crc8)):
                    self.locked = True
                else:
                    del self.buf[:1]
                    self.bytes_resynced += 1
                    continue
            if len(self.buf) < fl:
                return
            if not _header_ok(self.buf, 0, fl, self.check_crc8):
                self.locked = False
                self.resync_events += 1
                continue
            h = parse_bbheader(bytes(self.buf[:BB_HEADER_LEN]))
            if self.first_header is None:
                self.first_header = h
            n = h["dfl"] // 8
            payload = bytes(self.buf[BB_HEADER_LEN:BB_HEADER_LEN + n])
            del self.buf[:fl]
            self.frames += 1
            self.stream_stats[(h["matype1"], h["isi"])] += 1
            if n < fl - BB_HEADER_LEN:
                self.partial_frames += 1
            if self.isi is not None and not h["sis"] and h["isi"] != self.isi:
                self.frames_wrong_isi += 1
                continue
            yield payload

    def stream_report(self) -> list[str]:
        lines = []
        for (m1, isi), cnt in self.stream_stats.most_common():
            sis = bool((m1 >> 5) & 1)
            lines.append(f"  MATYPE-1=0x{m1:02X} {'SIS' if sis else f'MIS ISI={isi}'}: "
                         f"{cnt} frames")
        return lines


def build_bbframe(data_field: bytes, frame_len: int, isi: int = 0,
                  rolloff_code: int = 0b10, ccm: bool = True,
                  sis: bool = False) -> bytes:
    """Build a BBFRAME carrying a generic continuous stream (simulator only)."""
    usable = frame_len - BB_HEADER_LEN
    if len(data_field) > usable:
        raise ValueError(f"data field {len(data_field)} > {usable} bytes")
    m1 = (0b01 << 6) | ((1 if sis else 0) << 5) | ((1 if ccm else 0) << 4) | rolloff_code
    hdr = bytearray([m1, 0 if sis else (isi & 0xFF)])
    hdr += (0).to_bytes(2, "big")                    # UPL = 0 for continuous
    hdr += (len(data_field) * 8).to_bytes(2, "big")  # DFL in bits
    hdr += bytes([0])                                # SYNC
    hdr += (0).to_bytes(2, "big")                    # SYNCD
    hdr.append(crc8(bytes(hdr)))
    return bytes(hdr) + data_field + b"\x00" * (usable - len(data_field))
