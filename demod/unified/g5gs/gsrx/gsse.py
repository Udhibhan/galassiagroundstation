"""GSSE - GomSpace Stream Encapsulation.

Ref: GomSpace ICD 1045318 "NanoCom Link S, X, SX RF Interface", appendix A. It is the same
in rev 1.0 and rev 1.2, so this module serves both the current Link X and the later SX.
What GSSE carries differs between revisions, see gs_rx.py --stack.

GSSE turns a continuous byte stream (the concatenated DVB-S2 BBFRAME data fields on
X-band, or the CCSDS 131.0-B-3 code block stream on S-band) back into discrete packets.

Transmit chain (ICD figure A.1):

    payload, append CRC-32 (4 bytes), byte-stuff, append TAIL, prepend HEAD

Receive is the reverse: hunt for HEAD, buffer until TAIL, de-stuff, check CRC.

v2 changes (see CHANGES.md):
  * stuff/destuff/CRC run at C speed (bytes.replace / binascii). Rev 1.0 GSUFTP packets
    are ~10 MB and the v1 pure-Python loops took tens of seconds per frame. The v1 loops
    stay as *_reference for the tests.
  * GsseDeframer re-syncs on a new HEAD seen inside an open frame (stuffing guarantees
    HEAD cannot occur inside a valid frame), so a lost TAIL no longer costs the next good
    frame too.
  * GsseDeframer default max_frame raised to 16 MiB (v1: 256 KiB, which silently dropped
    every rev 1.0 GSUFTP packet as "lost TAIL").
  * Incremental TAIL search, no O(n^2) rescans of a growing 10 MB buffer.
"""

from __future__ import annotations

import binascii

HEAD = bytes.fromhex("48454144")   # ASCII "HEAD"
TAIL = bytes.fromhex("5441494C")   # ASCII "TAIL"

# The stuffer works on the first *three* bytes of each marker. Any occurrence
# of "HEA" or "TAI" in the payload gets a 0x00 glued behind it, which makes it
# impossible for a full 4-byte marker to appear inside a frame.
STUFF_PATTERNS = (HEAD[:3], TAIL[:3])   # 0x484541, 0x544149


# --------------------------------------------------------------------------
# CRC-32 (a.k.a. CRC-32/MPEG-2)
# --------------------------------------------------------------------------
# poly 0x04C11DB7, init 0xFFFFFFFF, no input/output reflection, no final XOR.
# The ICD writes the polynomial as 0x104C11DB7 (with the implicit x^32 term).

def _make_table() -> list[int]:
    table = []
    for i in range(256):
        reg = i << 24
        for _ in range(8):
            reg = ((reg << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if reg & 0x80000000 \
                else (reg << 1) & 0xFFFFFFFF
        table.append(reg)
    return table


_CRC_TABLE = _make_table()

# Byte bit-reversal table, used to run CRC-32/MPEG-2 through zlib's reflected
# CRC-32 (binascii.crc32) at C speed:
#   mpeg2(data) = rev32( crc32_reflected(rev8(data)) ^ 0xFFFFFFFF )
# (same polynomial and init; reflecting input and output is equivalent to
# reversing every byte and the final register; binascii also applies a final
# XOR of 0xFFFFFFFF which we undo).
_REV8 = bytes(int(f"{i:08b}"[::-1], 2) for i in range(256))


def _rev32(x: int) -> int:
    return int(f"{x:032b}"[::-1], 2)


def crc32_mpeg2_reference(data: bytes, reg: int = 0xFFFFFFFF) -> int:
    """v1 table-driven implementation; kept as the reference for tests."""
    for b in data:
        reg = ((reg << 8) & 0xFFFFFFFF) ^ _CRC_TABLE[((reg >> 24) ^ b) & 0xFF]
    return reg


def crc32_mpeg2(data: bytes) -> int:
    """CRC-32 as specified by DVB-GSE / used by GSSE. Bytes processed in order
    of increasing position, accumulator initialised to 0xFFFFFFFF."""
    return _rev32(binascii.crc32(bytes(data).translate(_REV8)) ^ 0xFFFFFFFF)


def append_crc(payload: bytes) -> bytes:
    """payload -> payload || CRC32 (big endian, 4 bytes)."""
    return payload + crc32_mpeg2(payload).to_bytes(4, "big")


def check_crc(payload_with_crc: bytes) -> bool:
    """True if the trailing 4 bytes are a valid CRC over everything before them.

    Non-reflected with no final XOR, so the CRC over data *including* its own
    CRC field is 0 for a good frame.
    """
    if len(payload_with_crc) < 4:
        return False
    return crc32_mpeg2(payload_with_crc) == 0


# --------------------------------------------------------------------------
# Byte stuffing
# --------------------------------------------------------------------------
# Why bytes.replace() is exact here (v1 avoided it):
#  * "HEA" and "TAI" cannot overlap themselves or each other (no suffix of one
#    is a prefix of either), so the v1 left-to-right scan stuffs *every*
#    occurrence, and so does replace().
#  * The inserted 0x00 is not part of either pattern, so stuffing never
#    creates a new occurrence.
#  * Hence in a stuffed stream every "HEA"/"TAI" is followed by its stuffing
#    zero, and replace("HEA\0" -> "HEA") removes exactly those zeros: a genuine
#    payload zero after "HEA" appears as "HEA\0\0" and only the first is removed.
# test_vectors.py fuzzes this against the v1 reference loops.

def stuff(data: bytes) -> bytes:
    """Insert 0x00 after every 0x484541 ("HEA") and 0x544149 ("TAI")."""
    return bytes(data).replace(b"HEA", b"HEA\x00").replace(b"TAI", b"TAI\x00")


def destuff(data: bytes) -> bytes:
    """Inverse of stuff(): 0x48454100 -> 0x484541, 0x54414900 -> 0x544149."""
    return bytes(data).replace(b"HEA\x00", b"HEA").replace(b"TAI\x00", b"TAI")


def stuff_reference(data: bytes) -> bytes:
    """v1 implementation, kept for the equivalence tests."""
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        if data[i] in (0x48, 0x54) and data[i:i + 3] in STUFF_PATTERNS:
            out += data[i:i + 3]
            out.append(0x00)
            i += 3
        else:
            out.append(data[i])
            i += 1
    return bytes(out)


def destuff_reference(data: bytes) -> bytes:
    """v1 implementation, kept for the equivalence tests."""
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        if (data[i] in (0x48, 0x54) and data[i:i + 3] in STUFF_PATTERNS
                and i + 3 < n and data[i + 3] == 0x00):
            out += data[i:i + 3]
            i += 4
        else:
            out.append(data[i])
            i += 1
    return bytes(out)


# --------------------------------------------------------------------------
# Framing
# --------------------------------------------------------------------------

def encode(payload: bytes) -> bytes:
    """Full transmit chain: one payload -> one on-the-wire GSSE frame."""
    return HEAD + stuff(append_crc(payload)) + TAIL


def decode(frame: bytes) -> bytes:
    """Full receive chain for one complete HEAD..TAIL frame.

    Raises ValueError on a bad frame.
    """
    if not frame.startswith(HEAD) or not frame.endswith(TAIL):
        raise ValueError("frame is not delimited by HEAD/TAIL markers")
    body = destuff(frame[len(HEAD):-len(TAIL)])
    if len(body) < 4:
        raise ValueError("frame too short to hold payload + CRC")
    if not check_crc(body):
        raise ValueError("CRC mismatch")
    return body[:-4]


class GsseDeframer:
    """Streaming HEAD/TAIL hunter for a lossy, continuous byte stream.

    Feed it whatever bytes arrive (partial frames are fine, byte boundaries do
    not matter) and it yields verified payloads. ICD A.4 state machine:

      * discard everything until a HEAD marker is seen
      * buffer until a TAIL marker is seen
      * de-stuff, verify CRC, hand the payload up

    plus one v2 rule: a HEAD seen while a frame is open means the previous
    TAIL was lost; abandon the open frame and start a new one at that HEAD.

    Counters: frames_ok, frames_crc_bad, frames_abandoned (open frame cut off
    by a new HEAD), frames_oversize (> max_frame with no TAIL),
    bytes_discarded (junk outside any frame - normal at start-of-pass).
    """

    def __init__(self, max_frame: int = 16 << 20):
        self.buf = bytearray()
        self.in_frame = False
        self.max_frame = max_frame
        self._scan = 0               # bytes of buf already searched in-frame
        self.frames_ok = 0
        self.frames_crc_bad = 0
        self.frames_abandoned = 0
        self.frames_oversize = 0
        self.bytes_discarded = 0
        self.max_frame_seen = 0

    def feed(self, data: bytes):
        """Consume bytes, yield complete verified payloads."""
        self.buf += data
        while True:
            if not self.in_frame:
                idx = self.buf.find(HEAD)
                if idx < 0:
                    # Keep the last 3 bytes: a marker may straddle this chunk.
                    keep = min(len(self.buf), len(HEAD) - 1)
                    self.bytes_discarded += len(self.buf) - keep
                    del self.buf[:len(self.buf) - keep]
                    return
                self.bytes_discarded += idx
                del self.buf[:idx + len(HEAD)]
                self.in_frame = True
                self._scan = 0

            start = max(0, self._scan - (len(TAIL) - 1))
            t_idx = self.buf.find(TAIL, start)
            h_idx = self.buf.find(HEAD, start)

            if h_idx >= 0 and (t_idx < 0 or h_idx < t_idx):
                # New HEAD before any TAIL: the open frame lost its TAIL.
                self.frames_abandoned += 1
                self.bytes_discarded += h_idx
                del self.buf[:h_idx + len(HEAD)]
                self._scan = 0
                continue

            if t_idx < 0:
                self._scan = len(self.buf)
                if len(self.buf) > self.max_frame:
                    self.frames_oversize += 1
                    self.in_frame = False
                    self.bytes_discarded += len(self.buf)
                    self.buf.clear()
                return

            body = destuff(self.buf[:t_idx])
            del self.buf[:t_idx + len(TAIL)]
            self.in_frame = False
            self.max_frame_seen = max(self.max_frame_seen, len(body))
            if len(body) >= 4 and check_crc(body):
                self.frames_ok += 1
                yield body[:-4]
            else:
                self.frames_crc_bad += 1
