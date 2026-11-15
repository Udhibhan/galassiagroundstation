"""Minimal IPv4 + UDP parsing, with fragment reassembly.

Only the rev 1.2 stack uses this (RF ICD 1045318 rev 1.2 fig 3.1: GSUFTP over UDP over
IPv4 over GSSE). The rev 1.0 Link X puts GSUFTP packets straight into GSSE and never gets
here, see gs_rx.py --stack.

Why reassembly: rev 1.2 GSUFTP sends 50 000-byte chunks in one UDP datagram, larger than
the radio interface MTU, so each chunk arrives as several IP fragments. Lose one fragment
and the whole chunk is lost.

v2 changes (see CHANGES.md):
  * UDP checksum is verified when non-zero (pseudo-header included). The IP ID is only
    16 bits and GSUFTP loops for a whole pass, so a stale fragment of an earlier datagram
    can get glued into a later one with the same ID. v1 would have handed that corrupted
    chunk to GSUFTP; the checksum catches it.
  * Fragment expiry is counted in packets fed, not wall-clock seconds, so a file replay
    gives the same result however fast it runs.
  * Pending-reassembly table is capped.
"""

from __future__ import annotations

PROTO_UDP = 17


def ip_checksum(data: bytes) -> int:
    if len(data) % 2:
        data += b"\x00"
    total = sum(int.from_bytes(data[i:i + 2], "big") for i in range(0, len(data), 2))
    while total >> 16:
        total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF


def _ip4(b: bytes) -> str:
    return ".".join(str(x) for x in b)


def _ip4b(s: str) -> bytes:
    return bytes(int(x) for x in s.split("."))


class IPv4Reassembler:
    """Feed whole IPv4 packets, get back complete datagrams.

    Yields (src, dst, protocol, payload) tuples.
    """

    def __init__(self, expire_after: int = 2000, max_pending: int = 256,
                 verify_checksum: bool = True):
        self.expire_after = expire_after      # packets, not seconds
        self.max_pending = max_pending
        self.verify_checksum = verify_checksum
        self.frags: dict = {}
        self.n = 0
        self.stats = {"pkt": 0, "not_ipv4": 0, "bad_cksum": 0,
                      "frag": 0, "reassembled": 0, "expired": 0}

    def feed(self, pkt: bytes):
        self.n += 1
        self.stats["pkt"] += 1
        if len(pkt) < 20 or (pkt[0] >> 4) != 4:
            self.stats["not_ipv4"] += 1
            return
        ihl = (pkt[0] & 0x0F) * 4
        total_len = int.from_bytes(pkt[2:4], "big")
        if ihl < 20 or total_len < ihl or len(pkt) < total_len:
            self.stats["not_ipv4"] += 1
            return
        if self.verify_checksum and ip_checksum(pkt[:ihl]) != 0:
            self.stats["bad_cksum"] += 1
            return
        ident = int.from_bytes(pkt[4:6], "big")
        flags_frag = int.from_bytes(pkt[6:8], "big")
        mf = bool(flags_frag & 0x2000)
        offset = (flags_frag & 0x1FFF) * 8
        proto = pkt[9]
        src, dst = _ip4(pkt[12:16]), _ip4(pkt[16:20])
        payload = pkt[ihl:total_len]

        if not mf and offset == 0:
            yield (src, dst, proto, payload)
            return

        self.stats["frag"] += 1
        key = (src, dst, ident, proto)
        entry = self.frags.setdefault(key, {"parts": {}, "total": None, "n0": self.n})
        entry["parts"][offset] = payload
        if not mf:
            entry["total"] = offset + len(payload)

        if entry["total"] is not None:
            out = bytearray()
            for off in sorted(entry["parts"]):
                if off != len(out):
                    break                 # hole - wait for more (or expire)
                out += entry["parts"][off]
            if len(out) == entry["total"]:
                del self.frags[key]
                self.stats["reassembled"] += 1
                yield (src, dst, proto, bytes(out))

        self._expire()

    def _expire(self):
        stale = [k for k, v in self.frags.items() if self.n - v["n0"] > self.expire_after]
        if len(self.frags) - len(stale) > self.max_pending:
            by_age = sorted(self.frags, key=lambda k: self.frags[k]["n0"])
            stale = by_age[:len(self.frags) - self.max_pending]
        for key in stale:
            self.frags.pop(key, None)
            self.stats["expired"] += 1


def udp_checksum(src: str, dst: str, udp: bytes) -> int:
    pseudo = _ip4b(src) + _ip4b(dst) + bytes([0, PROTO_UDP]) + len(udp).to_bytes(2, "big")
    c = ip_checksum(pseudo + udp)
    return c if c else 0xFFFF


def parse_udp(payload: bytes, src: str | None = None, dst: str | None = None,
              verify: bool = True):
    """(src_port, dst_port, data, cksum_state) or None if not plausible UDP.
    cksum_state: 'ok', 'none' (sender disabled it) or 'bad'."""
    if len(payload) < 8:
        return None
    sport = int.from_bytes(payload[0:2], "big")
    dport = int.from_bytes(payload[2:4], "big")
    length = int.from_bytes(payload[4:6], "big")
    if length < 8 or length > len(payload):
        return None
    cks = int.from_bytes(payload[6:8], "big")
    state = "none"
    if cks and verify and src and dst:
        state = "ok" if ip_checksum(
            _ip4b(src) + _ip4b(dst) + bytes([0, PROTO_UDP])
            + length.to_bytes(2, "big") + payload[:length]) == 0 else "bad"
    return sport, dport, payload[8:length], state


def build_udp_ip(src: str, dst: str, sport: int, dport: int, data: bytes,
                 ident: int = 0, mtu: int = 9000, with_checksum: bool = True):
    """Build an IPv4/UDP datagram, fragmenting to `mtu` (simulator only)."""
    udp = bytearray(sport.to_bytes(2, "big") + dport.to_bytes(2, "big"))
    udp += (8 + len(data)).to_bytes(2, "big") + b"\x00\x00"
    udp += data
    if with_checksum:
        udp[6:8] = udp_checksum(src, dst, bytes(udp)).to_bytes(2, "big")
    payload = bytes(udp)

    out = []
    max_payload = ((mtu - 20) // 8) * 8
    offset = 0
    while offset < len(payload):
        chunk = payload[offset:offset + max_payload]
        more = offset + len(chunk) < len(payload)
        flags_frag = (0x2000 if more else 0) | (offset // 8)
        hdr = bytearray([0x45, 0x00])
        hdr += (20 + len(chunk)).to_bytes(2, "big")
        hdr += ident.to_bytes(2, "big")
        hdr += flags_frag.to_bytes(2, "big")
        hdr += bytes([64, PROTO_UDP, 0, 0])
        hdr += _ip4b(src) + _ip4b(dst)
        hdr[10:12] = ip_checksum(bytes(hdr)).to_bytes(2, "big")
        out.append(bytes(hdr) + chunk)
        offset += len(chunk)
    return out


def looks_like_ipv4(pkt: bytes) -> bool:
    """Strict check used by gs_rx --stack auto: version 4, sane IHL/length and
    a correct header checksum (a ~1 in 65536 accident for non-IP data)."""
    if len(pkt) < 20 or (pkt[0] >> 4) != 4:
        return False
    ihl = (pkt[0] & 0x0F) * 4
    tl = int.from_bytes(pkt[2:4], "big")
    return ihl >= 20 and ihl <= tl <= len(pkt) and ip_checksum(pkt[:ihl]) == 0
