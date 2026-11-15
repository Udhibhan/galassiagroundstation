"""
GSUFTP - GomSpace Unidirectional File Transfer Protocol (receiver side).

References:
  * rev 1.2 (later SX): RF ICD 1045318 rev 1.2 appendix B - GSUFTP over UDP,
    50 000-byte chunks, header = three NUL-terminated ASCII strings.
  * rev 1.0 (current Link X): GSUFTP User Manual 1038382 rev 1.1 sec. 4 -
    files split into "~10 MB packets which are prepended with information
    regarding the file name, packet number, etc.", carried directly in GSSE
    (Data ICD 1045208 fig 2.4). The rev 1.0 header layout is NOT documented.
    This module ASSUMES it is the same NUL-terminated ASCII layout; the first
    real capture confirms or refutes that (gs_rx.py --dump-payloads).

Wire format (rev 1.2 ICD figure B.1):

    <file identifier>\\0<total chunk number>\\0<chunk number>\\0<file data>

v2 changes (see CHANGES.md):
  * No fixed chunk size: files are assembled by concatenating chunks 1..N in
    order, and the observed chunk size is reported. A size mismatch between
    non-final chunks is flagged (expected 50 000 for rev 1.2).
  * A packet whose total-chunk count disagrees with what is held for that
    file id restarts that file (new version of the file on the spacecraft)
    instead of being silently mixed in.
  * SHA-256 + MD5 of every completed file are printed and written to
    <out>/MANIFEST.txt for the test record.
"""

from __future__ import annotations

import hashlib
import os
import time

CHUNK_SIZE_REV12 = 50000   # rev 1.2 ICD B.2 "chunks of 50 kB"
CHUNK_SIZE = CHUNK_SIZE_REV12   # v1 name, used by tx_simulator


def chunk_offset(chunk_number: int, chunk_size: int = CHUNK_SIZE_REV12) -> int:
    """Byte offset of a 1-indexed chunk within the reassembled file."""
    return (chunk_number - 1) * chunk_size


def parse_packet(pkt: bytes, max_name: int = 4096):
    """Split one GSUFTP packet into (file_id, total_chunks, chunk, data).

    Raises ValueError if the header is malformed - which is also how a
    payload that is not GSUFTP at all is recognised.
    """
    fields = []
    start = 0
    for i in range(3):
        limit = start + (max_name if i == 0 else 12)
        end = pkt.find(b"\x00", start, limit + 1)
        if end < 0:
            raise ValueError("truncated / non-GSUFTP header")
        fields.append(pkt[start:end])
        start = end + 1
    file_id, total_s, chunk_s = fields
    try:
        file_id_str = file_id.decode("ascii")
        if not file_id_str.isprintable():
            raise ValueError("non-printable file id")
        total = int(total_s.decode("ascii"))
        chunk = int(chunk_s.decode("ascii"))
    except (UnicodeDecodeError, ValueError) as exc:
        raise ValueError(f"non-ASCII / non-numeric GSUFTP header: {exc}")
    if not total_s.isdigit() or not chunk_s.isdigit():
        raise ValueError("chunk fields must be decimal digits")
    if not file_id_str or total < 1 or not (1 <= chunk <= total):
        raise ValueError("implausible GSUFTP header values")
    return file_id_str, total, chunk, pkt[start:]


def build_packet(file_id: str, total: int, chunk: int, data: bytes) -> bytes:
    """Inverse of parse_packet(); used for testing and bench transmitters."""
    return (file_id.encode("ascii") + b"\x00"
            + str(total).encode("ascii") + b"\x00"
            + str(chunk).encode("ascii") + b"\x00"
            + data)


def _safe_join(root: str, rel: str) -> str:
    """Join a spacecraft-supplied relative path onto the output directory
    without letting '../..' escape it."""
    rel = rel.replace("\\", "/").lstrip("/")
    parts = [p for p in rel.split("/") if p not in ("", ".", "..")]
    if not parts:
        raise ValueError("empty file identifier")
    return os.path.join(root, *parts)


class GsuftpReceiver:
    """Implements the seven receiver steps of rev 1.2 ICD B.3 (same logic is
    used for rev 1.0). Chunks are held in RAM until a file is complete."""

    def __init__(self, out_dir: str, verbose: bool = True,
                 expected_chunk: int | None = None):
        self.out_dir = out_dir
        self.verbose = verbose
        self.expected_chunk = expected_chunk
        self.partial: dict[str, dict] = {}
        self.completed: dict[str, dict] = {}     # file_id -> info
        self.stats = {"pkt": 0, "dup": 0, "new": 0, "bad": 0, "files": 0,
                      "restarted": 0, "size_warn": 0}
        self.chunk_sizes = set()

    def handle_packet(self, pkt: bytes):
        """Process one GSUFTP packet. Returns the output path when a file was
        just completed, else None."""
        try:
            file_id, total, chunk, data = parse_packet(pkt)
        except ValueError:
            self.stats["bad"] += 1
            return None
        self.stats["pkt"] += 1

        # B.3 steps 2-3: already have the whole file? discard.
        if file_id in self.completed:
            self.stats["dup"] += 1
            return None

        entry = self.partial.get(file_id)
        if entry is not None and entry["total"] != total:
            self.stats["restarted"] += 1
            if self.verbose:
                print(f"  {file_id}: total changed {entry['total']} -> {total}, restarting")
            entry = None
        if entry is None:
            entry = {"total": total, "chunks": {}, "first_seen": time.time()}
            self.partial[file_id] = entry

        # B.3 steps 4-5: already have this chunk? discard.
        if chunk in entry["chunks"]:
            self.stats["dup"] += 1
            return None

        # B.3 step 6: store it.
        entry["chunks"][chunk] = data
        self.stats["new"] += 1
        if chunk < total:
            self.chunk_sizes.add(len(data))
            if self.expected_chunk and len(data) != self.expected_chunk:
                self.stats["size_warn"] += 1
        if self.verbose:
            print(f"  {file_id}: chunk {chunk}/{total} "
                  f"({len(entry['chunks'])}/{total} held, {len(data)} B)")

        # B.3 step 7: complete? write it out.
        if len(entry["chunks"]) == total:
            return self._write(file_id, entry)
        return None

    def _write(self, file_id: str, entry: dict) -> str | None:
        try:
            path = _safe_join(self.out_dir, file_id)
        except ValueError:
            return None
        sizes = {len(entry["chunks"][n]) for n in range(1, entry["total"])}
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        sha, md5 = hashlib.sha256(), hashlib.md5()
        with open(path, "wb") as f:
            for n in range(1, entry["total"] + 1):
                f.write(entry["chunks"][n])
                sha.update(entry["chunks"][n])
                md5.update(entry["chunks"][n])
        info = {"path": path, "size": os.path.getsize(path),
                "sha256": sha.hexdigest(), "md5": md5.hexdigest(),
                "chunks": entry["total"], "chunk_size": sorted(sizes)}
        del self.partial[file_id]
        self.completed[file_id] = info
        self.stats["files"] += 1
        with open(os.path.join(self.out_dir, "MANIFEST.txt"), "a") as m:
            m.write(f"{info['sha256']}  {file_id}  size={info['size']} "
                    f"chunks={info['chunks']} md5={info['md5']}\n")
        if self.verbose:
            warn = "" if len(sizes) <= 1 else f"  WARNING unequal chunk sizes {sorted(sizes)}"
            print(f"  COMPLETE -> {path} ({info['size']} B) sha256={info['sha256']}{warn}")
        return path

    def report(self) -> str:
        s = self.stats
        lines = [f"GSUFTP: {s['files']} file(s) complete, {s['new']} new chunks, "
                 f"{s['dup']} duplicates, {s['bad']} unparseable, "
                 f"{s['restarted']} restarted"]
        if self.chunk_sizes:
            lines.append(f"  non-final chunk size(s) seen: {sorted(self.chunk_sizes)} B"
                         + (f"  ({s['size_warn']} differ from expected "
                            f"{self.expected_chunk})" if s["size_warn"] else ""))
        for file_id, info in sorted(self.completed.items()):
            lines.append(f"  OK  {file_id}: {info['size']} B sha256={info['sha256']}")
        for file_id, e in sorted(self.partial.items()):
            missing = [n for n in range(1, e["total"] + 1) if n not in e["chunks"]]
            lines.append(f"  INCOMPLETE {file_id}: have {len(e['chunks'])}/"
                         f"{e['total']}, missing {missing[:20]}"
                         + (" ..." if len(missing) > 20 else ""))
        return "\n".join(lines)
