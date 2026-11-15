#!/usr/bin/env python3
"""
gsuftp_combine - laptop replacement for GomSpace `gsuftp-combine` (no host build).

GSUFTP User Manual 1038382 sec. 3 and 6: `gsuftp-split` writes the file parts
plus a metadata file holding "the name of the original file, the name of the
resulting file parts, and the MD5 hashes of both the original file and its
parts". The *layout* of that metadata file is not documented, so this tool
does not assume one. It reads every small file in the input directory that
names other files in the directory, and accepts it as the metadata for a set
of parts only if all of these hold:

  * every part it names exists, and each part's MD5 appears in it;
  * the parts, joined in the order the metadata names them, have an MD5 that
    also appears in it (the original file's MD5).

Only then is the combined file written. Nothing is written on any mismatch,
so a wrong guess about the format cannot produce a wrong file silently.
Encrypted splits (--key) are not supported: the plan does not use them.

  python3 gsuftp_combine.py C3_parts/ C3_files/             # combine
  python3 gsuftp_combine.py C3_parts/ --inspect             # show what it found
  python3 gsuftp_combine.py C3_parts/ C3_files/ --order name --name tv2_random_1M.bin \\
          --part-md5 txsplit.md5                             # fallback, see below

Fallback when the metadata cannot be interpreted (e.g. binary): --order name
joins the parts in natural name order; --part-md5 checks each part against an
`md5sum` listing taken on the SDR (`md5sum /data/g5test/txsplit/*`), and
--name sets the output name. The result must still be checked with sha256sum.

Exit code: 0 if at least one file was combined and every check passed.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import re
import sys

HEX32 = re.compile(rb"(?<![0-9a-fA-F])[0-9a-fA-F]{32}(?![0-9a-fA-F])")
MAX_META = 1 << 20          # a metadata file is small; parts may be anything


def md5(path: str) -> str:
    h = hashlib.md5()
    with open(path, "rb") as f:
        for blk in iter(lambda: f.read(1 << 20), b""):
            h.update(blk)
    return h.hexdigest()


def natural_key(name: str):
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", name)]


def list_files(d: str) -> list[str]:
    """Relative paths of all files under d (gs_rx may keep sub-directories)."""
    out = []
    for root, _, names in os.walk(d):
        for n in names:
            if n == "MANIFEST.txt":
                continue
            out.append(os.path.relpath(os.path.join(root, n), d))
    return sorted(out, key=natural_key)


def find_mentions(text: bytes, names: list[str]) -> list[str]:
    """Files whose name (or basename) appears in text as a whole token, in order of
    first appearance. Longer names claim their bytes first, so 'x.part10' is not also
    counted as 'x.part1'."""
    cands = {}
    for rel in names:
        for key in {rel, os.path.basename(rel)}:
            cands.setdefault(key, rel)
    taken = bytearray(len(text))
    hits = []
    for key in sorted(cands, key=len, reverse=True):
        pat = rb"(?<![\w.\-])" + re.escape(key.encode()) + rb"(?![\w.\-])"
        for m in re.finditer(pat, text):
            if any(taken[m.start():m.end()]):
                continue
            taken[m.start():m.end()] = b"\x01" * (m.end() - m.start())
            hits.append((m.start(), cands[key]))
    order = []
    for _, rel in sorted(hits):
        if rel not in order:
            order.append(rel)
    return order


def guess_output_name(text: bytes, parts: list[str], meta_rel: str) -> str | None:
    """A file-name-looking token in the metadata that is not a part or the metadata."""
    skip = {os.path.basename(p) for p in parts} | {os.path.basename(meta_rel)}
    for tok in re.findall(rb"[\w][\w.\-/]*\.[\w]+|[\w][\w.\-]{2,}", text):
        name = os.path.basename(tok.decode("utf-8", "replace"))
        if name in skip or HEX32.fullmatch(tok) or name.isdigit():
            continue
        if any(name in os.path.basename(p) for p in parts) or "." in name:
            return name
    return None


def try_metadata(in_dir: str, meta_rel: str, files: list[str], part_md5: dict):
    path = os.path.join(in_dir, meta_rel)
    if os.path.getsize(path) > MAX_META:
        return None
    text = open(path, "rb").read()
    hexes = {h.decode().lower() for h in HEX32.findall(text)}
    if not hexes:
        return None
    parts = [f for f in find_mentions(text, [f for f in files if f != meta_rel])]
    if not parts:
        return None
    sums = {p: part_md5.get(p) or md5(os.path.join(in_dir, p)) for p in parts}
    missing = [p for p in parts if sums[p] not in hexes]
    whole = hashlib.md5()
    for p in parts:
        with open(os.path.join(in_dir, p), "rb") as f:
            for blk in iter(lambda: f.read(1 << 20), b""):
                whole.update(blk)
    return {"meta": meta_rel, "parts": parts, "part_md5": sums, "bad_parts": missing,
            "whole_md5": whole.hexdigest(), "whole_ok": whole.hexdigest() in hexes,
            "hexes": hexes, "name": guess_output_name(text, parts, meta_rel),
            "text": text}


def write_combined(in_dir: str, parts: list[str], out_path: str):
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    sha, m5 = hashlib.sha256(), hashlib.md5()
    with open(out_path, "wb") as out:
        for p in parts:
            with open(os.path.join(in_dir, p), "rb") as f:
                for blk in iter(lambda: f.read(1 << 20), b""):
                    out.write(blk)
                    sha.update(blk)
                    m5.update(blk)
    return sha.hexdigest(), m5.hexdigest()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("in_dir", help="received parts + metadata (gs_rx --out directory)")
    ap.add_argument("out_dir", nargs="?", help="where combined files are written")
    ap.add_argument("--inspect", action="store_true",
                    help="print the candidate metadata files and what was matched; write nothing")
    ap.add_argument("--order", choices=["meta", "name"], default="meta",
                    help="meta (default): part order from the metadata; name: natural name order "
                         "of every file except --exclude, for metadata that cannot be read")
    ap.add_argument("--exclude", action="append", default=[],
                    help="with --order name: file(s) to leave out (e.g. the metadata file)")
    ap.add_argument("--name", help="output file name (default: taken from the metadata)")
    ap.add_argument("--part-md5", help="md5sum listing of the parts taken on the SDR; each "
                                       "received part must match it")
    a = ap.parse_args(argv)

    files = list_files(a.in_dir)
    if not files:
        sys.exit(f"{a.in_dir}: no files")

    ref = {}
    if a.part_md5:
        for line in open(a.part_md5):
            bits = line.split()
            if len(bits) >= 2 and HEX32.fullmatch(bits[0].encode()):
                ref[os.path.basename(bits[-1].lstrip("*"))] = bits[0].lower()

    def check_ref(parts):
        bad = []
        for p in parts:
            want = ref.get(os.path.basename(p))
            if ref and want is None:
                bad.append(f"{p}: not in {a.part_md5}")
            elif want and want != md5(os.path.join(a.in_dir, p)):
                bad.append(f"{p}: MD5 differs from {a.part_md5}")
        return bad

    if a.order == "name":
        parts = [f for f in files if os.path.basename(f) not in
                 {os.path.basename(x) for x in a.exclude}]
        print(f"joining {len(parts)} part(s) in name order: {', '.join(parts)}")
        bad = check_ref(parts)
        for b in bad:
            print(f"  FAIL {b}")
        if not a.name or not a.out_dir:
            sys.exit("--order name needs out_dir and --name")
        if bad:
            sys.exit("not written")
        sha, m5 = write_combined(a.in_dir, parts, os.path.join(a.out_dir, a.name))
        print(f"  wrote {os.path.join(a.out_dir, a.name)} md5={m5} sha256={sha}")
        print("  (no metadata check: confirm with sha256sum -c)")
        return 0

    results = []
    for rel in files:
        r = try_metadata(a.in_dir, rel, files, {})
        if r:
            results.append(r)
    if not results:
        print("no file here names other files together with MD5 hashes, so no metadata "
              "file was recognised.\nInspect the candidates (e.g. xxd <file> | head) and "
              "use --order name --name ... --part-md5 ... instead.")
        return 1

    status = 0
    done = 0
    for r in results:
        ok = r["whole_ok"] and not r["bad_parts"]
        refbad = check_ref(r["parts"]) if ref else []
        ok = ok and not refbad
        print(f"metadata {r['meta']}: {len(r['parts'])} part(s) "
              f"[{', '.join(r['parts'])}]")
        for p in r["parts"]:
            flag = "ok " if p not in r["bad_parts"] else "BAD"
            print(f"  {flag} {p} md5={r['part_md5'][p]}")
        for b in refbad:
            print(f"  FAIL {b}")
        print(f"  joined md5={r['whole_md5']} "
              + ("matches the metadata (original file)" if r["whole_ok"] else
                 "NOT in the metadata - wrong order, missing part or different format"))
        if a.inspect:
            shown = r["text"][:600]
            print("  metadata content (first 600 B):")
            try:
                print("    " + shown.decode("utf-8").replace("\n", "\n    "))
            except UnicodeDecodeError:
                print("    (binary) " + shown.hex())
            continue
        if not ok:
            print("  not written")
            status = 1
            continue
        name = a.name or r["name"]
        if not name:
            print("  output name not found in the metadata: rerun with --name")
            status = 1
            continue
        if not a.out_dir:
            sys.exit("out_dir is required unless --inspect")
        out = os.path.join(a.out_dir, os.path.basename(name))
        sha, m5 = write_combined(a.in_dir, r["parts"], out)
        print(f"  wrote {out} md5={m5} sha256={sha}")
        done += 1
    if not a.inspect and done == 0:
        status = 1
    return status


if __name__ == "__main__":
    sys.exit(main())
