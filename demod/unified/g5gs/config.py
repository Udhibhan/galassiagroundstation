"""Presets, MODCOD tables and B205mini settings shared by the RX, the TX and the CLIs.

Everything here is plain Python (no GNU Radio), so GRC can evaluate it while editing.
The receiver defaults are the validated ones from part 2:
  - CCM PLS filter + dummy frames (field_test 2b/2c, Link X)
  - AGC3 ref 1.37 (03102026_agc_findings.md) + post-AGC ref 1.0 (19102026_postagc_results.md)
  - pl_freq_est_period 17 (03102026_HANDOFF.md, doppler_test/RESULTS.md)
  - num_recv_frames / num_send_frames 512 (b205mini_stress_test, 2026-10-04)
"""
from __future__ import annotations

import os
import shutil
import sys
from dataclasses import dataclass
from datetime import datetime, timezone

# K_BCH (bits) per FECFRAME size (ETSI EN 302 307-1 Tables 5a and 5b)
KBCH = {"normal": {"1/4": 16008, "1/3": 21408, "2/5": 25728, "1/2": 32208, "3/5": 38688,
                   "2/3": 43040, "3/4": 48408, "4/5": 51648, "5/6": 53840, "8/9": 57472,
                   "9/10": 58192},
        "short": {"1/4": 3072, "1/3": 5232, "2/5": 6312, "1/2": 7032, "3/5": 9552,
                  "2/3": 10632, "3/4": 11712, "4/5": 12432, "5/6": 13152, "8/9": 14232}}

# DVB-S2 MODCOD numbers as used by the Link X `modcod` parameter (QPSK and 8PSK only:
# gr-dvbs2rx cannot demodulate 16APSK/32APSK, i.e. Link X MODCOD 18-28)
MODCODS = {1: "QPSK1/4", 2: "QPSK1/3", 3: "QPSK2/5", 4: "QPSK1/2", 5: "QPSK3/5",
           6: "QPSK2/3", 7: "QPSK3/4", 8: "QPSK4/5", 9: "QPSK5/6", 10: "QPSK8/9",
           11: "QPSK9/10", 12: "8PSK3/5", 13: "8PSK2/3", 14: "8PSK3/4", 15: "8PSK5/6",
           16: "8PSK8/9", 17: "8PSK9/10"}

B205_CEILING = 61.44e6          # B205mini master clock maximum
AD936X_MAX_BW = 56e6
USB_FRAMES = 512                # num_recv_frames / num_send_frames (stress test 2026-10-04)
LIVE_DECODE_MAX_SPS = 20e6      # rough live-demod ceiling on Compewter (stress test)
MIN_FREE_BYTES = 2 << 30        # recording stops when this little is left on the disk
# Python blocks at tens of MS/s must get big chunks: with GNU Radio's default ~4k items per
# call, the GIL hand-offs between several Python block threads made a TX chain 6x slower
# (2026-10-24, Compewter; worse on the P-cores than on the shared-L2 E-cores)
PY_CHUNK = 1 << 16


@dataclass(frozen=True)
class Preset:
    name: str
    label: str
    sym_rate: float
    sps: int
    modcod: str
    frame_size: str


PRESETS = {
    "ops20": Preset("ops20", "20 MBd ops (2 sps, 40 MS/s)", 20e6, 2, "QPSK1/4", "normal"),
    "linkx2": Preset("linkx2", "Link X 2 MBd (4 sps, 8 MS/s)", 2e6, 4, "QPSK1/4", "normal"),
    "field1": Preset("field1", "Field test 1 MBd (2 sps, 2 MS/s)", 1e6, 2, "QPSK1/4", "normal"),
}
DEFAULT_PRESET = "ops20"


@dataclass(frozen=True)
class Config:
    """A preset with the user's overrides applied (0 / '' = take the preset's value)."""
    preset: str
    sym_rate: float
    sps: int
    samp_rate: float
    modcod: str            # canonical, e.g. "QPSK1/4"
    constellation: str     # "QPSK" / "8PSK"
    code_rate: str         # "1/4"
    frame_size: str        # "normal" / "short"
    modcod_num: int
    kbch_bytes: int        # BBFRAME length = K_BCH / 8 (UDP payload, gs_rx frame length)
    plframe_syms: int      # PLFRAME length in symbols (pilots on)

    def describe(self):
        return (f"{self.modcod} {self.frame_size}, {self.sym_rate / 1e6:g} MBd x {self.sps} sps "
                f"= {self.samp_rate / 1e6:g} MS/s (preset {self.preset})")


def parse_modcod(modcod) -> tuple[str, str]:
    """'4', 4, 'QPSK1/2' or '8psk2/3' -> ('QPSK', '1/2')"""
    m = str(modcod).strip().upper().replace(" ", "")
    if m.isdigit():
        if int(m) not in MODCODS:
            raise ValueError(f"MODCOD {m}: only 1-17 (QPSK/8PSK) can be decoded")
        m = MODCODS[int(m)]
    for constellation in ("QPSK", "8PSK"):
        if m.startswith(constellation):
            return constellation, m[len(constellation):]
    raise ValueError(f"MODCOD {modcod!r}: use a Link X number 1-17 or e.g. QPSK1/4, 8PSK2/3")


def plframe_syms(constellation: str, frame_size: str, pilots: bool = True) -> int:
    bits = {"QPSK": 2, "8PSK": 3}[constellation]
    nsym = (64800 if frame_size == "normal" else 16200) // bits
    slots = nsym // 90
    return 90 + nsym + (((slots - 1) // 16) * 36 if pilots else 0)


def resolve(preset=DEFAULT_PRESET, sym_rate=0, sps=0, modcod="", frame_size="") -> Config:
    if preset not in PRESETS:
        raise ValueError(f"preset {preset!r}: choose one of {', '.join(PRESETS)}")
    p = PRESETS[preset]
    sym_rate = float(sym_rate) or p.sym_rate
    sps_f = float(sps) or p.sps
    if abs(sps_f - round(sps_f)) > 1e-9 or round(sps_f) % 2:
        raise ValueError(f"sps {sps_f:g}: gr-dvbs2rx symbol_sync_cc needs an even integer")
    sps = int(round(sps_f))
    frame_size = (frame_size or p.frame_size).strip().lower()
    if frame_size not in KBCH:
        raise ValueError(f"frame size {frame_size!r}: normal or short")
    constellation, code_rate = parse_modcod(modcod or p.modcod)
    if code_rate not in KBCH[frame_size]:
        raise ValueError(f"{constellation}{code_rate} does not exist with {frame_size} FECFRAMEs")
    canon = constellation + code_rate
    num = next(k for k, v in MODCODS.items() if v == canon)
    return Config(preset, sym_rate, sps, sym_rate * sps, canon, constellation, code_rate,
                  frame_size, num, KBCH[frame_size][code_rate] // 8,
                  plframe_syms(constellation, frame_size))


def preset_choices():
    """(names, labels) for a GUI dropdown."""
    return list(PRESETS), [p.label for p in PRESETS.values()]


# ---------------------------------------------------------------------------- B205mini
RX_ANTENNAS = ("RX2", "TX/RX")       # B205mini receive ports


def antenna_name(a) -> str:
    """'rx2' / 'TX/RX' / 'txrx' -> the UHD port name; anything else is an error."""
    k = str(a).strip().upper().replace(" ", "")
    k = {"TXRX": "TX/RX", "TX-RX": "TX/RX"}.get(k, k)
    if k not in RX_ANTENNAS:
        raise ValueError(f"antenna {a!r}: the B205mini receives on {' or '.join(RX_ANTENNAS)}")
    return k


def default_mcr(fs: float) -> float:
    """Largest power-of-two multiple of fs that fits under 61.44 MHz (decimation 1, 2, 4..)."""
    d = 1
    while fs * d * 2 <= B205_CEILING + 1:
        d *= 2
    return fs * d


def default_bw(fs: float) -> float:
    return min(AD936X_MAX_BW, fs)


def device_args(args: str, fs: float, direction: str, mcr: float = 0) -> str:
    """UHD device string: user args + 512 USB frames + master clock (unless given)."""
    key = "num_recv_frames" if direction == "rx" else "num_send_frames"
    parts = [x.strip() for x in str(args).split(",") if x.strip()]
    keys = {x.split("=")[0].strip() for x in parts}
    if key not in keys:
        parts.append(f"{key}={USB_FRAMES}")
    if "master_clock_rate" not in keys:
        parts.append(f"master_clock_rate={(mcr or default_mcr(fs)):.0f}")
    return ",".join(parts)


# ---------------------------------------------------------------------------- misc
def log_event(msg, who="RX"):
    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {who} EVENT: {msg}", flush=True)


def utc_stamp(t=None) -> str:
    """2026-10-24T153012Z, for file names."""
    d = datetime.fromtimestamp(t, timezone.utc) if t else datetime.now(timezone.utc)
    return d.strftime("%Y%m%dT%H%M%SZ")


def iso_utc(t) -> str:
    """unix s -> 2026-10-24T15:30:12.345Z"""
    return datetime.fromtimestamp(t, timezone.utc).isoformat(timespec="milliseconds")[:-6] + "Z"


def free_bytes(path: str) -> int:
    p = os.path.abspath(path)
    while not os.path.exists(p):
        p = os.path.dirname(p)
    return shutil.disk_usage(p).free


def path(p):
    """'~/x' -> '/home/.../x'; '' stays ''."""
    return os.path.expanduser(str(p)) if p else ""


def default_out_dir(mode, input_file="", rec_dir="."):
    """Where decoded files go unless out_dir is given: next to the input file in file mode
    (<file>_out), else <rec_dir>/downlink_<UTC>."""
    if mode == "file" and input_file:
        return os.path.splitext(path(input_file))[0] + "_out"
    return os.path.join(path(rec_dir) or ".", "downlink_" + utc_stamp())


def panels_set(panels) -> set:
    """'all' / 'none' / 'spectrum,status,...' -> set of panel names."""
    s = str(panels).strip().lower()
    if s in ("", "none", "0", "off"):
        return set()
    if s == "all":
        return set(PANELS)
    out = {x.strip() for x in s.split(",") if x.strip()}
    bad = out - set(PANELS)
    if bad:
        print(f"unknown panel(s) {sorted(bad)}; have {', '.join(PANELS)}", file=sys.stderr)
    return out & set(PANELS)


PANELS = ("spectrum", "constellation", "status", "doppler", "files")
