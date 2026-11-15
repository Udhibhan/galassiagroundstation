"""Qt panels for the unified RX / TX GUIs, and the switch that turns panels off.

Each panel is a QWidget made by a GRC block. It gets the top block (`self` in the generated
code) and finds the blocks it shows by their GRC ids, on its first timer tick. Panels poll
(the GNU Radio getters are cheap) and do nothing while inactive.

PanelSwitch: one checkbox per panel group. Unticking a group hides its widgets, pauses its
panels and disables its valves (blocks.copy in front of each Qt sink), so a switched-off
panel costs no CPU. The initial state comes from the `panels` parameter (all / none / list).
"""
from __future__ import annotations

import os
import threading
import time

from PyQt5 import Qt, QtCore

from . import config as C
from . import probe

GREEN, RED, AMBER, GREY = "#1a7f37", "#c62828", "#b26a00", "#777777"


def _fmt_s(s):
    if s is None:
        return "-"
    s = int(round(s))
    if abs(s) >= 3600:
        return f"{s // 3600}h{(abs(s) % 3600) // 60:02d}m"
    if abs(s) >= 60:
        return f"{s // 60}m{abs(s) % 60:02d}s"
    return f"{s}s"


def _gb(b):
    return f"{b / 2**30:.1f} GB"


class _Panel(Qt.QGroupBox):
    """Base: a titled grid of label rows, polled by a QTimer while active."""

    def __init__(self, top, title, interval_ms=500, active=True):
        super().__init__(title)
        self.top = top
        self.grid = Qt.QGridLayout(self)
        self.grid.setColumnStretch(1, 1)
        self.rows = {}
        self.b = None
        self.timer = Qt.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.interval = interval_ms
        self.set_active(active)

    def row(self, key, label):
        r = self.grid.rowCount()
        lab = Qt.QLabel(label)
        lab.setStyleSheet(f"color: {GREY};")
        val = Qt.QLabel("-")
        val.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        val.setWordWrap(True)
        self.grid.addWidget(lab, r, 0, QtCore.Qt.AlignTop)
        self.grid.addWidget(val, r, 1)
        self.rows[key] = val
        return val

    def put(self, key, text, color=None):
        w = self.rows[key]
        w.setText(text)
        w.setStyleSheet(f"color: {color};" if color else "")

    def set_active(self, on):
        self.active = bool(on)
        if on:
            self.timer.start(self.interval)
        else:
            self.timer.stop()

    def _tick(self):
        if self.b is None:
            self.b = probe.resolve(self.top)
            self.setup()
        try:
            self.update_view()
        except Exception as e:                  # noqa: BLE001  (keep the GUI alive)
            self.setTitle(f"{self.title().split(' [')[0]} [error: {e}]")

    def setup(self):
        pass

    def update_view(self):
        pass


# ---------------------------------------------------------------------------- RX status
class RxStatusPanel(_Panel):
    def __init__(self, top, sym_rate, cfg_text="", dummy_frames=True, active=True):
        super().__init__(top, "Receiver status", 500, active)
        self.sym_rate, self.dummy = sym_rate, dummy_frames
        self.row("cfg", "Settings").setText(cfg_text)
        for k, lab in (("input", "Input"), ("rec", "Recording"), ("disk", "Disk"),
                       ("level", "Input level"), ("ovf", "Overflows"), ("gate", "Demod"),
                       ("lock", "PL sync"), ("cfo", "Carrier offset"), ("snr", "SNR"),
                       ("frames", "PLFRAMEs"), ("bch", "BCH / LDPC"), ("gs", "GSSE / files")):
            self.row(k, lab)
        self.was_locked = None
        self.t_lock = None
        self.done_logged = False
        # the end-of-run link summary is printed just before the GSSE report
        Qt.QTimer.singleShot(0, self._hook)

    def _hook(self):
        dec = getattr(self.top, probe.IDS["dec"], None)
        if dec is not None:
            dec.before_report.append(
                lambda: probe.print_summary(self.top, self.sym_rate, self.dummy))

    def setup(self):
        src = self.b["src"]
        if src is not None and src.mode == "file":
            # the radio controls do nothing in file mode
            for name in ("_rx_gain_slider_win", "_antenna_sel_tool_bar"):
                w = getattr(self.top, name, None)
                if w is not None:
                    w.setVisible(False)
            for cb in self.top.findChildren(Qt.QCheckBox):
                if cb.text() == "Record to disk":
                    cb.setVisible(False)

    def update_view(self):
        b = self.b
        src, gate, dec = b["src"], b["gate"], b["dec"]
        st = src.stats() if src is not None else {}
        mode = st.get("mode", "?")
        if mode == "file":
            p = st.get("progress") or 0.0
            fin = dec is not None and dec.finished
            self.put("input", f"file, {100 * p:.1f} % of {st.get('file_seconds', 0):.1f} s"
                     + ("  - FINISHED" if fin else ""), GREEN if fin else None)
            for k in ("rec", "disk", "level", "ovf"):
                self.put(k, "-")
        else:
            self.put("input", f"B205mini ({'record + monitor' if mode == 'record' else 'live decode'}),"
                     f" port {st.get('antenna', '?')}")
            if st.get("recording"):
                self.put("rec", f"{os.path.basename(st['path'])}  {st['seconds']:.0f} s, "
                         f"{st['bytes'] / 1e9:.1f} GB", GREEN)
            else:
                self.put("rec", st.get("warning") or f"off ({st.get('status', '')})",
                         RED if st.get("warning") else GREY)
            left = st.get("seconds_left")
            self.put("disk", f"{_gb(st.get('free', 0))} free, {_fmt_s(left)} of recording "
                     f"before the {C.MIN_FREE_BYTES / 2**30:g} GB stop",
                     RED if left is not None and left < 60 else AMBER if left and left < 300
                     else None)
            lv, pk = st.get("level_dbfs"), st.get("peak_dbfs")
            if lv is not None:
                clip = pk is not None and pk > -1.0
                self.put("level", f"{lv:.1f} dBFS rms, peak {pk:.1f} dBFS"
                         + ("  CLIPPING: lower the RX gain" if clip else ""),
                         RED if clip else None)
            self.put("ovf", f"{st.get('overflows', 0)} gaps, {st.get('lost', 0)} samples lost",
                     RED if st.get("overflows") else None)
        if gate is not None:
            g = gate.stats()
            if g["mode"] == "duty":
                age = time.time() - g["last_window_unix"] if g.get("last_window_unix") else None
                self.put("gate", f"duty-cycled: {g['window_s']:g} s every {g['period_s']:g} s, "
                         f"{g['windows']} windows" + (f", last {_fmt_s(age)} ago" if age else ""))
            elif g["mode"] == "drop":
                self.put("gate", f"continuous; {g['dropped']} samples dropped (demod behind), "
                         f"queue {100 * g['queue_fill']:.0f} %",
                         RED if g["dropped"] else None)
            else:
                self.put("gate", "every sample (file)")
        s = probe.link_stats(b, self.sym_rate, self.dummy)
        if "locked" in s:
            lk = s["locked"]
            if lk != self.was_locked:
                if self.was_locked is not None:
                    C.log_event("PL sync " + ("LOCKED" if lk else "lost lock"))
                self.was_locked = lk
            self.put("lock", "LOCKED" if lk else "searching", GREEN if lk else RED)
            self.put("cfo", f"{s['cfo_hz'] / 1e3:+.3f} kHz")
            self.put("frames", f"{s['sof']} SOF, {s['plframes']} data, {s['dummy']} dummy"
                     + (f", {s['rejected']} rejected" if s.get("rejected") else ""))
        if "snr_db" in s:
            self.put("snr", f"{s['snr_db']:.1f} dB (demapper estimate)")
        if "bch_frames" in s:
            self.put("bch", f"{s['bch_frames']} frames, {s['bch_errors']} uncorrectable"
                     + (f", LDPC {s['ldpc_iter']:.1f} it" if "ldpc_iter" in s else ""),
                     AMBER if s["bch_errors"] else None)
        if dec is not None:
            d = dec.stats()
            nfiles = len(d["files"])
            self.put("gs", f"{d['bbframes']} BBFRAMEs, GSSE {d['gsse_ok']} ok / "
                     f"{d['gsse_crc_bad']} CRC fail, {nfiles} file(s)"
                     + (f"; hashes {'PASS' if d['ok'] else 'FAIL'}" if d["ok"] is not None else ""),
                     GREEN if d["ok"] else RED if d["ok"] is False else None)


# ---------------------------------------------------------------------------- Doppler
class DopplerPanel(_Panel):
    """Pass information + predicted Doppler vs measured residual carrier offset."""

    def __init__(self, top, sym_rate, history_s=600, active=True):
        super().__init__(top, "Doppler / pass", 500, active)
        self.sym_rate = sym_rate
        self.history = history_s
        for k, lab in (("tle", "TLE"), ("now", "Sample time"), ("geo", "El / Az / range"),
                       ("dop", "Predicted Doppler"), ("res", "Residual (PL sync)"),
                       ("next", "Pass")):
            self.row(k, lab)
        self.plot = None
        try:
            import pyqtgraph as pg
            pg.setConfigOptions(antialias=True)
            self.plot = pg.PlotWidget()
            self.plot.setMinimumHeight(180)
            self.plot.addLegend()
            self.plot.showGrid(x=True, y=True, alpha=0.3)
            self.plot.setLabel("left", "kHz")
            self.plot.setLabel("bottom", "s (sample time)")
            self.c_pred = self.plot.plot([], [], pen=pg.mkPen("#1f77b4", width=2),
                                         name="predicted Doppler (removed)")
            self.c_meas = self.plot.plot([], [], pen=pg.mkPen("#d62728", width=2),
                                         name="residual CFO (PL sync)")
            self.grid.addWidget(self.plot, self.grid.rowCount(), 0, 1, 2)
        except ImportError:
            pass
        self.table = Qt.QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["AOS (UTC)", "TCA", "LOS", "max el", "Doppler kHz"])
        self.table.horizontalHeader().setSectionResizeMode(Qt.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setMaximumHeight(150)
        self.grid.addWidget(self.table, self.grid.rowCount(), 0, 1, 2)
        self.t, self.pred, self.meas = [], [], []
        self.t_origin = None
        self.passes = None
        self._pass_thread = None

    def setup(self):
        pc = self.b["precomp"]
        if pc is None or not pc.active:
            self.put("tle", "none: Doppler pre-compensation OFF (load a TLE to enable)", GREY)
            return
        s = pc.satrec
        from . import doppler_lib as dp
        age = (time.time() - dp.tle_epoch_unix(s)) / 86400
        self.put("tle", f"{pc.model.name} (NORAD {s.satnum}), epoch "
                 f"{C.iso_utc(dp.tle_epoch_unix(s))} ({age:+.1f} d)",
                 AMBER if abs(age) > 3 else None)

    def _load_passes(self, t0):
        try:
            self.passes = self.b["precomp"].passes(t0, 24.0)
        except Exception as e:                  # noqa: BLE001
            self.passes = []
            C.log_event(f"pass prediction failed: {e}")

    def update_view(self):
        pc, p = self.b["precomp"], self.b["plsync"]
        meas = p.get_freq_offset() * self.sym_rate if p is not None else None
        if pc is None or not pc.active:
            if meas is not None:
                self.put("res", f"{meas / 1e3:+.3f} kHz (no pre-comp: this is the whole offset)")
            return
        t = pc.now_unix()
        if t is None:
            return
        if self.passes is None and self._pass_thread is None:
            self._pass_thread = threading.Thread(target=self._load_passes, args=(t,), daemon=True)
            self._pass_thread.start()
        g = pc.predicted(t)
        self.put("now", C.iso_utc(t))
        self.put("geo", f"{g['el_deg']:.1f} deg / {g['az_deg']:.1f} deg / "
                 f"{g['range_m'] / 1e3:.0f} km", None if g["el_deg"] > 0 else GREY)
        self.put("dop", f"{g['doppler_hz'] / 1e3:+.2f} kHz at RF (sign {pc.sign:+g}"
                 + (f", + {pc.offset_hz:g} Hz offset" if pc.offset_hz else "") + ")")
        if meas is not None:
            self.put("res", f"{meas / 1e3:+.3f} kHz", AMBER if abs(meas) > 0.1 * self.sym_rate
                     else None)
        self._update_pass(t)
        if self.plot is not None:
            if self.t_origin is None:
                self.t_origin = t
                self.plot.setLabel("bottom", f"s after {C.iso_utc(t)[11:19]} UTC (sample time)")
            self.t.append(t - self.t_origin)
            self.pred.append(g["doppler_hz"] / 1e3)
            self.meas.append(meas / 1e3 if meas is not None else float("nan"))
            while self.t and self.t[-1] - self.t[0] > self.history:
                del self.t[0], self.pred[0], self.meas[0]
            self.c_pred.setData(self.t, self.pred)
            self.c_meas.setData(self.t, self.meas)

    def _update_pass(self, t):
        if not self.passes:
            self.put("next", "computing passes..." if self.passes is None else
                     "no pass above 5 deg in the next 24 h")
            return
        cur = next((x for x in self.passes if x["aos_unix"] <= t <= x["los_unix"]), None)
        nxt = next((x for x in self.passes if x["aos_unix"] > t), None)
        if cur:
            dt = cur["tca_unix"] - t
            tca = f"TCA in {_fmt_s(dt)}" if dt >= 0 else f"TCA {_fmt_s(-dt)} ago"
            self.put("next", f"IN PASS: {tca}, LOS in "
                     f"{_fmt_s(cur['los_unix'] - t)} (max el {cur['max_el_deg']:.0f} deg)", GREEN)
        elif nxt:
            self.put("next", f"next AOS in {_fmt_s(nxt['aos_unix'] - t)} "
                     f"(max el {nxt['max_el_deg']:.0f} deg, {nxt['duration_s']:.0f} s)")
        if self.table.rowCount() == 0:
            for x in self.passes[:12]:
                r = self.table.rowCount()
                self.table.insertRow(r)
                for c, v in enumerate((x["aos_utc"][:19], x["tca_utc"][11:19], x["los_utc"][11:19],
                                       f"{x['max_el_deg']:.1f}",
                                       f"{x['doppler_aos_hz'] / 1e3:+.0f} .. "
                                       f"{x['doppler_los_hz'] / 1e3:+.0f}")):
                    self.table.setItem(r, c, Qt.QTableWidgetItem(v))


# ---------------------------------------------------------------------------- files
class FilesPanel(_Panel):
    def __init__(self, top, active=True):
        super().__init__(top, "GSSE / GSUFTP", 1000, active)
        for k, lab in (("bb", "BBFRAMEs"), ("gsse", "GSSE"), ("kinds", "Payloads"),
                       ("ftp", "GSUFTP"), ("partial", "In progress"), ("hash", "Hash check")):
            self.row(k, lab)
        self.table = Qt.QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["file", "size (B)", "sha256", "check"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.grid.addWidget(self.table, self.grid.rowCount(), 0, 1, 2)
        self.shown = 0

    def update_view(self):
        dec = self.b["dec"]
        if dec is None:
            return
        d = dec.stats()
        self.put("bb", f"{d['bbframes']} ok, {d['resyncs']} re-syncs, {d['partial']} partial")
        self.put("gsse", f"{d['gsse_ok']} ok, {d['gsse_crc_bad']} CRC-32 fail, "
                 f"{d['gsse_abandoned']} abandoned", AMBER if d["gsse_crc_bad"] else None)
        k = d["kinds"]
        self.put("kinds", f"{k['gsuftp_direct']} GSUFTP direct (rev 1.0), {k['ipv4']} IPv4 "
                 f"(rev 1.2), {k['unknown']} unknown"
                 + (f"; UDP raw {d['udp_raw_bytes']} B" if d["udp_raw_bytes"] else ""),
                 AMBER if k["unknown"] else None)
        f = d["gsuftp"]
        self.put("ftp", f"{f['files']} file(s), {f['new']} new chunks, {f['dup']} duplicates, "
                 f"{f['bad']} unparseable")
        self.put("partial", ", ".join(f"{os.path.basename(n)} {h}/{t}"
                                      for n, (h, t) in list(d["partial_files"].items())[:6]) or "-")
        want = d["want"]
        if want:
            got = {os.path.basename(fid) for fid, _, _ in d["files"]}
            passed = sum(1 for _, _, v in d["files"] if v == "PASS")
            self.put("hash", f"{passed}/{len(want)} expected files PASS"
                     + (f", missing {', '.join(sorted(set(want) - got))}" if set(want) - got
                        else ""), GREEN if passed == len(want) else None)
        else:
            self.put("hash", "no hash file given", GREY)
        for fid, info, verdict in d["files"][self.shown:]:
            r = self.table.rowCount()
            self.table.insertRow(r)
            for c, v in enumerate((fid, str(info["size"]), info["sha256"][:16] + "...",
                                   verdict or "-")):
                it = Qt.QTableWidgetItem(v)
                if c == 3 and verdict in ("PASS", "FAIL"):
                    it.setForeground(Qt.QColor(GREEN if verdict == "PASS" else RED))
                self.table.setItem(r, c, it)
            self.shown += 1


# ---------------------------------------------------------------------------- TX status
class TxStatusPanel(_Panel):
    def __init__(self, top, ids, cfg_text="", active=True):
        super().__init__(top, "Transmitter status", 500, active)
        self.ids = ids
        self.row("cfg", "Settings").setText(cfg_text)
        for k, lab in (("payload", "Payload"), ("frames", "PLFRAMEs"), ("imp", "Impairments"),
                       ("dop", "Doppler sim"), ("out", "Output")):
            self.row(k, lab)

    def _tick(self):
        if self.b is None:
            self.b = {k: getattr(self.top, v, None) for k, v in self.ids.items()}
        try:
            self.update_view()
        except Exception as e:                  # noqa: BLE001
            self.setTitle(f"Transmitter status [error: {e}]")

    def update_view(self):
        b = self.b
        if b.get("payload") is not None:
            self.put("payload", b["payload"].describe_progress())
        if b.get("framer") is not None:
            s = b["framer"].stats()
            self.put("frames", f"{s['data']} data, {s['dummy']} dummy")
        if b.get("imp") is not None:
            s = b["imp"].stats()
            self.put("imp", s["text"], RED if not s["carrier"] else None)
            self.put("dop", s.get("doppler") or "off", None if s.get("doppler") else GREY)
        if b.get("sink") is not None:
            self.put("out", b["sink"].describe())


# ---------------------------------------------------------------------------- switch
class PanelSwitch(Qt.QWidget):
    """Checkboxes that switch panel groups on and off at run time.

    groups: {name: [GRC ids]}. For each id: <id>.set_enabled(on) if it has it (valves),
    <id>.set_active(on) (g5gs panels), and the widget self._<id>_win (Qt sinks) is shown /
    hidden.
    """

    def __init__(self, top, groups, panels="all", main_row=2):
        super().__init__()
        self.top, self.groups, self.main_row = top, groups, main_row
        lay = Qt.QHBoxLayout(self)
        lay.setContentsMargins(4, 0, 4, 0)
        lay.addWidget(Qt.QLabel("Panels:"))
        on = C.panels_set(panels)
        self.boxes = {}
        for name in groups:
            cb = Qt.QCheckBox(name)
            cb.setChecked(name in on)
            cb.toggled.connect(lambda v, n=name: self.apply(n, v))
            lay.addWidget(cb)
            self.boxes[name] = cb
        lay.addStretch(1)
        # apply once all blocks exist (they are created after this widget)
        Qt.QTimer.singleShot(0, lambda: self._first(on))

    def _first(self, on):
        # GRC gives every grid row stretch 1: keep the switch and control rows compact
        lay = getattr(self.top, "top_grid_layout", None)
        if lay is not None:
            for r in range(lay.rowCount()):
                lay.setRowStretch(r, 1 if r >= self.main_row else 0)
        for n in self.groups:
            self.apply(n, n in on)

    def apply(self, name, on):
        for gid in self.groups.get(name, []):
            obj = getattr(self.top, gid, None)
            if obj is None:
                continue
            if hasattr(obj, "set_enabled"):
                obj.set_enabled(bool(on))
            if hasattr(obj, "set_active"):
                obj.set_active(bool(on))
            for w in (getattr(self.top, f"_{gid}_win", None),
                      obj if isinstance(obj, Qt.QWidget) else None):
                if w is not None:
                    w.setVisible(bool(on))


def apply_panels(top, groups, panels):
    """Headless equivalent of PanelSwitch (no Qt event loop needed): valves + panels."""
    on = C.panels_set(panels)
    for name, ids in groups.items():
        for gid in ids:
            obj = getattr(top, gid, None)
            if obj is None:
                continue
            if hasattr(obj, "set_enabled"):
                obj.set_enabled(name in on)
            if hasattr(obj, "set_active"):
                obj.set_active(name in on)
