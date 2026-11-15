#!/usr/bin/env python3
"""Launcher for the unified RX / TX GUIs: pick a preset and the usual settings, press Start.

It runs rx_unified.py / tx_unified.py (the GRC-generated GUIs) with the matching
command-line options, and shows that command so it can go into a test record or be run
headless with rx_cli.py / tx_cli.py. Settings are remembered between runs.
"""
import os
import shlex
import subprocess
import sys

from PyQt5 import Qt, QtCore

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from g5gs import config as C  # noqa: E402

MODCOD_CHOICES = [("", "preset")] + [(str(k), f"{k}: {v}") for k, v in C.MODCODS.items()]


class PathField(Qt.QWidget):
    def __init__(self, kind="open", filt="All files (*)"):
        super().__init__()
        lay = Qt.QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.edit = Qt.QLineEdit()
        b = Qt.QPushButton("...")
        b.setFixedWidth(32)
        b.clicked.connect(self.browse)
        lay.addWidget(self.edit)
        lay.addWidget(b)
        self.kind, self.filt = kind, filt

    def browse(self):
        start = os.path.dirname(self.text()) or os.path.expanduser("~")
        if self.kind == "dir":
            p = Qt.QFileDialog.getExistingDirectory(self, "Choose a folder", start)
        elif self.kind == "save":
            p, _ = Qt.QFileDialog.getSaveFileName(self, "Output file", start, self.filt)
        else:
            p, _ = Qt.QFileDialog.getOpenFileName(self, "Choose a file", start, self.filt)
        if p:
            self.edit.setText(p)

    def text(self):
        return self.edit.text().strip()

    def setText(self, t):
        self.edit.setText(t)


class Form(Qt.QWidget):
    """A form whose fields map to command-line options; remembers values in QSettings."""

    def __init__(self, key):
        super().__init__()
        self.key = key
        self.lay = Qt.QFormLayout(self)
        self.fields = {}            # option -> (widget, kind, default)
        self.rows = {}

    def add(self, opt, label, widget, default=""):
        self.lay.addRow(label, widget)
        self.rows[opt] = widget
        self.fields[opt] = (widget, default)
        return widget

    def combo(self, opt, label, items, default=None):
        w = Qt.QComboBox()
        for val, text in items:
            w.addItem(text, val)
        if default is not None:
            w.setCurrentIndex(max(0, w.findData(default)))
        return self.add(opt, label, w, default)

    def line(self, opt, label, default="", tip=""):
        w = Qt.QLineEdit(str(default))
        w.setPlaceholderText(tip)
        return self.add(opt, label, w, default)

    def path(self, opt, label, kind="open", filt="All files (*)"):
        return self.add(opt, label, PathField(kind, filt), "")

    def check(self, opt, label, default=False):
        w = Qt.QCheckBox()
        w.setChecked(default)
        return self.add(opt, label, w, default)

    def value(self, opt):
        w = self.fields[opt][0]
        if isinstance(w, Qt.QComboBox):
            return w.currentData()
        if isinstance(w, Qt.QCheckBox):
            return "1" if w.isChecked() else "0"
        return w.text().strip()

    def args(self, skip=()):
        out = []
        for opt, (w, default) in self.fields.items():
            if opt.startswith("_") or opt in skip:
                continue
            v = self.value(opt)
            if isinstance(w, Qt.QCheckBox):
                if v != ("1" if default else "0"):
                    out += [f"--{opt}", v]
            elif v != "" and str(v) != str(default):
                out += [f"--{opt}", v]
        return out

    def save(self, s):
        for opt, (w, _) in self.fields.items():
            s.setValue(f"{self.key}/{opt}", self.value(opt))

    def load(self, s):
        for opt, (w, _) in self.fields.items():
            v = s.value(f"{self.key}/{opt}")
            if v is None:
                continue
            if isinstance(w, Qt.QComboBox):
                i = w.findData(v)
                if i >= 0:
                    w.setCurrentIndex(i)
            elif isinstance(w, Qt.QCheckBox):
                w.setChecked(v == "1")
            else:
                w.setText(v)


def rx_form():
    f = Form("rx")
    names, labels = C.preset_choices()
    f.combo("preset", "Preset", list(zip(names, labels)), C.DEFAULT_PRESET)
    f.combo("mode", "Mode", [("record", "Record + monitor (B205mini)"),
                             ("live", "Live decode (B205mini, <= ~20 MS/s)"),
                             ("file", "Decode a recording / IQ file")], "record")
    f.path("input-file", "Input file (file mode)", "open",
           "IQ (*.sc16 *.cs16 *.iq *.fc32 *.cfile);;All files (*)")
    f.path("rec-dir", "Recording folder", "dir")
    f.path("tle", "TLE / OMM (optional)", "open", "TLE (*.tle *.txt *.json *.csv);;All files (*)")
    f.line("sat", "Satellite in TLE file", "", "only if the file holds several")
    f.path("verify-hashes", "Expected sha256sum (optional)", "open")
    f.path("out-dir", "Decoded files folder", "dir")
    f.combo("modcod", "MODCOD", MODCOD_CHOICES, "")
    f.line("sym-rate", "Symbol rate (Bd)", "", "preset")
    f.line("centre-freq", "Centre frequency (Hz)", "1195e6")
    f.line("rx-gain", "RX gain (dB)", "30")
    f.combo("antenna", "RX port", [(a, a) for a in C.RX_ANTENNAS], "RX2")
    f.check("conj", "Conjugate (inverted IF)", False)
    f.line("panels", "Panels at start", "all", "all / none / spectrum,status,...")
    f.path("sim-input", "Rehearsal sc16 (no radio)", "open")
    f.line("_extra", "Extra options", "", "e.g. --agc-ref 1.2 --duty-period 3")
    return f


def tx_form():
    f = Form("tx")
    names, labels = C.preset_choices()
    f.combo("preset", "Preset", list(zip(names, labels)), C.DEFAULT_PRESET)
    f.combo("payload", "Payload", [("gsse", "Files as Link X/SX GSSE (+GSUFTP)"),
                                   ("bbframes", "BBFRAME file"),
                                   ("gse_tap0", "Standard GSE from tap0 (gr-dvbgse)")], "gsse")
    f.path("files", "File or folder to send", "dir")
    f.combo("stack", "Stack", [("rev1.0", "rev 1.0: GSUFTP in GSSE (Link X)"),
                               ("rev1.2", "rev 1.2: GSUFTP/UDP/IPv4 in GSSE (SX)"),
                               ("udpraw", "plain UDP (Branch C1)")], "rev1.0")
    f.line("repeat", "Passes (0 = loop)", "0")
    f.line("gap-dummies", "Dummy frames after each frame", "0")
    f.combo("output", "Output", [("b205", "B205mini"), ("file", "IQ file")], "b205")
    f.path("output-file", "Output file", "save", "sc16 (*.sc16);;fc32 (*.fc32 *.iq)")
    f.line("duration", "File length (s, 0 = payload end)", "0")
    f.line("tx-gain", "TX gain (dB)", "0")
    f.line("centre-freq", "Centre frequency (Hz)", "1195e6")
    f.line("cfo-hz", "CFO (Hz)", "0")
    f.line("esn0-db", "Es/N0 (dB, empty = no noise)", "")
    f.line("schedule", "Transients", "", "5:off,6:on,10:-20dB,12:0dB")
    f.path("tle", "Doppler TLE (optional)", "open", "TLE (*.tle *.txt *.json *.csv);;All files (*)")
    f.line("doppler-start-utc", "Doppler start UTC", "", "empty = now")
    f.combo("source", "Source", [("generate", "Generate"), ("replay", "Replay an IQ file")],
            "generate")
    f.path("replay-file", "Replay file", "open")
    f.line("panels", "Panels at start", "all")
    f.line("_extra", "Extra options", "", "e.g. --lead-dummies 500 --rms-dbfs -15")
    return f


class Launcher(Qt.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("G5 ground station launcher")
        self.settings = Qt.QSettings("galassia5", "g5_launch")
        lay = Qt.QVBoxLayout(self)
        self.tabs = Qt.QTabWidget()
        self.rx, self.tx = rx_form(), tx_form()
        for form, name in ((self.rx, "Receiver"), (self.tx, "Transmitter")):
            sc = Qt.QScrollArea()
            sc.setWidget(form)
            sc.setWidgetResizable(True)
            self.tabs.addTab(sc, name)
            form.load(self.settings)
        lay.addWidget(self.tabs)
        self.cmd = Qt.QPlainTextEdit()
        self.cmd.setReadOnly(True)
        self.cmd.setMaximumHeight(90)
        lay.addWidget(self.cmd)
        row = Qt.QHBoxLayout()
        for text, fn in (("Start GUI", self.start_gui), ("Start headless (CLI)", self.start_cli),
                         ("Copy command", self.copy)):
            b = Qt.QPushButton(text)
            b.clicked.connect(fn)
            row.addWidget(b)
        lay.addLayout(row)
        self.tabs.currentChanged.connect(self.refresh)
        for form in (self.rx, self.tx):
            for w, _ in form.fields.values():
                for sig in ("currentIndexChanged", "textChanged", "toggled"):
                    if hasattr(w, sig):
                        getattr(w, sig).connect(self.refresh)
                if isinstance(w, PathField):
                    w.edit.textChanged.connect(self.refresh)
        self.resize(720, 820)
        self.refresh()

    def form(self):
        return self.rx if self.tabs.currentIndex() == 0 else self.tx

    def argv(self, cli=False):
        f = self.form()
        rx = f is self.rx
        script = ("rx_cli.py" if cli else "rx_unified.py") if rx else \
            ("tx_cli.py" if cli else "tx_unified.py")
        skip = []
        if rx:
            mode = f.value("mode")
            skip += ["input-file"] if mode != "file" else ["rec-dir", "sim-input", "rx-gain",
                                                           "centre-freq", "antenna"]
        else:
            if f.value("output") != "file":
                skip += ["output-file", "duration"]
            if f.value("source") != "replay":
                skip += ["replay-file"]
        a = f.args(skip)
        if cli:
            a = [x for i, x in enumerate(a) if not (x == "--panels" or (i and a[i - 1] == "--panels"))]
        extra = shlex.split(f.value("_extra")) if f.value("_extra") else []
        return [sys.executable, os.path.join(HERE, script)] + a + extra

    def refresh(self, *_):
        self.cmd.setPlainText(" ".join(shlex.quote(x) for x in self.argv()))

    def _run(self, cli):
        for form in (self.rx, self.tx):
            form.save(self.settings)
        argv = self.argv(cli)
        if cli:
            term = next((t for t in ("gnome-terminal", "x-terminal-emulator", "xterm")
                         if subprocess.call(["which", t], stdout=subprocess.DEVNULL) == 0), None)
            if term == "gnome-terminal":
                argv = [term, "--", "bash", "-c", " ".join(map(shlex.quote, argv)) + "; exec bash"]
            elif term:
                argv = [term, "-e", "bash", "-c", " ".join(map(shlex.quote, argv)) + "; exec bash"]
        subprocess.Popen(argv, cwd=HERE)

    def start_gui(self):
        self._run(False)

    def start_cli(self):
        self._run(True)

    def copy(self):
        Qt.QApplication.clipboard().setText(self.cmd.toPlainText())


def main():
    app = Qt.QApplication(sys.argv)
    w = Launcher()
    w.show()
    app.exec_()


if __name__ == "__main__":
    main()
