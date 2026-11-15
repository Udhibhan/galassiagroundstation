#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: G5 unified TX
# Author: Galassia-5 ground station
# Description: Galassia-5 test transmitter: Link X/SX-style GSSE/GSUFTP (or gr-dvbgse GSE from tap0, or a BBFRAME file) -> DVB-S2 (gr-dtv) + dummy frames -> transients / CFO / TLE Doppler / AWGN -> B205mini or IQ file. README.md.
# GNU Radio version: 3.10.12.0

from PyQt5 import Qt
from gnuradio import qtgui
from PyQt5 import QtCore
from gnuradio import blocks
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
import g5gs
import sip
import threading



class tx_unified(gr.top_block, Qt.QWidget):

    def __init__(self, preset='ops20', sym_rate=0, sps=0, modcod='', frame_size='', panels='all', payload='gsse', files='', stack='rev1.0', chunk_size=0, repeat=0, flush=100, bbframe_in='', lead_dummies=200, gap_dummies=0, source='generate', replay_file='', output='b205', output_file='', duration=0, realtime=0, centre_freq=1195e6, tx_gain=0, device_args='', level_db=0, rms_dbfs=(-12), cfo_hz=0, esn0_db='', schedule='', tle='', sat='', doppler_start_utc='', doppler_dt=0, station='1.2977,103.7764,60', rf_freq=13.95e9):
        gr.top_block.__init__(self, "G5 unified TX", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("G5 unified TX")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except BaseException as exc:
            print(f"Qt GUI: Could not set Icon: {str(exc)}", file=sys.stderr)
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "tx_unified")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Parameters
        ##################################################
        self.preset = preset
        self.sym_rate = sym_rate
        self.sps = sps
        self.modcod = modcod
        self.frame_size = frame_size
        self.panels = panels
        self.payload = payload
        self.files = files
        self.stack = stack
        self.chunk_size = chunk_size
        self.repeat = repeat
        self.flush = flush
        self.bbframe_in = bbframe_in
        self.lead_dummies = lead_dummies
        self.gap_dummies = gap_dummies
        self.source = source
        self.replay_file = replay_file
        self.output = output
        self.output_file = output_file
        self.duration = duration
        self.realtime = realtime
        self.centre_freq = centre_freq
        self.tx_gain = tx_gain
        self.device_args = device_args
        self.level_db = level_db
        self.rms_dbfs = rms_dbfs
        self.cfo_hz = cfo_hz
        self.esn0_db = esn0_db
        self.schedule = schedule
        self.tle = tle
        self.sat = sat
        self.doppler_start_utc = doppler_start_utc
        self.doppler_dt = doppler_dt
        self.station = station
        self.rf_freq = rf_freq

        ##################################################
        # Variables
        ##################################################
        self.cfg = cfg = g5gs.config.resolve(preset, sym_rate, sps, modcod, frame_size)
        self.tx_ids = tx_ids = {'payload': 'tx_payload_0', 'framer': 'dvbs2_tx_0', 'imp': 'tx_impairments_0', 'sink': 'tx_sink_0'}
        self.tx_gain_slider = tx_gain_slider = tx_gain
        self.sig_power = sig_power = g5gs.tx.signal_power(source, replay_file, cfg.sps)
        self.samp_rate = samp_rate = cfg.samp_rate
        self.rs = rs = cfg.sym_rate
        self.panel_groups = panel_groups = {'spectrum': ['valve_spec', 'qtgui_freq_sink_tx'], 'status': ['tx_status_panel_0']}
        self.level_slider = level_slider = level_db
        self.dop_start = dop_start = g5gs.tx.doppler_start(tle, doppler_start_utc)
        self.cfo_slider = cfo_slider = cfo_hz
        self.cfg_text = cfg_text = f'{cfg.describe()}, payload {payload}' + (f' {files}' if files else '') + f', lead {lead_dummies} / gap {gap_dummies} dummies' + (f', TLE {tle}' if tle else '')
        self.carrier_on = carrier_on = 1

        ##################################################
        # Blocks
        ##################################################

        self._tx_gain_slider_range = qtgui.Range(0, 89, 1, tx_gain, 200)
        self._tx_gain_slider_win = qtgui.RangeWidget(self._tx_gain_slider_range, self.set_tx_gain_slider, "TX gain (dB, B205mini)", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._tx_gain_slider_win, 1, 0, 1, 1)
        for r in range(1, 2):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._level_slider_range = qtgui.Range(-60, 10, 1, level_db, 200)
        self._level_slider_win = qtgui.RangeWidget(self._level_slider_range, self.set_level_slider, "Level (dB, digital: fades / steps)", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._level_slider_win, 1, 1, 1, 1)
        for r in range(1, 2):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._cfo_slider_range = qtgui.Range(-0.2 * rs, 0.2 * rs, 100, cfo_hz, 200)
        self._cfo_slider_win = qtgui.RangeWidget(self._cfo_slider_range, self.set_cfo_slider, "CFO (Hz)", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._cfo_slider_win, 2, 0, 1, 1)
        for r in range(2, 3):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        _carrier_on_check_box = Qt.QCheckBox("Carrier on")
        self._carrier_on_choices = {True: 1, False: 0}
        self._carrier_on_choices_inv = dict((v,k) for k,v in self._carrier_on_choices.items())
        self._carrier_on_callback = lambda i: Qt.QMetaObject.invokeMethod(_carrier_on_check_box, "setChecked", Qt.Q_ARG("bool", self._carrier_on_choices_inv[i]))
        self._carrier_on_callback(self.carrier_on)
        _carrier_on_check_box.stateChanged.connect(lambda i: self.set_carrier_on(self._carrier_on_choices[bool(i)]))
        self.top_grid_layout.addWidget(_carrier_on_check_box, 2, 1, 1, 1)
        for r in range(2, 3):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.valve_spec = blocks.copy(gr.sizeof_gr_complex*1)
        self.valve_spec.set_enabled(True)
        self.tx_status_panel_0 = g5gs.TxStatusPanel(self, ids=tx_ids, cfg_text=cfg_text, active=True)
        self._tx_status_panel_0_win = self.tx_status_panel_0
        self.top_grid_layout.addWidget(self._tx_status_panel_0_win, 3, 0, 1, 1)
        for r in range(3, 4):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.tx_sink_0 = g5gs.tx_sink(mode=output, samp_rate=samp_rate, sym_rate=rs, rolloff=0.2, centre_freq=centre_freq, tx_gain=tx_gain_slider, device_args=device_args, lo_offset=(-1), output_file=output_file, output_format='auto', duration=duration, throttle=bool(realtime) or payload == 'gse_tap0', start_unix=dop_start, meta={'preset': preset, 'modcod': cfg.modcod, 'sym_rate': rs, 'sps': cfg.sps, 'payload': payload, 'files': files, 'esn0_db': esn0_db, 'cfo_hz': cfo_hz, 'tle': tle})
        self.tx_payload_0 = g5gs.tx_payload(mode=payload, files=files, stack=stack, chunk_size=chunk_size, repeat=repeat, flush=flush, bbframe_file=bbframe_in, frame_len=cfg.kbch_bytes, modcod=cfg.modcod, frame_size=cfg.frame_size, isi=0, sis=False, enabled=source == 'generate', gse_src_ip='44.0.0.3', gse_dst_ip='10.0.1.190')
        self.tx_impairments_0 = g5gs.impairments(samp_rate=samp_rate, signal_power=sig_power, rms_dbfs=rms_dbfs, level_db=level_slider, cfo_hz=cfo_slider, esn0_db=esn0_db, sps=cfg.sps, schedule=schedule, tle=tle, sat=sat, station=station, rf_freq=rf_freq, doppler_start_utc=dop_start if dop_start is not None else '', doppler_dt=doppler_dt, seed=0, carrier=bool(carrier_on))
        self.qtgui_freq_sink_tx = qtgui.freq_sink_c(
            1024, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            centre_freq, #fc
            samp_rate, #bw
            "TX spectrum", #name
            1,
            None # parent
        )
        self.qtgui_freq_sink_tx.set_update_time(0.10)
        self.qtgui_freq_sink_tx.set_y_axis((-140), 10)
        self.qtgui_freq_sink_tx.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_tx.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_tx.enable_autoscale(False)
        self.qtgui_freq_sink_tx.enable_grid(True)
        self.qtgui_freq_sink_tx.set_fft_average(0.2)
        self.qtgui_freq_sink_tx.enable_axis_labels(True)
        self.qtgui_freq_sink_tx.enable_control_panel(False)
        self.qtgui_freq_sink_tx.set_fft_window_normalized(False)

        self.qtgui_freq_sink_tx.disable_legend()


        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "blue", "blue", "blue", "blue",
            "blue", "blue", "blue", "blue", "blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_tx.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_tx.set_line_label(i, labels[i])
            self.qtgui_freq_sink_tx.set_line_width(i, widths[i])
            self.qtgui_freq_sink_tx.set_line_color(i, colors[i])
            self.qtgui_freq_sink_tx.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_tx_win = sip.wrapinstance(self.qtgui_freq_sink_tx.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qtgui_freq_sink_tx_win, 3, 1, 1, 1)
        for r in range(3, 4):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.panel_switch_0 = g5gs.PanelSwitch(self, groups=panel_groups, panels=panels, main_row=3)
        self._panel_switch_0_win = self.panel_switch_0
        self.top_grid_layout.addWidget(self._panel_switch_0_win, 0, 0, 1, 2)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.iq_select_0 = g5gs.iq_select(mode=source, replay_file=replay_file, replay_format='auto', replay_loop=True)
        self.dvbs2_tx_0 = g5gs.dvbs2_tx(modcod=cfg.modcod, frame_size=cfg.frame_size, sps=cfg.sps, rolloff=0.2, lead=lead_dummies, gap=gap_dummies, gold_code=0)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.dvbs2_tx_0, 0), (self.iq_select_0, 0))
        self.connect((self.iq_select_0, 0), (self.tx_impairments_0, 0))
        self.connect((self.tx_impairments_0, 0), (self.tx_sink_0, 0))
        self.connect((self.tx_impairments_0, 0), (self.valve_spec, 0))
        self.connect((self.tx_payload_0, 0), (self.dvbs2_tx_0, 0))
        self.connect((self.valve_spec, 0), (self.qtgui_freq_sink_tx, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "tx_unified")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_preset(self):
        return self.preset

    def set_preset(self, preset):
        self.preset = preset
        self.set_cfg(g5gs.config.resolve(self.preset, self.sym_rate, self.sps, self.modcod, self.frame_size))

    def get_sym_rate(self):
        return self.sym_rate

    def set_sym_rate(self, sym_rate):
        self.sym_rate = sym_rate
        self.set_cfg(g5gs.config.resolve(self.preset, self.sym_rate, self.sps, self.modcod, self.frame_size))

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.set_cfg(g5gs.config.resolve(self.preset, self.sym_rate, self.sps, self.modcod, self.frame_size))

    def get_modcod(self):
        return self.modcod

    def set_modcod(self, modcod):
        self.modcod = modcod
        self.set_cfg(g5gs.config.resolve(self.preset, self.sym_rate, self.sps, self.modcod, self.frame_size))

    def get_frame_size(self):
        return self.frame_size

    def set_frame_size(self, frame_size):
        self.frame_size = frame_size
        self.set_cfg(g5gs.config.resolve(self.preset, self.sym_rate, self.sps, self.modcod, self.frame_size))

    def get_panels(self):
        return self.panels

    def set_panels(self, panels):
        self.panels = panels

    def get_payload(self):
        return self.payload

    def set_payload(self, payload):
        self.payload = payload

    def get_files(self):
        return self.files

    def set_files(self, files):
        self.files = files
        self.set_cfg_text(f'{cfg.describe()}, payload {payload}' + (f' {files}' if self.files else '') + f', lead {lead_dummies} / gap {gap_dummies} dummies' + (f', TLE {tle}' if self.tle else ''))

    def get_stack(self):
        return self.stack

    def set_stack(self, stack):
        self.stack = stack

    def get_chunk_size(self):
        return self.chunk_size

    def set_chunk_size(self, chunk_size):
        self.chunk_size = chunk_size

    def get_repeat(self):
        return self.repeat

    def set_repeat(self, repeat):
        self.repeat = repeat

    def get_flush(self):
        return self.flush

    def set_flush(self, flush):
        self.flush = flush

    def get_bbframe_in(self):
        return self.bbframe_in

    def set_bbframe_in(self, bbframe_in):
        self.bbframe_in = bbframe_in

    def get_lead_dummies(self):
        return self.lead_dummies

    def set_lead_dummies(self, lead_dummies):
        self.lead_dummies = lead_dummies

    def get_gap_dummies(self):
        return self.gap_dummies

    def set_gap_dummies(self, gap_dummies):
        self.gap_dummies = gap_dummies

    def get_source(self):
        return self.source

    def set_source(self, source):
        self.source = source
        self.set_sig_power(g5gs.tx.signal_power(self.source, self.replay_file, cfg.sps))

    def get_replay_file(self):
        return self.replay_file

    def set_replay_file(self, replay_file):
        self.replay_file = replay_file
        self.set_sig_power(g5gs.tx.signal_power(self.source, self.replay_file, cfg.sps))

    def get_output(self):
        return self.output

    def set_output(self, output):
        self.output = output

    def get_output_file(self):
        return self.output_file

    def set_output_file(self, output_file):
        self.output_file = output_file

    def get_duration(self):
        return self.duration

    def set_duration(self, duration):
        self.duration = duration

    def get_realtime(self):
        return self.realtime

    def set_realtime(self, realtime):
        self.realtime = realtime

    def get_centre_freq(self):
        return self.centre_freq

    def set_centre_freq(self, centre_freq):
        self.centre_freq = centre_freq
        self.qtgui_freq_sink_tx.set_frequency_range(self.centre_freq, self.samp_rate)

    def get_tx_gain(self):
        return self.tx_gain

    def set_tx_gain(self, tx_gain):
        self.tx_gain = tx_gain
        self.set_tx_gain_slider(self.tx_gain)

    def get_device_args(self):
        return self.device_args

    def set_device_args(self, device_args):
        self.device_args = device_args

    def get_level_db(self):
        return self.level_db

    def set_level_db(self, level_db):
        self.level_db = level_db
        self.set_level_slider(self.level_db)

    def get_rms_dbfs(self):
        return self.rms_dbfs

    def set_rms_dbfs(self, rms_dbfs):
        self.rms_dbfs = rms_dbfs

    def get_cfo_hz(self):
        return self.cfo_hz

    def set_cfo_hz(self, cfo_hz):
        self.cfo_hz = cfo_hz
        self.set_cfo_slider(self.cfo_hz)

    def get_esn0_db(self):
        return self.esn0_db

    def set_esn0_db(self, esn0_db):
        self.esn0_db = esn0_db

    def get_schedule(self):
        return self.schedule

    def set_schedule(self, schedule):
        self.schedule = schedule

    def get_tle(self):
        return self.tle

    def set_tle(self, tle):
        self.tle = tle
        self.set_cfg_text(f'{cfg.describe()}, payload {payload}' + (f' {files}' if self.files else '') + f', lead {lead_dummies} / gap {gap_dummies} dummies' + (f', TLE {tle}' if self.tle else ''))
        self.set_dop_start(g5gs.tx.doppler_start(self.tle, self.doppler_start_utc))

    def get_sat(self):
        return self.sat

    def set_sat(self, sat):
        self.sat = sat

    def get_doppler_start_utc(self):
        return self.doppler_start_utc

    def set_doppler_start_utc(self, doppler_start_utc):
        self.doppler_start_utc = doppler_start_utc
        self.set_dop_start(g5gs.tx.doppler_start(self.tle, self.doppler_start_utc))

    def get_doppler_dt(self):
        return self.doppler_dt

    def set_doppler_dt(self, doppler_dt):
        self.doppler_dt = doppler_dt

    def get_station(self):
        return self.station

    def set_station(self, station):
        self.station = station

    def get_rf_freq(self):
        return self.rf_freq

    def set_rf_freq(self, rf_freq):
        self.rf_freq = rf_freq

    def get_cfg(self):
        return self.cfg

    def set_cfg(self, cfg):
        self.cfg = cfg

    def get_tx_ids(self):
        return self.tx_ids

    def set_tx_ids(self, tx_ids):
        self.tx_ids = tx_ids

    def get_tx_gain_slider(self):
        return self.tx_gain_slider

    def set_tx_gain_slider(self, tx_gain_slider):
        self.tx_gain_slider = tx_gain_slider
        self.tx_sink_0.set_tx_gain(self.tx_gain_slider)

    def get_sig_power(self):
        return self.sig_power

    def set_sig_power(self, sig_power):
        self.sig_power = sig_power

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.qtgui_freq_sink_tx.set_frequency_range(self.centre_freq, self.samp_rate)

    def get_rs(self):
        return self.rs

    def set_rs(self, rs):
        self.rs = rs

    def get_panel_groups(self):
        return self.panel_groups

    def set_panel_groups(self, panel_groups):
        self.panel_groups = panel_groups

    def get_level_slider(self):
        return self.level_slider

    def set_level_slider(self, level_slider):
        self.level_slider = level_slider
        self.tx_impairments_0.set_level_db(self.level_slider)

    def get_dop_start(self):
        return self.dop_start

    def set_dop_start(self, dop_start):
        self.dop_start = dop_start

    def get_cfo_slider(self):
        return self.cfo_slider

    def set_cfo_slider(self, cfo_slider):
        self.cfo_slider = cfo_slider
        self.tx_impairments_0.set_cfo(self.cfo_slider)

    def get_cfg_text(self):
        return self.cfg_text

    def set_cfg_text(self, cfg_text):
        self.cfg_text = cfg_text

    def get_carrier_on(self):
        return self.carrier_on

    def set_carrier_on(self, carrier_on):
        self.carrier_on = carrier_on
        self._carrier_on_callback(self.carrier_on)
        self.tx_impairments_0.set_carrier(bool(self.carrier_on))



def argument_parser():
    description = 'Galassia-5 test transmitter: Link X/SX-style GSSE/GSUFTP (or gr-dvbgse GSE from tap0, or a BBFRAME file) -> DVB-S2 (gr-dtv) + dummy frames -> transients / CFO / TLE Doppler / AWGN -> B205mini or IQ file. README.md.'
    parser = ArgumentParser(description=description)
    parser.add_argument(
        "--preset", dest="preset", type=str, default='ops20',
        help="Set Preset (ops20 / linkx2 / field1) [default=%(default)r]")
    parser.add_argument(
        "--sym-rate", dest="sym_rate", type=eng_float, default=eng_notation.num_to_str(float(0)),
        help="Set Symbol rate (0 = preset) [default=%(default)r]")
    parser.add_argument(
        "--sps", dest="sps", type=intx, default=0,
        help="Set Samples/symbol (0 = preset) [default=%(default)r]")
    parser.add_argument(
        "--modcod", dest="modcod", type=str, default='',
        help="Set MODCOD (empty = preset) [default=%(default)r]")
    parser.add_argument(
        "--frame-size", dest="frame_size", type=str, default='',
        help="Set Frame size (empty = preset) [default=%(default)r]")
    parser.add_argument(
        "--panels", dest="panels", type=str, default='all',
        help="Set Panels at start (all / none / list) [default=%(default)r]")
    parser.add_argument(
        "--payload", dest="payload", type=str, default='gsse',
        help="Set Payload (gsse / bbframes / gse_tap0) [default=%(default)r]")
    parser.add_argument(
        "--files", dest="files", type=str, default='',
        help="Set File or folder to send [default=%(default)r]")
    parser.add_argument(
        "--stack", dest="stack", type=str, default='rev1.0',
        help="Set Stack (rev1.0 / rev1.2 / udpraw) [default=%(default)r]")
    parser.add_argument(
        "--chunk-size", dest="chunk_size", type=intx, default=0,
        help="Set GSUFTP chunk (0 = default) [default=%(default)r]")
    parser.add_argument(
        "--repeat", dest="repeat", type=intx, default=0,
        help="Set Passes (0 = loop forever) [default=%(default)r]")
    parser.add_argument(
        "--flush", dest="flush", type=intx, default=100,
        help="Set Empty BBFRAMEs at the end [default=%(default)r]")
    parser.add_argument(
        "--bbframe-in", dest="bbframe_in", type=str, default='',
        help="Set BBFRAME file (bbframes payload) [default=%(default)r]")
    parser.add_argument(
        "--lead-dummies", dest="lead_dummies", type=intx, default=200,
        help="Set Dummy PLFRAMEs at the start [default=%(default)r]")
    parser.add_argument(
        "--gap-dummies", dest="gap_dummies", type=intx, default=0,
        help="Set Dummy PLFRAMEs after each frame [default=%(default)r]")
    parser.add_argument(
        "--source", dest="source", type=str, default='generate',
        help="Set Source (generate / replay) [default=%(default)r]")
    parser.add_argument(
        "--replay-file", dest="replay_file", type=str, default='',
        help="Set Replay IQ file [default=%(default)r]")
    parser.add_argument(
        "--output", dest="output", type=str, default='b205',
        help="Set Output (b205 / file) [default=%(default)r]")
    parser.add_argument(
        "--output-file", dest="output_file", type=str, default='',
        help="Set Output IQ file [default=%(default)r]")
    parser.add_argument(
        "--duration", dest="duration", type=eng_float, default=eng_notation.num_to_str(float(0)),
        help="Set File: seconds (0 = payload end) [default=%(default)r]")
    parser.add_argument(
        "--realtime", dest="realtime", type=intx, default=0,
        help="Set File: real time (1 = yes) [default=%(default)r]")
    parser.add_argument(
        "--centre-freq", dest="centre_freq", type=eng_float, default=eng_notation.num_to_str(float(1195e6)),
        help="Set Centre frequency [default=%(default)r]")
    parser.add_argument(
        "--tx-gain", dest="tx_gain", type=eng_float, default=eng_notation.num_to_str(float(0)),
        help="Set TX gain (dB) [default=%(default)r]")
    parser.add_argument(
        "--device-args", dest="device_args", type=str, default='',
        help="Set UHD device args [default=%(default)r]")
    parser.add_argument(
        "--level-db", dest="level_db", type=eng_float, default=eng_notation.num_to_str(float(0)),
        help="Set Level (dB) [default=%(default)r]")
    parser.add_argument(
        "--rms-dbfs", dest="rms_dbfs", type=eng_float, default=eng_notation.num_to_str(float((-12))),
        help="Set Output RMS at 0 dB (dBFS) [default=%(default)r]")
    parser.add_argument(
        "--cfo-hz", dest="cfo_hz", type=eng_float, default=eng_notation.num_to_str(float(0)),
        help="Set CFO (Hz) [default=%(default)r]")
    parser.add_argument(
        "--esn0-db", dest="esn0_db", type=str, default='',
        help="Set Es/N0 dB (empty = no noise) [default=%(default)r]")
    parser.add_argument(
        "--schedule", dest="schedule", type=str, default='',
        help="Set Transients (5:off,6:on,10:-20dB) [default=%(default)r]")
    parser.add_argument(
        "--tle", dest="tle", type=str, default='',
        help="Set Doppler TLE (empty = off) [default=%(default)r]")
    parser.add_argument(
        "--sat", dest="sat", type=str, default='',
        help="Set Satellite (if several sets) [default=%(default)r]")
    parser.add_argument(
        "--doppler-start-utc", dest="doppler_start_utc", type=str, default='',
        help="Set Doppler start UTC (empty = now) [default=%(default)r]")
    parser.add_argument(
        "--doppler-dt", dest="doppler_dt", type=eng_float, default=eng_notation.num_to_str(float(0)),
        help="Set Doppler time shift (s) [default=%(default)r]")
    parser.add_argument(
        "--station", dest="station", type=str, default='1.2977,103.7764,60',
        help="Set Station lat,lon,h [default=%(default)r]")
    parser.add_argument(
        "--rf-freq", dest="rf_freq", type=eng_float, default=eng_notation.num_to_str(float(13.95e9)),
        help="Set Downlink RF (Hz) [default=%(default)r]")
    return parser


def main(top_block_cls=tx_unified, options=None):
    if options is None:
        options = argument_parser().parse_args()

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls(preset=options.preset, sym_rate=options.sym_rate, sps=options.sps, modcod=options.modcod, frame_size=options.frame_size, panels=options.panels, payload=options.payload, files=options.files, stack=options.stack, chunk_size=options.chunk_size, repeat=options.repeat, flush=options.flush, bbframe_in=options.bbframe_in, lead_dummies=options.lead_dummies, gap_dummies=options.gap_dummies, source=options.source, replay_file=options.replay_file, output=options.output, output_file=options.output_file, duration=options.duration, realtime=options.realtime, centre_freq=options.centre_freq, tx_gain=options.tx_gain, device_args=options.device_args, level_db=options.level_db, rms_dbfs=options.rms_dbfs, cfo_hz=options.cfo_hz, esn0_db=options.esn0_db, schedule=options.schedule, tle=options.tle, sat=options.sat, doppler_start_utc=options.doppler_start_utc, doppler_dt=options.doppler_dt, station=options.station, rf_freq=options.rf_freq)

    tb.start()
    tb.flowgraph_started.set()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
