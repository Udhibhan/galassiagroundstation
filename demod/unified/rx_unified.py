#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: G5 unified RX
# Author: Galassia-5 ground station
# Description: Galassia-5 ground-station receiver: B205mini record + monitor / live decode / file decode, patched gr-dvbs2rx, optional TLE Doppler pre-comp, GSSE/GSUFTP files. Parameters = command-line options (rx_unified.py --help). README.md.
# GNU Radio version: 3.10.12.0

from PyQt5 import Qt
from gnuradio import qtgui
from PyQt5 import QtCore
from PyQt5.QtCore import QObject, pyqtSlot
from gnuradio import blocks
from gnuradio import dvbs2rx
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



class rx_unified(gr.top_block, Qt.QWidget):

    def __init__(self, agc='agc3', agc3_attack=(1e-2), agc3_decay=(1e-4), agc_rate=(1e-5), agc_ref=1.37, antenna='RX2', bbframe_file='~/received_files/linkx_binary_bbf.bbf', centre_freq=1150e6, conj=0, debug=0, device_args='', dummy_frames=1, duty_period=0, duty_window=1.0, frame_size='', gold_code=0, input_file='~/g5/part2/unknown_gsuftp_test/linkx_binary.sc16', input_format='auto', ldpc_max_trials=25, modcod='6', mode='live', out_dir='~/received_files', panels='all', pl_freq_est_period=17, post_agc_attack=(1e-3), post_agc_decay=(1e-5), post_agc_ref=1.0, precomp_dt=0, precomp_offset_hz=0, precomp_sign=1, preset='linkx2', realtime=0, rec_dir='~/iq/recordings', rec_prefix='g5', record=(-1), report_json='', rf_freq=13.95e9, rolloff=0.2, rrc_delay=25, rrc_nfilts=128, rx_gain=30, sat='', sim_input='', sps=0, stack='auto', start_utc='', station='1.2977,103.7764,60', sym_rate=0, sym_sync_damping=1.0, sym_sync_loop_bw=0.0045, tle='', udp_forward='', udp_raw_out='', udp_raw_ref='', verify_hashes=''):
        gr.top_block.__init__(self, "G5 unified RX", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("G5 unified RX")
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

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "rx_unified")

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
        self.agc = agc
        self.agc3_attack = agc3_attack
        self.agc3_decay = agc3_decay
        self.agc_rate = agc_rate
        self.agc_ref = agc_ref
        self.antenna = antenna
        self.bbframe_file = bbframe_file
        self.centre_freq = centre_freq
        self.conj = conj
        self.debug = debug
        self.device_args = device_args
        self.dummy_frames = dummy_frames
        self.duty_period = duty_period
        self.duty_window = duty_window
        self.frame_size = frame_size
        self.gold_code = gold_code
        self.input_file = input_file
        self.input_format = input_format
        self.ldpc_max_trials = ldpc_max_trials
        self.modcod = modcod
        self.mode = mode
        self.out_dir = out_dir
        self.panels = panels
        self.pl_freq_est_period = pl_freq_est_period
        self.post_agc_attack = post_agc_attack
        self.post_agc_decay = post_agc_decay
        self.post_agc_ref = post_agc_ref
        self.precomp_dt = precomp_dt
        self.precomp_offset_hz = precomp_offset_hz
        self.precomp_sign = precomp_sign
        self.preset = preset
        self.realtime = realtime
        self.rec_dir = rec_dir
        self.rec_prefix = rec_prefix
        self.record = record
        self.report_json = report_json
        self.rf_freq = rf_freq
        self.rolloff = rolloff
        self.rrc_delay = rrc_delay
        self.rrc_nfilts = rrc_nfilts
        self.rx_gain = rx_gain
        self.sat = sat
        self.sim_input = sim_input
        self.sps = sps
        self.stack = stack
        self.start_utc = start_utc
        self.station = station
        self.sym_rate = sym_rate
        self.sym_sync_damping = sym_sync_damping
        self.sym_sync_loop_bw = sym_sync_loop_bw
        self.tle = tle
        self.udp_forward = udp_forward
        self.udp_raw_out = udp_raw_out
        self.udp_raw_ref = udp_raw_ref
        self.verify_hashes = verify_hashes

        ##################################################
        # Variables
        ##################################################
        self.cfg = cfg = g5gs.config.resolve(preset, sym_rate, sps, modcod, frame_size)
        self.samp_rate = samp_rate = cfg.samp_rate
        self.rs = rs = cfg.sym_rate
        self.rx_gain_slider = rx_gain_slider = rx_gain
        self.record_on = record_on = int(mode == 'record') if record < 0 else int(bool(record))
        self.pls_filter = pls_filter = dvbs2rx.pls_filter(dvbs2rx.dvbs2_pls(cfg.constellation, cfg.code_rate, cfg.frame_size, True))
        self.panel_groups = panel_groups = {'spectrum': ['valve_spec_in', 'valve_spec_pl', 'qtgui_freq_sink_in', 'qtgui_freq_sink_pl'], 'constellation': ['valve_const', 'qtgui_const_sink_pl'], 'status': ['rx_status_panel_0'], 'doppler': ['doppler_panel_0'], 'files': ['files_panel_0']}
        self.out_dir_eff = out_dir_eff = out_dir or g5gs.config.default_out_dir(mode, input_file, rec_dir)
        self.meta = meta = {'preset': preset, 'modcod': cfg.modcod, 'frame_size': cfg.frame_size, 'sym_rate': rs, 'sps': cfg.sps, 'tle': tle, 'conj': bool(conj)}
        self.gate_cfg = gate_cfg = g5gs.auto_mode(mode, samp_rate, duty_period)
        self.cfg_text = cfg_text = f'{mode}: {cfg.describe()}' + (f', TLE {tle}' if tle else ', no TLE') + f', AGC {agc} {agc_ref:g}, post-AGC {post_agc_ref:g}, period {pl_freq_est_period}' + (', dummy frames' if dummy_frames else '')
        self.antenna_sel = antenna_sel = g5gs.config.antenna_name(antenna)

        ##################################################
        # Blocks
        ##################################################

        self.tabs = Qt.QTabWidget()
        self.tabs_widget_0 = Qt.QWidget()
        self.tabs_layout_0 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tabs_widget_0)
        self.tabs_grid_layout_0 = Qt.QGridLayout()
        self.tabs_layout_0.addLayout(self.tabs_grid_layout_0)
        self.tabs.addTab(self.tabs_widget_0, 'Spectrum')
        self.tabs_widget_1 = Qt.QWidget()
        self.tabs_layout_1 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tabs_widget_1)
        self.tabs_grid_layout_1 = Qt.QGridLayout()
        self.tabs_layout_1.addLayout(self.tabs_grid_layout_1)
        self.tabs.addTab(self.tabs_widget_1, 'Constellation')
        self.tabs_widget_2 = Qt.QWidget()
        self.tabs_layout_2 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tabs_widget_2)
        self.tabs_grid_layout_2 = Qt.QGridLayout()
        self.tabs_layout_2.addLayout(self.tabs_grid_layout_2)
        self.tabs.addTab(self.tabs_widget_2, 'Doppler / pass')
        self.tabs_widget_3 = Qt.QWidget()
        self.tabs_layout_3 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tabs_widget_3)
        self.tabs_grid_layout_3 = Qt.QGridLayout()
        self.tabs_layout_3.addLayout(self.tabs_grid_layout_3)
        self.tabs.addTab(self.tabs_widget_3, 'Files')
        self.top_grid_layout.addWidget(self.tabs, 2, 1, 1, 1)
        for r in range(2, 3):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._rx_gain_slider_range = qtgui.Range(0, 76, 1, rx_gain, 200)
        self._rx_gain_slider_win = qtgui.RangeWidget(self._rx_gain_slider_range, self.set_rx_gain_slider, "RX gain (dB)", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._rx_gain_slider_win, 1, 0, 1, 1)
        for r in range(1, 2):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        _record_on_check_box = Qt.QCheckBox("Record to disk")
        self._record_on_choices = {True: 1, False: 0}
        self._record_on_choices_inv = dict((v,k) for k,v in self._record_on_choices.items())
        self._record_on_callback = lambda i: Qt.QMetaObject.invokeMethod(_record_on_check_box, "setChecked", Qt.Q_ARG("bool", self._record_on_choices_inv[i]))
        self._record_on_callback(self.record_on)
        _record_on_check_box.stateChanged.connect(lambda i: self.set_record_on(self._record_on_choices[bool(i)]))
        self.top_grid_layout.addWidget(_record_on_check_box, 1, 1, 1, 1)
        for r in range(1, 2):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        # Create the options list
        self._antenna_sel_options = ['RX2', 'TX/RX']
        # Create the labels list
        self._antenna_sel_labels = ['RX2', 'TX/RX']
        # Create the combo box
        self._antenna_sel_tool_bar = Qt.QToolBar(self)
        self._antenna_sel_tool_bar.addWidget(Qt.QLabel("RX port" + ": "))
        self._antenna_sel_combo_box = Qt.QComboBox()
        self._antenna_sel_tool_bar.addWidget(self._antenna_sel_combo_box)
        for _label in self._antenna_sel_labels: self._antenna_sel_combo_box.addItem(_label)
        self._antenna_sel_callback = lambda i: Qt.QMetaObject.invokeMethod(self._antenna_sel_combo_box, "setCurrentIndex", Qt.Q_ARG("int", self._antenna_sel_options.index(i)))
        self._antenna_sel_callback(self.antenna_sel)
        self._antenna_sel_combo_box.currentIndexChanged.connect(
            lambda i: self.set_antenna_sel(self._antenna_sel_options[i]))
        # Create the radio buttons
        self.top_grid_layout.addWidget(self._antenna_sel_tool_bar, 0, 1, 1, 1)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.valve_spec_pl = blocks.copy(gr.sizeof_gr_complex*1)
        self.valve_spec_pl.set_enabled(True)
        self.valve_spec_in = blocks.copy(gr.sizeof_gr_complex*1)
        self.valve_spec_in.set_enabled(True)
        self.valve_const = blocks.copy(gr.sizeof_gr_complex*1)
        self.valve_const.set_enabled(True)
        self.rx_status_panel_0 = g5gs.RxStatusPanel(self, sym_rate=rs, cfg_text=cfg_text, dummy_frames=bool(dummy_frames), active=True)
        self._rx_status_panel_0_win = self.rx_status_panel_0
        self.top_grid_layout.addWidget(self._rx_status_panel_0_win, 2, 0, 1, 1)
        for r in range(2, 3):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.rx_source_0 = g5gs.rx_source(mode=mode, samp_rate=samp_rate, sym_rate=rs, input_file=input_file, input_format=input_format, realtime=bool(realtime), need_time=bool(tle), start_utc=start_utc, centre_freq=centre_freq, rx_gain=rx_gain_slider, device_args=device_args, lo_offset=(-1), record=bool(record_on), rec_dir=rec_dir, rec_prefix=rec_prefix, min_free=(2 * 2**30), meta=meta, conj=bool(conj), sim_input=sim_input, antenna=antenna_sel)
        self.qtgui_freq_sink_pl = qtgui.freq_sink_c(
            1024, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            rs, #bw
            "After PL sync (1 sample/symbol)", #name
            1,
            None # parent
        )
        self.qtgui_freq_sink_pl.set_update_time(0.10)
        self.qtgui_freq_sink_pl.set_y_axis((-140), 10)
        self.qtgui_freq_sink_pl.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_pl.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_pl.enable_autoscale(False)
        self.qtgui_freq_sink_pl.enable_grid(True)
        self.qtgui_freq_sink_pl.set_fft_average(0.2)
        self.qtgui_freq_sink_pl.enable_axis_labels(True)
        self.qtgui_freq_sink_pl.enable_control_panel(False)
        self.qtgui_freq_sink_pl.set_fft_window_normalized(False)

        self.qtgui_freq_sink_pl.disable_legend()


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
                self.qtgui_freq_sink_pl.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_pl.set_line_label(i, labels[i])
            self.qtgui_freq_sink_pl.set_line_width(i, widths[i])
            self.qtgui_freq_sink_pl.set_line_color(i, colors[i])
            self.qtgui_freq_sink_pl.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_pl_win = sip.wrapinstance(self.qtgui_freq_sink_pl.qwidget(), Qt.QWidget)
        self.tabs_grid_layout_0.addWidget(self._qtgui_freq_sink_pl_win, 1, 0, 1, 1)
        for r in range(1, 2):
            self.tabs_grid_layout_0.setRowStretch(r, 1)
        for c in range(0, 1):
            self.tabs_grid_layout_0.setColumnStretch(c, 1)
        self.qtgui_freq_sink_in = qtgui.freq_sink_c(
            1024, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            centre_freq, #fc
            samp_rate, #bw
            "Input (before AGC)", #name
            1,
            None # parent
        )
        self.qtgui_freq_sink_in.set_update_time(0.10)
        self.qtgui_freq_sink_in.set_y_axis((-140), 10)
        self.qtgui_freq_sink_in.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_in.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_in.enable_autoscale(False)
        self.qtgui_freq_sink_in.enable_grid(True)
        self.qtgui_freq_sink_in.set_fft_average(0.2)
        self.qtgui_freq_sink_in.enable_axis_labels(True)
        self.qtgui_freq_sink_in.enable_control_panel(False)
        self.qtgui_freq_sink_in.set_fft_window_normalized(False)

        self.qtgui_freq_sink_in.disable_legend()


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
                self.qtgui_freq_sink_in.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_in.set_line_label(i, labels[i])
            self.qtgui_freq_sink_in.set_line_width(i, widths[i])
            self.qtgui_freq_sink_in.set_line_color(i, colors[i])
            self.qtgui_freq_sink_in.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_in_win = sip.wrapinstance(self.qtgui_freq_sink_in.qwidget(), Qt.QWidget)
        self.tabs_grid_layout_0.addWidget(self._qtgui_freq_sink_in_win, 0, 0, 1, 1)
        for r in range(0, 1):
            self.tabs_grid_layout_0.setRowStretch(r, 1)
        for c in range(0, 1):
            self.tabs_grid_layout_0.setColumnStretch(c, 1)
        self.qtgui_const_sink_pl = qtgui.const_sink_c(
            2048, #size
            "PL sync output (data, pilots, headers)", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_const_sink_pl.set_update_time(0.20)
        self.qtgui_const_sink_pl.set_y_axis((-2), 2)
        self.qtgui_const_sink_pl.set_x_axis((-2), 2)
        self.qtgui_const_sink_pl.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, "")
        self.qtgui_const_sink_pl.enable_autoscale(False)
        self.qtgui_const_sink_pl.enable_grid(True)
        self.qtgui_const_sink_pl.enable_axis_labels(True)

        self.qtgui_const_sink_pl.disable_legend()

        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "blue", "blue", "blue", "blue",
            "blue", "blue", "blue", "blue", "blue"]
        styles = [0, 0, 0, 0, 0,
            0, 0, 0, 0, 0]
        markers = [0, 0, 0, 0, 0,
            0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_const_sink_pl.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_const_sink_pl.set_line_label(i, labels[i])
            self.qtgui_const_sink_pl.set_line_width(i, widths[i])
            self.qtgui_const_sink_pl.set_line_color(i, colors[i])
            self.qtgui_const_sink_pl.set_line_style(i, styles[i])
            self.qtgui_const_sink_pl.set_line_marker(i, markers[i])
            self.qtgui_const_sink_pl.set_line_alpha(i, alphas[i])

        self._qtgui_const_sink_pl_win = sip.wrapinstance(self.qtgui_const_sink_pl.qwidget(), Qt.QWidget)
        self.tabs_grid_layout_1.addWidget(self._qtgui_const_sink_pl_win, 0, 0, 1, 1)
        for r in range(0, 1):
            self.tabs_grid_layout_1.setRowStretch(r, 1)
        for c in range(0, 1):
            self.tabs_grid_layout_1.setColumnStretch(c, 1)
        self.post_agc_0 = g5gs.post_agc(ref=post_agc_ref, attack=post_agc_attack, decay=post_agc_decay)
        self.panel_switch_0 = g5gs.PanelSwitch(self, groups=panel_groups, panels=panels, main_row=2)
        self._panel_switch_0_win = self.panel_switch_0
        self.top_grid_layout.addWidget(self._panel_switch_0_win, 0, 0, 1, 1)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.gs_decoder_0 = g5gs.gs_decoder(frame_len=cfg.kbch_bytes, stack=stack, out_dir=out_dir_eff, verify_hashes=verify_hashes, bbframe_file=bbframe_file, udp_forward=udp_forward, udp_raw_out=udp_raw_out, udp_raw_ref=udp_raw_ref, report_json=report_json, dump_payloads='', isi=(-1), udp_port=0)
        self.g5gs_doppler_panel_0 = g5gs.DopplerPanel(self, sym_rate=20e6, history_s=600, active=True)
        self._g5gs_doppler_panel_0_win = self.g5gs_doppler_panel_0
        self.top_layout.addWidget(self._g5gs_doppler_panel_0_win)
        self.front_agc_0 = g5gs.front_agc(kind=agc, ref=agc_ref, attack=agc3_attack, decay=agc3_decay, rate=agc_rate, gain=1.0, max_gain=65536)
        self.dvbs2rx_xfecframe_demapper_cb_0 = dvbs2rx.xfecframe_demapper_cb(
          *dvbs2rx.params.translate("DVB-S2", cfg.frame_size, cfg.code_rate,
          cfg.constellation)[1:])
        self.dvbs2rx_symbol_sync_cc_0 = dvbs2rx.symbol_sync_cc(cfg.sps, sym_sync_loop_bw, sym_sync_damping, rolloff, rrc_delay, rrc_nfilts, 0)
        self.dvbs2rx_rotator_cc_0 = dvbs2rx.rotator_cc(0, True)
        self.dvbs2rx_plsync_cc_0 = dvbs2rx.plsync_cc(gold_code, pl_freq_est_period, cfg.sps, debug, False, bool(dummy_frames), pls_filter[0], pls_filter[1])
        self.dvbs2rx_ldpc_decoder_bb_0 = dvbs2rx.ldpc_decoder_bb(
            *dvbs2rx.params.translate('DVB-S2',
                cfg.frame_size,
                cfg.code_rate,
                cfg.constellation
            ),
            dvbs2rx.OM_MESSAGE,
            dvbs2rx.INFO_OFF,
            ldpc_max_trials,
            debug)
        self.dvbs2rx_bch_decoder_bb_0 = dvbs2rx.bch_decoder_bb(
            *dvbs2rx.params.translate('DVB-S2',
                cfg.frame_size,
                cfg.code_rate
            ),
            dvbs2rx.OM_MESSAGE,
            debug)
        self.dvbs2rx_bbdescrambler_bb_0 = dvbs2rx.bbdescrambler_bb(
            *dvbs2rx.params.translate('DVB-S2',
                cfg.frame_size,
                cfg.code_rate
            )
        )
        self.duty_gate_0 = g5gs.duty_gate(mode=gate_cfg[0], samp_rate=samp_rate, window=duty_window, period=gate_cfg[1] or 5.0, queue_s=0.5)
        self.doppler_precomp_0 = g5gs.doppler_precomp(tle=tle, sat=sat, samp_rate=samp_rate, station=station, rf_freq=rf_freq, sign=precomp_sign, dt=precomp_dt, offset_hz=precomp_offset_hz, start_utc=start_utc)
        self.doppler_panel_0 = g5gs.DopplerPanel(self, sym_rate=rs, history_s=900, active=True)
        self._doppler_panel_0_win = self.doppler_panel_0
        self.tabs_grid_layout_2.addWidget(self._doppler_panel_0_win, 0, 0, 1, 1)
        for r in range(0, 1):
            self.tabs_grid_layout_2.setRowStretch(r, 1)
        for c in range(0, 1):
            self.tabs_grid_layout_2.setColumnStretch(c, 1)


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.dvbs2rx_ldpc_decoder_bb_0, 'llr_pdu'), (self.dvbs2rx_xfecframe_demapper_cb_0, 'llr_pdu'))
        self.msg_connect((self.dvbs2rx_plsync_cc_0, 'rotator_phase_inc'), (self.dvbs2rx_rotator_cc_0, 'cmd'))
        self.connect((self.doppler_precomp_0, 0), (self.front_agc_0, 0))
        self.connect((self.duty_gate_0, 0), (self.doppler_precomp_0, 0))
        self.connect((self.dvbs2rx_bbdescrambler_bb_0, 0), (self.gs_decoder_0, 0))
        self.connect((self.dvbs2rx_bch_decoder_bb_0, 0), (self.dvbs2rx_bbdescrambler_bb_0, 0))
        self.connect((self.dvbs2rx_ldpc_decoder_bb_0, 0), (self.dvbs2rx_bch_decoder_bb_0, 0))
        self.connect((self.dvbs2rx_plsync_cc_0, 0), (self.dvbs2rx_xfecframe_demapper_cb_0, 0))
        self.connect((self.dvbs2rx_plsync_cc_0, 0), (self.valve_const, 0))
        self.connect((self.dvbs2rx_plsync_cc_0, 0), (self.valve_spec_pl, 0))
        self.connect((self.dvbs2rx_rotator_cc_0, 0), (self.dvbs2rx_symbol_sync_cc_0, 0))
        self.connect((self.dvbs2rx_symbol_sync_cc_0, 0), (self.post_agc_0, 0))
        self.connect((self.dvbs2rx_xfecframe_demapper_cb_0, 0), (self.dvbs2rx_ldpc_decoder_bb_0, 0))
        self.connect((self.front_agc_0, 0), (self.dvbs2rx_rotator_cc_0, 0))
        self.connect((self.post_agc_0, 0), (self.dvbs2rx_plsync_cc_0, 0))
        self.connect((self.rx_source_0, 0), (self.duty_gate_0, 0))
        self.connect((self.rx_source_0, 0), (self.valve_spec_in, 0))
        self.connect((self.valve_const, 0), (self.qtgui_const_sink_pl, 0))
        self.connect((self.valve_spec_in, 0), (self.qtgui_freq_sink_in, 0))
        self.connect((self.valve_spec_pl, 0), (self.qtgui_freq_sink_pl, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "rx_unified")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_agc(self):
        return self.agc

    def set_agc(self, agc):
        self.agc = agc

    def get_agc3_attack(self):
        return self.agc3_attack

    def set_agc3_attack(self, agc3_attack):
        self.agc3_attack = agc3_attack

    def get_agc3_decay(self):
        return self.agc3_decay

    def set_agc3_decay(self, agc3_decay):
        self.agc3_decay = agc3_decay

    def get_agc_rate(self):
        return self.agc_rate

    def set_agc_rate(self, agc_rate):
        self.agc_rate = agc_rate

    def get_agc_ref(self):
        return self.agc_ref

    def set_agc_ref(self, agc_ref):
        self.agc_ref = agc_ref

    def get_antenna(self):
        return self.antenna

    def set_antenna(self, antenna):
        self.antenna = antenna
        self.set_antenna_sel(g5gs.config.antenna_name(self.antenna))

    def get_bbframe_file(self):
        return self.bbframe_file

    def set_bbframe_file(self, bbframe_file):
        self.bbframe_file = bbframe_file

    def get_centre_freq(self):
        return self.centre_freq

    def set_centre_freq(self, centre_freq):
        self.centre_freq = centre_freq
        self.qtgui_freq_sink_in.set_frequency_range(self.centre_freq, self.samp_rate)

    def get_conj(self):
        return self.conj

    def set_conj(self, conj):
        self.conj = conj
        self.set_meta({'preset': self.preset, 'modcod': cfg.modcod, 'frame_size': cfg.frame_size, 'sym_rate': self.rs, 'sps': cfg.sps, 'tle': self.tle, 'conj': bool(self.conj)})

    def get_debug(self):
        return self.debug

    def set_debug(self, debug):
        self.debug = debug

    def get_device_args(self):
        return self.device_args

    def set_device_args(self, device_args):
        self.device_args = device_args

    def get_dummy_frames(self):
        return self.dummy_frames

    def set_dummy_frames(self, dummy_frames):
        self.dummy_frames = dummy_frames
        self.set_cfg_text(f'{mode}: {cfg.describe()}' + (f', TLE {tle}' if self.tle else ', no TLE') + f', AGC {agc} {agc_ref:g}, post-AGC {post_agc_ref:g}, period {pl_freq_est_period}' + (', dummy frames' if self.dummy_frames else ''))

    def get_duty_period(self):
        return self.duty_period

    def set_duty_period(self, duty_period):
        self.duty_period = duty_period
        self.set_gate_cfg(g5gs.auto_mode(self.mode, self.samp_rate, self.duty_period))

    def get_duty_window(self):
        return self.duty_window

    def set_duty_window(self, duty_window):
        self.duty_window = duty_window

    def get_frame_size(self):
        return self.frame_size

    def set_frame_size(self, frame_size):
        self.frame_size = frame_size
        self.set_cfg(g5gs.config.resolve(self.preset, self.sym_rate, self.sps, self.modcod, self.frame_size))

    def get_gold_code(self):
        return self.gold_code

    def set_gold_code(self, gold_code):
        self.gold_code = gold_code

    def get_input_file(self):
        return self.input_file

    def set_input_file(self, input_file):
        self.input_file = input_file
        self.set_out_dir_eff(self.out_dir or g5gs.config.default_out_dir(self.mode, self.input_file, self.rec_dir))

    def get_input_format(self):
        return self.input_format

    def set_input_format(self, input_format):
        self.input_format = input_format

    def get_ldpc_max_trials(self):
        return self.ldpc_max_trials

    def set_ldpc_max_trials(self, ldpc_max_trials):
        self.ldpc_max_trials = ldpc_max_trials

    def get_modcod(self):
        return self.modcod

    def set_modcod(self, modcod):
        self.modcod = modcod
        self.set_cfg(g5gs.config.resolve(self.preset, self.sym_rate, self.sps, self.modcod, self.frame_size))

    def get_mode(self):
        return self.mode

    def set_mode(self, mode):
        self.mode = mode
        self.set_gate_cfg(g5gs.auto_mode(self.mode, self.samp_rate, self.duty_period))
        self.set_out_dir_eff(self.out_dir or g5gs.config.default_out_dir(self.mode, self.input_file, self.rec_dir))
        self.set_record_on(int(self.mode == 'record') if self.record < 0 else int(bool(self.record)))

    def get_out_dir(self):
        return self.out_dir

    def set_out_dir(self, out_dir):
        self.out_dir = out_dir
        self.set_out_dir_eff(self.out_dir or g5gs.config.default_out_dir(self.mode, self.input_file, self.rec_dir))

    def get_panels(self):
        return self.panels

    def set_panels(self, panels):
        self.panels = panels

    def get_pl_freq_est_period(self):
        return self.pl_freq_est_period

    def set_pl_freq_est_period(self, pl_freq_est_period):
        self.pl_freq_est_period = pl_freq_est_period

    def get_post_agc_attack(self):
        return self.post_agc_attack

    def set_post_agc_attack(self, post_agc_attack):
        self.post_agc_attack = post_agc_attack

    def get_post_agc_decay(self):
        return self.post_agc_decay

    def set_post_agc_decay(self, post_agc_decay):
        self.post_agc_decay = post_agc_decay

    def get_post_agc_ref(self):
        return self.post_agc_ref

    def set_post_agc_ref(self, post_agc_ref):
        self.post_agc_ref = post_agc_ref

    def get_precomp_dt(self):
        return self.precomp_dt

    def set_precomp_dt(self, precomp_dt):
        self.precomp_dt = precomp_dt

    def get_precomp_offset_hz(self):
        return self.precomp_offset_hz

    def set_precomp_offset_hz(self, precomp_offset_hz):
        self.precomp_offset_hz = precomp_offset_hz

    def get_precomp_sign(self):
        return self.precomp_sign

    def set_precomp_sign(self, precomp_sign):
        self.precomp_sign = precomp_sign

    def get_preset(self):
        return self.preset

    def set_preset(self, preset):
        self.preset = preset
        self.set_cfg(g5gs.config.resolve(self.preset, self.sym_rate, self.sps, self.modcod, self.frame_size))
        self.set_meta({'preset': self.preset, 'modcod': cfg.modcod, 'frame_size': cfg.frame_size, 'sym_rate': self.rs, 'sps': cfg.sps, 'tle': self.tle, 'conj': bool(self.conj)})

    def get_realtime(self):
        return self.realtime

    def set_realtime(self, realtime):
        self.realtime = realtime

    def get_rec_dir(self):
        return self.rec_dir

    def set_rec_dir(self, rec_dir):
        self.rec_dir = rec_dir
        self.set_out_dir_eff(self.out_dir or g5gs.config.default_out_dir(self.mode, self.input_file, self.rec_dir))

    def get_rec_prefix(self):
        return self.rec_prefix

    def set_rec_prefix(self, rec_prefix):
        self.rec_prefix = rec_prefix

    def get_record(self):
        return self.record

    def set_record(self, record):
        self.record = record
        self.set_record_on(int(self.mode == 'record') if self.record < 0 else int(bool(self.record)))

    def get_report_json(self):
        return self.report_json

    def set_report_json(self, report_json):
        self.report_json = report_json

    def get_rf_freq(self):
        return self.rf_freq

    def set_rf_freq(self, rf_freq):
        self.rf_freq = rf_freq

    def get_rolloff(self):
        return self.rolloff

    def set_rolloff(self, rolloff):
        self.rolloff = rolloff

    def get_rrc_delay(self):
        return self.rrc_delay

    def set_rrc_delay(self, rrc_delay):
        self.rrc_delay = rrc_delay

    def get_rrc_nfilts(self):
        return self.rrc_nfilts

    def set_rrc_nfilts(self, rrc_nfilts):
        self.rrc_nfilts = rrc_nfilts

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = rx_gain
        self.set_rx_gain_slider(self.rx_gain)

    def get_sat(self):
        return self.sat

    def set_sat(self, sat):
        self.sat = sat

    def get_sim_input(self):
        return self.sim_input

    def set_sim_input(self, sim_input):
        self.sim_input = sim_input

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.set_cfg(g5gs.config.resolve(self.preset, self.sym_rate, self.sps, self.modcod, self.frame_size))

    def get_stack(self):
        return self.stack

    def set_stack(self, stack):
        self.stack = stack

    def get_start_utc(self):
        return self.start_utc

    def set_start_utc(self, start_utc):
        self.start_utc = start_utc

    def get_station(self):
        return self.station

    def set_station(self, station):
        self.station = station

    def get_sym_rate(self):
        return self.sym_rate

    def set_sym_rate(self, sym_rate):
        self.sym_rate = sym_rate
        self.set_cfg(g5gs.config.resolve(self.preset, self.sym_rate, self.sps, self.modcod, self.frame_size))

    def get_sym_sync_damping(self):
        return self.sym_sync_damping

    def set_sym_sync_damping(self, sym_sync_damping):
        self.sym_sync_damping = sym_sync_damping

    def get_sym_sync_loop_bw(self):
        return self.sym_sync_loop_bw

    def set_sym_sync_loop_bw(self, sym_sync_loop_bw):
        self.sym_sync_loop_bw = sym_sync_loop_bw

    def get_tle(self):
        return self.tle

    def set_tle(self, tle):
        self.tle = tle
        self.set_cfg_text(f'{mode}: {cfg.describe()}' + (f', TLE {tle}' if self.tle else ', no TLE') + f', AGC {agc} {agc_ref:g}, post-AGC {post_agc_ref:g}, period {pl_freq_est_period}' + (', dummy frames' if self.dummy_frames else ''))
        self.set_meta({'preset': self.preset, 'modcod': cfg.modcod, 'frame_size': cfg.frame_size, 'sym_rate': self.rs, 'sps': cfg.sps, 'tle': self.tle, 'conj': bool(self.conj)})

    def get_udp_forward(self):
        return self.udp_forward

    def set_udp_forward(self, udp_forward):
        self.udp_forward = udp_forward

    def get_udp_raw_out(self):
        return self.udp_raw_out

    def set_udp_raw_out(self, udp_raw_out):
        self.udp_raw_out = udp_raw_out

    def get_udp_raw_ref(self):
        return self.udp_raw_ref

    def set_udp_raw_ref(self, udp_raw_ref):
        self.udp_raw_ref = udp_raw_ref

    def get_verify_hashes(self):
        return self.verify_hashes

    def set_verify_hashes(self, verify_hashes):
        self.verify_hashes = verify_hashes

    def get_cfg(self):
        return self.cfg

    def set_cfg(self, cfg):
        self.cfg = cfg

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_gate_cfg(g5gs.auto_mode(self.mode, self.samp_rate, self.duty_period))
        self.qtgui_freq_sink_in.set_frequency_range(self.centre_freq, self.samp_rate)

    def get_rs(self):
        return self.rs

    def set_rs(self, rs):
        self.rs = rs
        self.set_meta({'preset': self.preset, 'modcod': cfg.modcod, 'frame_size': cfg.frame_size, 'sym_rate': self.rs, 'sps': cfg.sps, 'tle': self.tle, 'conj': bool(self.conj)})
        self.qtgui_freq_sink_pl.set_frequency_range(0, self.rs)

    def get_rx_gain_slider(self):
        return self.rx_gain_slider

    def set_rx_gain_slider(self, rx_gain_slider):
        self.rx_gain_slider = rx_gain_slider
        self.rx_source_0.set_rx_gain(self.rx_gain_slider)

    def get_record_on(self):
        return self.record_on

    def set_record_on(self, record_on):
        self.record_on = record_on
        self._record_on_callback(self.record_on)
        self.rx_source_0.set_recording(bool(self.record_on))

    def get_pls_filter(self):
        return self.pls_filter

    def set_pls_filter(self, pls_filter):
        self.pls_filter = pls_filter

    def get_panel_groups(self):
        return self.panel_groups

    def set_panel_groups(self, panel_groups):
        self.panel_groups = panel_groups

    def get_out_dir_eff(self):
        return self.out_dir_eff

    def set_out_dir_eff(self, out_dir_eff):
        self.out_dir_eff = out_dir_eff

    def get_meta(self):
        return self.meta

    def set_meta(self, meta):
        self.meta = meta

    def get_gate_cfg(self):
        return self.gate_cfg

    def set_gate_cfg(self, gate_cfg):
        self.gate_cfg = gate_cfg

    def get_cfg_text(self):
        return self.cfg_text

    def set_cfg_text(self, cfg_text):
        self.cfg_text = cfg_text

    def get_antenna_sel(self):
        return self.antenna_sel

    def set_antenna_sel(self, antenna_sel):
        self.antenna_sel = antenna_sel
        self._antenna_sel_callback(self.antenna_sel)
        self.rx_source_0.set_antenna(self.antenna_sel)



def argument_parser():
    description = 'Galassia-5 ground-station receiver: B205mini record + monitor / live decode / file decode, patched gr-dvbs2rx, optional TLE Doppler pre-comp, GSSE/GSUFTP files. Parameters = command-line options (rx_unified.py --help). README.md.'
    parser = ArgumentParser(description=description)
    parser.add_argument(
        "--agc", dest="agc", type=str, default='agc3',
        help="Set Front AGC (agc3 / agc) [default=%(default)r]")
    parser.add_argument(
        "--agc3-attack", dest="agc3_attack", type=eng_float, default=eng_notation.num_to_str(float((1e-2))),
        help="Set AGC3 attack [default=%(default)r]")
    parser.add_argument(
        "--agc3-decay", dest="agc3_decay", type=eng_float, default=eng_notation.num_to_str(float((1e-4))),
        help="Set AGC3 decay [default=%(default)r]")
    parser.add_argument(
        "--agc-rate", dest="agc_rate", type=eng_float, default=eng_notation.num_to_str(float((1e-5))),
        help="Set agc_cc rate [default=%(default)r]")
    parser.add_argument(
        "--agc-ref", dest="agc_ref", type=eng_float, default=eng_notation.num_to_str(float(1.37)),
        help="Set Front AGC reference [default=%(default)r]")
    parser.add_argument(
        "--antenna", dest="antenna", type=str, default='RX2',
        help="Set RX port (RX2 / TX/RX) [default=%(default)r]")
    parser.add_argument(
        "--bbframe-file", dest="bbframe_file", type=str, default='~/received_files/linkx_binary_bbf.bbf',
        help="Set Also write BBFRAMEs to [default=%(default)r]")
    parser.add_argument(
        "--centre-freq", dest="centre_freq", type=eng_float, default=eng_notation.num_to_str(float(1150e6)),
        help="Set Centre frequency [default=%(default)r]")
    parser.add_argument(
        "--conj", dest="conj", type=intx, default=0,
        help="Set Conjugate input (1 = yes) [default=%(default)r]")
    parser.add_argument(
        "--debug", dest="debug", type=intx, default=0,
        help="Set Debug level [default=%(default)r]")
    parser.add_argument(
        "--device-args", dest="device_args", type=str, default='',
        help="Set UHD device args [default=%(default)r]")
    parser.add_argument(
        "--dummy-frames", dest="dummy_frames", type=intx, default=1,
        help="Set Dummy frames (1 = on) [default=%(default)r]")
    parser.add_argument(
        "--duty-period", dest="duty_period", type=eng_float, default=eng_notation.num_to_str(float(0)),
        help="Set Monitor period (s, 0 = auto) [default=%(default)r]")
    parser.add_argument(
        "--duty-window", dest="duty_window", type=eng_float, default=eng_notation.num_to_str(float(1.0)),
        help="Set Monitor window (s) [default=%(default)r]")
    parser.add_argument(
        "--frame-size", dest="frame_size", type=str, default='',
        help="Set Frame size (empty = preset) [default=%(default)r]")
    parser.add_argument(
        "--gold-code", dest="gold_code", type=intx, default=0,
        help="Set Gold code [default=%(default)r]")
    parser.add_argument(
        "--input-file", dest="input_file", type=str, default='~/g5/part2/unknown_gsuftp_test/linkx_binary.sc16',
        help="Set Input file (file mode) [default=%(default)r]")
    parser.add_argument(
        "--input-format", dest="input_format", type=str, default='auto',
        help="Set Input format (auto / sc16 / fc32) [default=%(default)r]")
    parser.add_argument(
        "--ldpc-max-trials", dest="ldpc_max_trials", type=intx, default=25,
        help="Set LDPC max iterations [default=%(default)r]")
    parser.add_argument(
        "--modcod", dest="modcod", type=str, default='6',
        help="Set MODCOD (empty = preset) [default=%(default)r]")
    parser.add_argument(
        "--mode", dest="mode", type=str, default='live',
        help="Set Mode (record / live / file) [default=%(default)r]")
    parser.add_argument(
        "--out-dir", dest="out_dir", type=str, default='~/received_files',
        help="Set Decoded files folder (empty = auto) [default=%(default)r]")
    parser.add_argument(
        "--panels", dest="panels", type=str, default='all',
        help="Set Panels at start (all / none / list) [default=%(default)r]")
    parser.add_argument(
        "--pl-freq-est-period", dest="pl_freq_est_period", type=intx, default=17,
        help="Set PL freq. est. period (prime) [default=%(default)r]")
    parser.add_argument(
        "--post-agc-attack", dest="post_agc_attack", type=eng_float, default=eng_notation.num_to_str(float((1e-3))),
        help="Set Post-AGC attack / symbol [default=%(default)r]")
    parser.add_argument(
        "--post-agc-decay", dest="post_agc_decay", type=eng_float, default=eng_notation.num_to_str(float((1e-5))),
        help="Set Post-AGC decay / symbol [default=%(default)r]")
    parser.add_argument(
        "--post-agc-ref", dest="post_agc_ref", type=eng_float, default=eng_notation.num_to_str(float(1.0)),
        help="Set Post-AGC ref (0 = off) [default=%(default)r]")
    parser.add_argument(
        "--precomp-dt", dest="precomp_dt", type=eng_float, default=eng_notation.num_to_str(float(0)),
        help="Set Pre-comp time shift (s) [default=%(default)r]")
    parser.add_argument(
        "--precomp-offset-hz", dest="precomp_offset_hz", type=eng_float, default=eng_notation.num_to_str(float(0)),
        help="Set Pre-comp extra offset (Hz) [default=%(default)r]")
    parser.add_argument(
        "--precomp-sign", dest="precomp_sign", type=eng_float, default=eng_notation.num_to_str(float(1)),
        help="Set Pre-comp sign (+1 / -1) [default=%(default)r]")
    parser.add_argument(
        "--preset", dest="preset", type=str, default='linkx2',
        help="Set Preset (ops20 / linkx2 / field1) [default=%(default)r]")
    parser.add_argument(
        "--realtime", dest="realtime", type=intx, default=0,
        help="Set File in real time (1 = yes) [default=%(default)r]")
    parser.add_argument(
        "--rec-dir", dest="rec_dir", type=str, default='~/iq/recordings',
        help="Set Recording folder [default=%(default)r]")
    parser.add_argument(
        "--rec-prefix", dest="rec_prefix", type=str, default='g5',
        help="Set Recording prefix [default=%(default)r]")
    parser.add_argument(
        "--record", dest="record", type=intx, default=(-1),
        help="Set Record (-1 = auto, 0, 1) [default=%(default)r]")
    parser.add_argument(
        "--report-json", dest="report_json", type=str, default='',
        help="Set JSON report [default=%(default)r]")
    parser.add_argument(
        "--rf-freq", dest="rf_freq", type=eng_float, default=eng_notation.num_to_str(float(13.95e9)),
        help="Set Downlink RF (Hz) [default=%(default)r]")
    parser.add_argument(
        "--rolloff", dest="rolloff", type=eng_float, default=eng_notation.num_to_str(float(0.2)),
        help="Set Roll-off [default=%(default)r]")
    parser.add_argument(
        "--rrc-delay", dest="rrc_delay", type=intx, default=25,
        help="Set RRC delay [default=%(default)r]")
    parser.add_argument(
        "--rrc-nfilts", dest="rrc_nfilts", type=intx, default=128,
        help="Set Polyphase RRC branches [default=%(default)r]")
    parser.add_argument(
        "--rx-gain", dest="rx_gain", type=eng_float, default=eng_notation.num_to_str(float(30)),
        help="Set RX gain (dB) [default=%(default)r]")
    parser.add_argument(
        "--sat", dest="sat", type=str, default='',
        help="Set Satellite (if several sets) [default=%(default)r]")
    parser.add_argument(
        "--sim-input", dest="sim_input", type=str, default='',
        help="Set Rehearsal sc16 instead of radio [default=%(default)r]")
    parser.add_argument(
        "--sps", dest="sps", type=intx, default=0,
        help="Set Samples/symbol (0 = preset) [default=%(default)r]")
    parser.add_argument(
        "--stack", dest="stack", type=str, default='auto',
        help="Set GSSE stack (auto / rev1.0 / rev1.2) [default=%(default)r]")
    parser.add_argument(
        "--start-utc", dest="start_utc", type=str, default='',
        help="Set First-sample UTC (file, override) [default=%(default)r]")
    parser.add_argument(
        "--station", dest="station", type=str, default='1.2977,103.7764,60',
        help="Set Station lat,lon,h [default=%(default)r]")
    parser.add_argument(
        "--sym-rate", dest="sym_rate", type=eng_float, default=eng_notation.num_to_str(float(0)),
        help="Set Symbol rate (0 = preset) [default=%(default)r]")
    parser.add_argument(
        "--sym-sync-damping", dest="sym_sync_damping", type=eng_float, default=eng_notation.num_to_str(float(1.0)),
        help="Set Symbol sync damping [default=%(default)r]")
    parser.add_argument(
        "--sym-sync-loop-bw", dest="sym_sync_loop_bw", type=eng_float, default=eng_notation.num_to_str(float(0.0045)),
        help="Set Symbol sync loop BW [default=%(default)r]")
    parser.add_argument(
        "--tle", dest="tle", type=str, default='',
        help="Set TLE / OMM file (empty = off) [default=%(default)r]")
    parser.add_argument(
        "--udp-forward", dest="udp_forward", type=str, default='',
        help="Set Also send BBFRAMEs to host:port [default=%(default)r]")
    parser.add_argument(
        "--udp-raw-out", dest="udp_raw_out", type=str, default='',
        help="Set Branch C1: UDP payloads to [default=%(default)r]")
    parser.add_argument(
        "--udp-raw-ref", dest="udp_raw_ref", type=str, default='',
        help="Set Branch C1: reference file [default=%(default)r]")
    parser.add_argument(
        "--verify-hashes", dest="verify_hashes", type=str, default='',
        help="Set sha256sum file to check [default=%(default)r]")
    return parser


def main(top_block_cls=rx_unified, options=None):
    if options is None:
        options = argument_parser().parse_args()

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls(agc=options.agc, agc3_attack=options.agc3_attack, agc3_decay=options.agc3_decay, agc_rate=options.agc_rate, agc_ref=options.agc_ref, antenna=options.antenna, bbframe_file=options.bbframe_file, centre_freq=options.centre_freq, conj=options.conj, debug=options.debug, device_args=options.device_args, dummy_frames=options.dummy_frames, duty_period=options.duty_period, duty_window=options.duty_window, frame_size=options.frame_size, gold_code=options.gold_code, input_file=options.input_file, input_format=options.input_format, ldpc_max_trials=options.ldpc_max_trials, modcod=options.modcod, mode=options.mode, out_dir=options.out_dir, panels=options.panels, pl_freq_est_period=options.pl_freq_est_period, post_agc_attack=options.post_agc_attack, post_agc_decay=options.post_agc_decay, post_agc_ref=options.post_agc_ref, precomp_dt=options.precomp_dt, precomp_offset_hz=options.precomp_offset_hz, precomp_sign=options.precomp_sign, preset=options.preset, realtime=options.realtime, rec_dir=options.rec_dir, rec_prefix=options.rec_prefix, record=options.record, report_json=options.report_json, rf_freq=options.rf_freq, rolloff=options.rolloff, rrc_delay=options.rrc_delay, rrc_nfilts=options.rrc_nfilts, rx_gain=options.rx_gain, sat=options.sat, sim_input=options.sim_input, sps=options.sps, stack=options.stack, start_utc=options.start_utc, station=options.station, sym_rate=options.sym_rate, sym_sync_damping=options.sym_sync_damping, sym_sync_loop_bw=options.sym_sync_loop_bw, tle=options.tle, udp_forward=options.udp_forward, udp_raw_out=options.udp_raw_out, udp_raw_ref=options.udp_raw_ref, verify_hashes=options.verify_hashes)

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
