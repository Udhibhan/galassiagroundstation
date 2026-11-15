#!/usr/bin/env python3
"""Write rx_unified.grc and tx_unified.grc (then compile them with grcc).

The .grc files are the real GUI sources: open and edit them in GNU Radio Companion. This
script only made the first version (and can rebuild it from scratch; it overwrites).
    python3 tools/make_grc.py && grcc -o . rx_unified.grc && grcc -o . tx_unified.grc
Every `parameter` block becomes a command-line option of the generated script
(e.g. --preset linkx2 --mode file --input-file X.sc16); rx_cli.py / tx_cli.py reuse them.
"""
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")


class G:
    def __init__(self, gid, title, description):
        self.gid, self.title, self.desc = gid, title, description
        self.blocks, self.conns = [], []
        self.col = {}

    def add(self, name, bid, params, xy, state="enabled"):
        base = {"alias": "", "comment": "", "affinity": "", "maxoutbuf": "0", "minoutbuf": "0"}
        self.blocks.append({"name": name, "id": bid, "parameters": dict(base, **params),
                            "states": {"bus_sink": False, "bus_source": False,
                                       "bus_structure": None, "coordinate": list(xy),
                                       "rotation": 0, "state": state}})
        return name

    def param(self, name, label, typ, value, column, hide="none"):
        k = self.col.get(column, 0)
        self.col[column] = k + 1
        x, y = 8 + 176 * column, 1000 + 116 * k
        t = {"str": "str", "float": "eng_float", "int": "intx"}[typ]
        self.add(name, "parameter", {"label": label, "short_id": "", "type": t,
                                     "value": str(value), "hide": hide}, (x, y))

    def var(self, name, value, xy):
        self.add(name, "variable", {"value": value}, xy)

    def conn(self, a, b, ap="0", bp="0"):
        self.conns.append([a, str(ap), b, str(bp)])

    def chain(self, *names):
        for a, b in zip(names, names[1:]):
            self.conn(a, b)

    def write(self, path):
        opts = {"parameters": {
            "author": "Galassia-5 ground station", "catch_exceptions": "True",
            "category": "[GRC Hier Blocks]", "cmake_opt": "", "comment": "", "copyright": "",
            "description": self.desc, "gen_cmake": "On", "gen_linking": "dynamic",
            "generate_options": "qt_gui", "hier_block_src_path": ".:", "id": self.gid,
            "max_nouts": "0", "output_language": "python", "placement": "(0,0)",
            "qt_qss_theme": "", "realtime_scheduling": "", "run": "True",
            "run_command": "{python} -u {filename}", "run_options": "prompt",
            "sizing_mode": "fixed", "thread_safe_setters": "", "title": self.title,
            "window_size": "(1500,1000)"},
            "states": {"bus_sink": False, "bus_source": False, "bus_structure": None,
                       "coordinate": [8, 8], "rotation": 0, "state": "enabled"}}
        doc = {"options": opts, "blocks": self.blocks, "connections": self.conns,
               "metadata": {"file_format": 1, "grc_version": "3.10.12.0"}}
        with open(path, "w") as f:
            yaml.safe_dump(doc, f, sort_keys=False, width=100)
        print("wrote", path)


def freq_sink(name, title, bw, fc, hint):
    p = {"type": "complex", "name": f'"{title}"', "fftsize": "1024", "wintype":
         "window.WIN_BLACKMAN_hARRIS", "fc": fc, "bw": bw, "grid": "True", "autoscale": "False",
         "average": "0.2", "axislabels": "True", "freqhalf": "True", "legend": "False",
         "ymin": "-140", "ymax": "10", "units": "dB", "label": "Relative Gain",
         "update_time": "0.10", "nconnections": "1", "showports": "False", "ctrlpanel": "False",
         "norm_window": "False", "tr_mode": "qtgui.TRIG_MODE_FREE", "tr_level": "0.0",
         "tr_chan": "0", "tr_tag": '""', "gui_hint": hint, "label1": "", "width1": "1",
         "color1": '"blue"', "alpha1": "1.0"}
    for i in range(2, 11):
        p.update({f"label{i}": "''", f"width{i}": "1", f"color{i}": '"blue"', f"alpha{i}": "1.0"})
    return p


def const_sink(title, hint):
    p = {"type": "complex", "name": f'"{title}"', "size": "2048", "grid": "True",
         "autoscale": "False", "ymin": "-2", "ymax": "2", "xmin": "-2", "xmax": "2",
         "nconnections": "1", "update_time": "0.20", "gui_hint": hint,
         "tr_mode": "qtgui.TRIG_MODE_FREE", "tr_slope": "qtgui.TRIG_MODE_POS", "tr_level": "0.0",
         "tr_chan": "0", "tr_tag": '""', "legend": "False", "axislabels": "True"}
    for i in range(1, 11):
        p.update({f"label{i}": "''", f"width{i}": "1", f"color{i}": '"blue"',
                  f"style{i}": "0", f"marker{i}": "0", f"alpha{i}": "1.0"})
    return p


# ============================================================================ RX
def rx():
    g = G("rx_unified", "G5 unified RX",
          "Galassia-5 ground-station receiver: B205mini record + monitor / live decode / file "
          "decode, patched gr-dvbs2rx, optional TLE Doppler pre-comp, GSSE/GSUFTP files. "
          "Parameters = command-line options (rx_unified.py --help). README.md.")
    P = g.param
    # column 0: what to receive
    P("preset", "Preset (ops20 / linkx2 / field1)", "str", "ops20", 0)
    P("mode", "Mode (record / live / file)", "str", "record", 0)
    P("sym_rate", "Symbol rate (0 = preset)", "float", "0", 0)
    P("sps", "Samples/symbol (0 = preset)", "int", "0", 0)
    P("modcod", "MODCOD (empty = preset)", "str", "", 0)
    P("frame_size", "Frame size (empty = preset)", "str", "", 0)
    P("dummy_frames", "Dummy frames (1 = on)", "int", "1", 0)
    P("panels", "Panels at start (all / none / list)", "str", "all", 0)
    # column 1: input / radio
    P("input_file", "Input file (file mode)", "str", "", 1)
    P("input_format", "Input format (auto / sc16 / fc32)", "str", "auto", 1)
    P("realtime", "File in real time (1 = yes)", "int", "0", 1)
    P("start_utc", "First-sample UTC (file, override)", "str", "", 1)
    P("sim_input", "Rehearsal sc16 instead of radio", "str", "", 1)
    P("centre_freq", "Centre frequency", "float", "1195e6", 1)
    P("rx_gain", "RX gain (dB)", "float", "30", 1)
    P("device_args", "UHD device args", "str", "", 1)
    P("conj", "Conjugate input (1 = yes)", "int", "0", 1)
    P("antenna", "RX port (RX2 / TX/RX)", "str", "RX2", 1)
    # column 2: recording + outputs
    P("rec_dir", "Recording folder", "str", "~/iq/recordings", 2)
    P("rec_prefix", "Recording prefix", "str", "g5", 2)
    P("record", "Record (-1 = auto, 0, 1)", "int", "-1", 2)
    P("out_dir", "Decoded files folder (empty = auto)", "str", "", 2)
    P("stack", "GSSE stack (auto / rev1.0 / rev1.2)", "str", "auto", 2)
    P("verify_hashes", "sha256sum file to check", "str", "", 2)
    P("bbframe_file", "Also write BBFRAMEs to", "str", "", 2)
    P("udp_forward", "Also send BBFRAMEs to host:port", "str", "", 2)
    P("udp_raw_out", "Branch C1: UDP payloads to", "str", "", 2)
    P("udp_raw_ref", "Branch C1: reference file", "str", "", 2)
    P("report_json", "JSON report", "str", "", 2)
    # column 3: Doppler + gate
    P("tle", "TLE / OMM file (empty = off)", "str", "", 3)
    P("sat", "Satellite (if several sets)", "str", "", 3)
    P("station", "Station lat,lon,h", "str", "1.2977,103.7764,60", 3)
    P("rf_freq", "Downlink RF (Hz)", "float", "13.95e9", 3)
    P("precomp_sign", "Pre-comp sign (+1 / -1)", "float", "1", 3)
    P("precomp_dt", "Pre-comp time shift (s)", "float", "0", 3)
    P("precomp_offset_hz", "Pre-comp extra offset (Hz)", "float", "0", 3)
    P("duty_window", "Monitor window (s)", "float", "1.0", 3)
    P("duty_period", "Monitor period (s, 0 = auto)", "float", "0", 3)
    # column 4-5: demod tuning (validated defaults)
    P("agc", "Front AGC (agc3 / agc)", "str", "agc3", 4)
    P("agc_ref", "Front AGC reference", "float", "1.37", 4)
    P("agc3_attack", "AGC3 attack", "float", "1e-2", 4)
    P("agc3_decay", "AGC3 decay", "float", "1e-4", 4)
    P("agc_rate", "agc_cc rate", "float", "1e-5", 4)
    P("post_agc_ref", "Post-AGC ref (0 = off)", "float", "1.0", 4)
    P("post_agc_attack", "Post-AGC attack / symbol", "float", "1e-3", 4)
    P("post_agc_decay", "Post-AGC decay / symbol", "float", "1e-5", 4)
    P("pl_freq_est_period", "PL freq. est. period (prime)", "int", "17", 5)
    P("sym_sync_loop_bw", "Symbol sync loop BW", "float", "0.0045", 5)
    P("sym_sync_damping", "Symbol sync damping", "float", "1.0", 5)
    P("rolloff", "Roll-off", "float", "0.2", 5)
    P("rrc_delay", "RRC delay", "int", "25", 5)
    P("rrc_nfilts", "Polyphase RRC branches", "int", "128", 5)
    P("ldpc_max_trials", "LDPC max iterations", "int", "25", 5)
    P("gold_code", "Gold code", "int", "0", 5)
    P("debug", "Debug level", "int", "0", 5)

    g.add("import_0", "import", {"imports": "import g5gs\nfrom gnuradio import dvbs2rx"},
          (200, 8))
    V = g.var
    V("cfg", "g5gs.config.resolve(preset, sym_rate, sps, modcod, frame_size)", (330, 8))
    V("samp_rate", "cfg.samp_rate", (450, 8))
    V("rs", "cfg.sym_rate", (560, 8))
    V("gate_cfg", "g5gs.auto_mode(mode, samp_rate, duty_period)", (660, 8))
    V("pls_filter", "dvbs2rx.pls_filter(dvbs2rx.dvbs2_pls(cfg.constellation, cfg.code_rate, "
      "cfg.frame_size, True))", (800, 8))
    V("out_dir_eff", "out_dir or g5gs.config.default_out_dir(mode, input_file, rec_dir)",
      (330, 80))
    # dict literals, not dict(name=...): GRC's callback generator rewrites "name=" to "self.name="
    V("meta", "{'preset': preset, 'modcod': cfg.modcod, 'frame_size': cfg.frame_size, "
      "'sym_rate': rs, 'sps': cfg.sps, 'tle': tle, 'conj': bool(conj)}", (480, 80))
    V("cfg_text", "f'{mode}: {cfg.describe()}' + (f', TLE {tle}' if tle else ', no TLE') + "
      "f', AGC {agc} {agc_ref:g}, post-AGC {post_agc_ref:g}, period {pl_freq_est_period}' + "
      "(', dummy frames' if dummy_frames else '')", (660, 80))
    V("panel_groups", "{'spectrum': ['valve_spec_in', 'valve_spec_pl', 'qtgui_freq_sink_in', "
      "'qtgui_freq_sink_pl'], 'constellation': ['valve_const', 'qtgui_const_sink_pl'], "
      "'status': ['rx_status_panel_0'], 'doppler': ['doppler_panel_0'], "
      "'files': ['files_panel_0']}", (860, 80))
    g.add("rx_gain_slider", "variable_qtgui_range",
          {"label": "RX gain (dB)", "rangeType": "float", "value": "rx_gain", "start": "0",
           "stop": "76", "step": "1", "widget": "counter_slider", "orient": "QtCore.Qt.Horizontal",
           "min_len": "200", "gui_hint": "1,0,1,1"}, (1060, 8))
    g.add("antenna_sel", "variable_qtgui_chooser",
          {"label": "RX port", "type": "string", "num_opts": "2", "options": "[0, 1, 2]",
           "labels": "[]", "value": "g5gs.config.antenna_name(antenna)",
           "option0": "'RX2'", "label0": "RX2", "option1": "'TX/RX'", "label1": "TX/RX",
           "option2": "2", "label2": "", "option3": "3", "label3": "", "option4": "4",
           "label4": "", "widget": "combo_box", "orient": "Qt.QVBoxLayout",
           "gui_hint": "0,1,1,1"}, (1060, 200))
    g.add("record_on", "variable_qtgui_check_box",
          {"label": "Record to disk", "type": "int",
           "value": "int(mode == 'record') if record < 0 else int(bool(record))",
           "true": "1", "false": "0", "gui_hint": "1,1,1,1"}, (1060, 100))

    # GUI layout
    g.add("panel_switch_0", "g5gs_panel_switch", {"groups": "panel_groups", "panels": "panels",
                                                  "main_row": "2", "gui_hint": "0,0,1,1"},
          (1240, 8))
    g.add("tabs", "qtgui_tab_widget", {"num_tabs": "4", "label0": "Spectrum",
                                       "label1": "Constellation", "label2": "Doppler / pass",
                                       "label3": "Files", "gui_hint": "2,1,1,1"}, (1240, 100))
    g.add("rx_status_panel_0", "g5gs_rx_status_panel",
          {"sym_rate": "rs", "cfg_text": "cfg_text", "dummy_frames": "bool(dummy_frames)",
           "active": "True", "gui_hint": "2,0,1,1"}, (1240, 200))
    g.add("doppler_panel_0", "g5gs_doppler_panel",
          {"sym_rate": "rs", "history_s": "900", "active": "True",
           "gui_hint": "tabs@2: 0,0,1,1"}, (1240, 300))
    g.add("files_panel_0", "g5gs_files_panel", {"active": "True",
                                                "gui_hint": "tabs@3: 0,0,1,1"}, (1240, 400))

    # signal chain
    g.add("rx_source_0", "g5gs_rx_source",
          {"mode": "mode", "samp_rate": "samp_rate", "sym_rate": "rs", "input_file": "input_file",
           "input_format": "input_format", "realtime": "bool(realtime)", "need_time": "bool(tle)",
           "start_utc": "start_utc", "centre_freq": "centre_freq", "rx_gain": "rx_gain_slider",
           "device_args": "device_args", "lo_offset": "-1", "record": "bool(record_on)",
           "rec_dir": "rec_dir", "rec_prefix": "rec_prefix", "min_free": "2 * 2**30",
           "meta": "meta", "conj": "bool(conj)", "sim_input": "sim_input",
           "antenna": "antenna_sel"}, (8, 200))
    g.add("duty_gate_0", "g5gs_duty_gate",
          {"mode": "gate_cfg[0]", "samp_rate": "samp_rate", "window": "duty_window",
           "period": "gate_cfg[1] or 5.0", "queue_s": "0.5"}, (230, 200))
    g.add("doppler_precomp_0", "g5gs_doppler_precomp",
          {"tle": "tle", "sat": "sat", "samp_rate": "samp_rate", "station": "station",
           "rf_freq": "rf_freq", "sign": "precomp_sign", "dt": "precomp_dt",
           "offset_hz": "precomp_offset_hz", "start_utc": "start_utc"}, (420, 200))
    g.add("front_agc_0", "g5gs_front_agc",
          {"kind": "agc", "ref": "agc_ref", "attack": "agc3_attack", "decay": "agc3_decay",
           "rate": "agc_rate", "gain": "1.0", "max_gain": "65536"}, (640, 200))
    g.add("dvbs2rx_rotator_cc_0", "dvbs2rx_rotator_cc",
          {"phase_inc": "0", "tag_inc_update": "True"}, (820, 200))
    g.add("dvbs2rx_symbol_sync_cc_0", "dvbs2rx_symbol_sync_cc",
          {"sps": "cfg.sps", "loop_bw": "sym_sync_loop_bw", "damping_factor": "sym_sync_damping",
           "rolloff": "rolloff", "rrc_delay": "rrc_delay", "n_subfilt": "rrc_nfilts",
           "interp_method": "INTERP_POLYPHASE"}, (1000, 200))
    g.add("post_agc_0", "g5gs_post_agc", {"ref": "post_agc_ref", "attack": "post_agc_attack",
                                          "decay": "post_agc_decay"}, (8, 420))
    g.add("dvbs2rx_plsync_cc_0", "dvbs2rx_plsync_cc",
          {"gold_code": "gold_code", "freq_est_period": "pl_freq_est_period", "sps": "cfg.sps",
           "debug_level": "debug", "acm_vcm": "False", "multistream": "bool(dummy_frames)",
           "pls_filter_lo": "pls_filter[0]", "pls_filter_hi": "pls_filter[1]"}, (200, 420))
    g.add("dvbs2rx_xfecframe_demapper_cb_0", "dvbs2rx_xfecframe_demapper_cb",
          {"constellation": "cfg.constellation", "framesize": "cfg.frame_size",
           "rate": "cfg.code_rate"}, (440, 420))
    g.add("dvbs2rx_ldpc_decoder_bb_0", "dvbs2rx_ldpc_decoder_bb",
          {"standard": "DVB-S2", "framesize": "cfg.frame_size", "rate": "cfg.code_rate",
           "constellation": "cfg.constellation", "outputmode": "OM_MESSAGE",
           "infomode": "INFO_OFF", "max_trials": "ldpc_max_trials", "debug_level": "debug"},
          (660, 420))
    g.add("dvbs2rx_bch_decoder_bb_0", "dvbs2rx_bch_decoder_bb",
          {"standard": "DVB-S2", "framesize": "cfg.frame_size", "rate": "cfg.code_rate",
           "outputmode": "OM_MESSAGE", "debug_level": "debug"}, (880, 420))
    g.add("dvbs2rx_bbdescrambler_bb_0", "dvbs2rx_bbdescrambler_bb",
          {"standard": "DVB-S2", "framesize": "cfg.frame_size", "rate": "cfg.code_rate"},
          (1080, 420))
    g.add("gs_decoder_0", "g5gs_gs_decoder",
          {"frame_len": "cfg.kbch_bytes", "stack": "stack", "out_dir": "out_dir_eff",
           "verify_hashes": "verify_hashes", "bbframe_file": "bbframe_file",
           "udp_forward": "udp_forward", "udp_raw_out": "udp_raw_out",
           "udp_raw_ref": "udp_raw_ref", "report_json": "report_json", "dump_payloads": "''",
           "isi": "-1", "udp_port": "0"}, (1080, 560))
    g.chain("rx_source_0", "duty_gate_0", "doppler_precomp_0", "front_agc_0",
            "dvbs2rx_rotator_cc_0", "dvbs2rx_symbol_sync_cc_0", "post_agc_0",
            "dvbs2rx_plsync_cc_0", "dvbs2rx_xfecframe_demapper_cb_0",
            "dvbs2rx_ldpc_decoder_bb_0", "dvbs2rx_bch_decoder_bb_0",
            "dvbs2rx_bbdescrambler_bb_0", "gs_decoder_0")
    g.conn("dvbs2rx_ldpc_decoder_bb_0", "dvbs2rx_xfecframe_demapper_cb_0", "llr_pdu", "llr_pdu")
    g.conn("dvbs2rx_plsync_cc_0", "dvbs2rx_rotator_cc_0", "rotator_phase_inc", "cmd")

    # panels fed through valves (Copy blocks): switched off = no CPU
    def valve(name, xy):
        g.add(name, "blocks_copy", {"type": "complex", "enabled": "True", "vlen": "1",
                                    "showports": "False"}, xy)
    valve("valve_spec_in", (230, 300))
    g.add("qtgui_freq_sink_in", "qtgui_freq_sink_x",
          freq_sink("qtgui_freq_sink_in", "Input (before AGC)", "samp_rate", "centre_freq",
                    "tabs@0: 0,0,1,1"), (420, 300))
    valve("valve_spec_pl", (420, 560))
    g.add("qtgui_freq_sink_pl", "qtgui_freq_sink_x",
          freq_sink("qtgui_freq_sink_pl", "After PL sync (1 sample/symbol)", "rs", "0",
                    "tabs@0: 1,0,1,1"), (600, 560))
    valve("valve_const", (420, 660))
    g.add("qtgui_const_sink_pl", "qtgui_const_sink_x",
          const_sink("PL sync output (data, pilots, headers)", "tabs@1: 0,0,1,1"), (600, 660))
    g.chain("rx_source_0", "valve_spec_in", "qtgui_freq_sink_in")
    g.chain("dvbs2rx_plsync_cc_0", "valve_spec_pl", "qtgui_freq_sink_pl")
    g.chain("dvbs2rx_plsync_cc_0", "valve_const", "qtgui_const_sink_pl")
    g.write(os.path.join(ROOT, "rx_unified.grc"))


# ============================================================================ TX
def tx():
    g = G("tx_unified", "G5 unified TX",
          "Galassia-5 test transmitter: Link X/SX-style GSSE/GSUFTP (or gr-dvbgse GSE from tap0, "
          "or a BBFRAME file) -> DVB-S2 (gr-dtv) + dummy frames -> transients / CFO / TLE "
          "Doppler / AWGN -> B205mini or IQ file. README.md.")
    P = g.param
    P("preset", "Preset (ops20 / linkx2 / field1)", "str", "ops20", 0)
    P("sym_rate", "Symbol rate (0 = preset)", "float", "0", 0)
    P("sps", "Samples/symbol (0 = preset)", "int", "0", 0)
    P("modcod", "MODCOD (empty = preset)", "str", "", 0)
    P("frame_size", "Frame size (empty = preset)", "str", "", 0)
    P("panels", "Panels at start (all / none / list)", "str", "all", 0)
    P("payload", "Payload (gsse / bbframes / gse_tap0)", "str", "gsse", 1)
    P("files", "File or folder to send", "str", "", 1)
    P("stack", "Stack (rev1.0 / rev1.2 / udpraw)", "str", "rev1.0", 1)
    P("chunk_size", "GSUFTP chunk (0 = default)", "int", "0", 1)
    P("repeat", "Passes (0 = loop forever)", "int", "0", 1)
    P("flush", "Empty BBFRAMEs at the end", "int", "100", 1)
    P("bbframe_in", "BBFRAME file (bbframes payload)", "str", "", 1)
    P("lead_dummies", "Dummy PLFRAMEs at the start", "int", "200", 1)
    P("gap_dummies", "Dummy PLFRAMEs after each frame", "int", "0", 1)
    P("source", "Source (generate / replay)", "str", "generate", 2)
    P("replay_file", "Replay IQ file", "str", "", 2)
    P("output", "Output (b205 / file)", "str", "b205", 2)
    P("output_file", "Output IQ file", "str", "", 2)
    P("duration", "File: seconds (0 = payload end)", "float", "0", 2)
    P("realtime", "File: real time (1 = yes)", "int", "0", 2)
    P("centre_freq", "Centre frequency", "float", "1195e6", 2)
    P("tx_gain", "TX gain (dB)", "float", "0", 2)
    P("device_args", "UHD device args", "str", "", 2)
    P("level_db", "Level (dB)", "float", "0", 3)
    P("rms_dbfs", "Output RMS at 0 dB (dBFS)", "float", "-12", 3)
    P("cfo_hz", "CFO (Hz)", "float", "0", 3)
    P("esn0_db", "Es/N0 dB (empty = no noise)", "str", "", 3)
    P("schedule", "Transients (5:off,6:on,10:-20dB)", "str", "", 3)
    P("tle", "Doppler TLE (empty = off)", "str", "", 3)
    P("sat", "Satellite (if several sets)", "str", "", 3)
    P("doppler_start_utc", "Doppler start UTC (empty = now)", "str", "", 3)
    P("doppler_dt", "Doppler time shift (s)", "float", "0", 3)
    P("station", "Station lat,lon,h", "str", "1.2977,103.7764,60", 3)
    P("rf_freq", "Downlink RF (Hz)", "float", "13.95e9", 3)

    g.add("import_0", "import", {"imports": "import g5gs"}, (200, 8))
    V = g.var
    V("cfg", "g5gs.config.resolve(preset, sym_rate, sps, modcod, frame_size)", (330, 8))
    V("samp_rate", "cfg.samp_rate", (450, 8))
    V("rs", "cfg.sym_rate", (560, 8))
    V("cfg_text", "f'{cfg.describe()}, payload {payload}' + (f' {files}' if files else '') + "
      "f', lead {lead_dummies} / gap {gap_dummies} dummies' + (f', TLE {tle}' if tle else '')",
      (660, 8))
    V("tx_ids", "{'payload': 'tx_payload_0', 'framer': 'dvbs2_tx_0', 'imp': 'tx_impairments_0', "
      "'sink': 'tx_sink_0'}", (860, 8))
    V("panel_groups", "{'spectrum': ['valve_spec', 'qtgui_freq_sink_tx'], "
      "'status': ['tx_status_panel_0']}", (860, 80))
    V("sig_power", "g5gs.tx.signal_power(source, replay_file, cfg.sps)", (330, 80))
    V("dop_start", "g5gs.tx.doppler_start(tle, doppler_start_utc)", (480, 80))
    g.add("tx_gain_slider", "variable_qtgui_range",
          {"label": "TX gain (dB, B205mini)", "rangeType": "float", "value": "tx_gain",
           "start": "0", "stop": "89", "step": "1", "widget": "counter_slider",
           "orient": "QtCore.Qt.Horizontal", "min_len": "200", "gui_hint": "1,0,1,1"},
          (1060, 8))
    g.add("level_slider", "variable_qtgui_range",
          {"label": "Level (dB, digital: fades / steps)", "rangeType": "float",
           "value": "level_db", "start": "-60", "stop": "10", "step": "1",
           "widget": "counter_slider", "orient": "QtCore.Qt.Horizontal", "min_len": "200",
           "gui_hint": "1,1,1,1"}, (1060, 100))
    g.add("cfo_slider", "variable_qtgui_range",
          {"label": "CFO (Hz)", "rangeType": "float", "value": "cfo_hz",
           "start": "-0.2 * rs", "stop": "0.2 * rs", "step": "100", "widget": "counter_slider",
           "orient": "QtCore.Qt.Horizontal", "min_len": "200", "gui_hint": "2,0,1,1"},
          (1060, 200))
    g.add("carrier_on", "variable_qtgui_check_box",
          {"label": "Carrier on", "type": "int", "value": "1", "true": "1", "false": "0",
           "gui_hint": "2,1,1,1"}, (1060, 300))
    g.add("panel_switch_0", "g5gs_panel_switch", {"groups": "panel_groups", "panels": "panels",
                                                  "main_row": "3", "gui_hint": "0,0,1,2"},
          (1240, 8))
    g.add("tx_status_panel_0", "g5gs_tx_status_panel",
          {"ids": "tx_ids", "cfg_text": "cfg_text", "active": "True", "gui_hint": "3,0,1,1"},
          (1240, 100))

    g.add("tx_payload_0", "g5gs_tx_payload",
          {"mode": "payload", "files": "files", "stack": "stack", "chunk_size": "chunk_size",
           "repeat": "repeat", "flush": "flush", "bbframe_file": "bbframe_in",
           "frame_len": "cfg.kbch_bytes", "modcod": "cfg.modcod", "frame_size": "cfg.frame_size",
           "isi": "0", "sis": "False", "enabled": "source == 'generate'",
           "gse_src_ip": "44.0.0.3", "gse_dst_ip": "10.0.1.190"}, (8, 200))
    g.add("dvbs2_tx_0", "g5gs_dvbs2_tx",
          {"modcod": "cfg.modcod", "frame_size": "cfg.frame_size", "sps": "cfg.sps",
           "rolloff": "0.2", "lead": "lead_dummies", "gap": "gap_dummies", "gold_code": "0"},
          (230, 200))
    g.add("iq_select_0", "g5gs_iq_select",
          {"mode": "source", "replay_file": "replay_file", "replay_format": "auto",
           "replay_loop": "True"}, (460, 200))
    g.add("tx_impairments_0", "g5gs_tx_impairments",
          {"samp_rate": "samp_rate", "signal_power": "sig_power", "rms_dbfs": "rms_dbfs", "level_db": "level_slider",
           "cfo_hz": "cfo_slider", "esn0_db": "esn0_db", "sps": "cfg.sps",
           "schedule": "schedule", "tle": "tle", "sat": "sat", "station": "station",
           "rf_freq": "rf_freq", "doppler_start_utc": "dop_start if dop_start is not None else ''",
           "doppler_dt": "doppler_dt", "seed": "0", "carrier": "bool(carrier_on)"}, (660, 200))
    g.add("tx_sink_0", "g5gs_tx_sink",
          {"mode": "output", "samp_rate": "samp_rate", "sym_rate": "rs", "rolloff": "0.2",
           "centre_freq": "centre_freq", "tx_gain": "tx_gain_slider",
           "device_args": "device_args", "lo_offset": "-1", "output_file": "output_file",
           "output_format": "auto", "duration": "duration",
           "throttle": "bool(realtime) or payload == 'gse_tap0'",
           "start_unix": "dop_start",
           "meta": "{'preset': preset, 'modcod': cfg.modcod, 'sym_rate': rs, 'sps': cfg.sps, "
                   "'payload': payload, 'files': files, 'esn0_db': esn0_db, 'cfo_hz': cfo_hz, "
                   "'tle': tle}"},
          (880, 200))
    g.chain("tx_payload_0", "dvbs2_tx_0", "iq_select_0", "tx_impairments_0", "tx_sink_0")
    g.add("valve_spec", "blocks_copy", {"type": "complex", "enabled": "True", "vlen": "1",
                                        "showports": "False"}, (880, 320))
    g.add("qtgui_freq_sink_tx", "qtgui_freq_sink_x",
          freq_sink("qtgui_freq_sink_tx", "TX spectrum", "samp_rate", "centre_freq", "3,1,1,1"),
          (1060, 500))
    g.chain("tx_impairments_0", "valve_spec", "qtgui_freq_sink_tx")
    g.write(os.path.join(ROOT, "tx_unified.grc"))


if __name__ == "__main__":
    rx()
    tx()
