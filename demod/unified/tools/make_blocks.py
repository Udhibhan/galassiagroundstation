#!/usr/bin/env python3
"""Write the GRC block definitions (../grc/g5gs_*.block.yml) for the g5gs package.

The blocks are thin: each one calls a class in g5gs/. Rerun after changing a parameter list:
    python3 tools/make_blocks.py
"""
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "grc")
CAT = "[G5 Ground Station]"


def P(pid, label, dtype, default, hide="none", category=None, options=None, labels=None):
    d = {"id": pid, "label": label, "dtype": dtype, "default": default}
    if hide != "none":
        d["hide"] = hide
    if category:
        d["category"] = category
    if options:
        d["options"] = options
        if labels:
            d["option_labels"] = labels
    return d


GUI_HINT = {"id": "gui_hint", "label": "GUI Hint", "dtype": "gui_hint", "hide": "part"}


def args(params, skip=("gui_hint",)):
    return ", ".join(f"{p['id']}=${{{p['id']}}}" for p in params if p["id"] not in skip)


def stream(dtype, optional=False):
    d = {"domain": "stream", "dtype": dtype}
    if optional:
        d["optional"] = True
    return [d]


def block(bid, label, params, make, inputs=None, outputs=None, callbacks=None, doc="",
          widget=False):
    if widget:
        make = ("<%\n    win = 'self._%s_win'%id\n%>\\\n" + make +
                "\n${win} = self.${id}\n${gui_hint() % win}")
        params = params + [GUI_HINT]
    y = {"id": bid, "label": label, "category": CAT, "flags": ["python"],
         "parameters": params}
    if inputs:
        y["inputs"] = inputs
    if outputs:
        y["outputs"] = outputs
    y["templates"] = {"imports": "import g5gs", "make": make}
    if callbacks:
        y["templates"]["callbacks"] = callbacks
    y["documentation"] = doc.strip()
    y["file_format"] = 1
    return y


B = []

# ---------------------------------------------------------------------------- RX
p = [P("mode", "Mode (record / live / file)", "string", "record"),
     P("samp_rate", "Sample rate", "real", "40e6"),
     P("sym_rate", "Symbol rate", "real", "20e6"),
     P("input_file", "Input file (file mode)", "file_open", ""),
     P("input_format", "Input format (auto / sc16 / fc32)", "string", "auto"),
     P("realtime", "File: real time", "bool", "False", "part"),
     P("need_time", "File: put rx_time tags back", "bool", "False", "part"),
     P("start_utc", "File: first-sample UTC override", "string", "", "part"),
     P("centre_freq", "Centre frequency", "real", "1195e6", category="Radio"),
     P("rx_gain", "RX gain (dB)", "real", "30", category="Radio"),
     P("device_args", "UHD device args", "string", "", category="Radio"),
     P("lo_offset", "LO offset (-1 = auto)", "real", "-1", "part", category="Radio"),
     P("record", "Recording on", "bool", "True", category="Recording"),
     P("rec_dir", "Recording folder", "string", ".", category="Recording"),
     P("rec_prefix", "Recording file prefix", "string", "rx", category="Recording"),
     P("min_free", "Stop when this little is free (B)", "real", "2 * 2**30", "part",
       category="Recording"),
     P("meta", "Extra JSON metadata (dict)", "raw", "{}", "part", category="Recording"),
     P("conj", "Conjugate (inverted spectrum)", "bool", "False"),
     P("sim_input", "Rehearsal: sc16 file instead of the radio", "string", "", "part"),
     P("antenna", "RX port (RX2 / TX/RX)", "string", "RX2", category="Radio")]
B.append(block("g5gs_rx_source", "G5 RX Source", p, "g5gs.rx_source(" + args(p) + ")",
               outputs=stream("complex"),
               callbacks=["set_rx_gain(${rx_gain})", "set_recording(${record})",
                          "set_antenna(${antenna})"], doc="""
B205mini or IQ file input (g5gs/rx_source.py).
record: full-rate sc16 recording (with .start / .json sidecars, stops at 2 GB free) + complex
        output for monitoring.  live: the same, recording off by default.
file:   sc16 / fc32 file, as fast as the CPU allows (or real time).
Rehearsal: an sc16 file looped in real time instead of the radio."""))

p = [P("mode", "Mode (pass / drop / duty)", "string", "pass"),
     P("samp_rate", "Sample rate", "real", "40e6"),
     P("window", "Window (s)", "real", "1.0"),
     P("period", "Period (s)", "real", "5.0"),
     P("queue_s", "Queue (s)", "real", "0.5", "part")]
B.append(block("g5gs_duty_gate", "G5 Duty Gate", p, "g5gs.duty_gate(" + args(p) + ")",
               inputs=stream("complex"), outputs=stream("complex"), doc="""
Decouples the demod from the full-rate stream (g5gs/gate.py). pass: direct (file mode).
drop: everything while the demod keeps up, whole chunks dropped when it falls behind.
duty: window s out of every period s (record + monitor at 40 MS/s)."""))

p = [P("tle", "TLE / OMM file (empty = off)", "file_open", ""),
     P("sat", "Satellite (name / NORAD, if several)", "string", ""),
     P("samp_rate", "Sample rate", "real", "40e6"),
     P("station", "Station lat,lon,h", "string", "1.2977,103.7764,60"),
     P("rf_freq", "Downlink RF (Hz)", "real", "13.95e9"),
     P("sign", "Sign (+1, or -1 if inverted and not conjugated)", "real", "1.0", "part"),
     P("dt", "Time shift dt (s)", "real", "0.0", "part"),
     P("offset_hz", "Extra fixed offset (Hz)", "real", "0.0", "part"),
     P("start_utc", "Start UTC if no rx_time tag", "string", "", "part")]
B.append(block("g5gs_doppler_precomp", "G5 Doppler Pre-comp (TLE)", p,
               "g5gs.doppler_precomp(" + args(p) + ")", inputs=stream("complex"),
               outputs=stream("complex"), doc="""
Removes the SGP4-predicted carrier Doppler, sample by sample (g5gs/doppler_block.py,
field_test/doppler_precomp.py). Empty TLE = pass-through. Time from rx_time tags."""))

p = [P("kind", "AGC (agc3 / agc)", "string", "agc3"),
     P("ref", "Reference", "real", "1.37"),
     P("attack", "AGC3 attack", "real", "1e-2"),
     P("decay", "AGC3 decay", "real", "1e-4"),
     P("rate", "agc_cc rate", "real", "1e-5"),
     P("gain", "Initial gain", "real", "1.0", "part"),
     P("max_gain", "Max gain", "real", "65536", "part")]
B.append(block("g5gs_front_agc", "G5 Front AGC", p, "g5gs.front_agc(" + args(p) + ")",
               inputs=stream("complex"), outputs=stream("complex"), doc="""
AGC3 (default, ref 1.37: 03102026_agc_findings.md) or the original agc_cc."""))

p = [P("ref", "Reference (0 = off)", "real", "1.0"),
     P("attack", "Attack per symbol", "real", "1e-3"),
     P("decay", "Decay per symbol", "real", "1e-5")]
B.append(block("g5gs_post_agc", "G5 Post-AGC", p, "g5gs.post_agc(" + args(p) + ")",
               inputs=stream("complex"), outputs=stream("complex"), doc="""
agc2_cc between symbol sync and PL sync, holding the PL sync input at ~1
(19102026_postagc_results.md). ref 0 = bypass."""))

p = [P("frame_len", "BBFRAME length (B)", "int", "2001"),
     P("stack", "Stack (auto / rev1.0 / rev1.2)", "string", "auto"),
     P("out_dir", "Output folder", "string", "./downlink"),
     P("verify_hashes", "sha256sum file to check", "string", ""),
     P("bbframe_file", "Also write BBFRAMEs to", "string", ""),
     P("udp_forward", "Also send BBFRAMEs to host:port", "string", ""),
     P("udp_raw_out", "Branch C1: UDP payloads to", "string", "", "part"),
     P("udp_raw_ref", "Branch C1: reference file", "string", "", "part"),
     P("report_json", "JSON report", "string", "", "part"),
     P("dump_payloads", "Dump payloads to folder", "string", "", "part"),
     P("isi", "Only ISI (-1 = any)", "int", "-1", "part"),
     P("udp_port", "rev1.2 UDP port filter (0 = any)", "int", "0", "part")]
B.append(block("g5gs_gs_decoder", "G5 GSSE/GSUFTP Decoder", p, "g5gs.gs_decoder(" + args(p) + ")",
               inputs=stream("byte"), doc="""
gs_rx v2.2 inside the flowgraph (g5gs/gs_decoder.py): BBFRAMEs -> GSSE -> GSUFTP files."""))

p = [P("sym_rate", "Symbol rate", "real", "20e6"),
     P("cfg_text", "Settings text", "string", ""),
     P("dummy_frames", "Dummy frames on", "bool", "True"),
     P("active", "Active at start", "bool", "True", "part")]
B.append(block("g5gs_rx_status_panel", "G5 RX Status Panel", p,
               "g5gs.RxStatusPanel(self, " + args(p) + ")", widget=True, doc="""
Input / recording / disk / level / overflow / duty gate / lock / CFO / SNR / frames / BCH /
GSSE. Finds the blocks by their default GRC ids (g5gs/probe.py IDS)."""))

p = [P("sym_rate", "Symbol rate", "real", "20e6"),
     P("history_s", "Plot history (s)", "real", "600"),
     P("active", "Active at start", "bool", "True", "part")]
B.append(block("g5gs_doppler_panel", "G5 Doppler / Pass Panel", p,
               "g5gs.DopplerPanel(self, " + args(p) + ")", widget=True, doc="""
TLE, sample time, elevation / azimuth / range, predicted Doppler vs residual CFO, passes."""))

p = [P("active", "Active at start", "bool", "True", "part")]
B.append(block("g5gs_files_panel", "G5 Files Panel", p, "g5gs.FilesPanel(self, " + args(p) + ")",
               widget=True, doc="GSSE / GSUFTP counters and the received files with hash checks."))

p = [P("groups", "Groups {name: [block ids]}", "raw", "{}"),
     P("panels", "Panels on at start (all / none / list)", "string", "all"),
     P("main_row", "First grid row that may stretch", "int", "2", "part")]
B.append(block("g5gs_panel_switch", "G5 Panel Switch", p,
               "g5gs.PanelSwitch(self, " + args(p) + ")", widget=True, doc="""
One checkbox per group: unticking hides the widgets, pauses the panels and disables the
valves (Copy blocks) in front of the Qt sinks, so the panel costs no CPU."""))

# ---------------------------------------------------------------------------- TX
p = [P("mode", "Payload (gsse / bbframes / gse_tap0)", "string", "gsse"),
     P("files", "File or folder to send", "string", ""),
     P("stack", "Stack (rev1.0 / rev1.2 / udpraw)", "string", "rev1.0"),
     P("chunk_size", "GSUFTP chunk (0 = default)", "int", "0"),
     P("repeat", "Passes (0 = loop forever)", "int", "0"),
     P("flush", "Empty BBFRAMEs at the end", "int", "100"),
     P("bbframe_file", "BBFRAME file (bbframes mode)", "string", ""),
     P("frame_len", "BBFRAME length (B)", "int", "2001"),
     P("modcod", "MODCOD (gse_tap0)", "string", "QPSK1/4", "part"),
     P("frame_size", "Frame size (gse_tap0)", "string", "normal", "part"),
     P("isi", "ISI", "int", "0", "part"),
     P("sis", "Single input stream", "bool", "False", "part"),
     P("enabled", "Enabled (False when replaying)", "bool", "True", "part"),
     P("gse_src_ip", "gse_tap0 source IP", "string", "44.0.0.3", "part"),
     P("gse_dst_ip", "gse_tap0 destination IP", "string", "10.0.1.190", "part")]
B.append(block("g5gs_tx_payload", "G5 TX Payload", p, "g5gs.tx_payload(" + args(p) + ")",
               outputs=stream("byte"), doc="""
Bits for the DVB-S2 encoder: files as Link X/SX GSSE (+GSUFTP), a BBFRAME file, or the old
gr-dvbgse standard GSE from tap0 (g5gs/tx.py)."""))

p = [P("modcod", "MODCOD", "string", "QPSK1/4"),
     P("frame_size", "Frame size", "string", "normal"),
     P("sps", "Samples per symbol", "int", "2"),
     P("rolloff", "Roll-off", "real", "0.2"),
     P("lead", "Dummy PLFRAMEs before the data", "int", "200"),
     P("gap", "Dummy PLFRAMEs after each data frame", "int", "0"),
     P("gold_code", "Gold code", "int", "0", "part")]
B.append(block("g5gs_dvbs2_tx", "G5 DVB-S2 TX (gr-dtv + dummies + RRC)", p,
               "g5gs.dvbs2_tx(" + args(p) + ")", inputs=stream("byte"),
               outputs=stream("complex"), doc="""
gr-dtv BBscrambler, BCH, LDPC, interleaver, modulator, PL framer (pilots on), then Link X
dummy PLFRAMEs and the RRC at sps. One hier block because the gr-dtv GRC blocks only take
the MODCOD as fixed enums."""))

p = [P("mode", "Source (generate / replay)", "string", "generate"),
     P("replay_file", "Replay IQ file", "string", ""),
     P("replay_format", "Replay format (auto / sc16 / fc32)", "string", "auto", "part"),
     P("replay_loop", "Loop the replay", "bool", "True")]
B.append(block("g5gs_iq_select", "G5 IQ Select (generate / replay)", p,
               "g5gs.iq_select(" + args(p) + ")", inputs=stream("complex"),
               outputs=stream("complex"), doc="Pass the generated signal, or replay an IQ file."))

p = [P("samp_rate", "Sample rate", "real", "40e6"),
     P("signal_power", "Input mean power", "real", "1.0"),
     P("rms_dbfs", "Output RMS at level 0 dB (dBFS)", "real", "-12"),
     P("level_db", "Level (dB)", "real", "0"),
     P("cfo_hz", "CFO (Hz)", "real", "0"),
     P("esn0_db", "Es/N0 dB (empty = no noise)", "string", ""),
     P("sps", "Samples per symbol", "int", "2"),
     P("schedule", "Schedule (t:off, t:on, t:-20dB)", "string", ""),
     P("tle", "Doppler TLE (empty = off)", "string", ""),
     P("sat", "Satellite", "string", "", "part"),
     P("station", "Station lat,lon,h", "string", "1.2977,103.7764,60", "part"),
     P("rf_freq", "RF (Hz)", "real", "13.95e9", "part"),
     P("doppler_start_utc", "Doppler start (UTC / unix; empty = now)", "raw", "''"),
     P("doppler_dt", "Doppler time shift (s)", "real", "0", "part"),
     P("seed", "Noise seed", "int", "0", "part"),
     P("carrier", "Carrier on", "bool", "True")]
B.append(block("g5gs_tx_impairments", "G5 TX Impairments", p,
               "g5gs.impairments(" + args(p) + ")", inputs=stream("complex"),
               outputs=stream("complex"),
               callbacks=["set_level_db(${level_db})", "set_cfo(${cfo_hz})",
                          "set_carrier(${carrier})"], doc="""
Carrier on/off, level, transient schedule (signal time), CFO, TLE Doppler, AWGN."""))

p = [P("mode", "Output (b205 / file)", "string", "b205"),
     P("samp_rate", "Sample rate", "real", "40e6"),
     P("sym_rate", "Symbol rate", "real", "20e6"),
     P("rolloff", "Roll-off", "real", "0.2", "part"),
     P("centre_freq", "Centre frequency", "real", "1195e6"),
     P("tx_gain", "TX gain (dB)", "real", "0"),
     P("device_args", "UHD device args", "string", ""),
     P("lo_offset", "LO offset (-1 = auto)", "real", "-1", "part"),
     P("output_file", "Output file", "string", ""),
     P("output_format", "Output format (auto / sc16 / fc32)", "string", "auto"),
     P("duration", "File: seconds (0 = until the payload ends)", "real", "0"),
     P("throttle", "File: real time", "bool", "False"),
     P("start_unix", "First-sample time for the sidecar", "raw", "None", "part"),
     P("meta", "Extra JSON metadata", "raw", "{}", "part")]
B.append(block("g5gs_tx_sink", "G5 TX Sink", p, "g5gs.tx_sink(" + args(p) + ")",
               inputs=stream("complex"), callbacks=["set_tx_gain(${tx_gain})"], doc="""
B205mini (TX/RX) or an IQ file with .start/.json sidecars."""))

p = [P("ids", "Block ids {payload, framer, imp, sink}", "raw", "{}"),
     P("cfg_text", "Settings text", "string", ""),
     P("active", "Active at start", "bool", "True", "part")]
B.append(block("g5gs_tx_status_panel", "G5 TX Status Panel", p,
               "g5gs.TxStatusPanel(self, " + args(p) + ")", widget=True,
               doc="Payload progress, PLFRAME counts, impairments, Doppler, output."))


def main():
    os.makedirs(OUT, exist_ok=True)
    for b in B:
        with open(os.path.join(OUT, b["id"] + ".block.yml"), "w") as f:
            yaml.safe_dump(b, f, sort_keys=False, width=100)
    print(f"wrote {len(B)} blocks to {os.path.abspath(OUT)}")


if __name__ == "__main__":
    main()
