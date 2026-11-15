"""Front AGC (before the rotator) and post-AGC (between symbol sync and PL sync).

Front: agc3 (default, attack 1e-2 / decay 1e-4, ref 1.37: 03102026_agc_findings.md) or agc
(the original analog.agc_cc, rate 1e-5). The patched gr-dvbs2rx survives AGC overshoot, but
the PL sync only works when its input is near unit power, hence ref 1.37 for AGC3.

Post: agc2_cc holding the PL sync input at ~1 (19102026_postagc_results.md): fast attack 1e-3,
slow decay 1e-5 per symbol. ref 0 = off (the pre-2026-10-19 chain).
"""
from __future__ import annotations

from gnuradio import analog, blocks, gr


class front_agc(gr.hier_block2):
    def __init__(self, kind="agc3", ref=1.37, attack=1e-2, decay=1e-4, rate=1e-5, gain=1.0,
                 max_gain=65536):
        gr.hier_block2.__init__(self, "g5_front_agc", gr.io_signature(1, 1, gr.sizeof_gr_complex),
                                gr.io_signature(1, 1, gr.sizeof_gr_complex))
        if kind == "agc3":
            self.agc = analog.agc3_cc(attack, decay, ref, gain, 1)
            self.agc.set_max_gain(max_gain)
        elif kind == "agc":
            self.agc = analog.agc_cc(rate, ref, gain, max_gain)
        else:
            raise ValueError(f"AGC {kind!r}: agc3 or agc")
        self.kind = kind
        self.connect(self, self.agc, self)

    def gain(self):
        return self.agc.gain()


class post_agc(gr.hier_block2):
    def __init__(self, ref=1.0, attack=1e-3, decay=1e-5):
        gr.hier_block2.__init__(self, "g5_post_agc", gr.io_signature(1, 1, gr.sizeof_gr_complex),
                                gr.io_signature(1, 1, gr.sizeof_gr_complex))
        self.ref = ref
        if not ref:
            self.agc = None
            self.connect(self, blocks.copy(gr.sizeof_gr_complex), self)   # pass-through
            return
        self.agc = analog.agc2_cc(attack, decay or attack, ref, 1.0)
        self.agc.set_max_gain(1000)
        self.connect(self, self.agc, self)
