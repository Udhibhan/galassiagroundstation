"""g5gs: Galassia-5 ground-station blocks for GNU Radio (pure Python, no build step).

RX, in order: rx_source, duty_gate, doppler_precomp, front_agc, [gr-dvbs2rx rotator and
symbol sync], post_agc, [gr-dvbs2rx PL sync, demapper, LDPC, BCH, BB descrambler], gs_decoder
TX: tx_payload, dvbs2_tx, iq_select, impairments, tx_sink
GUI: RxStatusPanel, DopplerPanel, FilesPanel, TxStatusPanel, PanelSwitch
See ../README.md. The GRC block files are in ../grc (setup_grc.sh registers them).
"""
from . import config  # noqa: F401
from .config import PRESETS, resolve  # noqa: F401

from .agc import front_agc, post_agc  # noqa: F401,E402
from .doppler_block import doppler_precomp  # noqa: F401,E402
from .gate import auto_mode, duty_gate  # noqa: F401,E402
from .gs_decoder import gs_decoder  # noqa: F401,E402
from .rx_source import rx_source  # noqa: F401,E402
from .tx import dvbs2_tx, impairments, iq_select, tx_payload, tx_sink  # noqa: F401,E402


def __getattr__(name):
    # Qt panels: imported on first use, so headless tools don't need a display
    if name in ("RxStatusPanel", "DopplerPanel", "FilesPanel", "TxStatusPanel", "PanelSwitch",
                "panels"):
        import importlib
        panels = importlib.import_module(__name__ + ".panels")
        return panels if name == "panels" else getattr(panels, name)
    raise AttributeError(name)
