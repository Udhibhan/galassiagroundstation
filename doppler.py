"""Doppler-shifted receive frequency from range rate (trajectory.XYSample.range_rate_km_s)."""

from dataclasses import dataclass

from trajectory import XYSample

SPEED_OF_LIGHT_KM_S = 299_792.458

@dataclass
class DopplerResult:
    tx_freq_hz: float
    doppler_shift_hz: float
    rx_freq_hz: float

def compute_doppler(sample: XYSample, tx_freq_hz: float) -> DopplerResult:
    """Shift and tune frequency for a nominal tx_freq_hz.
    range_rate_km_s is negative when closing, positive when receding (Skyfield's sign)."""
    shift_hz = -sample.range_rate_km_s / SPEED_OF_LIGHT_KM_S * tx_freq_hz
    return DopplerResult(
        tx_freq_hz=tx_freq_hz,
        doppler_shift_hz=shift_hz,
        rx_freq_hz=tx_freq_hz + shift_hz,
    )

def doppler_series(samples: list[XYSample], tx_freq_hz: float) -> list[DopplerResult]:
    """compute_doppler over a whole trajectory."""
    return [compute_doppler(s, tx_freq_hz) for s in samples]