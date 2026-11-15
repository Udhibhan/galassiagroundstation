"""
doppler.py

Computes Doppler-shifted frequency from a satellite's range-rate
(closing/receding velocity relative to the ground station), for radio
tuning during a pass. 

Pulls range_rate_km_s from trajectory.XYSample.
"""

from dataclasses import dataclass

from trajectory import XYSample

SPEED_OF_LIGHT_KM_S = 299_792.458

@dataclass
class DopplerResult:
    tx_freq_hz: float
    doppler_shift_hz: float
    rx_freq_hz: float

def compute_doppler(sample: XYSample, tx_freq_hz: float) -> DopplerResult:
    """
    Given the nominal transmit frequency of the satellite (`tx_freq_hz`)
    and a trajectory sample carrying range_rate_km_s (negative = closing,
    positive = receding, matching Skyfield's convention), return the
    Doppler shift and the frequency the ground station should tune to.
    """
    shift_hz = -sample.range_rate_km_s / SPEED_OF_LIGHT_KM_S * tx_freq_hz
    return DopplerResult(
        tx_freq_hz=tx_freq_hz,
        doppler_shift_hz=shift_hz,
        rx_freq_hz=tx_freq_hz + shift_hz,
    )

def doppler_series(samples: list[XYSample], tx_freq_hz: float) -> list[DopplerResult]:
    """Convenience: compute Doppler for a whole precomputed trajectory series."""
    return [compute_doppler(s, tx_freq_hz) for s in samples]