"""
sdr_plot.py - live graph of the LNB IF power (same measurement as sdr_power.py). Receive only (RX2).

    python3 sdr_plot.py                      # uses config.SDR (IF, rate, gain)
    python3 sdr_plot.py --gain 10 --freq 1.2e9

Top: power (dBFS) vs time, ~10 readings/s. Bottom: spectrum of the latest block.
Stop app.py first, only one program can open the SDR. Close the window to quit.
"""
import argparse
import collections
import time

import matplotlib.pyplot as plt
import numpy as np
import uhd

import config

ap = argparse.ArgumentParser()
ap.add_argument("--args", default=config.SDR.args)
ap.add_argument("--freq", type=float, default=config.SDR.if_freq_hz)
ap.add_argument("--rate", type=float, default=config.SDR.rate_hz)
ap.add_argument("--gain", type=float, default=config.SDR.gain_db)
ap.add_argument("--seconds", type=float, default=120, help="time axis length")
a = ap.parse_args()

u = uhd.usrp.MultiUSRP(a.args)
u.set_rx_rate(a.rate, 0)
u.set_rx_freq(uhd.types.TuneRequest(a.freq), 0)
u.set_rx_gain(a.gain, 0)
u.set_rx_antenna("RX2", 0)                                   # RX2 only, never TX
st = u.get_rx_stream(uhd.usrp.StreamArgs("fc32", "sc16"))     # RX streamer only
print(f"{u.get_mboard_name()}  IF {a.freq/1e9:.3f} GHz  {u.get_rx_rate()/1e6:.1f} Msps  gain {u.get_rx_gain(0):.1f} dB")

N, SKIP = 200_000, 2000
buf = np.zeros((1, N + SKIP), dtype=np.complex64)


def block():
    cmd = uhd.types.StreamCMD(uhd.types.StreamMode.num_done)
    cmd.num_samps, cmd.stream_now = N + SKIP, True
    st.issue_stream_cmd(cmd)
    md, got = uhd.types.RXMetadata(), 0
    while got < N + SKIP:
        k = st.recv(buf[:, got:], md, 1.0)
        if k == 0:
            if md.error_code != uhd.types.RXMetadataErrorCode.none:
                print("RX error:", md.strerror())
            break
        got += k
    return buf[0, SKIP:got]


plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
ts, ps = collections.deque(), collections.deque()
(l1,) = ax1.plot([], [], lw=1)
ax1.set_xlabel("time (s)"); ax1.set_ylabel("power (dBFS)"); ax1.grid(True)
(l2,) = ax2.plot([], [], lw=0.7)
ax2.set_xlabel("offset from IF (MHz)"); ax2.set_ylabel("dB"); ax2.grid(True)
t0 = time.monotonic()
fft_n = 4096
f_mhz = np.fft.fftshift(np.fft.fftfreq(fft_n, 1 / a.rate)) / 1e6

while plt.fignum_exists(fig.number):
    x = block()
    if len(x) == 0:
        plt.pause(0.2)
        continue
    p = float(np.mean(np.abs(x) ** 2))
    peak = float(np.abs(x).max())
    t = time.monotonic() - t0
    ts.append(t); ps.append(10 * np.log10(p + 1e-30))
    while ts and ts[0] < t - a.seconds:
        ts.popleft(); ps.popleft()
    l1.set_data(ts, ps)
    ax1.relim(); ax1.autoscale_view()
    sat = peak > config.SDR.saturation_fs
    ax1.set_title(f"{ps[-1]:.2f} dBFS   peak |x| {peak:.2f}" + ("   SATURATED - reduce gain" if sat else ""),
                  color="red" if sat else "black")
    w = x[: (len(x) // fft_n) * fft_n].reshape(-1, fft_n) * np.hanning(fft_n)
    spec = 10 * np.log10(np.mean(np.abs(np.fft.fft(w, axis=1)) ** 2, axis=0) + 1e-30)
    l2.set_data(f_mhz, np.fft.fftshift(spec))
    ax2.relim(); ax2.autoscale_view()
    plt.pause(0.05)
