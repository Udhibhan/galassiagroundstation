#!/usr/bin/env bash
# demod_env.sh - run a command inside the demod environment (GNU Radio 3.10 + the PATCHED gr-dvbs2rx).
#
#   ./demod_env.sh gnuradio-companion demod/unified/rx_unified.grc     (open the receiver flowgraph)
#   ./demod_env.sh dvbs2-rx --help
#   ./demod_env.sh python3 -c "from gnuradio import dvbs2rx; print(dvbs2rx.__file__)"
#   ./demod_env.sh dvbs2-tx --source file --in-file x.ts --modcod qpsk1/4 --pilots --snr 6 | \
#       ./demod_env.sh dvbs2-rx --sink file --out-file out.ts --modcod qpsk1/4 --pilots on
#
# The env lives OUTSIDE radioconda (~/gs_demod_env, made with mamba; GNU Radio 3.10.12, UHD, build tools).
# The patched library is installed in ./gr_dvbs2rx (built from Smolrdwarf/gr-dvbs2rx-patched, branch transient-fix).
# Nothing here touches radioconda. RECEIVE ONLY: never use dvbs2-tx with a UHD sink on the B206mini.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV="${GS_DEMOD_ENV:-$HOME/gs_demod_env}"
MAMBA="${MAMBA:-$HOME/radioconda/bin/mamba}"
PFX="$HERE/gr_dvbs2rx"
export PYTHONPATH="$PFX/lib/python3.14/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$PFX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PATH="$PFX/bin:$PATH"
# GNU Radio Companion only knows the patched dvbs2rx_* blocks if their .block.yml folder is on its block path
# (otherwise rx_unified.grc shows them missing and "Port is not connected" errors).
export GRC_BLOCKS_PATH="$PFX/share/gnuradio/grc/blocks${GRC_BLOCKS_PATH:+:$GRC_BLOCKS_PATH}"
# The B206mini needs UHD's firmware / FPGA images. This env has none (-> "No devices found"): reuse radioconda's (same UHD 4.11.0).
if [ -z "${UHD_IMAGES_DIR:-}" ] && [ -d "$HOME/radioconda/share/uhd/images" ] && [ ! -d "$ENV/share/uhd/images" ]; then
    export UHD_IMAGES_DIR="$HOME/radioconda/share/uhd/images"
fi
exec "$MAMBA" run -p "$ENV" "$@"
