#!/usr/bin/env bash
# Run the whole ground station:
#     ./run.sh                    real hardware, whatever is plugged in gets detected (GNSS, SDR, gimbal via the Pi)
#     ./run.sh --pi 192.168.1.50  gimbal through the Pi relay (or GS_PI=192.168.1.50 ./run.sh)
#     ./run.sh --sim              everything simulated, no hardware
# Opens http://localhost:8080. Ctrl-C stops it; motors stay enabled and holding.
cd "$(dirname "$0")"
PY="${GS_PYTHON:-}"
if [ -z "$PY" ]; then
  for c in "$HOME/radioconda/bin/python" python3; do
    if command -v "$c" >/dev/null 2>&1 && "$c" -c "import numpy" 2>/dev/null; then PY="$c"; break; fi
  done
fi
[ -z "$PY" ] && { echo "No Python with numpy found. Set GS_PYTHON=/path/to/python (radioconda: ~/radioconda/bin/python)."; exit 1; }
exec "$PY" app.py "$@"
