#!/bin/bash
# One-time setup per laptop: make the g5gs package importable and its blocks visible in GRC.
#   - a .pth file in the user site-packages points Python at this folder (so `import g5gs`
#     works in GRC, in the generated flowgraphs and in the CLIs)
#   - GNU Radio's user config (~/.config/gnuradio/config.conf on GR 3.10) gets
#     [grc] local_blocks_path = <this folder>/grc
#   - checks (read-only) that the gr-dvbs2rx in use is the transient-patched build
# No sudo needed; nothing is built here (README.md: "Setup on a new laptop"). Run it again if
# you move the folder. Python deps: numpy, PyYAML, pyqtgraph
# (Doppler plot; optional), sgp4 (only for TLE pre-comp / Doppler sim: pip install --user sgp4).
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SITE="$(python3 -m site --user-site)"
mkdir -p "$SITE"
echo "$HERE" > "$SITE/g5gs.pth"
echo "wrote $SITE/g5gs.pth -> $HERE"

CONF="$(python3 -c 'from gnuradio import gr; print(gr.paths.userconf())')/config.conf"
mkdir -p "$(dirname "$CONF")"
touch "$CONF"
python3 - "$CONF" "$HERE/grc" <<'PY'
import configparser, sys
conf, path = sys.argv[1], sys.argv[2]
c = configparser.ConfigParser()
c.read(conf)
if not c.has_section("grc"):
    c.add_section("grc")
paths = [p for p in c.get("grc", "local_blocks_path", fallback="").split(":") if p]
if path not in paths:
    paths.append(path)
c.set("grc", "local_blocks_path", ":".join(paths))
with open(conf, "w") as f:
    c.write(f)
print(f"{conf}: [grc] local_blocks_path = {':'.join(paths)}")
PY
python3 -c "import g5gs, sys; print('import g5gs OK from', g5gs.__file__)"
python3 -c "import sgp4" 2>/dev/null && echo "sgp4 OK" || echo "NOTE: sgp4 missing (TLE features): pip install --user sgp4"
python3 -c "import pyqtgraph" 2>/dev/null && echo "pyqtgraph OK" || echo "NOTE: pyqtgraph missing (Doppler plot): sudo apt install python3-pyqtgraph"
echo
if python3 "$HERE/tools/check_dvbs2rx.py"; then
    echo
    echo "Restart GNU Radio Companion: the blocks are under 'G5 Ground Station'."
else
    echo
    echo "WARNING: fix the gr-dvbs2rx problem(s) above before using the receiver"
    echo "(the g5gs setup itself is done)."
fi
