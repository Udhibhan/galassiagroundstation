#!/usr/bin/env python3
"""Check that the gr-dvbs2rx GNU Radio actually loads is the transient-patched build.

Read-only: builds and installs nothing. Run by setup_grc.sh, or on its own:
    python3 tools/check_dvbs2rx.py            # check the installed library
    python3 tools/check_dvbs2rx.py --lib F    # only look for the patch marker in file F

Checks:
  - `from gnuradio import dvbs2rx` works (fails e.g. when a copy built for another GNU Radio
    version shadows the right one)
  - the libgnuradio-dvbs2rx.so that gets loaded contains the patch's log message
    "Input power" (the symbol-sync overload warning; the unpatched library has none)
  - only one Python package copy and one library copy are visible (a stale copy under /usr
    shadowed the /usr/local build on Compewter, 2026-09-27)
  - numpy is not a pip-installed 2.x shadowing Ubuntu's (breaks `from gnuradio import gr`)
Exit code 0 if all is well, 1 if there is something to fix.
"""
import argparse
import glob
import os
import subprocess
import sys

MARKER = b"Input power"
REPO = "https://github.com/Smolrdwarf/gr-dvbs2rx-patched (branch transient-fix)"


def marker_count(path):
    try:
        with open(path, "rb") as f:
            return f.read().count(MARKER)
    except OSError:
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lib", help="only check this library file for the patch marker")
    a = ap.parse_args()
    if a.lib:
        n = marker_count(a.lib)
        print(f"{a.lib}: " + ("not readable" if n is None else
                               "PATCHED" if n else "NOT patched (no 'Input power' marker)"))
        return 0 if n else 1

    problems = []
    print("gr-dvbs2rx check:")
    try:
        from gnuradio import gr
        print(f"  GNU Radio {gr.version()}")
    except Exception as e:                      # noqa: BLE001
        print(f"  PROBLEM: `from gnuradio import gr` fails: {e}")
        problems.append("gnuradio")
    try:
        import numpy as np
        user = os.path.expanduser("~/.local")
        print(f"  numpy {np.__version__} from {os.path.dirname(np.__file__)}")
        if int(np.__version__.split(".")[0]) >= 2 and np.__file__.startswith(user):
            print("  PROBLEM: a pip-installed numpy 2.x shadows Ubuntu's numpy; GNU Radio needs "
                  "the system one: pip uninstall numpy")
            problems.append("numpy")
    except ImportError:
        print("  PROBLEM: numpy missing")
        problems.append("numpy")

    # every Python package copy of gnuradio.dvbs2rx on sys.path
    pkgs = []
    for p in sys.path:
        d = os.path.join(p or ".", "gnuradio", "dvbs2rx")
        if os.path.isdir(d) and os.path.realpath(d) not in [os.path.realpath(x) for x in pkgs]:
            pkgs.append(d)
    try:
        from gnuradio import dvbs2rx
        used_pkg = os.path.dirname(os.path.abspath(dvbs2rx.__file__))
        print(f"  Python module: {used_pkg}")
    except Exception as e:                      # noqa: BLE001
        used_pkg = None
        print(f"  PROBLEM: `from gnuradio import dvbs2rx` fails: {e}")
        print("    (not installed, or a copy built for another GNU Radio version shadows it)")
        problems.append("import")
    if len(pkgs) > 1:
        print("  PROBLEM: more than one gnuradio/dvbs2rx package on the Python path:")
        for d in pkgs:
            print(f"    {d}" + ("   <- the one Python uses" if used_pkg and
                                os.path.realpath(d) == os.path.realpath(used_pkg) else ""))
        print("    remove the stale one (check it is not owned by a package: dpkg -S <dir>)")
        problems.append("duplicate package")

    # the library actually loaded into this process
    loaded = []
    if used_pkg:
        with open("/proc/self/maps") as f:
            for line in f:
                p = line.split()[-1]
                if "libgnuradio-dvbs2rx" in p and p not in loaded:
                    loaded.append(p)
    for p in loaded:
        n = marker_count(os.path.realpath(p))
        print(f"  library loaded: {os.path.realpath(p)}: "
              + ("PATCHED" if n else "NOT PATCHED"))
        if not n:
            print(f"  PROBLEM: this is the unpatched gr-dvbs2rx; build and install {REPO}")
            problems.append("unpatched")
    if used_pkg and not loaded:
        print("  PROBLEM: could not find the loaded libgnuradio-dvbs2rx")
        problems.append("lib not found")

    # every library copy the dynamic linker knows about
    try:
        out = subprocess.run(["ldconfig", "-p"], capture_output=True, text=True).stdout
        libs = sorted({os.path.realpath(line.split("=>")[-1].strip())
                       for line in out.splitlines() if "libgnuradio-dvbs2rx" in line})
    except OSError:
        libs = []
    libs += [os.path.realpath(x) for x in glob.glob("/usr/lib/libgnuradio-dvbs2rx.so*")
             + glob.glob("/usr/local/lib/libgnuradio-dvbs2rx.so*")
             if os.path.realpath(x) not in libs]
    if len(set(libs)) > 1:
        print("  PROBLEM: more than one libgnuradio-dvbs2rx installed:")
        for p in sorted(set(libs)):
            print(f"    {p} ({'patched' if marker_count(p) else 'NOT patched'})")
        problems.append("duplicate library")

    if problems:
        print(f"  => {len(problems)} problem(s): see above. Build/install steps: README.md, "
              "'Setup on a new laptop'.")
        return 1
    print("  => OK: the patched gr-dvbs2rx is installed and is the one in use")
    return 0


if __name__ == "__main__":
    sys.exit(main())
