"""Install the SeeYou Navigator plugin into X-Plane's PythonPlugins folder.

Usage:
    python tools/install_plugin.py [--xplane "D:/.../X-Plane 12"] [--dry-run]

Copies `plugin/PI_xp12_seeyou_navigator.py` and the `xp12_seeyou` package
(without caches) into `Resources/plugins/PythonPlugins/`, replacing a previous
copy of the package so that deleted modules do not linger. Also checks that
XPPython3's Python has pyserial. Restart X-Plane afterwards: XPPython3's
"Reload scripts" does not reload submodules. The X-Plane folder defaults to the
XPLANE_DIR environment variable.
"""

import argparse
import os
import shutil
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ENTRY = "PI_xp12_seeyou_navigator.py"
PACKAGE = "xp12_seeyou"
IGNORE = shutil.ignore_patterns("__pycache__", "*.pyc", "*.pyo")


def xppython_executable(xplane_dir):
    for sub in ("win_x64/python.exe", "mac_x64/python3", "lin_x64/python3"):
        path = os.path.join(xplane_dir, "Resources", "plugins", "XPPython3", sub)
        if os.path.isfile(path):
            return path
    return None


def check_pyserial(xplane_dir):
    python = xppython_executable(xplane_dir)
    if python is None:
        print("could not find XPPython3's Python to check for pyserial")
        return
    result = subprocess.run([python, "-c", "import serial; print(serial.__version__)"],
                            capture_output=True, text=True)
    if result.returncode == 0:
        print(f"pyserial {result.stdout.strip()} found in XPPython3's Python")
    else:
        print("WARNING: pyserial is missing in XPPython3's Python. Install it with\n"
              "  X-Plane menu Plugins > XPPython3 > Pip Package Installer > pyserial\n"
              f"or: \"{python}\" -m pip install pyserial")


def install(xplane_dir, dry_run=False):
    dest = os.path.join(xplane_dir, "Resources", "plugins", "PythonPlugins")
    if not os.path.isdir(dest):
        raise SystemExit(f"not found: {dest} (is XPPython3 installed and run once?)")
    pkg_dest = os.path.join(dest, PACKAGE)
    if os.path.exists(pkg_dest) and not os.path.isfile(os.path.join(pkg_dest, "__init__.py")):
        raise SystemExit(f"{pkg_dest} exists but is not a Python package; not touching it")
    actions = [f"copy {ENTRY} -> {dest}",
               f"{'replace' if os.path.exists(pkg_dest) else 'create'} {pkg_dest}"]
    for line in actions:
        print(("[dry run] " if dry_run else "") + line)
    if not dry_run:
        shutil.copy2(os.path.join(REPO, "plugin", ENTRY), os.path.join(dest, ENTRY))
        if os.path.exists(pkg_dest):
            shutil.rmtree(pkg_dest)
        shutil.copytree(os.path.join(REPO, PACKAGE), pkg_dest, ignore=IGNORE)
        count = sum(len(files) for _, _, files in os.walk(pkg_dest))
        print(f"installed {count} package files; restart X-Plane to load them")
    check_pyserial(xplane_dir)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Install the SeeYou Navigator plugin into X-Plane 12")
    parser.add_argument("--xplane", default=os.environ.get("XPLANE_DIR"),
                        help="X-Plane 12 folder (default: $XPLANE_DIR)")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    if not args.xplane:
        parser.error("give --xplane or set XPLANE_DIR")
    install(args.xplane, args.dry_run)


if __name__ == "__main__":
    sys.exit(main())
