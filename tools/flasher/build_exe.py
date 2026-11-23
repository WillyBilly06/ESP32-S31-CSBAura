"""Build tools/flasher/dist/S31Flasher.exe with PyInstaller.

    python tools/flasher/build_exe.py [--idf-env PATH]

esptool and pyserial are taken from the ESP-IDF Python environment (it has the esptool
that knows the ESP32-S31), so nothing is downloaded. Default: the newest
~/.espressif/python_env/idf*_env. PyInstaller itself must be importable by this Python
(pip install pyinstaller).
"""
import argparse
import glob
import os
import subprocess
import sys

here = os.path.dirname(os.path.abspath(__file__))


def idf_site_packages(env):
    if env is None:
        envs = sorted(glob.glob(os.path.expanduser("~/.espressif/python_env/idf*_env")))
        if not envs:
            sys.exit("no ESP-IDF Python environment found - pass --idf-env")
        env = envs[-1]
    for sp in (os.path.join(env, "Lib", "site-packages"), *glob.glob(os.path.join(env, "lib", "python3*", "site-packages"))):
        if os.path.isdir(os.path.join(sp, "esptool")):
            return sp
    sys.exit(f"no esptool in {env}")


ap = argparse.ArgumentParser()
ap.add_argument("--idf-env")
a = ap.parse_args()
sp = idf_site_packages(a.idf_env)
work = os.path.join(here, "build_pyi")
cmd = [sys.executable, "-m", "PyInstaller", "--noconfirm", "--onefile", "--windowed", "--name", "S31Flasher",
       "--paths", sp, "--collect-all", "esptool", "--hidden-import", "serial.tools.list_ports",
       "--distpath", os.path.join(here, "dist"), "--workpath", work, "--specpath", work,
       os.path.join(here, "s31_flasher.py")]
print(" ".join(cmd))
sys.exit(subprocess.call(cmd))
