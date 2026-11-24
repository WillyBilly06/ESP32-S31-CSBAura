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
# --collect-all imports esptool in the Python running PyInstaller: without the IDF environment
# on its path it collects nothing, and the exe has esptool's code but not its flasher stubs
# ("Flasher stub data is missing for ESP32-S31", slow ROM-loader flashing)
env = dict(os.environ, PYTHONPATH=os.pathsep.join([sp] + ([os.environ["PYTHONPATH"]] if os.environ.get("PYTHONPATH") else [])))
print(" ".join(cmd))
rc = subprocess.call(cmd, env=env)
if rc == 0:
    toc = os.path.join(work, "S31Flasher", "Analysis-00.toc")
    if os.path.isfile(toc) and "esp32s31.json" not in open(toc, encoding="utf-8", errors="replace").read():
        sys.exit("built, but without esptool's ESP32-S31 flasher stub - check the IDF environment")
sys.exit(rc)
