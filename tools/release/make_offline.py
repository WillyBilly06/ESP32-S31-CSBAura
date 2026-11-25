"""Build the S31 offline tools package: everything the first Build + Flash would download,
taken from the setup this project is built and tested with, so a PC needs no internet.

    python tools/release/make_offline.py <out_dir> [--idf third_party/esp-idf] [--env <python env>]

<out_dir>/S31-offline/ gets:
  dist/          ESP-IDF's tool archives for ESP32-S31 on Windows (the recommended versions in the
                 project's tools.json), each checked against tools.json's SHA-256
  wheels/        the Python packages of the tested ESP-IDF Python environment (pip freeze, CPython
                 3.12, win_amd64), plus pip and setuptools
  espidf.constraints.v6.1.txt   the constraints file that environment was installed with
  python/        python.org's installer of the Python the flasher installs (PY_VERSION)
  manifest.json  what is inside, with sizes and hashes
and <out_dir>/S31-offline-tools-win64.zip with that folder.

Tool archives come from %USERPROFILE%\\.espressif\\dist when they are there (same bytes as
Espressif's download, verified), else from Espressif's URL in tools.json. Wheels come through
`pip download` (pip's cache first). Needs the tested environment's Python (--env).
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import urllib.request
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
TARGET = "esp32s31"
PLATFORM = "win64"
PY_VERSION = "3.12.10"                     # = tools/flasher/s31_flasher.py PY_VERSION
PY_URL = f"https://www.python.org/ftp/python/{PY_VERSION}/python-{PY_VERSION}-amd64.exe"


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def fetch(url, dest):
    print(f"  downloading {url}")
    req = urllib.request.Request(url, headers={"User-Agent": "S31-make-offline"})
    with urllib.request.urlopen(req, timeout=120) as r, open(dest + ".part", "wb") as f:
        shutil.copyfileobj(r, f, 1 << 20)
    os.replace(dest + ".part", dest)


def tool_archives(idf):
    """(name, version, url, sha256, size) of every tool `install.ps1 esp32s31` installs"""
    with open(os.path.join(idf, "tools", "tools.json"), encoding="utf-8") as f:
        tj = json.load(f)
    out = []
    for t in tj["tools"]:
        install = t.get("install")
        for o in t.get("platform_overrides", []):          # cmake, ninja, idf-exe, ccache: always on Windows
            if PLATFORM in o.get("platforms", []) and "install" in o:
                install = o["install"]
        if install != "always":
            continue
        targets = t.get("supported_targets", ["all"])
        if "all" not in targets and TARGET not in targets:
            continue
        rec = [v for v in t["versions"] if v.get("status") == "recommended"]
        if not rec:
            continue
        v = rec[0]
        d = v.get(PLATFORM) or v.get("any")
        if not d:
            continue                                        # not a Windows tool
        out.append((t["name"], v["name"], d["url"], d["sha256"], d["size"]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--idf", default=os.path.join(ROOT, "third_party", "esp-idf"))
    ap.add_argument("--env", default=os.path.join(os.path.expanduser("~"), ".espressif", "python_env",
                                                  "idf6.1_py3.12_env"))
    a = ap.parse_args()
    pkg = os.path.join(os.path.abspath(a.out), "S31-offline")
    for sub in ("dist", "wheels", "python"):
        os.makedirs(os.path.join(pkg, sub), exist_ok=True)
    manifest = {"target": TARGET, "platform": PLATFORM, "python": PY_VERSION, "tools": [], "wheels": []}

    # 1 - tool archives
    cache = os.path.join(os.path.expanduser("~"), ".espressif", "dist")
    print("ESP-IDF tools for", TARGET)
    for name, ver, url, sh, size in tool_archives(a.idf):
        fn = os.path.basename(url)
        dest = os.path.join(pkg, "dist", fn)
        if not (os.path.isfile(dest) and sha256(dest) == sh):
            src = os.path.join(cache, fn)
            if os.path.isfile(src) and sha256(src) == sh:
                shutil.copyfile(src, dest)
            else:
                fetch(url, dest)
        if sha256(dest) != sh:
            sys.exit(f"{fn}: SHA-256 does not match tools.json")
        print(f"  {name} {ver}: {fn} ({size / 1e6:.0f} MB) ok")
        manifest["tools"].append({"name": name, "version": ver, "file": fn, "sha256": sh, "size": size})

    # 2 - wheels of the tested environment
    py = os.path.join(a.env, "Scripts", "python.exe")
    freeze = subprocess.run([py, "-m", "pip", "freeze", "--all"], capture_output=True, text=True, check=True).stdout
    reqs = [l.strip() for l in freeze.splitlines() if l.strip() and not l.startswith("-e") and "@" not in l]
    req_file = os.path.join(pkg, "wheels", "requirements-tested.txt")
    with open(req_file, "w", encoding="utf-8") as f:
        f.write("\n".join(reqs) + "\n")
    print(f"Python packages: {len(reqs)} pinned from {a.env}")
    # pip wheel in the tested environment itself (CPython 3.12, win_amd64 = the target): ready wheels
    # where PyPI has them, source-only packages (esptool 5.6.0 ...) built into wheels
    subprocess.run([py, "-m", "pip", "wheel", "--wheel-dir", os.path.join(pkg, "wheels"), "--no-deps",
                    "--extra-index-url", "https://dl.espressif.com/pypi",   # as ESP-IDF installs them
                    "-r", req_file], check=True)
    for fn in sorted(os.listdir(os.path.join(pkg, "wheels"))):
        if fn.endswith(".whl"):
            manifest["wheels"].append(fn)

    # 3 - constraints file
    cons = os.path.join(os.path.expanduser("~"), ".espressif", "espidf.constraints.v6.1.txt")
    shutil.copyfile(cons, os.path.join(pkg, os.path.basename(cons)))

    # 4 - Python installer
    pyexe = os.path.join(pkg, "python", os.path.basename(PY_URL))
    if not os.path.isfile(pyexe):
        fetch(PY_URL, pyexe)
    manifest["python_installer"] = {"file": os.path.basename(PY_URL), "sha256": sha256(pyexe)}

    with open(os.path.join(pkg, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=1)

    # 5 - zip (archives are compressed already: store them)
    zpath = os.path.join(os.path.abspath(a.out), "S31-offline-tools-win64.zip")
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED, allowZip64=True) as z:
        for d, _, files in os.walk(pkg):
            for fn in files:
                p = os.path.join(d, fn)
                arc = os.path.relpath(p, os.path.dirname(pkg))
                stored = fn.endswith((".zip", ".xz", ".gz", ".whl", ".exe"))
                z.write(p, arc, zipfile.ZIP_STORED if stored else zipfile.ZIP_DEFLATED)
    print(f"\n{zpath}: {os.path.getsize(zpath) / 1e6:.0f} MB, {len(manifest['tools'])} tools, "
          f"{len(manifest['wheels'])} wheels, Python {PY_VERSION}")


if __name__ == "__main__":
    main()
