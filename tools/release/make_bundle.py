"""The complete download: the repository at a commit (patched ESP-IDF, BTstack, firmware,
tools), S31Flasher.exe, and the offline tools package (make_offline.py), in one zip.

    python tools/release/make_bundle.py <out.zip> --repo <release checkout> --exe <S31Flasher.exe>
                                        --offline <folder with S31-offline> [--ref HEAD]

Unzipped, it is ESP32-S31-CSBAura/ with S31Flasher.exe and S31-offline/ next to README.md:
run the exe and Build + Flash, nothing is downloaded.
"""
import argparse
import os
import subprocess
import zipfile

TOP = "ESP32-S31-CSBAura"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--repo", required=True)
    ap.add_argument("--exe", required=True)
    ap.add_argument("--offline", required=True, help="the folder that holds S31-offline/")
    ap.add_argument("--ref", default="HEAD")
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    if os.path.exists(out):
        os.remove(out)
    # the repository exactly as committed (what GitHub's Download ZIP gives)
    subprocess.run(["git", "archive", "--format=zip", f"--prefix={TOP}/", "-o", out, a.ref], cwd=a.repo, check=True)
    sha = subprocess.run(["git", "rev-parse", a.ref], cwd=a.repo, capture_output=True, text=True, check=True).stdout.strip()
    pkg = os.path.join(a.offline, "S31-offline")
    if not os.path.isfile(os.path.join(pkg, "manifest.json")):
        raise SystemExit(f"no S31-offline/manifest.json in {a.offline} - run make_offline.py first")
    with zipfile.ZipFile(out, "a", zipfile.ZIP_DEFLATED, allowZip64=True) as z:
        z.write(a.exe, f"{TOP}/S31Flasher.exe", zipfile.ZIP_DEFLATED)
        for d, _, files in os.walk(pkg):
            for fn in files:
                p = os.path.join(d, fn)
                arc = f"{TOP}/" + os.path.relpath(p, a.offline).replace(os.sep, "/")
                stored = fn.endswith((".zip", ".xz", ".gz", ".whl", ".exe"))
                z.write(p, arc, zipfile.ZIP_STORED if stored else zipfile.ZIP_DEFLATED)
        z.writestr(f"{TOP}/S31-offline/BUNDLE.txt", f"repository commit {sha}\n")
    print(f"{out}: {os.path.getsize(out) / 1e6:.0f} MB (commit {sha[:10]})")


if __name__ == "__main__":
    main()
