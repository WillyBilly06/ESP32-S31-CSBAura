"""Copy this project's patched ESP-IDF into a folder (third_party/esp-idf of the release).

The copy holds exactly the ESP-IDF tree the firmware is built with: Espressif's commit
(idf-patches/IDF_COMMIT) with its submodules and our patches applied, without docs/,
examples/, CI files and git history, with upstream's own bytes (LF line endings). It adds:
  version.txt   the ESP-IDF version string (the build reads it instead of git)
  S31_IDF.json  commit + patches, so the S31 Flasher knows the folder is ready as it is

    python tools/release/idf_tree.py <patched esp-idf checkout> <out folder>

The checkout must be a git checkout at IDF_COMMIT with the patches applied and nothing else
changed. promote.py calls copy_tree() for the release.
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import gitexport  # noqa: E402

PATCHES = ["0010-controller-only-iso-rx.patch", "0011-btdm-task-stack-dma.patch"]
SKIP = ("docs/", "examples/", ".github/", ".gitlab/")
SKIP_FILES = {".gitmodules", ".gitlab-ci.yml"}
# deep folders nothing builds from: under third_party/esp-idf they pass Windows' 260
# character path limit (unzip and git clone fail). The pqcp driver compiles only its src/
# with mldsa-native/mldsa; framework/data_files is test certificates and keys.
_TPC = "components/mbedtls/mbedtls/tf-psa-crypto/"
SKIP_DEEP = tuple(_TPC + "drivers/pqcp/mldsa-native/" + d + "/" for d in
                  ("examples", "proofs", "test", "dev", "nix", "integration", "scripts")) + (
    _TPC + "framework/data_files/", "components/mbedtls/mbedtls/framework/data_files/")


def check(idf):
    """info, changed files: after checking the checkout is IDF_COMMIT + our patches only"""
    with open(os.path.join(ROOT, "idf-patches", "IDF_COMMIT"), encoding="utf-8") as f:
        commit = f.read().strip()
    head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=idf, capture_output=True, text=True).stdout.strip()
    if head != commit:
        raise SystemExit(f"{idf} is at {head or '?'}, not {commit}")
    patched = set()
    for p in PATCHES:
        pp = os.path.join(ROOT, "idf-patches", p)
        if subprocess.run(["git", "apply", "--check", "-R", pp], cwd=idf, capture_output=True).returncode:
            raise SystemExit(f"{p} is not applied in {idf}")
        with open(pp, encoding="utf-8") as f:
            patched |= {l[6:].strip() for l in f if l.startswith("+++ b/")}
    changed = gitexport.changed_paths(idf)
    if changed - patched:
        raise SystemExit(f"{idf} has other changes: {sorted(changed - patched)}")
    ver = subprocess.run(["git", "describe", "--always", "--tags"], cwd=idf, capture_output=True,
                         text=True).stdout.strip() + "-s31"
    return {"commit": commit, "patches": PATCHES, "version": ver}, changed


def keep(path):
    return not path.startswith(SKIP) and not path.startswith(SKIP_DEEP) and path not in SKIP_FILES


def _put(d, data):
    os.makedirs(os.path.dirname(d), exist_ok=True)
    with open(d, "wb") as fh:
        fh.write(data)


def copy_tree(idf, out, put=_put):
    """the patched tree into out (put(path, bytes) writes one file); returns [executable paths]"""
    info, changed = check(idf)
    execs, n = [], 0
    for path, data, exe in gitexport.export(idf, keep, changed):
        put(os.path.join(out, path), data)
        if exe:
            execs.append(path)
        n += 1
    put(os.path.join(out, "version.txt"), (info["version"] + "\n").encode("utf-8"))
    put(os.path.join(out, "S31_IDF.json"), (json.dumps(info, indent=2) + "\n").encode("utf-8"))
    print(f"ESP-IDF {info['version']}: {n} files -> {out}")
    return execs


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    if os.path.exists(sys.argv[2]):
        raise SystemExit(f"{sys.argv[2]} already exists")
    copy_tree(sys.argv[1], sys.argv[2])
