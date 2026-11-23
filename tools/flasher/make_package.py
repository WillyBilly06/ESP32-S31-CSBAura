"""Copy a finished build into a firmware package for the S31 Flasher.

    python tools/flasher/make_package.py [build_dir] [out_dir]

build_dir defaults to s31-btstack-speaker/build, out_dir to tools/flasher/firmware.
Only what flasher_args.json lists is copied (bootloader, partition table, app).
Prompt tones are never copied: they come from a real speaker's firmware and stay
local (put your own tones.bin into the package folder by hand if you have one).
"""
import json
import os
import shutil
import sys

here = os.path.dirname(os.path.abspath(__file__))
root = os.path.abspath(os.path.join(here, "..", ".."))
build = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else os.path.join(root, "s31-btstack-speaker", "build")
out = os.path.abspath(sys.argv[2]) if len(sys.argv) > 2 else os.path.join(here, "firmware")

with open(os.path.join(build, "flasher_args.json"), encoding="utf-8") as f:
    args = json.load(f)
keep = {"write_flash_args": args["write_flash_args"], "flash_files": args["flash_files"],
        "extra_esptool_args": args.get("extra_esptool_args", {})}
os.makedirs(out, exist_ok=True)
for rel in args["flash_files"].values():
    dst = os.path.join(out, rel)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copy2(os.path.join(build, rel), dst)
    print("copied", rel)
with open(os.path.join(out, "flasher_args.json"), "w", encoding="utf-8") as f:
    json.dump(keep, f, indent=2)
print("firmware package ready:", out)
