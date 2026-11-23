"""S31 Flasher - flash an ESP32-S31 as its own JBL Xtreme 5.

Every board comes out unique on its own (serial, Broadcast_ID and Bluetooth identity
come from the chip's factory MAC); this tool picks the color the JBL One app shows and
an optional name, and writes them to the board's "s31cfg" partition.

Window:   S31Flasher.exe
Command:  S31Flasher.exe --port COM5 --color black [--name "Party Box"] [--erase]
          S31Flasher.exe --port COM5 --color-id 0x01 --color-only     (just change the color)

The firmware comes from a folder next to the tool (default ./firmware), made from a
build with tools/flasher/make_package.py: flasher_args.json + the .bin files it lists.
An optional firmware/tones.bin (prompt tones, made locally with tools/tones_pack.py)
is written too.
"""
import argparse
import io
import json
import os
import struct
import sys
import tempfile
import threading

# JBL color IDs of the Xtreme 5, from JBL's own model config for product 2169
# ("official_color_name" in appstorage.onecloud.harman.com/jbl_one/app_config/v11/
# portable/2169/model_config.json). JBL One reads the ID from the BLE beacon (colorID).
COLORS = [
    ("Black", 0x00, True),
    ("White", 0x01, True),
    ("Blue", 0x02, True),
    ("Purple", 0x05, True),
    ("Camo", 0x1A, True),
]
CFG_OFFSET = 0xA10000          # partitions.csv "s31cfg"
CFG_SIZE = 0x1000
TONES_OFFSET = 0x810000        # partitions.csv "tones"
NAME_MAX = 32


def base_dir():
    """folder of the .exe (frozen) or of this script"""
    if getattr(sys, "frozen", False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(__file__))


def workspace_root():
    """the repo / workspace the tool sits in (dev use), or None for a shipped copy"""
    d = base_dir()
    for _ in range(5):
        if os.path.isfile(os.path.join(d, "s31-btstack-speaker", "CMakeLists.txt")):
            return d
        d = os.path.dirname(d)
    return None


def find_firmware(arg):
    """--firmware if given; inside the workspace the current build (exactly what the
    console flashes while testing); else the packaged ./firmware next to the tool"""
    if arg:
        return arg
    ws = workspace_root()
    if ws:
        build = os.path.join(ws, "s31-btstack-speaker", "build")
        if os.path.isfile(os.path.join(build, "flasher_args.json")):
            return build
    return os.path.join(base_dir(), "firmware")


def find_tones(fw_dir):
    """firmware/tones.bin, or the workspace's local tones image (re/local_tones)"""
    for p in (os.path.join(fw_dir, "tones.bin"),
              os.path.join(workspace_root() or "", "re", "local_tones", "tones.bin")):
        if os.path.isfile(p):
            return p
    return None


def crc16_arc(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def make_cfg(color_id, name):
    """the s31cfg partition image (see s31-btstack-speaker/main/s31_ident.c)"""
    nb = (name or "").encode("utf-8")[:NAME_MAX]
    body = b"S31C" + bytes([1, color_id & 0xFF, 0, len(nb)]) + nb.ljust(NAME_MAX, b"\0")
    body += struct.pack("<H", crc16_arc(body))
    return body.ljust(CFG_SIZE, b"\xff")


def load_package(fw_dir):
    path = os.path.join(fw_dir, "flasher_args.json")
    if not os.path.isfile(path):
        raise FileNotFoundError(f"no firmware package in {fw_dir} (flasher_args.json missing) - "
                                "build it with tools/flasher/make_package.py")
    with open(path, encoding="utf-8") as f:
        args = json.load(f)
    files = []
    for off, rel in args["flash_files"].items():
        p = os.path.join(fw_dir, rel)
        if not os.path.isfile(p):
            raise FileNotFoundError(f"{rel} is missing from {fw_dir}")
        files.append((off, p))
    tones = find_tones(fw_dir)
    if tones:
        files.append((hex(TONES_OFFSET), tones))
    return args, files


def run_esptool(argv, log):
    import esptool
    log("esptool " + " ".join(argv) + "\n")
    old_out, old_err = sys.stdout, sys.stderr

    class W(io.TextIOBase):
        def write(self, s):
            log(s)
            return len(s)

    sys.stdout = sys.stderr = W()
    try:
        esptool.main(argv)
    finally:
        sys.stdout, sys.stderr = old_out, old_err


def flash(port, color_id, name, fw_dir, erase, color_only, log):
    with tempfile.TemporaryDirectory() as tmp:
        cfg = os.path.join(tmp, "s31cfg.bin")
        with open(cfg, "wb") as f:
            f.write(make_cfg(color_id, name))
        if color_only:
            chip = "esp32s31"
            try:
                chip = load_package(fw_dir)[0]["extra_esptool_args"]["chip"]
            except Exception:
                pass
            run_esptool(["--chip", chip, "--port", port, "--no-stub", "write-flash", hex(CFG_OFFSET), cfg], log)
            log(f"\nColor set to 0x{color_id:02X}. The board restarts with it.\n")
            return
        args, files = load_package(fw_dir)
        extra = args.get("extra_esptool_args", {})
        base = ["--chip", extra.get("chip", "esp32s31"), "--port", port, "--baud", "921600",
                "--before", extra.get("before", "default-reset").replace("_", "-"),
                "--after", extra.get("after", "hard-reset").replace("_", "-")]
        if not extra.get("stub", True):
            base.append("--no-stub")
        if erase:
            ea = list(base)
            ea[ea.index("--after") + 1] = "no-reset"
            run_esptool(ea + ["erase-flash"], log)
        wf = [a.replace("_", "-") for a in args.get("write_flash_args", [])]
        pairs = []
        for off, p in files + [(hex(CFG_OFFSET), cfg)]:
            pairs += [off, p]
        run_esptool(base + ["write-flash"] + wf + pairs, log)
        log(f"\nDone: color 0x{color_id:02X}{', name ' + repr(name) if name else ''}. "
            "The board restarts as its own Xtreme 5.\n")


def ports():
    try:
        from serial.tools import list_ports
        return [p.device for p in list_ports.comports()]
    except Exception:
        return []


def color_from_args(a):
    if a.color_id is not None:
        return int(a.color_id, 0)
    for nm, cid, _ in COLORS:
        if nm.lower().startswith(a.color.lower()):
            return cid
    raise SystemExit(f"unknown color {a.color!r} - one of: " + ", ".join(c[0] for c in COLORS))


def gui(fw_dir):
    import tkinter as tk
    from tkinter import ttk, scrolledtext

    root = tk.Tk()
    root.title("S31 Flasher - JBL Xtreme 5")
    root.geometry("640x520")
    frm = ttk.Frame(root, padding=12)
    frm.pack(fill="both", expand=True)

    ttk.Label(frm, text="Port").grid(row=0, column=0, sticky="w")
    port_var = tk.StringVar()
    port_box = ttk.Combobox(frm, textvariable=port_var, values=ports(), width=18)
    port_box.grid(row=0, column=1, sticky="w")
    if port_box["values"]:
        port_box.current(0)
    ttk.Button(frm, text="Refresh", command=lambda: port_box.configure(values=ports())).grid(row=0, column=2, sticky="w")

    ttk.Label(frm, text="Color").grid(row=1, column=0, sticky="w", pady=(8, 0))
    names = [f"{n}  (0x{c:02X}{'' if ok else ', unconfirmed'})" for n, c, ok in COLORS]
    color_var = tk.StringVar(value=names[0])
    id_var = tk.StringVar(value="0x00")
    cb = ttk.Combobox(frm, textvariable=color_var, values=names, state="readonly", width=30)
    cb.grid(row=1, column=1, columnspan=2, sticky="w", pady=(8, 0))
    cb.bind("<<ComboboxSelected>>", lambda e: id_var.set(f"0x{COLORS[cb.current()][1]:02X}"))
    ttk.Label(frm, text="Color ID").grid(row=2, column=0, sticky="w")
    ttk.Entry(frm, textvariable=id_var, width=8).grid(row=2, column=1, sticky="w")

    ttk.Label(frm, text="Name (optional)").grid(row=3, column=0, sticky="w", pady=(8, 0))
    name_var = tk.StringVar()
    ttk.Entry(frm, textvariable=name_var, width=34).grid(row=3, column=1, columnspan=2, sticky="w", pady=(8, 0))

    erase_var = tk.BooleanVar(value=False)
    ttk.Checkbutton(frm, text="Erase the board first (only for a new board: forgets pairings and settings)",
                    variable=erase_var).grid(row=4, column=0, columnspan=3, sticky="w", pady=(8, 0))
    tones = find_tones(fw_dir)
    ttk.Label(frm, text=f"Firmware: {fw_dir}\nTones: {tones or 'none (the board keeps the ones it has)'}",
              justify="left").grid(row=5, column=0, columnspan=3, sticky="w", pady=(4, 0))

    out = scrolledtext.ScrolledText(frm, height=16, font=("Consolas", 9))
    out.grid(row=7, column=0, columnspan=3, sticky="nsew", pady=(8, 0))
    frm.rowconfigure(7, weight=1)
    frm.columnconfigure(2, weight=1)

    def log(s):
        root.after(0, lambda: (out.insert("end", s), out.see("end")))

    def go(color_only):
        port = port_var.get().strip()
        if not port:
            log("Pick a port first.\n")
            return
        try:
            cid = int(id_var.get(), 0)
        except ValueError:
            log("Color ID must be a number like 0x01.\n")
            return
        btns(False)

        def work():
            try:
                flash(port, cid, name_var.get().strip(), fw_dir, erase_var.get(), color_only, log)
            except SystemExit as e:
                log(f"\nesptool stopped ({e}).\n")
            except Exception as e:
                log(f"\nFAILED: {e}\n")
            finally:
                root.after(0, lambda: btns(True))

        threading.Thread(target=work, daemon=True).start()

    bar = ttk.Frame(frm)
    bar.grid(row=6, column=0, columnspan=3, sticky="w", pady=(8, 0))
    b1 = ttk.Button(bar, text="Flash", command=lambda: go(False))
    b2 = ttk.Button(bar, text="Set color only", command=lambda: go(True))
    b1.pack(side="left")
    b2.pack(side="left", padx=8)

    def btns(on):
        for b in (b1, b2):
            b.configure(state="normal" if on else "disabled")

    root.mainloop()


def main():
    ap = argparse.ArgumentParser(description="Flash an ESP32-S31 as its own JBL Xtreme 5")
    ap.add_argument("--port", help="serial port, e.g. COM5 (no port: open the window)")
    ap.add_argument("--color", default="black", help="black, white, blue, purple or camo")
    ap.add_argument("--color-id", help="raw JBL color ID, e.g. 0x01 (overrides --color)")
    ap.add_argument("--name", default="", help="default Bluetooth name (empty: S31 JBL Xtreme 5)")
    ap.add_argument("--firmware", help="firmware folder (default: this workspace's current build, "
                    "else ./firmware next to the tool)")
    ap.add_argument("--erase", action="store_true", help="erase the whole flash first")
    ap.add_argument("--color-only", action="store_true", help="only write the color / name")
    a = ap.parse_args()
    a.firmware = find_firmware(a.firmware)
    if not a.port:
        gui(a.firmware)
        return
    def cli_log(s):
        out = sys.__stdout__
        if out is not None:
            out.write(s)
            out.flush()

    flash(a.port, color_from_args(a), a.name, a.firmware, a.erase, a.color_only, cli_log)


if __name__ == "__main__":
    main()
