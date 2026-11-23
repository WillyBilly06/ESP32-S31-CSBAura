"""S31 Flasher - build the firmware and flash an ESP32-S31 as its own JBL Xtreme 5.

It builds s31-btstack-speaker from the project folder with the exact patched ESP-IDF this
project uses (ESP-IDF commit idf-patches/IDF_COMMIT + idf-patches/0010), then flashes it
with the board's color and name. It can set up that ESP-IDF and BTstack itself.

Every board comes out unique on its own (serial, Broadcast_ID and Bluetooth identity come
from the chip's factory MAC); the color and name go to the board's "s31cfg" partition.

Window:   S31Flasher.exe          (put it in the project folder, or browse to it)
Command:  S31Flasher.exe --port COM5 --color blue [--name "Party Box"] [--erase]
          S31Flasher.exe --setup                  (ESP-IDF + BTstack, once)
          S31Flasher.exe --port COM5 --color-id 0x1A --color-only
          S31Flasher.exe --port COM5 --no-build    (flash the last build)

Prompt tones: re/local_tones/tones.bin or <build>/tones.bin is written too if present.
"""
import argparse
import glob
import io
import json
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import threading

# JBL color IDs of the Xtreme 5, from JBL's own model config for product 2169
# ("official_color_name" in appstorage.onecloud.harman.com/jbl_one/app_config/v11/
# portable/2169/model_config.json). JBL One reads the ID from the BLE beacon (colorID).
COLORS = [
    ("Black", 0x00),
    ("White", 0x01),
    ("Blue", 0x02),
    ("Purple", 0x05),
    ("Camo", 0x1A),
]
CFG_OFFSET = 0xA10000          # partitions.csv "s31cfg"
CFG_SIZE = 0x1000
TONES_OFFSET = 0x810000        # partitions.csv "tones"
NAME_MAX = 32
PROJECT_SUBDIR = "s31-btstack-speaker"
IDF_URL = "https://github.com/espressif/esp-idf.git"
BTSTACK_URL = "https://github.com/bluekitchen/btstack.git"
WIN = os.name == "nt"


# ---------------------------------------------------------------- folders / settings
def base_dir():
    """folder of the .exe (frozen) or of this script"""
    if getattr(sys, "frozen", False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(__file__))


def is_project(d):
    return bool(d) and os.path.isfile(os.path.join(d, PROJECT_SUBDIR, "CMakeLists.txt"))


def find_project_near(d):
    d = os.path.abspath(d)
    for _ in range(5):
        if is_project(d):
            return d
        d = os.path.dirname(d)
    return None


def settings_path():
    return os.path.join(os.environ.get("APPDATA") or os.path.expanduser("~"), "S31Flasher", "settings.json")


def load_settings():
    try:
        with open(settings_path(), encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def save_settings(s):
    try:
        os.makedirs(os.path.dirname(settings_path()), exist_ok=True)
        with open(settings_path(), "w", encoding="utf-8") as f:
            json.dump(s, f, indent=2)
    except Exception:
        pass


def project_file(project, *parts):
    return os.path.join(project, *parts)


def idf_commit(project):
    with open(project_file(project, "idf-patches", "IDF_COMMIT"), encoding="utf-8") as f:
        return f.read().strip()


def idf_patches(project):
    """the ESP-IDF patches this firmware is built with (only 0010 today)"""
    return [project_file(project, "idf-patches", "0010-controller-only-iso-rx.patch")]


def default_idf_dir():
    return os.path.join(os.path.expanduser("~"), "esp", "esp-idf-s31")


def git(args, cwd=None):
    return subprocess.run(["git"] + args, cwd=cwd, capture_output=True, text=True,
                          creationflags=subprocess.CREATE_NO_WINDOW if WIN else 0)


def idf_state(idf, project):
    """'ready', or what is wrong with this ESP-IDF folder"""
    if not idf or not os.path.isfile(os.path.join(idf, "tools", "idf.py")):
        return "not set up"
    head = git(["rev-parse", "HEAD"], idf).stdout.strip()
    if head != idf_commit(project):
        return f"wrong commit ({head[:10] or '?'}; this project needs {idf_commit(project)[:10]})"
    for p in idf_patches(project):
        if git(["apply", "--check", "-R", p], idf).returncode != 0:
            return f"patch {os.path.basename(p)} not applied"
    return "ready"


def find_idf(project, arg=None):
    """an ESP-IDF folder at this project's commit with its patches: --idf, the saved one,
    IDF_PATH, the default install, then common places"""
    cands = [arg, load_settings().get("idf_path"), os.environ.get("IDF_PATH"), default_idf_dir()]
    for pat in ("~/esp/esp-idf*", "C:/Espressif/frameworks/esp-idf*", "D:/*/esp-idf*", "C:/*/esp-idf*", "E:/*/esp-idf*"):
        cands += sorted(glob.glob(os.path.expanduser(pat)))
    first = None
    for c in cands:
        if not c or not os.path.isdir(c):
            continue
        first = first or c
        if idf_state(c, project) == "ready":
            return c
    return arg or first or default_idf_dir()


def btstack_dir(project):
    return project_file(project, "third_party", "btstack")


def btstack_ready(project):
    return os.path.isfile(os.path.join(btstack_dir(project), "src", "hci.c"))


def find_tones(project, build):
    for p in (os.path.join(build, "tones.bin"),
              project_file(project, "re", "local_tones", "tones.bin"),
              project_file(project, "tones.bin")):
        if os.path.isfile(p):
            return p
    return None


# ---------------------------------------------------------------- running things
def clean_env(extra=None):
    """this process's environment without what PyInstaller adds for itself (a frozen exe
    must not hand its bundled Python / DLL paths to ESP-IDF's tools)"""
    # also no MSYS / Git Bash markers: ESP-IDF refuses to run when it sees MSYSTEM
    env = {k: v for k, v in os.environ.items()
           if not k.startswith(("_MEI", "_PYI", "PYTHONHOME", "PYTHONPATH", "MSYSTEM", "MINGW"))}
    mei = getattr(sys, "_MEIPASS", None)
    if mei:
        env["PATH"] = os.pathsep.join(x for x in env.get("PATH", "").split(os.pathsep)
                                      if x and not os.path.normcase(x).startswith(os.path.normcase(mei)))
    env["PYTHONUNBUFFERED"] = "1"
    env["PYTHONIOENCODING"] = "utf-8"
    env.update(extra or {})
    return env


def stream(cmd, log, cwd=None, env=None):
    """run cmd, send its output to log line by line, raise on failure"""
    log("> " + (cmd if isinstance(cmd, str) else " ".join(cmd)) + "\n")
    p = subprocess.Popen(cmd, cwd=cwd, env=env if env is not None else clean_env(),
                         stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, encoding="utf-8", errors="replace", bufsize=1,
                         creationflags=subprocess.CREATE_NO_WINDOW if WIN else 0)
    for line in p.stdout:
        log(line)
    if p.wait() != 0:
        raise RuntimeError(f"failed (exit {p.returncode})")


def idf_env(idf):
    """the variables ESP-IDF's export script expects (as the dev console sets them)"""
    return clean_env({"IDF_PATH": os.path.normpath(idf),
                      "IDF_TOOLS_PATH": os.path.join(os.path.expanduser("~"), ".espressif"),
                      "IDF_TARGET": "esp32s31"})


def in_idf_env(idf, command):
    """a shell command line that runs `command` with ESP-IDF's environment (IDF_PATH set)"""
    if WIN:
        ps = ("$ErrorActionPreference='Continue'; "
              "try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch {}; "
              '. "$env:IDF_PATH\\export.ps1" | Out-Null; '
              "if (-not (Get-Command idf.py -ErrorAction SilentlyContinue)) "
              "{ Write-Output 'ESP-IDF environment did not load (export.ps1) - run Set up'; exit 96 }; "
              f"{command}; if ($null -eq $LASTEXITCODE) {{ exit 97 }} else {{ exit $LASTEXITCODE }}")
        return ["powershell.exe", "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-Command", ps]
    return ["bash", "-c", f'. "$IDF_PATH/export.sh" >/dev/null && {command}']


def need_git(log):
    if shutil.which("git") is None:
        raise RuntimeError("git is not installed - get it from https://git-scm.com and try again")


def setup_idf(project, idf, log):
    """this project's ESP-IDF: Espressif's commit + our patch + the ESP32-S31 tools"""
    need_git(log)
    commit = idf_commit(project)
    if not os.path.isfile(os.path.join(idf, "tools", "idf.py")):
        if os.path.isdir(idf) and os.listdir(idf):
            raise RuntimeError(f"{idf} is not empty and not an ESP-IDF folder - pick another one")
        log(f"Downloading ESP-IDF {commit[:10]} into {idf} (about 1-2 GB, takes a while)...\n")
        os.makedirs(idf, exist_ok=True)
        stream(["git", "init", "-q"], log, cwd=idf)
        stream(["git", "remote", "add", "origin", IDF_URL], log, cwd=idf)
    stream(["git", "fetch", "--depth", "1", "origin", commit], log, cwd=idf)
    stream(["git", "checkout", "-q", "-f", commit], log, cwd=idf)
    stream(["git", "submodule", "update", "--init", "--recursive", "--depth", "1"], log, cwd=idf)
    for p in idf_patches(project):
        if git(["apply", "--check", "-R", p], idf).returncode == 0:
            log(f"{os.path.basename(p)}: already applied\n")
        else:
            stream(["git", "apply", p], log, cwd=idf)
    log("Installing the ESP32-S31 tools (compiler, CMake, Python environment)...\n")
    if WIN:
        stream(["powershell.exe", "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-File",
                os.path.join(idf, "install.ps1"), "esp32s31"], log, cwd=idf, env=idf_env(idf))
    else:
        stream(["bash", os.path.join(idf, "install.sh"), "esp32s31"], log, cwd=idf, env=idf_env(idf))
    s = load_settings()
    s["idf_path"] = idf
    save_settings(s)
    log("ESP-IDF ready.\n")


def setup_btstack(project, log):
    """BTstack at the pinned commit with this project's patches (third_party/btstack)"""
    need_git(log)
    d = btstack_dir(project)
    with open(project_file(project, "btstack-patches", "BTSTACK_COMMIT"), encoding="utf-8") as f:
        commit = f.read().strip()
    if not os.path.isdir(os.path.join(d, ".git")):
        os.makedirs(d, exist_ok=True)
        stream(["git", "init", "-q"], log, cwd=d)
        stream(["git", "remote", "add", "origin", BTSTACK_URL], log, cwd=d)
    stream(["git", "fetch", "--depth", "1", "origin", commit], log, cwd=d)
    stream(["git", "checkout", "-q", "-f", commit], log, cwd=d)
    for p in sorted(glob.glob(project_file(project, "btstack-patches", "*.patch"))):
        if git(["apply", "--check", "-R", p], d).returncode == 0:
            log(f"{os.path.basename(p)}: already applied\n")
        else:
            stream(["git", "apply", p], log, cwd=d)
    log("BTstack ready.\n")


def build(project, idf, log):
    state = idf_state(idf, project)
    if state != "ready":
        raise RuntimeError(f"ESP-IDF at {idf}: {state} - run Set up first")
    if not btstack_ready(project):
        raise RuntimeError("BTstack is missing (third_party/btstack) - run Set up first")
    proj = project_file(project, PROJECT_SUBDIR)
    log(f"Building {proj} with ESP-IDF {idf}...\n")
    stream(in_idf_env(idf, "idf.py --ccache build"), log, cwd=proj, env=idf_env(idf))


# ---------------------------------------------------------------- flashing
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


def load_build(build_dir):
    path = os.path.join(build_dir, "flasher_args.json")
    if not os.path.isfile(path):
        raise FileNotFoundError(f"no build in {build_dir} - build first")
    with open(path, encoding="utf-8") as f:
        args = json.load(f)
    files = []
    for off, rel in args["flash_files"].items():
        p = os.path.join(build_dir, rel)
        if not os.path.isfile(p):
            raise FileNotFoundError(f"{rel} is missing from {build_dir}")
        files.append((off, p))
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


def flash(project, port, color_id, name, erase, color_only, log):
    build_dir = project_file(project, PROJECT_SUBDIR, "build")
    with tempfile.TemporaryDirectory() as tmp:
        cfg = os.path.join(tmp, "s31cfg.bin")
        with open(cfg, "wb") as f:
            f.write(make_cfg(color_id, name))
        if color_only:
            run_esptool(["--chip", "esp32s31", "--port", port, "--no-stub", "write-flash", hex(CFG_OFFSET), cfg], log)
            log(f"\nColor set to 0x{color_id:02X}. The board restarts with it.\n")
            return
        args, files = load_build(build_dir)
        tones = find_tones(project, build_dir)
        if tones:
            files.append((hex(TONES_OFFSET), tones))
        log(f"Prompt tones: {tones or 'none found (the board keeps the ones it has)'}\n")
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
    for nm, cid in COLORS:
        if nm.lower().startswith(a.color.lower()):
            return cid
    raise SystemExit(f"unknown color {a.color!r} - one of: " + ", ".join(c[0] for c in COLORS))


# ---------------------------------------------------------------- window
SWATCH = {            # how each color is drawn in the window
    "Black": ("#1c1c1e", None),
    "White": ("#f2f2f2", None),
    "Blue": ("#1f5fbf", None),
    "Purple": ("#6b3fa0", None),
    "Camo": ("#5b6b45", ["#3f4a2f", "#8a8458", "#2c3324", "#6f7d52"]),
}


def gui(project, idf):
    import random
    import tkinter as tk
    from tkinter import ttk, scrolledtext, filedialog

    BG, CARD, INK, MUTED, ACCENT, OK, BAD = "#f4f5f7", "#ffffff", "#1d1d1f", "#6e6e73", "#ff6a00", "#1e8e3e", "#c5221f"
    root = tk.Tk()
    root.title("S31 Flasher - JBL Xtreme 5")
    root.geometry("820x720")
    root.minsize(720, 560)
    root.configure(bg=BG)
    st = ttk.Style(root)
    try:
        st.theme_use("clam")
    except tk.TclError:
        pass
    st.configure(".", background=BG, foreground=INK, font=("Segoe UI", 10))
    st.configure("Card.TFrame", background=CARD)
    st.configure("Card.TLabel", background=CARD, foreground=INK)
    st.configure("Muted.TLabel", background=CARD, foreground=MUTED, font=("Segoe UI", 9))
    st.configure("Step.TLabel", background=CARD, foreground=INK, font=("Segoe UI Semibold", 11))
    st.configure("Title.TLabel", background=BG, foreground=INK, font=("Segoe UI Semibold", 16))
    st.configure("Sub.TLabel", background=BG, foreground=MUTED)
    st.configure("Card.TCheckbutton", background=CARD)
    st.configure("Go.TButton", font=("Segoe UI Semibold", 12), padding=(18, 10), foreground="white", background=ACCENT)
    st.map("Go.TButton", background=[("disabled", "#d0d0d0"), ("active", "#e85f00")])
    st.configure("TProgressbar", background=ACCENT)

    proj_var = tk.StringVar(value=project or "")
    idf_var = tk.StringVar(value=idf or default_idf_dir())
    port_var = tk.StringVar()
    name_var = tk.StringVar()
    erase_var = tk.BooleanVar(value=False)
    color_var = tk.StringVar(value=COLORS[0][0])
    status_var = tk.StringVar(value="Ready.")
    setup_var = tk.StringVar()

    outer = ttk.Frame(root, padding=(18, 14))
    outer.pack(fill="both", expand=True)
    ttk.Label(outer, text="S31 Flasher", style="Title.TLabel").pack(anchor="w")
    ttk.Label(outer, text="Builds the firmware and turns this ESP32-S31 into its own JBL Xtreme 5.",
              style="Sub.TLabel").pack(anchor="w", pady=(0, 10))

    def card(step, title):
        f = ttk.Frame(outer, style="Card.TFrame", padding=(14, 10))
        f.pack(fill="x", pady=(0, 8))
        ttk.Label(f, text=f"{step}  {title}", style="Step.TLabel").grid(row=0, column=0, columnspan=4, sticky="w")
        f.columnconfigure(1, weight=1)
        return f

    # 1 - setup
    c1 = card("1", "Setup")
    setup_lbl = ttk.Label(c1, textvariable=setup_var, style="Card.TLabel")
    setup_lbl.grid(row=1, column=0, columnspan=3, sticky="w", pady=(4, 0))
    setup_btn = ttk.Button(c1, text="Set up ESP-IDF + BTstack")
    setup_btn.grid(row=1, column=3, sticky="e", pady=(4, 0))
    adv = ttk.Frame(c1, style="Card.TFrame")
    adv_shown = tk.BooleanVar(value=False)

    def browse(var, title):
        d = filedialog.askdirectory(title=title, initialdir=var.get() or None)
        if d:
            var.set(os.path.normpath(d))
            refresh_status()

    for r, (lbl, var) in enumerate((("Project folder", proj_var), ("ESP-IDF folder", idf_var))):
        ttk.Label(adv, text=lbl, style="Muted.TLabel").grid(row=r, column=0, sticky="w", padx=(0, 8))
        ttk.Entry(adv, textvariable=var).grid(row=r, column=1, sticky="ew", pady=2)
        ttk.Button(adv, text="Browse", command=lambda v=var, t=lbl: browse(v, t)).grid(row=r, column=2, padx=(6, 0))
    adv.columnconfigure(1, weight=1)

    def toggle_adv():
        if adv_shown.get():
            adv.grid_remove()
            adv_btn.configure(text="Folders...")
        else:
            adv.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(8, 0))
            adv_btn.configure(text="Hide folders")
        adv_shown.set(not adv_shown.get())

    adv_btn = ttk.Button(c1, text="Folders...", command=toggle_adv)
    adv_btn.grid(row=1, column=2, sticky="e", padx=(0, 8), pady=(4, 0))

    def refresh_status(*_):
        p, i = proj_var.get(), idf_var.get()
        if not is_project(p):
            setup_var.set("Project folder not found - open Folders... and pick the folder with s31-btstack-speaker")
            setup_lbl.configure(foreground=BAD)
            if not adv_shown.get():
                toggle_adv()
            return False
        ist, bt = idf_state(i, p), btstack_ready(p)
        ready = ist == "ready" and bt
        setup_var.set(("Ready to build" if ready else "Needs setup") +
                      f"   -   ESP-IDF: {ist}   -   BTstack: {'ready' if bt else 'missing'}")
        setup_lbl.configure(foreground=OK if ready else BAD)
        return ready

    # 2 - board
    c2 = card("2", "Board")
    ttk.Label(c2, text="Port", style="Card.TLabel").grid(row=1, column=0, sticky="w", pady=(4, 0), padx=(0, 8))
    pf = ttk.Frame(c2, style="Card.TFrame")
    pf.grid(row=1, column=1, columnspan=3, sticky="w", pady=(4, 0))
    port_box = ttk.Combobox(pf, textvariable=port_var, values=ports(), width=14)
    port_box.pack(side="left")
    ttk.Button(pf, text="Refresh", command=lambda: port_box.configure(values=ports())).pack(side="left", padx=(8, 0))
    last_port = load_settings().get("port")
    if last_port in port_box["values"]:
        port_var.set(last_port)
    elif port_box["values"]:
        port_box.current(0)
    ttk.Checkbutton(c2, text="New board: erase it first (forgets pairings and settings)", variable=erase_var,
                    style="Card.TCheckbutton").grid(row=2, column=0, columnspan=4, sticky="w", pady=(6, 0))

    # 3 - color swatches
    c3 = card("3", "Speaker color")
    ttk.Label(c3, text="The color the JBL One app shows for this speaker.", style="Muted.TLabel").grid(
        row=1, column=0, columnspan=4, sticky="w")
    sw_row = tk.Frame(c3, bg=CARD)
    sw_row.grid(row=2, column=0, columnspan=4, sticky="w", pady=(8, 2))
    tiles = {}

    def draw_tile(cv, cname, selected):
        cv.delete("all")
        base, spots = SWATCH[cname]
        if selected:
            cv.create_oval(6, 2, 78, 74, outline=ACCENT, width=4)
        cv.create_oval(12, 8, 72, 68, fill=base, outline="#c7c7cc" if cname == "White" else base)
        if spots:
            rnd = random.Random(7)
            for _ in range(14):
                x, y, r = rnd.randint(16, 64), rnd.randint(12, 60), rnd.randint(4, 9)
                if (x - 42) ** 2 + (y - 38) ** 2 < 26 ** 2:
                    cv.create_oval(x - r, y - r, x + r, y + r * 0.7, fill=rnd.choice(spots), width=0)
        cv.create_text(42, 88, text=cname, fill=ACCENT if selected else INK,
                       font=("Segoe UI", 9, "bold" if selected else "normal"))

    def pick(cname):
        color_var.set(cname)
        for n, cv in tiles.items():
            draw_tile(cv, n, n == cname)

    for i, (cname, _cid) in enumerate(COLORS):
        cv = tk.Canvas(sw_row, width=84, height=98, bg=CARD, highlightthickness=0, cursor="hand2")
        cv.grid(row=0, column=i, padx=(0, 10))
        cv.bind("<Button-1>", lambda e, n=cname: pick(n))
        tiles[cname] = cv
    pick(COLORS[0][0])

    # 4 - name
    c4 = card("4", "Name (optional)")
    ttk.Entry(c4, textvariable=name_var, width=36).grid(row=1, column=0, sticky="w", pady=(4, 0))
    ttk.Label(c4, text="Empty: S31 JBL Xtreme 5. JBL One can rename it later.", style="Muted.TLabel").grid(
        row=1, column=1, sticky="w", padx=(10, 0), pady=(4, 0))

    # actions + progress + log
    act = ttk.Frame(outer)
    act.pack(fill="x", pady=(6, 4))
    go_btn = ttk.Button(act, text="Build + Flash", style="Go.TButton")
    go_btn.pack(side="left")
    more = ttk.Menubutton(act, text="More")
    menu = tk.Menu(more, tearoff=False)
    more["menu"] = menu
    more.pack(side="left", padx=10)
    prog = ttk.Progressbar(act, mode="indeterminate", length=180)   # shown only while working
    ttk.Label(outer, textvariable=status_var, style="Sub.TLabel").pack(anchor="w")
    log_shown = tk.BooleanVar(value=False)
    log_btn = ttk.Button(outer, text="Show details")
    log_btn.pack(anchor="w", pady=(4, 0))
    out = scrolledtext.ScrolledText(outer, height=12, font=("Consolas", 9), bg="#111", fg="#ddd", insertbackground="#ddd")

    def toggle_log():
        if log_shown.get():
            out.pack_forget()
            log_btn.configure(text="Show details")
        else:
            out.pack(fill="both", expand=True, pady=(4, 0))
            log_btn.configure(text="Hide details")
        log_shown.set(not log_shown.get())

    log_btn.configure(command=toggle_log)

    def log(s):
        def put():
            out.insert("end", s)
            out.see("end")
            line = s.strip().splitlines()[-1] if s.strip() else ""
            if line.startswith(("Building", "Downloading", "Installing", "Writing at", "Wrote", "Done", "Color set",
                                "ESP-IDF ready", "BTstack ready", "FAILED", "Prompt tones")):
                status_var.set(line[:110])
        root.after(0, put)

    busy_widgets = [go_btn, more, setup_btn]

    def run(job, what):
        for w in busy_widgets:
            w.configure(state="disabled")
        prog.pack(side="right")
        prog.start(12)
        status_var.set(what + "...")

        def work():
            ok = False
            try:
                job()
                ok = True
            except SystemExit as e:
                log(f"\nesptool stopped ({e}).\n")
            except Exception as e:
                log(f"\nFAILED: {e}\n")
            finally:
                def done():
                    prog.stop()
                    prog.pack_forget()
                    for w in busy_widgets:
                        w.configure(state="normal")
                    refresh_status()
                    if not ok:
                        status_var.set("Failed - see the details below.")
                        if not log_shown.get():
                            toggle_log()
                root.after(0, done)

        threading.Thread(target=work, daemon=True).start()

    def color_id():
        return dict(COLORS)[color_var.get()]

    def needs_port():
        port = port_var.get().strip()
        if not port:
            status_var.set("Pick the board's port first (step 2).")
        return port or None

    def remember():
        s = load_settings()
        s["idf_path"] = idf_var.get()
        s["project"] = proj_var.get()
        if port_var.get().strip():
            s["port"] = port_var.get().strip()
        save_settings(s)

    def do_setup():
        p, i = proj_var.get(), idf_var.get()
        if not is_project(p):
            refresh_status()
            return
        remember()

        def job():
            if idf_state(i, p) != "ready":
                setup_idf(p, i, log)
            if not btstack_ready(p):
                setup_btstack(p, log)
            log("Setup complete.\n")
        run(job, "Setting up ESP-IDF and BTstack (the first time downloads a few GB)")

    def do_flash(do_build):
        port = needs_port()
        if not port or not is_project(proj_var.get()):
            refresh_status()
            return
        if do_build and not refresh_status():
            status_var.set("Run step 1 (Set up) first.")
            return
        remember()
        p, i, cid, nm, er = proj_var.get(), idf_var.get(), color_id(), name_var.get().strip(), erase_var.get()

        def job():
            if do_build:
                build(p, i, log)
            flash(p, port, cid, nm, er, False, log)
        run(job, f"{'Building and flashing' if do_build else 'Flashing'} ({color_var.get()})")

    def do_color():
        port = needs_port()
        if port:
            remember()
            cid, nm = color_id(), name_var.get().strip()
            run(lambda: flash(proj_var.get(), port, cid, nm, False, True, log), f"Setting the color to {color_var.get()}")

    go_btn.configure(command=lambda: do_flash(True))
    setup_btn.configure(command=do_setup)
    menu.add_command(label="Set color / name only (keeps everything else)", command=do_color)
    menu.add_command(label="Flash the last build (no build)", command=lambda: do_flash(False))

    saved = load_settings()
    if not project and saved.get("project"):
        proj_var.set(saved["project"])
    refresh_status()
    root.mainloop()


def main():
    ap = argparse.ArgumentParser(description="Build and flash an ESP32-S31 as its own JBL Xtreme 5")
    ap.add_argument("--port", help="serial port, e.g. COM5 (no port and no --setup: open the window)")
    ap.add_argument("--color", default="black", help="black, white, blue, purple or camo")
    ap.add_argument("--color-id", help="raw JBL color ID, e.g. 0x1A (overrides --color)")
    ap.add_argument("--name", default="", help="default Bluetooth name (empty: S31 JBL Xtreme 5)")
    ap.add_argument("--project", help="project folder (default: the one this tool sits in)")
    ap.add_argument("--idf", help="ESP-IDF folder (default: found automatically, or ~/esp/esp-idf-s31)")
    ap.add_argument("--setup", action="store_true", help="set up this project's ESP-IDF and BTstack")
    ap.add_argument("--no-build", action="store_true", help="flash the last build without building")
    ap.add_argument("--erase", action="store_true", help="erase the whole flash first")
    ap.add_argument("--color-only", action="store_true", help="only write the color / name")
    a = ap.parse_args()
    project = a.project or find_project_near(base_dir()) or find_project_near(os.getcwd())
    idf = find_idf(project, a.idf) if is_project(project) else (a.idf or default_idf_dir())
    if not a.port and not a.setup:
        gui(project, idf)
        return
    if not is_project(project):
        raise SystemExit("no project folder found - pass --project <folder with s31-btstack-speaker>")

    def cli_log(s):
        out = sys.__stdout__
        if out is not None:
            out.write(s)
            out.flush()

    if a.setup:
        if idf_state(idf, project) != "ready":
            setup_idf(project, idf, cli_log)
        if not btstack_ready(project):
            setup_btstack(project, cli_log)
        cli_log(f"ESP-IDF: {idf_state(idf, project)} ({idf}) | BTstack: {'ready' if btstack_ready(project) else 'missing'}\n")
        if not a.port:
            return
    if not a.color_only and not a.no_build:
        build(project, idf, cli_log)
    flash(project, a.port, color_from_args(a), a.name, a.erase, a.color_only, cli_log)


if __name__ == "__main__":
    main()
