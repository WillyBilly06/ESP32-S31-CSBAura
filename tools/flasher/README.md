# S31 Flasher

`S31Flasher.exe` builds the firmware from this project folder and flashes an ESP32-S31 as a
JBL Xtreme 5. Everything happens in one window: pick the board, pick the speaker color,
press **Build + Flash**.

Everything it builds from comes with the repository, already patched:
- **ESP-IDF** in [`third_party/esp-idf`](../../third_party/esp-idf/S31_README.md): Espressif's
  commit [`idf-patches/IDF_COMMIT`](../../idf-patches/IDF_COMMIT) with
  [`idf-patches/0010`](../../idf-patches/0010-controller-only-iso-rx.patch) applied.
- **BTstack** in [`third_party/btstack`](../../third_party/btstack/S31_README.md).

Only ESP-IDF's tools (compiler, CMake, Ninja, a Python environment) can't live in the
repository. The first **Build + Flash** installs them by itself.

## What you need

- Windows 10 or 11.
- The whole repository: **Code → Download ZIP** on GitHub, unpacked, or `git clone`.
- `S31Flasher.exe` from the [Releases](../../../../releases) page.
- An ESP32-S31 board with 16 MB flash and PSRAM, and a USB data cable.
- About 2 GB of free disk space for ESP-IDF's tools, and internet for the first build.

You don't need git, and you don't need to install Python yourself. If the PC has no Python
3.10 to 3.13, the first build installs Python 3.12 from python.org for this user only, into
`%LOCALAPPDATA%\S31Flasher\python` (not on PATH, no file associations).

## First time

1. Unpack the repository to a **short path**, for example `C:\S31\ESP32-S31-CSBAura`.
   It must be under 80 characters, without spaces or accented letters. ESP-IDF's deepest
   files are 175 characters deep inside it, and Windows paths stop at 260.
2. Put `S31Flasher.exe` into that folder, next to `README.md` and `s31-btstack-speaker`.
3. Run it. If Windows SmartScreen warns about an unknown app, choose **More info → Run anyway**.
4. **1 Setup** says **Ready to build**, and whether ESP-IDF's tools are installed yet.
5. Flash a board (below). The first **Build + Flash** installs ESP-IDF's ESP32-S31 tools with
   ESP-IDF's own installer, into `%USERPROFILE%\.espressif` (about 1 GB downloaded, several
   minutes, once), then builds. **Install tools** does only that part, if you want it done
   before plugging in a board.

Already have this exact ESP-IDF somewhere? Open **Folders...** and pick it. The flasher
checks the commit and the patch.

## Flash a board

1. Plug in the board. If it doesn't show up, hold **BOOT**, press **RESET**, then release
   **BOOT**.
2. **2 Board**: pick its port (press **Refresh** after plugging it in).
3. **3 Speaker color**: click a color.
4. **4 Name** (optional): the Bluetooth name. Empty means `S31 JBL Xtreme 5`.
5. Press **Build + Flash**. The first build takes a few minutes; later ones are much faster.
   The progress bar and **Show log** show what it is doing.
6. The board restarts by itself. The first time, pair it from the phone; later flashes keep
   the pairing.

Close any serial monitor that has the board's port open before flashing: only one program
can use the port at a time.

**More** has:
- **Set color / name only**: rewrites only the color and name. It keeps the firmware, the
  pairings and the settings, and takes a few seconds.
- **Flash the last build**: flashes what was built before, without building again.

## Every board is its own Xtreme 5

Every ESP32-S31 has its own factory-burned Bluetooth MAC, and the firmware derives the whole
speaker identity from it. Boards never collide, even several flashed from the same build, and
a board keeps its identity and pairings across flashes:

| What | Comes from |
|---|---|
| Bluetooth address (Classic + LE) | the chip's factory MAC (unique per chip) |
| Beacon address and `macCrc`, which JBL One and JBL speakers use to tell speakers apart | that MAC |
| Serial number | `S31X5` + the last 3 MAC bytes, for example `S31X5F4084D` |
| Standard Auracast Broadcast_ID | derived from the MAC |
| Color shown in JBL One | picked here |
| Name | picked here, or `S31 JBL Xtreme 5`; JBL One can rename it later |

The flasher can't give one board a new Bluetooth address on each flash. The ESP32-S31
controller accepts another address, but with it the board loses its stored keys, phones
fail to reconnect, and LE Audio breaks.

The JBL party Broadcast_ID (`0x112233`) is the same on every board on purpose. Real Xtreme 5
speakers only join a party with that ID.

## Colors

JBL One reads the color from the speaker's BLE beacon. The IDs are JBL's own for the Xtreme 5
(its `model_config.json`, "official_color_name"):

| Color | ID |
|---|---|
| Black | `0x00` |
| White | `0x01` |
| Blue | `0x02` |
| Purple | `0x05` |
| Camo | `0x1A` |

## Prompt tones

No tones come with the project. If `tones.bin` is in the project folder (make it with
[`tools/tones_pack.py`](../tones_pack.py), see the main README), it is flashed too.
Otherwise the board keeps the tones it has.

## Troubleshooting

| Problem | Fix |
|---|---|
| No port in the list | Use a data cable (not charge-only), press **Refresh**, try BOOT + RESET. |
| `could not open port` / `Access is denied` | Another program (serial monitor, console) has the port open. Close it. |
| `Failed to connect to ESP32-S31` | Hold BOOT, press RESET, release BOOT, press **Build + Flash** again. |
| Installing the tools fails | Check the internet connection and press **Build + Flash** (or **Install tools**) again; it continues where it stopped. |
| `project folder's path is too long` | Move the repository to a shorter folder, for example `C:\S31\ESP32-S31-CSBAura`. |
| Build fails with a path error | Move the project to a short path without spaces, for example `C:\S31\...`. |
| Phone doesn't find or connect to the board | Press the pairing button (GPIO 42), then pick the speaker on the phone. If the phone had paired with it before an **Erase**, forget it on the phone first. |
| JBL One shows the wrong color | Use **Set color / name only** with the right color, then close and reopen JBL One. |

The full log is under **Show log**. Copy it when asking for help.

## Command line

`python tools/flasher/s31_flasher.py` does the same steps (the exe is window-only):

```
python tools/flasher/s31_flasher.py --setup
python tools/flasher/s31_flasher.py --port COM5 --color blue [--name "Party Box"] [--erase]
python tools/flasher/s31_flasher.py --port COM5 --color camo --color-only
python tools/flasher/s31_flasher.py --port COM5 --no-build
```

| Option | Does |
|---|---|
| `--port` | serial port, for example `COM5` |
| `--color` / `--color-id` | `black`, `white`, `blue`, `purple`, `camo`, or a raw ID like `0x1A` |
| `--name` | Bluetooth name |
| `--setup` | only install ESP-IDF's tools (a build does it too when they are missing) |
| `--no-build` | flash the last build |
| `--color-only` | write only color + name |
| `--erase` | erase the whole flash first (pairings, settings, tones) |
| `--idf`, `--project` | use these folders instead of finding them |

This needs `esptool` and `pyserial`; ESP-IDF's Python environment has both.

## For maintainers

Build the exe (esptool is taken from the ESP-IDF Python environment):

```
pip install pyinstaller
python tools/flasher/build_exe.py      # -> tools/flasher/dist/S31Flasher.exe
```

`third_party/esp-idf` and `third_party/btstack` are made by `tools/release/promote.py` from
git checkouts at the pinned commits with the patches applied. It exports git's own bytes
(not a CRLF working tree), leaves out what nothing builds from, and checks that the result is
exactly upstream plus our patches.
