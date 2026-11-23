# S31 Flasher

Builds the firmware from this project folder and flashes an ESP32-S31 as its own JBL
Xtreme 5, in one window: pick the board, pick the speaker color, press **Build + Flash**.

It builds with the **exact ESP-IDF this project uses**: Espressif's ESP-IDF at the commit in
[`idf-patches/IDF_COMMIT`](../../idf-patches/IDF_COMMIT) with
[`idf-patches/0010`](../../idf-patches/0010-controller-only-iso-rx.patch) applied. If it does
not find that on the PC, **Set up ESP-IDF + BTstack** downloads and prepares it (once; a few GB):
ESP-IDF at that commit + the patch + the ESP32-S31 tools, and BTstack at
[`btstack-patches/BTSTACK_COMMIT`](../../btstack-patches/BTSTACK_COMMIT) + its patches into
`third_party/btstack`. Needs [git](https://git-scm.com) and Python 3.10+ installed.

## Use

1. Put `S31Flasher.exe` in the project folder (or open **Folders...** and point it there).
2. **1 Setup**: "Ready to build", or press **Set up ESP-IDF + BTstack**.
3. **2 Board**: the board's port. Tick "erase it first" only for a new board.
4. **3 Speaker color**: click the color.
5. **4 Name**: optional.
6. **Build + Flash**.

**More** has **Set color / name only** (keeps the firmware, pairings and settings) and
**Flash the last build**.

## Every board is its own Xtreme 5

| What | Where it comes from |
|---|---|
| Bluetooth address (Classic + LE) | the chip's factory MAC (unique per chip) |
| Beacon `macCrc` (how JBL One and JBL speakers tell speakers apart) | CRC-16/ARC of that MAC |
| Serial number | `S31X5` + the last 3 MAC bytes, e.g. `S31X5F4084D` |
| Standard Auracast Broadcast_ID | derived from the MAC |
| Color shown in JBL One | picked here (written to the `s31cfg` partition) |
| Name | picked here, or `S31 JBL Xtreme 5`; JBL One can rename it later |

The JBL party Broadcast_ID (`0x112233`) is the same on every board on purpose: every real
Xtreme 5 uses it.

## Colors

JBL One reads the color from the speaker's BLE beacon. The IDs are JBL's own for the
Xtreme 5 (its `model_config.json`, "official_color_name"):

| Color | ID |
|---|---|
| Black | `0x00` |
| White | `0x01` |
| Blue | `0x02` |
| Purple | `0x05` |
| Camo | `0x1A` |

## Prompt tones

Not part of the project (they come from a real speaker's firmware). If
`re/local_tones/tones.bin` or `tones.bin` exists in the project folder, it is flashed too
(make it with `tools/tones_pack.py`); otherwise the board keeps the tones it has.

## Command line

`python tools/flasher/s31_flasher.py` takes the same steps (the exe is a window only):

```
python tools/flasher/s31_flasher.py --setup
python tools/flasher/s31_flasher.py --port COM5 --color blue [--name "Party Box"] [--erase]
python tools/flasher/s31_flasher.py --port COM5 --color camo --color-only
python tools/flasher/s31_flasher.py --port COM5 --no-build
```

(needs `esptool` and `pyserial`, e.g. ESP-IDF's Python environment)

## Build the exe

```
pip install pyinstaller
python tools/flasher/build_exe.py      # -> tools/flasher/dist/S31Flasher.exe
```

esptool is taken from the ESP-IDF Python environment.
