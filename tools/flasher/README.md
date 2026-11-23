# S31 Flasher

Flashes an ESP32-S31 as its own JBL Xtreme 5: one window, pick the port, the color and
(optionally) a name, press **Flash**. No ESP-IDF or Python needed on the PC that flashes.

Every board comes out as a different speaker on its own:

| What | Where it comes from |
|---|---|
| Bluetooth address (Classic + LE) | the chip's factory MAC (unique per chip) |
| Beacon `macCrc` (JBL One / JBL speakers tell speakers apart by it) | CRC-16/ARC of that MAC |
| Serial number | `S31X5` + the last 3 MAC bytes, e.g. `S31X5F4084D` |
| Standard Auracast Broadcast_ID | derived from the MAC |
| Color shown in JBL One | picked here (written to the `s31cfg` partition) |
| Name | picked here, or `S31 JBL Xtreme 5`; JBL One can rename it later |

The JBL party Broadcast_ID (`0x112233`) is the same on every board on purpose: every real
Xtreme 5 uses it.

## Colors

JBL One reads the color from the speaker's BLE beacon and shows the matching picture.
The IDs are JBL's own for the Xtreme 5 (its `model_config.json`, "official_color_name"):

| Color | ID |
|---|---|
| Black | `0x00` |
| White | `0x01` |
| Blue | `0x02` |
| Purple | `0x05` |
| Camo | `0x1A` |

**Set color only** rewrites just the color / name (keeps pairings and settings), handy to
try an ID: the board restarts and JBL One shows the new picture.

## Use

```
S31Flasher.exe                                     window
S31Flasher.exe --port COM5 --color blue --erase    new board, blue
S31Flasher.exe --port COM5 --color-id 0x03 --color-only
```

`--erase` (the window's "Erase the board first") wipes the flash first: use it for a new
board; it forgets phones and JBL One settings.

The firmware comes from the `firmware` folder next to the exe. Prompt tones are not part
of it (they come from a real speaker's firmware): put your own `tones.bin` (made with
`tools/tones_pack.py`) into `firmware/` and the flasher writes it too.

## Build it (developers)

1. Build the firmware: `idf.py -C s31-btstack-speaker build`
2. `python tools/flasher/make_package.py` -> `tools/flasher/firmware/`
3. `python tools/flasher/build_exe.py` -> `tools/flasher/dist/S31Flasher.exe`
   (PyInstaller; esptool comes from the ESP-IDF Python environment)

Ship `S31Flasher.exe` and the `firmware` folder together (zip them for a release).
