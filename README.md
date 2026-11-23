# ESP32-S31 CSBAura

Firmware that turns an **ESP32-S31** board into a speaker that behaves like a **JBL Xtreme 5**:
phones stream to it over Classic Bluetooth or LE Audio, real Xtreme 5 speakers join its Auracast
broadcast with their own Auracast button (JBL party), it can join another JBL party master, and
the JBL One app sees it as an Xtreme 5.

Every flashed board is **its own Xtreme 5**: address, serial number, beacon identity and
standard Auracast Broadcast_ID all come from the chip's factory MAC, and the color shown in JBL
One is picked when flashing.

## Quick start: flash a board

1. Get `S31Flasher.exe` and its `firmware` folder (a release zip, or build them, see below).
2. Connect the board over USB and run `S31Flasher.exe`.
3. Pick the port and the color, optionally a name, tick **Erase the board first** for a new
   board, press **Flash**.

| Color | ID |
|---|---|
| Black | `0x00` |
| White | `0x01` |
| Blue | `0x02` |
| Purple | `0x05` |
| Camo | `0x1A` |

**Set color only** changes the color / name of a flashed board and keeps its pairings.
Details: [`tools/flasher/README.md`](tools/flasher/README.md).

## Features

| Feature | What it does | Source |
|---|---|---|
| A2DP sink | SBC or AAC-LC from the phone, resampled to 48 kHz with clock recovery; AVRCP volume, play/pause, track info | `a2dp_in.c` |
| LE Audio sink | BAP unicast (LC3) with volume control; Play over LE media control after a switch from A2DP | `le_audio.c` |
| Auracast (JBL party host) | Broadcast with the advertising and BIG layout of a real Xtreme 5 master: Xtreme 5 speakers join with their own Auracast button | `auracast.c`, `jbl_beacon.c` |
| Party LISTEN | Plays another JBL party master's broadcast in sync | `party.c` |
| JBL One app | JBL V5 protocol over GATT (BR/EDR and LE): settings, EQ, lights, rename, party, Auracast Assistant | `jbl_v5.c`, `jbl_gatt.c` |
| Local output | 48 kHz 32-bit stereo I2S to a DAC/amp | `audio_out.c` |
| Prompt tones | From a `tones` partition you build from your own files | `tones.c`, `tools/tones_pack.py` |

More on the modules: [`s31-btstack-speaker/README.md`](s31-btstack-speaker/README.md).

## Hardware

- ESP32-S31 board with 16 MB flash and PSRAM.
- An I2S DAC or amplifier (optional; Auracast works without it).
- Five push buttons to GND (optional; internal pull-ups, active low).

| Signal | GPIO |
|---|---|
| I2S BCK / DOUT / LRCK | 35 / 37 / 39 |
| Button: Auracast on/off | 36 |
| Button: volume up / down | 38 / 40 |
| Button: Bluetooth pairing | 42 |
| Button: play / pause | 44 |

## Build from source

### 1. ESP-IDF

ESP-IDF **v6.1** with ESP32-S31 support (`release/v6.1`, built against commit `c43d665a96`), with
[`idf-patches/0010-controller-only-iso-rx.patch`](idf-patches) applied (required: without it a
controller-only build drops every received ISO packet). See [`idf-patches/README.md`](idf-patches/README.md).

```bash
cd <esp-idf>
git apply <repo>/idf-patches/0010-controller-only-iso-rx.patch
./install.sh esp32s31 && . ./export.sh      # Windows: install.ps1 / export.ps1
```

### 2. BTstack

BTstack goes into `third_party/btstack` (not part of this repository; its license allows
personal, non-commercial use), at the commit in `btstack-patches/BTSTACK_COMMIT`, with the
patches applied:

```bash
git clone https://github.com/bluekitchen/btstack third_party/btstack
cd third_party/btstack
git checkout $(cat ../../btstack-patches/BTSTACK_COMMIT)
git apply ../../btstack-patches/*.patch
```

### 3. Build and flash

```bash
cd s31-btstack-speaker
idf.py build
idf.py -p <PORT> flash monitor        # console at 921600 baud
```

`idf.py flash` writes bootloader, partition table and app; the board then has the default
color (Black). To pick the color, or to make the flasher for other people:

```bash
python tools/flasher/make_package.py   # build -> tools/flasher/firmware/
python tools/flasher/build_exe.py      # -> tools/flasher/dist/S31Flasher.exe
```

### Flash layout

| Name | Offset | Size | Use |
|---|---|---|---|
| `nvs` | 0x9000 | 24 KB | Pairings, settings |
| `phy_init` | 0xF000 | 4 KB | PHY calibration |
| `factory` | 0x10000 | 4 MB | Application |
| `jblota` | 0x410000 | 4 MB | Captured JBL One OTA image (not executed) |
| `tones` | 0x810000 | 2 MB | Prompt tones |
| `s31cfg` | 0xA10000 | 4 KB | This board's color and name (written by the flasher) |

## Prompt tones

No tone files are shipped. The firmware plays the Xtreme 5 prompt set from the `tones` partition
and stays silent if it is empty. Put your own tone files (MP3, or 16 kHz s16 mono `.pcm`, named
`poweron`, `poweroff`, `bt_cnt`, `bt_pair`, `ent_aura`, `exi_aura`, `vol_max`, `indicate`,
`c_err`, `batlow`, `mic_con`, `usound`, `0`...`9`) in a folder and pack them (needs `ffmpeg`):

```bash
python tools/tones_pack.py <dir with tone files> tones.bin
```

Put `tones.bin` into the flasher's `firmware` folder and it is written with the firmware.

## Using it

1. Power the board: it is **S31 JBL Xtreme 5** (or the name you picked). Press the pairing
   button for a new phone.
2. Play music. With Auracast off it plays on the local output.
3. Press the Auracast button to host: Xtreme 5 speakers join with their own Auracast button.
4. With the phone idle, the board joins another JBL party master nearby.
5. JBL One shows it as an Xtreme 5 with its settings, EQ, lights and party pages.
