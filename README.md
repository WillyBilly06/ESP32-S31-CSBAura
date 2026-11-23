# ESP32-S31 CSBAura

This firmware turns an **ESP32-S31** board into a speaker that behaves like a **JBL Xtreme 5**:
- Phones stream to it over Classic Bluetooth (A2DP) or LE Audio.
- Real Xtreme 5 speakers join its Auracast broadcast with their own Auracast button (JBL party).
- It can join another JBL party master.
- The JBL One app sees it as an Xtreme 5.

Each flash makes the board **its own Xtreme 5**, with its own Bluetooth address, serial
number, beacon identity and Auracast Broadcast_ID. You pick the speaker color shown in JBL
One when you flash.

## Contents

- [Quick start (Windows, S31 Flasher)](#quick-start-windows-s31-flasher)
- [What's in this repository](#whats-in-this-repository)
- [Features](#features)
- [Hardware](#hardware)
- [Using the speaker](#using-the-speaker)
- [Build from source (any OS)](#build-from-source-any-os)
- [Prompt tones](#prompt-tones)
- [Troubleshooting](#troubleshooting)
- [Licenses](#licenses)

## Quick start (Windows, S31 Flasher)

1. **Get the repository.** On GitHub use **Code → Download ZIP** and unpack it, or
   `git clone` it. It already contains the patched ESP-IDF and BTstack, so nothing has to be
   cloned or patched. Keep the folder's path short (under 80 characters, no spaces), for
   example `C:\S31\ESP32-S31-CSBAura`; Windows can't hold ESP-IDF's deepest files deeper.
2. **Get the flasher.** Download `S31Flasher.exe` from the [Releases](../../releases) page and
   put it into that folder, next to this README.
3. **Run `S31Flasher.exe`.** If SmartScreen warns about an unknown app, choose
   **More info → Run anyway**.
4. **Nothing to set up.** **1 Setup** says "Ready to build". The first **Build + Flash**
   installs ESP-IDF's compiler and tools by itself (about 1 GB into
   `%USERPROFILE%\.espressif`, several minutes, once). If the PC has no Python 3.10 to 3.13,
   it installs Python 3.12 from python.org for this user first.
5. **Plug in the board** and pick its port under **2 Board**.
6. **Click a color** under **3 Speaker color**. Optionally type a name under **4 Name**.
7. **Press Build + Flash.** The first build takes a few minutes. Then the board restarts.
8. **Pair it** from the phone's Bluetooth settings: **S31 JBL Xtreme 5**, or your name.

| Color | ID |
|---|---|
| Black | `0x00` |
| White | `0x01` |
| Blue | `0x02` |
| Purple | `0x05` |
| Camo | `0x1A` |

Every board is its own Xtreme 5: its Bluetooth address, serial, beacon identity and
Auracast Broadcast_ID come from the chip's factory MAC, so two boards never collide. A board
keeps its identity and pairings when you flash it again. **More → Set color / name only**
changes only the color or name, in a few seconds.

All details, including the command line and troubleshooting:
[`tools/flasher/README.md`](tools/flasher/README.md).

## What's in this repository

| Path | What |
|---|---|
| `s31-btstack-speaker/` | The firmware (ESP-IDF project) |
| `third_party/esp-idf/` | ESP-IDF by Espressif, already patched, see [`S31_README.md`](third_party/esp-idf/S31_README.md) |
| `third_party/btstack/` | BTstack by BlueKitchen, already patched (only the parts the firmware uses), see [`S31_README.md`](third_party/btstack/S31_README.md) |
| `btstack-patches/` | Our BTstack patches and the BTstack commit they apply to |
| `idf-patches/` | Our ESP-IDF patch and the ESP-IDF commit it applies to |
| `tools/flasher/` | The S31 Flasher (source of `S31Flasher.exe`) |
| `tools/release/` | How `third_party` is made from the upstream repositories (`idf_tree.py`, `gitexport.py`) |
| `tools/tones_pack.py` | Packs your own prompt tone files into `tones.bin` |

## Features

| Feature | What it does | Source |
|---|---|---|
| A2DP sink | SBC or AAC-LC from the phone, resampled to 48 kHz with clock recovery; AVRCP volume, play/pause, track info | `a2dp_in.c` |
| LE Audio sink | BAP unicast (LC3) with volume control; Play over LE media control after a switch from A2DP | `le_audio.c` |
| Auracast (JBL party host) | Broadcast with the advertising and BIG layout of a real Xtreme 5 master: Xtreme 5 speakers join with their own Auracast button | `auracast.c`, `jbl_beacon.c` |
| Party LISTEN | Plays another JBL party master's broadcast in sync | `party.c` |
| JBL One app | JBL V5 protocol over GATT (BR/EDR and LE): settings, EQ, lights, rename, party, Auracast Assistant | `jbl_v5.c`, `jbl_gatt.c` |
| Speaker identity | Serial, beacon and Broadcast_ID from the chip MAC; color and name from the `s31cfg` partition | `s31_ident.c` |
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

## Using the speaker

1. **Pair a phone.** A board with no phone paired starts in pairing mode for 3 minutes.
   Otherwise, or for another phone, press the pairing button.
2. **Play music.** With Auracast off it plays on the local I2S output. LE Audio phones can
   switch to LE Audio in their Bluetooth settings or in JBL One.
3. **Host a party.** Press the Auracast button (or use JBL One). Real Xtreme 5 speakers join
   when you press their own Auracast button.
4. **Join a party.** With the phone idle, the board joins another JBL party master nearby.
5. **JBL One** shows it as an Xtreme 5 in the chosen color, with its settings, EQ, lights and
   party pages.

## Build from source (any OS)

The flasher only automates these steps. To do them by hand:

### 1. ESP-IDF

You need ESP-IDF **v6.1** with ESP32-S31 support. Use exactly commit `c43d665a96` of
`release/v6.1` (see `idf-patches/IDF_COMMIT`), with
[`idf-patches/0010-controller-only-iso-rx.patch`](idf-patches) applied. That patch is
required: without it, a controller-only build drops every received ISO packet. See
[`idf-patches/README.md`](idf-patches/README.md).

**Easiest:** use the one in this repository. It is already patched; only its tools need
installing, once:

```bash
cd third_party/esp-idf
./install.sh esp32s31 && . ./export.sh      # Windows: .\install.ps1 esp32s31; . .\export.ps1
```

**Or with git**, somewhere else:

```bash
git clone --no-checkout https://github.com/espressif/esp-idf esp-idf-s31
cd esp-idf-s31
git checkout $(cat <repo>/idf-patches/IDF_COMMIT)
git submodule update --init --recursive --depth 1
git apply <repo>/idf-patches/0010-controller-only-iso-rx.patch
./install.sh esp32s31 && . ./export.sh
```

### 2. BTstack

Nothing to do: `third_party/btstack` is part of this repository, already patched. To make it
again from BlueKitchen's repository, see
[`third_party/btstack/S31_README.md`](third_party/btstack/S31_README.md).

### 3. Build and flash

```bash
cd s31-btstack-speaker
idf.py build
idf.py -p <PORT> flash monitor        # console at 921600 baud
```

`idf.py flash` writes the bootloader, partition table and app. It doesn't write the
`s31cfg` partition, so a board that never had one shows as Black, and a board flashed by the
S31 Flasher before keeps its color and name. To set the color and name, use the flasher (it
also works from the command line):

```bash
python tools/flasher/s31_flasher.py --port <PORT> --color blue --no-build
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

`tones.bin` in the repository root holds the Xtreme 5 prompt set, and the S31 Flasher writes
it with the firmware. The tones are JBL's audio, so this repository has to stay **private**
while it holds them.

### Your own sounds

1. Make a folder and put sound files in it, **named after the prompt they replace**. WAV,
   MP3, FLAC, OGG, M4A/AAC or raw `.pcm` all work, at any sample rate, mono or stereo;
   they're converted to 24 kHz mono.

   | File name | Plays when |
   |---|---|
   | `poweron` | the speaker starts |
   | `poweroff` | the speaker powers off |
   | `bt_pair` | pairing mode starts (button, or LE Audio switched in JBL One) |
   | `bt_cnt` | a phone has connected |
   | `ent_aura` / `exi_aura` | Auracast is turned on / off |
   | `vol_max` | the volume hits its maximum or minimum |
   | `indicate` | JBL One identifies the speaker, or a stereo pair is set up |
   | `c_err` | JBL One's firmware update is refused |
   | `batlow`, `mic_con`, `usound`, `0` ... `9` | packed, not played by the firmware yet |

2. Pack them. This needs [ffmpeg](https://ffmpeg.org) on the PATH:

   ```bash
   # replace only the tones in the folder, keep all the others:
   python tools/tones_pack.py my_tones tones.bin --base tones.bin

   # or a completely new set (prompts without a file stay silent):
   python tools/tones_pack.py my_tones tones.bin
   ```

   It prints every tone and its length. The whole set must fit in 2 MB, about 43 s of
   sound.

3. **Build + Flash** (or **More → Flash the last build**) with the S31 Flasher. It writes
   the new `tones.bin` from the project folder.

Keep prompts short (under 1–2 s, except `bt_pair`). The firmware lowers the music while a
tone plays.

## Troubleshooting

| Problem | Fix |
|---|---|
| The board doesn't show up as a port | Use a USB data cable; hold BOOT, press RESET, release BOOT. |
| Flashing says the port is busy | Close any serial monitor using the port. |
| The phone can't connect | Press the pairing button (GPIO 42) and pick the speaker on the phone. After an **Erase**, forget the old pairing on the phone first. |
| Xtreme 5 speakers don't join the party | Press their Auracast button while the board hosts; keep them within a few meters. |
| No sound from the I2S output | Check the DAC wiring (GPIO 35/37/39) and that Auracast is off for local playback. |

More: [`tools/flasher/README.md`](tools/flasher/README.md#troubleshooting).

## Licenses

- **BTstack** (`third_party/btstack`) is © BlueKitchen GmbH under its own
  [license](third_party/btstack/LICENSE). Redistribution and use are allowed **for personal,
  non-commercial purposes only**. For anything else, get a commercial license from
  BlueKitchen. This project, built with it, therefore carries the same restriction.
- **ESP-IDF** (`third_party/esp-idf`) is © Espressif Systems under Apache-2.0; some of its
  components carry their own licenses in their folders.
- JBL, Xtreme and Auracast are trademarks of their owners. This is an independent hobby
  project, not affiliated with or endorsed by Harman or JBL.
