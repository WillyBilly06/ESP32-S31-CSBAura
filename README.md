# ESP32-S31 CSBAura

Firmware that turns an **ESP32-S31** board into a speaker that behaves like a **JBL Xtreme 5** on the
radio side: phones stream to it over Classic Bluetooth, real Xtreme 5 speakers join its Auracast
broadcast with their own Auracast button (JBL party), it can itself join another JBL party master,
and the JBL One app sees it as an Xtreme 5.

It has its own identity (device name, Classic address, `macCrc`), so it shows up as a separate
Xtreme 5 rather than a clone of an existing speaker.

The main firmware is **[`s31-btstack-speaker/`](s31-btstack-speaker)** (BTstack host on the S31's
controller). Everything else in the repository is earlier bring-up work, tools, patches and notes.

## Features (`s31-btstack-speaker`)

| Feature | What it does | Source |
|---|---|---|
| A2DP sink | SBC or AAC-LC (Helix decoder) from the phone, decoded on core 0, resampled 44.1 -> 48 kHz with clock recovery into a small PSRAM ring (constant, low added latency); AVRCP volume, play/pause, track info; reconnects to the last phone | `a2dp_in.c` |
| Auracast broadcast (JBL party host) | BAP broadcast source with the extended/periodic advertising and BIG layout of a real Xtreme 5 master, so Xtreme 5 speakers join natively with their Auracast button, no app or phone needed | `auracast.c`, `jbl_beacon.c` |
| Party LISTEN | When the phone is idle, the S31 syncs to another JBL party master's broadcast, decodes it (LC3, with PLC) and plays it in sync with the master | `party.c` |
| Local I2S output | 48 kHz 32-bit stereo I2S to a DAC/amp; plays the broadcast in sync with the receivers, phone audio when Auracast is off, or the LISTEN stream | `audio_out.c` |
| Buttons | Auracast on/off, volume up/down, pairing, play/pause | `buttons.c` |
| JBL One app | JBL "V5" protocol server over GATT on BR/EDR and LE (Android and iOS app): device info, volume, media control, JBL party on/off, rename (Classic and LE name), lights, auto power off, Playtime Boost, settings in NVS | `jbl_v5.c`, `jbl_gatt.c` |
| Auracast Assistant (JBL One) | "Broadcast to Others" runs a **standard** Auracast broadcast (its own Broadcast ID and name, BASE only, optional password) that any Auracast receiver can join; "Join Nearby Broadcast" lists any Auracast broadcast and receives it, with the password if it is encrypted. Either one turns the JBL party broadcast and listening off until the Auracast button (or the app's party switch) turns it back on | `party.c`, `auracast.c`, `jbl_v5.c` |
| EQ | The app's EQ biquads applied to the audio on core 0 | `v5_eq.c` |
| OTA capture | An OTA image sent by the app is received and stored in the `jblota` partition for read-back (it is not executed) | `jbl_v5.c` |
| Prompt tones | Power on/off, connect, pairing, volume limit, ... played from a `tones` flash partition that you build from your own files | `tones.c`, `tools/tones_pack.py` |
| Soft power off | "Power off" from the app (or auto power off) plays the power-off tone and shuts the radio down; any button restarts | `power.c` |
| Link tuning | EDR 3 Mbps packets, QoS poll interval, controller scheduling priorities so A2DP and the BIG share the radio | `link_tune.c`, `bt_sched_tune.c` |

Notes:
- At the time of writing Auracast is **off at power-up** (`PARTY_BOOT_AURACAST` in `party.c`);
  the Auracast button turns hosting (and LISTEN) on. With Auracast off, phone audio plays only on
  the local I2S output.
- The exact BIG layout (ISO interval, BN, IRC, ...) is still being tuned; the current values are
  in `s31-btstack-speaker/main/auracast.c` / `auracast.h`, not repeated here.
- Tested with Android phones (Samsung, REDMAGIC) and an iPhone, for both A2DP and JBL One. The
  standard-broadcast Assistant modes are new and not yet tested against other vendors' receivers.

More detail on the modules: [`s31-btstack-speaker/README.md`](s31-btstack-speaker/README.md).

## Repository map

| Path | Contents |
|---|---|
| [`s31-btstack-speaker/`](s31-btstack-speaker) | **Main firmware**: ESP32-S31 + BTstack (controller-only IDF build, VHCI), emulates a JBL Xtreme 5 |
| [`s31-jbl-speaker/`](s31-jbl-speaker) | Earlier build of the same speaker on ESP-IDF's Bluedroid host (NimBLE overlay present); also holds `components/liblc3`, which the main firmware uses |
| [`s31-auracast-tx/`](s31-auracast-tx) | First Auracast broadcast source test (test tone), base of `s31-jbl-speaker` |
| [`s31-auracast-rx/`](s31-auracast-rx) | Auracast sink that receives a real Xtreme 5 broadcast and logs BIGInfo / timing |
| [`esp32s31-auracast/`](esp32s31-auracast) | Original starting-point scaffold (one button cycling idle / broadcast / receive) |
| [`s31-sniffer/`](s31-sniffer) | Passive extended-adv / periodic-adv logger used to capture a real speaker's adverts (`monitor.py`) |
| [`s31-csb-probe/`](s31-csb-probe) | Raw-HCI probe: does the S31 controller implement Connectionless Slave Broadcast (PartyBoost)? |
| [`tools/`](tools) | `s31_console/` (web console + `s31ctl.py` CLI for build/flash/monitor), `tones_pack.py`, resampler checks (`rs_check.py`, `rs_host_check.py`, `rs_host_test.c`) |
| [`idf-patches/`](idf-patches) | Patches against ESP-IDF (Bluedroid patches for `s31-jbl-speaker`, controller/transport patches for the main firmware) |
| [`btstack-patches/`](btstack-patches) | Patches against BTstack plus the BTstack commit they apply to (`BTSTACK_COMMIT`) |
| [`re/`](re) | Reverse-engineering notes on the S31 controller scheduler (`controller_sched.md`); `re/local_tones/` is local only |
| [`apk/`](apk) | Protocol notes from reading the JBL apps (`notes_*.md`); the apps themselves stay local |
| [`docs/`](docs) | Test results and research reports (see below) |
| `third_party/btstack/` | BTstack checkout, **not tracked** (see build steps) |

Not in the repository (see `.gitignore`): BTstack, decompiled apps and APKs, firmware disassembly,
extracted speaker tones, build output and logs.

## Hardware

- ESP32-S31 dev board with 16 MB flash and PSRAM (the partition table and `sdkconfig.defaults`
  assume 16 MB flash).
- An I2S DAC or amplifier for local audio (optional; Auracast works without it).
- Five push buttons to GND (optional; internal pull-ups, active low).

| Signal | GPIO |
|---|---|
| I2S BCK | 35 |
| I2S DOUT | 37 |
| I2S LRCK (WS) | 39 |
| Button: Auracast on/off | 36 |
| Button: volume up (repeats while held) | 38 |
| Button: volume down (repeats while held) | 40 |
| Button: Bluetooth pairing (discoverable for 3 min) | 42 |
| Button: play / pause | 44 |

I2S format: Philips, 48 kHz, 32-bit stereo, clocked from the APLL.

## Build and flash (`s31-btstack-speaker`)

### 1. ESP-IDF

ESP-IDF **v6.1 with ESP32-S31 support and the patches in [`idf-patches/`](idf-patches)**. The main
firmware was last built against the `release/v6.1` branch (commit `c43d665a96`). For this
firmware the relevant patches are:

- `0010-controller-only-iso-rx.patch`: **required**. Without it a controller-only build drops (and
  leaks) every received ISO packet, so party LISTEN gets no audio.
- `0009-btdm-lpclk-force-main-xtal.patch`: the project's `CMakeLists.txt` sets
  `BTDM_LPCLK_FORCE_MAIN_XTAL=1` for it. It does not apply cleanly to the current `release/v6.1`
  tree; whether it is still needed there has not been verified.
- `0001` to `0007` patch the Bluedroid host and matter only for `s31-jbl-speaker`.

```bash
cd <esp-idf>
git apply <repo>/idf-patches/0010-controller-only-iso-rx.patch
./install.sh esp32s31 && . ./export.sh      # Windows: install.ps1 / export.ps1
```

### 2. BTstack

BTstack is cloned into `third_party/btstack` (gitignored; its license allows personal,
non-commercial use) and patched:

```bash
cd <repo>
git clone https://github.com/bluekitchen/btstack third_party/btstack
cd third_party/btstack
git checkout $(cat ../../btstack-patches/BTSTACK_COMMIT)
git apply ../../btstack-patches/0001-avdtp-mtu-overridable.patch \
          ../../btstack-patches/0002-gap-big-create-test.patch
```

- `0001` lets the firmware override the AVDTP L2CAP MTU.
- `0002` adds `gap_big_create_test()` (HCI LE Create BIG Test), used to set the BIG layout
  explicitly.

### 3. Build and flash

```bash
cd s31-btstack-speaker
idf.py build
idf.py -p <PORT> flash monitor
```

- The target (`esp32s31`) is already set in the tracked `sdkconfig`; run `idf.py set-target esp32s31`
  only for a fresh config.
- The build also needs `s31-jbl-speaker/components/liblc3` (pulled in via `EXTRA_COMPONENT_DIRS`).
- The console runs at **921600 baud** (`idf.py monitor -b 921600` if your monitor does not pick it up).
- `idf.py flash` writes bootloader, partition table and app only; the `tones` and `jblota`
  partitions are left as they are.

Alternatively use the web console in [`tools/s31_console/`](tools/s31_console/README.md). Its
defaults point at `s31-jbl-speaker` and a different IDF tree, so pass both:

```powershell
tools\s31_console\start_console.ps1 -Port <PORT> -Project <repo>\s31-btstack-speaker -IdfPath <esp-idf>
python tools\s31_console\s31ctl.py flash --build --wait     # use the IDF python env
```

### Flash layout

From [`s31-btstack-speaker/partitions.csv`](s31-btstack-speaker/partitions.csv):

| Name | Offset | Size | Use |
|---|---|---|---|
| `nvs` | 0x9000 | 24 KB | Bonding keys, settings |
| `phy_init` | 0xF000 | 4 KB | PHY calibration |
| `factory` | 0x10000 | 4 MB | Application |
| `jblota` | 0x410000 | 4 MB | JBL One OTA capture: header sector at 0x410000, image from 0x411000 |
| `tones` | 0x810000 | 2 MB | Prompt tones image |

To read back a captured OTA image: read the 4 KB header at 0x410000 (magic `JBLOTA01`, then the
image size as little-endian u32), then read that many bytes from 0x411000, for example
`python -m esptool --chip esp32s31 -p <PORT> read-flash 0x410000 0x1000 ota_hdr.bin`.

## Prompt tones

No tone files are shipped. The firmware plays the Xtreme 5 prompt set from the `tones` partition
and stays silent if it is empty. To fill it, put your own tone files in a local directory (for
example under `re/local_tones/`, which is gitignored):

- MP3, or raw 16 kHz signed 16-bit mono `.pcm`;
- named after the tone, with names shorter than 12 characters: `poweron`, `poweroff`, `bt_cnt`,
  `bt_pair`, `ent_aura`, `exi_aura`, `vol_max`, `indicate`, `c_err`, `batlow`, `mic_con`, `usound`,
  `0` ... `9` (see `s31-btstack-speaker/main/tones.h`).

Then pack and write the image (needs `ffmpeg` on `PATH`):

```bash
python tools/tones_pack.py <dir with tone files> tones.bin
python -m esptool --chip esp32s31 -p <PORT> write-flash 0x810000 tones.bin
```

The JBL One feedback-tone switch turns the prompts off.

## Using it

1. Power the board. It is connectable as **S31 JBL Xtreme 5** over Classic Bluetooth and
   reconnects to the last phone. Press the pairing button to make it discoverable for a new phone.
2. Play music from the phone. With Auracast off it plays on the local I2S output only.
3. Press the Auracast button to host: Xtreme 5 speakers then join by pressing their own Auracast
   button, as many as are in range.
4. With Auracast on and the phone idle (about 1.5 s), the S31 looks for another JBL party master
   and plays its broadcast; music from the phone takes over again at once.
5. Optional: open JBL One; the S31 appears as an Xtreme 5 with its settings, EQ and party pages.
   Under Auracast Assistant, "Broadcast to Others" makes it a standard Auracast source and "Join
   Nearby Broadcast" makes it receive any Auracast broadcast; the JBL party stays off until you
   turn it on again.

## Documentation

| Document | Contents |
|---|---|
| [`s31-btstack-speaker/README.md`](s31-btstack-speaker/README.md) | Main firmware: threads, modules, build variants |
| [`docs/RESULTS.md`](docs/RESULTS.md) | Hardware test results: what the Xtreme 5 broadcasts, receive/send tests, native join |
| [`docs/FEASIBILITY.md`](docs/FEASIBILITY.md) | Early feasibility study: Auracast vs PartyBoost on the S31 |
| [`docs/xtreme5_firmware_notes.md`](docs/xtreme5_firmware_notes.md) | Why static analysis of the Xtreme 5 firmware is blocked |
| [`docs/xtreme4_*.md`](docs) | Xtreme 4 firmware research reports (Auracast, PartyBoost, discovery) |
| [`re/controller_sched.md`](re/controller_sched.md) | S31 controller BR/EDR vs LE scheduling notes, basis of `bt_sched_tune.c` |
| [`apk/notes_*.md`](apk) | JBL app protocol notes: advert format, join flow, V5 server |
| [`idf-patches/README.md`](idf-patches/README.md) | ESP-IDF patches (lists 0001-0006 only; see the patch files for 0007, 0009, 0010) |
| [`tools/s31_console/README.md`](tools/s31_console/README.md) | Web console, its HTTP API and the `s31ctl.py` CLI |

The READMEs in `s31-jbl-speaker/`, `s31-auracast-tx/` and `s31-auracast-rx/` are still the
README of the ESP-IDF example each project started from and do not describe the projects as they
are now. `docs/RESULTS.md` describes them better.
