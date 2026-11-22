# ESP32-S31 CSBAura

Turns an **ESP32-S31** into a JBL Xtreme 5-class speaker on the radio side:

- **Auracast broadcast master.** JBL speakers join it with their own Auracast button, with no app and no phone, in JBL party mode (light stays on). Any number of speakers can listen, since it's a broadcast.
- **A2DP sink.** A phone streams music over Classic Bluetooth (BR/EDR). The S31 resamples it to 48 kHz, LC3-encodes it and re-broadcasts it over Auracast.
- **JBL One app control.** The S31 answers JBL's V5 protocol over GATT on BR/EDR and BLE, so JBL One sees it as an Xtreme 5. It reports battery, lights and settings, and applies the app's EQ to the audio.

It has its own identity (name, Classic MAC, `macCrc`), so it is a different Xtreme 5 from the user's real one, not a clone of it.

> Status: native join, A2DP → Auracast and the JBL One control link all work on hardware.
> Open issue: occasional garble on the receiving speaker while the phone streams, a BR/EDR + LE ISO coexistence problem.
> The PCM going into the encoder is clean: 0 underruns, 0 drops, 0 discontinuities.

## Hardware

| Part | Notes |
|---|---|
| ESP32-S31 dev board | 16 MB flash, 16 MB octal PSRAM, on COM5 in all the logs |
| JBL Xtreme 5 (fw 26.40.10) | Receiver, and the reference that every advert/protocol field was copied from |
| A phone | A2DP source; JBL One for the control features |

## Repository layout

| Path | What it is |
|---|---|
| [`s31-jbl-speaker/`](s31-jbl-speaker) | **Main firmware.** JBL master beacon, BAP broadcast source, A2DP bridge, V5 server, GATT services, speaker assistant |
| [`s31-auracast-tx/`](s31-auracast-tx) | First broadcast source test (tone), the base of `s31-jbl-speaker` |
| [`s31-auracast-rx/`](s31-auracast-rx) | Auracast sink: receives the Xtreme 5's broadcast, logs BIGInfo and adv timing |
| [`s31-sniffer/`](s31-sniffer) | Passive scanner/PA-sync logger used to capture the real Xtreme 5's adverts and periodic data |
| [`s31-csb-probe/`](s31-csb-probe) | Early probe for JBL's Classic connectionless (PartyBoost/CSB) traffic |
| [`esp32s31-auracast/`](esp32s31-auracast) | Original starting-point sketch (button cycling idle/broadcast/receive) |
| [`idf-patches/`](idf-patches) | Two guarded Bluedroid patches the main firmware needs |
| [`apk/notes_*.md`](apk) | Notes from reading the JBL One / JBL Portable apps: advert format, join flow, V5 server |
| `RESULTS.md`, `FEASIBILITY.md`, `xtreme*_*.md` | Test results and research reports |

The decompiled apps, the Ghidra output and the jadx binaries stay local and are not in this repo (see `.gitignore`).

## How the native join works

A JBL speaker in "looking for a master" mode only joins a broadcast whose extended advert looks exactly like a JBL master's. The fields below were copied from captures of a real Xtreme 5 hosting.

| Field | Value |
|---|---|
| Broadcast Audio Announcement `0x1852` | Broadcast_ID `0x112233` |
| Public Broadcast Announcement `0x1856` | `04 00` (High Quality, unencrypted) |
| Appearance | `0x0885` |
| Broadcast Name + Complete Local Name | `ESP32-S31's JBL Xtreme 5` |
| Harman manufacturer data | `15 ff 57 00` + 16 × `00` + `df fd` |
| Periodic data | BASE (LC3 48_2, 2 BIS, 100 octets) + JBL `0xFDDF` TLV block |
| `0xFDDF` beacon (legacy adv) | PID `69 21`, the S31's **own** `macCrc` (CRC-16/ARC of its Classic MAC), partyInfo `0xB440` (Party, Primary) |

The BIG uses the Xtreme 5's layout: ISO interval 20 ms, 8 subevents, burst 2, IRC 4, 2M PHY, 80 ms presentation delay. It is created with `CONFIG_BT_ISO_TEST_PARAMS`.

There is more detail in [`RESULTS.md`](RESULTS.md) and [`apk/notes_adv.md`](apk/notes_adv.md).

## Build and flash (main firmware)

You need ESP-IDF v6.1 with ESP32-S31 support and the patches in [`idf-patches/`](idf-patches) applied.

```bash
cd s31-jbl-speaker
idf.py set-target esp32s31
idf.py -p COM5 flash monitor
```

Key config, set in `sdkconfig.defaults`:

- `CONFIG_BTDM_CTRL_MODE_BTDM` (dual mode)
- A2DP sink with SBC bitpool 53
- `CONFIG_BT_ISO_TEST_PARAMS`
- PSRAM for the audio ring buffer
- Custom 16 MB partition table with a 4 MB factory app
- LE TX power +20 dBm
- Bluedroid on core 1, so core 0 is left to the controller

## Using it

1. Power the S31. It advertises as **S31 JBL Xtreme 5** (Classic) and hosts the Auracast broadcast.
2. Pair the phone and play music. The S31 reconnects to the last phone on its own at power-up.
3. Press the Auracast button on a JBL Xtreme 5. It joins within a few seconds, and more speakers can join the same way.
4. Optional: open JBL One. The S31 appears as an Xtreme 5 with its EQ and settings pages.

## Firmware modules (`s31-jbl-speaker/main`)

| File | Role |
|---|---|
| `main.c` | BAP broadcast source, LC3 encode (2 × mono BIS), JBL ext-adv fields, beacon update task, TX stats |
| `a2dp_bridge.c` | A2DP sink, 44.1 → 48 kHz resampler with drift control, PSRAM ring buffer, last-phone reconnect |
| `jbl_v5_server.c` | JBL V5 protocol server (Get/Set/Notify), EQ biquads applied before encoding |
| `jbl_gatt.c` | Excelpoint V5 services + Device Information over LE and BR/EDR |
| `jbl_assistant.c` | Watches nearby JBL speakers (`0xFDDF`), logs native joins, and falls back to an assisted V5 join |
| `bluedroid/adv.c` | Extended/periodic advertising and the legacy JBL beacon |
