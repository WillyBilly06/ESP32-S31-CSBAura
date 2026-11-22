# ESP32-S31 ↔ JBL Xtreme 5 over Auracast — test results

Both directions were tested on real hardware (ESP32-S31 on COM5, JBL Xtreme 5 v26.40.10).

## What the Xtreme 5 broadcasts (Auracast button on)
Seen in nRF Connect and in the S31 logs:

| Item | Value |
|---|---|
| Address | `78:66:F3:AB:11:CE`, the speaker's main bonded address, advertising set 2 |
| Broadcast Audio Announcement (`0x1852`) | Broadcast_ID **0x112233**, the same after every on/off toggle |
| Public Broadcast Announcement (`0x1856`) | `04 00`: High Quality, **not encrypted**, no metadata |
| Broadcast Name (AD `0x30`) | `BillyDaKidz's JBL Xtreme 5` |
| Periodic advertising | 100 ms |
| Stream | LC3, 48 kHz, 10 ms frames, 100 octets/frame, **2 BIS (Front-Left + Front-Right)**, 1 subgroup |

There is a second advertiser, `5C:DE:BC:F2:B5:90`. It carries Service Data under 16-bit UUID **`0xFDDF`**: a connectable JBL beacon. One byte near its end changes when Auracast is toggled (`…40 00 00` when off). It is the JBL-specific part. The audio itself is plain, standard Auracast.

The broadcast shows up as a normal Auracast stream on a Samsung phone. Playing it needs an LE Audio sink.

## Receive: Xtreme 5 → S31 ✅
Project: [s31-auracast-rx](s31-auracast-rx). Built from `D:\ESP32-S31\LE_Audio_Broadcast_Sink`, which was left untouched.

- Locks onto the Xtreme 5 by Broadcast_ID `0x112233`, or by a broadcast name containing "JBL".
- Result: PA sync and BASE in under 1 s, BIG not encrypted, both BIS synced. About **200 LC3 frames/s decoded, 0 failures**, PCM peak around 21–23k (real music).
- Audio output is I2S on GPIO2 = BCLK, GPIO3 = WS, GPIO4 = DOUT, as in the original project. A DAC or amp is needed to hear it.

## Send: S31 → Xtreme 5 ✅ (joined through the JBL One app)
Project: [s31-auracast-tx](s31-auracast-tx). Built from `D:\ESP32-S31\USB_Audio_to_Auracast`, which was left untouched.

- Open Auracast broadcast **"ESP32-S31 Test Tone"**, Broadcast_ID `0x5331A5`, LC3 48_2_1, 2 BIS stereo.
- Announces `0x1852`, `0x1856` (High Quality) and a Broadcast Name, the same way the Xtreme 5 does.
- Audio is a generated beep: 440 Hz left / 660 Hz right, 0.4 s on / 0.6 s off. About 100 frames/s sent steadily.
- **The Xtreme 5 played the beeps after being pointed at the broadcast in JBL One → Auracast Assistant.**
- Pressing the Xtreme 5's Auracast button does not join a non-JBL broadcast on its own. That path would need the S31 to also send the `0xFDDF` host beacon (not done).

## S31 as a JBL Auracast master: native button join ✅
Project: [s31-jbl-speaker](s31-jbl-speaker). **Pressing the Xtreme 5's own Auracast button makes it join the S31's broadcast in JBL party mode. The light stays on, and there's no app or phone involved.** Verified 2026-11-21: the speaker went from "Secondary, looking for a master" to "Secondary, connected" 2.6 s after the button press, and the S31 never connected to it.

What a JBL master has to send. All of this was copied from captures of the real Xtreme 5 hosting:

| Where | Field | S31 value |
|---|---|---|
| Ext adv (broadcast set, SID 2) | `0x1852` Broadcast_ID | **`0x112233`**: every JBL master uses this fixed ID |
| | `0x1856` PBA | `04 00` |
| | Appearance | `0x0885` |
| | Broadcast Name (0x30) **and** Complete Local Name (0x09) | "ESP32-S31's JBL Xtreme 5" |
| | **Harman mfg AD**: company `0x0057`, 16 × `00`, `df fd` | the "JBL master" marker. **Without it the speaker never picks the S31.** |
| Periodic adv | BASE, then a JBL mfg AD (company `0xFDDF`) with TLVs `02:1f`, `f0:11 NN` (counter), `11:` 17-byte status (`… 3c 00 00 00 00 00 24 b9 01`) | copied; NN and one status byte tick |
| Legacy connectable beacon (own random addr) | Service Data `0xFDDF`, V5 layout: PID `69 21`, mask `0x97C0`, **macCrc = CRC-16/ARC of the S31's own Classic MAC** (`0xE00D`), battery, misc `0x9080`, partyMethod `0xC0`, partyInfo `0xB440` (Primary, connected) | its own identity. Copying the Xtreme 5's CRC (`0x10CD`) makes the speaker treat the S31 as itself. |

Also in that firmware:
- **A2DP sink.** It shows up as "S31 JBL Xtreme 5" over Classic Bluetooth. Phone music is resampled to 48 kHz and re-broadcast on Auracast. With no phone it sends the test tone.
- **Fallback assistant** ([jbl_assistant.c](s31-jbl-speaker/main/jbl_assistant.c)). If a JBL speaker is "looking for a master" for 15 s without joining, the S31 connects and pushes the join. It uses the JBL V5 protocol on GATT service `65786365-…-2e636f6d0000`: ScanControl `0x0B81`, then SubgroupPlay `0x0B85` + GroupSelection `0x0B83`. That path plays, but the speaker's party light goes off.
- **Any number of speakers.** Speakers are tracked by macCrc. Auracast has no receiver limit, and the native join needs no connection.

Decoded from JBL One 2.9.4 (jadx, [apk/notes_adv.md](apk/notes_adv.md), [apk/notes_join.md](apk/notes_join.md)):
- The V5 frame layout is `u16 0xDD00 | u16 cmd | u8 n | u8 i | u16 len | payload`.
- Party feature IDs are `0x2004` and `0x2C00`–`0x2C81`. A speaker stores no group ID or partner for Auracast party.

## Build / flash
Both projects use the IDF tree that already builds your S31 projects:

```powershell
$env:IDF_PATH="D:\ESP32-S31\esp-idf-v6.1-codecs"; $env:IDF_TARGET="esp32s31"
. "$env:IDF_PATH\export.ps1"
idf.py build
idf.py -p COM5 -b 921600 flash
```
Release COM5 from any serial monitor before flashing. For a fresh config, run `idf.py --preview set-target esp32s31` first.

## Not done yet
- Real audio in and out: a DAC/amp on the receiver, and a real music source (USB audio, line-in or A2DP) on the sender instead of the tone.
- One firmware that switches between receive and send, for example from a button.
- ~~Button-path join~~: done in s31-jbl-speaker (see above).
- Joining more than one JBL speaker at once: the design supports it, but it's untested because only one speaker was available.
- JBL One doesn't list the S31 as a speaker, because the S31 doesn't answer V5 device-info requests.
- The receiver's log line printed the speaker address backwards. That is fixed in the source but not rebuilt or flashed yet.
