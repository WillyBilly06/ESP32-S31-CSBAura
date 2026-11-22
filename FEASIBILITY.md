# ESP32-S31 ↔ JBL: feasibility of Auracast and PartyBoost

Built from the four RE reports in this folder plus ESP-IDF research. Ratings are
engineering judgement. Claims tagged **[FW]** come from the Xtreme 4 firmware;
**[SPEC]** from the Bluetooth Core spec; **[GUESS]** is inference.

## Verdict

| Goal | How | Feasibility | Effort |
|---|---|---|---|
| S31 **broadcasts Auracast**, Xtreme 4 / Charge 6 / other Auracast speakers play it | LE Audio BIS, `pbp/pbs` example | **High** | Weeks (mostly the audio pipeline) |
| S31 **receives** a JBL Auracast broadcast | LE Audio BIS sink, `pbp/pbk` example | **Medium-high** | Weeks |
| S31 streams music to **one** Charge 5 | Plain A2DP source (Classic) | **High** | Days |
| S31 streams to **several** Charge 5s at once | Feed an **Xtreme 4 in PartyBoost mode** over A2DP; the Xtreme 4 rebroadcasts | **High** (no RE) | Days |
| S31 **is** a PartyBoost master, Charge 5s join it directly over CPB | CSB HCI + reimplemented JBL protocol | **Blocked today** (CPB unsupported in the S31 controller, IDF-15191), then **Low** | Months, research-grade, once CPB ships |

The Charge 5 has **no Auracast**, so CPB/PartyBoost really is the only *broadcast*
route to it. But broadcasting isn't the only way to get music onto it.

## Auracast: high
- Espressif's status page marks "Bluetooth LE Audio and Auracast™ Broadcast Audio" and
  "Bluetooth 5.2: Isochronous Channels (BIS/CIS)" as ✅ supported on the S31. ESP-IDF
  support is preview on the **master branch** only, with no release version yet.
- The S31 has BLE 5.4 LE Audio with BIS/CIS; ESP-IDF ships broadcast source and sink
  examples. The Xtreme 4 uses standard LE-Audio HCI (`LE_Create_BIG 0x2068`,
  `LE_BIG_Create_Sync 0x206B`) **[FW]** and advertises the standard `0x1852`
  Broadcast Audio Announcement **[FW]**. Nothing proprietary sits in the way.
- Risks: the ESP-IDF LE-Audio API is a preview; confirm S31 sink support on your IDF
  version; the S31 is new silicon; interop details (LC3 preset, broadcast code, how the
  JBL app attaches the speaker) need testing on hardware.

## PartyBoost over CPB: low. Three gates, any of which can end it

**Gate 1. Does the S31 controller implement CSB at all?** **Not today. Espressif
lists it as planned.** The official ESP32-S31 status page
(developer.espressif.com/hardware/esp32s31, updated 15 Nov 2026) marks, under
*Bluetooth Classic Controller*:
- ⏳ "BR/EDR Connectionless Peripheral Broadcast (CPB) logical transport (IDF-15191)":
  unsupported, tracked internally
- ✅ "BR/EDR Active Peripheral Broadcast (APB) logical transport", plus ACL, SCO, eSCO

So the controller does not do CPB yet, and raw VHCI can't get around a feature the
controller firmware lacks. If IDF-15191 ships, the CSB HCI commands (`0x0441 Set_CSB`,
`0x0443 Start_Sync_Train`, `0x0C76 Set_CSB_Data` **[SPEC]**) should become
available over HCI, and gates 2 and 3 become the real work. `s31-csb-probe/` is
still useful: expect "UNKNOWN COMMAND" on today's IDF master, then re-run it after
each IDF update to see when CPB lands.

APB is not a substitute. It broadcasts to speakers *already connected* to the S31 over
ACL (at most 7), and a Charge 5 doesn't listen for PartyBoost audio on APB. It
expects CPB **[FW: Xtreme 4 uses CSB]**.

**Gate 2. JBL's proprietary protocol on top of CSB.** Partly recovered, partly not.
What we have **[FW]**:
- Discovery over BLE: AD `0xFF`, company `0xFDDF`/`0x0057`, nested tag `0x02` mode byte.
- Roles: master/slave, ABIS messages `0xB1` (master cfg, 12 B), `0xB2` (slave cfg,
  36 B), `0xB3/0xB4` (handover), gated by a link-quality threshold.
- Payload: about 679-byte slots, sent three times, with a 13-byte header/CRC. 679 bytes
  is exactly the maximum payload of a **2-DH5** packet **[SPEC]**, so PartyBoost almost
  certainly sends 2-DH5 CSB packets with triple redundancy.

What we don't have:
- The mode-byte values, the advertising payload contents (very likely the master's
  BD_ADDR/clock hints, so slaves can catch the sync train **[GUESS]**), the CSB interval
  and LT_ADDR, and the slot header format. All of it lives in RAM at runtime **[FW]**.
  Recovering it means more targeted decompilation of the code that fills those structs,
  plus live captures.
- Live capture is the hard part. CSB is BR/EDR EDR traffic. Cheap sniffers (Ubertooth)
  can't follow EDR or the sync train reliably; a real BR/EDR analyser (Ellisys, Frontline)
  costs thousands. On-device debugging (SWD/UART on the Xtreme 4 board) is the other
  option, if the board exposes it.

**Gate 3. What does a Charge 5 actually expect?** Unknown, and it's a different
firmware. The Charge 5 is a 2021, BT 5.1 speaker. The Xtreme 4 audio path
`A2DP → LC3/"LBR" → csb_codec` **[FW]** may be what it uses with *new* speakers.
For Charge 5 compatibility it may switch to an older codec or framing, possibly SBC
**[GUESS]**. Matching the Charge 5 means reversing **its** firmware's slave side too.
There may also be pairing or authentication in the handshake that we haven't seen yet.

**Bottom line:** possible in principle only if gate 1 passes, and then a multi-month
reverse-engineering project with expensive tooling and no guarantee. Not a practical
way to get music onto a Charge 5.

## The practical route to Charge 5s (no RE)
1. **One Charge 5:** the S31 is an A2DP source and the Charge 5 an ordinary A2DP sink.
   Standard Classic BT. ESP-IDF's A2DP is single-sink, so one speaker at a time.
2. **Several Charge 5s:** stream A2DP from the S31 into an **Xtreme 4**, switch the
   Xtreme 4 to PartyBoost, and let the Charge 5s join it. The Xtreme 4 is the only JBL
   Auracast model that is backward-compatible with older PartyBoost speakers, so it
   bridges the two worlds: S31 → (A2DP) → Xtreme 4 → (PartyBoost/CPB) → Charge 5s.

## Recommended order
1. Use the Xtreme 4 bridge for Charge 5s now. Native PartyBoost waits on Espressif's
   CPB support (IDF-15191); re-run `s31-csb-probe/` after IDF updates to catch it.
2. Build Auracast source on the S31 (`esp32s31-auracast/`). This is the solid core.
3. Add an A2DP-source mode for Charge 5s, used directly or through the Xtreme 4 bridge.
4. Only if step 1 passes and you want a research project: target gate 2 with more
   decompilation of the RAM-struct fillers, then get a capture setup.

## Corrections to earlier reports
- `0x0C76` is **Set_CSB_Data**, not Set_CSB. Set_CSB is **0x0441**; sync train is
  `0x0443/0x0444`; R/W sync-train params are `0x0C77/0x0C78` **[SPEC]**.
- `0x0C7A/0x0C7B/0x0C7D` are Secure Connections / Authenticated Payload Timeout / OOB
  commands, **not** CSB. The firmware's evidence for CSB rests on `0x0C76/0x0C77` plus
  the `csb_codec`/`broadcast_master` code, not on those three.
- Truncated_Page is `0x043F` (Truncated_Page_Cancel `0x0440`) **[SPEC]**. The Ghidra
  report's "0x0437–0x043C Truncated_Page" label is off; those opcodes need re-checking.
- The probe has been fixed to test `0x0C77` (pure read) and `0x0441` (Enable=0).
