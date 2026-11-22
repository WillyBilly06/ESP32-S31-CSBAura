# JBL Xtreme 5 firmware — why static RE is blocked

File: `HM_JBL_Xtreme5_260928_v26.40.10_ota.bin` (1,609,424 bytes).

## What the header says (plaintext)
- Magic **`AOTA`** = Actions OTA container (not the Harman `HMHEADER` format the Xtreme 4 uses).
- Build string: `JBL_Xtreme5_v26.40.10_2609281042`.
- SoC string: **`ats2875h_xtreme_5`** → **Actions Semiconductor ATS2875H** BT-audio SoC.

So the Xtreme 5 is a **completely different platform** from the Xtreme 4 (which was a ZG Micro SoC with plaintext ARM Thumb-2). Findings from the Xtreme 4 do **not** reliably describe the Xtreme 5.

## Why we can't reverse it
- **Entropy ~7.97/8.0 across all 99 × 16 KB blocks** (only the first block dips to 7.70 — the plaintext header).
- **Near-flat byte histogram** in a mid-file sample: all 256 byte values present, counts min 185 / max 690 / avg 256.
- **No standard decompression** (zlib/gzip/raw-deflate) engages anywhere; the gzip/zlib "magics" found are coincidental bytes in random data.

Together these are the signature of an **encrypted** payload (Actions `AOTA` images are signed/encrypted). Ghidra and string analysis get nothing from ciphertext, so the Xtreme 4 approach does not transfer here.

## What it would take to unpack (not practical)
- The device-specific OTA **decryption key**, which lives in the ATS2875H secure-boot/bootrom or in Actions/JBL signing infrastructure — not in this file and not publicly available.
- Or a hardware attack on the chip (bootrom/glitch). Out of scope, hardware-dependent, low ROI.

## Consequence for the project
Chasing the Xtreme 5's **proprietary button-mode broadcast** cannot be done from this firmware. The viable route is **over-the-air capture** of what the speaker transmits with the Auracast button on:
- BLE extended + periodic advertising carrying a **BIGInfo** ⇒ audio is LE Audio BIS (promising on the S31, which does BIS).
- Only the JBL manufacturer discovery ad + Classic activity, no LE periodic/BIG ⇒ audio is on Classic CSB/CPB (blocked on the S31 today, IDF-15191).

The standard "Broadcast to Others" / Auracast Assistant path is already confirmed to be real, public, phone-discoverable Auracast — that interop works regardless of this firmware.
