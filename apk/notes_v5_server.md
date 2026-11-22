# JBL One 2.9.4: what a V5 portable (Xtreme 5, PID 2169) must do to be listed and controlled

Paths are relative to `src_one/sources/`. "PVM" = `com/harman/webview/models/PortableV5HybridViewModel.java`.
All multi-byte values are little-endian unless stated. Frame = `u16 0xDD00 | u16 cmd | u8 pktCount | u8 pktIdx | u16 len | payload`
(`com/harman/v5protocol/b.java:74-88`).

Sections 1-2 were traced directly (g3 was checked against the dexdump bytecode, because jadx mangled it).
Sections 3-4 come from a parallel pass over the EQ and product-page code; their key claims were spot-checked
(PVM:3214-3232, `V5PortableEqInfo.java:553-575`, `PortableEQCategoryEnum.java:22-28`).

---------------------------------------------------------------------------------------------------
## 1. Discovery to UI

### 1a. The BLE scan pipeline (all must hold)
1. **Legacy advertising only.** `startScan(null filters, ScanSettings.Builder().setScanMode(..).build())`. No `setLegacy(false)`,
   so extended-only advertising is never reported (`com/harman/discover/BLEScanner$genLoopScanJob$1.java:132-134`).
2. **PID source.** `BLEScanner.S()` first checks manufacturer data for company 0x0057 / 0x0ECB, then service data 0xFDDF bytes 0-1
   (LE, so `69 21` gives "2169") (`BLEScanner.java:430-480`, `T()` 533-543). The PID must pass `a50.y1(pid) || a50.x1(pid)`,
   which means it is in the cloud `configsSupportModels` / cross-Auracast list (`ConfigurationImpl.java:2231-2283`).
   - Avoid any manufacturer data with ID 0x0057/0x0ECB whose first two bytes form a different known PID. That data is checked first.
3. **Parser choice.** `a50.o(pid)` gives HmBleAdvFormat V5, which selects the `auf` parser (`BLEScanner.java:674-690`).
   `auf.i()` checks y1/x1 again (`defpackage/auf.java:58-61`).
4. **Device class.** `s7f.b()` maps category Portable (from cloud `ProductCategory`) to `PortableV5BTDevice`
   (`defpackage/s7f.java:233-249`). `misc.isSupportBrEdr` (misc bit7) adds `PROTOCOL_GATT_BR_EDR` (`s7f.java:290-294`).
   - UUID = "V5_" + btMacAddressCrc (`guf.java:166`).
   - secureBleSupport = misc2 bit15 (`guf.java:132`, `x23.java:71`).
5. **Local name required.** `BLEScanner.W()` drops the device if `scanRecord.getDeviceName()` (adv or scan-response Local Name)
   is null or empty (`BLEScanner.java:597-602`). **The S31 must put a Complete/Shortened Local Name in the adv or scan response.**
6. **Merge on every advert.** `DeviceStore.m0 -> l0 -> D0`, with merger `kh0` calling `old.k7(new)` (`DeviceStore.java:243-247, 386-424`;
   `defpackage/kh0.java`). `BaseBTDevice.k7()` **overwrites `deviceNameCRC`, `secondDeviceNameCRC` and `isSPPConnected` with the
   values from the fresh advert** (`BaseBTDevice.java:642-666`). This matters for 1b.

### 1b. "isBTConnected" (fp0.A7) is the gate for everything
`A7 = BaseBTDevice.c8() = isSPPConnected || s0(deviceNameCRC) || s0(secondDeviceNameCRC)` (`BaseBTDevice.java:572-578`; `fp0.java:82-88`).

- **`s0(crc)`** matches against the phone's own Bluetooth name CRC (`Tools.java:2558-2574`).
  - CRC = CRC-16/ARC over `BluetoothAdapter.getName().getBytes()` (UTF-8), as 4 lowercase hex digits (`t51.java:22-48`).
  - Three variants are accepted: full name; name zero-padded/truncated to 31 bytes; to 63 bytes (`Tools.java:299-325`).
  - The advert field is read as an LE u16 and printed as hex (`v5protocol/discover/a.java:130-131`).
  - So **srcName1Crc (mask bit14) = CRC16/ARC(phone BT name), sent LE.** Example: "Pixel 8" gives 0xf53f, sent as `3f f5`.
- **`isSPPConnected`** is set only by the A2DP / LE-Audio profile poll:
  - `SPPScanner.a.c()` gets `getConnectedDevices()` for profile 2 (A2DP) or 22 (LE Audio) and calls `DeviceStore.C0()`
    (`SPPScanner.java:276-296`; `DeviceStore.java:326-382`).
  - C0 matches a BLE device whose `macAddressCrc == CRC16/ARC(connected device MAC bytes)` (`DeviceStore.java:618-633`;
    `guf.p8` `guf.java:444-447`; `Tools.u3` 2691-2701).
  - On a match, `Z()` sets `isSPPConnected=true`, `deviceNameCRC=phone CRC`, and **macAddress = the A2DP device address**
    (`DeviceStore.java:728-744`).
  - Verified: CRC16/ARC(78 66 f3 ab 11 ce) = 0x10cd, which equals the real speaker's advert macCrc.
- **Problem for the S31.** It advertises mask 0x97C0, with no bit14/13 (srcName CRCs). Its A7 therefore relies only on the A2DP poll.
  Every new advert then resets `isSPPConnected` and `deviceNameCRC` through `k7()`, so A7 flickers false.
  - The real speaker advertises mask 0xD7C0, i.e. **with srcName1Crc**.
  - **Fix: get the phone's name (HCI Remote Name Request after the ACL/A2DP connects), compute CRC16/ARC, and advertise mask
    bit14 with the LE u16. That gives mask 0xD7C0.** Optionally add bit13 srcName2Crc for a second source.

### 1c. Visibility rule for a PortableV5Device (ExtsKt.j3 -> g3)
`j3` (`ExtsKt.java:4710-4763`) hides demo mode first (`h3` = misc2 demo bit, `suf.java:126-131`). It then calls `g3`
(`ExtsKt.java:4495-4506`, verified in bytecode):
```
if (!y1(pid)) return false
if ((A7 || isLeAudioConnected) && (isLeGattConnected(E7) || misc.isConnectable(bit15))) return true
return S1(dev)        // = LocalCacheAdapter.t(uuid) "hadAuthed" (ExtsKt.java:2161-2168, LocalCacheAdapter.java:2446-2459)
```
- `UniversalDeviceObserver.M()` marks a portable V5 as authed (`LocalCacheAdapter.d0` "markAuthed", which also sets the in-memory
  map) as soon as `A7` is true (`UniversalDeviceObserver.java:1719-1751`).
- So in practice: **A7 true once leads to authed, and authed keeps the device visible while it is online.**
- Offline cache merge removes devices whose counterfeit result is FAIL (`MyProductsViewModel.java:2592-2635`, `aa4.i`).

### 1d. Transport and connection choice
- `MyProductsViewModel.q6()` (`:2884-2905`) first returns early if `!aa4.f(dev)` (counterfeit-blocked, see 2e).
  It then runs `onPortableV5DeviceOnlineOrUpdate$4` (`MyProductsViewModel$onPortableV5DeviceOnlineOrUpdate$4.java:67-87`):
  - If `A7 && misc.isConnectable && !secureBle`: it calls `z6()` (LE GATT). For a PortableV5Device, **z6 returns false unless LE Audio
    is connected** (`MyProductsViewModel.java:3504-3508`). **Nothing connects.**
  - Otherwise, if `A7 && Z5 && !standby`: it calls `CommonExtsKt.z()`, i.e. the **BR/EDR GATT connect**.
    - `Z5` = cloud feature `BrEdr` for the PID, and not LE Audio, and not already LE-GATT (`MyProductsViewModel.java:2538-2550`;
      `a50.e1` 939-951).
  - Same choice in `X5`/`ExtsKt.w1`: PortableV5 uses BR/EDR (1) when the PID has the BrEdr feature and LE Audio is not cached
    (`ExtsKt.java:5626-5629`; `MyProductsViewModel.java:2493-2510`).
  - **Recommendation:** advertise misc bit15 isConnectable = 0 (the real speaker's sample misc 0x10c0 has it 0). Set bit7
    isSupportBrEdr = 1. Set misc2 secureBle = 0 or omit misc2.
- **BR/EDR session** (`com/harman/connect/BaseBrEdrSession.java`, `V5BaseBrEdrSession.java`):
  - The session exists only if `macAddress` is set (`BaseBTDevice.java:736-766`). That happens only via the A2DP/LE-Audio match in 1b,
    so **A2DP must be connected, not just bonded, and the advertised macCrc must be CRC16 of the exact address the phone sees.**
  - `getRemoteDevice(mac).connectGatt(ctx, false, cb, TRANSPORT_BREDR=1)` (`BaseBrEdrSession.java:717-758, 826-832`).
    The app does no bonding and no createBond. It relies on the existing classic bond/ACL.
  - The sequence after connect:
    1. Wait 500 ms, then `discoverServices` (`:101, 448-485`).
    2. Look for service `65786365-6c70-6f69-6e74-2e636f6d0000`. If it lacks the characteristics, scan all services for one with both.
    3. Rx (notify) = `...2e636f6d0001`; Tx (write) = `...2e636f6d0002` (`V5BaseBrEdrSession.java:198-203, 279-320`).
  - **Tx is written as WRITE_TYPE_NO_RESPONSE** (`setWriteType(1)`, `V5BaseBrEdrSession.java:322`). Give 0002 the Write and
    Write-Without-Response properties and accept ATT Write Command.
  - It enables notify via CCCD 0x2902 (`G0`, `BaseBrEdrSession.java:557-620`; failure is only logged, `V5BaseBrEdrSession.java:368`).
  - It waits 500 ms, then `C0(500)`: **no requestMtu on BR/EDR; the app assumes MTU 500** (`V5BaseBrEdrSession.java:384`;
    `BaseBrEdrSession.java:521-537`).
    - App writes are split at mtu-8 = 492 B per packet (`rl6.java:38-62`).
    - The splitter re-wraps the buffer each loop, so multi-packet app writes are broken. In practice every app command is < 492 B.
    - **The S31 BR/EDR ATT MTU must accept writes of at least 160 B** (the EQ set); 500+ is safest.
  - The secure char `...0006` is LE-only (`PortableV5GattSession.java:55`) and is used only if secureBle=1. It is not needed.
  - The "excelpoint.cnl" service is unused.
- The LE path (`BaseGattSession.java:1173` transport=2, `V5GattSession.java:1561` requestMtu(500)) is used only with LE Audio.
- **The S31 must expose the excelpoint GATT database over BR/EDR ATT (L2CAP PSM 0x001F).** Check that the ESP32-S31 Bluedroid
  build serves GATT on BR/EDR. **I did not verify this.** If it doesn't, the phone's connectGatt(transport=1) fails and the
  device never connects.

---------------------------------------------------------------------------------------------------
## 2. Connection handshake

### 2a. What "connected" means
- The BR/EDR state becomes CONNECTED right after service discovery, the CCCD write and the 500 ms delay (`BaseBrEdrSession.C0`).
- **No V5 command exchange is required.** The app sends no MTU feature (0x0001), heartbeat or ProductID Get on connect.
  Nothing in the app constructs `V5MTU`; the only `V5HeartBeat` creation is the reply in 2c.
- The connect fails (W() disconnect) if: services are empty, Rx/Tx are not found in any service, or ACL/GATT drops.
- There is no app-level connect timeout on BR/EDR. It relies on the Android GATT timeout.

### 2b. What the app sends after CONNECTED (listeners on BR/EDR status = CONNECTED)
1. **Counterfeit coordinator** (`MyProductsViewModel.java:1883-1889` -> `V5CounterfeitCoordinator.s` `:589-621`). See 2e.
2. **PortableV5DeviceStateMaintainer** (`com/harman/music/mini/PortableV5DeviceStateMaintainer.java:63-89`).
   - It sends one Get: `[2D43 SongName, 2D44 ArtistName, 2D42 AudioVolume, 2D40 AudioOperation, 2D41 AudioMuteStatus]`
     if the cloud feature MusicPlayer is set, plus `2C01 PartyGroupType` if AirTouchSetting is set.
   - Example: `00 DD 01 00 01 00 0C 00 43 2D 44 2D 42 2D 40 2D 41 2D 01 2C`
3. **Product page** (when opened): see section 4.

### 2c. Request/response and heartbeat rules
- Commands are serialized on one send thread. Each write waits up to 6000 ms for `onCharacteristicWrite`
  (`V5BaseBrEdrSession.java:1561-1601`; `rl6.e()` = 6000).
- A command then waits up to 6000 ms for a frame with **the same cmd id** (Get=1 reply cmd 1, Set=2 reply cmd 2)
  (`V5BaseBrEdrSession.java:538-597`). A missing reply stalls the queue 6 s and throws V5ResponseTimeoutException.
  **Always reply.**
- Only cmd 1 and cmd 3 payloads are parsed into features (`V5BaseBrEdrSession.java:563-586`). A cmd-2 reply payload is ignored;
  send `00 00`.
- Payload = `u16 status (00 00 = OK)` + `{u16 id, u16 len, value}*` (`defpackage/swf.java:21-70`).
  - Any non-zero status discards the whole payload.
  - Unknown ids are skipped. The wire `len` is used, not the table size.
- Reassembly:
  - Each GATT notification must be exactly one V5 package.
  - A message is complete when `pktIdx == pktCount-1` (`V5BaseBrEdrSession.java:1610-1628`).
  - Single packet = `pktCount 01, pktIdx 00`.
- **Heartbeat** is device-initiated and optional:
  - The device sends Notify `00 DD 03 00 01 00 06 00 | 00 00 | 02 00 00 00`.
  - The app answers with cmd 4 (hzf) `00 DD 04 00 01 00 04 00 | 02 00 00 00`, fire-and-forget
    (`V5BaseBrEdrSession.java:521-522, 599-601`; `defpackage/hzf.java`).
  - Do not answer the app's cmd 4.
- Device-to-app Forward identifier 0xDD01 is only for a TWS partner. Use 0xDD00.

### 2d. Get example (the real speaker, as in the user's capture)
- App: `00 DD 01 00 01 00 02 00 04 00` (Get ProductID)
- Device: `00 DD 01 00 01 00 08 00 00 00 04 00 02 00 69 21`

### 2e. Counterfeit check (the likely next blocker)
- **When it runs:** `aa4.g(dev)` (`defpackage/aa4.java:93-115`) is true when:
  - the cloud counterfeit config for the PID exists, is signature-valid, has `enable=true`, and its firmware-version condition holds
    (`aa4.l` 245-250), and
  - there is no cached PASS.
- **What it sends** (`com/harman/counterfeit/V5CounterfeitVerifier*.java`, `V5BaseDeviceExtsKt.java:555, 693`; timeout 6000 ms):
  - **Set 0x004A** AuthenticationAlgorithm(algoIndex), expecting a V5AuthAlgorithm back.
  - **Set 0x004B** AdvancedAuthenticateDevice(challenge), expecting a 0x004B dev-info back.
  - The response is checked by the native `CounterfeitLib` against firmware public signatures from the cloud config.
  - **A non-genuine device cannot produce a valid answer.**
- **Results:**
  - FAIL, or UNKNOWN past `unknownThreshold`, with `showPopup` causes: counterfeit popup, BR/EDR + GATT disconnect
    (`MyProductsViewModel.java:1955-1990`), a block on further auto-connect (`q6` -> `aa4.f`, `aa4.java:72-91`), and removal from the
    offline list.
  - With no reply at all, the app gets UNKNOWN (null to the native lib).
- **Unknown:** whether the check is enabled for 2169 is cloud config and not in the APK. Watch logcat tag `V5Counterfeit`.

---------------------------------------------------------------------------------------------------
## 3. EQ (Xtreme 5 = 7-band "portable V5" EQ)

### 3a. Which path
- **Portable path (0x0E7E/0x0E7F):** used only if cloud feature `InAppEQAlgorithm` (`a50.b2`, `a50.java:835-843`) is set for the PID.
  Otherwise the generic 0x0E02 path is used (PVM:3214 vs 3261, 5256 vs 5345). Real Xtreme 5 firmware is portable-path;
  **support both.**
- **Presets:** for 2169 the presets come from the cloud, else `assets/portable_v5_7_bands_preset_eq.json` (`defpackage/t48.java:345`).

### 3b. Reading the EQ: there is no Get
1. JS `getPortableEQ` calls `H7(CURRENT_EQ=0xFF)`.
2. The app sends a **Set (cmd 2) of 0x0E01 EqInfoQuery** with value `FF` (`V5EqInfoQuery.java:61-63`; PVM:3215).
3. It waits 6 s for **0x0E7E EqInfoPortable arriving in a cmd 1/3 frame** (PVM:3218-3232).
4. **The device must: (1) reply cmd 2 `00 00`, then (2) send Notify cmd 3 with 0x0E7E.** An unsolicited 0x0E7E Notify also refreshes the UI.

### 3c. 0x0E7E value (device to app), parsed at `V5PortableEqInfo.java:553-629`. Always at least 8 bytes.
| off | size | meaning |
|---|---|---|
| 0 | 1 | category: VOCAL 02, SIGNATURE 80, RELAXING 81, ENERGETIC 82, EXTREME 83, CHILL 85, CUSTOM C1 (00 is also read as CUSTOM; unknown values read as SIGNATURE) (`PortableEQCategoryEnum.java:22-28`) |
| 1..7 | 7 | gain index for bands 0..6 (read only when CUSTOM; for presets send 0C x7) |

- Bands are fixed in the app (`V5PortableEqInfo.java:576-611`):

  | band | type | freq | Q |
  |---|---|---|---|
  | 0 | LowShelf | 125 Hz | 0.707 |
  | 1 | Peaking | 250 Hz | 2 |
  | 2 | Peaking | 500 Hz | 2 |
  | 3 | Peaking | 1 kHz | 2 |
  | 4 | Peaking | 2 kHz | 2 |
  | 5 | Peaking | 4 kHz | 2 |
  | 6 | HighShelf | 8 kHz | 0.707 |

- Gain index, bands 1-6: `gain_dB = 6.0 - 0.5*idx`, idx 0..24 (00=+6, 0C=0, 18=-6).
- Gain index, band 0: +6..0 in 0.5 dB steps (idx 0..12), then 0.75 dB steps down to -9 dB at idx 24 (tables `:282-283, 612-613`).
  Out-of-range indices throw.

### 3d. Setting the EQ (app to device): Set 0x0E7F EqSetPortable (PVM:5260-5307; `V5PortableEqInfo.java:264-487`)
| part | size | content |
|---|---|---|
| category | 1 | category byte |
| gain indices | 7 | only present when CUSTOM |
| coefficients | 7 × 20 | per band `b0,b1,b2,-a1,-a2` as int32 LE Q29, normalized by a0, fs = 48000 |

- So the value is 148 B for custom and 141 B for a preset. The firmware can ignore the coefficients and use the indices, or use the
  coefficients directly as biquads.
- The app does not check the reply status. Still reply cmd 2 `00 00`, then send a Notify 0x0E7E echo to keep the UI in sync.

### 3e. Example frames (custom: -3, +2, 0, -1.5, +1, +3, +4.5 dB, giving indices 10 08 0C 0F 0A 06 03)
- App query: `00 DD 02 00 01 00 05 00 01 0E 01 00 FF`
- Device: `00 DD 02 00 01 00 02 00 00 00`, then
  `00 DD 03 00 01 00 0E 00 00 00 7E 0E 08 00 C1 10 08 0C 0F 0A 06 03`
- App set: `00 DD 02 00 01 00 98 00 7F 0E 94 00 C1 10 08 0C 0F 0A 06 03` + 140 coefficient bytes
  (full hex in the sub-report; script at the session scratchpad `eq.py`).
- Device: `00 DD 02 00 01 00 02 00 00 00` (+ optional 0x0E7E Notify).
- Preset example (ENERGETIC) Notify value: `82 0C 0C 0C 0C 0C 0C 0C`.

### 3f. Fallback 0x0E02 EqInfo layout (if InAppEQAlgorithm is absent) (`V5EqInfo.java:392-435, 554-627`)
1. u8 category, u8 status, u8 bandCount.
2. f32 calib, f32 sampleRate, f32 leftGain, f32 rightGain (0x4F800000 = null).
3. Per band: u8 type, f32 freq, f32 gain, f32 Q.
4. Optional UTF-8 name.

- Flag `EQBigEndian` makes the floats big-endian and parses from byte 6 (`PortableV5GattSession.java:235-243`, `V5BaseBrEdrSession.java:503-505`).

### 3g. Related one-byte features
- **SmartEQ 0x0E80:** bit7 = on. Read with Get [0E80, 2D47] (PVM:4107, 5511).
- **BassBoost 0x2009:** `(on<<7) | level`, where level 1 = Bass1, 2 = Bass2+, 3 = XL off, 4 = XL on, 7F = none.
  The UI sends 0x81 or 0x82 (`V5BassBoost.java:30-33, 176-228`; PVM:4341-4354).
- **PlaytimeBoost 0x2006:** 01 = on, 00 = off (`V5PlaytimeBoost.java:62-76`; PVM:3759, 5182).

---------------------------------------------------------------------------------------------------
## 4. Product page features

### 4a. How the page behaves
- The page renders from cached state. No feature is mandatory, but **every Get/Set needs a reply** (6 s stall otherwise).
- UnsupportedFeatureList 0x0000 is not read on the portable path (only Cam/HomeBT/Luxury).

### 4b. Requests
- **On page open:**
  - Set 0x0031 SyncTime (16 B) (PVM:1767). Reply `00 00`.
  - Then JS `getDeviceInfo` sends one Get (`PortableV5HybridViewModel$getDeviceInfo$1.java:36-75`; `PortableV5ExtsKt.java:2246-2300`):
    - Always: `000D, 000C, 0008, 2C80, 2C01, 0C40, 1008`.
    - If the PID has the feature: `2009` (BassBoost), `0D45` (BassBoostLight), `0B80` (AuracastAssistant), `2C00` (AirTouchSetting),
      `0012` (StandbySetting), and light features (LightShow 0D40, LightingStatus 0D00, LightColorPresets 0D46, LightBrightness 0D41,
      RGBColorLooping 0D80).
- **Later single Gets:** `2006` (playtime), `0C40` (feedback tone, 6 s), `0045` (power status, 6 s), `2009`,
  `2D43 2D44 2D42 2D41 0BA1` (play info), and the EQ query (3b).
- **After a 2C80 reply:** Get `2C04`, or `2C06` + `2C04` if LLS is on.
- **Settings page:** `0012, 2C00, 0056, 0BA0` (`product/setting/PortableV5ProductSettingViewModel.java:163`).

### 4c. Feature values
| id | name | size | layout | sane value |
|---|---|---|---|---|
| 000D | LeftDeviceBatteryStatus | 1 | b7 charging, b0-6 % | `50` |
| 000C | FirmwareVersion | 3 | bytes joined "a.b.c" | `1A 1D 0C` |
| 0008 | LeftSerialNumber | n | ASCII | 16 chars |
| 0006 | DeviceName | n | UTF-8 | "JBL Xtreme 5" |
| 0004 | ProductID | 2 | `69 21` | |
| 0005 | ColorID | 2 | | `00 01` |
| 000A | DeviceMacAddress | 6 | | |
| 2C80 | LLSConnectionStatus | 1 | 01 = on | `00` |
| 2C01 | PartyGroupType | 1 | 0 Normal … 5 AirTouchStereo | `00` |
| 2C00 | PartyStatus | 1 | b7 auracastParty … | `80` |
| 2C04 | PartyChannelType | 1 | | `00` |
| 0C40 | FeedbackToneStatusForV5Portable | 1 | b7 = on | `80` |
| 1008 | TotalPlaybackTime | 4 | u32 minutes | `B0 04 00 00` |
| 2009 | BassBoost | 1 | see 3g | `81` |
| 0D45 | BassBoostLight | 1 | | `00` |
| 0B80 | AuracastAssistantInfo | 0 | empty = idle | len 0 |
| 0012 | AutoPowerOff | 2 | u16, b15 on, b0-14 seconds | `08 87` (on, 1800 s) |
| 0013 | AutoStandby | 2 | same as AutoPowerOff | `08 87` |
| 2006 | PlaytimeBoost | 1 | 01 = on | `00` |
| 0045 | PowerStatus | 1 | 01 = on | `01` |
| 0056 | RemotePowerEnableStatus | 1 | | `00` |
| 0BA0 | LEAudioStatusForV5Portable | 1 | b7 = on | `00` |
| 2D42 | AudioVolume | 1 | 0..100 | `32` |
| 2D41 | AudioMuteStatus | 1 | | `00` |
| 2D40 | AudioOperation | 1 | 1 pause, 2 play, … | `01` |
| 0BA1 | ActiveAudioSource | 1 | 1 = BT | `01` |
| 2D43 / 2D44 | SongName / ArtistName | n | UTF-8 | len 0 |
| 2004 | AuracastMode | 1 | | `01` |
| 0D62 | ChargingLight | 1 | | `00` |

### 4d. Sets and Notifies
- **Sets** (reply cmd 2 `00 00`):
  - Playtime boost: `06 20 01 00 0v`
  - Feedback tone: `40 0C 01 00 80/00`
  - Bass boost: `09 20 01 00 xx`
  - Volume: `42 2D 01 00 vv`
  - Auto power off: `12 00 02 00 lo hi`
- **Unsolicited Notifies the app handles** (format: cmd 3, `00 00` + records): 000D, 2D42, 2D41, 0BA1, 2D40, 2D43, 2D44, 2006, 0C40,
  0012, 2C00-2C04, 2C80, 0045, 0056, 2009, 0E7E, 0006, 0002.

---------------------------------------------------------------------------------------------------
## 5. Checklist for the S31
1. Use legacy ADV_IND with a Local Name (adv or scan response) and 0xFDDF service data. Mask **0xD7C0**:
   - colorID
   - **srcName1Crc = CRC16/ARC(phone BT name), sent LE**
   - btMacAddressCrc = CRC16/ARC(BR/EDR MAC as the phone sees it), sent LE
   - battery, misc, partyMethod, partyInfo, stereoGroupId
2. misc: bit15 isConnectable = **0**, bit7 isSupportBrEdr = 1. No misc2 (or misc2 = 0).
3. Keep A2DP **connected**, not just bonded. Its address must be the one the macCrc is computed from.
4. Serve the excelpoint service over **BR/EDR ATT**:
   - 0001 notify + CCCD
   - 0002 write-without-response
   - accept writes ≥ 160 B
5. Answer every cmd 1/2 with the same cmd id and status `00 00`. Implement the 0x0E01-to-0x0E7E notify flow and 0x0E7F.
   Answer the 4b Get list.
6. If logcat shows `V5Counterfeit` with enable=true for 2169, the device will be flagged. The emulator cannot satisfy that
   challenge/response.
