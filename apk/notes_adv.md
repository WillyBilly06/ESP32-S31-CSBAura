# JBL Xtreme 5 BLE adv notes (from JBL One 2.9.4, src_one)

Parser: com/harman/v5protocol/discover/a.java (V5BLELegacyAdvParseMixin) d() L73-352
Service data UUID 0xFDDF, all multi-byte fields little-endian.

[0-1]  pid LE (69 21 -> "2169")                     L106
[2-3]  presence mask LE (c0 d7 -> 0xd7c0)          L109-128, table L50
then fields in mask order bit15..bit1, present only if bit set:
 b15 colorID 1B            L129
 b14 srcName1Crc 2B        L130  (CRC16 of connected phone BT name)
 b13 srcName2Crc 2B        L131
 b12 btMacAddressCrc 2B    L132  (CRC16/ARC of speaker classic MAC) -> UUID "V5_"+crc (guf.java:166)
 b11 btMacAddress 6B       L133-144
 b10 battery 1B            L145-154 (b7 charging, b0-6 level)
 b9  deviceMiscInfo 2B     L155-193
 b8  partyMethodInfo 1B    L194-201 (b7 auracast, b6 LLStereo, b5 OneCast/LEA pairing)
 b7  partyInfo 2B          L202-285 (b15 on, b13-14 group, b11-12 conn, b9-10 role, b7-8 chan, b6 join)
 b6  stereoGroupId 1B      L286
 b5 fw3 3B, b4 deviceId 3B, b3 partyLight 2B, b2 fw5 5B, b1 misc2 2B

CRC16: defpackage/t51.java (table a[1]=0xC0C1 => CRC-16/ARC, init 0, no xorout)
MAC bytes for CRC: Tools.L1 (Tools.java:969-992) "AA:BB:..:FF" -> [AA..FF]
Phone name CRCs: Tools.java:298-325, s0() 2558-2572; t51.c/d (d pads/truncates with Arrays.copyOf)

Sample decode 69 21 c0 d7 00 60 90 cd 10 35 c0 10 c0 40 aa 00:
 pid 2169, mask d7c0, color 00, srcName1Crc 0x9060, macCrc 0x10cd, batt 53% not charging,
 misc 0x10c0 (LED, BR/EDR, JBL app connected), partyMethod c0 (Auracast + LLStereo),
 partyInfo 0xaa40 (on, Party, Connecting, Secondary, Full, join=1), stereoGroupId 0
 0xb440 = on, Party, Connected, Primary ; 0xac40 = on, Party, Connecting, Primary ; 0x0040 = off

Periodic-adv 0xFDDF TLV / Broadcast ID / BASE: NOT parsed anywhere in either app (no
PeriodicAdvertising / BluetoothLeBroadcast* / 0x1852 references). Firmware-only.
Xtreme 4 pid 20dc (command/portable/a.java:2662). Xtreme 5 pid 2169 not hard-coded (remote config).
