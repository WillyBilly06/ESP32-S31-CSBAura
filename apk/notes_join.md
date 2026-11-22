# JBL Xtreme 5 (V5 protocol) Auracast join - notes from JBL One 2.9.4

Paths relative to src_one/sources/.

## Transport
- GATT service 65786365-6c70-6f69-6e74-2e636f6d0000 ("excelpoint.com"); write ...0002, notify ...0001.
  BaseGattSession.java:130-142, V5GattSession.java:236. Secure-write alt char ...0006 (PortableV5GattSession.java:55),
  used only if adv says secureBleSupport==1 (BaseBTDevice.java:695). No app-layer crypto.
- BR/EDR: same GATT over classic (connectGatt(...,transport=1)) BaseBrEdrSession.java:104-112,758.
- "excelpoint.cnl" (...636e6c0000) is not referenced anywhere in either app.

## V5 frame (com/harman/v5protocol/b.java:74-88), all little-endian
  u16 identifier (0xDD00 Direct, 0xDD01 Forward(to TWS partner), 0xDD02 Sub)  V5Identifier.java:15-17
  u16 commandId  (1=GetDevInfo, 2=SetDevInfo, 3=Notify(dev->app), 4=NotifyAck/heartbeat, 0x101-0x104 OTA, 0x201, 0x2FF analytics)
  u8  packetCount, u8 packetIndex, u16 payloadLen, payload
- Get payload: list of u16 featureIds (defpackage/yyf.java)
- Set payload: repeated {u16 featId, u16 len, value} (defpackage/i2g.java)
- Response/Notify payload: u16 status (0x0000 = OK) then repeated {u16 featId, u16 len, value} (defpackage/swf.java)
- Responses use same commandId as request (V5GattSession.java:1073-1108). No checksum.

## Auracast-assistant features (V5DevInfoFeatID.java:185-196)
 0x0B80 AssistantInfo (R)   0x0B81 ScanControl (RW,2)  0x0B82 GroupInfo (R,40/41 - one notify per found source)
 0x0B83 GroupSelection (RW) 0x0B84 SubgroupInfo (R)    0x0B85 SubgroupPlay (RW,3)
 0x0B86 BroadcasterEnable   0x0B87 BroadcasterStart    0x0B88 BroadcasterPassword(16)
 0x0B89 AssistantMacAddress(6) 0x0B8A BroadcasterName  0x0B8B SubgroupName (n x {u8 idx, 32B name})
Party: 0x2004 AuracastMode (0 Normal,1 On,2 OnUnlinked,3 OnLinked), 0x2005 AuracastStandardQuality,
 0x2C00 PartyStatus bitmap, 0x2C01 PartyGroupType, 0x2C02 ConnStatus, 0x2C03 Role, 0x2C04 Channel,
 0x2C05 GroupName(16), 0x2C06 GroupID(4 ASCII), 0x2C07 GroupAction, 0x2C81 PartnerMac, 0x2C80 LLS status.

## Example frames
 scan start 60 s : 00 DD 02 00 01 00 06 00 | 81 0B 02 00 01 3C
 scan stop       : 00 DD 02 00 01 00 06 00 | 81 0B 02 00 00 3C
 join AA:BB:CC:DD:EE:FF random, subgroup 0, std quality, no code:
   00 DD 02 00 01 00 14 00 | 85 0B 03 00 01 00 00 | 83 0B 09 00 01 01 AA BB CC DD EE FF FF
 leave: same with 85 0B 03 00 00 <idx> <q> | 83 0B 09 00 00 <type> <addr> FF
 query status    : 00 DD 01 00 01 00 02 00 | 80 0B
 "press Auracast button": 00 DD 02 00 01 00 05 00 | 04 20 01 00 01
