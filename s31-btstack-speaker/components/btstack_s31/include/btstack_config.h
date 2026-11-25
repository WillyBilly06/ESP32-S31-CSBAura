/*
 * BTstack configuration for the ESP32-S31 JBL speaker (controller-only build).
 */
#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

/* ---- port ---------------------------------------------------------------- */
#define HAVE_ASSERT
#define HAVE_EMBEDDED_TIME_MS
#define HAVE_FREERTOS_INCLUDE_PREFIX
#define HAVE_FREERTOS_TASK_NOTIFICATIONS
#define HAVE_MALLOC

/* ---- logging ------------------------------------------------------------- */
#if S31_DEV                     /* BTstack error log: development builds only */
#define ENABLE_LOG_ERROR
#endif
#define ENABLE_PRINTF_HEXDUMP

/* ---- BR/EDR: A2DP sink (+ AVRCP), GATT over BR/EDR for JBL One ----------- */
#define ENABLE_CLASSIC
#define ENABLE_GATT_OVER_CLASSIC
/* CTKD: one LE pairing (Android pairs a dual-mode LE Audio speaker over LE) also
 * gives the Classic link key, so the phone switches LE Audio <-> A2DP without a new
 * pairing. The derived key is P-256; with BR/EDR Secure Connections off here the
 * link comes up E0 and BTstack's downgrade check (only active with SC on) stays out. */
#define ENABLE_CROSS_TRANSPORT_KEY_DERIVATION
#define NVM_NUM_LINK_KEYS 16
/* one 2-DH5 (679 B payload) per A2DP media packet: at bitpool 53 the phone packs
 * 5 SBC frames (~608 B), and each media packet is ONE baseband exchange that fits
 * between Auracast BIG events (btstack-patches/0001 makes this overridable) */
/* 1005: 8 SBC frames (bitpool 53) per 3-DH5, ~43 media packets/s. 663 (5 frames,
 * ~69/s) starved a Galaxy S26 at a 20 ms poll; the iPhone's losses at 1005 came
 * from the same poll limit (no room for retries), now fixed in link_tune.c. */
#define AVDTP_L2CAP_MTU 1005

/* ---- LE: JBL beacon + V5 GATT, Auracast source/sink ---------------------- */
#define ENABLE_BLE
#define ENABLE_LE_PERIPHERAL
#define ENABLE_LE_CENTRAL
#define ENABLE_LE_SECURE_CONNECTIONS
/* P-256 in software (the S31 controller has no HCI P-256 commands): BTstack's
 * micro-ecc, compiled with its public symbols renamed (see the component's
 * CMakeLists) because the IDF bt component links its own, ABI-incompatible copy. */
#define ENABLE_MICRO_ECC_FOR_LE_SECURE_CONNECTIONS
/* AES on the host, not the controller's HCI LE Encrypt: BTstack does not check LE Encrypt's
 * status, and on some pairings the S31 controller's answer was not a result - the LE SC
 * Confirm went out as RAM bytes (00 00 00 00 2e 00 13 f8 2f 04 c0 00...) and the phone failed
 * the pairing (0x05 / confirm value failed, "Couldn't pair"). btstack_aes128_calc() is ours
 * (btstack_aes128_esp.c): the S31's AES peripheral, with BTstack's rijndael as the fallback. */
#define HAVE_AES128
#define ENABLE_LE_DATA_LENGTH_EXTENSION
#define ENABLE_LE_EXTENDED_ADVERTISING
#define ENABLE_LE_PERIODIC_ADVERTISING
#define ENABLE_LE_ISOCHRONOUS_STREAMS
#define ENABLE_ATT_DELAYED_RESPONSE
#define NVM_NUM_DEVICE_DB_ENTRIES 16
#define MAX_ATT_DB_SIZE 1024

/* ---- HCI buffers --------------------------------------------------------- */
/* 2-DH5 ACL (679 B payload) for A2DP media; ISO SDUs are 100 B. */
#define HCI_ACL_PAYLOAD_SIZE      (1021 + 4)
#define HCI_HOST_ACL_PACKET_LEN   1024
#define HCI_HOST_ACL_PACKET_NUM   12
#define HCI_HOST_SCO_PACKET_LEN   0
#define HCI_HOST_SCO_PACKET_NUM   0
#define HCI_ISO_PAYLOAD_SIZE      (251)
/* transport ring buffer: ISO SDUs received while listening to a master */
#define S31_HOST_ISO_PACKET_LEN   (HCI_ISO_PAYLOAD_SIZE + 4)
#define S31_HOST_ISO_PACKET_NUM   16
#define MAX_NR_HCI_ISO_STREAMS    6   /* broadcast BIG 2 + listen sync 2 + LE Audio CIS 1 + spare */

#endif
