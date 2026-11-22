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
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

/* ---- BR/EDR: A2DP sink (+ AVRCP), GATT over BR/EDR for JBL One ----------- */
#define ENABLE_CLASSIC
#define ENABLE_GATT_OVER_CLASSIC
#define ENABLE_CROSS_TRANSPORT_KEY_DERIVATION
#define NVM_NUM_LINK_KEYS 16
/* one 2-DH5 (679 B payload) per A2DP media packet: at bitpool 53 the phone packs
 * 5 SBC frames (~608 B), and each media packet is ONE baseband exchange that fits
 * between Auracast BIG events (btstack-patches/0001 makes this overridable) */
#define AVDTP_L2CAP_MTU 1005   /* 8 SBC frames (bitpool 53) per 3-DH5; Android caps at 1005 with EDR 3M, 663 without */

/* ---- LE: JBL beacon + V5 GATT, Auracast source/sink ---------------------- */
#define ENABLE_BLE
#define ENABLE_LE_PERIPHERAL
#define ENABLE_LE_CENTRAL
#define ENABLE_LE_SECURE_CONNECTIONS
/* P-256 in software (the S31 controller has no HCI P-256 commands): BTstack's
 * micro-ecc, compiled with its public symbols renamed (see the component's
 * CMakeLists) because the IDF bt component links its own, ABI-incompatible copy. */
#define ENABLE_MICRO_ECC_FOR_LE_SECURE_CONNECTIONS
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
#define MAX_NR_HCI_ISO_STREAMS    4

#endif
