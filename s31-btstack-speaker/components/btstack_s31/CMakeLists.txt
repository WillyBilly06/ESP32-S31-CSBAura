# BTstack host for the ESP32-S31 (controller-only build, VHCI transport).
# BTstack itself lives in ../../third_party/btstack (git clone of
# https://github.com/bluekitchen/btstack, kept out of this repo: its license
# allows personal, non-commercial use). Only the parts this speaker needs are
# compiled: core, Classic (A2DP sink + AVRCP + SDP), BLE (ATT/GATT/SM),
# LE Audio helpers (BASE builder/parser), the Bluedroid SBC codec, micro-ecc
# and the FreeRTOS run loop. LC3 comes from our own components/liblc3.

get_filename_component(BTSTACK_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../third_party/btstack" ABSOLUTE)
if(NOT EXISTS "${BTSTACK_ROOT}/src/hci.c")
    message(FATAL_ERROR "BTstack not found at ${BTSTACK_ROOT} - git clone https://github.com/bluekitchen/btstack third_party/btstack")
endif()

set(B ${BTSTACK_ROOT})

set(core_srcs
    ${B}/src/ad_parser.c
    ${B}/src/btstack_audio.c
    ${B}/src/btstack_crypto.c
    ${B}/src/btstack_hid_parser.c
    ${B}/src/btstack_linked_list.c
    ${B}/src/btstack_linked_queue.c
    ${B}/src/btstack_ltv_builder.c
    ${B}/src/btstack_memory.c
    ${B}/src/btstack_memory_pool.c
    ${B}/src/btstack_ring_buffer.c
    ${B}/src/btstack_run_loop.c
    ${B}/src/btstack_run_loop_base.c
    ${B}/src/btstack_tlv.c
    ${B}/src/btstack_util.c
    ${B}/src/hci.c
    ${B}/src/hci_cmd.c
    ${B}/src/hci_dump.c
    ${B}/src/hci_event.c
    ${B}/src/l2cap.c
    ${B}/src/l2cap_signaling.c
)

set(classic_srcs
    ${B}/src/classic/a2dp.c
    ${B}/src/classic/a2dp_sink.c
    ${B}/src/classic/avdtp.c
    ${B}/src/classic/avdtp_acceptor.c
    ${B}/src/classic/avdtp_initiator.c
    ${B}/src/classic/avdtp_sink.c
    ${B}/src/classic/avdtp_util.c
    ${B}/src/classic/avrcp.c
    ${B}/src/classic/avrcp_controller.c
    ${B}/src/classic/avrcp_target.c
    ${B}/src/classic/btstack_link_key_db_tlv.c
    ${B}/src/classic/btstack_sbc_bluedroid.c
    ${B}/src/classic/btstack_sbc_decoder_bluedroid.c
    ${B}/src/classic/btstack_sbc_plc.c
    ${B}/src/classic/device_id_server.c
    ${B}/src/classic/gatt_sdp.c
    ${B}/src/classic/sdp_client.c
    ${B}/src/classic/sdp_server.c
    ${B}/src/classic/sdp_util.c
)

set(ble_srcs
    ${B}/src/ble/att_db.c
    ${B}/src/ble/att_db_util.c
    ${B}/src/ble/att_dispatch.c
    ${B}/src/ble/att_server.c
    ${B}/src/ble/gatt_client.c
    ${B}/src/ble/le_device_db_tlv.c
    ${B}/src/ble/sm.c
    ${B}/src/le-audio/le_audio_base_builder.c
    ${B}/src/le-audio/le_audio_base_parser.c
    ${B}/src/le-audio/le_audio_util.c
)

file(GLOB sbc_srcs ${B}/3rd-party/bluedroid/decoder/srce/*.c)

set(platform_srcs
    ${B}/platform/freertos/btstack_run_loop_freertos.c
    ${B}/platform/embedded/hci_dump_embedded_stdout.c
    ${B}/3rd-party/micro-ecc/uECC.c
)

idf_component_register(
    SRCS ${core_srcs} ${classic_srcs} ${ble_srcs} ${sbc_srcs} ${platform_srcs}
         "hci_transport_s31_vhci.c" "btstack_tlv_esp32.c"
    INCLUDE_DIRS "include"
                 ${B}/src ${B}/src/classic ${B}/src/ble ${B}/src/le-audio
                 ${B}/platform/freertos ${B}/platform/embedded
                 ${B}/3rd-party/bluedroid/decoder/include
                 ${B}/3rd-party/bluedroid/encoder/include
                 ${B}/3rd-party/micro-ecc ${B}/3rd-party/yxml ${B}/3rd-party/md5
    REQUIRES bt nvs_flash freertos
)
# third-party code: keep its warnings out of -Werror
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-unused-variable -Wno-unused-but-set-variable
                       -Wno-unused-function -Wno-format -Wno-implicit-fallthrough -O2)
