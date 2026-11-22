set(srcs "main.c" "jbl_gatt.c" "a2dp_bridge.c" "jbl_assistant.c" "jbl_v5_server.c" "bt_sched_tune.c")

if(CONFIG_BT_BLUEDROID_ENABLED)
    list(APPEND srcs "bluedroid/adv.c")
else()
    list(APPEND srcs "nimble/adv.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       REQUIRES bt nvs_flash liblc3 esp_ringbuf esp_psram esp_timer)

# Radio priority profile: Auracast first copies on top (15), A2DP next (8),
# then BIS repetitions / adverts. Measured: A2DP_FIRST lost ~65% of Auracast
# SDUs while music played (phone link perfect); see bt_sched_tune.h.
target_compile_definitions(${COMPONENT_LIB} PRIVATE BT_SCHED_MODE=BT_SCHED_MODE_AURACAST_FIRST)
