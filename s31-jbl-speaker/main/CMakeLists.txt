set(srcs "main.c" "jbl_gatt.c" "a2dp_bridge.c" "jbl_assistant.c" "jbl_v5_server.c")

if(CONFIG_BT_BLUEDROID_ENABLED)
    list(APPEND srcs "bluedroid/adv.c")
else()
    list(APPEND srcs "nimble/adv.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       REQUIRES bt nvs_flash liblc3 esp_ringbuf esp_psram esp_timer)
