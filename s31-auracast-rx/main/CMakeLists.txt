set(srcs "main.c" "lc3_i2s_player.c")

if(CONFIG_BT_BLUEDROID_ENABLED)
    list(APPEND srcs "bluedroid/scan.c")
else()
    list(APPEND srcs "nimble/scan.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       REQUIRES bt nvs_flash liblc3 esp_driver_i2s esp_timer)
