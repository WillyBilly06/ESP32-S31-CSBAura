# Mirrors the pbp/pbs example layout. Copy adv.h + the matching adv.c
# (bluedroid/ or nimble/) from that example into this main/ folder, and this
# file compiles the right one for your selected host stack.
set(srcs "auracast_main.c")
if(CONFIG_BT_BLUEDROID_ENABLED)
    list(APPEND srcs "bluedroid/adv.c")
else()
    list(APPEND srcs "nimble/adv.c")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash driver esp_timer
)
