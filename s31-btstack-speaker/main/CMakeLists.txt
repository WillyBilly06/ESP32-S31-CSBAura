idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS "."
                       REQUIRES btstack_s31 liblc3 helix_aac nvs_flash esp_timer esp_driver_i2s esp_driver_uart esp_partition)

# controller BIGInfo parser wrapper (ctrl_wrap.c): stereo pair secondary diagnostics
target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=r_sym_ble_rOljucnfkqMusVymFaAm")
