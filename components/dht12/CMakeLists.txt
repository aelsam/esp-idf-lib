set(req driver esp32 freertos esp_idf_lib_helpers)

idf_component_register(
    SRCS "dht12.c"
    INCLUDE_DIRS .
    REQUIRES ${req}
)
