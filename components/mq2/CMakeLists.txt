set(req driver esp32 freertos esp_idf_lib_helpers esp_adc_cal)

idf_component_register(
    SRCS "mq2.c"
    INCLUDE_DIRS .
    REQUIRES ${req}
)
