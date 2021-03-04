idf_component_register(
    SRCS "bmi270.c" "bmi2.c" "common.c"
    INCLUDE_DIRS .
    REQUIRES i2cdev log esp_idf_lib_helpers
)
