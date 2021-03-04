set(COMPONENT_ADD_INCLUDEDIRS "./HLW8112/include" )

set(COMPONENT_SRCS "./HLW8112/HLW8112.c")
set(COMPONENT_PRIV_REQUIRES "json" "nvs_flash" "esp_http_client")

register_component( )
component_compile_options(-Wno-unused-function)