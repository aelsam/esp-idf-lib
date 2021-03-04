set(COMPONENT_ADD_INCLUDEDIRS "./HLW8110/include" )

set(COMPONENT_SRCS "./HLW8110/HLW8110.c")
set(COMPONENT_PRIV_REQUIRES "json" "esp_http_client")

register_component( )
component_compile_options(-Wno-unused-function)