idf_component_register(
    SRCS "env_mapping.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos esp_common
    # math library is provided by the toolchain (libm)
)
