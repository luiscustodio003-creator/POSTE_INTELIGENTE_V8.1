idf_component_register(
    SRCS "post_config.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        log
)
