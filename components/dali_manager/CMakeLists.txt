idf_component_register(
    SRCS "dali_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        freertos
        log
        config
)
