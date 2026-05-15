idf_component_register(
    SRCS "tracking_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        radar_manager
        state_machine
        config
        freertos
        esp_timer
        log
        newlib
)