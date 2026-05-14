idf_component_register(
    SRCS "tracking_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        radar_manager
        system_monitor
        display_manager
        wifi_manager
        config
        freertos
        esp_timer
        log
        newlib
)