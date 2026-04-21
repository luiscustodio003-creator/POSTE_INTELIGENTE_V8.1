idf_component_register(
    SRCS "udp_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        lwip
        esp_timer
        freertos
        log
        config
        system_monitor
)
