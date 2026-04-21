idf_component_register(
    SRCS "comm_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        udp_manager
        wifi_manager
        config
        esp_timer
        log
)
