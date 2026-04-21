idf_component_register(
    SRCS "system_monitor.c"
    INCLUDE_DIRS "."
    REQUIRES
        state_machine
        radar_manager
        display_manager
        udp_manager
        comm_manager
        wifi_manager
        dali_manager
        config
        nvs_flash
        freertos
        esp_timer
        esp_system
        log
)
