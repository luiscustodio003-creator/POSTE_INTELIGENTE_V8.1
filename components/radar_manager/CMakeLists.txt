idf_component_register(
    SRCS
        "radar_manager.c"
        "radar_manager_task.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        config
        freertos
        esp_timer
        system_monitor
        log
        tracking_manager
)