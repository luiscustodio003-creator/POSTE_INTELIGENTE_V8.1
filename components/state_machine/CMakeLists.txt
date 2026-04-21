idf_component_register(
    SRCS
        "state_machine.c"
        "state_machine_task.c"
    INCLUDE_DIRS "."
    REQUIRES
        dali_manager
        comm_manager
        radar_manager
        display_manager
        system_monitor
        config
        freertos
        esp_timer
        esp_system
        log
)
