idf_component_register(
    SRCS
        "display_manager.c"
        "display_manager_task.c"
    INCLUDE_DIRS "."
    REQUIRES
        display_driver
        state_machine
        radar_manager
       
        config
        esp_lcd
)