idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        system_monitor
        nvs_flash
        esp_netif
        esp_event
        log
        config
        
)