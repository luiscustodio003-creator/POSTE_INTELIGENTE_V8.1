idf_component_register(
    SRCS "wifi_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        display_manager
        esp_wifi
        esp_event
        esp_netif
        esp_timer
        freertos
        log
        config
)
