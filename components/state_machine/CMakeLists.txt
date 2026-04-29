# ============================================================
# MÁQUINA DE ESTADOS — BUILD ESP-IDF
# @file      CMakeLists.txt
# @version   5.0  |  2026-04-29
# PROJECTO   : Poste Inteligente v8
# AUTORES    : Luis Custódio | Tiago Moreno
#
# MUDANÇAS v4 → v5:
#   - MANTIDO:  esp_system  (esp_timer_get_time via esp_timer)
# ============================================================

idf_component_register(
    SRCS
        "fsm_core.c"
        "fsm_events.c"
        "fsm_network.c"
        "fsm_timer.c"
        "fsm_task.c"
    INCLUDE_DIRS "."
    REQUIRES
        dali_manager
        comm_manager
        radar_manager
        display_manager
        tracking_manager
        system_monitor
        config
        freertos
        esp_timer
        esp_system
        log
)
