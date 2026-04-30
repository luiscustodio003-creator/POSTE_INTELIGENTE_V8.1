/* ============================================================
   SUPERVISOR DO SISTEMA
   @file      system_monitor.c
   @version   5.1  |  2026-04-29
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   CORRECÇÕES v5.0 → v5.1:
   - display_manager_set_hw_status() → display_manager_set_hardware()
   - wifi_manager_set_ip_callback() removido (não existe)
     UDP iniciado por polling em _monitor_task (padrão original)
   - Bloco #else USE_RADAR removido (só radar real)
============================================================ */
#include "system_monitor.h"
#include "state_machine.h"
#include "radar_manager.h"
#include "display_manager.h"
#include "udp_manager.h"
#include "comm_manager.h"
#include "wifi_manager.h"
#include "dali_manager.h"
#include "system_config.h"
#include "post_config.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "SYS_MON";

static uint64_t s_hb_ms[MOD_COUNT] = {0};
static bool     s_comm_ok          = false;

static const uint32_t s_timeout_ms[MOD_COUNT] = { 500, 500, 200, 100 };
static const char    *s_nome[MOD_COUNT]        = { "FSM","RADAR","DISPLAY","UDP" };

void system_monitor_heartbeat(monitor_module_t mod)
{
    if (mod < MOD_COUNT)
        s_hb_ms[mod] = (uint64_t)(esp_timer_get_time() / 1000ULL);
}

static void _monitor_task(void *arg)
{
    ESP_LOGI(TAG, "monitor_task | Core %d | Prio 7", xPortGetCoreID());
    esp_task_wdt_add(NULL);

    uint64_t agora = (uint64_t)(esp_timer_get_time() / 1000ULL);
    for (int i = 0; i < MOD_COUNT; i++) s_hb_ms[i] = agora;

    while (1) {
        esp_task_wdt_reset();

        agora = (uint64_t)(esp_timer_get_time() / 1000ULL);
        for (int i = 0; i < MOD_COUNT; i++) {
            uint64_t delta = agora - s_hb_ms[i];
            if (delta > (uint64_t)s_timeout_ms[i])
                ESP_LOGW(TAG, "%s sem heartbeat %llums",
                         s_nome[i], (unsigned long long)delta);
        }

        if (wifi_manager_is_connected()) {
            if (!s_comm_ok) {
                if (comm_init()) {
                    s_comm_ok = true;
                    ESP_LOGI(TAG, "UDP activo");
                }
            }
        } else {
            if (s_comm_ok) {
                s_comm_ok = false;
                display_manager_set_wifi(false, NULL);
            }
        }

        if (s_comm_ok) {
            char nL[MAX_IP_LEN] = {0}, nR[MAX_IP_LEN] = {0};
            udp_manager_get_neighbors(nL, nR);
            display_manager_set_neighbors(nL, nR,
                comm_left_online(), comm_right_online());
            display_manager_set_wifi(true, wifi_manager_get_ip());
        }

        display_manager_set_hardware(
            radar_get_status_str(),
            state_machine_radar_ok(),
            (uint8_t)dali_get_brightness()
        );
        display_manager_set_status(state_machine_get_state_name());
        display_manager_set_traffic(state_machine_get_T(), state_machine_get_Tc());

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void system_monitor_start(void)
{
    ESP_LOGI(TAG, "╔═══════════════════════════════════╗");
    ESP_LOGI(TAG, "║  Poste Inteligente v8 | %-10s║", POSTE_NAME);
    ESP_LOGI(TAG, "╚═══════════════════════════════════╝");

    post_config_init();
    ESP_LOGI(TAG, "[1] ID=%d %s pos=%d",
             post_get_id(), post_get_name(), POST_POSITION);

    dali_init();
    dali_set_brightness(LIGHT_MIN);
    radar_init(RADAR_MODE_UART);
    int baud = radar_auto_detect_baud();
    ESP_LOGI(TAG, "[2] Radar UART baud=%d | DALI min=%d%%", baud, LIGHT_MIN);
    radar_diagnostic();

    display_manager_init();
    ESP_LOGI(TAG, "[3] Display OK");

    state_machine_init();
    ESP_LOGI(TAG, "[4] FSM OK");

    wifi_manager_init();
    ESP_LOGI(TAG, "[5] WiFi STA iniciado");

    state_machine_task_start();
    radar_manager_task_start();
    display_manager_task_start();
    udp_manager_task_start();
    ESP_LOGI(TAG, "[6] Tasks criadas");

    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms     = SYSTEM_WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    esp_task_wdt_reconfigure(&wdt_cfg);
    ESP_LOGI(TAG, "[7] WDT %ds | Sistema operacional", SYSTEM_WDT_TIMEOUT_S);

    xTaskCreatePinnedToCore(_monitor_task, "monitor_task",
                            3072, NULL, 7, NULL, 1);
}
