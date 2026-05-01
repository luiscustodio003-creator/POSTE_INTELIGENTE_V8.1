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
    /* ── Banner principal ── */
    printf("\n");
    printf("╔═══════════════════════════════════════╗\n");
    printf("║    Poste Inteligente v8  |  %-9s  ║\n", POSTE_NAME);
    printf("╠═══════════════════════════════════════╣\n");
    printf("║  Projecto: Luis Custodio | T. Moreno  ║\n");
    printf("╚═══════════════════════════════════════╝\n");
    printf("\n");

    /* ── [1] Identidade ── */
    post_config_init();
    printf("┌───────────────────────────────────────┐\n");
    printf("│ [1] ID: %-2d  Nome: %-12s  Pos: %d │\n",
           post_get_id(), post_get_name(), POST_POSITION);
    printf("└───────────────────────────────────────┘\n");

    /* ── [2] Hardware ── */
    dali_init();
    dali_set_brightness(LIGHT_MIN);
    radar_init(RADAR_MODE_UART);
    int baud = radar_auto_detect_baud();
    printf("┌───────────────────────────────────────┐\n");
    printf("│ [2] Radar  UART  baud=%-6d           │\n", baud);
    printf("│     DALI   GPIO26  min=%d%%              │\n", LIGHT_MIN);
    printf("└───────────────────────────────────────┘\n");
    radar_diagnostic();

    /* ── [3] Display ── */
    display_manager_init();
    printf("┌───────────────────────────────────────┐\n");
    printf("│ [3] Display  ST7789  OK               │\n");
    printf("└───────────────────────────────────────┘\n");

    /* ── [4] FSM ── */
    state_machine_init();
    printf("┌───────────────────────────────────────┐\n");
    printf("│ [4] FSM  inicializada                 │\n");
    printf("└───────────────────────────────────────┘\n");

    /* ── [5] WiFi ── */
    wifi_manager_init();
    printf("┌───────────────────────────────────────┐\n");
    printf("│ [5] WiFi  STA  SSID: %-16s │\n", WIFI_SSID);
    printf("└───────────────────────────────────────┘\n");

    /* ── [6] Tasks ── */
    state_machine_task_start();
    radar_manager_task_start();
    display_manager_task_start();
    udp_manager_task_start();
    printf("┌───────────────────────────────────────┐\n");
    printf("│ [6] Tasks criadas                     │\n");
    printf("│     FSM    Core1  Prio6  100ms        │\n");
    printf("│     RADAR  Core0  Prio6  100ms        │\n");
    printf("│     DISP   Core0  Prio4   20ms        │\n");
    printf("│     UDP    Core0  Prio5   10ms        │\n");
    printf("└───────────────────────────────────────┘\n");

    /* ── [7] Watchdog ── */
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms     = SYSTEM_WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    esp_task_wdt_reconfigure(&wdt_cfg);
    printf("┌───────────────────────────────────────┐\n");
    printf("│ [7] WDT %-2ds  Sistema operacional      │\n", SYSTEM_WDT_TIMEOUT_S);
    printf("╘═══════════════════════════════════════╛\n");
    printf("\n");

    xTaskCreatePinnedToCore(_monitor_task, "monitor_task",
                            3072, NULL, 7, NULL, 1);
}
