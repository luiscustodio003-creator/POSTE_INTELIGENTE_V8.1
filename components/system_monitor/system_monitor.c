#include <inttypes.h>
/* ============================================================
   SYSTEM MONITOR — IMPLEMENTAÇÃO
   @file      system_monitor.c
   @version   1.0  |  2026-04-07
   Projecto  : Poste Inteligente v8
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)
============================================================ */
#include "system_monitor.h"
#include "state_machine.h"
#include "radar_manager.h"
#include "display_manager.h"
#include "udp_manager.h"
#include "comm_manager.h"
#include "wifi_manager.h"
#include "dali_manager.h"
#include "post_config.h"
#include "system_config.h"
#include "wifi_manager.h"  
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "SYS_MON";

/* ── Timestamps de heartbeat por módulo ───────────────────── */
static volatile uint64_t s_hb_ms[MOD_COUNT] = {0};

static const uint32_t s_timeout_ms[MOD_COUNT] = {
    [MOD_FSM]     = MOD_FSM_TIMEOUT_MS,
    [MOD_RADAR]   = MOD_RADAR_TIMEOUT_MS,
    [MOD_DISPLAY] = MOD_DISPLAY_TIMEOUT_MS,
    [MOD_UDP]     = MOD_UDP_TIMEOUT_MS,
};

static const char *s_nome[MOD_COUNT] = {
    [MOD_FSM]     = "FSM",
    [MOD_RADAR]   = "RADAR",
    [MOD_DISPLAY] = "DISPLAY",
    [MOD_UDP]     = "UDP",
};

/* Flag: protocolo UDP iniciado (após Wi-Fi disponível) */
static volatile bool s_comm_ok = false;

/* ============================================================
   system_monitor_heartbeat
   Regista timestamp de actividade. Thread-safe:
   escrita de uint64_t alinhada em Xtensa LX6 é atómica.
============================================================ */
void system_monitor_heartbeat(monitor_module_t mod)
{
    if (mod < MOD_COUNT)
        s_hb_ms[mod] = (uint64_t)(esp_timer_get_time() / 1000ULL);
}

/* ============================================================
   system_monitor_is_alive
============================================================ */
bool system_monitor_is_alive(monitor_module_t mod)
{
    if (mod >= MOD_COUNT) return false;
    uint64_t agora = (uint64_t)(esp_timer_get_time() / 1000ULL);
    return (agora - s_hb_ms[mod]) < (uint64_t)s_timeout_ms[mod];
}

/* ============================================================
   _monitor_task — Core 1, Prioridade 7, Período 200ms
   A task de maior prioridade da aplicação.
   Garante que o watchdog é sempre alimentado.
============================================================ */
static void _monitor_task(void *arg)
{
    ESP_LOGI(TAG, "monitor_task | Core %d | Prio 7 | WDT %ds",
             xPortGetCoreID(), SYSTEM_WDT_TIMEOUT_S);

    /* Regista esta task no hardware watchdog */
    esp_task_wdt_add(NULL);

    /* Inicializa timestamps para evitar falso alarme no arranque */
    uint64_t agora = (uint64_t)(esp_timer_get_time() / 1000ULL);
    for (int i = 0; i < MOD_COUNT; i++) s_hb_ms[i] = agora;

    while (1) {
        /* ── 1. Alimenta hardware watchdog ───────────────── */
        esp_task_wdt_reset();

        /* ── 2. Verifica heartbeats de todos os módulos ──── */
        agora = (uint64_t)(esp_timer_get_time() / 1000ULL);
        for (int i = 0; i < MOD_COUNT; i++) {
            uint64_t delta = agora - s_hb_ms[i];
            if (delta > (uint64_t)s_timeout_ms[i]) {
                ESP_LOGW(TAG, "Módulo %s sem heartbeat há %llums",
                    s_nome[i], (unsigned long long)delta);
            }
        }

        /* DEPOIS: */
        if (wifi_manager_is_connected()) {
            if (!s_comm_ok) {
                if (comm_init()) {
                        s_comm_ok = true;
                        ESP_LOGI(TAG, "Protocolo UDP activo — vizinhos a descobrir");
                    }
                }
            } else {
                /* WiFi perdido — reseta estado para reiniciar quando voltar */
                if (s_comm_ok) {
                    s_comm_ok = false;
                    ESP_LOGW(TAG, "WiFi perdido — comm resetado");
                    display_manager_set_wifi(false, NULL);
                }
            }

        /* ── 4. Actualiza display com estado de rede ────── */
        if (s_comm_ok) {
            char nL[MAX_IP_LEN], nR[MAX_IP_LEN];
            udp_manager_get_neighbors(nL, nR);
            display_manager_set_neighbors(nL, nR,
                comm_left_online(), comm_right_online());
            display_manager_set_wifi(true, wifi_manager_get_ip());
        } else {
            display_manager_set_wifi(false, "Desligado");
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ============================================================
   system_monitor_start
   Ponto de entrada único. Chamado por app_main().
   Inicializa módulos, cria tasks, configura WDT. Não retorna.

   Ordem de inicialização:
   [1] Configuração NVS (ID, nome, posição)
   [2] Hardware: DALI + Radar (independentes da rede)
   [3] Display LVGL (mostra estado desde o primeiro frame)
   [4] FSM (estado IDLE inicial)
   [5] Wi-Fi STA (UDP iniciado pelo monitor quando IP disponível)
   [6] Tasks por core e prioridade
   [7] Hardware watchdog
   [loop] monitor_task — não retorna
============================================================ */
void system_monitor_start(void)
{
    ESP_LOGI(TAG, "╔═══════════════════════════════════╗");
    ESP_LOGI(TAG, "║  Poste Inteligente v8 | %-10s║", POSTE_NAME);
    ESP_LOGI(TAG, "╚═══════════════════════════════════╝");

    /* [1] Configuração persistente */
    post_config_init();
    ESP_LOGI(TAG, "[1] ID=%d  %s  pos=%d",post_get_id(), post_get_name(), POST_POSITION);

    /* [2] Hardware: DALI + Radar */
    dali_init();
    dali_set_brightness(LIGHT_MIN);

#if USE_RADAR
    radar_init(RADAR_MODE_UART);
    int baud = radar_auto_detect_baud();
    ESP_LOGI(TAG, "[2] Radar UART | baud=%d", baud);
    radar_diagnostic();
#else
    radar_init(RADAR_MODE_SIMULATED);
    ESP_LOGI(TAG, "[2] Radar SIMULADO");
#endif
    ESP_LOGI(TAG, "[2] DALI OK | brilho mínimo %d%%", LIGHT_MIN);

    /* [3] Display LVGL */
    display_manager_init();
    ESP_LOGI(TAG, "[3] Display LVGL OK");

    /* [4] FSM */
    state_machine_init();
    ESP_LOGI(TAG, "[4] FSM OK — estado IDLE");

    /* [5] Wi-Fi (UDP iniciado pelo monitor ao obter IP) */
    wifi_manager_init();
    ESP_LOGI(TAG, "[5] Wi-Fi STA iniciado");

    /* [6] Tasks — distribuição dual-core */
    state_machine_task_start();     /* Core 1, Prio 6, 6144B */
    radar_manager_task_start();     /* Core 1, Prio 5, 4096B */
    display_manager_task_start();   /* Core 1, Prio 4, 8192B */
    udp_manager_task_start();       /* Core 0, Prio 5, 4096B */
    ESP_LOGI(TAG, "[6] Tasks criadas e fixadas nos cores");

    /* [7] Hardware watchdog */
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms     = SYSTEM_WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    esp_task_wdt_reconfigure(&wdt_cfg);
    ESP_LOGI(TAG, "[7] WDT HW: %ds | Sistema operacional", SYSTEM_WDT_TIMEOUT_S);

    /* Monitor em loop — fixa no Core 1, Prio 7 */
    xTaskCreatePinnedToCore(_monitor_task, "monitor_task",
                            3072, NULL, 7, NULL, 1);

    /* app_main termina — stack libertada pelo FreeRTOS */
}

