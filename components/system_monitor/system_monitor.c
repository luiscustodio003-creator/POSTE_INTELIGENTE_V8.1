/* ============================================================
   SUPERVISOR DO SISTEMA — INICIALIZAÇÃO E MONITORIZAÇÃO
   @file      system_monitor.c
   @version   5.0  |  2026-04-29
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Supervisor central. Inicializa todos os módulos em ordem
   correcta e segura, distribui tasks pelos cores, alimenta
   o hardware watchdog, monitoriza heartbeats e gere conectividade.

   ORDEM DE INICIALIZAÇÃO:
   ─────────────────────────
     [1] post_config_init()         — NVS: ID, nome, posição do poste
     [2] dali_init()                — Hardware DALI (brilho mínimo)
         radar_init(UART)           — Hardware HLK-LD2450
     [3] display_manager_init()     — LVGL + ST7789 (mostra estado imediato)
     [4] state_machine_init()       — FSM em estado IDLE
     [5] wifi_manager_init()        — STA + IP estático (UDP iniciado ao obter IP)
     [6] Tasks (por core e prioridade)
     [7] WDT hardware configurado

   DISTRIBUIÇÃO DUAL-CORE:
   ─────────────────────────
     Core 0 (PRO_CPU):
       udp_task      Prio 5  4096B  ~10ms   ← tráfego de rede
       radar_task    Prio 5  4096B  100ms   ← leitura UART HLK-LD2450
       (WiFi stack   Prio 22-23 — gerido pelo ESP-IDF)

     Core 1 (APP_CPU):
       monitor_task  Prio 7  3072B  200ms   ← alimenta WDT
       fsm_task      Prio 6  6144B  100ms   ← controlo principal
       display_task  Prio 4  8192B   20ms   ← LVGL 50Hz

   HARDWARE WATCHDOG:
   ──────────────────
     Apenas monitor_task registada no WDT.
     Timeout: SYSTEM_WDT_TIMEOUT_S → panic + reboot.
     monitor_task verifica heartbeats dos outros módulos
     e regista aviso LOGW se algum exceder o timeout.

   MUDANÇAS v4 → v5:
   ──────────────────
     - REMOVIDO: bloco #else USE_RADAR com radar_init(SIMULATED)
     - REMOVIDO: log "Radar SIMULADO"
     - SIMPLIFICADO: [2] inicializa sempre radar_init(RADAR_MODE_UART)
     - MANTIDO: toda a lógica de monitorização e WDT
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

/* ── Timestamps dos últimos heartbeats por módulo ─────────── */
static uint64_t s_last_hb[MOD_COUNT] = {0};

/* Timeouts de heartbeat por módulo (em ms) */
#define HB_TIMEOUT_FSM      500
#define HB_TIMEOUT_RADAR    500
#define HB_TIMEOUT_DISPLAY  200
#define HB_TIMEOUT_UDP      100

static const uint32_t s_hb_timeout[MOD_COUNT] = {
    HB_TIMEOUT_FSM,
    HB_TIMEOUT_RADAR,
    HB_TIMEOUT_DISPLAY,
    HB_TIMEOUT_UDP,
};

static const char *s_mod_nome[MOD_COUNT] = {
    "FSM", "RADAR", "DISPLAY", "UDP"
};


/* ============================================================
   system_monitor_heartbeat
   ─────────────────────────
   @brief Regista timestamp do último heartbeat de um módulo.
          Thread-safe: escrita atómica de uint64_t em Xtensa LX6.
============================================================ */
void system_monitor_heartbeat(monitor_module_t mod)
{
    if (mod < MOD_COUNT) {
        s_last_hb[mod] = (uint64_t)(esp_timer_get_time() / 1000ULL);
    }
}


/* ============================================================
   _monitor_task — Core 1, Prioridade 7, Stack 3072B, 200ms
   ──────────────────────────────────────────────────────────
   Verifica heartbeats, alimenta WDT, actualiza display de rede.
============================================================ */
static void _monitor_task(void *arg)
{
    ESP_LOGI(TAG, "monitor_task | Core %d | Prio 7 | 200ms", xPortGetCoreID());

    /* Regista esta task no hardware watchdog */
    esp_task_wdt_add(NULL);

    /* Inicializa timestamps para evitar falsos alarmes no arranque */
    uint64_t agora = (uint64_t)(esp_timer_get_time() / 1000ULL);
    for (int i = 0; i < MOD_COUNT; i++) {
        s_last_hb[i] = agora;
    }

    while (1) {

        /* ── Alimenta o hardware watchdog ─────────────────────── */
        esp_task_wdt_reset();

        agora = (uint64_t)(esp_timer_get_time() / 1000ULL);

        /* ── Verifica heartbeats de cada módulo ───────────────── */
        for (int i = 0; i < MOD_COUNT; i++) {
            uint64_t elapsed = agora - s_last_hb[i];
            if (elapsed > (uint64_t)s_hb_timeout[i]) {
                ESP_LOGW(TAG, "Sem heartbeat: %s (%llums)",
                         s_mod_nome[i], elapsed);
            }
        }

        /* ── Actualiza display com estado da rede ─────────────── */
        char neb_esq[20] = {0};
        char neb_dir[20] = {0};
        udp_manager_get_neighbors(neb_esq, neb_dir);

        bool wifi_ok  = comm_status_ok();
        bool left_ok  = (neb_esq[0] != '\0');
        bool right_ok = (neb_dir[0] != '\0');

        display_manager_set_hw_status(
            wifi_ok ? "OK" : "OFFLINE",
            state_machine_radar_ok() ? "OK" : "FALHA",
            wifi_ok,
            dali_get_brightness()
        );
        display_manager_set_neighbors(neb_esq, neb_dir, left_ok, right_ok);
        display_manager_set_traffic(
            state_machine_get_T(),
            state_machine_get_Tc()
        );
        display_manager_set_speed((int)state_machine_get_last_speed());

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}


/* ============================================================
   _on_wifi_got_ip
   ────────────────
   @brief Callback invocado pelo wifi_manager quando o IP é atribuído.
          Inicia o udp_manager — requer IP válido para bind do socket.
============================================================ */
static void _on_wifi_got_ip(void)
{
    ESP_LOGI(TAG, "WiFi: IP obtido — a iniciar UDP");
    if (udp_manager_init()) {
        udp_manager_task_start();
        ESP_LOGI(TAG, "UDP iniciado");
    } else {
        ESP_LOGE(TAG, "Falha ao iniciar UDP");
    }
}


/* ============================================================
   system_monitor_start
   ─────────────────────
   @brief Ponto de entrada do sistema. Chamado por app_main().
          Inicializa módulos, cria tasks, configura WDT.
          Não retorna — o monitor_task corre indefinidamente.
============================================================ */
void system_monitor_start(void)
{
    ESP_LOGI(TAG, "╔═══════════════════════════════════╗");
    ESP_LOGI(TAG, "║  Poste Inteligente v8 | %-10s║", POSTE_NAME);
    ESP_LOGI(TAG, "╚═══════════════════════════════════╝");

    /* [1] Configuração persistente NVS */
    post_config_init();
    ESP_LOGI(TAG, "[1] ID=%d  %s  pos=%d",
             post_get_id(), post_get_name(), POST_POSITION);

    /* [2] Hardware: DALI + Radar HLK-LD2450 */
    dali_init();
    dali_set_brightness(LIGHT_MIN);

    radar_init(RADAR_MODE_UART);
    int baud = radar_auto_detect_baud();
    ESP_LOGI(TAG, "[2] Radar UART | baud=%d", baud);
    radar_diagnostic();
    ESP_LOGI(TAG, "[2] DALI OK | brilho mínimo %d%%", LIGHT_MIN);

    /* [3] Display LVGL — mostra estado desde o primeiro frame */
    display_manager_init();
    ESP_LOGI(TAG, "[3] Display LVGL OK");

    /* [4] FSM — estado IDLE inicial */
    state_machine_init();
    ESP_LOGI(TAG, "[4] FSM OK — estado IDLE");

    /* [5] Wi-Fi STA — UDP iniciado pelo callback ao obter IP */
    wifi_manager_set_ip_callback(_on_wifi_got_ip);
    wifi_manager_init();
    ESP_LOGI(TAG, "[5] Wi-Fi STA iniciado");

    /* [6] Tasks — distribuição dual-core
       ORDEM OBRIGATÓRIA:
         state_machine_task_start() primeiro — inicializa tracking_manager
         radar_manager_task_start() depois  — começa a enviar frames ao tracking */
    state_machine_task_start();     /* Core 1, Prio 6, 6144B */
    radar_manager_task_start();     /* Core 0, Prio 5, 4096B */
    display_manager_task_start();   /* Core 1, Prio 4, 8192B */
    ESP_LOGI(TAG, "[6] Tasks criadas e fixadas nos cores");

    /* [7] Hardware watchdog */
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms     = SYSTEM_WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    esp_task_wdt_reconfigure(&wdt_cfg);
    ESP_LOGI(TAG, "[7] WDT HW: %ds | Sistema operacional",
             SYSTEM_WDT_TIMEOUT_S);

    /* Monitor em loop — fixa no Core 1, Prio 7 */
    xTaskCreatePinnedToCore(
        _monitor_task,
        "monitor_task",
        3072,
        NULL,
        7,
        NULL,
        1   /* Core 1 */
    );

    /* app_main termina — stack libertada pelo FreeRTOS */
}
