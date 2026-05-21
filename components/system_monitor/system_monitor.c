/* ============================================================
   MÓDULO     : system_monitor
   FICHEIRO   : system_monitor.c — Supervisor de módulos, watchdog e diagnóstico
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)
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
#include "web_manager.h"
#include "web_data_provider.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_netif_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <time.h>

static const char *TAG = "SYS_MON";

static uint64_t s_hb_ms[MOD_COUNT] = {0};
static bool     s_comm_ok          = false;

/* timestamps do supervisor de estados (0 = inactivo) */
static uint64_t s_sup_autonomo_ms = 0;
static uint64_t s_sup_safe_ms     = 0;
static uint64_t s_sup_wifi_ms     = 0;

static const char    *s_nome[MOD_COUNT] = { "FSM","RADAR","DISPLAY","UDP" };
static const uint64_t s_timeout_ms[MOD_COUNT] = {
    MOD_FSM_TIMEOUT_MS,
    MOD_RADAR_TIMEOUT_MS,
    MOD_DISPLAY_TIMEOUT_MS,
    MOD_UDP_TIMEOUT_MS,
};

void system_monitor_heartbeat(monitor_module_t mod)
{
    if (mod < MOD_COUNT)
        s_hb_ms[mod] = (uint64_t)(esp_timer_get_time() / 1000ULL);
}

/* ── _supervisao ─────────────────────────────────────────────
   Supervisor passivo: emite alertas e tenta re-init comm quando seguro.
   NUNCA altera estado da FSM. Nunca actua com tráfego em curso. */
static void _supervisao(uint64_t agora)
{
    system_state_t estado = state_machine_get_state();

    /* Guarda de tráfego — não actuar com veículos em curso */
    bool em_trafego = (estado == STATE_LIGHT_ON  ||
                       estado == STATE_OBSTACULO  ||
                       state_machine_get_T()  > 0 ||
                       state_machine_get_Tc() > 0);
    if (em_trafego) {
        s_sup_autonomo_ms = 0;
        s_sup_safe_ms     = 0;
        s_sup_wifi_ms     = 0;
        return;
    }

    bool wifi_on      = wifi_manager_is_connected();
    bool wifi_enabled = wifi_manager_is_enabled();

    /* 1. AUTONOMO com WiFi ligado há muito tempo
       Pode indicar falha de socket UDP sem que o FSM saiba.
       Acção: tenta re-init comm (idempotente se já OK). */
    if (estado == STATE_AUTONOMO && wifi_on && wifi_enabled) {
        if (s_sup_autonomo_ms == 0) s_sup_autonomo_ms = agora;
        if ((agora - s_sup_autonomo_ms) > SUP_AUTONOMO_MS) {
            ESP_LOGW(TAG, "[SUP] AUTONOMO há %llus com WiFi OK — re-init comm",
                     (unsigned long long)((agora - s_sup_autonomo_ms) / 1000ULL));
            if (!s_comm_ok)
                s_comm_ok = comm_init();
            s_sup_autonomo_ms = agora;  /* reset: novo ciclo de 30s */
        }
    } else {
        s_sup_autonomo_ms = 0;
    }

    /* 2. SAFE_MODE prolongado — alerta de radar em falha longa
       Sem acção: radar é hardware, só técnico pode resolver. */
    if (estado == STATE_SAFE_MODE) {
        if (s_sup_safe_ms == 0) s_sup_safe_ms = agora;
        if ((agora - s_sup_safe_ms) > SUP_SAFE_MS) {
            ESP_LOGE(TAG, "[SUP] SAFE_MODE há %llus — radar em falha prolongada",
                     (unsigned long long)((agora - s_sup_safe_ms) / 1000ULL));
            s_sup_safe_ms = agora;  /* reset: repete a cada 60s */
        }
    } else {
        s_sup_safe_ms = 0;
    }

    /* 3. WiFi desligado sem ser SAFE_MODE — alerta periódico
       A reconexão automática (WIFI_RECONNECT_MS) já está activa no wifi_manager.
       Apenas registamos para diagnóstico. */
    if (!wifi_on && wifi_enabled) {
        if (s_sup_wifi_ms == 0) s_sup_wifi_ms = agora;
        if ((agora - s_sup_wifi_ms) > SUP_WIFI_MS) {
            ESP_LOGW(TAG, "[SUP] WiFi offline há %llus — reconexão automática activa",
                     (unsigned long long)((agora - s_sup_wifi_ms) / 1000ULL));
            s_sup_wifi_ms = agora;
        }
    } else {
        s_sup_wifi_ms = 0;
    }
}


/* ── _watermark_check — stack headroom a cada ~30s ───────────
   Emite WARN se < 256 words livres, LOGE se < 64 words.
   xTaskGetHandle exige INCLUDE_xTaskGetHandle=1 (activado por defeito no ESP-IDF v5). */
static void _watermark_check(void)
{
    static const char *const task_names[] = {
        "fsm_task", "radar_task", "display_task", "udp_task", "monitor_task"
    };
    for (int i = 0; i < (int)(sizeof(task_names) / sizeof(task_names[0])); i++) {
        TaskHandle_t h = xTaskGetHandle(task_names[i]);
        if (!h) continue;
        UBaseType_t wm = uxTaskGetStackHighWaterMark(h);
        if (wm < 64)
            ESP_LOGE(TAG, "[STK] %s: %u words — STACK OVERFLOW iminente!",
                     task_names[i], (unsigned)wm);
        else if (wm < 256)
            ESP_LOGW(TAG, "[STK] %s: %u words livres — margem baixa",
                     task_names[i], (unsigned)wm);
        else
            ESP_LOGD(TAG, "[STK] %s: %u words livres", task_names[i], (unsigned)wm);
    }
}

static void _monitor_task(void *arg)
{
    ESP_LOGI(TAG, "monitor_task | Core %d | Prio 7", xPortGetCoreID());
    esp_task_wdt_add(NULL);

    uint64_t agora = (uint64_t)(esp_timer_get_time() / 1000ULL);
    for (int i = 0; i < MOD_COUNT; i++) s_hb_ms[i] = agora;

    uint32_t watermark_counter = 0;

    while (1) {
        esp_task_wdt_reset();

        agora = (uint64_t)(esp_timer_get_time() / 1000ULL);
        for (int i = 0; i < MOD_COUNT; i++) {
            uint64_t delta = agora - s_hb_ms[i];
            if (delta > (uint64_t)s_timeout_ms[i] * MOD_HEARTBEAT_CRITICAL_MULT)
                ESP_LOGE(TAG, "[WDT] %s CRÍTICO sem heartbeat %llums",
                         s_nome[i], (unsigned long long)delta);
            else if (delta > (uint64_t)s_timeout_ms[i])
                ESP_LOGW(TAG, "[WDT] %s sem heartbeat %llums",
                         s_nome[i], (unsigned long long)delta);
        }

        if (++watermark_counter >= 150) {   /* 150 × 200ms = 30s */
            watermark_counter = 0;
            _watermark_check();
        }

        wifi_manager_tick();
        _supervisao(agora);

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
    wifi_manager_init_auto();
    printf("┌───────────────────────────────────────┐\n");
    printf("│ [5] WiFi  STA  SSID: %-16s │\n", WIFI_SSID);
    printf("└───────────────────────────────────────┘\n");

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
       [6] WEB MANAGER - INTERFACE DE MONITORIZAÇÃO
       ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    
    /* Esperar WiFi obter IP (obrigatório antes do servidor HTTP) */
    printf("┌───────────────────────────────────────┐\n");
    printf("│ [6] Web  A aguardar IP do WiFi...     │\n");
    
    int timeout = 0;
    while (!wifi_manager_is_connected() && timeout < 100) {
        vTaskDelay(pdMS_TO_TICKS(100));
        timeout++;
    }
    
    if (wifi_manager_is_connected()) {
        /* SNTP — sincroniza relógio real (necessário para estatísticas nocturnas) */
        setenv("TZ", POSTE_TIMEZONE, 1);
        tzset();
        esp_sntp_config_t sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
        esp_netif_sntp_init(&sntp_cfg);
        ESP_LOGI(TAG, "SNTP iniciado | TZ=%s", POSTE_TIMEZONE);

        /* Inicializar agregador de dados */
        web_data_provider_init();
        
        /* Inicializar servidor HTTP */
        esp_err_t ret = web_manager_init();
        
        if (ret == ESP_OK) {
            /* Obter IP para mostrar na box */
            const char* ip = wifi_manager_get_ip();
            
            printf("│     ✓ Servidor HTTP activo           │\n");
            printf("│     IP: %-29s │\n", ip);
            printf("│     Porta: 80                         │\n");
            printf("│                                       │\n");
            printf("│     Acesso via browser:               │\n");
            printf("│     http://%-26s │\n", ip);
            printf("└───────────────────────────────────────┘\n");
        } else {
            printf("│     ✗ ERRO ao iniciar servidor web   │\n");
            printf("│     Código: %-25s │\n", esp_err_to_name(ret));
            printf("└───────────────────────────────────────┘\n");
        }
    } else {
        printf("│     ✗ TIMEOUT - WiFi sem IP           │\n");
        printf("│     Servidor web NÃO iniciado         │\n");
        printf("└───────────────────────────────────────┘\n");
    }

    /* ── [7] Tasks ── */
    state_machine_task_start();
    radar_manager_task_start();
    display_manager_task_start();
    udp_manager_task_start();
    printf("┌───────────────────────────────────────┐\n");
    printf("│ [7] Tasks criadas                     │\n");
    printf("│     FSM    Core1  Prio6  100ms        │\n");
    printf("│     RADAR  Core0  Prio6  100ms        │\n");
    printf("│     DISP   Core0  Prio4   20ms        │\n");
    printf("│     UDP    Core0  Prio5   10ms        │\n");
    printf("└───────────────────────────────────────┘\n");

    /* ── [8] Watchdog ── */
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms     = SYSTEM_WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    esp_task_wdt_reconfigure(&wdt_cfg);
    printf("┌───────────────────────────────────────┐\n");
    printf("│ [8] WDT %-2ds  Sistema operacional      │\n", SYSTEM_WDT_TIMEOUT_S);
    printf("╘═══════════════════════════════════════╛\n");
    printf("\n");

    xTaskCreatePinnedToCore(_monitor_task, "monitor_task",3072, NULL, 7, NULL, 1);
}