/* ============================================================
   MÓDULO     : fsm_core
   FICHEIRO   : fsm_core.c — Variáveis de estado globais e ciclo principal da FSM
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)
============================================================ */

#include "fsm_core.h"
#include "fsm_timer.h"
#include "fsm_network.h"
#include "comm_manager.h"
#include "system_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static const char *TAG = "FSM_CORE";

/* ── Variáveis de estado ──────────────────────────────────── */
system_state_t g_fsm_state          = STATE_IDLE;
int            g_fsm_T              = 0;
int            g_fsm_Tc             = 0;
int            g_fsm_enviados_dir   = 0;
float          g_fsm_last_speed     = 0.0f;
bool           g_fsm_apagar_pend    = false;
bool           g_fsm_radar_ok       = true;
int            g_fsm_radar_fail_cnt = 0;
int            g_fsm_radar_ok_cnt   = 0;
bool           g_fsm_right_online   = true;

uint64_t g_fsm_last_detect_ms    = 0;
uint64_t g_fsm_left_offline_ms   = 0;
bool     g_fsm_left_was_offline  = false;
uint64_t g_fsm_master_claim_ms   = 0;
uint64_t g_fsm_sem_vizinho_ms    = 0;
uint64_t g_fsm_obstaculo_last_ms = 0;

/* g_fsm_counters_mux: partilhado por fsm_events.c e fsm_timer.c.
   Protege ++ e -- sobre g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir entre Core 0/1. */
portMUX_TYPE g_fsm_counters_mux = portMUX_INITIALIZER_UNLOCKED;

/* g_fsm_acender_em_ms e g_fsm_tc_timeout_ms: escritos de Core 0 (UDP task) e
   lidos de Core 1 (FSM task). uint64_t não é atómico em Xtensa LX6 (32-bit). */
static portMUX_TYPE   s_acender_mux   = portMUX_INITIALIZER_UNLOCKED;
static uint64_t       s_acender_em_ms = 0;

static portMUX_TYPE   s_tc_mux         = portMUX_INITIALIZER_UNLOCKED;
static uint64_t       s_tc_timeout_ms  = 0;

static portMUX_TYPE   s_spd_fb_mux     = portMUX_INITIALIZER_UNLOCKED;
static uint64_t       s_spd_fb_ms      = 0;

uint64_t fsm_acender_em_ms_get(void)
{
    portENTER_CRITICAL(&s_acender_mux);
    uint64_t v = s_acender_em_ms;
    portEXIT_CRITICAL(&s_acender_mux);
    return v;
}

void fsm_acender_em_ms_set(uint64_t v)
{
    portENTER_CRITICAL(&s_acender_mux);
    s_acender_em_ms = v;
    portEXIT_CRITICAL(&s_acender_mux);
}

uint64_t fsm_tc_timeout_ms_get(void)
{
    portENTER_CRITICAL(&s_tc_mux);
    uint64_t v = s_tc_timeout_ms;
    portEXIT_CRITICAL(&s_tc_mux);
    return v;
}

void fsm_tc_timeout_ms_set(uint64_t v)
{
    portENTER_CRITICAL(&s_tc_mux);
    s_tc_timeout_ms = v;
    portEXIT_CRITICAL(&s_tc_mux);
}

uint64_t fsm_spd_fallback_ms_get(void)
{
    portENTER_CRITICAL(&s_spd_fb_mux);
    uint64_t v = s_spd_fb_ms;
    portEXIT_CRITICAL(&s_spd_fb_mux);
    return v;
}

void fsm_spd_fallback_ms_set(uint64_t v)
{
    portENTER_CRITICAL(&s_spd_fb_mux);
    s_spd_fb_ms = v;
    portEXIT_CRITICAL(&s_spd_fb_mux);
}

/* ID do último veículo que gerou TC_INC — evita duplicados por re-entrada. */
uint16_t g_fsm_tc_last_vehicle_id = 0;

bool g_fsm_acender_instantaneo = false;


/* ── Utilitários internos ─────────────────────────────────── */

uint64_t fsm_agora_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

void fsm_agendar_apagar(void)
{
    g_fsm_apagar_pend    = true;
    g_fsm_last_detect_ms = fsm_agora_ms();
    fsm_acender_em_ms_set(0);
}

void fsm_obstaculo_keepalive(void)
{
    if (g_fsm_state == STATE_OBSTACULO) {
        g_fsm_obstaculo_last_ms = fsm_agora_ms();
    }
}


/* ── fsm_verificar_radar ──────────────────────────────────────
   Debounce bidirecional: exige N frames consecutivos para mudar estado.
   teve_frame: true se tracking_manager recebeu frame válido neste ciclo.
   RADAR_FAIL_COUNT=80 ciclos × 100ms = 8s antes de SAFE MODE. */
void fsm_verificar_radar(bool teve_frame, bool comm_ok)
{
    (void)comm_ok;

    if (!teve_frame) {
        g_fsm_radar_ok_cnt = 0;
        g_fsm_radar_fail_cnt++;

        if (g_fsm_radar_fail_cnt >= RADAR_FAIL_COUNT && g_fsm_radar_ok) {
            g_fsm_radar_ok = false;
            ESP_LOGW(TAG, "[RADAR] %d ciclos sem frame — SAFE MODE activado.",
                     RADAR_FAIL_COUNT);
            /* Estado e WiFi geridos por fsm_network_estados_degradados no
               mesmo ciclo — separação de responsabilidades. */
        }

    } else {
        g_fsm_radar_fail_cnt = 0;

        if (!g_fsm_radar_ok) {
            g_fsm_radar_ok_cnt++;
            if (g_fsm_radar_ok_cnt >= RADAR_OK_COUNT) {
                g_fsm_radar_ok     = true;
                g_fsm_radar_ok_cnt = 0;
                ESP_LOGI(TAG, "[RADAR] %d frames OK consecutivos — saída de SAFE MODE.",
                         RADAR_OK_COUNT);
                /* Estado e WiFi geridos por fsm_network_estados_degradados no
                   mesmo ciclo — separação de responsabilidades. */
            }
        }
    }
}


/* ── state_machine_init ───────────────────────────────────── */
void state_machine_init(void)
{
    g_fsm_state                = STATE_IDLE;
    g_fsm_T                    = 0;
    g_fsm_Tc                   = 0;
    g_fsm_enviados_dir         = 0;
    g_fsm_tc_last_vehicle_id   = 0;
    g_fsm_last_speed           = 0.0f;
    g_fsm_apagar_pend          = false;
    g_fsm_radar_ok             = true;
    g_fsm_radar_fail_cnt       = 0;
    g_fsm_radar_ok_cnt         = 0;
    g_fsm_right_online         = true;
    fsm_acender_em_ms_set(0);
    fsm_tc_timeout_ms_set(0);
    fsm_spd_fallback_ms_set(0);
    g_fsm_acender_instantaneo  = false;
    g_fsm_master_claim_ms      = 0;
    g_fsm_sem_vizinho_ms       = 0;
    g_fsm_obstaculo_last_ms    = 0;
    g_fsm_left_was_offline     = false;
    g_fsm_left_offline_ms      = 0;
    g_fsm_last_detect_ms       = 0;

    ESP_LOGI(TAG, "FSM Produção inicializada — Modo Hardware Real");
}


/* ── state_machine_update — ciclo a 100ms ─────────────────── */
void state_machine_update(bool comm_ok, bool is_master, bool radar_teve_frame)
{
    fsm_verificar_radar(radar_teve_frame, comm_ok);
    fsm_network_vizinhos(comm_ok, is_master);
    fsm_timer_update(comm_ok, is_master);
    fsm_network_master(comm_ok, is_master);
    fsm_network_estados_degradados(comm_ok, is_master);
}


/* ── Getters públicos ─────────────────────────────────────── */
system_state_t state_machine_get_state(void) { return g_fsm_state; }
int   state_machine_get_T(void)         { return g_fsm_T; }
int   state_machine_get_Tc(void)        { return g_fsm_Tc; }
float state_machine_get_last_speed(void){ return g_fsm_last_speed; }
bool  state_machine_radar_ok(void)      { return g_fsm_radar_ok; }
bool  sm_is_obstaculo(void)             { return g_fsm_state == STATE_OBSTACULO; }

const char *state_machine_get_state_name(void)
{
    switch (g_fsm_state) {
        case STATE_IDLE:      return "IDLE";
        case STATE_LIGHT_ON:  return "LIGHT ON";
        case STATE_SAFE_MODE: return "SAFE MODE";
        case STATE_MASTER:    return "MASTER";
        case STATE_AUTONOMO:  return "AUTONOMO";
        case STATE_OBSTACULO: return "OBSTACULO";
        default:              return "---";
    }
}

uint8_t fsm_core_get_duty_cycle(void) {
    switch (g_fsm_state) {
        case STATE_IDLE:
        case STATE_MASTER:
        case STATE_AUTONOMO:
            return LIGHT_MIN;
        case STATE_LIGHT_ON:
        case STATE_OBSTACULO:
            return LIGHT_MAX;
        case STATE_SAFE_MODE:
            return LIGHT_SAFE_MODE;
        default:
            return LIGHT_MIN;
    }
}
