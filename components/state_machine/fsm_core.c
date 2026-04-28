/* ============================================================
   MÓDULO     : fsm_core
   FICHEIRO   : fsm_core.c — Núcleo da FSM
   VERSÃO     : 1.0  |  2026-04-26
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   - Declaração e inicialização de todas as variáveis de estado
   - Utilitários internos partilhados (agora_ms, agendar_apagar)
   - Monitorização da saúde do radar com debounce
   - Getters públicos do estado da FSM
   - Ponto de entrada state_machine_update() que delega
     nos sub-módulos fsm_timer, fsm_network

   DEPENDÊNCIAS:
   ─────────────
   fsm_core.h, fsm_timer.h, fsm_network.h
   comm_manager.h, dali_manager.h, system_config.h
============================================================ */

#include "fsm_core.h"
#include "fsm_timer.h"
#include "fsm_network.h"
#include "fsm_sim.h"
#include "comm_manager.h"
#include "dali_manager.h"
#include "system_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "FSM";

/* ============================================================
   VARIÁVEIS DE ESTADO — definições
   Declaradas extern em fsm_core.h para acesso pelos sub-módulos.
============================================================ */
system_state_t g_fsm_state          = STATE_IDLE;
int            g_fsm_T              = 0;
int            g_fsm_Tc             = 0;
float          g_fsm_last_speed     = 0.0f;
bool           g_fsm_apagar_pend    = false;
bool           g_fsm_radar_ok       = true;
int            g_fsm_radar_fail_cnt = 0;
int            g_fsm_radar_ok_cnt   = 0;
bool           g_fsm_right_online   = true;
bool           g_fsm_era_autonomo   = false;

uint64_t g_fsm_last_detect_ms    = 0;
uint64_t g_fsm_left_offline_ms   = 0;
uint64_t g_fsm_tc_timeout_ms     = 0;
bool     g_fsm_left_was_offline  = false;
uint64_t g_fsm_acender_em_ms     = 0;
uint64_t g_fsm_master_claim_ms   = 0;
uint64_t g_fsm_sem_vizinho_ms    = 0;
uint64_t g_fsm_obstaculo_last_ms = 0;


/* ============================================================
   UTILITÁRIOS INTERNOS
============================================================ */

uint64_t fsm_agora_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

void fsm_agendar_apagar(void)
{
    g_fsm_apagar_pend    = true;
    g_fsm_last_detect_ms = fsm_agora_ms();
}

/* Monitoriza saúde do radar com debounce bidirecional.
   Exige RADAR_FAIL_COUNT ciclos para declarar falha e
   RADAR_OK_COUNT frames para declarar recuperação. */
void fsm_verificar_radar(bool teve_frame, bool comm_ok)
{
    if (teve_frame) {
        g_fsm_radar_fail_cnt = 0;
        g_fsm_radar_ok_cnt++;
        if (!g_fsm_radar_ok && g_fsm_radar_ok_cnt >= RADAR_OK_COUNT) {
            g_fsm_radar_ok = true;
            ESP_LOGI(TAG, "Radar recuperado após %d frames", RADAR_OK_COUNT);
            if (g_fsm_state == STATE_SAFE_MODE)
                g_fsm_state = STATE_IDLE;
        }
    } else {
        g_fsm_radar_ok_cnt = 0;
        g_fsm_radar_fail_cnt++;
        if (g_fsm_radar_ok && g_fsm_radar_fail_cnt >= RADAR_FAIL_COUNT) {
            g_fsm_radar_ok = false;
            ESP_LOGW(TAG, "Radar FAIL após %d ciclos", RADAR_FAIL_COUNT);
        }
    }
    (void)comm_ok;
}


/* ============================================================
   state_machine_init — inicialização de todas as variáveis
============================================================ */
void state_machine_init(void)
{
    g_fsm_state             = STATE_IDLE;
    g_fsm_T                 = 0;
    g_fsm_Tc                = 0;
    g_fsm_last_speed        = 0.0f;
    g_fsm_apagar_pend       = false;
    g_fsm_radar_ok          = true;
    g_fsm_radar_fail_cnt    = 0;
    g_fsm_radar_ok_cnt      = 0;
    g_fsm_right_online      = true;
    g_fsm_era_autonomo      = false;
    g_fsm_acender_em_ms     = 0;
    g_fsm_master_claim_ms   = 0;
    g_fsm_sem_vizinho_ms    = 0;
    g_fsm_obstaculo_last_ms = 0;
    g_fsm_left_was_offline  = false;
    g_fsm_left_offline_ms   = 0;
    g_fsm_tc_timeout_ms     = 0;
    g_fsm_last_detect_ms    = 0;

#if USE_RADAR == 0
    fsm_sim_init();
#endif

    ESP_LOGI(TAG, "FSM v1.0 inicializada — estado IDLE");
}


/* ============================================================
   state_machine_update — ciclo de manutenção a 100ms
   Delega nos sub-módulos por responsabilidade.
============================================================ */
void state_machine_update(bool comm_ok, bool is_master,
                          bool radar_teve_frame)
{
#if USE_RADAR == 0
    fsm_sim_update();
    radar_teve_frame = true;
#endif

    /* Passo 2: saúde do radar */
    fsm_verificar_radar(radar_teve_frame, comm_ok);

    /* Passos 3-5: gestão de vizinhos e T preso */
    fsm_network_vizinhos(comm_ok, is_master);

    /* Passos 6-9,12: timers, apagamento, obstáculo, Tc, MASTER_CLAIM */
    fsm_timer_update(comm_ok, is_master);

    /* Passo 10: papel MASTER */
    fsm_network_master(comm_ok, is_master);

    /* Passo 11: estados degradados SAFE_MODE / AUTONOMO */
    fsm_network_estados_degradados(comm_ok, is_master);
}


/* ============================================================
   GETTERS PÚBLICOS
============================================================ */
system_state_t state_machine_get_state(void)
{
    return g_fsm_state;
}

int state_machine_get_T(void)   { return g_fsm_T; }
int state_machine_get_Tc(void)  { return g_fsm_Tc; }

float state_machine_get_last_speed(void) { return g_fsm_last_speed; }
bool  state_machine_radar_ok(void)       { return g_fsm_radar_ok; }
bool  sm_is_obstaculo(void) { return g_fsm_state == STATE_OBSTACULO; }

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
