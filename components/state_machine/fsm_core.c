/* ============================================================
   MÓDULO     : fsm_core
   FICHEIRO   : fsm_core.c
   VERSÃO     : 2.0  |  2026-04-27
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno

   ALTERAÇÕES v1.0 → v2.0:
   ─────────────────────────
   1. fsm_verificar_radar(): quando radar muda de estado envia
      RADAR_FAIL ou RADAR_OK a ambos os vizinhos via
      comm_notify_radar_status(). Vizinhos reagem imediatamente:
        viz.esq → AUTONOMO (não pode confiar no PASSED de B)
        viz.dir → MASTER   (B não vai enviar TC_INC)
   2. Adicionadas variáveis g_fsm_left_radar_fail e
      g_fsm_right_radar_fail — flags de notificação recebida.
   3. state_machine_update(): parâmetro comm_ok removido de
      fsm_verificar_radar (radar é independente do WiFi).
   4. Sai de SAFE MODE → IDLE apenas quando radar recupera
      (não depende do WiFi para esta transição).

   DEPENDÊNCIAS:
   ─────────────
   fsm_core.h, fsm_network.h, fsm_timer.h, fsm_sim.h
   comm_manager.h, dali_manager.h, system_config.h
============================================================ */

#include "fsm_core.h"
#include "fsm_network.h"
#include "fsm_timer.h"
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
============================================================ */
system_state_t g_fsm_state           = STATE_IDLE;
int            g_fsm_T               = 0;
int            g_fsm_Tc              = 0;
float          g_fsm_last_speed      = 0.0f;
bool           g_fsm_apagar_pend     = false;
bool           g_fsm_radar_ok        = true;
int            g_fsm_radar_fail_cnt  = 0;
int            g_fsm_radar_ok_cnt    = 0;
bool           g_fsm_right_online    = true;
bool           g_fsm_era_autonomo    = false;
bool           g_fsm_left_radar_fail = false;
bool           g_fsm_right_radar_fail= false;

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

/* ============================================================
   fsm_verificar_radar
   ──────────────────────────────────────────────────────────
   Debounce bidirecional:
     FAIL : RADAR_FAIL_COUNT ciclos (100ms) sem frame → radar KO
            → SAFE MODE + envia RADAR_FAIL a viz.esq e viz.dir
     OK   : RADAR_OK_COUNT frames consecutivos → radar recuperado
            → IDLE + envia RADAR_OK a viz.esq e viz.dir

   Vizinhos reagem ao RADAR_FAIL/OK via callbacks UDP:
     on_radar_fail_received() em fsm_events.c
============================================================ */
void fsm_verificar_radar(bool teve_frame)
{
    if (teve_frame) {
        g_fsm_radar_fail_cnt = 0;
        g_fsm_radar_ok_cnt++;

        if (!g_fsm_radar_ok && g_fsm_radar_ok_cnt >= RADAR_OK_COUNT) {
            g_fsm_radar_ok = true;
            ESP_LOGI(TAG, "Radar RECUPERADO — envia RADAR_OK a vizinhos");

            /* Avisa ambos os vizinhos que o radar recuperou */
            comm_notify_radar_status(true);

            /* Sai de SAFE MODE → IDLE */
            if (g_fsm_state == STATE_SAFE_MODE)
                g_fsm_state = STATE_IDLE;
        }
    } else {
        g_fsm_radar_ok_cnt = 0;
        g_fsm_radar_fail_cnt++;

        if (g_fsm_radar_ok && g_fsm_radar_fail_cnt >= RADAR_FAIL_COUNT) {
            g_fsm_radar_ok = false;
            ESP_LOGW(TAG, "Radar FAIL — envia RADAR_FAIL a vizinhos");

            /* Avisa ambos os vizinhos que o radar falhou.
               viz.esq → AUTONOMO  (não pode confiar em PASSED de B)
               viz.dir → MASTER    (B não vai enviar TC_INC) */
            comm_notify_radar_status(false);

            /* Entra em SAFE MODE — cego, luz 50% */
            if (g_fsm_state != STATE_OBSTACULO) {
                g_fsm_era_autonomo = false;
                g_fsm_state = STATE_SAFE_MODE;
                dali_set_brightness(LIGHT_SAFE_MODE);
            }
        }
    }
}


/* ============================================================
   state_machine_init
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
    g_fsm_left_radar_fail   = false;
    g_fsm_right_radar_fail  = false;
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

    ESP_LOGI(TAG, "FSM v2.0 inicializada — estado IDLE");
}


/* ============================================================
   state_machine_update — ciclo de manutenção a 100ms
   ──────────────────────────────────────────────────────────
   Ordem de execução:
     1. Simulador (USE_RADAR=0)
     2. Saúde do radar (debounce + RADAR_FAIL/OK UDP)
     3. Vizinhos (online/offline)
     4. Timers (ETA, apagamento, Tc timeout, MASTER_CLAIM)
     5. Papel MASTER
     6. Estados degradados (AUTONOMO, SAFE_MODE)
============================================================ */
void state_machine_update(bool comm_ok, bool is_master,
                          bool radar_teve_frame)
{
    /* 1. Simulador — substitui radar físico em bancada */
#if USE_RADAR == 0
    fsm_sim_update();
    radar_teve_frame = true;
#endif

    /* 2. Saúde do radar — debounce + notifica vizinhos */
    fsm_verificar_radar(radar_teve_frame);

    /* 3. Vizinhos — detecta online/offline viz.esq e viz.dir */
    fsm_network_vizinhos(comm_ok, is_master);

    /* 4. Timers — ETA pré-acendimento, apagamento, Tc, MASTER_CLAIM */
    fsm_timer_update(comm_ok, is_master);

    /* 5. Papel MASTER — IDLE↔MASTER conforme topologia */
    fsm_network_master(comm_ok, is_master);

    /* 6. Estados degradados — AUTONOMO e SAFE_MODE */
    fsm_network_estados_degradados(comm_ok, is_master);
}


/* ============================================================
   GETTERS PÚBLICOS
============================================================ */
system_state_t state_machine_get_state(void)     { return g_fsm_state; }
int            state_machine_get_T(void)          { return g_fsm_T; }
int            state_machine_get_Tc(void)         { return g_fsm_Tc; }
float          state_machine_get_last_speed(void) { return g_fsm_last_speed; }
bool           state_machine_radar_ok(void)       { return g_fsm_radar_ok; }
bool           sm_is_obstaculo(void) { return g_fsm_state == STATE_OBSTACULO; }

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
