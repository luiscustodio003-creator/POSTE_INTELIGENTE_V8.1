/* ============================================================
   MÓDULO     : fsm_core
   FICHEIRO   : fsm_core.c — Núcleo da FSM (Versão Produção)
   VERSÃO     : 2.6  |  2026-05-01
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   - Declaração e inicialização de todas as variáveis de estado.
   - Monitorização da saúde do radar real com debounce.
   - Ponto de entrada state_machine_update() (Sem Simulação).

   CORRECÇÕES v2.5 → v2.6:
   ─────────────────────────
   - fsm_verificar_radar() usa radar_is_connected() como fonte
     de verdade para distinguir "sem alvos" de "sensor morto":
       teve_frame=false + UART activa  → sem alvos, normal → OK
       teve_frame=false + UART morta   → sensor desligado  → SAFE_MODE
       teve_frame=true                 → sensor com alvos  → OK
============================================================ */

#include "fsm_core.h"
#include "fsm_timer.h"
#include "fsm_network.h"
#include "comm_manager.h"
#include "radar_manager.h"
#include "system_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "FSM_CORE";

/* ============================================================
   VARIÁVEIS DE ESTADO — Mantidas conforme original
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
    g_fsm_acender_em_ms  = 0;
}

/* ============================================================
   fsm_obstaculo_keepalive
============================================================ */
void fsm_obstaculo_keepalive(void)
{
    if (g_fsm_state == STATE_OBSTACULO) {
        g_fsm_obstaculo_last_ms = fsm_agora_ms();
    }
}

/* ============================================================
   fsm_verificar_radar
   ──────────────────────────────────────────────────────────
   Usa radar_is_connected() como fonte de verdade.

   O HLK-LD2450 envia frames APENAS quando detecta alvos.
   Sem alvos, a UART pode ficar silenciosa por longos períodos
   — isso é comportamento NORMAL, não falha do sensor.

   radar_is_connected() usa s_last_read_ok interno do driver:
   fica false após NO_FRAME_LIMIT ciclos sem qualquer BYTE
   na UART. Mesmo sem alvos, o sensor envia frames vazios
   periodicamente — se não há bytes, o sensor está morto.

   LÓGICA:
     teve_frame=true                    → sensor com alvos → OK
     teve_frame=false + UART activa     → sem alvos, normal → OK
     teve_frame=false + UART sem bytes  → sensor morto → SAFE_MODE
============================================================ */
void fsm_verificar_radar(bool teve_frame, bool comm_ok)
{
    bool sensor_conectado = radar_is_connected();

    if (teve_frame) {
        /* Frame com alvos — sensor a funcionar */
        g_fsm_radar_fail_cnt = 0;
        g_fsm_radar_ok_cnt++;

        if (!g_fsm_radar_ok) {
            g_fsm_radar_ok     = true;
            g_fsm_radar_ok_cnt = 0;
            ESP_LOGI(TAG, "[RADAR] Recuperado — sensor activo.");
            if (g_fsm_state == STATE_SAFE_MODE)
                g_fsm_state = STATE_IDLE;
        }

    } else if (!sensor_conectado) {
        /* Sem bytes na UART — sensor fisicamente desligado */
        g_fsm_radar_ok_cnt = 0;
        g_fsm_radar_fail_cnt++;

        /* Debounce 3 ciclos (300ms) antes de declarar SAFE_MODE */
        if (g_fsm_radar_ok && g_fsm_radar_fail_cnt >= 3) {
            g_fsm_radar_ok = false;
            ESP_LOGE(TAG, "[RADAR] Sensor desligado — SAFE MODE activado.");
        }

    } else {
        /* Sem alvos mas UART activa — comportamento normal */
        g_fsm_radar_fail_cnt = 0;

        /* Recupera se estava em SAFE_MODE e sensor voltou */
        if (!g_fsm_radar_ok) {
            g_fsm_radar_ok     = true;
            g_fsm_radar_ok_cnt = 0;
            ESP_LOGI(TAG, "[RADAR] UART activa — saída de SAFE MODE.");
            if (g_fsm_state == STATE_SAFE_MODE)
                g_fsm_state = STATE_IDLE;
        }
    }

    (void)comm_ok;
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
    g_fsm_acender_em_ms     = 0;
    g_fsm_master_claim_ms   = 0;
    g_fsm_sem_vizinho_ms    = 0;
    g_fsm_obstaculo_last_ms = 0;
    g_fsm_left_was_offline  = false;
    g_fsm_left_offline_ms   = 0;
    g_fsm_tc_timeout_ms     = 0;
    g_fsm_last_detect_ms    = 0;

    ESP_LOGI(TAG, "FSM Produção inicializada — Modo Hardware Real");
}

/* ============================================================
   state_machine_update — Ciclo a 100ms
============================================================ */
void state_machine_update(bool comm_ok, bool is_master, bool radar_teve_frame)
{
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
