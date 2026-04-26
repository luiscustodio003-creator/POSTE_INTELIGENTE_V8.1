/* ============================================================
   MÓDULO     : fsm_timer
   FICHEIRO   : fsm_timer.c — Timers e timeouts da FSM
   VERSÃO     : 1.0  |  2026-04-26
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Passos 5, 6, 7, 8, 9 e 12 do state_machine_update().
   Cada passo é uma função independente chamada por fsm_timer_update().
============================================================ */

#include "fsm_timer.h"
#include "fsm_core.h"
#include "comm_manager.h"
#include "dali_manager.h"
#include "system_config.h"
#include "esp_log.h"

static const char *TAG = "FSM_TMR";


/* ── Passo 5: T preso quando vizinho esquerdo está offline ── */
static void _passo5_t_preso(uint64_t agora)
{
    if (g_fsm_left_was_offline && g_fsm_T > 0 &&
        (agora - g_fsm_left_offline_ms) > T_STUCK_TIMEOUT_MS) {
        g_fsm_T = 0;
        ESP_LOGW(TAG, "T forçado a 0 — viz. esq. offline há %llus",
                 (unsigned long long)(T_STUCK_TIMEOUT_MS / 1000));
    }
}


/* ── Passo 6: Timer de pré-acendimento (ETA) ──────────────── */
static void _passo6_pre_acendimento(uint64_t agora)
{
    if (g_fsm_acender_em_ms > 0 && agora >= g_fsm_acender_em_ms) {
        g_fsm_acender_em_ms = 0;
        if (g_fsm_Tc > 0) {
            dali_fade_up(g_fsm_last_speed);
            if (g_fsm_state == STATE_IDLE   ||
                g_fsm_state == STATE_MASTER ||
                g_fsm_state == STATE_AUTONOMO)
                g_fsm_state = STATE_LIGHT_ON;
            ESP_LOGI(TAG, "Pré-acendimento: luz ON (ETA atingido)");
        }
    }
}


/* ── Passo 7: Timeout de apagamento ───────────────────────── */
static void _passo7_apagamento(uint64_t agora, bool is_master)
{
    if (!g_fsm_apagar_pend) return;
    if (g_fsm_T != 0 || g_fsm_Tc != 0) return;
    if ((agora - g_fsm_last_detect_ms) < TRAFIC_TIMEOUT_MS) return;

    g_fsm_apagar_pend   = false;
    g_fsm_last_speed    = 0.0f;
    g_fsm_acender_em_ms = 0;

    if (g_fsm_state == STATE_LIGHT_ON  ||
        g_fsm_state == STATE_AUTONOMO  ||
        g_fsm_state == STATE_OBSTACULO) {

        const char *estado_str = is_master         ? "MASTER"   :
                                 g_fsm_era_autonomo ? "AUTONOMO" : "IDLE";
        if (is_master) {
            g_fsm_state = STATE_MASTER;
        } else if (g_fsm_era_autonomo) {
            g_fsm_state        = STATE_AUTONOMO;
            g_fsm_era_autonomo = false;
        } else {
            g_fsm_state = STATE_IDLE;
        }
        dali_fade_down();
        ESP_LOGI(TAG, "Apagamento — volta a %s", estado_str);
    }
}


/* ── Passo 8: Remoção automática de OBSTÁCULO ─────────────── */
static void _passo8_obstaculo(uint64_t agora, bool is_master)
{
    if (g_fsm_state != STATE_OBSTACULO) return;
    if (g_fsm_obstaculo_last_ms == 0) return;
    if ((agora - g_fsm_obstaculo_last_ms) < OBSTACULO_REMOVE_MS) return;

    g_fsm_state             = is_master ? STATE_MASTER : STATE_IDLE;
    g_fsm_obstaculo_last_ms = 0;
    dali_fade_down();
    ESP_LOGW(TAG, "[OBSTACULO] REMOVIDO — sem detecção há %us",
             OBSTACULO_REMOVE_MS / 1000);
}


/* ── Passo 9: Timeout de segurança Tc ─────────────────────── */
static void _passo9_tc_timeout(uint64_t agora)
{
    if (g_fsm_Tc <= 0 || g_fsm_tc_timeout_ms == 0) return;
    if (agora <= g_fsm_tc_timeout_ms) return;

    ESP_LOGW(TAG, "Timeout Tc — veículo não chegou (Tc=%d→0)", g_fsm_Tc);
    g_fsm_Tc            = 0;
    g_fsm_tc_timeout_ms = 0;
    fsm_agendar_apagar();
}


/* ── Passo 12: Heartbeat MASTER_CLAIM ─────────────────────── */
static void _passo12_master_claim(uint64_t agora, bool is_master)
{
    if (!is_master || POST_POSITION != 0) return;
    if ((agora - g_fsm_master_claim_ms) < MASTER_CLAIM_HB_MS) return;

    g_fsm_master_claim_ms = agora;
    comm_send_master_claim();
}


/* ============================================================
   fsm_timer_update — ponto de entrada único
   Chamado por state_machine_update() a cada 100ms.
============================================================ */
void fsm_timer_update(bool comm_ok, bool is_master)
{
    uint64_t agora = fsm_agora_ms();

    _passo5_t_preso(agora);
    _passo6_pre_acendimento(agora);
    _passo7_apagamento(agora, is_master);
    _passo8_obstaculo(agora, is_master);
    _passo9_tc_timeout(agora);
    _passo12_master_claim(agora, is_master);

    (void)comm_ok;
}
