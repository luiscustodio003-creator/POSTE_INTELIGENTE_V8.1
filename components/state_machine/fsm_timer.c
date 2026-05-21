/* ============================================================
   MÓDULO     : fsm_timer
   FICHEIRO   : fsm_timer.c — Gestão de timeouts: tráfego, ETA, obstáculo, heartbeat
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)
============================================================ */

#include "fsm_timer.h"
#include "fsm_core.h"
#include "comm_manager.h"
#include "system_config.h"
#include "esp_log.h"

static const char *TAG = "FSM_TMR";


/* ── Passo 5: T preso com vizinho esquerdo offline ─────────── */
static void _passo5_verificar_t_estagnado(uint64_t agora)
{
    if (g_fsm_left_was_offline && g_fsm_T > 0) {
        if ((agora - g_fsm_left_offline_ms) > T_STUCK_TIMEOUT_MS) {
            g_fsm_T = 0;
            ESP_LOGW(TAG, "T resetado: vizinho esquerdo offline há muito tempo.");
        }
    }
}


/* ── Passo 6: Pré-acendimento por ETA ─────────────────────── */
static void _passo6_processar_eta(uint64_t agora)
{
    uint64_t acender_em = fsm_acender_em_ms_get();
    if (acender_em > 0 && agora >= acender_em) {
        fsm_acender_em_ms_set(0);

        if (g_fsm_Tc > 0) {
            if (g_fsm_state == STATE_IDLE   ||
                g_fsm_state == STATE_MASTER ||
                g_fsm_state == STATE_AUTONOMO) {
                g_fsm_state = STATE_LIGHT_ON;
            }
            ESP_LOGI(TAG, "ETA atingido: estado → LIGHT_ON para veículo em aproximação.");
        }
    }
}


/* ── Passo 7: Apagamento após tráfego ─────────────────────── */
static void _passo7_gestao_apagamento(uint64_t agora, bool is_master)
{
    if (!g_fsm_apagar_pend) return;

    if (g_fsm_T > 0 || g_fsm_Tc > 0) {
        g_fsm_apagar_pend = false;
        return;
    }

    if ((agora - g_fsm_last_detect_ms) < TRAFIC_TIMEOUT_MS) return;

    g_fsm_apagar_pend = false;

    if (is_master && POST_POSITION == 0) {
        g_fsm_state = STATE_MASTER;
        ESP_LOGI(TAG, "Apagamento → STATE_MASTER (pos=0).");
    } else if (!comm_right_online() && !comm_left_online()) {
        g_fsm_state = STATE_AUTONOMO;
        ESP_LOGI(TAG, "Apagamento → AUTONOMO (sem vizinhos).");
    } else {
        g_fsm_state = STATE_IDLE;
        ESP_LOGI(TAG, "Apagamento → IDLE.");
    }
}


/* ── Passo 8: Limpeza de obstáculo após 8s sem detecção ───── */
static void _passo8_limpeza_obstaculo(uint64_t agora, bool is_master)
{
    if (g_fsm_state != STATE_OBSTACULO) return;

    if ((agora - g_fsm_obstaculo_last_ms) >= OBSTACULO_REMOVE_MS) {
        ESP_LOGI(TAG, "[OBSTÁCULO] Removido — sem detecção há %llums",
                 (unsigned long long)OBSTACULO_REMOVE_MS);

        if (g_fsm_T > 0) g_fsm_T--;

        if (g_fsm_T == 0 && g_fsm_Tc == 0) {
            fsm_agendar_apagar();
        } else {
            g_fsm_state = STATE_LIGHT_ON;
            ESP_LOGI(TAG, "Outros veículos presentes → LIGHT_ON.");
        }
    }
}


/* ── Passo 9: Timeout de segurança UDP ────────────────────────
   Limpa env_dir e Tc quando UDP perdido após todos os retries.
   NÃO toca em T — veículo está na estrada, não desapareceu. */
static void _passo9_timeout_seguranca_tc(uint64_t agora)
{
    uint64_t tc_deadline = fsm_tc_timeout_ms_get();
    if (tc_deadline == 0) return;
    if (agora <= tc_deadline) return;

    bool algo_resetado = false;

    if (g_fsm_Tc > 0) {
        ESP_LOGW(TAG, "[TMR] Tc timeout — limpeza UDP (Tc=%d) — T mantém-se=%d",
                 g_fsm_Tc, g_fsm_T);
        g_fsm_Tc    = 0;
        algo_resetado = true;
    }

    if (g_fsm_enviados_dir > 0) {
        ESP_LOGW(TAG, "[TMR] env_dir timeout — limpeza UDP (env_dir=%d) — T mantém-se=%d",
                 g_fsm_enviados_dir, g_fsm_T);
        g_fsm_enviados_dir = 0;
        algo_resetado = true;
    }

    if (algo_resetado) {
        fsm_tc_timeout_ms_set(0);
        ESP_LOGI(TAG, "[TMR] Após limpeza UDP: T=%d Tc=%d env_dir=%d",
                 g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);
    }
}


/* ── Passo 12: Heartbeat de master (qualquer MASTER) ─────────
   Correcção: MASTER temporário (pos>0) também envia heartbeat.
   Sem isto, o cluster à direita perde autoridade após 15s e
   pode eleger um segundo MASTER dentro do mesmo cluster. */
static void _passo12_master_heartbeat(uint64_t agora, bool is_master)
{
    if (!is_master) return;

    if ((agora - g_fsm_master_claim_ms) >= MASTER_CLAIM_HB_MS) {
        g_fsm_master_claim_ms = agora;
        comm_send_master_claim_id(POSTE_ID);
    }
}


/* ── fsm_timer_update — ponto de entrada único ────────────── */
void fsm_timer_update(bool comm_ok, bool is_master)
{
    uint64_t agora = fsm_agora_ms();

    _passo5_verificar_t_estagnado(agora);
    _passo6_processar_eta(agora);
    _passo7_gestao_apagamento(agora, is_master);
    _passo8_limpeza_obstaculo(agora, is_master);
    _passo9_timeout_seguranca_tc(agora);
    _passo12_master_heartbeat(agora, is_master);

    (void)comm_ok;
}
