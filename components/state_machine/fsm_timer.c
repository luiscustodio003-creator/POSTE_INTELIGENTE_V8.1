/* ============================================================
   MÓDULO     : fsm_timer
   FICHEIRO   : fsm_timer.c — Gestão de Timers e Timeouts
   VERSÃO     : 1.3  |  2026-05-03
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   - Processamento de timeouts de tráfego e rede.
   - Gestão de pré-acendimento (ETA).
   - Recuperação automática de estados de erro (Obstáculo).
   - Heartbeat de rede para o papel de Master.

   ALTERAÇÕES v1.2 → v1.3:
   ─────────────────────────
   - CORRIGIDO: _passo9_timeout_seguranca_tc()
     Antes: só resetava g_fsm_Tc quando o timeout expirava.
     Agora: também reseta g_fsm_enviados_dir e faz T-- para
     cada envio pendente sem confirmação — evita que env_dir
     fique preso indefinidamente quando um veículo anunciado
     a B não chegou (saiu do radar de A antes de entrar em B).
     A condição de activação foi alargada: dispara quando
     g_fsm_Tc > 0 OU g_fsm_enviados_dir > 0, não apenas Tc.
============================================================ */

#include "fsm_timer.h"
#include "fsm_core.h"
#include "comm_manager.h"
#include "system_config.h"
#include "esp_log.h"

static const char *TAG = "FSM_TMR";


/* ── Passo 5: Proteção contra T preso (Vizinho offline) ───── */
static void _passo5_verificar_t_estagnado(uint64_t agora)
{
    if (g_fsm_left_was_offline && g_fsm_T > 0) {
        if ((agora - g_fsm_left_offline_ms) > T_STUCK_TIMEOUT_MS) {
            g_fsm_T = 0;
            ESP_LOGW(TAG, "T resetado: vizinho esquerdo offline há muito tempo.");
        }
    }
}


/* ── Passo 6: Execução de Pré-acendimento (ETA) ───────────── */
static void _passo6_processar_eta(uint64_t agora)
{
    if (g_fsm_acender_em_ms > 0 && agora >= g_fsm_acender_em_ms) {
        g_fsm_acender_em_ms = 0;

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


/* ── Passo 7: Timeout de Apagamento ───────────────────────── */
static void _passo7_gestao_apagamento(uint64_t agora, bool is_master)
{
    if (!g_fsm_apagar_pend) return;

    /* Só apaga se não houver tráfego local (T) nem remoto (Tc) */
    if (g_fsm_T != 0 || g_fsm_Tc != 0) return;

    if ((agora - g_fsm_last_detect_ms) >= TRAFIC_TIMEOUT_MS) {
        g_fsm_apagar_pend   = false;
        g_fsm_last_speed    = 0.0f;
        g_fsm_acender_em_ms = 0;

        if (is_master) {
            g_fsm_state = STATE_MASTER;
        } else if (g_fsm_era_autonomo) {
            g_fsm_state = STATE_AUTONOMO;
            g_fsm_era_autonomo = false;
        } else {
            g_fsm_state = STATE_IDLE;
        }

        ESP_LOGI(TAG, "Timeout de tráfego: estado → %s",
                 state_machine_get_state_name());
    }
}


/* ── Passo 8: Auto-remoção de Estado de Obstáculo ─────────── */
static void _passo8_limpeza_obstaculo(uint64_t agora, bool is_master)
{
    (void)is_master;

    if (g_fsm_state != STATE_OBSTACULO || g_fsm_obstaculo_last_ms == 0) return;
    if ((agora - g_fsm_obstaculo_last_ms) < OBSTACULO_REMOVE_MS) return;

    if (g_fsm_T > 0) g_fsm_T--;
    g_fsm_obstaculo_last_ms = 0;

    ESP_LOGW(TAG, "Obstáculo removido por inactividade → T=%d Tc=%d",
             g_fsm_T, g_fsm_Tc);

    if (g_fsm_T == 0 && g_fsm_Tc == 0) {
        fsm_agendar_apagar();
        ESP_LOGI(TAG, "Apagamento agendado após remoção de obstáculo.");
    } else {
        g_fsm_state = STATE_LIGHT_ON;
        ESP_LOGI(TAG, "Outros veículos presentes → LIGHT_ON.");
    }
}


/* ── Passo 9: Timeout de Segurança (Tc + env_dir) ─────────────
   Activado quando:
     - g_fsm_Tc > 0: veículo anunciado pela esquerda não chegou
     - g_fsm_enviados_dir > 0: veículo anunciado à direita não
       foi confirmado por B (B nunca enviou PASSED)

   Em ambos os casos, o TC_TIMEOUT_MS deve ter sido definido
   quando o TC_INC foi enviado/recebido.

   Acção:
     - Reseta Tc e env_dir
     - Para cada env_dir pendente, faz T-- (o veículo perdeu-se)
     - Agenda apagamento se T=0 e Tc=0
──────────────────────────────────────────────────────────── */
static void _passo9_timeout_seguranca_tc(uint64_t agora)
{
    /* Sem timeout definido — nada a fazer */
    if (g_fsm_tc_timeout_ms == 0) return;

    /* Timeout ainda não expirou */
    if (agora <= g_fsm_tc_timeout_ms) return;

    bool algo_resetado = false;

    /* Reset de Tc — veículo anunciado da esquerda não chegou */
    if (g_fsm_Tc > 0) {
        ESP_LOGW(TAG, "[TMR] Tc timeout — veículo não chegou (Tc=%d)", g_fsm_Tc);
        g_fsm_Tc = 0;
        algo_resetado = true;
    }

    /* Reset de env_dir — B não confirmou os veículos que A anunciou.
       Para cada envio pendente, T-- pois o veículo perdeu-se entre postes. */
    if (g_fsm_enviados_dir > 0) {
        ESP_LOGW(TAG, "[TMR] env_dir timeout — B não confirmou (env_dir=%d T=%d)",
                 g_fsm_enviados_dir, g_fsm_T);
        while (g_fsm_enviados_dir > 0) {
            if (g_fsm_T > 0) g_fsm_T--;
            g_fsm_enviados_dir--;
        }
        algo_resetado = true;
    }

    if (algo_resetado) {
        g_fsm_tc_timeout_ms = 0;
        ESP_LOGI(TAG, "[TMR] Após timeout: T=%d Tc=%d env_dir=%d",
                 g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);
        if (g_fsm_T == 0 && g_fsm_Tc == 0)
            fsm_agendar_apagar();
    }
}


/* ── Passo 12: Manutenção do Papel Master (Heartbeat) ─────── */
static void _passo12_master_heartbeat(uint64_t agora, bool is_master)
{
    if (!is_master || POST_POSITION != 0) return;

    if ((agora - g_fsm_master_claim_ms) >= MASTER_CLAIM_HB_MS) {
        g_fsm_master_claim_ms = agora;
        comm_send_master_claim();
    }
}


/* ============================================================
   fsm_timer_update — ponto de entrada único
============================================================ */
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
