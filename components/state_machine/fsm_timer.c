/* ============================================================
   MÓDULO     : fsm_timer
   FICHEIRO   : fsm_timer.c — Gestão de Timers e Timeouts
   VERSÃO     : 1.2  |  2026-04-30
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   - Processamento de timeouts de tráfego e rede.
   - Gestão de pré-acendimento (ETA).
   - Recuperação automática de estados de erro (Obstáculo).
   - Heartbeat de rede para o papel de Master.

   CORRECÇÕES v1.1 → v1.2:
   ─────────────────────────
   - REMOVIDO: dali_fade_up() em _passo6_processar_eta().
     A mudança de estado para LIGHT_ON é detectada por
     fsm_aplicar_luz() em fsm_task.c no ciclo seguinte.
   - REMOVIDO: dali_fade_down() em _passo7_gestao_apagamento().
     Idem — fsm_aplicar_luz() aplica o fade down ao detectar
     a transição para IDLE / MASTER / AUTONOMO.
   - REMOVIDO: #include "dali_manager.h" — já não necessário.
   - REMOVIDO: fsm_timer_process_timeouts() — função duplicada
     que repetia a lógica do _passo9_timeout_seguranca_tc().
     A única função pública é fsm_timer_update().
   - REMOVIDO: segundo #include "system_config.h" duplicado.
   - MANTIDO: toda a lógica de timeouts e transições de estado.
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
            /* Muda estado — fsm_aplicar_luz() aplica dali no ciclo seguinte */
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

        /* Define estado de destino — fsm_aplicar_luz() aplica dali no ciclo seguinte */
        if (is_master) {
            g_fsm_state = STATE_MASTER;
        } else if (g_fsm_era_autonomo) {
            g_fsm_state = STATE_AUTONOMO;
            g_fsm_era_autonomo = false;
        } else {
            g_fsm_state = STATE_IDLE;
        }

        ESP_LOGI(TAG, "Timeout de tráfego: estado → %s", state_machine_get_state_name());
    }
}

/* ── Passo 8: Auto-remoção de Estado de Obstáculo ─────────── */
/*
   CORRECÇÃO v1.2:
   O obstáculo ocupa T=1. Ao ser removido por inactividade,
   T-- é necessário antes de apagar — senão T fica preso em 1
   e o passo 7 nunca apaga a luz.
   Se ainda há outros veículos (T>1), volta a LIGHT_ON.
*/
static void _passo8_limpeza_obstaculo(uint64_t agora, bool is_master)
{
    (void)is_master;

    if (g_fsm_state != STATE_OBSTACULO || g_fsm_obstaculo_last_ms == 0) return;

    if ((agora - g_fsm_obstaculo_last_ms) < OBSTACULO_REMOVE_MS) return;

    /* Obstáculo desapareceu — decrementa T do slot do obstáculo */
    if (g_fsm_T > 0) g_fsm_T--;
    g_fsm_obstaculo_last_ms = 0;

    ESP_LOGW(TAG, "Obstáculo removido por inactividade → T=%d Tc=%d",
             g_fsm_T, g_fsm_Tc);

    if (g_fsm_T == 0 && g_fsm_Tc == 0) {
        /* Nenhum veículo — apagamento normal via passo 7 */
        fsm_agendar_apagar();
        ESP_LOGI(TAG, "Apagamento agendado após remoção de obstáculo.");
    } else {
        /* Ainda há outros veículos — volta a LIGHT_ON */
        g_fsm_state = STATE_LIGHT_ON;
        ESP_LOGI(TAG, "Outros veículos presentes → LIGHT_ON.");
    }
}

/* ── Passo 9: Timeout de Segurança do Tráfego Remoto (Tc) ──── */
static void _passo9_timeout_seguranca_tc(uint64_t agora)
{
    if (g_fsm_Tc <= 0 || g_fsm_tc_timeout_ms == 0) return;

    if (agora > g_fsm_tc_timeout_ms) {
        ESP_LOGW(TAG, "Veículo esperado (Tc) não chegou. Limpando registo.");
        g_fsm_Tc = 0;
        g_fsm_tc_timeout_ms = 0;
        fsm_agendar_apagar();
    }
}

/* ── Passo 12: Manutenção do Papel Master (Heartbeat) ─────── */
static void _passo12_master_heartbeat(uint64_t agora, bool is_master)
{
    /* Apenas o poste na posição 0 pode ser Master Claimer */
    if (!is_master || POST_POSITION != 0) return;

    if ((agora - g_fsm_master_claim_ms) >= MASTER_CLAIM_HB_MS) {
        g_fsm_master_claim_ms = agora;
        comm_send_master_claim();
    }
}


/* ============================================================
   fsm_timer_update — ponto de entrada único
   ──────────────────────────────────────────
   Executa todos os passos de timer do ciclo 100ms.
   Chamada por state_machine_update() em fsm_core.c.
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

    (void)comm_ok; /* Reservado para uso futuro */
}
