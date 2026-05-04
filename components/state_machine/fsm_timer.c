/* ============================================================
   MÓDULO     : fsm_timer
   FICHEIRO   : fsm_timer.c — Gestão de Timers e Timeouts
   VERSÃO     : 1.4  |  2026-05-04
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   - Processamento de timeouts de tráfego e rede.
   - Gestão de pré-acendimento (ETA).
   - Recuperação automática de estados de erro (Obstáculo).
   - Heartbeat de rede para o papel de Master.

   ALTERAÇÕES v1.3 → v1.4:
   ─────────────────────────
   - CORRIGIDO: _passo9_timeout_seguranca_tc()

     BUG 1 — SITUAÇÃO:
       O timeout fazia T-- para cada env_dir pendente, assumindo
       que o veículo se tinha "perdido" entre postes.
       Mas a topologia é linear sem saídas — todo o veículo que
       A detecta chega obrigatoriamente a B. A única excepção é
       o modo obstáculo, que tem tratamento próprio (passo 8).
       Resultado: com TC_TIMEOUT_MS demasiado curto (inferior ao
       tempo de trânsito entre postes), o passo 9 disparava antes
       de B enviar o PASSED, fazendo T-- prematuramente em A.
       Quando o PASSED de B chegava depois, on_prev_passed_received()
       fazia outro T--, tornando T negativo.

     CORRECÇÃO:
       O passo 9 deixa de fazer T-- em env_dir.
       Apenas limpa env_dir como limpeza administrativa de contadores
       UDP — o veículo está na estrada, não desapareceu.
       T só desce quando on_prev_passed_received() recebe o PASSED
       real de B, ou pelo passo 5 (T preso com vizinho offline).
       O TC_TIMEOUT_MS mantém-se como segurança contra pacotes UDP
       completamente perdidos após todos os retries — mas não
       interfere com a contagem de tráfego.

   ALTERAÇÕES v1.2 → v1.3:
   ─────────────────────────
   - CORRIGIDO: _passo9 também resetava g_fsm_enviados_dir e
     fazia T-- por cada env_dir pendente (versão anterior).
============================================================ */

#include "fsm_timer.h"
#include "fsm_core.h"
#include "comm_manager.h"
#include "system_config.h"
#include "esp_log.h"

static const char *TAG = "FSM_TMR";


/* ── Passo 5: Protecção contra T preso (Vizinho offline) ───── */
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


/* ── Passo 7: Gestão de Apagamento ────────────────────────── */
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
        ESP_LOGI(TAG, "Apagamento → STATE_MASTER (poste MASTER).");
    } else if (!g_fsm_right_online && !comm_left_online()) {
        g_fsm_state = STATE_AUTONOMO;
        ESP_LOGI(TAG, "Apagamento → AUTONOMO (sem vizinhos).");
    } else {
        g_fsm_state = STATE_IDLE;
        ESP_LOGI(TAG, "Apagamento → IDLE.");
    }
}


/* ── Passo 8: Limpeza de Obstáculo ────────────────────────── */
static void _passo8_limpeza_obstaculo(uint64_t agora, bool is_master)
{
    if (g_fsm_state != STATE_OBSTACULO) return;

    if ((agora - g_fsm_obstaculo_last_ms) >= OBSTACULO_REMOVE_MS) {
        ESP_LOGI(TAG, "[OBSTÁCULO] Removido — sem detecção há %llums",
                 (unsigned long long)OBSTACULO_REMOVE_MS);

        /* O veículo obstáculo saiu — T-- correspondente */
        if (g_fsm_T > 0) g_fsm_T--;

        if (g_fsm_T == 0 && g_fsm_Tc == 0) {
            fsm_agendar_apagar();
        } else {
            g_fsm_state = STATE_LIGHT_ON;
            ESP_LOGI(TAG, "Outros veículos presentes → LIGHT_ON.");
        }
    }
}


/* ── Passo 9: Timeout de Segurança UDP ────────────────────────
   CORRIGIDO v1.4: topologia linear sem saídas.

   ANTES (errado):
     Fazia T-- por cada env_dir pendente, assumindo que o veículo
     se perdera. Com TC_TIMEOUT_MS curto, disparava antes de B
     responder → T-- duplo quando o PASSED chegava depois.

   AGORA (correcto):
     Apenas limpa env_dir e Tc como limpeza de contadores UDP.
     NÃO toca em T — o veículo está na estrada, não desapareceu.
     T só desce via on_prev_passed_received() (PASSED real de B)
     ou via passo 5 (vizinho offline há muito tempo).

   O timeout serve apenas para proteger contra pacotes UDP
   completamente perdidos após todos os retries do udp_manager,
   evitando que env_dir e Tc fiquem presos indefinidamente.
──────────────────────────────────────────────────────────── */
static void _passo9_timeout_seguranca_tc(uint64_t agora)
{
    if (g_fsm_tc_timeout_ms == 0) return;
    if (agora <= g_fsm_tc_timeout_ms) return;

    bool algo_resetado = false;

    /* Tc preso — veículo anunciado de A não chegou a B via UDP.
       Apenas limpa o contador — não toca em T. */
    if (g_fsm_Tc > 0) {
        ESP_LOGW(TAG, "[TMR] Tc timeout — limpeza UDP (Tc=%d) — T mantém-se=%d",
                 g_fsm_Tc, g_fsm_T);
        g_fsm_Tc    = 0;
        algo_resetado = true;
    }

    /* env_dir preso — B não confirmou via PASSED após todos os retries.
       Limpa o contador mas NÃO faz T-- — o veículo está na estrada. */
    if (g_fsm_enviados_dir > 0) {
        ESP_LOGW(TAG, "[TMR] env_dir timeout — limpeza UDP (env_dir=%d) — T mantém-se=%d",
                 g_fsm_enviados_dir, g_fsm_T);
        g_fsm_enviados_dir = 0;
        algo_resetado = true;
    }

    if (algo_resetado) {
        g_fsm_tc_timeout_ms = 0;
        ESP_LOGI(TAG, "[TMR] Após limpeza UDP: T=%d Tc=%d env_dir=%d",
                 g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);
        /* Não agenda apagamento — T pode ainda ser > 0 (veículo em trânsito) */
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
