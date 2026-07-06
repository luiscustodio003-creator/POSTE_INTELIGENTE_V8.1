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
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <stdio.h>
#include <stdarg.h>

static const char *TAG = "FSM_TMR";


/* ── _box_* — desenho de caixa auto-alinhada (ver fsm_events.c) ── */
#define BOX_W 48

static void _box_top(void)
{
    printf("\n┌");
    for (int i = 0; i < BOX_W; i++) printf("─");
    printf("┐\n");
}

static void _box_bottom(void)
{
    printf("└");
    for (int i = 0; i < BOX_W; i++) printf("─");
    printf("┘\n\n");
}

static void _box_line(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    printf("│%-*s│\n", BOX_W, buf);
}


/* ── _print_fadedown_box ─────────────────────────────────────
   Destaque visual no log no momento exacto em que o fade-down
   é accionado (T=0 Tc=0 env_dir=0 confirmados há TRAFIC_TIMEOUT_MS). */
static void _print_fadedown_box(const char *estado_novo)
{
    _box_top();
    _box_line(" FADE DOWN INICIADO   P%-2d", POST_POSITION);
    _box_line(" T=0 Tc=0 env_dir=0 confirmados");
    _box_line(" Novo estado: %s", estado_novo);
    _box_bottom();
}


/* ── Passo 5: T preso com vizinho esquerdo offline ─────────── */
static void _passo5_verificar_t_estagnado(uint64_t agora)
{
    if (g_fsm_left_was_offline && g_fsm_T > 0) {
        if ((agora - g_fsm_left_offline_ms) > T_STUCK_TIMEOUT_MS) {
            portENTER_CRITICAL(&g_fsm_counters_mux);
            g_fsm_T = 0;
            portEXIT_CRITICAL(&g_fsm_counters_mux);
            ESP_LOGW(TAG, "T resetado: vizinho esquerdo offline há muito tempo.");
        }
    }
}


/* ── Passo 5c: Fallback de acendimento quando SPD não chega ───
   Quando TC_INC é recebido mas o SPD correspondente se perde na
   rede WiFi, o fallback garante que a luz acende ao fim de
   SPD_FALLBACK_MS (1s) em vez de ficar à espera indefinidamente.
   on_spd_received() cancela este timer se o SPD chegar a tempo. */
static void _passo5c_spd_fallback(uint64_t agora)
{
    uint64_t fb = fsm_spd_fallback_ms_get();
    if (fb == 0 || agora < fb) return;

    fsm_spd_fallback_ms_set(0);

    if (g_fsm_Tc > 0 && fsm_acender_em_ms_get() == 0) {
        fsm_acender_em_ms_set(agora);
        ESP_LOGW(TAG, "[SPD_FALLBACK] SPD perdido — acende imediatamente (Tc=%d)", g_fsm_Tc);
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
        _print_fadedown_box("MASTER (pos=0)");
    } else if (!comm_right_online() && !comm_left_online()) {
        g_fsm_state = STATE_AUTONOMO;
        ESP_LOGI(TAG, "Apagamento → AUTONOMO (sem vizinhos).");
        _print_fadedown_box("AUTONOMO (sem vizinhos)");
    } else {
        g_fsm_state = STATE_IDLE;
        ESP_LOGI(TAG, "Apagamento → IDLE.");
        _print_fadedown_box("IDLE");
    }
}


/* ── Passo 8: Limpeza de obstáculo após 8s sem detecção ───── */
static void _passo8_limpeza_obstaculo(uint64_t agora, bool is_master)
{
    if (g_fsm_state != STATE_OBSTACULO) return;

    if ((agora - g_fsm_obstaculo_last_ms) >= OBSTACULO_REMOVE_MS) {
        ESP_LOGI(TAG, "[OBSTÁCULO] Removido — sem detecção há %llums",
                 (unsigned long long)OBSTACULO_REMOVE_MS);

        portENTER_CRITICAL(&g_fsm_counters_mux);
        if (g_fsm_T > 0) g_fsm_T--;
        bool obs_all_clear = (g_fsm_T == 0 && g_fsm_Tc == 0 && g_fsm_enviados_dir == 0);
        portEXIT_CRITICAL(&g_fsm_counters_mux);

        if (obs_all_clear) {
            fsm_agendar_apagar();
        } else {
            g_fsm_state = STATE_LIGHT_ON;
            ESP_LOGI(TAG, "Outros veículos presentes → LIGHT_ON.");
        }
    }
}


/* ── Passo 5b: REMOVIDO ───────────────────────────────────────
   T só se resolve por confirmação real (PASSED) ou detecção local
   (EVT_LOCAL) — nunca por T_STUCK_TIMEOUT_MS. Risco aceite. */


/* ── Passo 9: Rede de segurança de ÚLTIMO RECURSO ─────────────
   NÃO faz parte do funcionamento normal do handoff — o T/Tc/env_dir
   esperam indefinidamente pela confirmação real (PASSED). Isto só
   dispara ao fim de TC_TIMEOUT_MS (60s, generoso de propósito) se
   essa confirmação NUNCA chegar — pacote definitivamente perdido, ou
   veículo que saiu da via sem nunca ser confirmado pelo poste
   seguinte. Sem isto, esses dois casos deixavam a luz acesa para
   sempre, sem qualquer forma de recuperação automática. */
static void _passo9_seguranca_ultimo_recurso(uint64_t agora)
{
    uint64_t tc_deadline = fsm_tc_timeout_ms_get();
    if (tc_deadline == 0) return;
    if (agora <= tc_deadline) return;

    portENTER_CRITICAL(&g_fsm_counters_mux);
    int  snap_tc  = g_fsm_Tc;
    int  snap_env = g_fsm_enviados_dir;
    if (snap_tc  > 0) g_fsm_Tc = 0;
    if (snap_env > 0) {
        if (g_fsm_T >= snap_env) g_fsm_T -= snap_env;
        else                     g_fsm_T  = 0;
        g_fsm_enviados_dir = 0;
    }
    bool tmr_all_clear = (g_fsm_T == 0 && g_fsm_Tc == 0);
    portEXIT_CRITICAL(&g_fsm_counters_mux);

    bool algo_resetado = (snap_tc > 0 || snap_env > 0);
    if (!algo_resetado) {
        fsm_tc_timeout_ms_set(0);
        return;
    }

    ESP_LOGE(TAG, "[TMR] SEGURANÇA ÚLTIMO RECURSO — confirmação nunca chegou "
                  "(Tc=%d env_dir=%d) → limpeza forçada após %llus",
             snap_tc, snap_env, (unsigned long long)(TC_TIMEOUT_MS / 1000ULL));

    fsm_tc_timeout_ms_set(0);
    if (tmr_all_clear) fsm_agendar_apagar();
}


/* ── Passo 11b: Heartbeat de obstáculo → vizinho direito ──────
   Enquanto em STATE_OBSTACULO, reenvia OBSTACULO a cada TC_TIMEOUT_MS/2.
   Mantém o TC_TIMEOUT do vizinho direito renovado — sem isto, se o veículo
   desaparecer sem chegar ao poste seguinte, o vizinho fica com Tc=1 e
   TC_TIMEOUT=0 → luz acesa indefinidamente (bug fix v5.4). */
static uint64_t s_obstaculo_hb_ms = 0;

static void _passo11b_obstaculo_heartbeat(uint64_t agora)
{
    if (g_fsm_state != STATE_OBSTACULO) {
        s_obstaculo_hb_ms = 0;
        return;
    }
    if (!g_fsm_right_online) return;

    if (s_obstaculo_hb_ms > 0 &&
        (agora - s_obstaculo_hb_ms) < (TC_TIMEOUT_MS / 2)) return;

    s_obstaculo_hb_ms = agora;
    comm_send_obstaculo(g_fsm_tc_last_vehicle_id, g_fsm_last_speed, 0);
    ESP_LOGD(TAG, "[OBST] heartbeat → vizinho direito (id=%u vel=%.1f)",
             (unsigned)g_fsm_tc_last_vehicle_id, g_fsm_last_speed);
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
    _passo5c_spd_fallback(agora);
    _passo6_processar_eta(agora);
    _passo7_gestao_apagamento(agora, is_master);
    _passo8_limpeza_obstaculo(agora, is_master);
    _passo9_seguranca_ultimo_recurso(agora);
    _passo11b_obstaculo_heartbeat(agora);
    _passo12_master_heartbeat(agora, is_master);

    (void)comm_ok;
}
