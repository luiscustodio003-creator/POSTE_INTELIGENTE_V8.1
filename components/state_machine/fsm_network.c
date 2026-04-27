/* ============================================================
   MÓDULO     : fsm_network
   FICHEIRO   : fsm_network.c
   VERSÃO     : 2.0  |  2026-04-27
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno

   ALTERAÇÕES v1.0 → v2.0:
   ─────────────────────────
   1. fsm_network_vizinhos(): removida chamada a comm_left_known()
      (função não existia). Substituída por lógica baseada em
      POST_POSITION + flag g_fsm_left_was_offline.
      Regra: só marca offline se vizinho já esteve online antes
      (evita MASTER prematuro no arranque durante DISCOVER).

   2. fsm_network_estados_degradados():
      AUTONOMO agora tem 3 condições de entrada:
        a) sem WiFi → imediato
        b) WiFi OK + sem vizinhos → após AUTONOMO_DELAY_MS
        c) viz.esq notificou RADAR_FAIL → imediato
           (g_fsm_left_radar_fail=true, tratado em fsm_events)

      SAFE MODE:
        Radar KO → sempre SAFE MODE (independente do WiFi).
        Sai apenas quando radar recupera (on_radar_ok_received).

   3. fsm_network_master():
      Poste assume MASTER quando:
        a) is_master (pos=0) — normal
        b) viz.esq offline há AUTONOMO_DELAY_MS — normal
        c) viz.dir notificou RADAR_FAIL → imediato
           (g_fsm_right_radar_fail=true, tratado em on_radar_fail_received)
      Cede MASTER quando:
        a) viz.esq volta online E pos>0
        b) viz.dir notificou RADAR_OK
           (g_fsm_right_radar_fail=false, tratado em on_radar_ok_received)

   4. T_STUCK_TIMEOUT_MS: força T=0 quando viz.esq offline há
      demasiado tempo (PASSED nunca chegará).

   DEPENDÊNCIAS:
   ─────────────
   fsm_core.h, fsm_events.h, comm_manager.h,
   dali_manager.h, system_config.h
============================================================ */

#include "fsm_network.h"
#include "fsm_core.h"
#include "fsm_events.h"
#include "comm_manager.h"
#include "dali_manager.h"
#include "system_config.h"
#include "esp_log.h"

static const char *TAG = "FSM_NET";

/* T preso: viz.esq offline → PASSED nunca chegará → força T=0 */
#define T_STUCK_TIMEOUT_MS   (TRAFIC_TIMEOUT_MS * 3)


/* ============================================================
   fsm_network_vizinhos — detecta mudanças de conectividade
   ──────────────────────────────────────────────────────────
   Passo 3: viz. direito online/offline
   Passo 4: viz. esquerdo online/offline + T preso

   REGRA arranque: g_fsm_left_was_offline só é marcado true
   depois do viz.esq ter estado online pelo menos uma vez.
   Antes disso (DISCOVER em curso) não marcamos offline para
   não assumir MASTER prematuramente.
============================================================ */
void fsm_network_vizinhos(bool comm_ok, bool is_master)
{
    uint64_t agora = fsm_agora_ms();

    /* ── Passo 3: Viz. direito ────────────────────────────── */
    bool dir_ok = comm_right_online();
    if (!dir_ok &&  g_fsm_right_online) sm_on_right_neighbor_offline();
    if ( dir_ok && !g_fsm_right_online) sm_on_right_neighbor_online();

    /* ── Passo 4: Viz. esquerdo ───────────────────────────── */
    /* Poste 0 nunca tem viz. esquerdo */
    if (POST_POSITION == 0) {
        g_fsm_left_was_offline = false;
        g_fsm_left_offline_ms  = 0;
        (void)comm_ok; (void)is_master;
        return;
    }

    bool esq_ok = comm_left_online();

    /* Só marca offline se viz.esq já esteve online antes.
       Estado inicial: g_fsm_left_was_offline=false.
       Primeira vez que esq_ok fica true → vizinho descoberto.
       Se depois ficar false → marca offline.                  */
    if (!esq_ok && !g_fsm_left_was_offline &&
        comm_left_was_ever_online()) {
        g_fsm_left_was_offline = true;
        g_fsm_left_offline_ms  = agora;
        ESP_LOGW(TAG, "Viz. esq. ficou offline");
    }

    /* Viz. esq. voltou online */
    if (esq_ok && g_fsm_left_was_offline) {
        g_fsm_left_was_offline = false;
        g_fsm_left_radar_fail  = false;  /* reset flag RADAR_FAIL */
        if (g_fsm_state == STATE_MASTER && POST_POSITION > 0) {
            g_fsm_state = STATE_IDLE;
            comm_send_master_claim();
        }
        ESP_LOGI(TAG, "Viz. esq. voltou online → IDLE");
    }

    /* T preso: viz.esq offline há demasiado tempo.
       PASSED nunca chegará — força T=0 para não manter
       luz acesa indefinidamente. */
    if (g_fsm_left_was_offline && g_fsm_T > 0 &&
        (agora - g_fsm_left_offline_ms) > T_STUCK_TIMEOUT_MS) {
        ESP_LOGW(TAG, "T forçado a 0 — viz.esq offline há %lus",
                 (unsigned long)(T_STUCK_TIMEOUT_MS / 1000));
        g_fsm_T = 0;
        fsm_agendar_apagar();
    }

    (void)comm_ok; (void)is_master;
}


/* ============================================================
   fsm_network_master — gestão do papel MASTER
   ──────────────────────────────────────────────────────────
   MASTER quando:
     a) is_master (pos=0) — sempre
     b) viz.esq offline há AUTONOMO_DELAY_MS — temporário
     c) viz.dir notificou RADAR_FAIL — imediato
        (já tratado em on_radar_fail_received, flag =true)

   Cede MASTER quando:
     a) viz.esq volta online E pos>0
     b) viz.dir notificou RADAR_OK
        (já tratado em on_radar_ok_received, flag =false)
============================================================ */
void fsm_network_master(bool comm_ok, bool is_master)
{
    uint64_t agora = fsm_agora_ms();

    /* a) Transição normal IDLE ↔ MASTER */
    if (is_master && g_fsm_state == STATE_IDLE)
        g_fsm_state = STATE_MASTER;

    if (!is_master && g_fsm_state == STATE_MASTER &&
        !g_fsm_left_was_offline && !g_fsm_right_radar_fail) {
        g_fsm_state = STATE_IDLE;
        ESP_LOGI(TAG, "MASTER cedido → IDLE");
    }

    /* b) Viz.esq offline há AUTONOMO_DELAY_MS → MASTER temporário */
    if (!is_master && g_fsm_left_was_offline && g_fsm_radar_ok &&
        (agora - g_fsm_left_offline_ms) > AUTONOMO_DELAY_MS &&
        g_fsm_state == STATE_IDLE) {
        g_fsm_state = STATE_MASTER;
        ESP_LOGW(TAG, "MASTER temporário — viz.esq offline");
    }

    /* Viz.esq voltou → cede MASTER (se pos>0 e sem RADAR_FAIL dir) */
    if (!is_master && !g_fsm_left_was_offline &&
        !g_fsm_right_radar_fail &&
        g_fsm_state == STATE_MASTER && POST_POSITION > 0) {
        g_fsm_state = STATE_IDLE;
        ESP_LOGI(TAG, "Viz.esq voltou → cede MASTER → IDLE");
    }

    (void)comm_ok;
}


/* ============================================================
   fsm_network_estados_degradados — AUTONOMO e SAFE_MODE
   ──────────────────────────────────────────────────────────
   SAFE_MODE:
     radar KO → sempre SAFE MODE (independente do WiFi e vizinhos)
     Sai apenas quando radar recupera (via on_radar_ok_received
     e fsm_verificar_radar → g_fsm_state = STATE_IDLE).

   AUTONOMO — 3 condições:
     a) sem WiFi → imediato (radar OK)
     b) WiFi OK + sem vizinhos → após AUTONOMO_DELAY_MS
     c) viz.esq RADAR_FAIL → imediato (já em on_radar_fail_received)
        Aqui apenas mantemos consistência de estado.

   Sai de AUTONOMO quando:
     - WiFi volta E tem vizinhos E viz.esq sem RADAR_FAIL
     - on_radar_ok_received(from_left=true) trata o caso (c)
============================================================ */
void fsm_network_estados_degradados(bool comm_ok, bool is_master)
{
    uint64_t agora   = fsm_agora_ms();
    bool tem_vizinho = comm_left_online() || comm_right_online();

    /* ── SAFE MODE: radar KO ─────────────────────────────────
       fsm_verificar_radar() já faz a transição e envia RADAR_FAIL.
       Aqui apenas garantimos que não saímos sem radar recuperado. */
    if (!g_fsm_radar_ok &&
        g_fsm_state != STATE_SAFE_MODE &&
        g_fsm_state != STATE_OBSTACULO) {
        g_fsm_era_autonomo = false;
        g_fsm_state = STATE_SAFE_MODE;
        dali_set_brightness(LIGHT_SAFE_MODE);
        ESP_LOGW(TAG, "SAFE MODE — radar KO");
    }

    /* ── AUTONOMO: sem WiFi → imediato ──────────────────────── */
    if (!comm_ok && g_fsm_radar_ok &&
        g_fsm_state != STATE_SAFE_MODE &&
        g_fsm_state != STATE_LIGHT_ON  &&
        g_fsm_state != STATE_OBSTACULO) {
        if (g_fsm_state != STATE_AUTONOMO) {
            g_fsm_state = STATE_AUTONOMO;
            ESP_LOGW(TAG, "AUTONOMO — sem WiFi");
        }
    }

    /* ── AUTONOMO: WiFi OK + sem vizinhos → após delay ──────── */
    if (g_fsm_radar_ok && comm_ok && !tem_vizinho) {
        if (g_fsm_sem_vizinho_ms == 0) g_fsm_sem_vizinho_ms = agora;
        if ((agora - g_fsm_sem_vizinho_ms) > AUTONOMO_DELAY_MS &&
            g_fsm_state != STATE_SAFE_MODE &&
            g_fsm_state != STATE_LIGHT_ON  &&
            g_fsm_state != STATE_OBSTACULO &&
            g_fsm_state != STATE_AUTONOMO) {
            g_fsm_state = STATE_AUTONOMO;
            ESP_LOGW(TAG, "AUTONOMO — sem vizinhos há %lus",
                     (unsigned long)(AUTONOMO_DELAY_MS / 1000));
        }
    } else {
        g_fsm_sem_vizinho_ms = 0;
    }

    /* ── Sai de AUTONOMO: WiFi volta + vizinhos + esq sem FAIL── */
    if (g_fsm_state == STATE_AUTONOMO && comm_ok &&
        tem_vizinho && !g_fsm_left_radar_fail) {
        g_fsm_sem_vizinho_ms = 0;
        g_fsm_era_autonomo   = false;
        g_fsm_state = is_master ? STATE_MASTER : STATE_IDLE;
        ESP_LOGI(TAG, "Saiu de AUTONOMO → %s",
                 is_master ? "MASTER" : "IDLE");
    }

    (void)is_master;
}
