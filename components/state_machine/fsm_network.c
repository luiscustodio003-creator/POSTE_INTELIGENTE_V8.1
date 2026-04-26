/* ============================================================
   MÓDULO     : fsm_network
   FICHEIRO   : fsm_network.c — Gestão de rede da FSM
   VERSÃO     : 1.0  |  2026-04-26
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Passos 3, 4, 10 e 11 do state_machine_update().
   Isola toda a lógica de conectividade UDP e transições
   de estado relacionadas com a rede.
============================================================ */

#include "fsm_network.h"
#include "fsm_core.h"
#include "fsm_events.h"
#include "comm_manager.h"
#include "dali_manager.h"
#include "system_config.h"
#include "esp_log.h"

static const char *TAG = "FSM_NET";


/* ============================================================
   fsm_network_vizinhos — Passos 3 e 4
   Gestão de conectividade dos vizinhos esquerdo e direito.
============================================================ */
void fsm_network_vizinhos(bool comm_ok, bool is_master)
{
    uint64_t agora = fsm_agora_ms();

    /* ── Passo 3: Vizinho direito ─────────────────────────── */
    bool dir_ok = comm_right_online();
    if (!dir_ok &&  g_fsm_right_online) sm_on_right_neighbor_offline();
    if ( dir_ok && !g_fsm_right_online) sm_on_right_neighbor_online();

    /* ── Passo 4: Vizinho esquerdo ────────────────────────── */
    bool esq_ok       = comm_left_online();
    bool esq_conhecido = comm_left_known();

    /* Só marca offline se o vizinho já foi descoberto antes.
       No arranque, vizinho desconhecido != vizinho offline.
       Evita assumir MASTER temporário por falsa ausência. */
    if (!esq_ok && esq_conhecido && !g_fsm_left_was_offline) {
        g_fsm_left_was_offline = true;
        g_fsm_left_offline_ms  = agora;
        ESP_LOGW(TAG, "Vizinho esq. ficou offline");
    }
    if (esq_ok && g_fsm_left_was_offline) {
        g_fsm_left_was_offline = false;
        if (g_fsm_state == STATE_MASTER && POST_POSITION > 0) {
            g_fsm_state = STATE_IDLE;
            comm_send_master_claim();
        }
        ESP_LOGI(TAG, "Vizinho esq. voltou online");
    }
    /* Vizinho nunca descoberto — garante flag limpa */
    if (!esq_conhecido && g_fsm_left_was_offline) {
        g_fsm_left_was_offline = false;
        g_fsm_left_offline_ms  = 0;
    }

    (void)comm_ok;
    (void)is_master;
}


/* ============================================================
   fsm_network_master — Passo 10
   Gestão do papel MASTER na cadeia.
============================================================ */
void fsm_network_master(bool comm_ok, bool is_master)
{
    uint64_t agora = fsm_agora_ms();

    /* Transições IDLE ↔ MASTER baseadas no papel actual */
    if ( is_master && g_fsm_state == STATE_IDLE)
        g_fsm_state = STATE_MASTER;

    if (!is_master && g_fsm_state == STATE_MASTER) {
        g_fsm_state = STATE_IDLE;
        ESP_LOGI(TAG, "MASTER cedido");
    }

    /* Vizinho esq. offline há AUTONOMO_DELAY_MS → assume MASTER temporário */
    if (!is_master && g_fsm_left_was_offline && g_fsm_radar_ok &&
        (agora - g_fsm_left_offline_ms) > AUTONOMO_DELAY_MS &&
        g_fsm_state == STATE_IDLE) {
        g_fsm_state = STATE_MASTER;
        ESP_LOGW(TAG, "MASTER temporário — viz. esq. offline");
    }

    /* Vizinho esq. voltou — cede MASTER se não for o poste 0 */
    if (!is_master && !g_fsm_left_was_offline &&
        g_fsm_state == STATE_MASTER && POST_POSITION > 0) {
        g_fsm_state = STATE_IDLE;
        ESP_LOGI(TAG, "Viz. esq. voltou — cede MASTER → IDLE");
    }

    (void)comm_ok;
}


/* ============================================================
   fsm_network_estados_degradados — Passo 11
   Gestão de SAFE_MODE e AUTONOMO.
============================================================ */
void fsm_network_estados_degradados(bool comm_ok, bool is_master)
{
    uint64_t agora        = fsm_agora_ms();
    bool tem_vizinho_esq  = comm_left_online();
    bool tem_vizinho      = tem_vizinho_esq || comm_right_online();

    /* SAFE MODE — radar KO independentemente do WiFi.
       Sem radar o poste está sempre cego — luz a 50%.
       Única excepção: OBSTACULO tem prioridade (luz máxima). */
    if (!g_fsm_radar_ok &&
        g_fsm_state != STATE_SAFE_MODE &&
        g_fsm_state != STATE_OBSTACULO) {
        g_fsm_era_autonomo = false;
        g_fsm_state = STATE_SAFE_MODE;
        dali_set_brightness(LIGHT_SAFE_MODE);
        ESP_LOGW(TAG, "SAFE MODE — radar KO (wifi=%d viz_esq=%d)",
                 (int)comm_ok, (int)tem_vizinho_esq);
    }

    /* Sai de SAFE MODE apenas quando radar recupera */
    if (g_fsm_state == STATE_SAFE_MODE && g_fsm_radar_ok) {
        g_fsm_state = STATE_IDLE;
        ESP_LOGI(TAG, "Saiu de SAFE MODE → IDLE (radar recuperado)");
    }

    /* AUTONOMO — radar OK + sem WiFi → imediato */
    if (!comm_ok && g_fsm_radar_ok &&
        g_fsm_state != STATE_SAFE_MODE &&
        g_fsm_state != STATE_LIGHT_ON  &&
        g_fsm_state != STATE_OBSTACULO) {
        g_fsm_state = STATE_AUTONOMO;
        ESP_LOGW(TAG, "AUTONOMO — sem WiFi");
    }

    /* AUTONOMO — radar OK + WiFi OK + sem vizinhos → após AUTONOMO_DELAY_MS */
    if (g_fsm_radar_ok && comm_ok && !tem_vizinho) {
        if (g_fsm_sem_vizinho_ms == 0) g_fsm_sem_vizinho_ms = agora;
        if ((agora - g_fsm_sem_vizinho_ms) > AUTONOMO_DELAY_MS &&
            g_fsm_state != STATE_SAFE_MODE &&
            g_fsm_state != STATE_LIGHT_ON  &&
            g_fsm_state != STATE_OBSTACULO) {
            g_fsm_state = STATE_AUTONOMO;
            ESP_LOGW(TAG, "AUTONOMO — sem vizinhos há %lus",
                     (unsigned long)(AUTONOMO_DELAY_MS / 1000));
        }
    } else {
        g_fsm_sem_vizinho_ms = 0;
    }

    /* Sai de AUTONOMO quando WiFi volta E tem vizinhos */
    if (g_fsm_state == STATE_AUTONOMO && comm_ok && tem_vizinho) {
        g_fsm_sem_vizinho_ms = 0;
        g_fsm_era_autonomo   = false;
        g_fsm_state = is_master ? STATE_MASTER : STATE_IDLE;
        ESP_LOGI(TAG, "Saiu de AUTONOMO → %s",
                 is_master ? "MASTER" : "IDLE");
    }
}
