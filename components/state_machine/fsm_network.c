/* ============================================================
   MÓDULO     : fsm_network
   FICHEIRO   : fsm_network.c — Gestão de rede da FSM
   VERSÃO     : 2.5  |  2026-04-29
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   - Gestão de conectividade UDP (vizinhos esquerdo/direito)
   - Lógica de eleição de MASTER temporário
   - Transições para estados degradados (SAFE_MODE, AUTONOMO)
   - Inteligência topológica dinâmica (abstração de N postes)
============================================================ */

#include "fsm_network.h"
#include "fsm_core.h"
#include "fsm_events.h"
#include "comm_manager.h"
#include "dali_manager.h"
#include "system_config.h"
#include "esp_log.h"
#include <inttypes.h>

static const char *TAG = "FSM_NET";

/* ============================================================
   fsm_network_vizinhos — Passos 3 e 4
   Gestão de conectividade dos vizinhos esquerdo e direito.
============================================================ */
void fsm_network_vizinhos(bool comm_ok, bool is_master)
{
    (void)comm_ok;
    (void)is_master;
    
    uint64_t agora = fsm_agora_ms();

    /* ── Passo 3: Vizinho direito ─────────────────────────── */
    bool dir_ok = comm_right_online();
    if (!dir_ok && g_fsm_right_online)  sm_on_right_neighbor_offline();
    if (dir_ok  && !g_fsm_right_online) sm_on_right_neighbor_online();

    /* ── Passo 4: Vizinho esquerdo ────────────────────────── */
    bool esq_ok        = comm_left_online();
    bool esq_conhecido = comm_left_known();

    /* Deteção de queda do vizinho esquerdo */
    if (!esq_ok && esq_conhecido && !g_fsm_left_was_offline) {
        g_fsm_left_was_offline = true;
        g_fsm_left_offline_ms  = agora;
        ESP_LOGW(TAG, "Vizinho esq. ficou offline");
    }

    /* Recuperação do vizinho esquerdo */
    if (esq_ok && g_fsm_left_was_offline) {
        g_fsm_left_was_offline = false;
        
        /* Se éramos MASTER devido à queda do vizinho, cedemos o papel */
        if (g_fsm_state == STATE_MASTER && POST_POSITION > 0) {
            g_fsm_state = STATE_IDLE;
            comm_send_master_claim();
        }
        ESP_LOGI(TAG, "Vizinho esq. voltou online");
    }

    /* Caso o vizinho deixe de ser "conhecido" (ex: reset de rede) */
    if (!esq_conhecido && g_fsm_left_was_offline) {
        g_fsm_left_was_offline = false;
        g_fsm_left_offline_ms  = 0;
    }
}

/* ============================================================
   fsm_network_master — Passo 10
   Gestão do papel MASTER na cadeia.
============================================================ */
void fsm_network_master(bool comm_ok, bool is_master)
{
    (void)comm_ok;
    uint64_t agora = fsm_agora_ms();

    /* Sincronização com o papel atribuído pelo comm_manager */
    if (is_master && g_fsm_state == STATE_IDLE) {
        g_fsm_state = STATE_MASTER;
    } 
    else if (!is_master && g_fsm_state == STATE_MASTER) {
        /* Só cede se não for MASTER forçado por ausência de vizinho */
        if (!g_fsm_left_was_offline) {
            g_fsm_state = STATE_IDLE;
            ESP_LOGI(TAG, "MASTER cedido");
        }
    }

    /* Promoção a MASTER temporário por timeout do vizinho esquerdo */
    if (!is_master && g_fsm_left_was_offline && g_fsm_radar_ok &&
        (agora - g_fsm_left_offline_ms) > AUTONOMO_DELAY_MS &&
        g_fsm_state == STATE_IDLE) {
        
        g_fsm_state = STATE_MASTER;
        ESP_LOGW(TAG, "MASTER temporário — viz. esq. offline prolongado");
    }
}

/* ============================================================
   fsm_network_estados_degradados — Passo 11
   Gestão de SAFE_MODE e AUTONOMO com inteligência topológica.
============================================================ */
void fsm_network_estados_degradados(bool comm_ok, bool is_master)
{
    uint64_t agora = fsm_agora_ms();

    /* ── 1. GESTÃO DO SAFE MODE (Falha Crítica de Hardware) ──── */
    if (!g_fsm_radar_ok) {
        if (g_fsm_state != STATE_SAFE_MODE) {
            g_fsm_state = STATE_SAFE_MODE;
            dali_set_brightness(LIGHT_SAFE_MODE);
            ESP_LOGE(TAG, "SAFE MODE: Falha crítica no Radar.");
        }
        return; 
    }

    /* Recuperação de SAFE MODE */
    if (g_fsm_state == STATE_SAFE_MODE && g_fsm_radar_ok) {
        g_fsm_state = is_master ? STATE_MASTER : STATE_IDLE;
        ESP_LOGI(TAG, "Recuperação: Radar OK. Saída para %s.", state_machine_get_state_name());
    }

    /* ── 2. GESTÃO DO MODO AUTÓNOMO (Lógica Dinâmica de Topologia) ─── */
    
    /**
     * @brief Inteligência de Fim de Linha:
     * Verificamos se o comm_manager conhece um vizinho à direita.
     * Se comm_left_known() for false, este poste é o fim da linha (N).
     * A ausência de vizinho direito só é erro se houver um vizinho configurado.
     */
    bool tem_vizinho_teorico_dir = comm_right_known();
    bool falta_vizinho_obrigatorio = (tem_vizinho_teorico_dir && !comm_right_online());

    /* CONDIÇÃO: Falha de WiFi OU perda de um vizinho que deveria existir */
    if (!comm_ok || falta_vizinho_obrigatorio) {
        
        if (!comm_ok) {
            /* Falha total de WiFi: todos os postes passam a modo isolado */
            if (g_fsm_state != STATE_AUTONOMO && g_fsm_state != STATE_LIGHT_ON && g_fsm_state != STATE_OBSTACULO) {
                g_fsm_state = STATE_AUTONOMO;
                ESP_LOGW(TAG, "Modo AUTÓNOMO: Falha na stack WiFi.");
            }
        } 
        else {
            /* Falha de vizinho: apenas para postes intermédios configurados */
            if (g_fsm_sem_vizinho_ms == 0) g_fsm_sem_vizinho_ms = agora;

            if ((agora - g_fsm_sem_vizinho_ms) > AUTONOMO_DELAY_MS) {
                if (g_fsm_state != STATE_AUTONOMO && g_fsm_state != STATE_LIGHT_ON && g_fsm_state != STATE_OBSTACULO) {
                    g_fsm_state = STATE_AUTONOMO;
                    ESP_LOGW(TAG, "Modo AUTÓNOMO: Vizinho direito configurado não responde.");
                }
            }
        }
    } 
    else {
        /* Reset do timer se a rede estabilizou ou se é o último poste (sem vizinho teórico) */
        g_fsm_sem_vizinho_ms = 0;
        
        if (g_fsm_state == STATE_AUTONOMO) {
            g_fsm_state = is_master ? STATE_MASTER : STATE_IDLE;
            ESP_LOGI(TAG, "Rede OK: Saída de modo AUTÓNOMO.");
        }
    }
}