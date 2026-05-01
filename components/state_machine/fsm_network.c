/* ============================================================
   MÓDULO     : fsm_network
   FICHEIRO   : fsm_network.c — Gestão de rede da FSM
   VERSÃO     : 2.8  |  2026-05-01
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   - Gestão de conectividade UDP (vizinhos esquerdo/direito)
   - Lógica de eleição de MASTER temporário
   - Transições para estados degradados (SAFE_MODE, AUTONOMO)
   - Inteligência topológica dinâmica (abstração de N postes)

   LÓGICA DE ESTADOS DEGRADADOS (v2.7):
   ──────────────────────────────────────
   SAFE_MODE  → radar em FALHA física (independente de WiFi/vizinhos)
   AUTONOMO   → radar OK + WiFi OK + sem vizinhos conhecidos na rede
                Opera localmente: detecta e acende, não propaga TC_INC
   MASTER     → radar OK + WiFi OK + vizinhos OK + primeiro da cadeia
   IDLE       → radar OK + WiFi OK + vizinhos OK + não é primeiro

   CORRECÇÕES v2.6 → v2.8:
   ─────────────────────────
   - CORRIGIDO v2.7: AUTONOMO agora activa quando radar OK + WiFi OK
     mas sem vizinhos descobertos na rede.
   - CORRIGIDO v2.8: SAFE_MODE não activa quando há veículos activos
     (T>0 ou Tc>0) — tráfego activo prova que o radar está funcional
     mesmo que o contador de falhas tenha atingido o limite.
============================================================ */

#include "fsm_network.h"
#include "fsm_core.h"
#include "fsm_events.h"
#include "comm_manager.h"
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

    /* Detecção de queda do vizinho esquerdo */
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
   ──────────────────────────────────────────────────────────
   Prioridade das transições (ordem decrescente):

     1. SAFE_MODE  — radar falhou → luz a 50%, ignora tudo o resto
     2. AUTONOMO   — radar OK + WiFi OK + sem vizinhos conhecidos
                     Opera localmente sem coordenação UDP
     3. MASTER/IDLE — rede completa, coordenação normal

   NOTA: AUTONOMO não é uma degradação — é o estado normal de
   um poste isolado ou único em teste. Quando vizinhos aparecem,
   o poste transita automaticamente para MASTER ou IDLE.
============================================================ */
void fsm_network_estados_degradados(bool comm_ok, bool is_master)
{
    /* ── 1. SAFE MODE — falha física do radar ──────────────────
       Condição exclusiva: g_fsm_radar_ok = false.
       Não interessa WiFi nem vizinhos.
       Luz fica a LIGHT_SAFE_MODE (50%) via fsm_aplicar_luz().

       CORRECÇÃO v2.8: Se há veículos activos (T>0 ou Tc>0),
       o radar está claramente a funcionar — não entra em SAFE_MODE.
       O HLK-LD2450 pode perder alvos estáticos temporariamente
       mas o tracking confirma que o sensor está operacional.
    ────────────────────────────────────────────────────────── */
    if (!g_fsm_radar_ok) {
        /* Se há tráfego activo, o radar está a funcionar — ignora */
        if (g_fsm_T > 0 || g_fsm_Tc > 0) return;

        if (g_fsm_state != STATE_SAFE_MODE) {
            g_fsm_state = STATE_SAFE_MODE;
            /* dali_safe_mode() NÃO é chamado aqui — responsabilidade
               de fsm_aplicar_luz() em fsm_task.c no ciclo seguinte. */
            ESP_LOGE(TAG, "SAFE MODE: Falha crítica no Radar.");
        }
        return; /* Não avalia mais nada enquanto radar falhar */
    }

    /* Recuperação de SAFE MODE quando radar volta */
    if (g_fsm_state == STATE_SAFE_MODE) {
        /* Sai para AUTONOMO — vizinhos são avaliados no próximo bloco */
        g_fsm_state = STATE_AUTONOMO;
        g_fsm_sem_vizinho_ms = 0;
        ESP_LOGI(TAG, "Recuperação SAFE MODE → AUTONOMO (radar OK)");
    }

    /* ── 2. AUTONOMO — radar OK + sem vizinhos conhecidos ──────
       AUTONOMO é o estado normal quando:
         - Poste único em teste (sem outros postes na rede)
         - Primeiro arranque antes dos vizinhos responderem
         - WiFi ligado mas sem outros postes UDP na rede

       Saída de AUTONOMO: quando pelo menos um vizinho é descoberto.
       Entrada em AUTONOMO: quando não há vizinhos conhecidos.

       NOTA: WiFi em falha real (comm_ok=false) também mantém
       AUTONOMO — o poste continua a detectar e acender localmente.
    ────────────────────────────────────────────────────────── */
    bool tem_vizinho_esq = comm_left_known();
    bool tem_vizinho_dir = comm_right_known();
    bool tem_vizinhos    = tem_vizinho_esq || tem_vizinho_dir;

    if (!tem_vizinhos) {
        /* Sem vizinhos conhecidos → AUTONOMO (radar OK garantido aqui) */
        if (g_fsm_state != STATE_AUTONOMO  &&
            g_fsm_state != STATE_LIGHT_ON  &&
            g_fsm_state != STATE_OBSTACULO) {
            g_fsm_state = STATE_AUTONOMO;
            ESP_LOGI(TAG, "AUTONOMO: Radar OK, WiFi OK, sem vizinhos na rede.");
        }
        return;
    }

    /* ── 3. Vizinhos presentes — avalia saúde da rede ──────────
       Chegamos aqui apenas quando há pelo menos um vizinho
       conhecido. Avalia se estão operacionais.
    ────────────────────────────────────────────────────────── */

    /* Saída de AUTONOMO quando vizinhos aparecem */
    if (g_fsm_state == STATE_AUTONOMO) {
        g_fsm_state = is_master ? STATE_MASTER : STATE_IDLE;
        g_fsm_sem_vizinho_ms = 0;
        ESP_LOGI(TAG, "Vizinhos descobertos: saída de AUTONOMO → %s",
                 state_machine_get_state_name());
        return;
    }

    /* Vizinho direito conhecido mas não operacional → timeout → AUTONOMO */
    bool vizinho_dir_operacional = comm_right_online();
    bool falta_vizinho_dir = tem_vizinho_dir && !vizinho_dir_operacional;

    if (falta_vizinho_dir || !comm_ok) {
        uint64_t agora = fsm_agora_ms();
        if (g_fsm_sem_vizinho_ms == 0) {
            g_fsm_sem_vizinho_ms = agora;
            ESP_LOGW(TAG, "Vizinho dir. não responde — timeout em %llus",
                     (unsigned long long)(AUTONOMO_DELAY_MS / 1000ULL));
        }
        if ((agora - g_fsm_sem_vizinho_ms) > AUTONOMO_DELAY_MS) {
            if (g_fsm_state != STATE_AUTONOMO  &&
                g_fsm_state != STATE_LIGHT_ON  &&
                g_fsm_state != STATE_OBSTACULO) {
                g_fsm_state = STATE_AUTONOMO;
                ESP_LOGW(TAG, "AUTONOMO: Vizinho dir. sem resposta após %llus.",
                         (unsigned long long)(AUTONOMO_DELAY_MS / 1000ULL));
            }
        }
    } else {
        /* Rede estável — reset do timer de vizinho em falta */
        if (g_fsm_sem_vizinho_ms != 0) {
            g_fsm_sem_vizinho_ms = 0;
        }
    }
}
