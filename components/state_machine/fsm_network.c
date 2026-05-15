/* ============================================================
   MÓDULO     : fsm_network
   FICHEIRO   : fsm_network.c — Gestão de vizinhos, eleição de master, AUTONOMO, SAFE_MODE
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)
============================================================ */

#include "fsm_network.h"
#include "fsm_core.h"
#include "fsm_events.h"
#include "comm_manager.h"
#include "system_config.h"
#include "wifi_manager.h"
#include "esp_log.h"
#include <inttypes.h>

static const char *TAG = "FSM_NET";

static int      s_master_id_conhecido  = 0;
static uint64_t s_master_claim_last_ms = 0;
static bool     s_wifi_was_disabled    = false;
static uint64_t s_master_isolado_ms    = 0;


/* ── fsm_network_master_claim_relay ──────────────────────────
   Regista o master activo e propaga MASTER_CLAIM pela cadeia.
   Cede MASTER se este poste era temporário. */
void fsm_network_master_claim_relay(int from_id, int master_id)
{
    uint64_t agora = fsm_agora_ms();

    ESP_LOGI(TAG, "[MASTER_CLAIM] relay: from=%d master=%d | estado=%s",
             from_id, master_id, state_machine_get_state_name());

    s_master_id_conhecido  = master_id;
    s_master_claim_last_ms = agora;

    if (g_fsm_state == STATE_MASTER && POST_POSITION > 0) {
        g_fsm_left_was_offline = false;
        g_fsm_left_offline_ms  = 0;
        g_fsm_state            = STATE_IDLE;
        ESP_LOGI(TAG, "[MASTER_CLAIM] Cedemos MASTER → IDLE (master real id=%d)",
                 master_id);
    }

    if (comm_right_known()) {
        comm_send_master_claim_id(master_id);
        ESP_LOGD(TAG, "[MASTER_CLAIM] Relay → vizinho direito (master=%d)", master_id);
    }
}


/* ── fsm_network_vizinhos — Passos 3 e 4 ─────────────────── */
void fsm_network_vizinhos(bool comm_ok, bool is_master)
{
    (void)comm_ok;
    (void)is_master;

    uint64_t agora = fsm_agora_ms();

    bool dir_ok = comm_right_online();
    if (!dir_ok && g_fsm_right_online)  sm_on_right_neighbor_offline();
    if (dir_ok  && !g_fsm_right_online) sm_on_right_neighbor_online();

    bool esq_ok        = comm_left_online();
    bool esq_conhecido = comm_left_known();

    if (!esq_ok && esq_conhecido && !g_fsm_left_was_offline) {
        g_fsm_left_was_offline = true;
        g_fsm_left_offline_ms  = agora;
        ESP_LOGW(TAG, "[REDE] Vizinho esq. offline — MASTER em %llus",
                 (unsigned long long)(AUTONOMO_DELAY_MS / 1000ULL));
    }

    if (esq_ok && g_fsm_left_was_offline) {
        g_fsm_left_was_offline = false;
        g_fsm_left_offline_ms  = 0;

        if (g_fsm_state == STATE_MASTER && POST_POSITION > 0) {
            g_fsm_state = STATE_IDLE;
            ESP_LOGI(TAG, "[REDE] Viz.esq. voltou → cedemos MASTER → IDLE");

            int master_real = (s_master_id_conhecido > 0) ? s_master_id_conhecido : 0;
            if (comm_right_known()) {
                comm_send_master_claim_id(master_real);
                ESP_LOGI(TAG, "[REDE] MASTER_CLAIM(id=%d) → cadeia direita", master_real);
            }
        } else {
            ESP_LOGI(TAG, "[REDE] Vizinho esq. voltou online");
        }
    }

    if (!esq_conhecido && g_fsm_left_was_offline) {
        g_fsm_left_was_offline = false;
        g_fsm_left_offline_ms  = 0;
    }
}


/* ── fsm_network_master — Passo 10 ───────────────────────────
   Promoção a MASTER temporário só ocorre se não existe master
   com ID menor ainda activo (verifica MASTER_CLAIM_TIMEOUT). */
void fsm_network_master(bool comm_ok, bool is_master)
{
    (void)comm_ok;
    uint64_t agora = fsm_agora_ms();

    if (is_master && g_fsm_state == STATE_IDLE) {
        g_fsm_state           = STATE_MASTER;
        s_master_id_conhecido = POSTE_ID;
        ESP_LOGI(TAG, "[MASTER] Papel MASTER confirmado");
    }
    else if (!is_master && g_fsm_state == STATE_MASTER) {
        if (!g_fsm_left_was_offline) {
            g_fsm_state = STATE_IDLE;
            ESP_LOGI(TAG, "[MASTER] MASTER cedido (comm_manager)");
        }
    }

    bool pode_promover = (g_fsm_state != STATE_MASTER    &&
                          g_fsm_state != STATE_SAFE_MODE &&
                          g_fsm_state != STATE_OBSTACULO);

    if (!is_master &&
        g_fsm_left_was_offline &&
        g_fsm_radar_ok &&
        pode_promover &&
        (agora - g_fsm_left_offline_ms) > AUTONOMO_DELAY_MS) {

        /* Não promover se existe master com ID menor activo. */
        bool master_menor_existe = false;

        if (s_master_claim_last_ms > 0 &&
            (agora - s_master_claim_last_ms) < MASTER_CLAIM_TIMEOUT_MS) {
            if (s_master_id_conhecido > 0 && s_master_id_conhecido < POSTE_ID) {
                master_menor_existe = true;
                ESP_LOGI(TAG, "[MASTER] Master id=%d activo — NÃO promovo (nós id=%d)",
                         s_master_id_conhecido, POSTE_ID);
            }
        }

        if (!master_menor_existe) {
            g_fsm_state           = STATE_MASTER;
            s_master_id_conhecido = POSTE_ID;

            ESP_LOGW(TAG, "[MASTER] MASTER temporário — viz.esq. offline há %llus",
                     (unsigned long long)((agora - g_fsm_left_offline_ms) / 1000ULL));

            if (comm_right_known()) {
                comm_send_master_claim_id(POSTE_ID);
                ESP_LOGI(TAG, "[MASTER] MASTER_CLAIM(id=%d) → cadeia direita (imediato)",
                         POSTE_ID);
            }

            g_fsm_master_claim_ms = agora;
        }
    }
}


/* ── fsm_network_estados_degradados — Passo 11 ───────────────
   Prioridade: SAFE_MODE > AUTONOMO > MASTER/IDLE. */
void fsm_network_estados_degradados(bool comm_ok, bool is_master)
{
    /* 1. SAFE MODE — falha física do radar.
       WiFi desligado intencionalmente: sem radar não há TC_INC/SPD válidos.
       Os vizinhos vêem este poste como OFFLINE → linha divide-se correctamente
       em dois clusters independentes. Pi+1 promove a MASTER por não ter vizinho esq. */
    if (!g_fsm_radar_ok) {
        if (g_fsm_state != STATE_SAFE_MODE) {
            g_fsm_state = STATE_SAFE_MODE;
            ESP_LOGW(TAG, "[REDE] → SAFE_MODE (radar offline → WiFi off)");

            if (wifi_manager_is_enabled() && !s_wifi_was_disabled) {
                wifi_manager_disable();
                s_wifi_was_disabled = true;
            }
        }
        return;
    }

    if (g_fsm_state == STATE_SAFE_MODE && g_fsm_radar_ok) {
        ESP_LOGI(TAG, "[REDE] Saída SAFE_MODE → radar recuperado");

        if (s_wifi_was_disabled) {
            wifi_manager_enable();
            s_wifi_was_disabled = false;
        }

        g_fsm_state = STATE_IDLE;
        ESP_LOGI(TAG, "[REDE] Estado: IDLE (aguarda discovery de vizinhos)");
        return;
    }

    /* 2. AUTONOMO — sem vizinhos conhecidos */
    bool tem_vizinho_esq = comm_left_known();
    bool tem_vizinho_dir = comm_right_known();
    bool algum_conhecido  = tem_vizinho_esq || tem_vizinho_dir;

    if (!algum_conhecido) {
        if (g_fsm_state != STATE_AUTONOMO  &&
            g_fsm_state != STATE_MASTER    &&
            g_fsm_state != STATE_LIGHT_ON  &&
            g_fsm_state != STATE_OBSTACULO) {

            if (is_master) {
                g_fsm_state = STATE_MASTER;
                ESP_LOGI(TAG, "[REDE] MASTER isolado (sem vizinhos)");
            } else {
                g_fsm_state = STATE_AUTONOMO;
                ESP_LOGW(TAG, "[REDE] → AUTONOMO (sem vizinhos conhecidos)");
            }
        }
        return;
    }

    /* 3. Saída de AUTONOMO — só quando há vizinho operacional.
       Sem esta guarda, algum_conhecido=true (active nunca reseta) causaria
       um loop AUTONOMO → MASTER → AUTONOMO quando todos os vizinhos estão OFFLINE. */
    if (g_fsm_state == STATE_AUTONOMO) {
        bool algum_op = comm_right_online() || comm_left_online();
        if (algum_op) {
            g_fsm_state          = is_master ? STATE_MASTER : STATE_IDLE;
            g_fsm_sem_vizinho_ms = 0;
            ESP_LOGI(TAG, "[REDE] Saída AUTONOMO → %s (vizinho operacional)",
                     state_machine_get_state_name());
        }
        return;
    }

    bool dir_operacional = comm_right_online();
    bool esq_operacional = comm_left_online();
    bool falta_dir       = tem_vizinho_dir && !dir_operacional;

    if (falta_dir && !esq_operacional && !is_master && !comm_ok) {
        uint64_t agora = fsm_agora_ms();
        if (g_fsm_sem_vizinho_ms == 0) {
            g_fsm_sem_vizinho_ms = agora;
            ESP_LOGW(TAG, "[REDE] Todos vizinhos em falha — AUTONOMO em %llus",
                     (unsigned long long)(AUTONOMO_DELAY_MS / 1000ULL));
        }
        if ((agora - g_fsm_sem_vizinho_ms) > AUTONOMO_DELAY_MS) {
            if (g_fsm_state != STATE_AUTONOMO  &&
                g_fsm_state != STATE_LIGHT_ON  &&
                g_fsm_state != STATE_OBSTACULO &&
                g_fsm_state != STATE_MASTER) {
                g_fsm_state = STATE_AUTONOMO;
                ESP_LOGW(TAG, "[REDE] → AUTONOMO: todos os vizinhos sem resposta");
            }
        }
    } else {
        if (g_fsm_sem_vizinho_ms != 0)
            g_fsm_sem_vizinho_ms = 0;
    }

    /* MASTER isolado: POST_POSITION > 0 com todos os vizinhos conhecidos offline.
       Cobre cenários de dupla falha (Pi+Pj) e falha de WiFi pura.
       Timer separado de g_fsm_sem_vizinho_ms para não interferir com o caminho acima. */
    if (g_fsm_state == STATE_MASTER && POST_POSITION > 0 &&
        !dir_operacional && !esq_operacional) {
        uint64_t agora = fsm_agora_ms();
        if (s_master_isolado_ms == 0) {
            s_master_isolado_ms = agora;
            ESP_LOGW(TAG, "[MASTER] Isolado (todos vizinhos offline) — AUTONOMO em %llus",
                     (unsigned long long)(AUTONOMO_DELAY_MS / 1000ULL));
        }
        if ((agora - s_master_isolado_ms) > AUTONOMO_DELAY_MS) {
            g_fsm_state         = STATE_AUTONOMO;
            s_master_isolado_ms = 0;
            ESP_LOGW(TAG, "[MASTER] Isolado → AUTONOMO");
        }
    } else {
        s_master_isolado_ms = 0;
    }
}
