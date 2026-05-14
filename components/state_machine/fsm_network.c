/* ============================================================
   MÓDULO     : fsm_network
   FICHEIRO   : fsm_network.c — Gestão de rede da FSM
   VERSÃO     : 3.1  |  2026-05-08
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   - Gestão de conectividade UDP (vizinhos esquerdo/direito)
   - Eleição e propagação de MASTER em cadeia (relay completo)
   - Failover a meio da linha: sub-segmento mantém liderança local
   - Transições para estados degradados (SAFE_MODE, AUTONOMO)

   ALTERAÇÕES v3.0 → v3.1:
   ─────────────────────────
   BUG 6 CORRIGIDO: Promoção indevida em topologia A←B(off)←C.
   
   PROBLEMA:
     Cenário: A (master pos=0) ← B (pos=1 offline) ← C (pos=2)
     C detectava B offline e promovia-se a MASTER após 5s,
     MESMO recebendo MASTER_CLAIM de A via broadcast/relay.
     RESULTADO: 2 masters simultâneos (A e C)!
   
   SOLUÇÃO:
     Antes de promover a MASTER temporário, verifica se:
     - Recebeu MASTER_CLAIM nos últimos MASTER_CLAIM_TIMEOUT_MS
     - O master conhecido tem ID MENOR que este poste
     - Se SIM → NÃO promove (master activo continua a coordenar)
   
   IMPACTO:
     Cenário A←B(off)←C:
       - C detecta B offline
       - C aguarda AUTONOMO_DELAY (5s)
       - C verifica: recebeu CLAIM de A há 2s (< 15s timeout)
       - C verifica: A(id=1) < C(id=3)
       - C NÃO se promove ✅
       - A continua único master ✅
     
     Cenário A-B-C, A falha:
       - C detecta B offline
       - C aguarda 5s
       - C verifica: último CLAIM foi há 20s (> 15s timeout)
       - C promove-se a MASTER ✅
       - Sub-segmento C-D-E funciona coordenado ✅

   CORRECÇÕES v2.9 → v3.0 (mantidas):
   ───────────────────────────────────
   BUG 1: MASTER_CLAIM relay completo
   BUG 2: Notificação imediata ao promover
   BUG 3: Guarda alargada para promoção
   BUG 4: AUTONOMO não sobrepõe MASTER
   BUG 5: Propagação do ID real na recuperação
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

/* ── Tracking do MASTER actual ──────────────────────────── */
static int      s_master_id_conhecido   = 0;     /* ID do master */
static uint64_t s_master_claim_last_ms  = 0;     /* Timestamp último CLAIM */

/* ── Tracking do estado WiFi (safe mode) ────────────────── */
static bool     s_wifi_was_disabled     = false; /* WiFi foi desligado? */

/* ============================================================
   fsm_network_master_claim_relay
   ──────────────────────────────────────────────────────────
   FIX BUG 1 + BUG 5 + BUG 6 (v3.1).
   Chamado por on_master_claim_received_ext() em fsm_events.c.

   1. Regista o ID do MASTER real
   2. Regista timestamp da recepção (NOVO v3.1)
   3. Se era MASTER temporário → cede → STATE_IDLE
   4. Relay: se este poste não é o final, envia MASTER_CLAIM
      preservando master_id ao longo de toda a cadeia
============================================================ */
void fsm_network_master_claim_relay(int from_id, int master_id)
{
    uint64_t agora = fsm_agora_ms();
    
    ESP_LOGI(TAG, "[MASTER_CLAIM] relay: from=%d master=%d | estado=%s",
             from_id, master_id, state_machine_get_state_name());

    /* ── Actualiza tracking (NOVO v3.1) ────────────────────── */
    s_master_id_conhecido  = master_id;
    s_master_claim_last_ms = agora;

    /* Cede MASTER se éramos temporários */
    if (g_fsm_state == STATE_MASTER && POST_POSITION > 0) {
        g_fsm_left_was_offline = false;
        g_fsm_left_offline_ms  = 0;
        g_fsm_state            = STATE_IDLE;
        ESP_LOGI(TAG, "[MASTER_CLAIM] Cedemos MASTER → IDLE (master real id=%d)",
                 master_id);
    }

    /* Relay ao vizinho direito preservando master_id */
    if (comm_right_known()) {
        comm_send_master_claim_id(master_id);
        ESP_LOGD(TAG, "[MASTER_CLAIM] Relay → vizinho direito (master=%d)", master_id);
    }
}


/* ============================================================
   fsm_network_vizinhos — Passos 3 e 4
   ──────────────────────────────────────────────────────────
   Passo 3: Vizinho direito — detecta online/offline, limpa Tc.
   Passo 4: Vizinho esquerdo:
     Queda   → inicia timer para promoção a MASTER (FIX BUG 2/3)
     Retorno → cede MASTER + propaga ID real (FIX BUG 5)
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

    /* Detecção de queda */
    if (!esq_ok && esq_conhecido && !g_fsm_left_was_offline) {
        g_fsm_left_was_offline = true;
        g_fsm_left_offline_ms  = agora;
        ESP_LOGW(TAG, "[REDE] Vizinho esq. offline — MASTER em %llus",
                 (unsigned long long)(AUTONOMO_DELAY_MS / 1000ULL));
    }

    /* Recuperação */
    if (esq_ok && g_fsm_left_was_offline) {
        g_fsm_left_was_offline = false;
        g_fsm_left_offline_ms  = 0;

        if (g_fsm_state == STATE_MASTER && POST_POSITION > 0) {
            g_fsm_state = STATE_IDLE;
            ESP_LOGI(TAG, "[REDE] Viz.esq. voltou → cedemos MASTER → IDLE");

            /* FIX BUG 5: propaga o MASTER real, não este poste */
            int master_real = (s_master_id_conhecido > 0) ? s_master_id_conhecido : 0;
            if (comm_right_known()) {
                comm_send_master_claim_id(master_real);
                ESP_LOGI(TAG, "[REDE] MASTER_CLAIM(id=%d) → cadeia direita",
                         master_real);
            }
        } else {
            ESP_LOGI(TAG, "[REDE] Vizinho esq. voltou online");
        }
    }

    /* Vizinho deixou de ser conhecido (reset de rede) */
    if (!esq_conhecido && g_fsm_left_was_offline) {
        g_fsm_left_was_offline = false;
        g_fsm_left_offline_ms  = 0;
    }
}


/* ============================================================
   fsm_network_master — Passo 10
   ──────────────────────────────────────────────────────────
   FIX BUG 2: ao ser promovido envia MASTER_CLAIM imediato.
   FIX BUG 3: guarda alargada — aceita LIGHT_ON e outros.
   FIX BUG 6 (v3.1): verifica MASTER_CLAIM antes de promover.
============================================================ */
void fsm_network_master(bool comm_ok, bool is_master)
{
    (void)comm_ok;
    uint64_t agora = fsm_agora_ms();

    /* Sincronização com o papel atribuído pelo comm_manager */
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

    /* ── Promoção a MASTER temporário ─────────────────────────
       FIX BUG 3: pode_promover aceita LIGHT_ON, AUTONOMO, etc.
       FIX BUG 6 (v3.1): verifica MASTER_CLAIM antes de promover.
    ─────────────────────────────────────────────────────────── */
    bool pode_promover = (g_fsm_state != STATE_MASTER    &&
                          g_fsm_state != STATE_SAFE_MODE &&
                          g_fsm_state != STATE_OBSTACULO);

    if (!is_master &&
        g_fsm_left_was_offline &&
        g_fsm_radar_ok &&
        pode_promover &&
        (agora - g_fsm_left_offline_ms) > AUTONOMO_DELAY_MS) {

        /* ── VERIFICAÇÃO NOVA v3.1 ──────────────────────────────
           Antes de promover, verifica se há um MASTER com ID
           MENOR que ainda está activo (enviando MASTER_CLAIM).
           
           CENÁRIO CRÍTICO: A (id=1) ← B (id=2 off) ← C (id=3)
           - C detecta B offline
           - C aguarda AUTONOMO_DELAY (5s)
           - MAS A continua a enviar MASTER_CLAIM via broadcast!
           - C verifica: último CLAIM há 2s (< 15s timeout)
           - C verifica: A(id=1) < C(id=3)
           - C NÃO se promove ✅
           
           CENÁRIO VÁLIDO: A-B-C, A falha
           - C detecta B offline
           - C aguarda 5s
           - Último CLAIM foi há 20s (> 15s timeout)
           - C promove-se a MASTER ✅
        ──────────────────────────────────────────────────────── */
        bool master_menor_existe = false;

        if (s_master_claim_last_ms > 0 &&
            (agora - s_master_claim_last_ms) < MASTER_CLAIM_TIMEOUT_MS) {
            
            /* Master conhecido ainda está activo */
            if (s_master_id_conhecido > 0 && s_master_id_conhecido < POSTE_ID) {
                master_menor_existe = true;
                ESP_LOGI(TAG, "[MASTER] Master id=%d activo — NÃO promovo (nós id=%d)",
                         s_master_id_conhecido, POSTE_ID);
            }
        }

        /* Só promove se NÃO há master menor activo */
        if (!master_menor_existe) {
            g_fsm_state           = STATE_MASTER;
            s_master_id_conhecido = POSTE_ID;

            ESP_LOGW(TAG, "[MASTER] MASTER temporário — viz.esq. offline há %llus",
                     (unsigned long long)((agora - g_fsm_left_offline_ms) / 1000ULL));

            /* FIX BUG 2: notifica sub-segmento IMEDIATAMENTE */
            if (comm_right_known()) {
                comm_send_master_claim_id(POSTE_ID);
                ESP_LOGI(TAG, "[MASTER] MASTER_CLAIM(id=%d) → cadeia direita (imediato)",
                         POSTE_ID);
            }

            g_fsm_master_claim_ms = agora;
        }
    }
}


/* ============================================================
   fsm_network_estados_degradados — Passo 11
   ──────────────────────────────────────────────────────────
   Prioridade: SAFE_MODE > AUTONOMO > MASTER/IDLE

   FIX BUG 4: AUTONOMO não é activado se g_fsm_state == STATE_MASTER.
============================================================ */
void fsm_network_estados_degradados(bool comm_ok, bool is_master)
{
            /* ── 1. SAFE MODE — falha física do radar  */

        if (!g_fsm_radar_ok) {
            if (g_fsm_state != STATE_SAFE_MODE) {
                g_fsm_state = STATE_SAFE_MODE;
                ESP_LOGW(TAG, "[REDE] → SAFE_MODE (radar offline)");
                
                /* CRÍTICO: Desliga WiFi para isolamento seguro */
                if (wifi_manager_is_enabled() && !s_wifi_was_disabled) {
                    ESP_LOGW(TAG, "[SAFE_MODE] 🔴 Desligando WiFi (radar sem detecção)");
                    ESP_LOGW(TAG, "[SAFE_MODE] Razão: Evitar propagação de TC sem veículos");
                    wifi_manager_disable();
                    s_wifi_was_disabled = true;
                }
            }
            return;
        }

        /* Saída de SAFE_MODE quando radar recupera */
        if (g_fsm_state == STATE_SAFE_MODE && g_fsm_radar_ok) {
            ESP_LOGI(TAG, "[REDE] Saída SAFE_MODE → radar recuperado");
            
            /* liga o  WiFi se foi desligado */
            if (s_wifi_was_disabled) {
                ESP_LOGI(TAG, "[SAFE_MODE] 🟢 Religando WiFi (radar operacional)");
                wifi_manager_enable();
                s_wifi_was_disabled = false;
            }
            
            /* Volta para IDLE (discovering) para reintegrar na rede */
            g_fsm_state = STATE_IDLE;
            
            ESP_LOGI(TAG, "[REDE] Estado: IDLE (aguarda discovery de vizinhos)");
            return;
        }

    /* ── 2. AUTONOMO — sem vizinhos conhecidos ─────────────────
       FIX BUG 4: não activar AUTONOMO se já somos STATE_MASTER.
    ─────────────────────────────────────────────────────────── */
    bool tem_vizinho_esq = comm_left_known();
    bool tem_vizinho_dir = comm_right_known();
    bool algum_conhecido  = tem_vizinho_esq || tem_vizinho_dir;

    if (!algum_conhecido) {
        /* Poste isolado */
        if (g_fsm_state != STATE_AUTONOMO  &&
            g_fsm_state != STATE_MASTER    &&   /* FIX BUG 4 */
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

    /* ── 3. Vizinhos presentes ──────────────────────────────── */

    /* Saída de AUTONOMO quando vizinhos aparecem */
    if (g_fsm_state == STATE_AUTONOMO) {
        g_fsm_state          = is_master ? STATE_MASTER : STATE_IDLE;
        g_fsm_sem_vizinho_ms = 0;
        ESP_LOGI(TAG, "[REDE] Saída AUTONOMO → %s (vizinhos descobertos)",
                 state_machine_get_state_name());
        return;
    }

    /* Viz.dir. em falha mas ainda há viz.esq. ou somos MASTER:
       não vai para AUTONOMO — ainda pertence à cadeia.         */
    bool dir_operacional = comm_right_online();
    bool esq_operacional = comm_left_online();
    bool falta_dir       = tem_vizinho_dir && !dir_operacional;

    if (falta_dir && !esq_operacional && !is_master && !comm_ok) {
        /* Completamente isolado — inicia timeout para AUTONOMO */
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
                g_fsm_state != STATE_MASTER) {  /* FIX BUG 4 */
                g_fsm_state = STATE_AUTONOMO;
                ESP_LOGW(TAG, "[REDE] → AUTONOMO: todos os vizinhos sem resposta");
            }
        }
    } else {
        if (g_fsm_sem_vizinho_ms != 0)
            g_fsm_sem_vizinho_ms = 0;
    }
}


/* ============================================================
   fsm_network_get_master_id
============================================================ */
int fsm_network_get_master_id(void)
{
    return s_master_id_conhecido;
}
