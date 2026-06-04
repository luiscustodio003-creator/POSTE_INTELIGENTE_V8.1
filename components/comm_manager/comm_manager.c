/* ============================================================
   MÓDULO     : comm_manager
   FICHEIRO   : comm_manager.c — Camada de abstracção UDP para a FSM
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)
============================================================ */
#include "comm_manager.h"
#include "udp_manager.h"
#include "system_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "COMM_MGR";

static bool s_iniciado = false;


/* ============================================================
   _vizinho_operacional
============================================================ */
static bool _vizinho_operacional(neighbor_t *v)
{
    if (!v || !v->active) return false;
    return (v->status == NEIGHBOR_OK || v->status == NEIGHBOR_OBSTACULO);
}


/* ============================================================
   _ip_vizinho_direito
============================================================ */
static const char *_ip_vizinho_direito(void)
{
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION + 1);
    if (!_vizinho_operacional(v)) return NULL;
    return v->ip;
}


/* ============================================================
   _ip_vizinho_direito_qualquer
   ──────────────────────────────────────────────────────────
   Retorna IP do vizinho direito mesmo que não esteja operacional,
   desde que esteja activo (conhecido). Usado para MASTER_CLAIM
   relay e OBSTACULO — precisamos de enviar mesmo que o poste
   esteja em SAFE ou outro estado degradado.
============================================================ */
static const char *_ip_vizinho_direito_qualquer(void)
{
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION + 1);
    if (!v || !v->active || v->status == NEIGHBOR_OFFLINE) return NULL;
    return v->ip;
}


/* ============================================================
   _ip_vizinho_esquerdo
============================================================ */
static const char *_ip_vizinho_esquerdo(void)
{
    if (POST_POSITION == 0) return NULL;
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION - 1);
    if (!v || !v->active || v->status == NEIGHBOR_OFFLINE) return NULL;
    return v->ip;
}


/* ============================================================
   _calcular_eta_ms
============================================================ */
static uint32_t _calcular_eta_ms(float speed_kmh)
{
    if (speed_kmh < 1.0f) speed_kmh = 1.0f;
    float dist_m   = (float)(POSTE_DIST_M - RADAR_DETECT_M);
    float speed_ms = speed_kmh / 3.6f;
    uint32_t eta   = (uint32_t)((dist_m / speed_ms) * 1000.0f);
    /* Antecipa MARGEM_ACENDER_MS para que P(i+1) acenda ANTES da chegada.
       Protege contra underflow: se velocidade for muito alta, eta mínimo = 0. */
    return (eta > MARGEM_ACENDER_MS) ? (eta - MARGEM_ACENDER_MS) : 0;
}


/* ============================================================
   comm_init
============================================================ */
bool comm_init(void)
{
    if (s_iniciado) return true;
    s_iniciado = udp_manager_init();
    if (s_iniciado)
        ESP_LOGI(TAG, "Comm iniciado — UDP v3.2 activo (obstáculo corrigido)");
    else
        ESP_LOGE(TAG, "Falha ao iniciar UDP");
    return s_iniciado;
}


/* ============================================================
   comm_status_ok
============================================================ */
bool comm_status_ok(void)
{
    if (s_iniciado && udp_manager_get_socket() < 0) {
        s_iniciado = false;
        ESP_LOGW(TAG, "Socket UDP inválido — comm offline");
    }
    if (!s_iniciado) return false;

    static uint64_t s_init_ms = 0;
    if (s_init_ms == 0)
        s_init_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);

    uint64_t agora = (uint64_t)(esp_timer_get_time() / 1000ULL);
    if ((agora - s_init_ms) < (uint64_t)AUTONOMO_DELAY_MS)
        return true;

    bool tem_vizinho_teorico_esq = (POST_POSITION > 0);

    neighbor_t *viz_esq = udp_manager_get_neighbor_by_pos(POST_POSITION - 1);
    neighbor_t *viz_dir = udp_manager_get_neighbor_by_pos(POST_POSITION + 1);

    bool algum_conhecido = (viz_esq && viz_esq->active) ||
                           (viz_dir && viz_dir->active);

    if (!algum_conhecido && !tem_vizinho_teorico_esq)
        return true;

    bool tem_operacional = _vizinho_operacional(viz_esq) ||
                           _vizinho_operacional(viz_dir);

    if (!tem_operacional && algum_conhecido)
        ESP_LOGD(TAG, "comm_status_ok: vizinhos conhecidos mas não operacionais");

    return tem_operacional || !algum_conhecido;
}


/* ============================================================
   comm_is_master
============================================================ */
bool comm_is_master(void)
{
    if (POST_POSITION == 0) return true;
    if (!s_iniciado) return false;

    neighbor_t *viz_esq = udp_manager_get_neighbor_by_pos(POST_POSITION - 1);

    if (!viz_esq || !viz_esq->active) return true;

    if (viz_esq->status == NEIGHBOR_OFFLINE ||
        viz_esq->status == NEIGHBOR_SAFE) {
        return true;
    }

    return false;
}


/* ============================================================
   comm_left_online
============================================================ */
bool comm_left_online(void)
{
    if (POST_POSITION == 0) return false;
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION - 1);
    return _vizinho_operacional(v);
}


/* ============================================================
   comm_right_online
============================================================ */
bool comm_right_online(void)
{
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION + 1);
    return _vizinho_operacional(v);
}


/* ============================================================
   comm_left_known
============================================================ */
bool comm_left_known(void)
{
    if (POST_POSITION == 0) return false;
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION - 1);
    return (v != NULL && v->active);
}


/* ============================================================
   comm_right_known
============================================================ */
bool comm_right_known(void)
{
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION + 1);
    return (v != NULL && v->active);
}


/* ============================================================
   comm_send_tc_inc
============================================================ */
void comm_send_tc_inc(float speed, int16_t x_mm)
{
    const char *ip = _ip_vizinho_direito();
    if (!ip) {
        ESP_LOGD(TAG, "TC_INC: sem vizinho direito operacional");
        return;
    }
    udp_manager_send_tc_inc(ip, speed, x_mm);
    udp_manager_send_tc_inc(ip, speed, x_mm); /* double-send: dedup no receptor previne Tc duplo */
    ESP_LOGD(TAG, "TC_INC x2 → %s | %.0f km/h | x=%dmm", ip, speed, (int)x_mm);
}


/* ============================================================
   comm_send_spd
============================================================ */
void comm_send_spd(float speed, int16_t x_mm)
{
    const char *ip = _ip_vizinho_direito();
    if (!ip) {
        ESP_LOGD(TAG, "SPD: sem vizinho direito operacional");
        return;
    }
    uint32_t eta_ms = _calcular_eta_ms(speed);
    udp_manager_send_spd(ip, speed, eta_ms, POSTE_DIST_M, x_mm);
    udp_manager_send_spd(ip, speed, eta_ms, POSTE_DIST_M, x_mm); /* double-send: SPD é idempotente */
    ESP_LOGD(TAG, "SPD x2 → %s | %.0f km/h | ETA=%lums",
             ip, speed, (unsigned long)eta_ms);
}


/* ============================================================
   comm_notify_prev_passed
============================================================ */
void comm_notify_prev_passed(float speed)
{
    const char *ip = _ip_vizinho_esquerdo();
    if (!ip) return;
    udp_manager_send_passed(ip, speed);
    ESP_LOGD(TAG, "PASSED → %s", ip);
}


/* ============================================================
   comm_send_master_claim
============================================================ */
void comm_send_master_claim(void)
{
    const char *ip = _ip_vizinho_direito_qualquer();
    if (!ip) return;
    udp_manager_send_master_claim(ip);
    ESP_LOGI(TAG, "MASTER_CLAIM → %s", ip);
}


/* ============================================================
   comm_send_master_claim_id
============================================================ */
void comm_send_master_claim_id(int master_id)
{
    const char *ip = _ip_vizinho_direito_qualquer();
    if (!ip) return;
    udp_manager_send_master_claim_id(ip, master_id);
    ESP_LOGI(TAG, "MASTER_CLAIM(id=%d) → %s", master_id, ip);
}


/* ============================================================
   comm_send_master_claim_relay  (NOVO v3.3)
   ──────────────────────────────────────────────────────────
   Relay de MASTER_CLAIM preservando seq e hop vindos do receptor UDP.
   Distingue-se de comm_send_master_claim_id() que origina um novo claim
   com seq novo; esta função propaga um claim já existente na cadeia.
============================================================ */
void comm_send_master_claim_relay(int master_id, uint16_t seq, uint8_t hop)
{
    const char *ip = _ip_vizinho_direito_qualquer();
    if (!ip) return;
    udp_manager_send_master_claim_relay(ip, master_id, seq, hop);
    ESP_LOGD(TAG, "MASTER_CLAIM relay(id=%d seq=%u hop=%u) → %s",
             master_id, (unsigned)seq, (unsigned)hop, ip);
}


/* ============================================================
   comm_send_obstaculo  (NOVO v3.2)
   ──────────────────────────────────────────────────────────
   Envia notificação de obstáculo ao vizinho direito.

   QUANDO CHAMAR:
     - Em fsm_events.c, caso SM_EVT_VEHICLE_OBSTACULO
     - Só quando g_fsm_right_online == true

   RESOLUÇÃO AUTOMÁTICA:
     - Usa _ip_vizinho_direito_qualquer() para permitir envio
       mesmo que vizinho esteja em SAFE_MODE ou OBSTACULO.
     - Só falha se vizinho estiver OFFLINE ou não conhecido.

   EFEITO NO RECEPTOR:
     - Cancela TC_TIMEOUT (sabe que veículo parou)
     - Mantém Tc (veículo ainda presente na linha)
     - Luz fica acesa até receber PASSED real
============================================================ */
void comm_send_obstaculo(uint16_t vehicle_id, float speed, int16_t x_mm)
{
    const char *ip = _ip_vizinho_direito_qualquer();
    if (!ip) {
        ESP_LOGW(TAG, "OBSTACULO: sem vizinho direito conhecido");
        return;
    }
    
    udp_manager_send_obstaculo(ip, vehicle_id, speed, x_mm);
    
    ESP_LOGW(TAG, "OBSTACULO → %s | ID=%u vel=%.1f x=%d",
             ip, (unsigned int)vehicle_id, speed, (int)x_mm);
}
