/* ============================================================
   COMM MANAGER — IMPLEMENTAÇÃO
   @file      comm_manager.c
   @version   3.1  |  2026-05-02
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   Camada de abstracção sobre o udp_manager.
   Resolve IPs de vizinhos, calcula ETA e chama as funções
   de envio correctas sem expor detalhes de protocolo à FSM.

   ALTERAÇÕES v3.0 → v3.1:
   ─────────────────────────
   - ADICIONADO: comm_send_master_claim_id(int master_id)
     Envia MASTER_CLAIM com ID do MASTER real para relay em cadeia.
     Chama udp_manager_send_master_claim_id() que usa o novo
     formato de protocolo "MASTER_CLAIM:<from_id>:<master_id>".
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
   relay — precisamos de enviar mesmo que o poste esteja em SAFE.
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
    return (uint32_t)((dist_m / speed_ms) * 1000.0f);
}


/* ============================================================
   comm_init
============================================================ */
bool comm_init(void)
{
    if (s_iniciado) return true;
    s_iniciado = udp_manager_init();
    if (s_iniciado)
        ESP_LOGI(TAG, "Comm iniciado — UDP v3.1 activo");
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
    ESP_LOGD(TAG, "TC_INC → %s | %.0f km/h | x=%dmm", ip, speed, (int)x_mm);
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
    ESP_LOGD(TAG, "SPD → %s | %.0f km/h | ETA=%lums",
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
   ──────────────────────────────────────────────────────────
   Anuncia que ESTE poste é MASTER ao vizinho direito.
   Usa o formato original "MASTER_CLAIM:<POSTE_ID>".
   Mantido para compatibilidade com código existente.
============================================================ */
void comm_send_master_claim(void)
{
    const char *ip = _ip_vizinho_direito_qualquer();
    if (!ip) return;
    udp_manager_send_master_claim(ip);
    ESP_LOGI(TAG, "MASTER_CLAIM → %s", ip);
}


/* ============================================================
   comm_send_master_claim_id  (NOVO v3.1)
   ──────────────────────────────────────────────────────────
   Relay de MASTER_CLAIM preservando o ID do MASTER original.
   Usa o novo formato "MASTER_CLAIM:<POSTE_ID>:<master_id>".

   Envia ao vizinho direito mesmo que esteja em SAFE_MODE
   (o relay tem de chegar a toda a cadeia independentemente
   do estado do vizinho, desde que esteja activo e não OFFLINE).

   @param master_id  ID do MASTER real (não muda ao longo dos relays)
============================================================ */
void comm_send_master_claim_id(int master_id)
{
    const char *ip = _ip_vizinho_direito_qualquer();
    if (!ip) return;
    udp_manager_send_master_claim_id(ip, master_id);
    ESP_LOGI(TAG, "MASTER_CLAIM(id=%d) → %s", master_id, ip);
}
