/* ============================================================
   COMM MANAGER — IMPLEMENTAÇÃO
   @file      comm_manager.c
   @version   3.0  |  2026-04-29
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   Camada de abstracção sobre o udp_manager.
   Resolve IPs de vizinhos, calcula ETA e chama as funções
   de envio correctas sem expor detalhes de protocolo à FSM.

   ALTERAÇÕES v2.1 → v3.0:
   ─────────────────────────
   1. _ip_vizinho_direito() — salta vizinhos em SAFE_MODE ou FAIL.
      Se o vizinho direito está em SAFE, não lhe envia TC_INC
      (ele não tem radar, não pode processar o veículo).

   2. comm_is_master() — SAFE e FAIL contam como "ausente".
      Se o vizinho esquerdo está em SAFE_MODE, este poste
      assume MASTER temporariamente (vizinho esq. está cego).

   3. comm_status_ok() — SAFE no vizinho não conta como "online"
      para efeitos de verificação de conectividade.

   4. comm_notify_prev_passed() — usa udp_manager_send_passed()
      dedicado em vez de TC_INC com velocidade negativa (legado).
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
   ──────────────────────────────────────────────────────────
   Retorna true se o vizinho está online E operacional.
   SAFE_MODE e FAIL não contam como operacional — o poste
   existe na rede mas não consegue processar veículos.
============================================================ */
static bool _vizinho_operacional(neighbor_t *v)
{
    if (!v || !v->active) return false;
    return (v->status == NEIGHBOR_OK || v->status == NEIGHBOR_OBSTACULO);
}


/* ============================================================
   _ip_vizinho_direito
   ──────────────────────────────────────────────────────────
   Retorna IP do vizinho direito APENAS se estiver operacional.
   Vizinho em SAFE_MODE ou FAIL → retorna NULL.
   Não enviamos TC_INC a um poste sem radar — ele não consegue
   processar nem propagar o veículo.
============================================================ */
static const char *_ip_vizinho_direito(void)
{
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION + 1);
    if (!_vizinho_operacional(v)) return NULL;
    return v->ip;
}


/* ============================================================
   _ip_vizinho_esquerdo
   ──────────────────────────────────────────────────────────
   Retorna IP do vizinho esquerdo se estiver activo e não offline.
   Para PASSED: enviamos mesmo que esteja em SAFE (para T-- funcionar).
   O poste em SAFE ainda processa callbacks UDP — só o radar é que falha.
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
   ETA = distância restante / velocidade
   Distância restante = POSTE_DIST_M - RADAR_DETECT_M (metros)
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
        ESP_LOGI(TAG, "Comm iniciado — UDP v3.0 activo");
    else
        ESP_LOGE(TAG, "Falha ao iniciar UDP");
    return s_iniciado;
}


/* ============================================================
   comm_status_ok
   ──────────────────────────────────────────────────────────
   True se UDP operacional E pelo menos um vizinho operacional.
   Vizinhos em SAFE ou FAIL não contam — não propagam veículos.
============================================================ */
bool comm_status_ok(void)
{
    if (s_iniciado && udp_manager_get_socket() < 0) {
        s_iniciado = false;
        ESP_LOGW(TAG, "Socket UDP inválido — comm offline");
    }
    if (!s_iniciado) return false;

    /* Período de graça após arranque */
    static uint64_t s_init_ms = 0;
    if (s_init_ms == 0)
        s_init_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);

    uint64_t agora = (uint64_t)(esp_timer_get_time() / 1000ULL);
    if ((agora - s_init_ms) < (uint64_t)AUTONOMO_DELAY_MS)
        return true;

    /* ── CORRECÇÃO: poste isolado (fim de linha ou único) ── */
    /* Se não há vizinho teórico configurado à esquerda nem à direita,
       este poste opera sozinho — comm_ok deve ser true se UDP activo. */
    bool tem_vizinho_teorico_esq = (POST_POSITION > 0);
    bool tem_vizinho_teorico_dir = false; /* descoberto dinamicamente */

    /* Verifica se algum vizinho foi alguma vez descoberto */
    neighbor_t *viz_esq = udp_manager_get_neighbor_by_pos(POST_POSITION - 1);
    neighbor_t *viz_dir = udp_manager_get_neighbor_by_pos(POST_POSITION + 1);

    bool algum_conhecido = (viz_esq && viz_esq->active) ||
                           (viz_dir && viz_dir->active);

    /* Se nunca descobriu vizinhos E a posição indica que pode estar isolado,
       considera comm_ok — o UDP está activo mesmo sem vizinhos.
       Isto evita que um poste único em teste entre em AUTONOMO desnecessariamente. */
    if (!algum_conhecido && !tem_vizinho_teorico_esq) {
        /* POST_POSITION == 0 sem vizinhos descobertos = poste único ou MASTER isolado */
        return true;
    }

    bool tem_operacional = _vizinho_operacional(viz_esq) ||
                           _vizinho_operacional(viz_dir);

    if (!tem_operacional && algum_conhecido)
        ESP_LOGD(TAG, "comm_status_ok: vizinhos conhecidos mas não operacionais");

    return tem_operacional || !algum_conhecido;
}

/* ============================================================
   comm_is_master
   ──────────────────────────────────────────────────────────
   True se este poste é ou deve ser MASTER.
   Condições:
     - Posição 0 → sempre MASTER
     - Vizinho esquerdo OFFLINE → assume MASTER
     - Vizinho esquerdo em SAFE_MODE → assume MASTER
       (vizinho esq. está cego, não pode ser líder efectivo)
     - Vizinho esquerdo em FAIL → assume MASTER
============================================================ */
bool comm_is_master(void)
{
    if (POST_POSITION == 0) return true;
    if (!s_iniciado) return false;

    neighbor_t *viz_esq = udp_manager_get_neighbor_by_pos(POST_POSITION - 1);

    if (!viz_esq || !viz_esq->active) return false;

    /* Assume MASTER se vizinho esq. está ausente ou não operacional */
    if (viz_esq->status == NEIGHBOR_OFFLINE ||
        viz_esq->status == NEIGHBOR_SAFE    ||
        viz_esq->status == NEIGHBOR_FAIL) {
        return true;
    }

    return false;
}


/* ============================================================
   comm_left_online — vizinho esquerdo online e operacional
============================================================ */
bool comm_left_online(void)
{
    if (POST_POSITION == 0) return false;
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION - 1);
    return _vizinho_operacional(v);
}


/* ============================================================
   comm_right_online — vizinho direito online e operacional
============================================================ */
bool comm_right_online(void)
{
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION + 1);
    return _vizinho_operacional(v);
}


/* ============================================================
   comm_left_known — vizinho esquerdo foi descoberto (activo)
============================================================ */
bool comm_left_known(void)
{
    if (POST_POSITION == 0) return false;
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION - 1);
    return (v != NULL && v->active);
}


/* ============================================================
   comm_right_known — vizinho direito foi descoberto (activo)
============================================================ */
bool comm_right_known(void)
{
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION + 1);
    return (v != NULL && v->active);
}


/* ============================================================
   comm_send_tc_inc — envia TC_INC ao vizinho direito
   Só envia se o vizinho direito estiver operacional.
   Vizinho em SAFE_MODE → NULL → sem envio.
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
   comm_send_spd — calcula ETA e envia SPD ao vizinho direito
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
   comm_notify_prev_passed — notifica vizinho esquerdo (T--)
   ──────────────────────────────────────────────────────────
   Usa a mensagem PASSED dedicada (v5.0).
   Enviamos mesmo que o vizinho esquerdo esteja em SAFE_MODE
   — ele ainda processa callbacks UDP e precisa de fazer T--.
============================================================ */
void comm_notify_prev_passed(float speed)
{
    const char *ip = _ip_vizinho_esquerdo();
    if (!ip) return;
    udp_manager_send_passed(ip, speed);
    ESP_LOGD(TAG, "PASSED → %s", ip);
}


/* ============================================================
   comm_send_master_claim — propaga liderança ao vizinho direito
============================================================ */
void comm_send_master_claim(void)
{
    const char *ip = _ip_vizinho_direito();
    if (!ip) return;
    udp_manager_send_master_claim(ip);
    ESP_LOGI(TAG, "MASTER_CLAIM → %s", ip);
}
