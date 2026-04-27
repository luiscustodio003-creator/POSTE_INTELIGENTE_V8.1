/* ============================================================
   COMM MANAGER — IMPLEMENTAÇÃO
   @file      comm_manager.c
   @version   4.0  |  2026-04-27
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno

   NOVIDADES v4.0:
   ────────────────
   1. comm_notify_radar_status(ok): envia MSG_RADAR_FAIL ou
      MSG_RADAR_OK a AMBOS os vizinhos (esq e dir).
      Chamado por fsm_verificar_radar() quando radar muda estado.

   2. comm_left_was_ever_online(): devolve flag s_left_ever_online
      que é marcada true quando viz.esq responde pela primeira vez.
      Usada em fsm_network_vizinhos para distinguir arranque de falha.

   3. comm_status_ok(): não retorna false por ausência de vizinhos.
      Apenas verifica socket válido. AUTONOMO por "sem vizinhos"
      é decidido pela FSM com AUTONOMO_DELAY_MS.
============================================================ */
#include "comm_manager.h"
#include "udp_manager.h"
#include "system_config.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG     = "COMM_MGR";
static bool s_iniciado     = false;
static bool s_left_ever_online = false;  /* viz.esq já respondeu alguma vez */

/* ── comm_init ──────────────────────────────────────────── */
bool comm_init(void)
{
    if (s_iniciado) return true;
    s_iniciado = udp_manager_init();
    if (s_iniciado)
        ESP_LOGI(TAG, "Comm v4.0 iniciado");
    else
        ESP_LOGE(TAG, "Falha ao iniciar UDP");
    return s_iniciado;
}

/* ── comm_status_ok — só verifica socket ─────────────────── */
bool comm_status_ok(void)
{
    if (!s_iniciado) return false;
    if (udp_manager_get_socket() < 0) {
        s_iniciado = false;
        ESP_LOGW(TAG, "Socket inválido — comm offline");
        return false;
    }
    return true;
}

/* ── comm_is_master ────────────────────────────────────────
   pos=0 → sempre MASTER.
   pos>0 → MASTER se viz.esq conhecido E offline.           */
bool comm_is_master(void)
{
    if (POST_POSITION == 0) return true;
    if (!s_iniciado) return false;
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION - 1);
    if (!v || !v->active) return false;
    return (v->status == NEIGHBOR_OFFLINE);
}

/* ── comm_left_online ─────────────────────────────────────── */
bool comm_left_online(void)
{
    if (POST_POSITION == 0) return false;
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION - 1);
    bool online = v && v->active && v->status == NEIGHBOR_OK;
    if (online) s_left_ever_online = true;   /* marca descoberto */
    return online;
}

/* ── comm_right_online ────────────────────────────────────── */
bool comm_right_online(void)
{
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION + 1);
    return v && v->active && v->status == NEIGHBOR_OK;
}

/* ── comm_left_was_ever_online ────────────────────────────── */
bool comm_left_was_ever_online(void)
{
    /* Actualiza a flag antes de devolver */
    comm_left_online();
    return s_left_ever_online;
}

/* ── IPs dos vizinhos ─────────────────────────────────────── */
static const char *_ip_dir(void)
{
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION + 1);
    if (!v || !v->active || v->status == NEIGHBOR_OFFLINE) return NULL;
    return v->ip;
}

static const char *_ip_esq(void)
{
    if (POST_POSITION == 0) return NULL;
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION - 1);
    if (!v || !v->active || v->status == NEIGHBOR_OFFLINE) return NULL;
    return v->ip;
}

/* ── ETA: (POSTE_DIST_M - RADAR_DETECT_M) / vel ──────────── */
static uint32_t _eta_ms(float kmh)
{
    if (kmh < 1.0f) kmh = 1.0f;
    float d = (float)(POSTE_DIST_M - RADAR_DETECT_M);
    return (uint32_t)((d / (kmh / 3.6f)) * 1000.0f);
}

/* ── comm_send_tc_inc ─────────────────────────────────────── */
void comm_send_tc_inc(float speed, int16_t x_mm)
{
    const char *ip = _ip_dir();
    if (!ip) { ESP_LOGD(TAG, "TC_INC: sem viz.dir"); return; }
    udp_manager_send_tc_inc(ip, speed, x_mm);
    ESP_LOGI(TAG, "TC_INC → %s | %.0f km/h | x=%dmm", ip, speed, (int)x_mm);
}

/* ── comm_send_spd ────────────────────────────────────────── */
void comm_send_spd(float speed, int16_t x_mm)
{
    const char *ip = _ip_dir();
    if (!ip) { ESP_LOGD(TAG, "SPD: sem viz.dir"); return; }
    uint32_t eta = _eta_ms(speed);
    udp_manager_send_spd(ip, speed, eta, POSTE_DIST_M, x_mm);
    ESP_LOGI(TAG, "SPD → %s | %.0f km/h | ETA=%lums", ip, speed, (unsigned long)eta);
}

/* ── comm_notify_prev_passed ──────────────────────────────── */
void comm_notify_prev_passed(float speed)
{
    const char *ip = _ip_esq();
    if (!ip) return;
    udp_manager_send_tc_inc(ip, -speed, 0);
    ESP_LOGI(TAG, "PASSED → %s", ip);
}

/* ── comm_send_master_claim ───────────────────────────────── */
void comm_send_master_claim(void)
{
    const char *ip = _ip_dir();
    if (!ip) return;
    udp_manager_send_master_claim(ip);
    ESP_LOGI(TAG, "MASTER_CLAIM → %s", ip);
}

/* ── comm_notify_radar_status ─────────────────────────────
   Envia estado do radar a AMBOS os vizinhos usando a função
   udp_manager_send_status() já existente no protocolo v4.1.

   Mapeamento de estados (reutiliza neighbor_status_t):
     radar_ok = false → STATUS:<id>:SAFE  (NEIGHBOR_SAFE)
     radar_ok = true  → STATUS:<id>:OK    (NEIGHBOR_OK)

   O receptor em _processar_mensagem() já trata STATUS e chama
   on_radar_fail_received() ou on_radar_ok_received() conforme
   o estado recebido e o IP de origem (esq ou dir).

   NOTA: udp_manager_send_status envia "STATUS:<id>:<estado>"
   que o parser já reconhece. Não é necessária nova mensagem.
──────────────────────────────────────────────────────────── */
void comm_notify_radar_status(bool radar_ok)
{
    const char       *ip_esq = _ip_esq();
    const char       *ip_dir = _ip_dir();
    neighbor_status_t estado = radar_ok ? NEIGHBOR_OK : NEIGHBOR_SAFE;

    if (ip_esq) {
        udp_manager_send_status(ip_esq, estado);
        ESP_LOGI(TAG, "RADAR_%s → viz.esq %s",
                 radar_ok ? "OK" : "FAIL", ip_esq);
    }
    if (ip_dir) {
        udp_manager_send_status(ip_dir, estado);
        ESP_LOGI(TAG, "RADAR_%s → viz.dir %s",
                 radar_ok ? "OK" : "FAIL", ip_dir);
    }
    if (!ip_esq && !ip_dir)
        ESP_LOGD(TAG, "RADAR_%s: sem vizinhos para notificar",
                 radar_ok ? "OK" : "FAIL");
}
