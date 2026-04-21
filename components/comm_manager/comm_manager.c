/* ============================================================
   COMM MANAGER — IMPLEMENTAÇÃO
   @file      comm_manager.c
   @version   2.1  |  2026-04-08
   Projecto  : Poste Inteligente v8
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

   Camada de abstracção sobre o udp_manager.
   Resolve IPs de vizinhos, calcula ETA e chama as funções
   de envio correctas sem expor detalhes de protocolo à FSM.

   Alterações v2.0 → v2.1:
   ─────────────────────────
   - Corrigido: PRIu32 → %lu + (unsigned long) em comm_send_spd().
   - Removido #include <inttypes.h>.
============================================================ */
#include "comm_manager.h"
#include "udp_manager.h"
#include "system_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char *TAG = "COMM_MGR";

/* Estado: UDP iniciado */
static bool s_iniciado = false;

/* ============================================================
   comm_init
   Inicializa socket UDP e inicia descoberta de vizinhos.
   Chamado pelo system_monitor quando Wi-Fi disponível.
============================================================ */
bool comm_init(void)
{
    if (s_iniciado) return true;
    s_iniciado = udp_manager_init();
    if (s_iniciado)
        ESP_LOGI(TAG, "Comm iniciado — protocolo UDP v4.1 activo");
    else
        ESP_LOGE(TAG, "Falha ao iniciar UDP");
    return s_iniciado;
}

/* ============================================================
   comm_status_ok — true se socket UDP válido e operacional
============================================================ */
bool comm_status_ok(void)
{
    /* Socket inválido — comm offline */
    if (s_iniciado) {
        if (udp_manager_get_socket() < 0) {
            s_iniciado = false;
            ESP_LOGW(TAG, "Socket UDP inválido — comm offline");
        }
    }

    /* Sem vizinhos conhecidos após inicialização — comm degradado.
     * Força AUTONOMO na FSM até descobrir pelo menos um vizinho. */
    if (s_iniciado) {
        neighbor_t *viz_esq = udp_manager_get_neighbor_by_pos(POST_POSITION - 1);
        neighbor_t *viz_dir = udp_manager_get_neighbor_by_pos(POST_POSITION + 1);

        bool tem_vizinho = (viz_esq && viz_esq->active &&
                            viz_esq->status != NEIGHBOR_OFFLINE) ||
                           (viz_dir && viz_dir->active &&
                            viz_dir->status != NEIGHBOR_OFFLINE);

        if (!tem_vizinho) {
            /* Sem vizinhos — comporta-se como offline para a FSM */
            return false;
        }
    }

    return s_iniciado;
}

/* ============================================================
   comm_is_master
   True se este poste é o líder da cadeia.
   Condições: posição 0 OU vizinho esquerdo offline.
============================================================ */
bool comm_is_master(void)
{
    /* Poste 0 é sempre MASTER independentemente do WiFi */
    if (POST_POSITION == 0) return true;

    /* Sem WiFi — não assume MASTER, vai para AUTONOMO */
    if (!s_iniciado) return false;

    neighbor_t *viz_esq =
        udp_manager_get_neighbor_by_pos(POST_POSITION - 1);

    /* Vizinho ainda não descoberto — aguarda DISCOVER */
    if (!viz_esq || !viz_esq->active) return false;

    /* Vizinho conhecido mas offline — assume MASTER */
    if (viz_esq->status == NEIGHBOR_OFFLINE) return true;

    return false;
}
/* ============================================================
   comm_left_online — true se vizinho esquerdo online
============================================================ */
bool comm_left_online(void)
{
    if (POST_POSITION == 0) return false;
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION - 1);
    return v && v->active && v->status == NEIGHBOR_OK;
}

/* ============================================================
   comm_right_online — true se vizinho direito online
============================================================ */
bool comm_right_online(void)
{
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION + 1);
    return v && v->active && v->status == NEIGHBOR_OK;
}

/* ============================================================
   _ip_vizinho_direito — IP do vizinho direito ou NULL
============================================================ */
static const char *_ip_vizinho_direito(void)
{
    neighbor_t *v = udp_manager_get_neighbor_by_pos(POST_POSITION + 1);
    if (!v || !v->active || v->status == NEIGHBOR_OFFLINE) return NULL;
    return v->ip;
}

/* ============================================================
   _ip_vizinho_esquerdo — IP do vizinho esquerdo ou NULL
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
   Exemplo: 50m postes, 7m detecção, 50 km/h
     ETA = (50-7) / (50/3.6) * 1000 = 43 / 13.89 * 1000 ≈ 3096ms
============================================================ */
static uint32_t _calcular_eta_ms(float speed_kmh)
{
    if (speed_kmh < 1.0f) speed_kmh = 1.0f;  /* Evita divisão por zero */
    float dist_m   = (float)(POSTE_DIST_M - RADAR_DETECT_M);
    float speed_ms = speed_kmh / 3.6f;
    return (uint32_t)((dist_m / speed_ms) * 1000.0f);
}

/* ============================================================
   comm_send_tc_inc — envia TC_INC ao vizinho direito
============================================================ */
void comm_send_tc_inc(float speed, int16_t x_mm)
{
    const char *ip = _ip_vizinho_direito();
    if (!ip) {
        ESP_LOGD(TAG, "TC_INC: sem vizinho direito disponível");
        return;
    }
    udp_manager_send_tc_inc(ip, speed, x_mm);
    ESP_LOGD(TAG, "TC_INC → %s | %.0f km/h | x=%dmm", ip, speed, (int)x_mm);
}

/* ============================================================
   comm_send_spd — calcula ETA e envia SPD ao vizinho direito
   CORRIGIDO v2.1: PRIu32 → %lu + (unsigned long)
============================================================ */
void comm_send_spd(float speed, int16_t x_mm)
{
    const char *ip = _ip_vizinho_direito();
    if (!ip) {
        ESP_LOGD(TAG, "SPD: sem vizinho direito disponível");
        return;
    }
    uint32_t eta_ms = _calcular_eta_ms(speed);
    udp_manager_send_spd(ip, speed, eta_ms, POSTE_DIST_M, x_mm);
    ESP_LOGD(TAG, "SPD → %s | %.0f km/h | ETA=%lums | x=%dmm",
             ip, speed, (unsigned long)eta_ms, (int)x_mm);
}

/* ============================================================
   comm_notify_prev_passed — notifica vizinho esquerdo (T--)
   Usa velocidade negativa no TC_INC como sinal de "passed".
============================================================ */
void comm_notify_prev_passed(float speed)
{
    const char *ip = _ip_vizinho_esquerdo();
    if (!ip) return;
    udp_manager_send_tc_inc(ip, -speed, 0);
    ESP_LOGD(TAG, "PASSED → %s (T-- no poste esquerdo)", ip);
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
