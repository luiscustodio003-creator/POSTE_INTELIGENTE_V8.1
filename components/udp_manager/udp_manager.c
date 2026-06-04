/* ============================================================
   MÓDULO     : udp_manager
   FICHEIRO   : udp_manager.c — Gestão UDP: descoberta, vizinhos e protocolo
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)
============================================================ */
#include "udp_manager.h"
#include "state_machine.h"
#include "system_monitor.h"
#include "system_config.h"
#include "wifi_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

static const char *TAG = "UDP_MGR";

/* ── Estado interno ────────────────────────────────────────── */
static int         s_socket   = -1;
static neighbor_t  s_vizinhos[MAX_NEIGHBORS];
static uint32_t    s_ultimo_disc = 0;
static udp_stats_t s_stats       = {0};

/* ── MASTER_CLAIM: anti-loop (TTL) e deduplicação ──────────── */
#define MASTER_CLAIM_MAX_HOPS  20u

/* Valores do último MASTER_CLAIM recebido — usados pelo relay em fsm_network.c. */
static uint16_t s_relay_seq            = 0;
static uint8_t  s_relay_hop            = 0;

/* Contador de sequence number para claims originados neste poste. */
static uint16_t s_claim_seq_local      = 0;

/* Tabela de deduplicação: último seq processado por master_id (índice 0-255).
   Evita falsos positivos quando dois masters diferentes usam o mesmo seq.
   512 bytes de RAM — aceitável para ESP32. */
static uint16_t s_last_seq_por_master[256] = {0};


/* ============================================================
   UTILITÁRIOS INTERNOS
============================================================ */

static uint32_t _agora_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static const char *_status_str(neighbor_status_t s)
{
    switch (s) {
        case NEIGHBOR_OK:        return "OK";
        case NEIGHBOR_OFFLINE:   return "OFFLINE";
        case NEIGHBOR_SAFE:      return "SAFE";
        case NEIGHBOR_AUTO:      return "AUTO";
        case NEIGHBOR_OBSTACULO: return "OBST";
        default:                 return "?";
    }
}

static neighbor_status_t _str_para_status(const char *s)
{
    if (!s)                    return NEIGHBOR_OFFLINE;
    if (!strcmp(s, "OK"))      return NEIGHBOR_OK;
    if (!strcmp(s, "SAFE"))    return NEIGHBOR_SAFE;
    if (!strcmp(s, "AUTO"))    return NEIGHBOR_AUTO;
    if (!strcmp(s, "OBST"))    return NEIGHBOR_OBSTACULO;
    return NEIGHBOR_OFFLINE;
}

static const char *_estado_local_str(void)
{
    switch (state_machine_get_state()) {
        case STATE_SAFE_MODE: return "SAFE";
        case STATE_AUTONOMO:  return "AUTO";
        case STATE_OBSTACULO: return "OBST";
        default:              return "OK";
    }
}

static bool _enviar_para(const char *ip, const char *msg)
{
    if (s_socket < 0 || !ip || !msg) return false;

    struct sockaddr_in dest = {0};
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(UDP_PORT);
    inet_aton(ip, &dest.sin_addr);

    int r = sendto(s_socket, msg, strlen(msg), 0,
                   (struct sockaddr *)&dest, sizeof(dest));
    if (r > 0) {
        s_stats.pkts_enviados++;
        return true;
    }
    return false;
}


/* ============================================================
   _encontrar_ou_criar_vizinho
============================================================ */
static neighbor_t *_encontrar_ou_criar_vizinho(const char *ip, int id, int pos)
{
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (s_vizinhos[i].active && s_vizinhos[i].id == id) {
            if (!s_vizinhos[i].discover_ok) {
                s_vizinhos[i].discover_ok = true;
                ESP_LOGI(TAG, "Vizinho ID=%d pos=%d reconectado", id, pos);
            }
            return &s_vizinhos[i];
        }
    }

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!s_vizinhos[i].active) {
            s_vizinhos[i].active       = true;
            s_vizinhos[i].discover_ok  = true;
            s_vizinhos[i].id           = id;
            s_vizinhos[i].position     = (pos >= 0) ? pos : 999;
            s_vizinhos[i].status       = NEIGHBOR_OK;
            s_vizinhos[i].last_seen    = _agora_ms();
            strncpy(s_vizinhos[i].ip, ip, MAX_IP_LEN - 1);
            ESP_LOGI(TAG, "Vizinho novo: ID=%d pos=%d IP=%s", id, pos, ip);
            return &s_vizinhos[i];
        }
    }
    return NULL;
}


/* ============================================================
   _processar_mensagem — parser de todos os tipos UDP
============================================================ */
static void _processar_mensagem(const char *msg, const char *ip)
{
    if (!msg || !ip) return;

    s_stats.pkts_recebidos++;

    /* ── DISCOVER:<id>:<pos> ──────────────────────────────────── */
    if (strncmp(msg, "DISCOVER:", 9) == 0) {
        int id = 0, pos = 0;
        sscanf(msg + 9, "%d:%d", &id, &pos);
        if (id == POSTE_ID) {
            const char *my_ip = wifi_manager_get_ip();
            if (my_ip &&
                strcmp(my_ip, "---")     != 0 &&
                strcmp(my_ip, "OFFLINE") != 0 &&
                strcmp(ip,    my_ip)     != 0)
                ESP_LOGE(TAG, "COLISÃO POSTE_ID=%d — remoto=%s local=%s — verificar NVS!",
                         POSTE_ID, ip, my_ip);
            return;
        }

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, pos);
        if (!v) return;

        v->position  = pos;
        v->last_seen = _agora_ms();

        char resp[32];
        snprintf(resp, sizeof(resp), "DISCOVER_ACK:%d:%d:%s",
                 POSTE_ID, POST_POSITION, _estado_local_str());
        _enviar_para(ip, resp);
        return;
    }

    /* ── DISCOVER_ACK:<id>:<pos>:<estado> ─────────────────────── */
    if (strncmp(msg, "DISCOVER_ACK:", 13) == 0) {
        int id = 0, pos = 0;
        char est_str[8] = {0};
        sscanf(msg + 13, "%d:%d:%7s", &id, &pos, est_str);
        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, pos);
        if (v) {
            v->position  = pos;
            v->status    = _str_para_status(est_str);
            v->last_seen = _agora_ms();
        }
        return;
    }

    /* ── STATUS:<id>:<estado> ─────────────────────────────────── */
    if (strncmp(msg, "STATUS:", 7) == 0) {
        int id = 0;
        char est_str[8] = {0};
        sscanf(msg + 7, "%d:%7s", &id, est_str);
        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, -1);
        if (v) {
            v->status    = _str_para_status(est_str);
            v->last_seen = _agora_ms();
            ESP_LOGD(TAG, "[RX] STATUS ID=%d → %s", id, est_str);
        }
        return;
    }

    /* ── TC_INC:<id>:<vel>:<x_mm> ────────────────────────────── */
    if (strncmp(msg, "TC_INC:", 7) == 0) {
        int id = 0, x_mm = 0;
        float vel = 0.0f;
        sscanf(msg + 7, "%d:%f:%d", &id, &vel, &x_mm);
        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, -1);
        if (v) v->last_seen = _agora_ms();

        ESP_LOGD(TAG, "[RX] TC_INC ID=%d vel=%.0f x=%d", id, vel, x_mm);
        s_stats.tc_inc_recebidos++;
        on_tc_inc_received((uint16_t)id, vel, (int16_t)x_mm);
        return;
    }

    /* ── PASSED:<id>:<vel> ─────────────────────────────────────── */
    if (strncmp(msg, "PASSED:", 7) == 0) {
        int id = 0;
        float vel = 0.0f;
        sscanf(msg + 7, "%d:%f", &id, &vel);
        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, -1);
        if (v) v->last_seen = _agora_ms();

        ESP_LOGD(TAG, "[RX] PASSED ID=%d vel=%.0f", id, vel);
        on_prev_passed_received(vel);
        return;
    }

    /* ── SPD:<id>:<vel>:<eta_ms>:<dist_m>:<x_mm> ────────────── */
    if (strncmp(msg, "SPD:", 4) == 0) {
        int id = 0, x_mm = 0;
        float vel = 0.0f;
        unsigned long ul_eta = 0, ul_dist = 0;
        sscanf(msg + 4, "%d:%f:%lu:%lu:%d",
               &id, &vel, &ul_eta, &ul_dist, &x_mm);
        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, -1);
        if (v) v->last_seen = _agora_ms();

        ESP_LOGD(TAG, "[RX] SPD ID=%d vel=%.0f eta=%lums", id, vel, ul_eta);
        on_spd_received(vel, (uint32_t)ul_eta, (int16_t)x_mm);
        return;
    }

    /* ── OBSTACULO:<from_id>:<vehicle_id>:<speed>:<x_mm> ─────────
       NOVO v5.3 — Notificação de obstáculo

       FORMATO:
         from_id    = ID do poste que enviou (POST_POSITION esq.)
         vehicle_id = ID do veículo parado (tracking_manager)
         speed      = Velocidade quando parou (para logs)
         x_mm       = Posição lateral (para logs)

       ACÇÃO:
         Chama on_obstaculo_received() que cancela TC_TIMEOUT
         e mantém Tc (veículo ainda presente na linha).
    ──────────────────────────────────────────────────────────── */
    if (strncmp(msg, "OBSTACULO:", 10) == 0) {
        int from_id = 0;
        unsigned int vehicle_id = 0;
        int x_mm = 0;
        float speed = 0.0f;
        
        sscanf(msg + 10, "%d:%u:%f:%d", &from_id, &vehicle_id, &speed, &x_mm);
        
        if (from_id == POSTE_ID) return;  /* Ignora eco próprio */
        
        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, from_id, -1);
        if (v) {
            v->last_seen = _agora_ms();
            /* Pode actualizar status para NEIGHBOR_OBSTACULO se desejado */
        }
        
        ESP_LOGW(TAG, "═══════════════════════════════════════");
        ESP_LOGW(TAG, "  [RX] OBSTACULO de ID=%d", from_id);
        ESP_LOGW(TAG, "  vehicle_id=%u | vel=%.1f | x=%d", 
                 vehicle_id, speed, x_mm);
        ESP_LOGW(TAG, "═══════════════════════════════════════");
        
        s_stats.obstaculo_recebidos++;
        on_obstaculo_received((uint16_t)vehicle_id, speed, (int16_t)x_mm);
        return;
    }

    /* ── MASTER_CLAIM:<from_id>[:<master_id>] ────────────────────
       Suporta dois formatos:
         v5.1 (antigo): "MASTER_CLAIM:<id>"
           → from_id = master_id = id  (compatibilidade)
         v5.2 (novo):   "MASTER_CLAIM:<from_id>:<master_id>"
           → relay completo com ID do MASTER real preservado
    ─────────────────────────────────────────────────────────── */
    if (strncmp(msg, "MASTER_CLAIM:", 13) == 0) {
        int      from_id   = 0;
        int      master_id = 0;
        unsigned seq       = 0;
        unsigned hop       = 0;

        int n = sscanf(msg + 13, "%d:%d:%u:%u", &from_id, &master_id, &seq, &hop);
        if (n < 2) { master_id = from_id; }
        /* seq e hop ficam em 0 se ausentes — compatibilidade com formato v5.1/v5.2 */

        if (from_id == POSTE_ID) return;

        /* TTL: bloqueia mensagem se hop count exceder o máximo.
           Protege contra broadcast storm em topologias com loop físico. */
        if (hop >= MASTER_CLAIM_MAX_HOPS) {
            ESP_LOGE(TAG, "[RX] MASTER_CLAIM hop=%u >= %u — loop detectado, descartado",
                     hop, MASTER_CLAIM_MAX_HOPS);
            return;
        }

        /* Deduplicação por (master_id, seq): descarta cópias do mesmo claim.
           Rastreia por master_id para evitar falsos positivos entre masters diferentes.
           seq=0 indica formato legacy — sem deduplicação para compatibilidade. */
        if (seq != 0u && (unsigned)master_id < 256u) {
            if (seq == (unsigned)s_last_seq_por_master[master_id]) {
                ESP_LOGD(TAG, "[RX] MASTER_CLAIM master=%d seq=%u duplicado — descartado",
                         master_id, seq);
                return;
            }
            s_last_seq_por_master[master_id] = (uint16_t)seq;
        }

        /* Guarda seq e hop+1 para o relay em fsm_network_master_claim_relay(). */
        s_relay_seq = (uint16_t)seq;
        s_relay_hop = (uint8_t)((hop + 1u < MASTER_CLAIM_MAX_HOPS) ? hop + 1u : MASTER_CLAIM_MAX_HOPS);

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, from_id, -1);
        if (v) v->last_seen = _agora_ms();

        ESP_LOGI(TAG, "[RX] MASTER_CLAIM from=%d master=%d seq=%u hop=%u",
                 from_id, master_id, seq, hop);
        on_master_claim_received_ext(from_id, master_id);
        return;
    }

    ESP_LOGD(TAG, "[RX] Mensagem desconhecida de %s: %.40s", ip, msg);
}


/* ============================================================
   _verificar_timeouts
============================================================ */
static void _verificar_timeouts(uint32_t agora)
{
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!s_vizinhos[i].active)      continue;
        if (!s_vizinhos[i].discover_ok) continue;

        uint32_t delta = agora - s_vizinhos[i].last_seen;
        if (delta > NEIGHBOR_TIMEOUT_MS &&
            s_vizinhos[i].status != NEIGHBOR_OFFLINE) {
            s_vizinhos[i].status = NEIGHBOR_OFFLINE;
            s_stats.timeouts_vizinhos++;
            ESP_LOGI(TAG, "Vizinho ID=%d → OFFLINE (%lums sem resposta)",
                     s_vizinhos[i].id, (unsigned long)delta);
        }
    }
}


/* ============================================================
   udp_task_run — Core 0, Prio 5
============================================================ */
static void udp_task_run(void *arg)
{
    (void)arg;
    char               rx_buf[160];
    struct sockaddr_in origem;

    ESP_LOGI(TAG, "udp_task v5.3 | Core %d | Porto %d",
             xPortGetCoreID(), UDP_PORT);

    while (s_socket < 0) {
        system_monitor_heartbeat(MOD_UDP);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* Jitter de arranque: evita colisão de DISCOVERs em boot simultâneo.
       POST_POSITION×30ms separa postes adjacentes; +0-199ms cobre resto.
       Cap em 2000ms: evita atraso excessivo em linhas longas (pos>60). */
    uint32_t jitter_ms = (uint32_t)POST_POSITION * 30u + (uint32_t)(esp_random() % 200u);
    if (jitter_ms > 2000u) jitter_ms = 2000u;
    if (jitter_ms > 0) {
        ESP_LOGI(TAG, "Boot jitter: %lums (pos=%d)", (unsigned long)jitter_ms, POST_POSITION);
        vTaskDelay(pdMS_TO_TICKS(jitter_ms));
    }

    udp_manager_discover();
    s_ultimo_disc = _agora_ms();

    while (1) {
        socklen_t len = sizeof(origem);
        int r = recvfrom(s_socket, rx_buf, sizeof(rx_buf) - 1,
                         0, (struct sockaddr *)&origem, &len);
        if (r > 0) {
            rx_buf[r] = '\0';
            char ip_str[MAX_IP_LEN];
            inet_ntoa_r(origem.sin_addr, ip_str, sizeof(ip_str));
            _processar_mensagem(rx_buf, ip_str);
        }

        uint32_t agora = _agora_ms();

        if ((agora - s_ultimo_disc) >= DISCOVER_INTERVAL_MS) {
            udp_manager_discover();
            s_ultimo_disc = agora;
        }

        _verificar_timeouts(agora);
        system_monitor_heartbeat(MOD_UDP);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


/* ============================================================
   udp_manager_init
============================================================ */
bool udp_manager_init(void)
{
    s_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_socket < 0) {
        ESP_LOGE(TAG, "socket() falhou: errno=%d", errno);
        return false;
    }

    int bcast = 1;
    setsockopt(s_socket, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));

    struct timeval tv = { .tv_sec = 0, .tv_usec = 10000 };
    setsockopt(s_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(UDP_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(s_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() falhou: errno=%d", errno);
        close(s_socket);
        s_socket = -1;
        return false;
    }

    memset(s_vizinhos, 0, sizeof(s_vizinhos));
    memset(&s_stats,   0, sizeof(s_stats));
    s_ultimo_disc = 0;  /* força DISCOVER imediato na próxima iteração do task */

    ESP_LOGI(TAG, "UDP socket OK | Porto %d", UDP_PORT);
    return true;
}

void udp_manager_task_start(void)
{
    xTaskCreatePinnedToCore(udp_task_run, "udp_task",
                            4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "udp_task | Core 0 | Prio 5 | Stack 4096");
}


/* ============================================================
   API DE ENVIO
============================================================ */

void udp_manager_discover(void)
{
    if (s_socket < 0) return;

    struct sockaddr_in dest = {0};
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(UDP_PORT);
    dest.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    char msg[32];
    snprintf(msg, sizeof(msg), "DISCOVER:%d:%d", POSTE_ID, POST_POSITION);
    sendto(s_socket, msg, strlen(msg), 0,
           (struct sockaddr *)&dest, sizeof(dest));
    s_stats.pkts_enviados++;
}

bool udp_manager_send_tc_inc(const char *ip, float speed, int16_t x_mm)
{
    char msg[48];
    snprintf(msg, sizeof(msg), "TC_INC:%d:%.1f:%d",
             POSTE_ID, speed, (int)x_mm);
    bool ok = _enviar_para(ip, msg);
    if (ok) s_stats.tc_inc_enviados++;
    return ok;
}

bool udp_manager_send_passed(const char *ip, float speed)
{
    char msg[32];
    snprintf(msg, sizeof(msg), "PASSED:%d:%.1f", POSTE_ID, speed);
    return _enviar_para(ip, msg);
}

bool udp_manager_send_spd(const char *ip, float speed,
                           uint32_t eta_ms, uint32_t dist_m, int16_t x_mm)
{
    char msg[80];
    snprintf(msg, sizeof(msg), "SPD:%d:%.1f:%lu:%lu:%d",
             POSTE_ID, speed,
             (unsigned long)eta_ms,
             (unsigned long)dist_m,
             (int)x_mm);
    return _enviar_para(ip, msg);
}

bool udp_manager_send_status(const char *ip, neighbor_status_t status)
{
    char msg[32];
    snprintf(msg, sizeof(msg), "STATUS:%d:%s", POSTE_ID, _status_str(status));
    return _enviar_para(ip, msg);
}

bool udp_manager_send_master_claim(const char *ip)
{
    /* Alias para compatibilidade — usa o formato v5.3 com seq. */
    return udp_manager_send_master_claim_id(ip, POSTE_ID);
}

bool udp_manager_send_master_claim_id(const char *ip, int master_id)
{
    /* Gera novo seq para claim original; salta 0 (reservado para formato legacy). */
    if (++s_claim_seq_local == 0u) s_claim_seq_local = 1u;

    char msg[56];
    snprintf(msg, sizeof(msg), "MASTER_CLAIM:%d:%d:%u:0",
             POSTE_ID, master_id, (unsigned)s_claim_seq_local);

    /* Triple-send: melhora entrega no primeiro hop sem ACK explícito.
       Com 5% de packet loss: P(todos perdidos) = 0.05³ ≈ 0.01%.
       Deduplicação no receptor garante processamento único. */
    bool ok = false;
    for (int i = 0; i < 3; i++) ok |= _enviar_para(ip, msg);

    ESP_LOGD(TAG, "[TX] MASTER_CLAIM from=%d master=%d seq=%u hop=0 x3 → %s",
             POSTE_ID, master_id, (unsigned)s_claim_seq_local, ip);
    return ok;
}

bool udp_manager_send_master_claim_relay(const char *ip, int master_id,
                                         uint16_t seq, uint8_t hop)
{
    char msg[56];
    snprintf(msg, sizeof(msg), "MASTER_CLAIM:%d:%d:%u:%u",
             POSTE_ID, master_id, (unsigned)seq, (unsigned)hop);

    /* Triple-send no relay: cada nó amplifica fiabilidade da cadeia.
       9 hops × 99.99% = 99.9% fim-a-fim (vs 63% sem retry). */
    bool ok = false;
    for (int i = 0; i < 3; i++) ok |= _enviar_para(ip, msg);

    ESP_LOGD(TAG, "[TX] MASTER_CLAIM relay from=%d master=%d seq=%u hop=%u x3 → %s",
             POSTE_ID, master_id, (unsigned)seq, (unsigned)hop, ip);
    return ok;
}

uint16_t udp_manager_get_relay_seq(void) { return s_relay_seq; }
uint8_t  udp_manager_get_relay_hop(void) { return s_relay_hop; }

/* ============================================================
   udp_manager_send_obstaculo  (NOVO v5.3)
   ──────────────────────────────────────────────────────────
   Envia notificação de obstáculo ao vizinho direito.

   FORMATO: "OBSTACULO:<from_id>:<vehicle_id>:<speed>:<x_mm>"

   QUANDO CHAMAR:
     - Em fsm_events.c, caso SM_EVT_VEHICLE_OBSTACULO
     - Só se g_fsm_right_online == true

   EFEITO NO RECEPTOR:
     - Cancela TC_TIMEOUT (sabe que veículo parou)
     - Mantém Tc (veículo ainda presente na linha)
     - Luz fica acesa até receber PASSED real
============================================================ */
bool udp_manager_send_obstaculo(const char *ip, uint16_t vehicle_id,
                                float speed, int16_t x_mm)
{
    char msg[64];
    snprintf(msg, sizeof(msg), "OBSTACULO:%d:%u:%.1f:%d",
             POSTE_ID, (unsigned int)vehicle_id, speed, (int)x_mm);
    
    bool ok = _enviar_para(ip, msg);
    
    if (ok) {
        s_stats.obstaculo_enviados++;
        ESP_LOGW(TAG, "[TX] OBSTACULO → %s | ID=%u vel=%.1f x=%d",
                 ip, (unsigned int)vehicle_id, speed, (int)x_mm);
    }
    
    return ok;
}


/* ============================================================
   CONSULTA DE VIZINHOS
============================================================ */

void udp_manager_get_neighbors(char *nebL, char *nebR)
{
    strncpy(nebL, "---", MAX_IP_LEN);
    strncpy(nebR, "---", MAX_IP_LEN);
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!s_vizinhos[i].active) continue;
        if (s_vizinhos[i].position == POST_POSITION - 1)
            strncpy(nebL, s_vizinhos[i].ip, MAX_IP_LEN - 1);
        if (s_vizinhos[i].position == POST_POSITION + 1)
            strncpy(nebR, s_vizinhos[i].ip, MAX_IP_LEN - 1);
    }
}

neighbor_t *udp_manager_get_neighbor_by_pos(int position)
{
    for (int i = 0; i < MAX_NEIGHBORS; i++)
        if (s_vizinhos[i].active && s_vizinhos[i].position == position)
            return &s_vizinhos[i];
    return NULL;
}

void udp_manager_get_stats(udp_stats_t *out)
{
    if (out) *out = s_stats;
}

int udp_manager_get_socket(void)
{
    return s_socket;
}


/* ============================================================
   CALLBACKS WEAK — substituídos pela state_machine (fsm_events.c)
============================================================ */
__attribute__((weak)) void on_tc_inc_received(float speed, int16_t x_mm)
{ (void)speed; (void)x_mm; }

__attribute__((weak)) void on_prev_passed_received(float speed)
{ (void)speed; }

__attribute__((weak)) void on_spd_received(float speed, uint32_t eta_ms,
                                            int16_t x_mm)
{ (void)speed; (void)eta_ms; (void)x_mm; }

__attribute__((weak)) void on_master_claim_received(int from_id)
{ (void)from_id; }

__attribute__((weak)) void on_master_claim_received_ext(int from_id, int master_id)
{
    on_master_claim_received(from_id);
    (void)master_id;
}

/* ── NOVO v5.3 — Callback de obstáculo ────────────────────── */
__attribute__((weak)) void on_obstaculo_received(uint16_t vehicle_id,
                                                  float speed,
                                                  int16_t x_mm)
{
    /* Implementação weak vazia — fsm_events.c substitui com lógica real */
    (void)vehicle_id;
    (void)speed;
    (void)x_mm;
}
