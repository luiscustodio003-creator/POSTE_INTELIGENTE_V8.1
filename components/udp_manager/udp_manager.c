/* ============================================================
   UDP MANAGER — IMPLEMENTAÇÃO
   @file      udp_manager.c
   @version   5.2  |  2026-05-02
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÕES v5.1 → v5.2:
   ──────────────────────────────────────────────────────────
   - ADICIONADO: udp_manager_send_master_claim_id(ip, master_id)
     Envia "MASTER_CLAIM:<POSTE_ID>:<master_id>" preservando
     o ID do MASTER real ao longo de toda a cadeia de relays.

   - ADICIONADO: on_master_claim_received_ext(from_id, master_id)
     Callback weak com dois parâmetros. Implementado em fsm_events.c.

   - ALTERADO: parser de MASTER_CLAIM suporta ambos os formatos:
     Antigo: "MASTER_CLAIM:<id>"            → from_id = master_id
     Novo:   "MASTER_CLAIM:<from>:<master>" → relay completo
     Compatibilidade total com postes v5.1 ainda não actualizados.
============================================================ */
#include "udp_manager.h"
#include "state_machine.h"
#include "system_monitor.h"
#include "system_config.h"
#include "esp_log.h"
#include "esp_timer.h"
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
static int         s_socket      = -1;
static bool        s_iniciado    = false;
static neighbor_t  s_vizinhos[MAX_NEIGHBORS];
static uint32_t    s_ultimo_disc = 0;
static udp_stats_t s_stats       = {0};


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
    if (!strcmp(s, "FAIL"))    return NEIGHBOR_SAFE;
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
    if (r < 0) {
        ESP_LOGW(TAG, "[TX] Falha → %s errno=%d", ip, errno);
        return false;
    }

    s_stats.pkts_enviados++;
    ESP_LOGD(TAG, "[TX] → %s : %s", ip, msg);
    return true;
}

static neighbor_t *_encontrar_ou_criar_vizinho(const char *ip, int id, int pos)
{
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (s_vizinhos[i].active && strcmp(s_vizinhos[i].ip, ip) == 0) {
            if (id  >= 0) s_vizinhos[i].id      = id;
            if (pos >= 0) s_vizinhos[i].position = pos;
            return &s_vizinhos[i];
        }
    }
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!s_vizinhos[i].active) {
            memset(&s_vizinhos[i], 0, sizeof(neighbor_t));
            strncpy(s_vizinhos[i].ip, ip, MAX_IP_LEN - 1);
            s_vizinhos[i].id       = id;
            s_vizinhos[i].position = pos;
            s_vizinhos[i].status   = NEIGHBOR_OK;
            s_vizinhos[i].active   = true;
            ESP_LOGI(TAG, "Novo vizinho: ID=%d pos=%d IP=%s", id, pos, ip);
            return &s_vizinhos[i];
        }
    }
    ESP_LOGW(TAG, "Tabela cheia — vizinho %s ignorado", ip);
    return NULL;
}


/* ============================================================
   _processar_mensagem
============================================================ */
static void _processar_mensagem(char *msg, const char *ip)
{
    if (!msg || !ip) return;

    s_stats.pkts_recebidos++;

    /* ── DISCOVER:<id>:<pos> ─────────────────────────────────── */
    if (strncmp(msg, "DISCOVER:", 9) == 0) {
        int id = 0, pos = -1;
        sscanf(msg + 9, "%d:%d", &id, &pos);
        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, pos);
        if (v) {
            v->last_seen   = _agora_ms();
            v->discover_ok = true;
            if (v->status == NEIGHBOR_OFFLINE) v->status = NEIGHBOR_OK;
        }

        /* Responde com estado REAL deste poste */
        char resp[32];
        snprintf(resp, sizeof(resp), "STATUS:%d:%s",
                 POSTE_ID, _estado_local_str());
        _enviar_para(ip, resp);

        ESP_LOGD(TAG, "[RX] DISCOVER ID=%d pos=%d → STATUS:%s",
                 id, pos, _estado_local_str());
        return;
    }

    /* ── STATUS:<id>:<estado> ───────────────────────────────── */
    if (strncmp(msg, "STATUS:", 7) == 0) {
        int id = 0; char est[16] = {0};
        sscanf(msg + 7, "%d:%15s", &id, est);
        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, -1);
        if (v) {
            v->last_seen = _agora_ms();
            v->status    = _str_para_status(est);
        }
        ESP_LOGD(TAG, "[RX] STATUS ID=%d estado=%s", id, est);
        return;
    }

    /* ── TC_INC:<id>:<vel>:<x_mm> ────────────────────────────── */
    if (strncmp(msg, "TC_INC:", 7) == 0) {
        int id = 0, x_mm = 0; float vel = 0.0f;
        sscanf(msg + 7, "%d:%f:%d", &id, &vel, &x_mm);
        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, -1);
        if (v) v->last_seen = _agora_ms();

        s_stats.tc_inc_recebidos++;
        ESP_LOGD(TAG, "[RX] TC_INC ID=%d vel=%.0f x=%dmm", id, vel, x_mm);

        if (vel > 0.0f)
            on_tc_inc_received(vel, (int16_t)x_mm);
        else if (vel < 0.0f)
            on_prev_passed_received(0.0f);
        return;
    }

    /* ── PASSED:<id>:<vel> ───────────────────────────────────── */
    if (strncmp(msg, "PASSED:", 7) == 0) {
        int id = 0; float vel = 0.0f;
        sscanf(msg + 7, "%d:%f", &id, &vel);
        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, -1);
        if (v) v->last_seen = _agora_ms();

        ESP_LOGD(TAG, "[RX] PASSED ID=%d vel=%.0f", id, vel);
        on_prev_passed_received(vel);
        return;
    }

    /* ── SPD:<id>:<vel>:<eta_ms>:<dist_m>:<x_mm> ──────────────── */
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

    /* ── MASTER_CLAIM:<from_id>[:<master_id>] ────────────────────
       Suporta dois formatos:
         v5.1 (antigo): "MASTER_CLAIM:<id>"
           → from_id = master_id = id  (compatibilidade)
         v5.2 (novo):   "MASTER_CLAIM:<from_id>:<master_id>"
           → relay completo com ID do MASTER real preservado

       Chama on_master_claim_received_ext() que em fsm_events.c
       delega para fsm_network_master_claim_relay().
    ─────────────────────────────────────────────────────────── */
    if (strncmp(msg, "MASTER_CLAIM:", 13) == 0) {
        int from_id   = 0;
        int master_id = 0;

        /* Tenta ler dois campos; se só um → formato antigo */
        int n = sscanf(msg + 13, "%d:%d", &from_id, &master_id);
        if (n < 2) master_id = from_id;  /* compatibilidade v5.1 */

        /* Ignora mensagens próprias (eco) */
        if (from_id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, from_id, -1);
        if (v) v->last_seen = _agora_ms();

        ESP_LOGI(TAG, "[RX] MASTER_CLAIM from=%d master=%d", from_id, master_id);
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
void udp_task_run(void *arg)
{
    (void)arg;
    char               rx_buf[160];
    struct sockaddr_in origem;

    ESP_LOGI(TAG, "udp_task v5.2 | Core %d | Porto %d",
             xPortGetCoreID(), UDP_PORT);

    while (s_socket < 0) {
        system_monitor_heartbeat(MOD_UDP);
        vTaskDelay(pdMS_TO_TICKS(200));
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
    s_iniciado = true;

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
    char msg[32];
    snprintf(msg, sizeof(msg), "MASTER_CLAIM:%d", POSTE_ID);
    return _enviar_para(ip, msg);
}

/* ============================================================
   udp_manager_send_master_claim_id  (NOVO v5.2)
   ──────────────────────────────────────────────────────────
   Envia "MASTER_CLAIM:<POSTE_ID>:<master_id>".
   O from_id (POSTE_ID) identifica quem fez o relay.
   O master_id identifica o MASTER real — não muda ao longo da cadeia.
============================================================ */
bool udp_manager_send_master_claim_id(const char *ip, int master_id)
{
    char msg[40];
    snprintf(msg, sizeof(msg), "MASTER_CLAIM:%d:%d", POSTE_ID, master_id);
    bool ok = _enviar_para(ip, msg);
    ESP_LOGD(TAG, "[TX] MASTER_CLAIM from=%d master=%d → %s",
             POSTE_ID, master_id, ip);
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

size_t udp_manager_get_all_neighbors(neighbor_t *list, size_t max)
{
    size_t n = 0;
    for (int i = 0; i < MAX_NEIGHBORS && n < max; i++)
        if (s_vizinhos[i].active) list[n++] = s_vizinhos[i];
    return n;
}

void udp_manager_reset_neighbor(int position)
{
    neighbor_t *v = udp_manager_get_neighbor_by_pos(position);
    if (v) {
        v->last_seen   = 0;
        v->discover_ok = false;
        v->status      = NEIGHBOR_OFFLINE;
        ESP_LOGI(TAG, "Vizinho pos=%d marcado para redescoberta", position);
    }
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

/* Callback legado — mantido para compatibilidade */
__attribute__((weak)) void on_master_claim_received(int from_id)
{ (void)from_id; }

/* Callback novo v5.2 — delega para on_master_claim_received se não implementado */
__attribute__((weak)) void on_master_claim_received_ext(int from_id, int master_id)
{
    /* Por defeito, chama o callback legado com from_id.
       fsm_events.c implementa esta função correctamente. */
    on_master_claim_received(from_id);
    (void)master_id;
}
