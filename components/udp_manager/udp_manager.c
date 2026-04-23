/* ============================================================
   MÓDULO     : udp_manager
   FICHEIRO   : udp_manager.c — Implementação
   VERSÃO     : 4.3  |  2026-04-23
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÕES v4.1 → v4.3:
   ─────────────────────────
   - DISCOVER_ACK adicionado — resposta explícita ao DISCOVER
     com ID e posição do poste respondente.
   - PING adicionado — verifica se vizinho está vivo.
     Responde com STATUS imediato. Complementa o DISCOVER.
   - _parse_int() — parsing seguro de inteiros com strtol.
   - Validação do IP em _enviar_para() com inet_aton check.
   - _encontrar_ou_criar_vizinho() actualiza last_seen e
     discover_ok sempre que vê o vizinho — evita falsos OFFLINE.
   - STATUS com posição incluída na resposta ao DISCOVER —
     garante que get_neighbor_by_pos() encontra o vizinho.
   - TC_INC mantido com lógica vel<0 → PASSED (retrocompat.).
   - SO_BROADCAST e SO_RCVTIMEO mantidos no init.
   - Todas as funções de envio mantidas e completas.
   - Logs organizados com prefixos consistentes.

   PROTOCOLO UDP (mensagens):
   ───────────────────────────
   DISCOVER:<id>:<pos>              → broadcast, descobre vizinhos
   DISCOVER_ACK:<id>:<pos>         → unicast, resposta ao DISCOVER
   STATUS:<id>:<pos>:<estado>      → unicast, estado do poste
   PING:<id>                       → unicast, verifica vida do vizinho
   TC_INC:<id>:<vel>:<x_mm>       → unicast, veículo a caminho
                                      vel<0 → sinal PASSED (T--)
   SPD:<id>:<vel>:<eta>:<dist>:<x> → unicast, velocidade + ETA
   MASTER_CLAIM:<id>               → unicast, propaga liderança
============================================================ */

#include "udp_manager.h"
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


/* ============================================================
   ESTADO INTERNO
============================================================ */
static int        s_socket      = -1;
static bool       s_iniciado    = false;
static neighbor_t s_vizinhos[MAX_NEIGHBORS];
static uint32_t   s_ultimo_disc = 0;


/* ============================================================
   UTILITÁRIOS
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
        case NEIGHBOR_FAIL:      return "FAIL";
        case NEIGHBOR_SAFE:      return "SAFE";
        case NEIGHBOR_AUTO:      return "AUTO";
        case NEIGHBOR_OBSTACULO: return "OBST";
        default:                 return "?";
    }
}

static neighbor_status_t _str_para_status(const char *s)
{
    if (!s)                   return NEIGHBOR_OFFLINE;
    if (!strcmp(s, "OK"))     return NEIGHBOR_OK;
    if (!strcmp(s, "FAIL"))   return NEIGHBOR_FAIL;
    if (!strcmp(s, "SAFE"))   return NEIGHBOR_SAFE;
    if (!strcmp(s, "AUTO"))   return NEIGHBOR_AUTO;
    if (!strcmp(s, "OBST"))   return NEIGHBOR_OBSTACULO;
    return NEIGHBOR_OFFLINE;
}

/* Parsing seguro de inteiro — retorna false se inválido */
static bool _parse_int(const char *str, int *out)
{
    if (!str || !out) return false;
    char *end;
    long v = strtol(str, &end, 10);
    if (end == str || *end != '\0') return false;
    *out = (int)v;
    return true;
}


/* ============================================================
   ENVIO UDP BASE
============================================================ */
static bool _enviar_para(const char *ip, const char *msg)
{
    if (s_socket < 0 || !ip || !msg) return false;

    struct sockaddr_in dest = {0};
    dest.sin_family = AF_INET;
    dest.sin_port   = htons(UDP_PORT);

    /* Valida IP antes de enviar */
    if (inet_aton(ip, &dest.sin_addr) == 0) {
        ESP_LOGW(TAG, "IP inválido: %s", ip);
        return false;
    }

    int r = sendto(s_socket, msg, strlen(msg), 0,
                   (struct sockaddr *)&dest, sizeof(dest));
    if (r < 0) {
        ESP_LOGW(TAG, "[UDP↑] Erro → %s errno=%d", ip, errno);
        return false;
    }

    ESP_LOGD(TAG, "[UDP↑] %s → %s", ip, msg);
    return true;
}


/* ============================================================
   GESTÃO DE VIZINHOS
============================================================ */
static neighbor_t *_encontrar_ou_criar_vizinho(const char *ip,
                                                int id, int pos)
{
    if (!ip || ip[0] == '\0') return NULL;

    /* Procura vizinho existente por IP */
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (s_vizinhos[i].active &&
            strcmp(s_vizinhos[i].ip, ip) == 0) {
            /* Actualiza campos se fornecidos */
            if (id  >= 0) s_vizinhos[i].id       = id;
            if (pos >= 0) s_vizinhos[i].position  = pos;
            /* Actualiza always — evita falsos OFFLINE */
            s_vizinhos[i].last_seen   = _agora_ms();
            s_vizinhos[i].discover_ok = true;
            if (s_vizinhos[i].status == NEIGHBOR_OFFLINE)
                s_vizinhos[i].status = NEIGHBOR_OK;
            return &s_vizinhos[i];
        }
    }

    /* Cria novo vizinho no primeiro slot livre */
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!s_vizinhos[i].active) {
            memset(&s_vizinhos[i], 0, sizeof(neighbor_t));
            strncpy(s_vizinhos[i].ip, ip, MAX_IP_LEN - 1);
            s_vizinhos[i].id          = id;
            s_vizinhos[i].position    = pos;
            s_vizinhos[i].status      = NEIGHBOR_OK;
            s_vizinhos[i].active      = true;
            s_vizinhos[i].discover_ok = true;
            s_vizinhos[i].last_seen   = _agora_ms();
            ESP_LOGI(TAG, "[VIZ] Novo ID=%d pos=%d IP=%s", id, pos, ip);
            return &s_vizinhos[i];
        }
    }

    ESP_LOGW(TAG, "[VIZ] Tabela cheia — ignorar %s", ip);
    return NULL;
}


/* ============================================================
   PROCESSAMENTO DE MENSAGENS
============================================================ */
static void _processar_mensagem(char *msg, const char *ip)
{
    if (!msg || !ip) return;

    /* ── DISCOVER:<id>:<pos> ─────────────────────────────── */
    if (strncmp(msg, "DISCOVER:", 9) == 0) {
        int id = 0, pos = -1;
        if (sscanf(msg + 9, "%d:%d", &id, &pos) < 1) return;
        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, pos);
        if (v) v->status = NEIGHBOR_OK;

        /* Responde com ACK incluindo posição — essencial para
           get_neighbor_by_pos() funcionar correctamente */
        char ack[40];
        snprintf(ack, sizeof(ack), "DISCOVER_ACK:%d:%d",
                 POSTE_ID, POST_POSITION);
        _enviar_para(ip, ack);

        ESP_LOGI(TAG, "[UDP↓] DISCOVER de ID=%d pos=%d", id, pos);
        return;
    }

    /* ── DISCOVER_ACK:<id>:<pos> ─────────────────────────── */
    if (strncmp(msg, "DISCOVER_ACK:", 13) == 0) {
        int id = 0, pos = -1;
        if (sscanf(msg + 13, "%d:%d", &id, &pos) < 1) return;
        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, pos);
        if (v) v->status = NEIGHBOR_OK;

        ESP_LOGI(TAG, "[UDP↓] DISCOVER_ACK de ID=%d pos=%d", id, pos);
        return;
    }

    /* ── STATUS:<id>:<pos>:<estado> ─────────────────────── */
    if (strncmp(msg, "STATUS:", 7) == 0) {
        int id = 0, pos = -1;
        char est[16] = {0};

        /* Tenta formato novo com posição */
        int n = sscanf(msg + 7, "%d:%d:%15s", &id, &pos, est);
        if (n < 3) {
            /* Fallback: formato antigo sem posição */
            pos = -1;
            sscanf(msg + 7, "%d:%15s", &id, est);
        }
        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, pos);
        if (v) v->status = _str_para_status(est);

        ESP_LOGD(TAG, "[UDP↓] STATUS de ID=%d pos=%d: %s", id, pos, est);
        return;
    }

    /* ── PING:<id> ───────────────────────────────────────── */
    if (strncmp(msg, "PING:", 5) == 0) {
        int id = 0;
        if (!_parse_int(msg + 5, &id)) return;
        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, -1);
        if (v) v->status = NEIGHBOR_OK;

        /* Responde com STATUS imediato */
        char resp[40];
        snprintf(resp, sizeof(resp), "STATUS:%d:%d:OK",
                 POSTE_ID, POST_POSITION);
        _enviar_para(ip, resp);

        ESP_LOGD(TAG, "[UDP↓] PING de ID=%d → PONG", id);
        return;
    }

    /* ── TC_INC:<id>:<vel>:<x_mm> ───────────────────────── */
    if (strncmp(msg, "TC_INC:", 7) == 0) {
        int id = 0, x_mm = 0;
        float vel = 0.0f;
        sscanf(msg + 7, "%d:%f:%d", &id, &vel, &x_mm);
        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, -1);
        if (v) v->last_seen = _agora_ms();

        if (vel > 0.0f) {
            /* Veículo a caminho — incrementa Tc */
            ESP_LOGI(TAG, "[UDP↓] TC_INC ID=%d vel=%.0f x=%dmm",
                     id, vel, x_mm);
            on_tc_inc_received(vel, (int16_t)x_mm);
        } else {
            /* Velocidade negativa = sinal PASSED — decrementa T */
            ESP_LOGI(TAG, "[UDP↓] PASSED (via TC_INC neg) de ID=%d", id);
            on_prev_passed_received();
        }
        return;
    }

    /* ── SPD:<id>:<vel>:<eta_ms>:<dist_m>:<x_mm> ───────── */
    if (strncmp(msg, "SPD:", 4) == 0) {
        int id = 0, x_mm = 0;
        float vel = 0.0f;
        unsigned long ul_eta = 0, ul_dist = 0;

        sscanf(msg + 4, "%d:%f:%lu:%lu:%d",
               &id, &vel, &ul_eta, &ul_dist, &x_mm);
        (void)ul_dist; /* dist_m não usado actualmente */

        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, -1);
        if (v) v->last_seen = _agora_ms();

        ESP_LOGI(TAG, "[UDP↓] SPD ID=%d vel=%.0f eta=%lums x=%dmm",
                 id, vel, ul_eta, x_mm);

        on_spd_received(vel, (uint32_t)ul_eta, (int16_t)x_mm);
        return;
    }

    /* ── MASTER_CLAIM:<id> ───────────────────────────────── */
    if (strncmp(msg, "MASTER_CLAIM:", 13) == 0) {
        int id = 0;
        if (!_parse_int(msg + 13, &id)) return;
        if (id == POSTE_ID) return;

        ESP_LOGI(TAG, "[UDP↓] MASTER_CLAIM de ID=%d", id);
        on_master_claim_received(id);
        return;
    }

    ESP_LOGD(TAG, "[UDP↓] Mensagem desconhecida de %s: %s", ip, msg);
}


/* ============================================================
   TIMEOUT DE VIZINHOS
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
            ESP_LOGW(TAG, "[VIZ] ID=%d → OFFLINE (%lums sem resposta)",
                     s_vizinhos[i].id, (unsigned long)delta);
        }
    }
}


/* ============================================================
   UDP TASK — Core 0, Prio 5
============================================================ */
void udp_task_run(void *arg)
{
    char               rx_buf[160];
    struct sockaddr_in origem;

    ESP_LOGI(TAG, "udp_task | Core %d | Porto %d",
             xPortGetCoreID(), UDP_PORT);

    /* Aguarda socket válido — criado por comm_init() após WiFi.
       Heartbeat enviado para evitar falso timeout no monitor. */
    while (s_socket < 0) {
        system_monitor_heartbeat(MOD_UDP);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* DISCOVER imediato após socket disponível */
    udp_manager_discover();
    s_ultimo_disc = _agora_ms();

    while (1) {
        /* 1. Recebe pacote (bloqueia até 10ms via SO_RCVTIMEO) */
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

        /* 2. DISCOVER periódico */
        if ((agora - s_ultimo_disc) >= DISCOVER_INTERVAL_MS) {
            udp_manager_discover();
            s_ultimo_disc = agora;
            ESP_LOGD(TAG, "[UDP] DISCOVER periódico");
        }

        /* 3. Verifica timeouts de vizinhos */
        _verificar_timeouts(agora);

        /* 4. Heartbeat ao monitor */
        system_monitor_heartbeat(MOD_UDP);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


/* ============================================================
   INIT — cria socket e faz bind
============================================================ */
bool udp_manager_init(void)
{
    if (s_iniciado) {
        ESP_LOGW(TAG, "Já iniciado");
        return true;
    }

    memset(s_vizinhos, 0, sizeof(s_vizinhos));
    s_ultimo_disc = 0;

    s_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_socket < 0) {
        ESP_LOGE(TAG, "Falha socket errno=%d", errno);
        return false;
    }

    /* Permite envio broadcast (necessário para DISCOVER) */
    int bc = 1;
    setsockopt(s_socket, SOL_SOCKET, SO_BROADCAST, &bc, sizeof(bc));

    /* recvfrom não bloqueia mais de 10ms — mantém task responsiva */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 10000 };
    setsockopt(s_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(UDP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(s_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Falha bind porto %d errno=%d", UDP_PORT, errno);
        close(s_socket);
        s_socket = -1;
        return false;
    }

    s_iniciado = true;
    ESP_LOGI(TAG, "UDP v4.3 OK | porto %d", UDP_PORT);
    return true;
}


/* ============================================================
   START TASK — Core 0, Prio 5
============================================================ */
void udp_manager_task_start(void)
{
    xTaskCreatePinnedToCore(udp_task_run, "udp_task",
                            4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "udp_task | Core 0 | Prio 5 | Stack 4096");
}


/* ============================================================
   API DE ENVIO
============================================================ */

/* DISCOVER em broadcast — descobre vizinhos na rede */
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
}

/* DISCOVER_ACK — resposta unicast ao DISCOVER com posição */
bool udp_manager_send_discover_ack(const char *ip)
{
    char msg[40];
    snprintf(msg, sizeof(msg), "DISCOVER_ACK:%d:%d",
             POSTE_ID, POST_POSITION);
    return _enviar_para(ip, msg);
}

/* PING — verifica se vizinho está vivo */
void udp_manager_send_ping(const char *ip)
{
    char msg[16];
    snprintf(msg, sizeof(msg), "PING:%d", POSTE_ID);
    _enviar_para(ip, msg);
}

/* TC_INC — veículo a caminho com posição lateral */
bool udp_manager_send_tc_inc(const char *ip, float speed, int16_t x_mm)
{
    char msg[48];
    snprintf(msg, sizeof(msg), "TC_INC:%d:%.1f:%d",
             POSTE_ID, speed, (int)x_mm);
    return _enviar_para(ip, msg);
}

/* SPD — velocidade, ETA e distância */
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

/* STATUS — estado actual deste poste com posição */
bool udp_manager_send_status(const char *ip, neighbor_status_t status)
{
    char msg[40];
    snprintf(msg, sizeof(msg), "STATUS:%d:%d:%s",
             POSTE_ID, POST_POSITION, _status_str(status));
    return _enviar_para(ip, msg);
}

/* MASTER_CLAIM — propaga liderança em cadeia */
bool udp_manager_send_master_claim(const char *ip)
{
    char msg[24];
    snprintf(msg, sizeof(msg), "MASTER_CLAIM:%d", POSTE_ID);
    return _enviar_para(ip, msg);
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
        if (s_vizinhos[i].active &&
            s_vizinhos[i].position == position)
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

int udp_manager_get_socket(void)
{
    return s_socket;
}


/* ============================================================
   CALLBACKS WEAK — substituídos pela state_machine.c
============================================================ */
__attribute__((weak))
void on_tc_inc_received(float speed, int16_t x_mm)
{ (void)speed; (void)x_mm; }

__attribute__((weak))
void on_prev_passed_received(void) {}

__attribute__((weak))
void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm)
{ (void)speed; (void)eta_ms; (void)x_mm; }

__attribute__((weak))
void on_master_claim_received(int from_id) { (void)from_id; }
