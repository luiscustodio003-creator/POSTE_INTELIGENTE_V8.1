/* ============================================================
   UDP MANAGER — IMPLEMENTAÇÃO
   @file      udp_manager.c
   @version   5.0  |  2026-04-24
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   MELHORIAS v4.1 → v5.0:
   ──────────────────────────────────────────────────────────
   1. udp_manager_send_passed() dedicado — mensagem PASSED explícita
      em vez de TC_INC com velocidade negativa como sinalização.
      Mais claro e permite ACK específico no futuro.
   2. udp_manager_get_stats() — estatísticas de diagnóstico:
      contadores de pacotes enviados/recebidos e TC_INC.
   3. udp_manager_reset_neighbor() — força redescoberta de vizinho.
   4. Todos os contadores de estatísticas integrados em _enviar_para()
      e _processar_mensagem() sem overhead adicional.
   5. Log [RX] e [TX] padronizados: prefixo consistente em todos
      os tipos de mensagem para facilitar filtragem no monitor série.
   6. #include <inttypes.h> removido do topo do ficheiro .h.
   7. Comentário de protocolo em cada bloco de processamento.
   8. Vizinho próprio (POSTE_ID) ignorado em todas as mensagens
      recebidas — prevenção de loop de auto-echo.
   9. Timeout de vizinho com log mais informativo (delta em ms).
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

/* ── Estado interno ────────────────────────────────────────── */
static int        s_socket      = -1;
static bool       s_iniciado    = false;
static neighbor_t s_vizinhos[MAX_NEIGHBORS];
static uint32_t   s_ultimo_disc = 0;

/* Contadores de estatísticas de diagnóstico */
static udp_stats_t s_stats = {0};


/* ============================================================
   UTILITÁRIOS INTERNOS
============================================================ */

/* Retorna timestamp actual em milissegundos */
static uint32_t _agora_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* Converte neighbor_status_t em string para o protocolo UDP */
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

/* Converte string do protocolo em neighbor_status_t */
static neighbor_status_t _str_para_status(const char *s)
{
    if (!s)                    return NEIGHBOR_OFFLINE;
    if (!strcmp(s, "OK"))      return NEIGHBOR_OK;
    if (!strcmp(s, "FAIL"))    return NEIGHBOR_FAIL;
    if (!strcmp(s, "SAFE"))    return NEIGHBOR_SAFE;
    if (!strcmp(s, "AUTO"))    return NEIGHBOR_AUTO;
    if (!strcmp(s, "OBST"))    return NEIGHBOR_OBSTACULO;
    return NEIGHBOR_OFFLINE;
}


/* ============================================================
   _enviar_para — envio UDP unicast para IP específico
   Incrementa contador de pacotes enviados para estatísticas.
============================================================ */
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


/* ============================================================
   _encontrar_ou_criar_vizinho
   ──────────────────────────────────────────────────────────
   Procura vizinho por IP na tabela de vizinhos.
   Se não existir, cria entrada nova no primeiro slot livre.
   Actualiza ID e posição se fornecidos (>= 0).
============================================================ */
static neighbor_t *_encontrar_ou_criar_vizinho(const char *ip,
                                                int id, int pos)
{
    /* Procura vizinho existente por IP */
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (s_vizinhos[i].active &&
            strcmp(s_vizinhos[i].ip, ip) == 0) {
            if (id  >= 0) s_vizinhos[i].id       = id;
            if (pos >= 0) s_vizinhos[i].position  = pos;
            return &s_vizinhos[i];
        }
    }

    /* Cria novo vizinho no primeiro slot livre */
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

    ESP_LOGW(TAG, "Tabela cheia (MAX=%d) — vizinho %s ignorado",
             MAX_NEIGHBORS, ip);
    return NULL;
}


/* ============================================================
   _processar_mensagem
   ──────────────────────────────────────────────────────────
   Descodifica e despacha mensagem recebida.
   Todos os campos opcionais (x_mm) são retro-compatíveis:
   sscanf deixa o campo a 0 se ausente na mensagem.
============================================================ */
static void _processar_mensagem(char *msg, const char *ip)
{
    if (!msg || !ip) return;

    s_stats.pkts_recebidos++;

    /* ── DISCOVER:<id>:<pos> ─────────────────────────────────── */
    if (strncmp(msg, "DISCOVER:", 9) == 0) {
        int id = 0, pos = -1;
        sscanf(msg + 9, "%d:%d", &id, &pos);
        if (id == POSTE_ID) return;  /* Ignora o próprio broadcast */

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, pos);
        if (v) {
            v->last_seen   = _agora_ms();
            v->discover_ok = true;
            if (v->status == NEIGHBOR_OFFLINE) v->status = NEIGHBOR_OK;
        }

        /* Responde com STATUS imediato */
        char resp[32];
        snprintf(resp, sizeof(resp), "STATUS:%d:OK", POSTE_ID);
        _enviar_para(ip, resp);
        ESP_LOGD(TAG, "[RX] DISCOVER ID=%d pos=%d → STATUS:OK", id, pos);
        return;
    }

    /* ── STATUS:<id>:<estado> ────────────────────────────────── */
    if (strncmp(msg, "STATUS:", 7) == 0) {
        int  id = 0; char est[16] = {0};
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

    /* ── TC_INC:<id>:<vel>:<x_mm> ─────────────────────────────
       vel > 0  → veículo a caminho   (Tc++)
       vel < 0  → veículo passou      (T--)   [legado v4.x]
       vel == 0 → ignorado                                      */
    if (strncmp(msg, "TC_INC:", 7) == 0) {
        int id = 0, x_mm = 0; float vel = 0.0f;
        sscanf(msg + 7, "%d:%f:%d", &id, &vel, &x_mm);
        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, -1);
        if (v) v->last_seen = _agora_ms();

        s_stats.tc_inc_recebidos++;
        ESP_LOGD(TAG, "[RX] TC_INC ID=%d vel=%.0f x=%dmm", id, vel, x_mm);

        if (vel > 0.0f)
            on_tc_inc_received(vel, (int16_t)x_mm);   /* Tc++ */
        else if (vel < 0.0f)
            on_prev_passed_received();                  /* T--  */
        return;
    }

    /* ── PASSED:<id>:<vel> ───────────────────────────────────── */
    /* Mensagem dedicada de passagem (v5.0) — mais clara que TC_INC negativo */
    if (strncmp(msg, "PASSED:", 7) == 0) {
        int id = 0; float vel = 0.0f;
        sscanf(msg + 7, "%d:%f", &id, &vel);
        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, -1);
        if (v) v->last_seen = _agora_ms();

        ESP_LOGD(TAG, "[RX] PASSED ID=%d vel=%.0f", id, vel);
        on_prev_passed_received();
        return;
    }

    /* ── SPD:<id>:<vel>:<eta_ms>:<dist_m>:<x_mm> ────────────── */
    /* NOTA: usa unsigned long para sscanf com %lu no ESP32
       (evita mismatch de tipos com uint32_t). */
    if (strncmp(msg, "SPD:", 4) == 0) {
        int id = 0, x_mm = 0;
        float vel = 0.0f;
        unsigned long ul_eta = 0, ul_dist = 0;
        sscanf(msg + 4, "%d:%f:%lu:%lu:%d",
               &id, &vel, &ul_eta, &ul_dist, &x_mm);
        uint32_t eta_ms = (uint32_t)ul_eta;
        (void)ul_dist;  /* dist_m disponível mas não usado localmente */

        if (id == POSTE_ID) return;

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip, id, -1);
        if (v) v->last_seen = _agora_ms();

        ESP_LOGD(TAG, "[RX] SPD ID=%d vel=%.0f eta=%lums x=%dmm",
                 id, vel, (unsigned long)eta_ms, x_mm);
        on_spd_received(vel, eta_ms, (int16_t)x_mm);
        return;
    }

    /* ── MASTER_CLAIM:<id> ───────────────────────────────────── */
    if (strncmp(msg, "MASTER_CLAIM:", 13) == 0) {
        int id = atoi(msg + 13);
        if (id == POSTE_ID) return;

        ESP_LOGI(TAG, "[RX] MASTER_CLAIM de ID=%d", id);
        on_master_claim_received(id);
        return;
    }

    ESP_LOGD(TAG, "[RX] Mensagem desconhecida de %s: %.40s", ip, msg);
}


/* ============================================================
   _verificar_timeouts
   ──────────────────────────────────────────────────────────
   Marca OFFLINE os vizinhos silenciosos há NEIGHBOR_TIMEOUT_MS.
   Chamado a cada ciclo da udp_task — custo O(MAX_NEIGHBORS).
============================================================ */
static void _verificar_timeouts(uint32_t agora)
{
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!s_vizinhos[i].active)      continue;
        if (!s_vizinhos[i].discover_ok) continue;  /* Aguarda primeiro DISCOVER */

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
   udp_task_run — task principal UDP (Core 0, Prio 5)
   ──────────────────────────────────────────────────────────
   Loop:
     1. Aguarda socket válido (criado após WiFi ter IP)
     2. DISCOVER imediato
     3. Loop: recebe pacotes → processa → DISCOVER periódico
              → verifica timeouts → heartbeat
============================================================ */
void udp_task_run(void *arg)
{
    char               rx_buf[160];
    struct sockaddr_in origem;

    ESP_LOGI(TAG, "udp_task v5.0 | Core %d | Porto %d",
             xPortGetCoreID(), UDP_PORT);

    /* Aguarda socket válido — criado por comm_init() após WiFi.
       Sem este wait, o DISCOVER inicial é enviado com socket=-1. */
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

        /* 2. DISCOVER periódico a cada DISCOVER_INTERVAL_MS */
        if ((agora - s_ultimo_disc) >= DISCOVER_INTERVAL_MS) {
            udp_manager_discover();
            s_ultimo_disc = agora;
        }

        /* 3. Verifica timeouts de vizinhos */
        _verificar_timeouts(agora);

        /* 4. Heartbeat ao system_monitor */
        system_monitor_heartbeat(MOD_UDP);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


/* ============================================================
   udp_manager_init — cria socket e faz bind
   ──────────────────────────────────────────────────────────
   Não cria task — isso é feito por udp_manager_task_start().
   SO_BROADCAST: permite envio de DISCOVER em broadcast.
   SO_RCVTIMEO=10ms: recvfrom não bloqueia mais de 10ms.
============================================================ */
bool udp_manager_init(void)
{
    if (s_iniciado) { ESP_LOGW(TAG, "Já iniciado"); return true; }

    memset(s_vizinhos, 0, sizeof(s_vizinhos));
    memset(&s_stats,   0, sizeof(s_stats));
    s_ultimo_disc = 0;

    s_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_socket < 0) {
        ESP_LOGE(TAG, "Falha ao criar socket errno=%d", errno);
        return false;
    }

    /* Permite envio de DISCOVER em broadcast */
    int bc = 1;
    setsockopt(s_socket, SOL_SOCKET, SO_BROADCAST, &bc, sizeof(bc));

    /* Timeout de recepção: 10ms → loop não bloqueia */
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
    ESP_LOGI(TAG, "UDP v5.0 OK | porto %d | socket=%d", UDP_PORT, s_socket);
    return true;
}


/* ============================================================
   udp_manager_task_start — cria task no Core 0, Prio 5
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

/* Envia DISCOVER em broadcast para descoberta de vizinhos */
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

/* Envia TC_INC — anúncio de veículo a caminho com posição lateral */
bool udp_manager_send_tc_inc(const char *ip, float speed, int16_t x_mm)
{
    char msg[48];
    snprintf(msg, sizeof(msg), "TC_INC:%d:%.1f:%d",
             POSTE_ID, speed, (int)x_mm);
    bool ok = _enviar_para(ip, msg);
    if (ok) s_stats.tc_inc_enviados++;
    return ok;
}

/* Envia PASSED — mensagem dedicada de passagem de veículo (v5.0) */
bool udp_manager_send_passed(const char *ip, float speed)
{
    char msg[32];
    snprintf(msg, sizeof(msg), "PASSED:%d:%.1f", POSTE_ID, speed);
    return _enviar_para(ip, msg);
}

/* Envia SPD — velocidade, ETA e distância ao vizinho direito */
bool udp_manager_send_spd(const char *ip, float speed,
                           uint32_t eta_ms, uint32_t dist_m, int16_t x_mm)
{
    char msg[80];
    /* Usa %lu + (unsigned long) para compatibilidade com ESP32 */
    snprintf(msg, sizeof(msg), "SPD:%d:%.1f:%lu:%lu:%d",
             POSTE_ID, speed,
             (unsigned long)eta_ms,
             (unsigned long)dist_m,
             (int)x_mm);
    return _enviar_para(ip, msg);
}

/* Envia STATUS com estado actual deste poste */
bool udp_manager_send_status(const char *ip, neighbor_status_t status)
{
    char msg[32];
    snprintf(msg, sizeof(msg), "STATUS:%d:%s", POSTE_ID, _status_str(status));
    return _enviar_para(ip, msg);
}

/* Envia MASTER_CLAIM para propagar liderança em cadeia */
bool udp_manager_send_master_claim(const char *ip)
{
    char msg[24];
    snprintf(msg, sizeof(msg), "MASTER_CLAIM:%d", POSTE_ID);
    return _enviar_para(ip, msg);
}


/* ============================================================
   CONSULTA DE VIZINHOS
============================================================ */

/* Obtém IPs dos vizinhos esquerdo e direito como strings */
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

/* Procura vizinho pela posição na cadeia de postes */
neighbor_t *udp_manager_get_neighbor_by_pos(int position)
{
    for (int i = 0; i < MAX_NEIGHBORS; i++)
        if (s_vizinhos[i].active && s_vizinhos[i].position == position)
            return &s_vizinhos[i];
    return NULL;
}

/* Copia todos os vizinhos activos para array externo */
size_t udp_manager_get_all_neighbors(neighbor_t *list, size_t max)
{
    size_t n = 0;
    for (int i = 0; i < MAX_NEIGHBORS && n < max; i++)
        if (s_vizinhos[i].active) list[n++] = s_vizinhos[i];
    return n;
}

/* Força redescoberta de vizinho — reset de last_seen */
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

/* Copia estatísticas actuais para estrutura de saída */
void udp_manager_get_stats(udp_stats_t *out)
{
    if (out) *out = s_stats;
}

int udp_manager_get_socket(void)
{
    return s_socket;
}


/* ============================================================
   CALLBACKS WEAK — substituídos pela state_machine.c
   ──────────────────────────────────────────────────────────
   Se a state_machine não estiver ligada, as versões weak
   garantem que o programa compila e liga sem erros.
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
