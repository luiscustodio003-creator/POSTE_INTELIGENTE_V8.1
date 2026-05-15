/* ============================================================
   WEB DATA PROVIDER — AGREGADOR DE DADOS
   @file      web_data_provider.c
   @version   2.0  |  2026-05-14
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custodio | Tiago Moreno

   ALTERAÇÕES v1.1 → v2.0:
   ─────────────────────────
   - ADICIONADO: rastreio horário nocturno (20h-7h) com 11 buckets
   - ADICIONADO: timer periódico 10s — acumula dados sem browser activo
   - ADICIONADO: web_data_get_night_stats() e web_data_get_status()
   - CORRIGIDO: energia integra duty real (dali_get_brightness) —
     inclui períodos de fade-up e fade-down
   - CORRIGIDO: extern g_fsm_right_online → comm_right_online()
   - CORRIGIDO: LED_POWER_W usa system_config.h LED_POWER_WATT
============================================================ */

#include "web_data_provider.h"
#include "state_machine.h"
#include "fsm_core.h"
#include "comm_manager.h"
#include "dali_manager.h"
#include "wifi_manager.h"
#include "udp_manager.h"
#include "radar_manager.h"
#include "post_config.h"
#include "system_config.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include <time.h>
#include <string.h>

static const char *TAG = "WEB_DATA";

/* ── Configuração ─────────────────────────────────────────── */
#define LED_POWER_W     ((float)LED_POWER_WATT)

/* ── Estado de distribuição de tempo ─────────────────────── */
static uint32_t time_in_save_s = 0;
static uint32_t time_in_min_s  = 0;
static uint32_t time_in_on_s   = 0;

/* ── Rastreio de energia e tempo ─────────────────────────── */
static uint64_t s_last_us      = 0;   /* último tick (µs desde boot)    */

/* ── Rastreio nocturno ────────────────────────────────────── */
static night_bucket_t s_night[NIGHT_BUCKETS];
static bool           s_night_init   = false;
static int            s_last_bucket  = -2;  /* -2 = não inicializado    */
static int            s_last_T       = 0;

/* ── Timer periódico ──────────────────────────────────────── */
static esp_timer_handle_t s_timer = NULL;


/* ── Auxiliares ───────────────────────────────────────────── */

static bool _time_synced(void)
{
    return time(NULL) > 1000000000L;  /* > ano 2001 = sincronizado */
}

static int _current_hour(void)
{
    time_t t; time(&t);
    struct tm tm; localtime_r(&t, &tm);
    return tm.tm_hour;
}

static int _hour_to_bucket(int h)
{
    if (h >= NIGHT_START_HOUR)             return h - NIGHT_START_HOUR;
    if (h <  NIGHT_END_HOUR)               return h + (24 - NIGHT_START_HOUR);
    return -1;  /* período diurno */
}

static const char *_get_role(void)
{
    return comm_is_master() ? "MASTER" : "SLAVE";
}

static void _init_night_buckets(void)
{
    memset(s_night, 0, sizeof(s_night));
    for (int i = 0; i < NIGHT_BUCKETS; i++) {
        int h = (NIGHT_START_HOUR + i) % 24;
        s_night[i].hour = (uint8_t)h;
        snprintf(s_night[i].label, sizeof(s_night[i].label), "%dh", h);
    }
    s_last_T = state_machine_get_T();
}

/* ── update_time_counters ─────────────────────────────────────
   Integra energia e veículos. Chamada pelo timer (10s) e a cada
   pedido à API. Thread-safe: lê atomics, acumula em static.
──────────────────────────────────────────────────────────── */
static void update_time_counters(void)
{
    uint64_t now_us = esp_timer_get_time();

    if (s_last_us == 0) {
        s_last_us = now_us;
        _init_night_buckets();
        return;
    }

    uint64_t delta_us = now_us - s_last_us;
    if (delta_us < 100000ULL) return;  /* < 100ms, ignorar */

    float delta_s = (float)delta_us / 1000000.0f;
    s_last_us = now_us;

    /* Duty actual (inclui estado de fade, porque dali_get_brightness()
       devolve o destino do fade actual)                               */
    uint8_t duty = dali_get_brightness();

    /* Distribuição de tempo por modo */
    uint32_t dsec = (uint32_t)delta_s;
    if      (duty <= 15) time_in_save_s += dsec;
    else if (duty <= 60) time_in_min_s  += dsec;
    else                 time_in_on_s   += dsec;

    /* Energia (Wh) para este intervalo */
    float dwh = ((float)duty / 100.0f) * LED_POWER_W * delta_s / 3600.0f;

    /* ── Rastreio nocturno ────────────────────────────────── */
    if (_time_synced()) {
        int h      = _current_hour();
        int bucket = _hour_to_bucket(h);

        /* Reiniciar buckets no início de cada noite (transição para bucket 0) */
        if (!s_night_init) {
            _init_night_buckets();
            s_night_init = true;
        } else if (bucket == 0 && s_last_bucket > 0) {
            /* Novo dia: bucket 0 (20h) depois de bucket >0 — nova noite */
            ESP_LOGI(TAG, "Nova noite detectada — a reiniciar buckets");
            _init_night_buckets();
        }

        if (bucket >= 0 && bucket < NIGHT_BUCKETS) {
            /* Veículos: delta de T desde o último tick */
            int T_now = state_machine_get_T();
            if (T_now > s_last_T) {
                s_night[bucket].vehicles += (uint32_t)(T_now - s_last_T);
            }
            s_last_T = T_now;

            /* Energia no bucket actual */
            s_night[bucket].energy_wh += dwh;
            s_last_bucket = bucket;
        } else {
            /* Período diurno — apenas actualizar s_last_T */
            s_last_T = state_machine_get_T();
        }
    }
}

static void _timer_cb(void *arg)
{
    (void)arg;
    update_time_counters();
}


/* ══════════════════════════════════════════════════════════
   API PÚBLICA
══════════════════════════════════════════════════════════ */

void web_data_provider_init(void)
{
    ESP_LOGI(TAG, "A iniciar agregador de dados v2.0...");

    s_last_us    = esp_timer_get_time();
    s_last_T     = state_machine_get_T();
    s_night_init = false;

    /* Timer periódico: actualiza buckets mesmo sem pedidos HTTP */
    esp_timer_create_args_t ta = {
        .callback = _timer_cb,
        .name     = "web_data",
    };
    esp_timer_create(&ta, &s_timer);
    esp_timer_start_periodic(s_timer, 10000000ULL);  /* 10s em µs */

    ESP_LOGI(TAG, "Agregador pronto | LED=%dW | Noite=%dh-%dh | Timer 10s",
             LED_POWER_WATT, NIGHT_START_HOUR, NIGHT_END_HOUR);
}

/* ── web_data_get_status ────────────────────────────────────
   JSON plano com estado actual do poste — para /api/status.
──────────────────────────────────────────────────────────── */
cJSON *web_data_get_status(void)
{
    update_time_counters();

    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;

    /* Identidade */
    cJSON_AddStringToObject(root, "name",  post_get_name());
    cJSON_AddNumberToObject(root, "id",    post_get_id());

    /* Estado FSM */
    cJSON_AddStringToObject(root, "state", state_machine_get_state_name());
    cJSON_AddNumberToObject(root, "duty",  fsm_core_get_duty_cycle());
    cJSON_AddNumberToObject(root, "T",     state_machine_get_T());
    cJSON_AddNumberToObject(root, "Tc",    state_machine_get_Tc());

    /* Radar */
    cJSON_AddStringToObject(root, "radar",    radar_get_status_str());
    cJSON_AddBoolToObject  (root, "radar_ok", state_machine_radar_ok());

    /* Rede */
    const char *ip = wifi_manager_get_ip();
    cJSON_AddStringToObject(root, "ip",      ip ? ip : "---");
    cJSON_AddStringToObject(root, "role",    _get_role());
    cJSON_AddBoolToObject  (root, "wifi_ok", wifi_manager_is_connected());

    /* Vizinhos */
    char nL[MAX_IP_LEN] = {0}, nR[MAX_IP_LEN] = {0};
    udp_manager_get_neighbors(nL, nR);
    cJSON_AddStringToObject(root, "neb_l_ip", nL[0] ? nL : "---");
    cJSON_AddBoolToObject  (root, "neb_l_ok", comm_left_online());
    cJSON_AddStringToObject(root, "neb_r_ip", nR[0] ? nR : "---");
    cJSON_AddBoolToObject  (root, "neb_r_ok", comm_right_online());

    /* Sistema */
    cJSON_AddNumberToObject(root, "uptime_s",
                            (double)(esp_timer_get_time() / 1000000ULL));
    cJSON_AddNumberToObject(root, "free_kb",
                            (int)(esp_get_free_heap_size() / 1024));

    return root;
}

/* ── web_data_get_night_stats ───────────────────────────────
   JSON com estatísticas horárias do período nocturno.
──────────────────────────────────────────────────────────── */
cJSON *web_data_get_night_stats(void)
{
    update_time_counters();

    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;

    bool synced = _time_synced();
    int  h_now  = synced ? _current_hour() : -1;

    cJSON_AddBoolToObject  (root, "synced", synced);
    cJSON_AddNumberToObject(root, "hour",   h_now);

    /* Array de buckets */
    cJSON *arr = cJSON_CreateArray();
    uint32_t tot_veh = 0;
    float    tot_wh  = 0.0f;

    for (int i = 0; i < NIGHT_BUCKETS; i++) {
        cJSON *b = cJSON_CreateObject();
        cJSON_AddNumberToObject(b, "h", s_night[i].hour);
        cJSON_AddStringToObject(b, "l", s_night[i].label);
        cJSON_AddNumberToObject(b, "v", (int)s_night[i].vehicles);
        cJSON_AddNumberToObject(b, "e", (double)s_night[i].energy_wh);
        cJSON_AddItemToArray(arr, b);
        tot_veh += s_night[i].vehicles;
        tot_wh  += s_night[i].energy_wh;
    }

    cJSON_AddItemToObject  (root, "buckets", arr);
    cJSON_AddNumberToObject(root, "veh",     (int)tot_veh);
    cJSON_AddNumberToObject(root, "wh",      (double)tot_wh);

    return root;
}

/* ── Funções existentes (mantidas para compatibilidade) ─────────────────── */

cJSON *web_data_get_line_status(void)
{
    update_time_counters();

    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;

    /* system */
    cJSON *sys = cJSON_CreateObject();
    cJSON_AddNumberToObject(sys, "uptime_s",
                            (double)(esp_timer_get_time() / 1000000ULL));
    cJSON_AddNumberToObject(sys, "free_heap_kb",
                            (int)(esp_get_free_heap_size() / 1024));
    cJSON_AddItemToObject(root, "system", sys);

    /* topology */
    cJSON *topo = cJSON_CreateObject();
    cJSON_AddNumberToObject(topo, "master_position",
                            comm_is_master() ? POST_POSITION : -1);
    cJSON_AddNumberToObject(topo, "active_slaves",
                            (comm_left_online() ? 1 : 0) +
                            (comm_right_online() ? 1 : 0));
    cJSON_AddItemToObject(root, "topology", topo);

    /* postes */
    cJSON *postes = cJSON_CreateArray();
    cJSON *p = cJSON_CreateObject();
    const char *ip = wifi_manager_get_ip();
    cJSON_AddNumberToObject(p, "position",  POST_POSITION);
    cJSON_AddStringToObject(p, "ip",        ip ? ip : "---");
    cJSON_AddBoolToObject  (p, "is_online", true);
    cJSON_AddStringToObject(p, "role",      _get_role());
    cJSON_AddStringToObject(p, "state",     state_machine_get_state_name());
    cJSON_AddNumberToObject(p, "T",         state_machine_get_T());
    cJSON_AddNumberToObject(p, "Tc",        state_machine_get_Tc());
    cJSON_AddNumberToObject(p, "duty_cycle",fsm_core_get_duty_cycle());
    cJSON_AddItemToArray(postes, p);
    cJSON_AddItemToObject(root, "postes", postes);

    /* stats */
    cJSON *stats = cJSON_CreateObject();
    energy_stats_t energy; web_data_get_energy_stats(0, 0, &energy);
    cJSON_AddNumberToObject(stats, "total_vehicles",
                            state_machine_get_T() + state_machine_get_Tc());
    cJSON_AddNumberToObject(stats, "total_energy",         energy.consumed_kwh);
    cJSON_AddNumberToObject(stats, "energy_saved_percent", energy.saved_percent);
    cJSON_AddItemToObject(root, "stats", stats);

    return root;
}

cJSON *web_data_get_poste_status(uint8_t position)
{
    (void)position;  /* apenas dados do próprio poste */
    update_time_counters();

    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;

    const char *ip = wifi_manager_get_ip();
    cJSON_AddNumberToObject(root, "position",  POST_POSITION);
    cJSON_AddStringToObject(root, "ip",        ip ? ip : "---");
    cJSON_AddStringToObject(root, "role",      _get_role());
    cJSON_AddStringToObject(root, "state",     state_machine_get_state_name());
    cJSON_AddNumberToObject(root, "T",         state_machine_get_T());
    cJSON_AddNumberToObject(root, "Tc",        state_machine_get_Tc());
    cJSON_AddNumberToObject(root, "duty_cycle",fsm_core_get_duty_cycle());

    /* neighbors */
    cJSON *nbs = cJSON_CreateArray();
    char nL[MAX_IP_LEN] = {0}, nR[MAX_IP_LEN] = {0};
    udp_manager_get_neighbors(nL, nR);
    if (comm_left_online()) {
        cJSON *n = cJSON_CreateObject();
        cJSON_AddNumberToObject(n, "position", POST_POSITION > 0 ? POST_POSITION - 1 : 0);
        cJSON_AddStringToObject(n, "ip",       nL[0] ? nL : "---");
        cJSON_AddBoolToObject  (n, "is_alive", true);
        cJSON_AddStringToObject(n, "side",     "left");
        cJSON_AddItemToArray(nbs, n);
    }
    if (comm_right_online()) {
        cJSON *n = cJSON_CreateObject();
        cJSON_AddNumberToObject(n, "position", POST_POSITION + 1);
        cJSON_AddStringToObject(n, "ip",       nR[0] ? nR : "---");
        cJSON_AddBoolToObject  (n, "is_alive", true);
        cJSON_AddStringToObject(n, "side",     "right");
        cJSON_AddItemToArray(nbs, n);
    }
    cJSON_AddItemToObject(root, "neighbors", nbs);

    /* time_stats */
    time_distribution_t td; web_data_get_time_distribution(0, 0, &td);
    cJSON *ts = cJSON_CreateObject();
    cJSON_AddNumberToObject(ts, "save_seconds", td.save_seconds);
    cJSON_AddNumberToObject(ts, "save_percent", td.save_percent);
    cJSON_AddNumberToObject(ts, "min_seconds",  td.min_seconds);
    cJSON_AddNumberToObject(ts, "min_percent",  td.min_percent);
    cJSON_AddNumberToObject(ts, "on_seconds",   td.on_seconds);
    cJSON_AddNumberToObject(ts, "on_percent",   td.on_percent);
    cJSON_AddItemToObject(root, "time_stats", ts);

    /* energy */
    energy_stats_t energy; web_data_get_energy_stats(0, 0, &energy);
    cJSON *eo = cJSON_CreateObject();
    cJSON_AddNumberToObject(eo, "consumed_kwh", energy.consumed_kwh);
    cJSON_AddNumberToObject(eo, "full_on_kwh",  energy.full_on_kwh);
    cJSON_AddNumberToObject(eo, "saved_kwh",    energy.saved_kwh);
    cJSON_AddNumberToObject(eo, "saved_percent",energy.saved_percent);
    cJSON_AddNumberToObject(eo, "power_w",      energy.power_w);
    cJSON_AddItemToObject(root, "energy", eo);

    return root;
}

void web_data_get_time_distribution(uint32_t start_ts, uint32_t end_ts,
                                    time_distribution_t *out)
{
    (void)start_ts; (void)end_ts;
    if (!out) return;
    update_time_counters();

    out->save_seconds = time_in_save_s;
    out->min_seconds  = time_in_min_s;
    out->on_seconds   = time_in_on_s;

    uint32_t total = time_in_save_s + time_in_min_s + time_in_on_s;
    if (total > 0) {
        out->save_percent = (time_in_save_s * 100.0f) / total;
        out->min_percent  = (time_in_min_s  * 100.0f) / total;
        out->on_percent   = (time_in_on_s   * 100.0f) / total;
    } else {
        out->save_percent = out->min_percent = out->on_percent = 0.0f;
    }
}

void web_data_get_energy_stats(uint32_t start_ts, uint32_t end_ts,
                               energy_stats_t *out)
{
    (void)start_ts; (void)end_ts;
    if (!out) return;
    update_time_counters();

    float save_h = time_in_save_s / 3600.0f;
    float min_h  = time_in_min_s  / 3600.0f;
    float on_h   = time_in_on_s   / 3600.0f;

    out->consumed_kwh = (LED_POWER_W * 0.10f * save_h +
                         LED_POWER_W * 0.50f * min_h  +
                         LED_POWER_W * 1.00f * on_h)  / 1000.0f;
    float total_h     = save_h + min_h + on_h;
    out->full_on_kwh  = LED_POWER_W * total_h / 1000.0f;
    out->saved_kwh    = out->full_on_kwh - out->consumed_kwh;
    out->saved_percent = (out->full_on_kwh > 0.001f)
                         ? (out->saved_kwh / out->full_on_kwh) * 100.0f : 0.0f;
    out->power_w = (uint16_t)LED_POWER_W;
}

uint8_t web_data_get_neighbors(neighbor_info_t *neighbors, uint8_t max)
{
    if (!neighbors || max == 0) return 0;
    uint8_t count = 0;
    char nL[MAX_IP_LEN] = {0}, nR[MAX_IP_LEN] = {0};
    udp_manager_get_neighbors(nL, nR);

    if (comm_left_online() && count < max) {
        neighbors[count].position    = POST_POSITION > 0 ? POST_POSITION - 1 : 0;
        strncpy(neighbors[count].ip, nL[0] ? nL : "---", MAX_IP_LEN - 1);
        neighbors[count].is_alive    = true;
        neighbors[count].last_seen_ts = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        count++;
    }
    if (comm_right_online() && count < max) {
        neighbors[count].position    = POST_POSITION + 1;
        strncpy(neighbors[count].ip, nR[0] ? nR : "---", MAX_IP_LEN - 1);
        neighbors[count].is_alive    = true;
        neighbors[count].last_seen_ts = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        count++;
    }
    return count;
}

void web_data_reset_stats(void)
{
    time_in_save_s = 0;
    time_in_min_s  = 0;
    time_in_on_s   = 0;
    memset(s_night, 0, sizeof(s_night));
    s_night_init  = false;
    s_last_us     = esp_timer_get_time();
    ESP_LOGI(TAG, "Estatísticas reiniciadas");
}
