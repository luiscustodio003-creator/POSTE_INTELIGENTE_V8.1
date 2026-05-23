/* ============================================================
   MÓDULO     : wifi_manager
   FICHEIRO   : wifi_manager.c — Gestão WiFi com eleição dinâmica de AP
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   Failover de AP:
     Se o AP (pos=0) cair, o poste vivo com menor POST_POSITION
     promove-se a AP após POST_POSITION × WIFI_AP_PROMOTE_BASE_MS.
     Quando o AP original regressa, o AP promovido demote-se a STA.
     POST_POSITION=0 é sempre AP — nunca demovido.
============================================================ */
#include "wifi_manager.h"
#include "display_manager.h"
#include "system_config.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "lwip/ip4_addr.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <string.h>

static const char *TAG = "WIFI_MGR";

/* ── Estado interno ──────────────────────────────────────── */
static bool     s_conectado    = false;  /* operacional: STA ligado OU é AP      */
static bool     s_modo_ap      = false;  /* actualmente em modo AP               */
static bool     s_promoted     = false;  /* AP por failover (não original)        */
static bool     s_demoting     = false;  /* sondagem de demoção em curso          */
static bool     s_demote_ok    = false;  /* flag: STA conectou durante sondagem   */
static int      s_retries      = 0;
static bool     s_wifi_enabled = true;
static char     s_ip[16]       = "---";

static uint64_t      s_disconnect_since_us = 0;  /* 1.ª desconexão STA (us)    */
static uint64_t      s_last_scan_us        = 0;  /* última sondagem de demoção  */
static esp_netif_t  *s_netif_sta           = NULL;
static esp_netif_t  *s_netif_ap            = NULL;

static portMUX_TYPE       s_ip_mux         = portMUX_INITIALIZER_UNLOCKED;
static esp_timer_handle_t s_reconect_timer = NULL;


/* ── _reconect_cb ────────────────────────────────────────── */
static void _reconect_cb(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Timer de reconexão disparado — a tentar ligar");
    s_retries = 0;
    esp_wifi_connect();
}


/* ── _apply_sta_ip — aplica IP estático no netif STA ────── */
static void _apply_sta_ip(void)
{
    if (!s_netif_sta) return;
    esp_netif_dhcpc_stop(s_netif_sta);
    esp_netif_ip_info_t ip_info = {0};
    IP4_ADDR(&ip_info.ip,      192, 168, 4, POST_POSITION + 1);
    IP4_ADDR(&ip_info.gw,      192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    esp_netif_set_ip_info(s_netif_sta, &ip_info);
}


/* ── _promote_to_ap ──────────────────────────────────────── */
static void _promote_to_ap(void)
{
    uint64_t elapsed_s = (esp_timer_get_time() - s_disconnect_since_us) / 1000000ULL;
    ESP_LOGW(TAG, "[AP FAILOVER] Promovendo pos=%d a AP (offline há %llus)",
             POST_POSITION, (unsigned long long)elapsed_s);

    if (s_reconect_timer) esp_timer_stop(s_reconect_timer);
    esp_wifi_disconnect();
    esp_wifi_stop();

    if (!s_netif_ap)
        s_netif_ap = esp_netif_create_default_wifi_ap();

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid_len       = (uint8_t)strlen(WIFI_SSID),
            .channel        = WIFI_AP_CHANNEL,
            .authmode       = WIFI_AUTH_WPA2_PSK,
            .max_connection = 10,
        }
    };
    strncpy((char *)ap_cfg.ap.ssid,     WIFI_SSID, sizeof(ap_cfg.ap.ssid) - 1);
    strncpy((char *)ap_cfg.ap.password, WIFI_PASS,  sizeof(ap_cfg.ap.password) - 1);

    /* IP do AP promovido = 192.168.4.1 (igual ao original).
       Outros postes não precisam alterar gateway nem IP fixo. */
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    esp_wifi_start();

    taskENTER_CRITICAL(&s_ip_mux);
    strncpy(s_ip, "192.168.4.1", sizeof(s_ip));
    taskEXIT_CRITICAL(&s_ip_mux);

    s_conectado           = true;
    s_modo_ap             = true;
    s_promoted            = true;
    s_disconnect_since_us = 0;
    s_retries             = 0;
    s_last_scan_us        = esp_timer_get_time();
    s_demoting            = false;
    s_demote_ok           = false;

    display_manager_set_wifi(true, s_ip);
    ESP_LOGW(TAG, "[AP FAILOVER] Sou AP: SSID=%s | 192.168.4.1", WIFI_SSID);
}


/* ── _try_demote_to_sta ──────────────────────────────────── */
static void _try_demote_to_sta(void)
{
    if (!s_netif_sta) {
        ESP_LOGE(TAG, "[AP FAILOVER] s_netif_sta NULL — demoção cancelada");
        s_last_scan_us = esp_timer_get_time();
        return;
    }

    ESP_LOGI(TAG, "[AP FAILOVER] pos=%d sondando AP original (pausa ~%ds)...",
             POST_POSITION, (WIFI_DEMOTE_RETRIES * 2));

    /* Stop/start necessário para activar STA em APSTA.
       AP clientes desconectam brevemente (~WIFI_DEMOTE_RETRIES × 2s). */
    esp_wifi_stop();

    wifi_config_t sta_cfg = {0};
    strncpy((char *)sta_cfg.sta.ssid,     WIFI_SSID, sizeof(sta_cfg.sta.ssid) - 1);
    strncpy((char *)sta_cfg.sta.password, WIFI_PASS,  sizeof(sta_cfg.sta.password) - 1);

    esp_wifi_set_mode(WIFI_MODE_APSTA);
    esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
    esp_wifi_start();
    /* WIFI_EVENT_STA_START dispara → handler chama esp_wifi_connect() */

    s_retries      = 0;
    s_demoting     = true;
    s_demote_ok    = false;
    s_last_scan_us = esp_timer_get_time();
}


/* ── _complete_demote ────────────────────────────────────── */
static void _complete_demote(void)
{
    ESP_LOGW(TAG, "[AP FAILOVER] Demoção completa — AP original restaurado, voltando a STA");

    /* Desactiva AP; clientes reencontram o AP original (mesmo SSID). */
    esp_wifi_stop();
    esp_wifi_set_mode(WIFI_MODE_STA);
    _apply_sta_ip();
    esp_wifi_start();
    /* STA_START → connect() → STA_CONNECTED → s_conectado=true */

    taskENTER_CRITICAL(&s_ip_mux);
    snprintf(s_ip, sizeof(s_ip), "192.168.4.%d", POST_POSITION + 1);
    taskEXIT_CRITICAL(&s_ip_mux);

    s_promoted  = false;
    s_modo_ap   = false;
    s_demoting  = false;
    s_demote_ok = false;
    s_conectado = false;  /* STA_CONNECTED irá setar */
    s_retries   = 0;
}


/* ── _abort_demote ───────────────────────────────────────── */
static void _abort_demote(void)
{
    ESP_LOGI(TAG, "[AP FAILOVER] Demoção abortada — AP original ainda ausente");

    esp_wifi_stop();
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_start();

    s_demoting     = false;
    s_retries      = 0;
    s_last_scan_us = esp_timer_get_time();
}


/* ── wifi_event_handler ──────────────────────────────────── */
static void wifi_event_handler(void *arg,
                                esp_event_base_t base,
                                int32_t          id,
                                void            *data)
{
    (void)arg;
    if (base != WIFI_EVENT) return;

    if (id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "STA iniciado — a tentar ligar ao AP");

    } else if (id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "%s | IP: 192.168.4.%d",
                 s_demoting ? "[APSTA] STA ligado durante sondagem" : "Ligado ao AP",
                 POST_POSITION + 1);

        taskENTER_CRITICAL(&s_ip_mux);
        snprintf(s_ip, sizeof(s_ip), "192.168.4.%d", POST_POSITION + 1);
        taskEXIT_CRITICAL(&s_ip_mux);

        s_retries             = 0;
        s_disconnect_since_us = 0;

        if (s_demoting) {
            /* AP original detectado — completa demoção em tick() (fora do handler). */
            s_demote_ok = true;
        } else {
            s_conectado = true;
            display_manager_set_wifi(true, s_ip);
        }

    } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_promoted && s_demoting) {
            /* Sondagem de demoção em APSTA — re-tentar (tick() aborta se esgotado). */
            if (s_retries < WIFI_DEMOTE_RETRIES) {
                s_retries++;
                esp_wifi_connect();
            }
        } else if (!s_promoted) {
            /* STA normal — registar 1.ª desconexão para timer de promoção. */
            if (s_disconnect_since_us == 0)
                s_disconnect_since_us = esp_timer_get_time();

            s_conectado = false;
            taskENTER_CRITICAL(&s_ip_mux);
            strncpy(s_ip, "---", sizeof(s_ip));
            taskEXIT_CRITICAL(&s_ip_mux);
            display_manager_set_wifi(false, NULL);

            if (s_retries < WIFI_RETRY_ATTEMPTS) {
                s_retries++;
                ESP_LOGI(TAG, "Desligado — retry %d/%d", s_retries, WIFI_RETRY_ATTEMPTS);
                esp_wifi_connect();
            } else {
                ESP_LOGW(TAG, "Retries esgotados — pausa %dms", WIFI_RECONNECT_MS);
                esp_timer_start_once(s_reconect_timer,
                                     (uint64_t)WIFI_RECONNECT_MS * 1000);
            }
        }

    } else if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *ev = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "[AP] Cliente conectado: " MACSTR, MAC2STR(ev->mac));

    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *ev = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGI(TAG, "[AP] Cliente desconectado: " MACSTR, MAC2STR(ev->mac));
    }
}


/* ── wifi_manager_init — STA com IP estático ────────────── */
void wifi_manager_init(void)
{
    esp_timer_create_args_t ta = { .callback = _reconect_cb, .name = "wifi_recon" };
    ESP_ERROR_CHECK(esp_timer_create(&ta, &s_reconect_timer));

    s_netif_sta = esp_netif_create_default_wifi_sta();
    _apply_sta_ip();

    ESP_LOGI(TAG, "IP fixo configurado: 192.168.4.%d (pos=%d)",
             POST_POSITION + 1, POST_POSITION);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));

    wifi_config_t wc = {0};
    strncpy((char *)wc.sta.ssid,     WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, WIFI_PASS,  sizeof(wc.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_modo_ap  = false;
    s_promoted = false;
    ESP_LOGI(TAG, "Wi-Fi STA | SSID: %s | IP: 192.168.4.%d", WIFI_SSID, POST_POSITION + 1);
}


/* ── wifi_manager_init_ap — AP permanente (pos=0) ───────── */
void wifi_manager_init_ap(void)
{
    s_netif_ap = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));

    wifi_config_t ap_config = {
        .ap = {
            .ssid            = WIFI_SSID,
            .ssid_len        = strlen(WIFI_SSID),
            .password        = WIFI_PASS,
            .channel         = WIFI_AP_CHANNEL,
            .authmode        = WIFI_AUTH_WPA2_PSK,
            .max_connection  = 10,
        }
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    snprintf(s_ip, sizeof(s_ip), "192.168.4.1");
    s_conectado = true;
    s_modo_ap   = true;
    s_promoted  = false;  /* AP original — imutável */

    display_manager_set_wifi(true, s_ip);
    ESP_LOGI(TAG, "[AP] Rede criada: %s | IP: 192.168.4.1", WIFI_SSID);
}


/* ── wifi_manager_init_auto ─────────────────────────────── */
void wifi_manager_init_auto(void)
{
    if (POST_POSITION == 0)
        wifi_manager_init_ap();
    else
        wifi_manager_init();
}


/* ── wifi_manager_tick — motor de failover (chamar ~200ms) ──
   Promoção STA→AP e demoção AP→STA sem depender do poste 0. */
void wifi_manager_tick(void)
{
    if (!s_wifi_enabled)          return;
    if (s_modo_ap && !s_promoted) return;  /* AP original (pos=0) — nunca toca */

    uint64_t now_us = esp_timer_get_time();

    /* ── Concluir demoção (STA conectou durante sondagem APSTA) ── */
    if (s_demote_ok) {
        _complete_demote();
        return;
    }

    /* ── Abortar sondagem (WIFI_DEMOTE_RETRIES esgotados) ──────── */
    if (s_demoting && s_retries >= WIFI_DEMOTE_RETRIES) {
        _abort_demote();
        return;
    }

    /* ── Promoção: STA desconectado há demasiado tempo ─────────── */
    if (!s_modo_ap && !s_conectado && s_disconnect_since_us > 0) {
        uint64_t promote_us = (uint64_t)POST_POSITION * WIFI_AP_PROMOTE_BASE_MS * 1000ULL;
        if ((now_us - s_disconnect_since_us) >= promote_us) {
            _promote_to_ap();
            return;
        }
    }

    /* ── Sondagem periódica de demoção (AP promovido) ──────────── */
    if (s_promoted && !s_demoting) {
        if ((now_us - s_last_scan_us) >= WIFI_AP_SCAN_INTERVAL_MS * 1000ULL) {
            _try_demote_to_sta();
        }
    }
}


/* ── Getters ─────────────────────────────────────────────── */
bool        wifi_manager_is_connected(void)   { return s_conectado; }
const char *wifi_manager_get_ip(void)         { return s_ip; }
bool        wifi_manager_is_enabled(void)     { return s_wifi_enabled; }
bool        wifi_manager_is_promoted_ap(void) { return s_promoted; }


/* ── wifi_manager_disable — desliga WiFi em SAFE MODE ────── */
void wifi_manager_disable(void)
{
    if (!s_wifi_enabled) {
        ESP_LOGW(TAG, "WiFi já estava desligado");
        return;
    }

    ESP_LOGW(TAG, "DESLIGANDO WiFi (SAFE MODE)");

    /* Se demoção estava em curso (modo APSTA activo), repõe AP antes de parar.
       Garante que o próximo esp_wifi_start() (em enable()) arranca em modo AP
       e não em APSTA, evitando conflito de DHCP com P0. */
    if (s_demoting)
        esp_wifi_set_mode(WIFI_MODE_AP);

    if (s_reconect_timer) esp_timer_stop(s_reconect_timer);
    esp_wifi_stop();

    s_wifi_enabled        = false;
    s_conectado           = false;
    s_disconnect_since_us = 0;  /* reset: não promover enquanto radar offline */
    s_demoting            = false;
    s_demote_ok           = false; /* anula resultado de sondagem pendente */

    taskENTER_CRITICAL(&s_ip_mux);
    strncpy(s_ip, "OFFLINE", sizeof(s_ip));
    taskEXIT_CRITICAL(&s_ip_mux);

    display_manager_set_wifi(false, NULL);
    ESP_LOGW(TAG, "WiFi desligado — poste isolado da rede");
}


/* ── wifi_manager_enable — religa WiFi após recuperação ──── */
void wifi_manager_enable(void)
{
    if (s_wifi_enabled) {
        ESP_LOGI(TAG, "WiFi já estava ligado");
        return;
    }

    ESP_LOGI(TAG, "RELIGANDO WiFi (radar recuperado)");

    /* Defesa em profundidade: garante modo AP antes de start,
       caso disable() tenha sido chamado antes do reset de modo ocorrer. */
    if (s_modo_ap)
        esp_wifi_set_mode(WIFI_MODE_AP);

    esp_wifi_start();
    s_wifi_enabled = true;
    s_retries      = 0;

    if (s_modo_ap) {
        /* AP (original ou promovido) — volta imediatamente operacional. */
        s_conectado = true;
        snprintf(s_ip, sizeof(s_ip), "192.168.4.1");
        display_manager_set_wifi(true, s_ip);
        if (s_promoted) {
            /* Força sondagem imediata: AP original pode ter regressado durante SAFE_MODE. */
            s_last_scan_us = 0;
        }
    }
    /* STA: STA_START → connect() → se falhar, s_disconnect_since_us regista */

    ESP_LOGI(TAG, "WiFi religado — a restaurar conectividade");
}
