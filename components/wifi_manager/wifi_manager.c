#include <inttypes.h>
/* ============================================================
   WIFI MANAGER — IMPLEMENTAÇÃO
   @file      wifi_manager.c
   @version   1.5  |  2026-04-07
   Projecto  : Poste Inteligente v8
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)
============================================================ */
#include "wifi_manager.h"
#include "display_manager.h"
#include "system_config.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <string.h>

static const char *TAG = "WIFI_MGR";

/* ── Estado interno ───────────────────────────────────────── */
static bool s_conectado   = false;
static char s_ip[16]      = "---";
static int  s_retries     = 0;
static bool s_modo_ap     = false;

/* Spinlock para proteger s_ip (acedido de event handler e monitor) */
static portMUX_TYPE s_ip_mux = portMUX_INITIALIZER_UNLOCKED;

/* ── Timer de reconexão após falha de retries ─────────────── */
static esp_timer_handle_t s_reconect_timer = NULL;

static void _reconect_cb(void *arg)
{
    ESP_LOGI(TAG, "Timer de reconexão disparado — a tentar ligar");
    s_retries = 0;
    esp_wifi_connect();
}

/* ============================================================
   wifi_event_handler
   Handler de eventos Wi-Fi e IP registado no event loop.
============================================================ */
static void wifi_event_handler(void *arg,
                                esp_event_base_t base,
                                int32_t          id,
                                void            *data)
{
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
            ESP_LOGI(TAG, "STA iniciado — a tentar ligar");

        } else if (id == WIFI_EVENT_STA_CONNECTED) {
            ESP_LOGI(TAG, "Ligado — aguarda IP");

        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            s_conectado = false;
            taskENTER_CRITICAL(&s_ip_mux);
            strncpy(s_ip, "---", sizeof(s_ip));
            taskEXIT_CRITICAL(&s_ip_mux);
            display_manager_set_wifi(false, NULL);

            if (s_retries < WIFI_RETRY_ATTEMPTS) {
                s_retries++;
                ESP_LOGI(TAG, "Desligado — retry %d/%d",
                         s_retries, WIFI_RETRY_ATTEMPTS);
                esp_wifi_connect();
            } else {
                ESP_LOGW(TAG, "Retries esgotados — pausa %dms",
                         WIFI_RECONNECT_MS);
                esp_timer_start_once(s_reconect_timer,
                                     (uint64_t)WIFI_RECONNECT_MS * 1000);
            }
        }

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        taskENTER_CRITICAL(&s_ip_mux);
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        taskEXIT_CRITICAL(&s_ip_mux);

        s_conectado = true;
        s_retries   = 0;
        ESP_LOGI(TAG, "IP obtido: %s", s_ip);
        display_manager_set_wifi(true, s_ip);
    }
}

/* ============================================================
   wifi_manager_init
============================================================ */
void wifi_manager_init(void)
{
    /* Timer de reconexão (one-shot) */
    esp_timer_create_args_t ta = {
        .callback = _reconect_cb,
        .name     = "wifi_recon",
    };
    ESP_ERROR_CHECK(esp_timer_create(&ta, &s_reconect_timer));

    /* Cria interface de rede STA */
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Regista handlers */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    /* Configura credenciais */
    wifi_config_t wc = {0};
    strncpy((char *)wc.sta.ssid,     WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, WIFI_PASS,  sizeof(wc.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi STA iniciado | SSID: %s", WIFI_SSID);
}

/* ── API ──────────────────────────────────────────────────── */
bool wifi_manager_is_connected(void) { return s_conectado; }

const char *wifi_manager_get_ip(void)
{
    /* Leitura sem lock — string pequena, worst case = leitura
     * parcialmente actualizada, aceitável para display */
    return s_ip;
}

void wifi_manager_reset_retry(void)
{
    s_retries = 0;
    esp_wifi_connect();
}


void wifi_manager_init_auto(void)
{
    if (POST_POSITION == 0) {
        /* Sou o MASTER — crio o AP */
        wifi_manager_assume_ap();
    } else {
        /* Sou IDLE — ligo ao AP do MASTER */
        /* Usa o código actual mas com WIFI_AP_SSID/PASS */
        wifi_manager_init();
    }
}



/* Nova função — cria AP com DHCP interno */
void wifi_manager_assume_ap(void)
{
    /* Para o modo STA actual */
    esp_wifi_stop();
    esp_wifi_deinit();

    /* Cria interface AP */
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

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

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "[AP] Rede criada: %s", WIFI_SSID);
    s_modo_ap = true;
}
