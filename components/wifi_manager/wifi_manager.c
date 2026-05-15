/* wifi_manager.c — v2.1 | 2026-05-14 | Poste Inteligente v8
   pos=0 → AP permanente (192.168.4.1)
   pos>0 → STA permanente (192.168.4.pos+1), IP estático, sem DHCP. */

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

static bool s_conectado    = false;
static char s_ip[16]       = "---";
static int  s_retries      = 0;
static bool s_modo_ap      = false;
static bool s_wifi_enabled = true;

/* Spinlock para s_ip (acedido de event handler e monitor). */
static portMUX_TYPE s_ip_mux = portMUX_INITIALIZER_UNLOCKED;

static esp_timer_handle_t s_reconect_timer = NULL;


static void _reconect_cb(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Timer de reconexão disparado — a tentar ligar");
    s_retries = 0;
    esp_wifi_connect();
}


static void wifi_event_handler(void *arg,
                                esp_event_base_t base,
                                int32_t          id,
                                void            *data)
{
    (void)arg;

    if (base == WIFI_EVENT) {

        if (id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
            ESP_LOGI(TAG, "STA iniciado — a tentar ligar ao AP");

        } else if (id == WIFI_EVENT_STA_CONNECTED) {
            ESP_LOGI(TAG, "Ligado ao AP — IP fixo: 192.168.4.%d", POST_POSITION + 1);
            taskENTER_CRITICAL(&s_ip_mux);
            snprintf(s_ip, sizeof(s_ip), "192.168.4.%d", POST_POSITION + 1);
            taskEXIT_CRITICAL(&s_ip_mux);
            s_conectado = true;
            s_retries = 0;
            display_manager_set_wifi(true, s_ip);

        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
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

        } else if (id == WIFI_EVENT_AP_STACONNECTED) {
            wifi_event_ap_staconnected_t *ev = (wifi_event_ap_staconnected_t *)data;
            ESP_LOGI(TAG, "[AP] Cliente conectado: " MACSTR, MAC2STR(ev->mac));

        } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
            wifi_event_ap_stadisconnected_t *ev = (wifi_event_ap_stadisconnected_t *)data;
            ESP_LOGI(TAG, "[AP] Cliente desconectado: " MACSTR, MAC2STR(ev->mac));
        }
    }
    /* IP estático — não há IP_EVENT_STA_GOT_IP. */
}


/* ── wifi_manager_init — STA com IP estático ────────────────
   Chamado por postes com POST_POSITION > 0. */
void wifi_manager_init(void)
{
    esp_timer_create_args_t ta = {
        .callback = _reconect_cb,
        .name     = "wifi_recon",
    };
    ESP_ERROR_CHECK(esp_timer_create(&ta, &s_reconect_timer));

    esp_netif_t *netif = esp_netif_create_default_wifi_sta();

    esp_netif_dhcpc_stop(netif);

    esp_netif_ip_info_t ip_info = {0};
    IP4_ADDR(&ip_info.ip,      192, 168, 4, POST_POSITION + 1);
    IP4_ADDR(&ip_info.gw,      192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &ip_info));

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

    s_modo_ap = false;
    ESP_LOGI(TAG, "Wi-Fi STA iniciado | SSID: %s | IP: 192.168.4.%d",
             WIFI_SSID, POST_POSITION + 1);
}


/* ── wifi_manager_init_ap — AP permanente (pos=0) ────────── */
void wifi_manager_init_ap(void)
{
    esp_netif_create_default_wifi_ap();

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

    display_manager_set_wifi(true, s_ip);
    ESP_LOGI(TAG, "[AP] Rede criada: %s | IP: 192.168.4.1", WIFI_SSID);
}


/* ── wifi_manager_init_auto — escolhe AP ou STA por posição── */
void wifi_manager_init_auto(void)
{
    if (POST_POSITION == 0)
        wifi_manager_init_ap();
    else
        wifi_manager_init();
}


/* ── Getters ──────────────────────────────────────────────── */

bool wifi_manager_is_connected(void)
{
    return s_conectado;
}

const char *wifi_manager_get_ip(void)
{
    return s_ip;
}


/* ── wifi_manager_disable — desliga WiFi em SAFE MODE ─────── */
void wifi_manager_disable(void)
{
    if (!s_wifi_enabled) {
        ESP_LOGW(TAG, "WiFi já estava desligado");
        return;
    }

    ESP_LOGW(TAG, "DESLIGANDO WiFi (SAFE MODE - radar offline)");

    esp_wifi_stop();

    s_wifi_enabled = false;
    s_conectado    = false;

    taskENTER_CRITICAL(&s_ip_mux);
    strncpy(s_ip, "OFFLINE", sizeof(s_ip));
    taskEXIT_CRITICAL(&s_ip_mux);

    display_manager_set_wifi(false, NULL);
    ESP_LOGW(TAG, "WiFi desligado — poste isolado da rede");
}


/* ── wifi_manager_enable — religa WiFi após recuperação ─────
   esp_wifi_start() dispara STA_START → handler chama connect(). */
void wifi_manager_enable(void)
{
    if (s_wifi_enabled) {
        ESP_LOGI(TAG, "WiFi já estava ligado");
        return;
    }

    ESP_LOGI(TAG, "RELIGANDO WiFi (radar recuperado)");

    esp_wifi_start();

    s_wifi_enabled = true;
    s_retries      = 0;

    if (s_modo_ap) {
        s_conectado = true;
        snprintf(s_ip, sizeof(s_ip), "192.168.4.1");
        display_manager_set_wifi(true, s_ip);
    }

    ESP_LOGI(TAG, "WiFi religado — a restaurar conectividade");
}


bool wifi_manager_is_enabled(void)
{
    return s_wifi_enabled;
}
