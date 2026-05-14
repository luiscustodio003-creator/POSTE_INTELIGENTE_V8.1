/* ============================================================
   WIFI MANAGER — IMPLEMENTAÇÃO
   @file      wifi_manager.c
   @version   2.0  |  2026-05-07
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÕES v1.5 → v2.0:
   ─────────────────────────
   PROBLEMA CORRIGIDO: IP dinâmico + mudança STA↔AP causava split-brain.
   
   SOLUÇÃO:
   • IP FIXO estático baseado em POST_POSITION (nunca DHCP)
   • Modo WiFi FIXO no arranque (NUNCA muda em runtime)
   • pos=0 → AP permanente (192.168.4.1)
   • pos>0 → STA permanente (192.168.4.X onde X=pos+1)
   • wifi_manager_assume_ap() REMOVIDA (só loga warning)
   
   VANTAGENS:
   ✅ IP nunca muda (mesmo que papel master↔slave mude)
   ✅ Sem perda de conexão em failover
   ✅ Sem conflito de IP entre postes
   ✅ Transições suaves master→slave→master

   DEPENDÊNCIAS:
   ─────────────
   system_config.h (POST_POSITION, WIFI_SSID, etc.)
   display_manager.h (notificação de estado WiFi)
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

/* ── Estado interno ───────────────────────────────────────── */
static bool s_conectado   = false;
static char s_ip[16]      = "---";
static int  s_retries     = 0;
static bool s_modo_ap     = false;
static bool s_wifi_enabled = true;

/* Spinlock para proteger s_ip (acedido de event handler e monitor) */
static portMUX_TYPE s_ip_mux = portMUX_INITIALIZER_UNLOCKED;

/* Timer de reconexão após falha de retries */
static esp_timer_handle_t s_reconect_timer = NULL;


/* ============================================================
   _reconect_cb — Callback do timer de reconexão
============================================================ */
static void _reconect_cb(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Timer de reconexão disparado — a tentar ligar");
    s_retries = 0;
    esp_wifi_connect();
}


/* ============================================================
   wifi_event_handler — Handler de eventos Wi-Fi e IP
============================================================ */
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
            ESP_LOGI(TAG, "Ligado ao AP — IP já está fixo (192.168.4.%d)",
                     POST_POSITION + 1);
            /* IP fixo configurado no init, não há evento GOT_IP */
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
                ESP_LOGI(TAG, "Desligado — retry %d/%d",
                         s_retries, WIFI_RETRY_ATTEMPTS);
                esp_wifi_connect();
            } else {
                ESP_LOGW(TAG, "Retries esgotados — pausa %dms",
                         WIFI_RECONNECT_MS);
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
    /* Nota: Não há IP_EVENT_STA_GOT_IP porque usamos IP estático! */
}


/* ============================================================
   wifi_manager_init — Inicializa em modo STA com IP fixo
   ──────────────────────────────────────────────────────────
   Chamado por postes com POST_POSITION > 0.
   Configura IP estático: 192.168.4.(POST_POSITION + 1)
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
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();

    /* ── IP ESTÁTICO (não DHCP!) ──────────────────────────
       Calcula IP baseado em POST_POSITION:
       pos=1 → 192.168.4.2
       pos=2 → 192.168.4.3
       pos=3 → 192.168.4.4
       ...
       
       CRÍTICO: IP nunca muda, mesmo que papel master/slave mude!
    ──────────────────────────────────────────────────────── */
    esp_netif_dhcpc_stop(netif);

    esp_netif_ip_info_t ip_info = {0};
    IP4_ADDR(&ip_info.ip,      192, 168, 4, POST_POSITION + 1);
    IP4_ADDR(&ip_info.gw,      192, 168, 4, 1);  /* Gateway = AP do master */
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &ip_info));

    ESP_LOGI(TAG, "IP fixo configurado: 192.168.4.%d (baseado em pos=%d)",
             POST_POSITION + 1, POST_POSITION);

    /* Inicializa stack WiFi */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Regista handlers de eventos */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));

    /* Configura credenciais do AP master */
    wifi_config_t wc = {0};
    strncpy((char *)wc.sta.ssid,     WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, WIFI_PASS,  sizeof(wc.sta.password) - 1);

    /* Inicia em modo STA */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_modo_ap = false;
    ESP_LOGI(TAG, "Wi-Fi STA iniciado | SSID: %s | IP: 192.168.4.%d",
             WIFI_SSID, POST_POSITION + 1);
}


/* ============================================================
   wifi_manager_init_ap — Cria AP (só pos=0)
   ──────────────────────────────────────────────────────────
   Chamado por poste com POST_POSITION == 0.
   Cria rede WiFi para outros postes se ligarem.
   IP fixo do AP: 192.168.4.1 (automático ESP-IDF)
============================================================ */
void wifi_manager_init_ap(void)
{
    /* Cria interface AP */
    esp_netif_create_default_wifi_ap();

    /* Inicializa stack WiFi */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Regista handlers de eventos */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));

    /* Configura AP */
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

    /* IP fixo do AP — ESP-IDF configura automaticamente 192.168.4.1 */
    snprintf(s_ip, sizeof(s_ip), "192.168.4.1");
    s_conectado = true;
    s_modo_ap   = true;

    display_manager_set_wifi(true, s_ip);
    ESP_LOGI(TAG, "[AP] Rede criada: %s | IP: 192.168.4.1", WIFI_SSID);
}


/* ============================================================
   wifi_manager_init_auto — Escolhe AP ou STA por posição
   ──────────────────────────────────────────────────────────
   Chamado no arranque do sistema (main.c).
   
   POST_POSITION == 0 → Cria AP (master físico)
   POST_POSITION > 0  → Liga-se ao AP (STA com IP fixo)
============================================================ */
void wifi_manager_init_auto(void)
{
    if (POST_POSITION == 0) {
        /* pos=0 → sempre AP, sempre 192.168.4.1 */
        wifi_manager_init_ap();
    } else {
        /* pos>0 → sempre STA, IP=192.168.4.(pos+1) */
        wifi_manager_init();
    }
}


/* ============================================================
   wifi_manager_assume_ap — REMOVIDA!
   ──────────────────────────────────────────────────────────
   Função OBSOLETA mantida por compatibilidade.
   
   PROBLEMA: Mudar STA→AP em runtime causava:
   • Perda de IP (transitava de .X para .1)
   • Perda de conexão durante transição
   • Split-brain quando master original recuperava
   
   SOLUÇÃO v2.0: Modo WiFi NUNCA muda!
   • pos=0 sempre AP
   • pos>0 sempre STA
   • Papel master/slave é LÓGICO (gerido por fsm_network)
   • IP permanece fixo independente do papel
============================================================ */
void wifi_manager_assume_ap(void)
{
    ESP_LOGW(TAG, "wifi_manager_assume_ap() OBSOLETA!");
    ESP_LOGW(TAG, "Modo WiFi NÃO muda em runtime (v2.0).");
    ESP_LOGW(TAG, "Papel master/slave é lógico, não físico (WiFi).");
    ESP_LOGW(TAG, "IP mantém-se: %s", s_ip);
    
    /* NÃO faz nada — modo WiFi é fixo! */
}


/* ============================================================
   API PÚBLICA — Getters
============================================================ */

bool wifi_manager_is_connected(void)
{
    return s_conectado;
}

const char *wifi_manager_get_ip(void)
{
    /* Leitura sem lock — string pequena, worst case = leitura
       parcialmente actualizada, aceitável para display */
    return s_ip;
}

bool wifi_manager_is_ap_mode(void)
{
    return s_modo_ap;
}

void wifi_manager_reset_retry(void)
{
    s_retries = 0;
    esp_wifi_connect();
}

/* ============================================================
   wifi_manager_disable — Desliga WiFi (safe mode)
   ──────────────────────────────────────────────────────────
   Chamado quando radar falha para evitar propagação de TC
   sem detecção de veículos.
============================================================ */
void wifi_manager_disable(void)
{
    if (!s_wifi_enabled) {
        ESP_LOGW(TAG, "WiFi já estava desligado");
        return;
    }
    
    ESP_LOGW(TAG, "🔴 DESLIGANDO WiFi (SAFE MODE - radar offline)");
    
    // Para WiFi
    esp_wifi_stop();
    
    // Actualiza estado
    s_wifi_enabled = false;
    s_conectado = false;
    
    taskENTER_CRITICAL(&s_ip_mux);
    strncpy(s_ip, "OFFLINE", sizeof(s_ip));
    taskEXIT_CRITICAL(&s_ip_mux);
    
    display_manager_set_wifi(false, NULL);
    
    ESP_LOGW(TAG, "WiFi desligado — poste isolado da rede");
}


/* ============================================================
   wifi_manager_enable — Religa WiFi (recuperação)
   ──────────────────────────────────────────────────────────
   Chamado quando radar recupera para restaurar conectividade.
============================================================ */
void wifi_manager_enable(void)
{
    if (s_wifi_enabled) {
        ESP_LOGI(TAG, "WiFi já estava ligado");
        return;
    }
    
    ESP_LOGI(TAG, "🟢 RELIGANDO WiFi (radar recuperado)");
    
    // Reinicia WiFi no modo original
    esp_wifi_start();
    
    // Actualiza estado
    s_wifi_enabled = true;
    s_retries = 0;
    
    // Reconecta se era STA
    if (!s_modo_ap) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "A reconectar ao AP...");
    }
    
    ESP_LOGI(TAG, "WiFi religado — a restaurar conectividade");
}


/* ============================================================
   wifi_manager_is_enabled — Verifica se WiFi está activo
============================================================ */
bool wifi_manager_is_enabled(void)
{
    return s_wifi_enabled;
}