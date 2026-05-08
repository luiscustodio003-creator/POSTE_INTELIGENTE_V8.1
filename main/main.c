/* ============================================================
   MAIN — PONTO DE ENTRADA
   @file      main.c
   @version   4.1  |  2026-05-07
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÕES v4.0 → v4.1:
   ─────────────────────────
   REMOVIDAS referências a wifi_mesh (sistema antigo).
   Sistema agora usa wifi_manager v2.0 com IP fixo.
   
   Responsabilidades:
   1. Inicializar NVS flash
   2. Inicializar infraestrutura de eventos de rede
   3. Delegar completamente ao system_monitor

   O main.c não cria tasks, não tem loops, não gere hardware.
   Tudo é orquestrado pelo system_monitor_start().
============================================================ */
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "system_monitor.h"

static const char *TAG = "MAIN";


/* ============================================================
   _init_nvs
   ──────────────────────────────────────────────────────────
   Inicializa NVS. Se corrompida (após flash de firmware novo),
   apaga e reinicia — garante arranque limpo sempre.
============================================================ */
static void _init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        
        ESP_LOGW(TAG, "NVS corrompida — a apagar e reinicializar");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS inicializada");
}


/* ============================================================
   app_main
   ──────────────────────────────────────────────────────────
   Stack de 4096: suficiente para as 3 chamadas de init.
   Após system_monitor_start() esta stack é libertada —
   o monitor corre na sua própria task.
============================================================ */
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  POSTE INTELIGENTE v8");
    ESP_LOGI(TAG, "  WiFi Manager v2.0 (IP fixo)");
    ESP_LOGI(TAG, "  FSM Network v3.1 (failover optimizado)");
    ESP_LOGI(TAG, "========================================");
    
    /* Base obrigatória antes de qualquer driver de rede */
    _init_nvs();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    ESP_LOGI(TAG, "Infraestrutura de rede inicializada");
    
    /* Delega tudo ao supervisor — não retorna */
    ESP_LOGI(TAG, "A iniciar system_monitor...");
    system_monitor_start();
    
    /* Nunca chega aqui */
}
