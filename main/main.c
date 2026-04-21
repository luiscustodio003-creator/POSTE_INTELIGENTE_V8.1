/* ============================================================
   MAIN — PONTO DE ENTRADA
   @file      main.c
   @version   4.0  |  2026-04-09
   Projecto  : Poste Inteligente v8
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

   Responsabilidades (mínimas por design v8):
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
   Inicializa NVS. Se corrompida (após flash de firmware novo),
   apaga e reinicia — garante arranque limpo sempre.
============================================================ */
static void _init_nvs(void)
{
    //nvs_flash_erase();   /* TEMPORÁRIO — remover após primeiro flash */

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
   Stack de 4096: suficiente para as 3 chamadas de init.
   Após system_monitor_start() esta stack é libertada —
   o monitor corre na sua própria task.
============================================================ */
void app_main(void)
{
    /* Base obrigatória antes de qualquer driver de rede */
    _init_nvs();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Delega tudo ao supervisor — não retorna */
    system_monitor_start();
}