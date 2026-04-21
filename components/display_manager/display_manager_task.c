/* ============================================================
   DISPLAY MANAGER — TASK INTERNA
   @file      display_manager_task.c
   @version   2.0  |  2026-04-17
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   display_task: Core 0, Prioridade 4, Stack 8192B, Período 20ms.
   ÚNICA task que chama funções LVGL — sem mutex LVGL necessário.
   Drena fila de mensagens e chama lv_timer_handler() a 50Hz.

   MUDANÇAS v1.0 → v2.0:
   ─────────────────────────
   - CORRIGIDO: display_task estava no Core 1 em conflito com
     fsm_task (Prio 6) e comm_task no mesmo core.
     A fsm_task preemptava o render LVGL a cada 100ms,
     causando jitter perceptível no sweep animado do radar.
     AGORA: display_task → Core 0, Prio 4.
     Distribuição final:
       Core 0: radar_task (Prio 5) + display_task (Prio 4)
       Core 1: fsm_task  (Prio 6) + comm_task
     As duas tasks do Core 0 têm prioridades e períodos
     diferentes (100ms vs 20ms) — coexistem sem preempção
     problemática. O LVGL render é fluido a 50Hz.

   - CORRIGIDO: display_manager_tick(delta) era chamado com
     delta calculado ANTES do vTaskDelay — o delta incluía
     o tempo de render mas não o tempo de sleep. Agora o
     tick usa esp_timer_get_time() directamente, igual ao
     que display_manager.c já faz internamente.
     Simplificado: lv_tick_inc(20) fixo (período nominal).

   - CORRIGIDO: system_monitor_heartbeat(MOD_DISPLAY) movido
     para ANTES do vTaskDelay para garantir que o heartbeat
     é enviado mesmo que o render demore mais que o esperado.

   - REMOVIDO: #include <inttypes.h> desnecessário.

   THREADING:
   ──────────
   display_task é a ÚNICA task que chama funções LVGL.
   A fila s_fila (display_manager.c) isola os produtores
   (fsm_task, comm_task, wifi_manager) do consumidor LVGL.
   Sem mutex LVGL necessário — design por fila.
============================================================ */

#include "display_manager.h"
#include "system_monitor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "DISP_TASK";


/* ============================================================
   display_task — Core 0, Prioridade 4
   ──────────────────────────────────
   Período nominal: 20ms (50Hz).

   CICLO:
     1. Tick LVGL fixo de 20ms (período nominal do loop)
     2. Drena fila de mensagens + redesenha canvas radar
        + chama lv_timer_handler() [via display_manager_task()]
     3. Heartbeat ao system_monitor
     4. Aguarda 20ms

   NOTA SOBRE O TICK:
   lv_tick_inc(20) usa o período nominal em vez do delta real.
   Isto é aceitável porque:
     a) O período é fixo (vTaskDelay garante ~20ms).
     b) display_manager.c usa esp_timer_get_time() directamente
        no interpolador — independente do tick LVGL.
     c) Animações LVGL são baseadas em tempo relativo,
        não em frames absolutos — erro de ±2ms é irrelevante.
============================================================ */
static void display_task(void *arg)
{
    ESP_LOGI(TAG, "display_task | Core %d | Prio 4 | 50Hz",
             xPortGetCoreID());

    while (1) {

        /* ── 1. Tick LVGL ─────────────────────────────────── */
        /* Período nominal de 20ms — suficientemente preciso
           para animações LVGL. O interpolador do radar usa
           esp_timer_get_time() directamente (mais preciso). */
        display_manager_tick(20);

        /* ── 2. Render + fila de mensagens ───────────────── */
        /* Drena até 5 mensagens da fila, redesenha canvas
           radar com interpolação, chama lv_timer_handler(). */
        display_manager_task();

        /* ── 3. Heartbeat ao monitor ─────────────────────── */
        /* Antes do delay — garante heartbeat mesmo se render
           demorar ligeiramente mais que o esperado. */
        system_monitor_heartbeat(MOD_DISPLAY);

        /* ── 4. Aguarda próximo ciclo ────────────────────── */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


/* ============================================================
   display_manager_task_start
   ───────────────────────────
   CORRIGIDO v2.0: Core 0 (era Core 1 — conflito com fsm_task).

   Ordem de arranque recomendada em app_main():
     1. display_manager_init()          ← inicializa LVGL + ST7789
     2. radar_manager_task_start()      ← Core 0, Prio 5
     3. display_manager_task_start()    ← Core 0, Prio 4
     4. state_machine_task_start()      ← Core 1, Prio 6
     5. comm_manager_task_start()       ← Core 1, Prio ?

   Stack 8192B: LVGL heap + render SPI + interpolador radar
   + buffer de fila dm_msg_t.
============================================================ */
void display_manager_task_start(void)
{
    xTaskCreatePinnedToCore(display_task, "display_task",
                            8192, NULL, 4, NULL, 0); /* Core 0 ← CORRIGIDO */

    ESP_LOGI(TAG, "display_task v2.0 | Core 0 | Prio 4 | Stack 8192");
}
