/* ============================================================
   RADAR MANAGER — TASK DE LEITURA
   @file      radar_manager_task.c
   @version   5.0  |  2026-04-29
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Lê o sensor HLK-LD2450 via UART a cada 100ms,
   alimenta o tracking_manager com o frame bruto,
   e notifica a fsm_task sobre a saúde do radar via atomic_bool.

   CICLO DE EXECUÇÃO:
   ───────────────────
     1. Lê frame UART → radar_read_data() (parse HLK-LD2450)
     2. Alimenta tracking_manager_update() com o frame
     3. Notifica fsm_task: tracking_manager_task_notify_frame(ok)
     4. Heartbeat ao system_monitor
     5. Aguarda 100ms

   SINCRONIZAÇÃO ENTRE TASKS:
   ────────────────────────────
     radar_task (Core 0, Prio 5):
       radar_read_data()                   → actualiza cache interna (spinlock)
       tracking_manager_update()           → exclusivo desta task
       tracking_manager_task_notify_frame() → escreve atomic_bool

     fsm_task (Core 1, Prio 6):
       atomic_exchange(&s_radar_teve_frame) → lê e repõe a false

   NOTA — tracking_manager_update() com count=0:
   ───────────────────────────────────────────────
     Chamar SEMPRE, mesmo quando ok=false ou count=0.
     O tracking avança os lost_frames dos slots em COASTING,
     garantindo que veículos que saíram são declarados EXITED
     mesmo quando o sensor não retorna frame válido nesse ciclo.

   NOTA — extern removido:
   ────────────────────────
     tracking_manager_task_notify_frame() era declarado com extern
     directamente no .c (acoplamento silencioso). Em v5.0 é incluído
     via tracking_manager.h — compilador valida a assinatura.

   MUDANÇAS v4.0 → v5.0:
   ──────────────────────
     - REMOVIDO: bloco #if USE_RADAR / #else (modo simulado)
     - REMOVIDO: extern void tracking_manager_task_notify_frame()
     - ADICIONADO: #include "tracking_manager.h" (declaração correcta)
     - SIMPLIFICADO: radar_task() sem ramificação condicional
     - MANTIDO: heartbeat, período 100ms, Core 0 Prio 5
============================================================ */

#include "radar_manager.h"
#include "tracking_manager.h"
#include "system_monitor.h"
#include "system_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "RADAR_TASK";


/* ============================================================
   radar_task — Core 0, Prioridade 5, Stack 4096B
   ──────────────────────────────────────────────
   Período: 100ms — alinhado com o HLK-LD2450 a ~10Hz.
============================================================ */
static void radar_task(void *arg)
{
    ESP_LOGI(TAG, "radar_task | Core %d | Prio 6 | 100ms | radar real",
             xPortGetCoreID());

    while (1) {

        /* ── 1. Lê frame UART e actualiza cache interna ──────── */
        /* radar_read_data() faz parse do frame HLK-LD2450:
             - Descodifica X, Y, velocidade com _hlk_decode_signed()
             - Formato: bit 15 = sinal, bits 0-14 = magnitude
             - X positivo = direita do sensor, negativo = esquerda
             - Y sempre positivo (distância frontal)
             - Velocidade: bit15=1 → a aproximar-se (negativo)
           Guarda em s_last_data protegida por spinlock interno. */
        radar_data_t dados = {0};
        bool ok = radar_read_data(&dados, NULL);

        /* ── 2. Alimenta o tracking_manager ──────────────────── */
        /* Chamado SEMPRE — mesmo com ok=false ou count=0.
           O tracking precisa de avançar os contadores de lost_frames
           mesmo em ciclos sem detecções. */
        tracking_manager_update(&dados);

        /* ── 3. Notifica a fsm_task sobre saúde do radar ─────── */
        /* true  = frame UART válido (com ou sem alvos)
           false = frame inválido ou timeout UART
           A FSM acumula falhas em RADAR_FAIL_COUNT para → SAFE_MODE */
        tracking_manager_task_notify_frame(ok);

        /* ── 4. Heartbeat ao system_monitor ───────────────────── */
        system_monitor_heartbeat(MOD_RADAR);

        /* ── 5. Aguarda próximo ciclo ──────────────────────────── */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/* ============================================================
   radar_manager_task_start
   ─────────────────────────
   @brief Cria a radar_task no Core 0, Prio 6.

   Prioridade 6 (igual à fsm_task) garante que a radar_task
   não é preemptada pela stack WiFi durante retries de ligação.
   Com Prio 5, o driver WiFi (Prio 23) causava starvation de
   até 1s, acumulando falhas no g_fsm_radar_fail_cnt e
   provocando entradas falsas em SAFE_MODE.

   Deve ser chamado APÓS state_machine_task_start() para garantir
   que o tracking_manager está inicializado antes do primeiro frame.

   Pinagem no Core 0 evita contenção com a fsm_task (Core 1).
============================================================ */
void radar_manager_task_start(void)
{
    xTaskCreatePinnedToCore(
        radar_task,
        "radar_task",
        4096,
        NULL,
        6,              /* Prio 6 — protege contra starvation do WiFi */
        NULL,
        0               /* Core 0 — PRO_CPU */
    );

    ESP_LOGI(TAG, "radar_task v5.1 | Core 0 | Prio 6 | Stack 4096B");
}
