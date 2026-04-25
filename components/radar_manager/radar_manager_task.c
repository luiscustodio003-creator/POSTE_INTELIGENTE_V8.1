/* ============================================================
   RADAR MANAGER — TASK INTERNA
   @file      radar_manager_task.c
   @version   4.0  |  2026-04-17
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Lê o HLK-LD2450 a 100ms, alimenta o tracking_manager com
   cada frame, e notifica a fsm_task sobre a saúde do radar
   via variável atómica (sem mutex).

   MUDANÇAS v1.0 → v4.0:
   ─────────────────────────
   - CORRIGIDO: radar_task estava no Core 1 em conflito com
     fsm_task (também Core 1, Prio 6 > Prio 5).
     AGORA: radar_task → Core 0, Prio 5.
     fsm_task          → Core 1, Prio 6.
     Cada task tem o seu core — sem preempção entre elas.

   - ADICIONADO: tracking_manager_update(&dados) após cada frame.
     O tracking_manager é o único consumidor do frame bruto.
     A fsm_task deixou de chamar radar_read_data_cached().

   - ADICIONADO: tracking_manager_task_notify_frame(bool) para
     comunicar saúde do radar à fsm_task via atomic_bool.

   - MANTIDO: system_monitor_heartbeat(MOD_RADAR) a cada ciclo.
   - MANTIDO: período de 100ms (alinhado com o HLK-LD2450 a 10Hz).
   - REMOVIDO: #include <inttypes.h> desnecessário.

   SINCRONIZAÇÃO:
   ──────────────
   radar_task (Core 0, 100ms) → tracking_manager_update()
                               → tracking_manager_task_notify_frame()
                                        ↓ atomic_bool
   fsm_task   (Core 1, 100ms) → state_machine_update(..., radar_frame)

   NOTA SOBRE PERÍODO:
   ────────────────────
   O HLK-LD2450 envia frames a ~10Hz (100ms).
   A radar_task corre a 100ms — sincronizado com o sensor.
   O tracking_manager processa cada frame individualmente,
   pelo que o período de 100ms é adequado para o nearest-neighbour
   e para os timers TRK_CONFIRM_FRAMES e TRK_LOST_FRAMES.
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
   Declaração da função de notificação atómica.
   Definida em state_machine_task.c — exportada sem header
   dedicado (será centralizada em pipeline_ipc.h na fase 3).
============================================================ */
extern void tracking_manager_task_notify_frame(bool teve_dados);


/* ============================================================
   radar_task — Core 0, Prioridade 5, Stack 4096B
   ──────────────────────────────────────────────
   Período: 100ms (alinhado com HLK-LD2450 a 10Hz).

   CICLO:
     1. Lê frame UART → atualiza cache interna (radar_manager)
     2. Alimenta tracking_manager com frame bruto
     3. Notifica fsm_task sobre saúde do radar (atomic_bool)
     4. Heartbeat ao system_monitor
     5. Aguarda 100ms

   THREADING:
     - radar_read_data() actualiza s_last_data com spinlock interno.
     - tracking_manager_update() acesso exclusivo desta task.
     - tracking_manager_task_notify_frame() escreve atomic_bool.
     - Sem mutex necessário entre estes três passos.
============================================================ */
static void radar_task(void *arg)
{
    ESP_LOGI(TAG, "radar_task | Core %d | Prio 5 | 100ms",
             xPortGetCoreID());

    while (1) {

#if USE_RADAR
        /* ── 1. Lê frame UART e actualiza cache interna ──────── */
        /* radar_read_data() faz parse do frame HLK-LD2450,
           aplica _hlk_decode_signed(), calcula distância real,
           detecta obstáculos estáticos e guarda em s_last_data
           protegida por spinlock (taskENTER/EXIT_CRITICAL).     */
        radar_data_t dados = {0};
        bool ok = radar_read_data(&dados, NULL);

        /* ── 2. Alimenta tracking_manager (v4.0) ─────────────── */
        /* Chamar SEMPRE — mesmo com ok=false ou count=0.
           O tracking avança lost_frames dos slots em COASTING,
           garantindo que veículos que saíram são declarados EXITED
           mesmo quando o radar não retorna frame válido.          */
        tracking_manager_update(&dados);

        /* ── 3. Notifica fsm_task sobre saúde do radar ───────── */
        /* true  = frame válido COM alvos → radar activo e com dados
           false = frame inválido OU sem alvos → radar pode estar
                   degradado (a FSM distingue com RADAR_FAIL_COUNT) */
        tracking_manager_task_notify_frame(ok && dados.count > 0);

#else
        /* Modo USE_RADAR=0: simulador corre na fsm_task (v4.0).
           Notifica sempre como "ok" para a FSM não entrar em SAFE_MODE. */
        tracking_manager_task_notify_frame(true);
#endif

        /* ── 4. Heartbeat ao monitor ──────────────────────────── */
        system_monitor_heartbeat(MOD_RADAR);

        /* ── 5. Aguarda próximo período ───────────────────────── */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/* ============================================================
   radar_manager_task_start
   ─────────────────────────
   CORRIGIDO v4.0: Core 0 (era Core 1 — conflito com fsm_task).

   Ordem de arranque recomendada em app_main():
     1. radar_manager_task_start()   ← Core 0
     2. state_machine_task_start()   ← Core 1
        (state_machine_task_start chama tracking_manager_init()
         antes de criar a fsm_task — garante inicialização antes
         do primeiro frame da radar_task chegar ao tracking)

   NOTA: tracking_manager_init() é chamado em
   state_machine_task_start() (state_machine_task.c v4.0).
   NÃO chamar aqui para evitar dupla inicialização.
============================================================ */
void radar_manager_task_start(void)
{
    xTaskCreatePinnedToCore(radar_task, "radar_task",
                            4096, NULL, 5, NULL, 0); /* Core 0 ← CORRIGIDO */

    ESP_LOGI(TAG, "radar_task v4.0 | Core 0 | Prio 5 | Stack 4096");
}
