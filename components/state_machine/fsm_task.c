/* ============================================================
   MÓDULO     : fsm_task
   FICHEIRO   : fsm_task.c — Task principal da FSM
   VERSÃO     : 1.0  |  2026-04-26
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Loop principal da FSM a 100ms (Core 1, Prio 6).
   - Lê estado atómico do radar (escrito pela radar_task)
   - Chama state_machine_update()
   - Lê e processa eventos do tracking_manager
   - Sincroniza o display com o estado actual
   - Envia heartbeat ao system_monitor
   - Suporta injecção de carro de teste via debugger

   NÃO lê radar directamente — responsabilidade da radar_task.
   NÃO chama tracking_manager_update() — responsabilidade da radar_task.

   SINCRONIZAÇÃO:
   ──────────────
   radar_task (Core 0, 10ms) → atomic_bool s_radar_teve_frame
   fsm_task   (Core 1, 100ms) → lê s_radar_teve_frame

   DEPENDÊNCIAS:
   ─────────────
   fsm_core.h, fsm_events.h, fsm_sim.h
   tracking_manager.h, display_manager.h
   comm_manager.h, radar_manager.h, dali_manager.h
   system_monitor.h, system_config.h
============================================================ */

#include "fsm_core.h"
#include "fsm_events.h"
#include "fsm_sim.h"
#include "state_machine.h"
#include "tracking_manager.h"
#include "system_monitor.h"
#include "comm_manager.h"
#include "radar_manager.h"
#include "dali_manager.h"
#include "display_manager.h"
#include "system_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdatomic.h>
#include <string.h>

static const char *TAG = "FSM_TASK";

/* ============================================================
   VARIÁVEL ATÓMICA — saúde do radar
   Escrita pela radar_task a cada frame válido.
   Lida pela fsm_task a cada ciclo de 100ms.
============================================================ */
static _Atomic bool s_radar_teve_frame = false;

/* ============================================================
   VARIÁVEIS DE TESTE VIA DEBUGGER (JTAG/GDB)
   Alterar em tempo real:
     (gdb) set g_test_car = true
     (gdb) set g_test_car_speed = 80.0
============================================================ */
volatile bool  g_test_car       = false;
volatile float g_test_car_speed = 50.0f;


/* ============================================================
   tracking_manager_task_notify_frame
   Chamada pela radar_task após cada frame válido do radar.
   Regista que o radar está activo para a fsm_task.
   Deve ser chamada EM TODOS OS FRAMES — mesmo sem alvos —
   para distinguir "radar sem alvos" de "radar morto".
============================================================ */
void tracking_manager_task_notify_frame(bool teve_dados)
{
    atomic_store(&s_radar_teve_frame, teve_dados);
}


/* ============================================================
   _atualiza_radar_display
   Lê dados de posição do tracking_manager e envia ao display.
============================================================ */
static void _atualiza_radar_display(void)
{
#if USE_RADAR == 0
    float x = 0.0f, y = 0.0f;
    if (fsm_sim_get_objeto(&x, &y)) {
        radar_obj_t obj = {0};
        obj.x_mm      = (int)x;
        obj.y_mm      = (int)y;
        obj.speed_kmh = state_machine_get_last_speed();
        display_manager_set_radar(&obj, 1);
    } else {
        display_manager_set_radar(NULL, 0);
    }

#else
    tracked_vehicle_t veiculos[TRK_MAX_VEHICLES];
    uint8_t count = 0;

    if (tracking_manager_get_vehicles(veiculos, &count) && count > 0) {
        radar_obj_t objs[TRK_MAX_VEHICLES];
        uint8_t n_objs = 0;
        bool em_obstaculo = sm_is_obstaculo();

        for (uint8_t i = 0; i < count && i < TRK_MAX_VEHICLES; i++) {
            tracked_vehicle_t *v = &veiculos[i];
            if (v->state == TRK_STATE_TENTATIVE) continue;

            if (em_obstaculo) {
                /* Obstáculo — posição fixa, speed=0 impede interpolação */
                if (v->state == TRK_STATE_CONFIRMED ||
                    v->state == TRK_STATE_APPROACHING) {
                    objs[n_objs].x_mm      = (int)v->x_mm;
                    objs[n_objs].y_mm      = (int)v->y_mm;
                    objs[n_objs].speed_kmh = 0.0f;
                    n_objs++;
                }
            } else {
                /* Modo normal — veículos em movimento */
                if (v->state == TRK_STATE_CONFIRMED ||
                    v->state == TRK_STATE_APPROACHING) {
                    objs[n_objs].x_mm      = (int)v->x_mm;
                    objs[n_objs].y_mm      = (int)v->y_mm;
                    objs[n_objs].speed_kmh = v->speed_kmh;
                    n_objs++;
                }
            }
        }
        display_manager_set_radar(n_objs > 0 ? objs : NULL, n_objs);
    } else {
        display_manager_set_radar(NULL, 0);
    }
#endif
}


/* ============================================================
   fsm_task — Core 1, Prioridade 6, Stack 6144B
============================================================ */
static void fsm_task(void *arg)
{
    ESP_LOGI(TAG, "fsm_task | Core %d | Prio 6 | aguarda 6s (radar)",
             xPortGetCoreID());

    /* Aguarda estabilização do HLK-LD2450 (6s em blocos de 200ms)
       com heartbeat para evitar falsos warnings no system_monitor. */
    for (int i = 0; i < 30; i++) {
        system_monitor_heartbeat(MOD_FSM);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    radar_flush_rx();
    ESP_LOGI(TAG, "fsm_task v1.0 activa — pipeline event-driven");

    while (1) {

        /* ── 1. Injecção de carro de teste via debugger ──────── */
        if (g_test_car) {
            g_test_car = false;
            sm_inject_test_car(g_test_car_speed);
        }

        /* ── 2. Lê estado atómico do radar ───────────────────── */
        bool radar_frame = atomic_exchange(&s_radar_teve_frame, false);

        /* ── 3. Ciclo da FSM ──────────────────────────────────── */
        state_machine_update(
            comm_status_ok(),
            comm_is_master(),
            radar_frame
        );

        /* ── 3.1 Lê e processa eventos do tracking_manager ───── */
        tracked_vehicle_t veiculos[TRK_MAX_VEHICLES];
        uint8_t count = 0;
        if (tracking_manager_get_vehicles(veiculos, &count)) {
            for (uint8_t i = 0; i < count; i++) {
                tracked_vehicle_t *v = &veiculos[i];

                if (v->event_detected_pending)
                    sm_process_event(SM_EVT_VEHICLE_DETECTED,
                        v->id, v->speed_kmh, v->eta_ms, (int16_t)v->x_mm);

                if (v->event_approach_pending)
                    sm_process_event(SM_EVT_VEHICLE_APPROACHING,
                        v->id, v->speed_kmh, v->eta_ms, (int16_t)v->x_mm);

                if (v->event_passed_pending)
                    sm_process_event(SM_EVT_VEHICLE_PASSED,
                        v->id, v->speed_kmh, 0, (int16_t)v->x_mm);

                if (v->event_obstaculo_pending)
                    sm_process_event(SM_EVT_VEHICLE_OBSTACULO,
                        v->id, v->speed_kmh, 0, (int16_t)v->x_mm);

                /* Limpa todas as flags só depois de processar todos */
                tracking_manager_clear_events(v->id);
            }
        }

        /* ── 4. Sincroniza display ────────────────────────────── */
        display_manager_set_status(state_machine_get_state_name());
        display_manager_set_traffic(state_machine_get_T(),
                                    state_machine_get_Tc());
        display_manager_set_speed((int)state_machine_get_last_speed());
        display_manager_set_hardware(radar_get_status_str(),
                                     radar_is_connected(),
                                     (uint8_t)dali_get_brightness());
        _atualiza_radar_display();

        /* ── 5. Heartbeat ao monitor ──────────────────────────── */
        system_monitor_heartbeat(MOD_FSM);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/* ============================================================
   state_machine_task_start — Core 1, Prio 6
============================================================ */
void state_machine_task_start(void)
{
    tracking_manager_init();

    xTaskCreatePinnedToCore(fsm_task, "fsm_task",
                            6144, NULL, 6, NULL, 1);

    ESP_LOGI(TAG, "fsm_task v1.0 criada | Core 1 | Prio 6 | Stack 6144");
}
