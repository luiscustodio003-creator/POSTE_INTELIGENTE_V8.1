/* ============================================================
   MÓDULO     : state_machine_task
   FICHEIRO   : state_machine_task.c
   VERSÃO     : 4.2  |  2026-04-23
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÕES v4.1 → v4.2:
   ─────────────────────────
   - Corrigido: bloco de debug e segundo vTaskDelay estavam
     fora do while(1) — nunca eram executados.
   - Adicionado: consumo de event_recuou_pending.
   - Log de debug periódico (1s) dentro do loop correcto.
============================================================ */

#include "state_machine.h"
#include "udp_manager.h"
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

/* Variável atómica partilhada entre radar_task (escrita) e fsm_task (leitura) */
static _Atomic bool s_radar_teve_frame = false;

/* Injecção de carro de teste via debugger JTAG/GDB */
volatile bool  g_test_car       = false;
volatile float g_test_car_speed = 50.0f;


/* ============================================================
   tracking_manager_task_notify_frame
   Chamada pela radar_task após cada frame válido do radar.
   Deve ser chamada em TODOS os frames — mesmo sem alvos.
============================================================ */
void tracking_manager_task_notify_frame(bool teve_dados)
{
    atomic_store(&s_radar_teve_frame, teve_dados);
}


/* ============================================================
   _atualiza_radar_display
   Envia posições dos veículos ao display_manager.
   Cada poste só mostra objectos do SEU radar local.
   TENTATIVE e EXITED são filtrados — não aparecem no canvas.
============================================================ */
static void _atualiza_radar_display(void)
{
#if USE_RADAR == 0
    float x = 0.0f, y = 0.0f;
    if (sim_get_objeto(&x, &y)) {
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

        for (uint8_t i = 0; i < count && i < TRK_MAX_VEHICLES; i++) {
            /* Ignora TENTATIVE — pode ser ruído */
            if (veiculos[i].state == TRK_STATE_TENTATIVE) continue;
            /* Ignora EXITED — já saiu do radar */
            if (veiculos[i].state == TRK_STATE_EXITED)    continue;

            objs[n_objs].x_mm      = (int)veiculos[i].x_mm;
            objs[n_objs].y_mm      = (int)veiculos[i].y_mm;
            objs[n_objs].speed_kmh = veiculos[i].speed_kmh;
            n_objs++;
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
    ESP_LOGI(TAG, "fsm_task | Core %d | Prio 6 | aguarda 6s",
             xPortGetCoreID());

    /* Aguarda estabilização do radar HLK-LD2450 (6s em blocos de 200ms) */
    for (int i = 0; i < 30; i++) {
        system_monitor_heartbeat(MOD_FSM);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    radar_flush_rx();
    ESP_LOGI(TAG, "fsm_task v4.2 activa | POST_POSITION=%d", POST_POSITION);

    /* Contador para log de debug periódico */
    static uint32_t s_debug_cnt = 0;

    while (1) {

        /* ── 1. Injecção de carro de teste via debugger ──────── */
        if (g_test_car) {
            g_test_car = false;
            sm_inject_test_car(g_test_car_speed);
        }

        /* ── 2. Lê saúde do radar (atómico) ─────────────────── */
        bool radar_frame = atomic_exchange(&s_radar_teve_frame, false);

        /* ── 3. Ciclo de manutenção da FSM ───────────────────── */
        state_machine_update(
            comm_status_ok(),
            comm_is_master(),
            radar_frame
        );

        /* ── 4. Lê eventos do tracking_manager → FSM ─────────── */
        tracked_vehicle_t veiculos[TRK_MAX_VEHICLES];
        uint8_t count = 0;
        if (tracking_manager_get_vehicles(veiculos, &count)) {
            for (uint8_t i = 0; i < count; i++) {
                tracked_vehicle_t *v = &veiculos[i];

                if (v->event_detected_pending) {
                    sm_process_event(SM_EVT_VEHICLE_DETECTED,
                        v->id, v->speed_kmh, v->eta_ms, (int16_t)v->x_mm);
                    tracking_manager_clear_events(v->id);
                }
                if (v->event_approach_pending) {
                    sm_process_event(SM_EVT_VEHICLE_APPROACHING,
                        v->id, v->speed_kmh, v->eta_ms, (int16_t)v->x_mm);
                    tracking_manager_clear_events(v->id);
                }
                if (v->event_passed_pending) {
                    sm_process_event(SM_EVT_VEHICLE_PASSED,
                        v->id, v->speed_kmh, 0, (int16_t)v->x_mm);
                    tracking_manager_clear_events(v->id);
                }
                if (v->event_obstaculo_pending) {
                    sm_process_event(SM_EVT_VEHICLE_OBSTACULO,
                        v->id, v->speed_kmh, 0, (int16_t)v->x_mm);
                    tracking_manager_clear_events(v->id);
                }
                /* Objecto recuou — T=0 imediato sem aguardar poste B */
                if (v->event_recuou_pending) {
                    sm_process_event(SM_EVT_VEHICLE_RECUOU,
                        v->id, v->speed_kmh, 0, (int16_t)v->x_mm);
                    tracking_manager_clear_events(v->id);
                }
            }
        }

        /* ── 5. Sincroniza display ────────────────────────────── */
        display_manager_set_status(state_machine_get_state_name());
        display_manager_set_traffic(state_machine_get_T(),
                                    state_machine_get_Tc());
        display_manager_set_speed((int)state_machine_get_last_speed());
        display_manager_set_hardware(radar_get_status_str(),
                                     radar_is_connected(),
                                     (uint8_t)dali_get_brightness());
        _atualiza_radar_display();

        /* ── 6. Heartbeat ao monitor ──────────────────────────── */
        system_monitor_heartbeat(MOD_FSM);

        /* ── 7. Log de debug periódico (1 linha a cada 1s) ──── */
        if (++s_debug_cnt >= 10) {
            s_debug_cnt = 0;
            char nebL[16] = "---", nebR[16] = "---";
            udp_manager_get_neighbors(nebL, nebR);
            ESP_LOGI("DEBUG",
                     "| POS=%d | %s | T=%d Tc=%d | vel=%.0f km/h | "
                     "radar=%s | esq=%s | dir=%s |",
                     POST_POSITION,
                     state_machine_get_state_name(),
                     state_machine_get_T(),
                     state_machine_get_Tc(),
                     state_machine_get_last_speed(),
                     state_machine_radar_ok() ? "OK" : "KO",
                     nebL, nebR);
        }

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
    ESP_LOGI(TAG, "fsm_task v4.2 criada | Core 1 | Prio 6 | Stack 6144");
}
