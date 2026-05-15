/* fsm_task.c — v5.1 | 2026-04-30 | Poste Inteligente v8
   Task principal da FSM — Core 1, Prio 6, Stack 6144B, ciclo 100ms. */

#include "fsm_core.h"
#include "fsm_events.h"
#include "state_machine.h"
#include "tracking_manager.h"
#include "display_manager.h"
#include "comm_manager.h"
#include "radar_manager.h"
#include "dali_manager.h"
#include "system_monitor.h"
#include "system_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "FSM_TASK";


/* ── _atualiza_radar_display ──────────────────────────────── */
static void _atualiza_radar_display(void)
{
    tracked_vehicle_t veiculos[TRK_MAX_VEHICLES];
    uint8_t count = 0;

    if (!tracking_manager_get_vehicles(veiculos, &count) || count == 0) {
        display_manager_set_radar(NULL, 0);
        return;
    }

    radar_obj_t alvos[RADAR_MAX_OBJ];
    uint8_t n_alvos      = 0;
    bool    em_obstaculo = sm_is_obstaculo();

    for (uint8_t i = 0; i < count && n_alvos < RADAR_MAX_OBJ; i++) {
        tracked_vehicle_t *v = &veiculos[i];

        if (em_obstaculo && v->obstaculo_frames >= OBSTACULO_MIN_FRAMES) {
            fsm_obstaculo_keepalive();
        }

        if (v->state != TRK_STATE_CONFIRMED &&
            v->state != TRK_STATE_APPROACHING) {
            continue;
        }

        alvos[n_alvos].x_mm      = (int)v->x_mm;
        alvos[n_alvos].y_mm      = (int)v->y_mm;
        /* Velocidade=0 em modo obstáculo: posição fixa no display. */
        alvos[n_alvos].speed_kmh = em_obstaculo ? 0.0f : v->speed_kmh;
        n_alvos++;
    }

    display_manager_set_radar(n_alvos > 0 ? alvos : NULL, n_alvos);
}


/* ── _processa_eventos_tracking ───────────────────────────── */
static void _processa_eventos_tracking(void)
{
    tracked_vehicle_t veiculos[TRK_MAX_VEHICLES];
    uint8_t count = 0;

    if (!tracking_manager_get_vehicles(veiculos, &count)) {
        return;
    }

    for (uint8_t i = 0; i < count; i++) {
        tracked_vehicle_t *v = &veiculos[i];

        if (!v->event_detected_pending   &&
            !v->event_approach_pending   &&
            !v->event_local_pending      &&
            !v->event_passed_pending     &&
            !v->event_obstaculo_pending) {
            continue;
        }

        if (v->event_detected_pending)
            sm_process_event(SM_EVT_VEHICLE_DETECTED,
                             v->id, v->speed_kmh, v->eta_ms, (int16_t)v->x_mm);

        if (v->event_approach_pending)
            sm_process_event(SM_EVT_VEHICLE_APPROACHING,
                             v->id, v->speed_kmh, v->eta_ms, (int16_t)v->x_mm);

        if (v->event_local_pending)
            sm_process_event(SM_EVT_VEHICLE_LOCAL,
                             v->id, v->speed_kmh, v->eta_ms, (int16_t)v->x_mm);

        if (v->event_passed_pending)
            sm_process_event(SM_EVT_VEHICLE_PASSED,
                             v->id, v->speed_kmh, 0, (int16_t)v->x_mm);

        if (v->event_obstaculo_pending)
            sm_process_event(SM_EVT_VEHICLE_OBSTACULO,
                             v->id, v->speed_kmh, 0, (int16_t)v->x_mm);

        tracking_manager_clear_events(v->id);
    }
}


/* ── fsm_aplicar_luz — ponto único de controlo DALI ─────────
   Só actua em transições de estado. */
static system_state_t s_ultimo_estado = STATE_IDLE;

static void fsm_aplicar_luz(void)
{
    system_state_t estado_actual = g_fsm_state;

    if (estado_actual == s_ultimo_estado) return;

    switch (estado_actual) {
        case STATE_LIGHT_ON:
            if (g_fsm_acender_instantaneo) {
                g_fsm_acender_instantaneo = false;
                dali_set_brightness(LIGHT_MAX);
            } else {
                dali_fade_up(g_fsm_last_speed);
            }
            break;
        case STATE_OBSTACULO:
            dali_set_brightness(LIGHT_MAX);
            break;
        case STATE_SAFE_MODE:
            dali_safe_mode();
            break;
        case STATE_IDLE:
        case STATE_MASTER:
        case STATE_AUTONOMO:
            dali_fade_down();
            break;
        default:
            break;
    }

    s_ultimo_estado = estado_actual;

    const char *descricao = "";
    switch (estado_actual) {
        case STATE_LIGHT_ON:  descricao = "→ LUZ ON (fade/instant por ETA)";  break;
        case STATE_OBSTACULO: descricao = "→ LUZ MÁXIMA (obstáculo parado)"; break;
        case STATE_SAFE_MODE: descricao = "→ LUZ 50% (radar em falha)";      break;
        case STATE_AUTONOMO:  descricao = "→ LUZ OFF (modo autónomo)";       break;
        case STATE_MASTER:    descricao = "→ LUZ OFF (master, aguarda)";     break;
        case STATE_IDLE:      descricao = "→ LUZ OFF (repouso)";             break;
        default: break;
    }
    ESP_LOGI(TAG, "[DALI] %s", descricao);
}


/* ── fsm_task — Core 1, Prio 6 ───────────────────────────── */
static void fsm_task(void *arg)
{
    ESP_LOGI(TAG, "fsm_task | Core %d | Prio 6 | a aguardar radar (6s)...",
             xPortGetCoreID());

    /* Aguarda estabilização do HLK-LD2450 (~5s).
       Dividido em blocos de 200ms para manter heartbeat contínuo. */
    for (int i = 0; i < 30; i++) {
        system_monitor_heartbeat(MOD_FSM);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    radar_flush_rx();
    g_fsm_radar_fail_cnt = 0;
    g_fsm_radar_ok_cnt   = 0;
    g_fsm_radar_ok       = true;
    if (g_fsm_state == STATE_SAFE_MODE)
        g_fsm_state = STATE_IDLE;

    ESP_LOGI(TAG, "[SISTEMA] FSM activa — modo hardware real");

    while (1) {
        bool radar_frame = tracking_manager_get_radar_status();

        bool comm_ok   = comm_status_ok();
        bool is_master = comm_is_master();
        state_machine_update(comm_ok, is_master, radar_frame);

        _processa_eventos_tracking();

        fsm_aplicar_luz();

        _atualiza_radar_display();
        display_manager_set_speed((int)state_machine_get_last_speed());

        system_monitor_heartbeat(MOD_FSM);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/* ── state_machine_task_start ────────────────────────────────
   Inicializa tracking_manager antes de arrancar a fsm_task.
   Ordem crítica: tracking pronto antes do primeiro frame do radar. */
void state_machine_task_start(void)
{
    tracking_manager_init();
    ESP_LOGI(TAG, "tracking_manager inicializado");

    xTaskCreatePinnedToCore(
        fsm_task,
        "fsm_task",
        6144,
        NULL,
        6,
        NULL,
        1
    );

    ESP_LOGI(TAG, "fsm_task v5.1 | Core 1 | Prio 6 | Stack 6144B");
}
