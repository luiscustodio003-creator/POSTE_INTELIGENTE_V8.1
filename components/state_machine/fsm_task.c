/* ============================================================
   MÓDULO     : fsm_task
   FICHEIRO   : fsm_task.c — Orquestração da Task FSM
   VERSÃO     : 2.3 — CORRIGIDO (Handover & Nomenclatura)
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   - Consome eventos do tracking_manager e injeta-os na FSM.
   - Sincroniza o estado da FSM com o display.
   - Ponto de entrada para a criação da task.
   - Resolve o erro de compilação usando 'event_approach_pending'.
============================================================ */

#include "fsm_core.h"
#include "fsm_events.h"
#include "tracking_manager.h"
#include "display_manager.h"
#include "comm_manager.h"
#include "radar_manager.h"
#include "dali_manager.h"
#include "system_monitor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "FSM_TSK";

/**
 * @brief Helper para atualizar a visualização do radar no display.
 */
static void _atualiza_radar_display(void) {
    tracked_vehicle_t veiculos[TRK_MAX_VEHICLES];
    uint8_t count = 0;
    if (tracking_manager_get_vehicles(veiculos, &count)) {
        radar_obj_t objs[RADAR_MAX_OBJ];
        for (int i = 0; i < count && i < RADAR_MAX_OBJ; i++) {
            objs[i].x_mm = veiculos[i].x_mm;
            objs[i].y_mm = veiculos[i].y_mm;
            objs[i].speed_kmh = veiculos[i].speed_kmh;
        }
        display_manager_set_radar(objs, count);
    }
}

/* ============================================================
   _atualiza_radar_display
   ────────────────────────────────────────────────────────────
   @brief Filtra e envia alvos confirmados para o display.
   @note  Impede que o Tc (aviso de rede) crie pontos fantasmas
          no radar antes da detecção física local.
============================================================ */



void fsm_task(void *pvParameters) {
    ESP_LOGI(TAG, "Task FSM iniciada no Core %d", xPortGetCoreID());

    while (1) {
        /* CORREÇÃO: comm_status_ok() verifica se o Wi-Fi está ligado.
           Isto permite que o último poste funcione mesmo sem vizinho à direita. */
        bool comm_ok = comm_status_ok(); 
        bool is_master = comm_is_master();
        
        // Verifica se o radar está a enviar dados
        bool radar_teve_frame = tracking_manager_get_radar_status();

        // Ciclo principal da FSM (Trata timeouts e transições de estado)
        state_machine_update(comm_ok, is_master, radar_teve_frame);

        // Processamento de eventos vindos do Tracking
        tracked_vehicle_t v_list[TRK_MAX_VEHICLES];
        uint8_t v_count = 0;
        
        if (tracking_manager_get_vehicles(v_list, &v_count)) {
            for (int i = 0; i < v_count; i++) {
                tracked_vehicle_t *v = &v_list[i];

                // 1. Deteção Inicial (Ligar Luz)
                if (v->event_detected_pending)
                    sm_process_event(SM_EVT_VEHICLE_DETECTED, v->id, v->speed_kmh, v->eta_ms, v->x_mm);

                // 2. MOMENTO LOCAL / HANDOVER
                // Usamos 'event_approach_pending' que é o trigger real do seu radar
                if (v->event_approach_pending)
                    sm_process_event(SM_EVT_VEHICLE_LOCAL, v->id, v->speed_kmh, v->eta_ms, v->x_mm);

                // 3. Saída do Radar
                if (v->event_passed_pending)
                    sm_process_event(SM_EVT_VEHICLE_PASSED, v->id, v->speed_kmh, 0, v->x_mm);

                // 4. Obstáculo Estático
                if (v->event_obstaculo_pending)
                    sm_process_event(SM_EVT_VEHICLE_OBSTACULO, v->id, v->speed_kmh, 0, v->x_mm);

                tracking_manager_clear_events(v->id);
            }
        }

        // Sincronização com o Display e Hardware
        display_manager_set_status(state_machine_get_state_name());
        display_manager_set_traffic(state_machine_get_T(), state_machine_get_Tc());
        display_manager_set_speed((int)state_machine_get_last_speed());
        display_manager_set_hardware(radar_get_status_str(), radar_is_connected(), (uint8_t)dali_get_brightness());
        _atualiza_radar_display();

        system_monitor_heartbeat(MOD_FSM);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Inicializa e lança a Task da FSM
 */
void state_machine_task_start(void) {
    tracking_manager_init();
    xTaskCreatePinnedToCore(fsm_task, "fsm_task", 4096, NULL, 6, NULL, 1);
    ESP_LOGI(TAG, "Task FSM lançada com sucesso.");
}

