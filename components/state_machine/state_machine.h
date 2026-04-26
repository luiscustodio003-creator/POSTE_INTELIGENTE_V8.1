/* ============================================================
   MÓDULO     : state_machine
   FICHEIRO   : state_machine.h — Ponto de entrada público
   VERSÃO     : 1.0  |  2026-04-26
   PROJECTO   : Poste Inteligente v8

   Este ficheiro mantém a interface pública original.
   Internamente a FSM está dividida em sub-módulos:
     fsm_core    — variáveis e getters
     fsm_events  — eventos e callbacks UDP
     fsm_network — vizinhos, MASTER, SAFE_MODE, AUTONOMO
     fsm_timer   — timers e timeouts
     fsm_sim     — simulador USE_RADAR=0
     fsm_task    — loop 100ms
============================================================ */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>
#include "system_config.h"

/* ============================================================
   ESTADOS DA FSM
============================================================ */
typedef enum {
    STATE_IDLE       = 0,
    STATE_LIGHT_ON,
    STATE_SAFE_MODE,
    STATE_MASTER,
    STATE_AUTONOMO,
    STATE_OBSTACULO,
} system_state_t;

/* ============================================================
   TIPOS DE EVENTO
============================================================ */
typedef enum {
    SM_EVT_VEHICLE_DETECTED  = 0,
    SM_EVT_VEHICLE_APPROACHING,
    SM_EVT_VEHICLE_PASSED,
    SM_EVT_VEHICLE_LOCAL,
    SM_EVT_VEHICLE_OBSTACULO,
} sm_event_type_t;

/* ============================================================
   CICLO DE VIDA
============================================================ */
void state_machine_init(void);
void state_machine_update(bool comm_ok, bool is_master,
                          bool radar_teve_frame);
void state_machine_task_start(void);

/* ============================================================
   PROCESSAMENTO DE EVENTOS
============================================================ */
void sm_process_event(sm_event_type_t type,
                      uint16_t vehicle_id,
                      float vel,
                      uint32_t eta_ms,
                      int16_t x_mm);

/* ============================================================
   GESTÃO DE VIZINHOS
============================================================ */
void sm_on_right_neighbor_offline(void);
void sm_on_right_neighbor_online(void);
void sm_inject_test_car(float vel);
void sm_on_radar_detect(float vel);

/* ============================================================
   GETTERS
============================================================ */
system_state_t state_machine_get_state(void);
const char    *state_machine_get_state_name(void);
int            state_machine_get_T(void);
int            state_machine_get_Tc(void);
float          state_machine_get_last_speed(void);
bool           state_machine_radar_ok(void);
bool           sm_is_obstaculo(void);

/* ============================================================
   SIMULADOR — apenas USE_RADAR == 0
============================================================ */
#if USE_RADAR == 0
void sim_init_mutex(void);
bool sim_get_objeto(float *x_mm, float *y_mm);
void sim_notificar_chegada(float vel_kmh, int16_t x_mm);
void _sim_update(void);
#endif

#endif /* STATE_MACHINE_H */