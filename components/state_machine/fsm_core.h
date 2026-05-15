/* fsm_core.h — v1.2 | 2026-05-04 | Poste Inteligente v8
   Variáveis de estado partilhadas entre sub-módulos da FSM. */

#ifndef FSM_CORE_H
#define FSM_CORE_H

#include <stdbool.h>
#include <stdint.h>
#include "state_machine.h"

/* ── Variáveis de estado ──────────────────────────────────── */
extern system_state_t g_fsm_state;
extern int            g_fsm_T;
extern int            g_fsm_Tc;
extern int            g_fsm_enviados_dir;
extern float          g_fsm_last_speed;
extern bool           g_fsm_apagar_pend;
extern bool           g_fsm_radar_ok;
extern int            g_fsm_radar_fail_cnt;
extern int            g_fsm_radar_ok_cnt;
extern bool           g_fsm_right_online;

extern uint64_t  g_fsm_last_detect_ms;
extern uint64_t  g_fsm_left_offline_ms;
extern uint64_t  g_fsm_tc_timeout_ms;
extern bool      g_fsm_left_was_offline;
extern uint64_t  g_fsm_acender_em_ms;
extern uint64_t  g_fsm_master_claim_ms;
extern uint64_t  g_fsm_sem_vizinho_ms;
extern uint64_t  g_fsm_obstaculo_last_ms;

/* ID do último veículo que gerou TC_INC — guarda contra TC_INC duplicado. */
extern uint16_t g_fsm_tc_last_vehicle_id;

/* Pré-acendimento instantâneo: true quando ETA < tempo de fade (carro muito rápido). */
extern bool g_fsm_acender_instantaneo;

/* ── Utilitários internos ─────────────────────────────────── */
uint64_t fsm_agora_ms(void);
void     fsm_agendar_apagar(void);
void     fsm_verificar_radar(bool teve_frame, bool comm_ok);
void     fsm_obstaculo_keepalive(void);

/* ── Ciclo de vida ────────────────────────────────────────── */
void state_machine_init(void);
void state_machine_update(bool comm_ok, bool is_master, bool radar_teve_frame);
void state_machine_task_start(void);

/* ── Getters públicos ─────────────────────────────────────── */
system_state_t state_machine_get_state(void);
const char    *state_machine_get_state_name(void);
int            state_machine_get_T(void);
int            state_machine_get_Tc(void);
float          state_machine_get_last_speed(void);
bool           state_machine_radar_ok(void);
bool           sm_is_obstaculo(void);
uint8_t        fsm_core_get_duty_cycle(void);

#endif /* FSM_CORE_H */
