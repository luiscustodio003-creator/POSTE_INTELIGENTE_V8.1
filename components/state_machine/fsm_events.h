/* ============================================================
   MÓDULO     : fsm_events
   FICHEIRO   : fsm_events.h
   VERSÃO     : 2.0  |  2026-04-27
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno

   RESPONSABILIDADE:
   ─────────────────
   Processamento de eventos do pipeline tracking→FSM
   e callbacks UDP recebidos do udp_manager.

   EVENTOS DO RADAR (sm_process_event):
   ──────────────────────────────────────
   SM_EVT_VEHICLE_DETECTED   → T++, ETA, luz, TC_INC→dir
   SM_EVT_VEHICLE_APPROACHING → actualiza ETA local
   SM_EVT_VEHICLE_PASSED     → T--, PASSED→esq (única fonte)
   SM_EVT_VEHICLE_LOCAL      → T++, Tc--, luz, TC_INC→dir
   SM_EVT_VEHICLE_OBSTACULO  → luz máxima, PASSED→dir

   CALLBACKS UDP (chamados pela udp_task):
   ────────────────────────────────────────
   on_tc_inc_received()       → Tc++, agenda ETA
   on_prev_passed_received()  → T--, agenda apagamento
   on_spd_received()          → refina ETA
   on_master_claim_received() → log
   on_radar_fail_received()   → viz notificou radar KO
                                 viz.esq→AUTONOMO, viz.dir→MASTER
   on_radar_ok_received()     → viz notificou radar recuperado

   DEPENDÊNCIAS:
   ─────────────
   fsm_core.h, comm_manager.h, dali_manager.h, state_machine.h
============================================================ */

#ifndef FSM_EVENTS_H
#define FSM_EVENTS_H

#include <stdbool.h>
#include <stdint.h>
#include "state_machine.h"

/* ── Ponto de entrada de eventos do radar ─────────────────── */
void sm_process_event(sm_event_type_t type,
                      uint16_t vehicle_id,
                      float vel,
                      uint32_t eta_ms,
                      int16_t x_mm);

/* ── Callbacks UDP ────────────────────────────────────────── */

/** Poste esquerdo detectou veículo → Tc++, agenda ETA */
void on_tc_inc_received(float speed, int16_t x_mm);

/** Poste direito confirmou chegada → T--, agenda apagamento */
void on_prev_passed_received(void);

/** Poste esquerdo envia ETA real → refina pré-acendimento */
void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm);

/** Poste vizinho reafirma liderança */
void on_master_claim_received(int from_id);

/**
 * @brief Vizinho notificou que o seu radar falhou.
 *        Se vier do viz.ESQUERDO → este poste entra em AUTONOMO
 *        (não receberá mais TC_INC nem PASSED desse lado).
 *        Se vier do viz.DIREITO → este poste assume MASTER
 *        (viz.dir não conseguirá enviar PASSED de volta).
 * @param from_left true se a notificação vem do viz. esquerdo
 */
void on_radar_fail_received(bool from_left);

/**
 * @brief Vizinho notificou que o seu radar recuperou.
 *        Repõe o modo normal: AUTONOMO→IDLE, MASTER→IDLE.
 * @param from_left true se a notificação vem do viz. esquerdo
 */
void on_radar_ok_received(bool from_left);

/* ── Gestão de vizinhos ───────────────────────────────────── */
void sm_on_right_neighbor_offline(void);
void sm_on_right_neighbor_online(void);

/* ── Compatibilidade e teste ──────────────────────────────── */
void sm_on_radar_detect(float vel);
void sm_inject_test_car(float vel);

#endif /* FSM_EVENTS_H */
