/* ============================================================
   MÓDULO     : fsm_events
   FICHEIRO   : fsm_events.h — Declarações de eventos da FSM
   VERSÃO     : 1.0  |  2026-04-26
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Processamento de todos os eventos do pipeline tracking → FSM
   e dos callbacks UDP recebidos do comm_manager.

   EVENTOS:
   ────────
   SM_EVT_VEHICLE_DETECTED   → prepara ETA (sem T++)
   SM_EVT_VEHICLE_APPROACHING → pré-acendimento local
   SM_EVT_VEHICLE_PASSED     → T--, SPD ao vizinho direito
   SM_EVT_VEHICLE_LOCAL      → Tc--, T++, luz ON, propaga
   SM_EVT_VEHICLE_OBSTACULO  → STATE_OBSTACULO, luz máxima

   CALLBACKS UDP:
   ──────────────
   on_tc_inc_received()      → Tc++
   on_prev_passed_received() → T--
   on_spd_received()         → agenda ETA
   on_master_claim_received()→ log

   DEPENDÊNCIAS:
   ─────────────
   fsm_core.h, comm_manager.h, dali_manager.h, state_machine.h
============================================================ */

#ifndef FSM_EVENTS_H
#define FSM_EVENTS_H

#include <stdbool.h>
#include <stdint.h>
#include "state_machine.h"

/* ── Ponto de entrada principal ───────────────────────────── */

/**
 * @brief Processa um evento do pipeline tracking → FSM.
 * @param type       Tipo de evento (SM_EVT_*)
 * @param vehicle_id ID do veículo (tracking_manager)
 * @param vel        Velocidade em km/h
 * @param eta_ms     ETA em ms (0 se não disponível)
 * @param x_mm       Posição lateral em mm
 */
void sm_process_event(sm_event_type_t type,
                      uint16_t vehicle_id,
                      float vel,
                      uint32_t eta_ms,
                      int16_t x_mm);

/* ── Callbacks UDP ────────────────────────────────────────── */

/** Recebe TC_INC do poste esquerdo → Tc++ */
void on_tc_inc_received(float speed, int16_t x_mm);

/** Recebe PASSED do poste direito → T-- */
void on_prev_passed_received(void);

/** Recebe SPD → actualiza ETA de pré-acendimento */
void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm);

/** Recebe MASTER_CLAIM → log */
void on_master_claim_received(int from_id);

/* ── Compatibilidade e injecção de teste ─────────────────── */

/** Shim v3.x → delega para SM_EVT_VEHICLE_LOCAL */
void sm_on_radar_detect(float vel);

/** Injeta carro de teste via debugger (JTAG/GDB) */
void sm_inject_test_car(float vel);

/* ── Gestão de vizinhos (chamadas pela fsm_network) ──────── */
void sm_on_right_neighbor_offline(void);
void sm_on_right_neighbor_online(void);

#endif /* FSM_EVENTS_H */
