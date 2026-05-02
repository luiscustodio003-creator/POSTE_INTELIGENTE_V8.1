/* ============================================================
   MÓDULO     : fsm_events
   FICHEIRO   : fsm_events.h — Declarações de eventos da FSM
   VERSÃO     : 3.2  |  2026-05-02
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Processamento de todos os eventos do pipeline tracking → FSM
   e dos callbacks UDP recebidos do comm_manager.

   ALTERAÇÕES v3.1 → v3.2:
   ─────────────────────────
   - ADICIONADO: on_master_claim_received_ext(from_id, master_id)
     Callback com dois parâmetros para suporte ao relay em cadeia.
     Delega para fsm_network_master_claim_relay().

   DEPENDÊNCIAS:
   ─────────────
   fsm_core.h, fsm_network.h, comm_manager.h, state_machine.h
============================================================ */

#ifndef FSM_EVENTS_H
#define FSM_EVENTS_H

#include <stdbool.h>
#include <stdint.h>
#include "state_machine.h"

/* ── Ponto de entrada principal ───────────────────────────── */

/**
 * @brief Processa um evento do pipeline tracking → FSM.
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
void on_prev_passed_received(float speed);

/** Recebe SPD → actualiza ETA de pré-acendimento */
void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm);

/**
 * @brief Callback legado — recebe MASTER_CLAIM formato antigo.
 *        Delega para on_master_claim_received_ext(from_id, from_id).
 */
void on_master_claim_received(int from_id);

/**
 * @brief Callback novo v3.2 — recebe MASTER_CLAIM com relay completo.
 *        Delega para fsm_network_master_claim_relay(from_id, master_id).
 */
void on_master_claim_received_ext(int from_id, int master_id);

/* ── Compatibilidade e injecção de teste ─────────────────── */

/** Shim v3.x → delega para SM_EVT_VEHICLE_LOCAL */
void sm_on_radar_detect(float vel);

/** Injeta carro de teste via debugger (JTAG/GDB) */
void sm_inject_test_car(float vel);

/* ── Gestão de vizinhos (chamadas pela fsm_network) ──────── */
void sm_on_right_neighbor_offline(void);
void sm_on_right_neighbor_online(void);

#endif /* FSM_EVENTS_H */
