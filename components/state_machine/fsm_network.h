/* ============================================================
   MÓDULO     : fsm_network
   FICHEIRO   : fsm_network.h
   VERSÃO     : 2.0  |  2026-04-27
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno

   RESPONSABILIDADE:
   ─────────────────
   Gestão de toda a lógica de rede e estados degradados.
   Chamado por state_machine_update() a 100ms.

   PASSOS:
   ────────
   fsm_network_vizinhos()          — Passos 3,4: online/offline
   fsm_network_master()            — Passo 5:    papel MASTER
   fsm_network_estados_degradados()— Passo 6:    AUTONOMO, SAFE_MODE

   DEPENDÊNCIAS:
   ─────────────
   fsm_core.h, fsm_events.h, comm_manager.h, system_config.h
============================================================ */

#ifndef FSM_NETWORK_H
#define FSM_NETWORK_H

#include <stdbool.h>

/** Passos 3,4 — detecta online/offline de ambos os vizinhos */
void fsm_network_vizinhos(bool comm_ok, bool is_master);

/** Passo 5 — gere papel MASTER (P0, temporário, RADAR_FAIL) */
void fsm_network_master(bool comm_ok, bool is_master);

/** Passo 6 — gere AUTONOMO (sem WiFi, sem viz., RADAR_FAIL esq)
              e SAFE_MODE (radar KO) */
void fsm_network_estados_degradados(bool comm_ok, bool is_master);

#endif /* FSM_NETWORK_H */
