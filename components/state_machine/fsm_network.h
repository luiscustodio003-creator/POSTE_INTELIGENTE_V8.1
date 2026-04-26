/* ============================================================
   MÓDULO     : fsm_network
   FICHEIRO   : fsm_network.h — Declarações de rede da FSM
   VERSÃO     : 1.0  |  2026-04-26
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Gestão de toda a lógica de rede e estados degradados:
   - Passo 3:  conectividade do vizinho direito
   - Passo 4:  vizinho esquerdo offline/online
   - Passo 10: papel MASTER
   - Passo 11: SAFE_MODE, AUTONOMO

   DEPENDÊNCIAS:
   ─────────────
   fsm_core.h, fsm_events.h, comm_manager.h, system_config.h
============================================================ */

#ifndef FSM_NETWORK_H
#define FSM_NETWORK_H

#include <stdbool.h>

/**
 * @brief Passos 3 e 4 — gestão de vizinhos esq. e dir.
 * @param comm_ok   true se UDP operacional
 * @param is_master true se este poste é MASTER
 */
void fsm_network_vizinhos(bool comm_ok, bool is_master);

/**
 * @brief Passo 10 — gestão do papel MASTER.
 * @param comm_ok   true se UDP operacional
 * @param is_master true se este poste é MASTER
 */
void fsm_network_master(bool comm_ok, bool is_master);

/**
 * @brief Passo 11 — estados degradados SAFE_MODE e AUTONOMO.
 * @param comm_ok   true se UDP operacional
 * @param is_master true se este poste é MASTER
 */
void fsm_network_estados_degradados(bool comm_ok, bool is_master);

#endif /* FSM_NETWORK_H */
