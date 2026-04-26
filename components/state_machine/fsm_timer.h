/* ============================================================
   MÓDULO     : fsm_timer
   FICHEIRO   : fsm_timer.h — Declarações dos timers da FSM
   VERSÃO     : 1.0  |  2026-04-26
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Gestão de todos os timers e timeouts da FSM:
   - Passo 5:  T preso quando vizinho esquerdo offline
   - Passo 6:  pré-acendimento por ETA
   - Passo 7:  timeout de apagamento (T=0, Tc=0)
   - Passo 8:  remoção automática de obstáculo
   - Passo 9:  timeout de segurança Tc
   - Passo 12: heartbeat MASTER_CLAIM

   DEPENDÊNCIAS:
   ─────────────
   fsm_core.h, comm_manager.h, dali_manager.h, system_config.h
============================================================ */

#ifndef FSM_TIMER_H
#define FSM_TIMER_H

#include <stdbool.h>

/**
 * @brief Executa todos os passos de timer do ciclo 100ms.
 * @param comm_ok   true se UDP operacional
 * @param is_master true se este poste é MASTER
 */
void fsm_timer_update(bool comm_ok, bool is_master);

#endif /* FSM_TIMER_H */
