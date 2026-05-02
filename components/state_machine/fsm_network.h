/* ============================================================
   MÓDULO     : fsm_network
   FICHEIRO   : fsm_network.h — Declarações de rede da FSM
   VERSÃO     : 3.0  |  2026-05-02
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Gestão de toda a lógica de rede e estados degradados:
   - Passo 3:  conectividade do vizinho direito
   - Passo 4:  vizinho esquerdo offline/online
   - Passo 10: papel MASTER (eleição, promoção, cedência)
   - Passo 11: SAFE_MODE, AUTONOMO
   - Relay:    propagação de MASTER_CLAIM em cadeia

   DEPENDÊNCIAS:
   ─────────────
   fsm_core.h, fsm_events.h, comm_manager.h, system_config.h

   ALTERAÇÕES v1.0 → v3.0:
   ─────────────────────────
   - ADICIONADO: fsm_network_master_claim_relay() — relay de MASTER_CLAIM
   - ADICIONADO: fsm_network_get_master_id()      — getter do MASTER actual
============================================================ */

#ifndef FSM_NETWORK_H
#define FSM_NETWORK_H

#include <stdbool.h>

/**
 * @brief Passos 3 e 4 — gestão de vizinhos esq. e dir.
 */
void fsm_network_vizinhos(bool comm_ok, bool is_master);

/**
 * @brief Passo 10 — gestão do papel MASTER.
 */
void fsm_network_master(bool comm_ok, bool is_master);

/**
 * @brief Passo 11 — estados degradados SAFE_MODE e AUTONOMO.
 */
void fsm_network_estados_degradados(bool comm_ok, bool is_master);

/**
 * @brief Relay de MASTER_CLAIM em cadeia.
 *        Chamado por on_master_claim_received_ext() em fsm_events.c.
 *
 * @param from_id   ID do poste que enviou a mensagem UDP
 * @param master_id ID do MASTER real (não muda ao longo dos relays)
 */
void fsm_network_master_claim_relay(int from_id, int master_id);

/**
 * @brief Retorna o ID do MASTER actual conhecido.
 */
int fsm_network_get_master_id(void);

#endif /* FSM_NETWORK_H */
