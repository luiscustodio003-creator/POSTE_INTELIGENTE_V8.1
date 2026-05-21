/* ============================================================
   COMM MANAGER — DECLARAÇÃO CORRIGIDA
   @file      comm_manager.h
   @version   3.2  |  2026-05-12
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÕES v3.1 → v3.2 (CORRECÇÃO UDP OBSTÁCULO):
   ───────────────────────────────────────────────────────────
   🔴 BUG UDP CORRIGIDO — Falta comunicação de obstáculo

   - ADICIONADO: comm_send_obstaculo(vehicle_id, speed, x_mm)
     Camada de abstracção sobre udp_manager_send_obstaculo().
     Resolve IP do vizinho direito automaticamente.
============================================================ */
#ifndef COMM_MANAGER_H
#define COMM_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

/* ── Ciclo de vida ────────────────────────────────────────── */
bool comm_init(void);

/* ── Estado de rede ───────────────────────────────────────── */
bool comm_status_ok(void);
bool comm_is_master(void);
bool comm_left_online(void);
bool comm_right_online(void);
bool comm_left_known(void);
bool comm_right_known(void);

/* ── Envio de mensagens ───────────────────────────────────── */
void comm_send_tc_inc(float speed, int16_t x_mm);
void comm_send_spd(float speed, int16_t x_mm);
void comm_notify_prev_passed(float speed);
void comm_send_master_claim(void);
void comm_send_master_claim_id(int master_id);

/**
 * @brief Relay de MASTER_CLAIM preservando seq e hop existentes (v3.3)
 *
 * Chamado por fsm_network_master_claim_relay() com os valores lidos via
 * udp_manager_get_relay_seq/hop(). Envia triple-send internamente.
 */
void comm_send_master_claim_relay(int master_id, uint16_t seq, uint8_t hop);

/**
 * @brief Envia notificação de obstáculo ao vizinho direito (NOVO v3.2)
 *
 * QUANDO USAR:
 *   - Em fsm_events.c, caso SM_EVT_VEHICLE_OBSTACULO
 *   - Só quando g_fsm_right_online == true
 *
 * EFEITO NO RECEPTOR:
 *   - Cancela TC_TIMEOUT (sabe que veículo parou)
 *   - Mantém Tc (veículo ainda presente na linha)
 *   - Luz fica acesa até receber PASSED real
 *
 * @param vehicle_id ID do veículo parado (tracking_manager)
 * @param speed      Velocidade quando parou (para logs)
 * @param x_mm       Posição lateral em mm
 */
void comm_send_obstaculo(uint16_t vehicle_id, float speed, int16_t x_mm);

#endif /* COMM_MANAGER_H */
