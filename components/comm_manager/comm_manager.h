/* ============================================================
   COMM MANAGER — DECLARAÇÃO
   @file      comm_manager.h
   @version   3.1  |  2026-05-02
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÕES v3.0 → v3.1:
   ─────────────────────────
   - ADICIONADO: comm_send_master_claim_id(int master_id)
     Relay de MASTER_CLAIM preservando o ID do MASTER original.
     Usado por fsm_network para propagar liderança em cadeia
     sem perder quem é o MASTER real ao longo dos relays.
     Diferente de comm_send_master_claim() que usa POSTE_ID local.
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

/**
 * @brief Relay de MASTER_CLAIM com ID do MASTER real.
 *
 * Envia "MASTER_CLAIM:<POSTE_ID>:<master_id>" ao vizinho direito.
 * Preserva master_id ao longo de toda a cadeia de relays,
 * garantindo que todos os postes sabem quem é o MASTER original.
 *
 * @param master_id  ID do MASTER real (poste mais à esquerda activo)
 */
void comm_send_master_claim_id(int master_id);

#endif /* COMM_MANAGER_H */
