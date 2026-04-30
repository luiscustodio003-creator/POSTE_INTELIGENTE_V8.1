/* ============================================================
   COMM MANAGER — DECLARAÇÃO
   @file      comm_manager.h
   @version   3.0  |  2026-04-29
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÕES v2.0 → v3.0:
   - comm_is_master(): SAFE e FAIL contam como ausente
   - comm_left_online() / comm_right_online(): só NEIGHBOR_OK
   - comm_right_known() adicionada
   - comm_left_known() duplicado removido
   - #include <inttypes.h> movido para dentro do guard
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

#endif /* COMM_MANAGER_H */
