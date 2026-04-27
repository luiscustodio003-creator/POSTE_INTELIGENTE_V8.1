/* ============================================================
   COMM MANAGER — DECLARAÇÃO
   @file      comm_manager.h
   @version   4.0  |  2026-04-27
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno

   NOVIDADES v4.0:
   ────────────────
   comm_notify_radar_status(ok) — envia RADAR_FAIL ou RADAR_OK
     a AMBOS os vizinhos (esq e dir).
   comm_left_was_ever_online()  — true se viz.esq já respondeu
     pelo menos uma vez (distingue "nunca descoberto" de "offline").

   PROTOCOLO RADAR_FAIL/RADAR_OK:
   ────────────────────────────────
   Quando radar falha → B envia RADAR_FAIL a A e a C.
     A recebe RADAR_FAIL do dir → AUTONOMO
     C recebe RADAR_FAIL do esq → MASTER imediato
   Quando radar recupera → B envia RADAR_OK a A e a C.
     A recebe RADAR_OK do dir → IDLE (ou MASTER se pos=0)
     C recebe RADAR_OK do esq → cede MASTER → IDLE
============================================================ */
#ifndef COMM_MANAGER_H
#define COMM_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

/* ── Ciclo de vida ──────────────────────────────────────── */
bool comm_init(void);

/* ── Estado ─────────────────────────────────────────────── */
/** true se socket UDP válido (não depende de vizinhos) */
bool comm_status_ok(void);

/** true se este poste tem papel MASTER */
bool comm_is_master(void);

/** true se viz. esquerdo está online */
bool comm_left_online(void);

/** true se viz. direito está online */
bool comm_right_online(void);

/** true se viz. esquerdo já respondeu pelo menos uma vez.
    Distingue "nunca descoberto" de "estava online, ficou offline". */
bool comm_left_was_ever_online(void);

/* ── Envio de mensagens ──────────────────────────────────── */

/** TC_INC ao viz.dir — anuncia detecção com velocidade e posição */
void comm_send_tc_inc(float speed, int16_t x_mm);

/** SPD ao viz.dir — envia ETA calculado para pré-acendimento */
void comm_send_spd(float speed, int16_t x_mm);

/** PASSED ao viz.esq — speed negativo codifica sinal PASSED */
void comm_notify_prev_passed(float speed);

/** MASTER_CLAIM ao viz.dir — propaga liderança em cadeia */
void comm_send_master_claim(void);

/**
 * @brief Envia RADAR_FAIL ou RADAR_OK a ambos os vizinhos.
 *        Chamado por fsm_verificar_radar() quando muda estado.
 * @param radar_ok true = RADAR_OK, false = RADAR_FAIL
 */
void comm_notify_radar_status(bool radar_ok);

#endif /* COMM_MANAGER_H */
