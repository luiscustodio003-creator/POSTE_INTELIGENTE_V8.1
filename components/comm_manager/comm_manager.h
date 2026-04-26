#include <inttypes.h>
/* ============================================================
   COMM MANAGER — DECLARAÇÃO
   @file      comm_manager.h
   @version   2.0  |  2026-04-07
   Projecto  : Poste Inteligente v8
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

   Camada de abstracção sobre o udp_manager.
   A state_machine chama estas funções sem conhecer IPs.
   O comm_manager resolve o vizinho correcto e calcula ETA.

   Alterações v1.x → v2.0:
   ─────────────────────────
   1. comm_send_tc_inc() e comm_send_spd() aceitam x_mm.
      Propaga posição lateral do alvo ao poste seguinte.
   2. comm_notify_prev_passed() mantém assinatura (não usa x_mm).
============================================================ */
#ifndef COMM_MANAGER_H
#define COMM_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

/* ── Ciclo de vida ────────────────────────────────────────── */

/**
 * @brief Inicializa UDP e inicia descoberta de vizinhos.
 *        Chamar apenas após Wi-Fi com IP obtido.
 * @return true se socket criado com sucesso
 */
bool comm_init(void);

/* ── Estado de rede ───────────────────────────────────────── */

/** true se UDP está operacional (socket válido) */
bool comm_status_ok(void);

/** true se este poste é MASTER (pos=0 ou viz. esq. offline) */
bool comm_is_master(void);

/** true se vizinho esquerdo está online */
bool comm_left_online(void);

/** true se vizinho direito está online */
bool comm_right_online(void);

/* ── Envio de mensagens ───────────────────────────────────── */

/**
 * @brief Envia TC_INC ao vizinho direito.
 *        Inclui x_mm para continuidade visual no canvas.
 * @param speed Velocidade do carro (km/h)
 * @param x_mm  Posição lateral do alvo em mm (0 = centro)
 */
void comm_send_tc_inc(float speed, int16_t x_mm);

/**
 * @brief Envia SPD ao vizinho direito com ETA calculado.
 *        ETA = (POSTE_DIST_M - RADAR_DETECT_M) / (speed/3.6)
 *        Inclui x_mm para continuidade visual no canvas.
 * @param speed Velocidade do carro (km/h)
 * @param x_mm  Posição lateral do alvo em mm
 */
void comm_send_spd(float speed, int16_t x_mm);

/**
 * @brief Notifica vizinho esquerdo que carro chegou (T--).
 *        Envia TC_INC com velocidade negativa.
 * @param speed Velocidade do carro (km/h, enviada como negativo)
 */
void comm_notify_prev_passed(float speed);

/**
 * @brief Envia MASTER_CLAIM em cadeia ao vizinho direito.
 */
void comm_send_master_claim(void);

bool comm_left_known(void);

#endif /* COMM_MANAGER_H */

