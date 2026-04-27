//#include <inttypes.h>
/* ============================================================
   UDP MANAGER — DECLARAÇÃO
   @file      udp_manager.h
   @version   4.0  |  2026-04-07
   Projecto  : Poste Inteligente v8
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

   Alterações v3.4 → v4.0:
   ─────────────────────────
   1. TC_INC e SPD incluem x_mm (posição lateral do alvo em mm).
      Garante continuidade visual do ponto no canvas entre postes.
      Retro-compatível: se campo ausente no pacote, usa 0 (centro).

   2. Assinaturas dos callbacks actualizadas:
      on_tc_inc_received(speed, x_mm)
      on_spd_received(speed, eta_ms, x_mm)

   3. udp_task_run() exposta como símbolo público para
      xTaskCreatePinnedToCore() em udp_manager_task_start().

   4. udp_manager_task_start() — cria task no Core 0.

   Protocolo v4.0 (texto simples, separado por ':'):
   ───────────────────────────────────────────────────
   DISCOVER:<id>:<pos>
   STATUS:<id>:<estado>
   TC_INC:<id>:<vel>:<x_mm>
   SPD:<id>:<vel>:<eta_ms>:<dist_m>:<x_mm>
   MASTER_CLAIM:<id>
============================================================ */
#ifndef UDP_MANAGER_H
#define UDP_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "system_config.h"

/* ── Estado de vizinhos ───────────────────────────────────── */
typedef enum {
    NEIGHBOR_OK        = 0,
    NEIGHBOR_OFFLINE   = 1,
    NEIGHBOR_FAIL      = 2,
    NEIGHBOR_SAFE      = 3,
    NEIGHBOR_AUTO      = 4,
    NEIGHBOR_OBSTACULO = 5,
} neighbor_status_t;

typedef struct {
    char              ip[MAX_IP_LEN];
    int               id;
    int               position;
    neighbor_status_t status;
    bool              active;
    bool              discover_ok;
    uint32_t          last_seen;
} neighbor_t;

/* ── Ciclo de vida ────────────────────────────────────────── */
bool udp_manager_init(void);
void udp_manager_task_start(void);   /* Core 0, Prio 5, 4096B */

/* ── Envio ────────────────────────────────────────────────── */
void udp_manager_discover(void);
bool udp_manager_send_tc_inc(const char *ip, float speed, int16_t x_mm);
bool udp_manager_send_spd(const char *ip, float speed,
                           uint32_t eta_ms, uint32_t dist_m, int16_t x_mm);
bool udp_manager_send_status(const char *ip, neighbor_status_t status);
bool udp_manager_send_master_claim(const char *ip);

/* ── Consulta de vizinhos ─────────────────────────────────── */
void        udp_manager_get_neighbors(char *nebL, char *nebR);
neighbor_t *udp_manager_get_neighbor_by_pos(int position);
size_t      udp_manager_get_all_neighbors(neighbor_t *list, size_t max);

/* ── Callbacks — implementados em fsm_events.c ────────────── */
void on_tc_inc_received(float speed, int16_t x_mm);
void on_prev_passed_received(void);
void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm);
void on_master_claim_received(int from_id);

/* Notificação de estado do radar do vizinho.
   from_left=true  → mensagem veio do viz. esquerdo
   from_left=false → mensagem veio do viz. direito */
void on_radar_fail_received(bool from_left);
void on_radar_ok_received(bool from_left);

/* ── Task interna exposta para pinagem de core ────────────── */
void udp_task_run(void *arg);

int udp_manager_get_socket(void);

#endif /* UDP_MANAGER_H */

