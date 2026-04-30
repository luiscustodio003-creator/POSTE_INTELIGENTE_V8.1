/* ============================================================
   UDP MANAGER — DECLARAÇÃO
   @file      udp_manager.h
   @version   5.1  |  2026-04-29
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÃO v5.0 → v5.1:
   - on_prev_passed_received() corrigida para float speed
     (era void — inconsistente com fsm_events.c e state_machine.h)
============================================================ */
#ifndef UDP_MANAGER_H
#define UDP_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "system_config.h"


/* ============================================================
   ESTADO DE VIZINHOS
============================================================ */
typedef enum {
    NEIGHBOR_OK        = 0,
    NEIGHBOR_OFFLINE   = 1,
    NEIGHBOR_FAIL      = 2,
    NEIGHBOR_SAFE      = 3,
    NEIGHBOR_AUTO      = 4,
    NEIGHBOR_OBSTACULO = 5,
} neighbor_status_t;


/* ============================================================
   ESTRUTURA DE VIZINHO
============================================================ */
typedef struct {
    char              ip[MAX_IP_LEN];
    int               id;
    int               position;
    neighbor_status_t status;
    bool              active;
    bool              discover_ok;
    uint32_t          last_seen;
} neighbor_t;


/* ============================================================
   ESTATÍSTICAS DE DIAGNÓSTICO
============================================================ */
typedef struct {
    uint32_t pkts_enviados;
    uint32_t pkts_recebidos;
    uint32_t timeouts_vizinhos;
    uint32_t tc_inc_enviados;
    uint32_t tc_inc_recebidos;
} udp_stats_t;


/* ============================================================
   CICLO DE VIDA
============================================================ */
bool udp_manager_init(void);
void udp_manager_task_start(void);


/* ============================================================
   ENVIO DE MENSAGENS
============================================================ */
void udp_manager_discover(void);
bool udp_manager_send_tc_inc(const char *ip, float speed, int16_t x_mm);
bool udp_manager_send_passed(const char *ip, float speed);
bool udp_manager_send_spd(const char *ip, float speed,
                           uint32_t eta_ms, uint32_t dist_m, int16_t x_mm);
bool udp_manager_send_status(const char *ip, neighbor_status_t status);
bool udp_manager_send_master_claim(const char *ip);


/* ============================================================
   CONSULTA DE VIZINHOS
============================================================ */
void        udp_manager_get_neighbors(char *nebL, char *nebR);
neighbor_t *udp_manager_get_neighbor_by_pos(int position);
size_t      udp_manager_get_all_neighbors(neighbor_t *list, size_t max);
void        udp_manager_reset_neighbor(int position);
void        udp_manager_get_stats(udp_stats_t *out);


/* ============================================================
   CALLBACKS — implementados em fsm_events.c
   Versões weak definidas em udp_manager.c
============================================================ */
void on_tc_inc_received(float speed, int16_t x_mm);
void on_prev_passed_received(float speed);   /* CORRIGIDO: era void */
void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm);
void on_master_claim_received(int from_id);


/* ============================================================
   TASK E SOCKET
============================================================ */
void udp_task_run(void *arg);
int  udp_manager_get_socket(void);


#endif /* UDP_MANAGER_H */
