/* ============================================================
   UDP MANAGER — DECLARAÇÃO
   @file      udp_manager.h
   @version   5.2  |  2026-05-02
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÕES v5.1 → v5.2:
   ─────────────────────────
   - ADICIONADO: udp_manager_send_master_claim_id(ip, master_id)
     Envia "MASTER_CLAIM:<from_id>:<master_id>" preservando o
     ID do MASTER original ao longo de toda a cadeia de relays.

   - ADICIONADO: on_master_claim_received_ext(from_id, master_id)
     Callback com dois parâmetros para suportar o novo formato.
     Versão weak definida no udp_manager, implementada no fsm_events.

   - MANTIDO: on_master_claim_received(from_id) — compatibilidade
     com código existente. Versão weak faz relay com from_id==master_id.

   - ALTERADO: parser de MASTER_CLAIM suporta ambos os formatos:
     "MASTER_CLAIM:<id>"          → formato antigo (from_id = master_id)
     "MASTER_CLAIM:<from>:<master>" → formato novo (relay completo)
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
    NEIGHBOR_SAFE      = 2,
    NEIGHBOR_AUTO      = 3,
    NEIGHBOR_OBSTACULO = 4,
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

/**
 * @brief Envia MASTER_CLAIM com ID do MASTER real (relay em cadeia).
 *
 * Formato: "MASTER_CLAIM:<POSTE_ID>:<master_id>"
 * O from_id (POSTE_ID) muda a cada relay; master_id mantém-se.
 *
 * @param ip        IP do destinatário
 * @param master_id ID do MASTER real (não muda ao longo dos relays)
 */
bool udp_manager_send_master_claim_id(const char *ip, int master_id);


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
void on_prev_passed_received(float speed);
void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm);

/**
 * @brief Callback legado — recebe MASTER_CLAIM com um só campo.
 *        Mantido para compatibilidade. A versão ext é preferida.
 */
void on_master_claim_received(int from_id);

/**
 * @brief Callback novo — recebe MASTER_CLAIM com from_id e master_id.
 *        Implementado em fsm_events.c, delega para fsm_network_master_claim_relay().
 */
void on_master_claim_received_ext(int from_id, int master_id);


/* ============================================================
   TASK E SOCKET
============================================================ */
void udp_task_run(void *arg);
int  udp_manager_get_socket(void);


#endif /* UDP_MANAGER_H */
