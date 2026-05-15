/* ============================================================
   UDP MANAGER — DECLARAÇÃO CORRIGIDA
   @file      udp_manager.h
   @version   5.4  |  2026-05-14
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÕES v5.3 → v5.4:
   ─────────────────────────
   - REMOVIDAS: udp_manager_get_all_neighbors(), udp_manager_reset_neighbor()
   - REMOVIDA declaração de udp_task_run (função agora static)

   ALTERAÇÕES v5.2 → v5.3:
   ─────────────────────────
   - ADICIONADO: udp_manager_send_obstaculo()
   - ADICIONADO: on_obstaculo_received() callback weak
   - FORMATO UDP: "OBSTACULO:<from_id>:<vehicle_id>:<speed>:<x_mm>"
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
    uint32_t obstaculo_enviados;    /* NOVO v5.3 */
    uint32_t obstaculo_recebidos;   /* NOVO v5.3 */
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
bool udp_manager_send_master_claim_id(const char *ip, int master_id);

/**
 * @brief Envia notificação de obstáculo ao vizinho direito (NOVO v5.3)
 *
 * Formato: "OBSTACULO:<from_id>:<vehicle_id>:<speed>:<x_mm>"
 *
 * QUANDO USAR:
 *   - Em fsm_events.c, caso SM_EVT_VEHICLE_OBSTACULO
 *   - Só se g_fsm_right_online == true
 *
 * EFEITO NO RECEPTOR:
 *   - Cancela TC_TIMEOUT (sabe que veículo parou)
 *   - Mantém Tc (veículo ainda presente na linha)
 *   - Luz fica acesa até receber PASSED real
 *
 * @param ip         IP do vizinho direito
 * @param vehicle_id ID do veículo parado
 * @param speed      Velocidade quando parou (para logs)
 * @param x_mm       Posição lateral em mm
 * @return true se enviado com sucesso
 */
bool udp_manager_send_obstaculo(const char *ip, uint16_t vehicle_id, 
                                float speed, int16_t x_mm);


/* ============================================================
   CONSULTA DE VIZINHOS
============================================================ */
void        udp_manager_get_neighbors(char *nebL, char *nebR);
neighbor_t *udp_manager_get_neighbor_by_pos(int position);
void        udp_manager_get_stats(udp_stats_t *out);


/* ============================================================
   CALLBACKS — implementados em fsm_events.c
   Versões weak definidas em udp_manager.c
============================================================ */
void on_tc_inc_received(float speed, int16_t x_mm);
void on_prev_passed_received(float speed);
void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm);
void on_master_claim_received(int from_id);
void on_master_claim_received_ext(int from_id, int master_id);

/**
 * @brief Callback quando vizinho esquerdo notifica obstáculo (NOVO v5.3)
 *
 * CHAMADO POR: udp_manager quando processa "OBSTACULO:..."
 *
 * IMPLEMENTADO EM: fsm_events.c
 *
 * ACÇÃO ESPERADA:
 *   - Cancela TC_TIMEOUT (sabe que veículo parou no poste anterior)
 *   - Mantém Tc inalterado (veículo ainda presente na "linha")
 *   - Actualiza g_fsm_last_detect_ms
 *   - Log estruturado para diagnóstico
 *
 * @param vehicle_id ID do veículo parado
 * @param speed      Velocidade quando parou
 * @param x_mm       Posição lateral em mm
 */
void on_obstaculo_received(uint16_t vehicle_id, float speed, int16_t x_mm);


/* ============================================================
   SOCKET
============================================================ */
int  udp_manager_get_socket(void);


#endif /* UDP_MANAGER_H */
