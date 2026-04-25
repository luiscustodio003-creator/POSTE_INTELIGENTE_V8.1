/* ============================================================
   UDP MANAGER — DECLARAÇÃO
   @file      udp_manager.h
   @version   5.0  |  2026-04-24
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Camada de transporte UDP para comunicação entre postes.
   Gere o socket, a tabela de vizinhos, a descoberta periódica
   e o despacho de mensagens recebidas.

   PROTOCOLO v5.0 (texto simples, campos separados por ':'):
   ──────────────────────────────────────────────────────────
   DISCOVER:<id>:<pos>
     → Broadcast periódico para descoberta de vizinhos.
       Resposta automática: STATUS:<id>:OK

   STATUS:<id>:<estado>
     → Resposta a DISCOVER. Actualiza tabela de vizinhos.
       estados: OK | OFFLINE | FAIL | SAFE | AUTO | OBST

   TC_INC:<id>:<vel>:<x_mm>
     → Anúncio de veículo a caminho (vel > 0) ou passagem (vel < 0).
       x_mm: posição lateral para continuidade no canvas.
       Retro-compatível: x_mm omitido → usa 0 (centro).

   SPD:<id>:<vel>:<eta_ms>:<dist_m>:<x_mm>
     → Actualização de velocidade e ETA para pré-acendimento.
       Retro-compatível: x_mm omitido → usa 0.

   MASTER_CLAIM:<id>
     → Propagação de liderança ao longo da cadeia.

   MELHORIAS v4.1 → v5.0:
   ──────────────────────────────────────────────────────────
   1. udp_manager_send_passed() dedicado — sinalização explícita
      de passagem em vez de TC_INC com velocidade negativa.
   2. udp_manager_get_stats() — estatísticas de diagnóstico:
      pacotes enviados, recebidos, timeouts de vizinhos.
   3. Remoção do #include <inttypes.h> desnecessário no topo.
   4. Comentários de protocolo actualizados e completos.
   5. udp_manager_reset_neighbor() adicionado para forçar
      redescoberta de um vizinho específico.
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
    NEIGHBOR_OK        = 0,  /* Online e a responder              */
    NEIGHBOR_OFFLINE   = 1,  /* Sem resposta há NEIGHBOR_TIMEOUT  */
    NEIGHBOR_FAIL      = 2,  /* Em falha declarada                */
    NEIGHBOR_SAFE      = 3,  /* Em SAFE MODE                      */
    NEIGHBOR_AUTO      = 4,  /* Em modo AUTONOMO                  */
    NEIGHBOR_OBSTACULO = 5,  /* Com obstáculo detectado           */
} neighbor_status_t;


/* ============================================================
   ESTRUTURA DE VIZINHO
============================================================ */
typedef struct {
    char              ip[MAX_IP_LEN]; /* Endereço IP do vizinho       */
    int               id;             /* POSTE_ID do vizinho          */
    int               position;       /* POST_POSITION do vizinho     */
    neighbor_status_t status;         /* Estado de conectividade      */
    bool              active;         /* Entrada válida na tabela     */
    bool              discover_ok;    /* Respondeu ao DISCOVER        */
    uint32_t          last_seen;      /* Timestamp da última resposta */
} neighbor_t;


/* ============================================================
   ESTATÍSTICAS DE DIAGNÓSTICO
============================================================ */
typedef struct {
    uint32_t pkts_enviados;      /* Total de pacotes UDP enviados    */
    uint32_t pkts_recebidos;     /* Total de pacotes UDP recebidos   */
    uint32_t timeouts_vizinhos;  /* Total de vizinhos marcados OFFLINE*/
    uint32_t tc_inc_enviados;    /* Total de TC_INC enviados         */
    uint32_t tc_inc_recebidos;   /* Total de TC_INC recebidos        */
} udp_stats_t;


/* ============================================================
   CICLO DE VIDA
============================================================ */

/** @brief Cria socket UDP e faz bind. Chamar após WiFi com IP. */
bool udp_manager_init(void);

/** @brief Cria udp_task no Core 0 (Prio 5, Stack 4096B). */
void udp_manager_task_start(void);


/* ============================================================
   ENVIO DE MENSAGENS
============================================================ */

/** @brief Envia DISCOVER em broadcast para descoberta de vizinhos. */
void udp_manager_discover(void);

/** @brief Envia TC_INC — anúncio de veículo a caminho. */
bool udp_manager_send_tc_inc(const char *ip, float speed, int16_t x_mm);

/** @brief Envia mensagem de PASSED — veículo passou, T-- no vizinho. */
bool udp_manager_send_passed(const char *ip, float speed);

/** @brief Envia SPD — velocidade e ETA para pré-acendimento. */
bool udp_manager_send_spd(const char *ip, float speed,
                           uint32_t eta_ms, uint32_t dist_m, int16_t x_mm);

/** @brief Envia STATUS com estado actual deste poste. */
bool udp_manager_send_status(const char *ip, neighbor_status_t status);

/** @brief Envia MASTER_CLAIM para propagar liderança na cadeia. */
bool udp_manager_send_master_claim(const char *ip);


/* ============================================================
   CONSULTA DE VIZINHOS
============================================================ */

/** @brief Preenche nebL e nebR com IPs dos vizinhos esq. e dir. */
void        udp_manager_get_neighbors(char *nebL, char *nebR);

/** @brief Retorna ponteiro para vizinho com a posição dada, ou NULL. */
neighbor_t *udp_manager_get_neighbor_by_pos(int position);

/** @brief Copia todos os vizinhos activos para array externo. */
size_t      udp_manager_get_all_neighbors(neighbor_t *list, size_t max);

/** @brief Força redescoberta de um vizinho (reset last_seen). */
void        udp_manager_reset_neighbor(int position);

/** @brief Retorna estatísticas de diagnóstico  */
void        udp_manager_get_stats(udp_stats_t *out);


/* ============================================================
   CALLBACKS — implementados em state_machine.c
   ──────────────────────────────────────────────────────────
   Versões weak definidas aqui, substituídas pela state_machine.
   Chamados pelo _processar_mensagem() no contexto da udp_task.
============================================================ */
void on_tc_inc_received(float speed, int16_t x_mm);
void on_prev_passed_received(void);
void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm);
void on_master_claim_received(int from_id);


/* ============================================================
   TASK E SOCKET — ACESSO INTERNO
============================================================ */

/** @brief Função de task UDP — exposta para pinagem de core. */
void udp_task_run(void *arg);

/** @brief Retorna descritor do socket UDP (-1 se inválido). */
int udp_manager_get_socket(void);


#endif /* UDP_MANAGER_H */
