/* ============================================================
   MÓDULO     : state_machine
   FICHEIRO   : state_machine.h — Declarações públicas
   VERSÃO     : 4.0  |  2026-04-17
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   MUDANÇAS v3.1 → v4.0:
   ─────────────────────────
   1. sm_event_type_t — enumeração dos tipos de evento da FSM.
   2. sm_process_event() — novo ponto de entrada event-driven.
      Substitui a leitura directa do radar pela FSM.
   3. state_machine_update() — assinatura alterada: recebe
      radar_teve_frame como parâmetro (em vez de ler o radar).
   4. sm_on_radar_detect() mantido como shim de compatibilidade.
      Será removido na v5.0 após integração do event_manager.
============================================================ */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>
#include "system_config.h"


/* ============================================================
   ESTADOS DA FSM
============================================================ */
typedef enum {
    STATE_IDLE       = 0,  /* Repouso — luz apagada ou em mínimo   */
    STATE_LIGHT_ON,        /* Detecção activa — luz acesa           */
    STATE_SAFE_MODE,       /* Falha dupla radar+UDP — luz máxima    */
    STATE_MASTER,          /* Líder da cadeia de postes             */
    STATE_AUTONOMO,        /* UDP em falha, radar OK — modo local   */
    STATE_OBSTACULO,       /* Objecto estático — luz máxima         */
} system_state_t;


/* ============================================================
   TIPOS DE EVENTO  ← NOVO v4.0
   ──────────────────────────────
   Produzidos pelo event_manager e consumidos por sm_process_event.
   Transportam os dados do veículo do tracking_manager para a FSM.
============================================================ */
typedef enum {
    SM_EVT_VEHICLE_DETECTED = 0,
    SM_EVT_VEHICLE_APPROACHING,
    SM_EVT_VEHICLE_PASSED,       /* saiu pela frente → aguarda B */
    SM_EVT_VEHICLE_RECUOU,       /* saiu por trás → T=0 imediato */
    SM_EVT_VEHICLE_LOCAL,
    SM_EVT_VEHICLE_OBSTACULO,   /* Objecto parado na zona — activar STATE_OBSTACULO */
} sm_event_type_t;


/* ============================================================
   CICLO DE VIDA DO MÓDULO
============================================================ */

/** @brief Inicializa a FSM. Chamar antes de state_machine_task_start(). */
void state_machine_init(void);

/**
 * @brief Ciclo de manutenção a 100ms (timers, vizinhos, MASTER).
 *        v4.0: NÃO lê radar directamente.
 *
 * @param comm_ok          true se UDP operacional.
 * @param is_master        true se este poste tem papel de MASTER.
 * @param radar_teve_frame true se o radar entregou frame válido
 *                         neste ciclo (fornecido pela radar_task).
 */
void state_machine_update(bool comm_ok, bool is_master,
                          bool radar_teve_frame);

/** @brief Cria fsm_task no Core 1 (prio 6, stack 6144B). */
void state_machine_task_start(void);


/* ============================================================
   PROCESSAMENTO DE EVENTOS  ← NOVO v4.0
============================================================ */

/**
 * @brief Processa um evento do pipeline tracking → event_manager.
 *        Ponto central de entrada event-driven da FSM.
 *        Chamado pelo event_manager após desencapsular da queue.
 *
 * @param type       Tipo de evento (SM_EVT_*)
 * @param vehicle_id ID estável do veículo (tracking_manager)
 * @param vel        Velocidade suavizada em km/h
 * @param eta_ms     ETA calculado (ms), 0 se não disponível
 * @param x_mm       Posição lateral em mm
 */
void sm_process_event(sm_event_type_t type,
                      uint16_t vehicle_id,
                      float vel,
                      uint32_t eta_ms,
                      int16_t x_mm);


/* ============================================================
   EVENTOS EXTERNOS (mantidos por compatibilidade)
============================================================ */

/**
 * @brief Shim de compatibilidade v3.x → v4.0.
 *        Delega para sm_process_event(SM_EVT_VEHICLE_LOCAL).
 *        Será removido na v5.0.
 * @deprecated Usar sm_process_event() directamente.
 */
void sm_on_radar_detect(float vel);

void sm_on_right_neighbor_offline(void);
void sm_on_right_neighbor_online(void);

/**
 * @brief Injeta carro de teste sem radar nem simulador.
 *        Delega para sm_process_event(SM_EVT_VEHICLE_LOCAL).
 */
void sm_inject_test_car(float vel);


/* ============================================================
   GETTERS
============================================================ */
system_state_t state_machine_get_state(void);
const char    *state_machine_get_state_name(void);
int            state_machine_get_T(void);
int            state_machine_get_Tc(void);
float          state_machine_get_last_speed(void);
bool           state_machine_radar_ok(void);
bool           sm_is_obstaculo(void);


/* ============================================================
   SIMULADOR — apenas USE_RADAR == 0
============================================================ */
#if USE_RADAR == 0
void sim_init_mutex(void);
bool sim_get_objeto(float *x_mm, float *y_mm);
void sim_notificar_chegada(float vel_kmh, int16_t x_mm);
void _sim_update(void);
#endif


#endif /* STATE_MACHINE_H */
