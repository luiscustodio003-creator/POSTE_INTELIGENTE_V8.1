/* ============================================================
   MÁQUINA DE ESTADOS — DECLARAÇÃO PÚBLICA
   @file      state_machine.h
   @version   5.0  |  2026-04-29
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Interface pública da máquina de estados do Poste Inteligente.
   Expõe apenas o necessário para os módulos externos:
     - Ciclo de vida (init, update, task_start)
     - Processamento de eventos do tracking_manager
     - Callbacks UDP recebidos pelo udp_manager
     - Getters de estado para display e system_monitor
     - Gestão de vizinhos online/offline

   ARQUITECTURA INTERNA (privada, não exposta aqui):
   ──────────────────────────────────────────────────
     fsm_core.c    — variáveis de estado e getters
     fsm_events.c  — eventos do tracking e callbacks UDP
     fsm_network.c — lógica de vizinhos, MASTER, SAFE_MODE, AUTONOMO
     fsm_timer.c   — gestão de timeouts e agendamentos
     fsm_task.c    — loop principal a 100ms (Core 1, Prio 6)

   PIPELINE DE DADOS (radar → FSM):
   ──────────────────────────────────
     radar_task (Core 0, 100ms)
         └─→ tracking_manager_update()
         └─→ tracking_manager_task_notify_frame()   [atomic_bool]
     fsm_task (Core 1, 100ms)
         └─→ state_machine_update()
         └─→ sm_process_event()  [por cada evento pendente]

   PROTOCOLO T/Tc — REGRAS FUNDAMENTAIS:
   ───────────────────────────────────────
     T  = veículos detectados pelo radar local
     Tc = veículos anunciados via UDP (a caminho)
     1. TC_INC enviado UMA VEZ por veículo — em EVT_LOCAL
     2. on_tc_inc_received() só faz Tc++ — nunca reenvia
     3. EVT_APPROACHING não envia TC_INC — só agenda ETA local
     4. EVT_PASSED envia SPD — nunca TC_INC
     5. Luz apaga só quando T==0 AND Tc==0 AND timeout expirou

   NOTA — MODO SIMULAÇÃO REMOVIDO:
   ─────────────────────────────────
     Esta versão (v5.0) não tem simulador.
     O sistema opera exclusivamente com o radar HLK-LD2450 real.
     A flag USE_RADAR deixou de existir neste módulo.
============================================================ */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>
#include "system_config.h"


/* ============================================================
   ESTADOS DA MÁQUINA DE ESTADOS
   ─────────────────────────────
   IDLE       — Repouso. Sem veículos. Luz apagada ou em mínimo.
   LIGHT_ON   — Veículo detectado. Luz acesa.
   SAFE_MODE  — Radar em falha. Luz a 50% por segurança.
   MASTER     — Líder da cadeia. Primeiro poste activo à esquerda.
   AUTONOMO   — Sem WiFi mas radar OK. Opera localmente.
   OBSTACULO  — Objecto parado detectado. Luz a 100%.
============================================================ */
typedef enum {
    STATE_IDLE       = 0,
    STATE_LIGHT_ON,
    STATE_SAFE_MODE,
    STATE_MASTER,
    STATE_AUTONOMO,
    STATE_OBSTACULO,
} system_state_t;


/* ============================================================
   TIPOS DE EVENTO
   ───────────────
   Gerados pelo tracking_manager e consumidos pela fsm_task.
   Cada evento corresponde a uma transição de estado do veículo.
============================================================ */
typedef enum {
    SM_EVT_VEHICLE_DETECTED   = 0, /* Primeiro avistamento — prepara ETA    */
    SM_EVT_VEHICLE_APPROACHING,    /* Veículo a aproximar-se — pré-acende   */
    SM_EVT_VEHICLE_PASSED,         /* Veículo saiu do radar — T--, SPD→dir  */
    SM_EVT_VEHICLE_LOCAL,          /* Veículo confirmado local — T++, propaga */
    SM_EVT_VEHICLE_OBSTACULO,      /* Veículo parado — STATE_OBSTACULO      */
} sm_event_type_t;


/* ============================================================
   CICLO DE VIDA
============================================================ */

/**
 * @brief Inicializa todas as variáveis da FSM.
 *        Chamar UMA VEZ em system_monitor_start() antes das tasks.
 */
void state_machine_init(void);

/**
 * @brief Ciclo de manutenção da FSM — chamado a cada 100ms pela fsm_task.
 *
 * @param comm_ok         true se WiFi/UDP operacional
 * @param is_master       true se este poste é o líder da cadeia
 * @param radar_teve_frame true se a radar_task recebeu frame válido no último ciclo
 */
void state_machine_update(bool comm_ok, bool is_master, bool radar_teve_frame);

/**
 * @brief Cria a fsm_task — Core 1, Prio 6, Stack 6144B.
 *        Chamar em system_monitor_start() após state_machine_init().
 */
void state_machine_task_start(void);


/* ============================================================
   PROCESSAMENTO DE EVENTOS
   ─────────────────────────
   Ponto central de entrada de eventos do pipeline tracking→FSM.
   Chamado pela fsm_task para cada evento pendente no tracking_manager.
============================================================ */

/**
 * @brief Processa um evento do tracking_manager.
 *
 * @param type       Tipo de evento (SM_EVT_*)
 * @param vehicle_id ID único do veículo (tracking_manager)
 * @param vel        Velocidade em km/h
 * @param eta_ms     ETA em milissegundos (0 se não disponível)
 * @param x_mm       Posição lateral em mm
 */
void sm_process_event(sm_event_type_t type,
                      uint16_t vehicle_id,
                      float vel,
                      uint32_t eta_ms,
                      int16_t x_mm);


/* ============================================================
   GESTÃO DE VIZINHOS
   ───────────────────
   Chamadas pelo comm_manager quando o estado de um vizinho muda.
============================================================ */

/** @brief Notifica que o vizinho direito ficou offline. */
void sm_on_right_neighbor_offline(void);

/** @brief Notifica que o vizinho direito voltou online. */
void sm_on_right_neighbor_online(void);


/* ============================================================
   CALLBACKS UDP
   ──────────────
   Implementados em fsm_events.c.
   Chamados pelo udp_manager no contexto da udp_task (Core 0).
   Versões __weak definidas no udp_manager, substituídas pela FSM.
============================================================ */

/** @brief Recebeu TC_INC do poste esquerdo → Tc++ */
void on_tc_inc_received(float speed, int16_t x_mm);

/** @brief Recebeu PASSED do poste direito → T-- */
void on_prev_passed_received(float speed);

/** @brief Recebeu SPD → actualiza ETA de pré-acendimento */
void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm);

/** @brief Recebeu MASTER_CLAIM → regista e responde se necessário */
void on_master_claim_received(int from_id);


/* ============================================================
   GETTERS DE ESTADO
   ──────────────────
   Thread-safe: leitura de variáveis globais da fsm_core.
   Chamados pelo display_manager, system_monitor e comm_manager.
============================================================ */

/** @brief Estado actual da FSM */
system_state_t state_machine_get_state(void);

/** @brief Nome legível do estado actual ("IDLE", "LIGHT_ON", ...) */
const char    *state_machine_get_state_name(void);

/** @brief Contador T — veículos detectados localmente */
int            state_machine_get_T(void);

/** @brief Contador Tc — veículos anunciados via UDP */
int            state_machine_get_Tc(void);

/** @brief Última velocidade registada em km/h */
float          state_machine_get_last_speed(void);

/** @brief true se o radar está a enviar frames válidos */
bool           state_machine_radar_ok(void);

/** @brief true se o estado actual é STATE_OBSTACULO */
bool           sm_is_obstaculo(void);


#endif /* STATE_MACHINE_H */
