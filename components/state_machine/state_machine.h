/* ============================================================
   MÓDULO     : state_machine
   FICHEIRO   : state_machine.h — Declarações públicas
   VERSÃO     : 5.0  |  2026-04-24
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Máquina de estados finita (FSM) central do poste.
   Recebe eventos do tracking_manager e mensagens UDP,
   controla a iluminação DALI e mantém os contadores T/Tc
   matematicamente correctos.

   PIPELINE DE DADOS:
   ──────────────────
   radar_manager → tracking_manager → state_machine_task
                                            ↓
                                     sm_process_event()
                                            ↓
                                      dali_manager
                                            ↓
                                     comm_manager (UDP)

   PROTOCOLO T/Tc:
   ───────────────
   T  = veículos detectados localmente pelo radar
   Tc = veículos anunciados via UDP (a caminho)
   Luz liga quando T>0 ou Tc>0
   Luz apaga TRAFIC_TIMEOUT_MS após T=0 e Tc=0

   ESTADOS DA FSM:
   ───────────────
   IDLE      → repouso, luz no mínimo
   LIGHT_ON  → veículo detectado, luz máxima
   MASTER    → líder da cadeia (pos=0 ou viz. esq. offline)
   AUTONOMO  → sem vizinhos UDP, opera localmente
   SAFE_MODE → falha de radar E sem info UDP
   OBSTACULO → objecto parado, luz máxima fixa

   MUDANÇAS v4.0 → v5.0:
   ──────────────────────
   - Constantes TC_TIMEOUT_MS e T_STUCK_TIMEOUT_MS movidas para
     system_config.h (fonte única de verdade)
   - sm_on_radar_detect() marcado deprecated com aviso de compilação
   - Cabeçalho de dependências actualizado
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
    STATE_IDLE       = 0,  /* Repouso — luz apagada ou no mínimo  */
    STATE_LIGHT_ON,        /* Detecção activa — luz acesa          */
    STATE_SAFE_MODE,       /* Falha dupla radar+UDP — luz máxima   */
    STATE_MASTER,          /* Líder da cadeia de postes            */
    STATE_AUTONOMO,        /* UDP em falha, radar OK — modo local  */
    STATE_OBSTACULO,       /* Objecto estático — luz máxima        */
} system_state_t;


/* ============================================================
   TIPOS DE EVENTO
   ──────────────────────────────────────────────────────────
   Produzidos pelo tracking_manager e consumidos por sm_process_event.
   Transportam os dados do veículo para a FSM de forma desacoplada.
============================================================ */
typedef enum {
    SM_EVT_VEHICLE_DETECTED  = 0, /* Veículo confirmado pelo tracking   */
    SM_EVT_VEHICLE_APPROACHING,   /* Veículo em aproximação activa       */
    SM_EVT_VEHICLE_PASSED,        /* Veículo saiu da zona do sensor      */
    SM_EVT_VEHICLE_LOCAL,         /* Veículo detectado localmente        */
    SM_EVT_VEHICLE_OBSTACULO,     /* Objecto parado — activar OBSTACULO  */
} sm_event_type_t;



/* ============================================================
   CICLO DE VIDA DO MÓDULO
============================================================ */

/** @brief Inicializa a FSM. Chamar antes de state_machine_task_start(). */
void state_machine_init(void);

/**
 * @brief Ciclo de manutenção a 100ms (timers, vizinhos, MASTER).
 *        NÃO lê radar directamente — recebe estado de saúde externo.
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
   PROCESSAMENTO DE EVENTOS
   ──────────────────────────────────────────────────────────
   Ponto central de entrada event-driven da FSM.
   Chamado pela state_machine_task após desencapsular do tracking.
============================================================ */

/**
 * @brief Processa um evento do pipeline tracking → FSM.
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
   EVENTOS EXTERNOS — GESTÃO DE VIZINHOS
============================================================ */

/** @brief Chamado quando vizinho direito fica offline. */
void sm_on_right_neighbor_offline(void);

/** @brief Chamado quando vizinho direito volta online. */
void sm_on_right_neighbor_online(void);

/**
 * @brief Injeta carro de teste sem radar nem simulador.
 *        Útil para validação via JTAG/GDB em bancada.
 * @param vel Velocidade simulada em km/h
 */
void sm_inject_test_car(float vel);

/**
 * @brief Shim de compatibilidade v3.x → v4.0.
 *        Delega para sm_process_event(SM_EVT_VEHICLE_LOCAL).
 * @deprecated Usar sm_process_event() directamente.
 */
void sm_on_radar_detect(float vel);


/* ============================================================
   GETTERS — ESTADO CORRENTE
============================================================ */
system_state_t state_machine_get_state(void);      /* Estado da FSM          */
const char    *state_machine_get_state_name(void); /* Nome em texto          */
int            state_machine_get_T(void);          /* Contador T             */
int            state_machine_get_Tc(void);         /* Contador Tc            */
float          state_machine_get_last_speed(void); /* Última velocidade km/h */
bool           state_machine_radar_ok(void);       /* Saúde do radar         */
bool           sm_is_obstaculo(void);              /* true se em OBSTACULO   */


/* ============================================================
   SIMULADOR — apenas USE_RADAR == 0
   ──────────────────────────────────────────────────────────
   Funções de simulação física de veículos para testes em bancada.
   O simulador gera carros virtuais com velocidades realistas.
============================================================ */
#if USE_RADAR == 0
void sim_init_mutex(void);
bool sim_get_objeto(float *x_mm, float *y_mm);
void sim_notificar_chegada(float vel_kmh, int16_t x_mm);
void _sim_update(void);
#endif


#endif /* STATE_MACHINE_H */
