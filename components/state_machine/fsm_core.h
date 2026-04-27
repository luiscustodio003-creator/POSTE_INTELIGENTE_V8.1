/* ============================================================
   MÓDULO     : fsm_core
   FICHEIRO   : fsm_core.h
   VERSÃO     : 2.0  |  2026-04-27
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno

   RESPONSABILIDADE:
   ─────────────────
   Variáveis de estado partilhadas entre todos os sub-módulos
   da FSM. Utilitários internos de tempo e agendamento.
   Ponto de entrada state_machine_update().

   MODOS DE OPERAÇÃO (v2.0):
   ──────────────────────────
   NORMAL   : radar OK + WiFi OK + vizinhos conhecidos
              → MASTER (P0) ou IDLE (P1..PN)
   AUTONOMO : radar OK + sem WiFi
              radar OK + WiFi OK + sem vizinhos
              radar OK + viz.dir recebeu RADAR_FAIL
   SAFE MODE: radar KO (independente de WiFi)
              → luz 50%, informa vizinhos via RADAR_FAIL

   NOVA MENSAGEM UDP (v2.0):
   ──────────────────────────
   RADAR_FAIL : enviada a viz.esq E viz.dir quando radar falha
   RADAR_OK   : enviada a viz.esq E viz.dir quando radar recupera

   DEPENDÊNCIAS:
   ─────────────
   state_machine.h — tipos públicos (system_state_t, sm_event_type_t)
   system_config.h — constantes globais
============================================================ */

#ifndef FSM_CORE_H
#define FSM_CORE_H

#include <stdbool.h>
#include <stdint.h>
#include "state_machine.h"

/* ============================================================
   CONSTANTES INTERNAS
============================================================ */

/* Ciclos sem frame radar para declarar FAIL (100ms/ciclo = 5s) */
#define RADAR_FAIL_COUNT        50

/* Frames consecutivos para declarar radar recuperado */
#define RADAR_OK_COUNT           3

/* Heartbeat MASTER_CLAIM — poste 0 reafirma liderança (ms) */
#define MASTER_CLAIM_HB_MS      30000ULL

/* Timeout de segurança Tc — veículo anunciado não chegou */
#define TC_TIMEOUT_MS           (TRAFIC_TIMEOUT_MS * 2)

/* Delay antes de entrar AUTONOMO por "sem vizinhos" (ms).
   Cobre o tempo do DISCOVER UDP inicial (~2-4s arranque). */
#ifndef AUTONOMO_DELAY_MS
#define AUTONOMO_DELAY_MS       10000ULL
#endif

/* ============================================================
   VARIÁVEIS DE ESTADO PARTILHADAS
   Definidas em fsm_core.c, declaradas extern aqui.
   Acesso exclusivo pela fsm_task, excepto callbacks UDP
   (escritas atómicas em tipos <=32bit — seguro sem mutex).
============================================================ */
extern system_state_t g_fsm_state;          /* estado actual da FSM         */
extern int            g_fsm_T;              /* veículos detectados local     */
extern int            g_fsm_Tc;             /* veículos anunciados (UDP)     */
extern float          g_fsm_last_speed;     /* última velocidade km/h        */
extern bool           g_fsm_apagar_pend;    /* fade down agendado            */
extern bool           g_fsm_radar_ok;       /* saúde do radar (debounce)     */
extern int            g_fsm_radar_fail_cnt; /* contador falhas radar         */
extern int            g_fsm_radar_ok_cnt;   /* contador recuperações radar   */
extern bool           g_fsm_right_online;   /* viz. dir. operacional         */
extern bool           g_fsm_era_autonomo;   /* era AUTONOMO antes LIGHT_ON   */

/* viz. esquerdo — notificou RADAR_FAIL */
extern bool           g_fsm_left_radar_fail;
/* viz. direito — notificou RADAR_FAIL */
extern bool           g_fsm_right_radar_fail;

extern uint64_t g_fsm_last_detect_ms;       /* último timestamp de detecção  */
extern uint64_t g_fsm_left_offline_ms;      /* quando viz.esq ficou offline  */
extern uint64_t g_fsm_tc_timeout_ms;        /* deadline Tc (segurança)       */
extern bool     g_fsm_left_was_offline;     /* viz.esq estava offline        */
extern uint64_t g_fsm_acender_em_ms;        /* timestamp pré-acendimento     */
extern uint64_t g_fsm_master_claim_ms;      /* último heartbeat MASTER_CLAIM */
extern uint64_t g_fsm_sem_vizinho_ms;       /* início do período sem viz.    */
extern uint64_t g_fsm_obstaculo_last_ms;    /* último timestamp obstáculo    */

/* ============================================================
   UTILITÁRIOS INTERNOS — disponíveis a todos os sub-módulos
============================================================ */

/** Retorna tempo actual em milissegundos desde boot */
uint64_t fsm_agora_ms(void);

/** Agenda fade down após TRAFIC_TIMEOUT_MS sem veículos */
void fsm_agendar_apagar(void);

/**
 * @brief Monitoriza saúde do radar com debounce bidirecional.
 *        FAIL após RADAR_FAIL_COUNT ciclos sem frame.
 *        RECUPERADO após RADAR_OK_COUNT frames consecutivos.
 *        Quando muda estado: envia RADAR_FAIL ou RADAR_OK
 *        a ambos os vizinhos via comm_notify_radar_status().
 */
void fsm_verificar_radar(bool teve_frame);

/* ============================================================
   CICLO DE VIDA
============================================================ */

/** Inicializa todas as variáveis. Chamar antes de task_start(). */
void state_machine_init(void);

/**
 * @brief Ciclo de manutenção chamado a 100ms pela fsm_task.
 * @param comm_ok          true se socket UDP válido
 * @param is_master        true se este poste tem papel MASTER
 * @param radar_teve_frame true se radar entregou frame neste ciclo
 */
void state_machine_update(bool comm_ok, bool is_master,
                          bool radar_teve_frame);

/** Cria fsm_task no Core 1 (prio 6, stack 6144B) */
void state_machine_task_start(void);

/* ============================================================
   GETTERS PÚBLICOS
============================================================ */
system_state_t state_machine_get_state(void);
const char    *state_machine_get_state_name(void);
int            state_machine_get_T(void);
int            state_machine_get_Tc(void);
float          state_machine_get_last_speed(void);
bool           state_machine_radar_ok(void);
bool           sm_is_obstaculo(void);

#endif /* FSM_CORE_H */
