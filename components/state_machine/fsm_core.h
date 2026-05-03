/* ============================================================
   MÓDULO     : fsm_core
   FICHEIRO   : fsm_core.h — Declarações do núcleo da FSM
   VERSÃO     : 1.1  |  2026-05-03
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Núcleo da máquina de estados. Contém as variáveis de estado
   partilhadas entre todos os sub-módulos da FSM (fsm_events,
   fsm_network, fsm_timer) e as funções de ciclo de vida.

   ALTERAÇÕES v1.0 → v1.1:
   ─────────────────────────
   - ADICIONADO: g_fsm_enviados_dir
     Contador de TC_INC enviados ao vizinho direito que ainda
     aguardam confirmação via PASSED. Independente de g_fsm_Tc
     (que representa veículos a caminho vindos da esquerda).

     Utilização:
       EVT_LOCAL: g_fsm_enviados_dir++ quando envia TC_INC a B
       EVT_PASSED: aguarda PASSED de B se g_fsm_enviados_dir > 0
       on_prev_passed_received: g_fsm_enviados_dir-- quando B confirma

   DEPENDÊNCIAS:
   ─────────────
   system_config.h — constantes de configuração
   state_machine.h — tipos públicos (system_state_t, sm_event_type_t)
============================================================ */

#ifndef FSM_CORE_H
#define FSM_CORE_H

#include <stdbool.h>
#include <stdint.h>
#include "state_machine.h"

/* ============================================================
   CONSTANTES INTERNAS
============================================================ */

#define MASTER_CLAIM_HB_MS  30000ULL

/* ============================================================
   VARIÁVEIS DE ESTADO — partilhadas entre sub-módulos
   Definidas em fsm_core.c, declaradas extern aqui.
   Acesso exclusivo pela fsm_task excepto callbacks UDP.
============================================================ */
extern system_state_t g_fsm_state;
extern int            g_fsm_T;
extern int            g_fsm_Tc;
extern int            g_fsm_enviados_dir;   /* TC_INC enviados a B aguardando PASSED */
extern float          g_fsm_last_speed;
extern bool           g_fsm_apagar_pend;
extern bool           g_fsm_radar_ok;
extern int            g_fsm_radar_fail_cnt;
extern int            g_fsm_radar_ok_cnt;
extern bool           g_fsm_right_online;
extern bool           g_fsm_era_autonomo;

extern uint64_t g_fsm_last_detect_ms;
extern uint64_t g_fsm_left_offline_ms;
extern uint64_t g_fsm_tc_timeout_ms;
extern bool     g_fsm_left_was_offline;
extern uint64_t g_fsm_acender_em_ms;
extern uint64_t g_fsm_master_claim_ms;
extern uint64_t g_fsm_sem_vizinho_ms;
extern uint64_t g_fsm_obstaculo_last_ms;

/* ============================================================
   UTILITÁRIOS INTERNOS — disponíveis a todos os sub-módulos
============================================================ */

/** Retorna tempo actual em milissegundos */
uint64_t fsm_agora_ms(void);

/** Agenda apagamento após TRAFIC_TIMEOUT */
void fsm_agendar_apagar(void);

/** Monitoriza saúde do radar com debounce bidirecional */
void fsm_verificar_radar(bool teve_frame, bool comm_ok);

/**
 * @brief Mantém o obstáculo "vivo" enquanto o radar o continua a detectar.
 *        Actualiza g_fsm_obstaculo_last_ms para que OBSTACULO_REMOVE_MS
 *        só comece a contar quando o obstáculo desaparece do radar.
 *        Inofensiva se chamada fora de STATE_OBSTACULO.
 */
void fsm_obstaculo_keepalive(void);

/* ============================================================
   CICLO DE VIDA
============================================================ */

/** Inicializa todas as variáveis da FSM */
void state_machine_init(void);

/** Ciclo de manutenção a 100ms */
void state_machine_update(bool comm_ok, bool is_master,
                          bool radar_teve_frame);

/** Cria fsm_task no Core 1 */
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
