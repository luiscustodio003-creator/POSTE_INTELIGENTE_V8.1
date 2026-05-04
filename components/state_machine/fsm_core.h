/* ============================================================
   MÓDULO     : fsm_core
   FICHEIRO   : fsm_core.h — Declarações do núcleo da FSM
   VERSÃO     : 1.2  |  2026-05-04
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Núcleo da máquina de estados. Contém as variáveis de estado
   partilhadas entre todos os sub-módulos da FSM (fsm_events,
   fsm_network, fsm_timer) e as funções de ciclo de vida.

   ALTERAÇÕES v1.1 → v1.2:
   ─────────────────────────
   - ADICIONADO: g_fsm_tc_last_vehicle_id
     Guarda o vehicle_id do último objecto que gerou um TC_INC.
     Usado em EVT_LOCAL para garantir que cada veículo físico
     só produz um TC_INC — mesmo que o tracking_manager gere
     vários EVT_LOCAL para o mesmo ID (ex: re-entrada no raio).

     CONTEXTO DO BUG CORRIGIDO:
     O tracking_manager atribui um ID único e estável (uint16_t)
     a cada objecto detectado. Esse ID era passado até sm_process_event()
     mas descartado com (void)vehicle_id — a FSM não sabia distinguir
     se um segundo EVT_LOCAL vinha do mesmo objecto ou de um diferente.
     A solução antiga usava "if (Tc==0) Tc=1" como anti-duplicado,
     o que impedia Tc de reflectir 2 veículos simultâneos em trânsito.
     Agora a protecção usa o ID real do objecto — mais correcta e sem
     efeitos colaterais sobre o contador Tc.

   ALTERAÇÕES v1.0 → v1.1:
   ─────────────────────────
   - ADICIONADO: g_fsm_enviados_dir
     Contador de TC_INC enviados ao vizinho direito que ainda
     aguardam confirmação via PASSED. Independente de g_fsm_Tc
     (que representa veículos a caminho vindos da esquerda).

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

extern uint64_t  g_fsm_last_detect_ms;
extern uint64_t  g_fsm_left_offline_ms;
extern uint64_t  g_fsm_tc_timeout_ms;
extern bool      g_fsm_left_was_offline;
extern uint64_t  g_fsm_acender_em_ms;
extern uint64_t  g_fsm_master_claim_ms;
extern uint64_t  g_fsm_sem_vizinho_ms;
extern uint64_t  g_fsm_obstaculo_last_ms;

/* ── NOVO v1.2 ────────────────────────────────────────────────
   ID do último objecto que gerou TC_INC.

   PROBLEMA (Bug 2):
     sm_process_event() recebia o vehicle_id mas descartava-o
     com (void)vehicle_id. Sem esse ID, a FSM não distinguia
     um segundo EVT_LOCAL do mesmo veículo (retry interno do
     tracking) de um EVT_LOCAL de um veículo diferente.
     O "if (Tc==0) Tc=1" protegia contra duplicados mas
     impedia Tc=2 quando dois carros reais estavam em trânsito.

   SOLUÇÃO:
     Guardar o ID do último veículo anunciado. Em EVT_LOCAL,
     só envia TC_INC se o vehicle_id for diferente do último
     registado. Dois veículos distintos têm IDs distintos →
     Tc incrementa correctamente para cada um.
     O mesmo veículo com EVT_LOCAL duplicado → TC_INC suprimido.
──────────────────────────────────────────────────────────── */
extern uint16_t g_fsm_tc_last_vehicle_id;

/* ============================================================
   UTILITÁRIOS INTERNOS — disponíveis a todos os sub-módulos
============================================================ */

/** Retorna tempo actual em milissegundos */
uint64_t fsm_agora_ms(void);

/** Agenda apagamento após TRAFIC_TIMEOUT */
void fsm_agendar_apagar(void);

/** Monitoriza saúde do radar com debounce bidirecional */
void fsm_verificar_radar(bool teve_frame, bool comm_ok);

/** Mantém o obstáculo "vivo" enquanto o radar o detecta */
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
