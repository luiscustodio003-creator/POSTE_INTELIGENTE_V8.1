/* ============================================================
   MÓDULO     : tracking_manager
   FICHEIRO   : tracking_manager.h — Declarações públicas
   VERSÃO     : 1.2  |  2026-05-03
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÕES v1.1 → v1.2:
   ─────────────────────────
   - ADICIONADO: event_local_pending na struct tracked_vehicle_t.
     Separa o conceito de "veículo confirmado fisicamente na zona
     local" (LOCAL → T++) de "veículo a aproximar-se" (APPROACHING
     → só ETA/pré-acendimento).

     Antes: event_approach_pending era usado para os dois fins —
       o fsm_task disparava SM_EVT_VEHICLE_LOCAL com ele, o que era
       conceptualmente errado e causava confusão.

     Depois:
       event_local_pending    → SM_EVT_VEHICLE_LOCAL    (T++, TC_INC)
       event_approach_pending → SM_EVT_VEHICLE_APPROACHING (só ETA)

     Activação no tracking_manager.c:
       event_detected_pending  : TENTATIVE → CONFIRMED
       event_approach_pending  : CONFIRMED → APPROACHING (speed_signed ≤ threshold)
       event_local_pending     : APPROACHING, distance_m ≤ RADAR_DETECT_M
       event_passed_pending    : APPROACHING/CONFIRMED → COASTING → EXITED
       event_obstaculo_pending : speed_kmh ≤ OBSTACULO_SPEED_MAX por N frames

   ALTERAÇÕES v1.0 → v1.1:
   ─────────────────────────
   - Adicionados campos de detecção de obstáculo estático:
       speed_kmh_max, obstaculo_frames, event_obstaculo_pending

   RESPONSABILIDADE:
   ─────────────────
   Camada intermédia entre radar_manager e state_machine.
   Recebe frames brutos (radar_data_t), associa alvos entre
   frames por proximidade (nearest-neighbour), atribui IDs
   estáveis, suaviza a velocidade e determina quando um veículo
   entrou, está a aproximar-se, parou ou saiu da zona.

   PIPELINE:
   ─────────
   radar_manager    → lê UART, valida frames, devolve radar_data_t
   tracking_manager → associa, filtra, gera tracked_vehicle_t[]
   fsm_task         → consome eventos, acciona FSM

   THREADING:
   ──────────
   tracking_manager_update()    — exclusivo da radar_task (Core 0)
   tracking_manager_get_vehicles() — thread-safe (mutex interno)
============================================================ */

#ifndef TRACKING_MANAGER_H
#define TRACKING_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "radar_manager.h"
#include "system_config.h"

/* ── Constantes de configuração ───────────────────────────── */

/* Raio de associação nearest-neighbour (mm).
   Cobre deslocamento de até 100 km/h entre frames de 100ms. */
#define TRK_ASSOC_RADIUS_MM     300

/* Frames consecutivos para confirmar detecção (anti-ruído) */
#define TRK_CONFIRM_FRAMES      2

/* Frames consecutivos sem alvo para declarar saída
   10 × 100ms = 1 segundo de tolerância */
#define TRK_LOST_FRAMES         10

/* Janela de média móvel para suavização de velocidade */
#define TRK_SPEED_WINDOW        5

/* Máximo de veículos rastreados simultaneamente */
#define TRK_MAX_VEHICLES        MAX_RADAR_TARGETS


/* ── Estados do ciclo de vida do veículo ──────────────────── */
typedef enum {
    TRK_STATE_TENTATIVE  = 0, /* Detectado, aguarda confirmação     */
    TRK_STATE_CONFIRMED,      /* Confirmado — ID atribuído           */
    TRK_STATE_APPROACHING,    /* Em aproximação activa               */
    TRK_STATE_COASTING,       /* Perdido temporariamente             */
    TRK_STATE_EXITED,         /* Saída confirmada                    */
} trk_state_t;


/* ── Veículo rastreado — estrutura principal de saída ──────── */
typedef struct {

    uint16_t    id;             /* ID único e estável da sessão       */
    trk_state_t state;          /* Estado actual no ciclo de vida     */

    float       x_mm;           /* Posição lateral em mm              */
    float       y_mm;           /* Posição longitudinal em mm         */
    float       distance_m;     /* Distância ao poste em metros       */
    float       speed_kmh;      /* Velocidade suavizada (km/h)        */
    float       speed_signed;   /* Negativo=aproxima, positivo=afasta */
    float       speed_kmh_max;  /* Velocidade máxima registada —
                                   distingue carro parado de obj. fixo */

    uint32_t    eta_ms;         /* ETA em ms (válido em APPROACHING)  */

    uint8_t     confirm_frames; /* Frames consecutivos confirmados    */
    uint8_t     lost_frames;    /* Frames consecutivos sem sinal      */
    uint32_t    total_frames;   /* Total de frames observados         */
    uint16_t    obstaculo_frames; /* Frames consecutivos parado na zona */

    bool        active;         /* Actualizado no último frame        */

    /* ── Flags de eventos pendentes ────────────────────────────
       Consumidas pela fsm_task a cada ciclo de 100ms.
       Limpas por tracking_manager_clear_events() após consumo.

       event_detected_pending  → SM_EVT_VEHICLE_DETECTED
         Activado: TENTATIVE → CONFIRMED (primeiro avistamento confirmado)
         FSM: prepara ETA, muda estado para LIGHT_ON. SEM T++.

       event_approach_pending  → SM_EVT_VEHICLE_APPROACHING
         Activado: CONFIRMED → APPROACHING (velocidade em direcção ao poste)
         FSM: só agenda pré-acendimento por ETA. SEM T++. SEM TC_INC.

       event_local_pending     → SM_EVT_VEHICLE_LOCAL       [NOVO v1.2]
         Activado: estado APPROACHING + distance_m ≤ RADAR_DETECT_M
         FSM: T++, Tc--, TC_INC→dir, PASSED→esq se Tc>0.
         É este o evento que representa "veículo na minha zona física".

       event_passed_pending    → SM_EVT_VEHICLE_PASSED
         Activado: COASTING → EXITED (radar perdeu o veículo)
         FSM: T-- (ou aguarda PASSED de B), SPD→dir.

       event_obstaculo_pending → SM_EVT_VEHICLE_OBSTACULO
         Activado: N frames consecutivos com vel ≤ OBSTACULO_SPEED_MAX
         FSM: STATE_OBSTACULO, luz 100%, posição fixa no display.
    ──────────────────────────────────────────────────────────── */
    bool        event_detected_pending;   /* VEHICLE_DETECTED         */
    bool        event_approach_pending;   /* VEHICLE_APPROACHING       */
    bool        event_local_pending;      /* VEHICLE_LOCAL    [v1.2]  */
    bool        event_passed_pending;     /* VEHICLE_PASSED            */
    bool        event_obstaculo_pending;  /* VEHICLE_OBSTACULO         */

} tracked_vehicle_t;


/* ── Estatísticas de diagnóstico ──────────────────────────── */
typedef struct {
    uint32_t total_vehicles_tracked;
    uint32_t frames_processed;
    uint32_t association_failures;
    uint8_t  current_active;
} trk_stats_t;


/* ── API pública ───────────────────────────────────────────── */

/** Inicializa o módulo — chamar uma vez antes de qualquer update */
void tracking_manager_init(void);

/** Processa um frame do radar — chamar exclusivamente da radar_task */
void tracking_manager_update(const radar_data_t *data);

/** Copia estado actual dos veículos — thread-safe */
bool tracking_manager_get_vehicles(tracked_vehicle_t *out,
                                   uint8_t *out_count);

/** Marca eventos de um veículo como consumidos */
void tracking_manager_clear_events(uint16_t vehicle_id);

/** Estatísticas de diagnóstico */
void tracking_manager_get_stats(trk_stats_t *out);

/** Reset completo do estado de tracking */
void tracking_manager_reset(void);

/** Nome legível de um estado */
const char *tracking_state_name(trk_state_t state);

/** Notifica sobre recepção de frame do radar (atomic) */
void tracking_manager_task_notify_frame(bool ok);

/** Retorna saúde do radar */
bool tracking_manager_get_radar_status(void);


#endif /* TRACKING_MANAGER_H */
