/* ============================================================
   MÓDULO     : tracking_manager
   FICHEIRO   : tracking_manager.h — Declarações públicas
   VERSÃO     : 1.1  |  2026-04-20
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÕES v1.0 → v1.1:
   ─────────────────────────
   - Adicionados campos de detecção de obstáculo estático
     à struct tracked_vehicle_t:
       speed_kmh_max        — velocidade máxima registada
       obstaculo_frames     — frames consecutivos parado
       event_obstaculo_pending — flag de evento para a FSM

   RESPONSABILIDADE:
   ─────────────────
   Camada intermédia entre radar_manager e event_manager.
   Recebe frames brutos (radar_data_t), associa alvos entre
   frames por proximidade (nearest-neighbour), atribui IDs
   estáveis, suaviza a velocidade e determina quando um veículo
   entrou, está a aproximar-se, parou ou saiu da zona.

   PIPELINE:
   ─────────
   radar_manager  → lê UART, valida frames, devolve radar_data_t
   tracking_manager → associa, filtra, gera tracked_vehicle_t[]
   state_machine_task → consome eventos, acciona FSM

   THREADING:
   ──────────
   tracking_manager_update() — exclusivo da radar_task.
   tracking_manager_get_vehicles() — thread-safe (mutex interno).
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

    /* Flags de eventos pendentes — consumidas pela state_machine_task */
    bool        event_detected_pending;   /* VEHICLE_DETECTED         */
    bool        event_approach_pending;   /* VEHICLE_APPROACHING       */
    bool        event_passed_pending;     /* VEHICLE_PASSED            */
    bool        event_obstaculo_pending;  /* VEHICLE_OBSTACULO         */
    bool        event_recuou_pending;    /* veículo recuou — T=0 imediato */
    

} tracked_vehicle_t;


/* ── Estatísticas de diagnóstico ──────────────────────────── */
typedef struct {
    uint32_t total_vehicles_tracked;
    uint32_t frames_processed;
    uint32_t association_failures;
    uint8_t  current_active;
} trk_stats_t;


/* ── API pública ───────────────────────────────────────────── */

/* Inicializa o módulo — chamar uma vez antes de qualquer update */
void tracking_manager_init(void);

/* Processa um frame do radar — chamar exclusivamente da radar_task */
void tracking_manager_update(const radar_data_t *data);

/* Copia estado actual dos veículos — thread-safe */
bool tracking_manager_get_vehicles(tracked_vehicle_t *out,
                                   uint8_t *out_count);

/* Marca eventos de um veículo como consumidos */
void tracking_manager_clear_events(uint16_t vehicle_id);

/* Estatísticas de diagnóstico */
void tracking_manager_get_stats(trk_stats_t *out);

/* Reset estado  tracking */
void tracking_manager_reset(void);

/* Nome legível de um estado */
const char *tracking_state_name(trk_state_t state);


#endif /* TRACKING_MANAGER_H */