/* ============================================================
   RADAR MANAGER — DECLARAÇÃO
   @file      radar_manager.h
   @version   3.3  |  2026-04-17
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   MUDANÇAS v3.2 → v3.3:
   ─────────────────────────
   - REMOVIDO: #include <inttypes.h> desnecessário.
     Consistente com radar_manager.c v3.3, state_machine v3.1+
     e display_manager v5.2.
   - SEM OUTRAS ALTERAÇÕES: toda a API mantém-se igual.
============================================================ */

#ifndef RADAR_MANAGER_H
#define RADAR_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "system_config.h"

/* ── Tipos ────────────────────────────────────────────────── */
typedef enum {
    RADAR_MODE_DEFAULT   = 0,
    RADAR_MODE_UART,
    RADAR_MODE_SIMULATED,
    RADAR_MODE_NETWORK,
} radar_mode_t;

/* Objecto para o canvas do display (com rasto).
 * NOTA v3.3: campos trail_x/trail_y/trail_len são ignorados
 * pelo display_manager v5.2+ que usa interpolador próprio.
 * Apenas x_mm, y_mm e speed_kmh são consumidos pelo display. */
typedef struct {
    int     x_mm;
    int     y_mm;
    int     trail_x[RADAR_TRAIL_MAX];
    int     trail_y[RADAR_TRAIL_MAX];
    uint8_t trail_len;
    float   speed_kmh;
} radar_obj_t;

/* Dados detalhados de detecção — para tracking_manager e FSM */
typedef struct {
    bool     detected;
    float    distance;           /* metros                        */
    float    speed;              /* km/h módulo (>= 0)            */
    float    speed_signed;       /* km/h com sinal (neg = aprox)  */
    int      x_mm;               /* posição lateral em mm         */
    int      y_mm;               /* distância frontal em mm       */
    uint16_t frames_estaticos;   /* frames consecutivos parado    */
    int      dist_mm_anterior;   /* distância no frame anterior   */
} radar_vehicle_t;

typedef struct {
    int             count;
    radar_vehicle_t targets[MAX_RADAR_TARGETS];
} radar_data_t;

/* Input de simulação (USE_RADAR=0) */
typedef struct {
    bool    active;
    int16_t distance;
} radar_simulated_input_t;

/* ── API Pública ──────────────────────────────────────────── */

/** Inicializa UART ou modo simulado */
void radar_init(radar_mode_t mode);

/** Lê frame da UART e actualiza cache interna.
 *  Chamado pela radar_task a cada 100ms. */
bool radar_read_data(radar_data_t *out_data,
                     radar_simulated_input_t *sim_input);

/** Lê da cache interna sem tocar a UART.
 *  Thread-safe via spinlock interno. */
bool radar_read_data_cached(radar_data_t *out_data);

/** Exporta o último frame completo (thread-safe). */
void radar_manager_get_last_data(radar_data_t *out);

/** Objectos com rasto para canvas do display.
 *  Mantido por compatibilidade — na v4.0 o display usa
 *  tracking_manager_get_vehicles() via _atualiza_radar_display(). */
uint8_t radar_manager_get_objects(radar_obj_t *objs, uint8_t max);

/** true se a receber frames válidos do sensor */
bool radar_is_connected(void);

/** Limpa backlog UART acumulado no arranque */
void radar_flush_rx(void);

/** Auto-detecção de baud rate do HLK-LD2450 */
int radar_auto_detect_baud(void);

/** "REAL" | "SIM" | "FAIL" */
const char *radar_get_status_str(void);

/** Janela de diagnóstico de 8s */
void radar_diagnostic(void);

/** Modo actual do radar */
radar_mode_t radar_get_mode(void);

/** true se há alvo a aproximar-se (speed_signed <= AFASTAR_THRESHOLD_KMH) */
bool radar_vehicle_in_range(const radar_data_t *data);

/** Velocidade módulo do alvo mais próximo a aproximar-se */
float radar_get_closest_speed(const radar_data_t *data);

/** true se alvo estático persistente (>= OBSTACULO_MIN_FRAMES) */
bool radar_static_object_present(const radar_data_t *data);

/** Cria radar_task — Core 0, Prio 5, Stack 4096B */
void radar_manager_task_start(void);

#endif /* RADAR_MANAGER_H */
