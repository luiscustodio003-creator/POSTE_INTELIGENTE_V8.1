/* radar_manager.h — v3.4 | 2026-05-14 | Poste Inteligente v8 */
#ifndef RADAR_MANAGER_H
#define RADAR_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "system_config.h"

typedef enum {
    RADAR_MODE_DEFAULT   = 0,
    RADAR_MODE_UART,
    RADAR_MODE_SIMULATED,
    RADAR_MODE_NETWORK,
} radar_mode_t;

/* Objecto para o canvas do display. */
typedef struct {
    int   x_mm;
    int   y_mm;
    float speed_kmh;
} radar_obj_t;

/* Dados de detecção por alvo — para tracking_manager. */
typedef struct {
    bool     detected;
    float    distance;       /* metros                       */
    float    speed;          /* km/h módulo (>= 0)           */
    float    speed_signed;   /* km/h com sinal (neg = aprox) */
    int      x_mm;
    int      y_mm;
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

void         radar_init(radar_mode_t mode);
bool         radar_read_data(radar_data_t *out_data, radar_simulated_input_t *sim_input);
void         radar_manager_get_last_data(radar_data_t *out);
bool         radar_is_connected(void);
void         radar_flush_rx(void);
int          radar_auto_detect_baud(void);
const char  *radar_get_status_str(void);
void         radar_diagnostic(void);
radar_mode_t radar_get_mode(void);
void         radar_manager_task_start(void);

#endif /* RADAR_MANAGER_H */
