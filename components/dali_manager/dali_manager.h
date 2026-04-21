#include <inttypes.h>
/* ============================================================
   DALI MANAGER — DECLARACAO v2.0
   Projecto  : Poste Inteligente v8
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)
============================================================ */
#ifndef DALI_MANAGER_H
#define DALI_MANAGER_H

#include <stdint.h>

/* Inicializa timer LEDC, canal PWM e servico de fade */
void    dali_init(void);

/* Controlo instantaneo (sem fade) */
void    dali_set_brightness(uint8_t brightness);
void    dali_turn_on(void);
void    dali_turn_off(void);
void    dali_safe_mode(void);

/* Fade gradual IEC 62386 */
void    dali_fade_up(float vel_kmh);
void    dali_fade_down(void);
void    dali_fade_stop(void);

/* Leitura de estado (thread-safe) */
uint8_t dali_get_brightness(void);

#endif /* DALI_MANAGER_H */
