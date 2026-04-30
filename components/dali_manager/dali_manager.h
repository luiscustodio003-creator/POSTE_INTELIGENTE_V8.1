/* ============================================================
   DALI MANAGER — DECLARACAO v3.0
   @file      dali_manager.h
   @version   3.0  |  2026-04-29
   Projecto  : Poste Inteligente v8
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

   Alterações v2.0 → v3.0:
   - #include <inttypes.h> movido para dentro do guard
   - ADICIONADO: dali_get_brightness_real()
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

/**
 * @brief Valor lógico de brilho (destino do fade).
 *        Pode ser optimista durante o fade.
 *        Thread-safe via spinlock.
 */
uint8_t dali_get_brightness(void);

/**
 * @brief Valor real do hardware LEDC durante o fade.
 *        Reflecte o brilho instantâneo enquanto o fade corre.
 *        Usar no system_monitor para a barra no display.
 */
uint8_t dali_get_brightness_real(void);

#endif /* DALI_MANAGER_H */
