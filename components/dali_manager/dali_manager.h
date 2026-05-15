/* ============================================================
   DALI MANAGER — DECLARAÇÃO v3.1 CORRIGIDA
   @file      dali_manager.h
   @version   3.2  |  2026-05-14
   Projecto  : Poste Inteligente v8
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

  
   - REMOVIDAS: dali_turn_on(), dali_turn_off() — nunca chamadas.
   - REMOVIDAS: dali_fade_stop(), dali_get_brightness_real() — nunca chamadas.
   - REMOVIDA: dali_test_curve() — ferramenta de diagnóstico sem chamadores.

   
   - CORRIGIDO: Curva logarítmica em _pct_to_duty() — LUT IEC 62386.
============================================================ */
#ifndef DALI_MANAGER_H
#define DALI_MANAGER_H

#include <stdint.h>

/* ══════════════════════════════════════════════════════════
   INICIALIZAÇÃO
   ══════════════════════════════════════════════════════════ */

/**
 * @brief Inicializa timer LEDC, canal PWM e serviço de fade.
 *        Configura automaticamente para LIGHT_MIN.
 *        Chamar UMA VEZ do system_monitor.
 */
void dali_init(void);


/* ══════════════════════════════════════════════════════════
   CONTROLO INSTANTÂNEO (sem fade)
   ══════════════════════════════════════════════════════════ */

/**
 * @brief Define brilho instantaneamente (fade 1ms).
 * @param brightness Percentagem 0-100 (limitada a LIGHT_MIN-LIGHT_MAX)
 */
void dali_set_brightness(uint8_t brightness);

/**
 * @brief Modo de segurança - brilho fixo a 50%.
 *        Usado quando radar está em falha.
 */
void dali_safe_mode(void);


/* ══════════════════════════════════════════════════════════
   FADE GRADUAL (IEC 62386)
   ══════════════════════════════════════════════════════════ */

/**
 * @brief Fade up para LIGHT_MAX com tempo adaptado à velocidade.
 * @param vel_kmh Velocidade do veículo detectado (km/h)
 * 
 * Tempos de fade (definidos em system_config.h):
 *   MODO_LABORATORIO=1:
 *     ≥3 km/h → 300ms | ≥2 km/h → 500ms | ≥1 km/h → 800ms
 *   MODO_LABORATORIO=0:
 *     ≥80 km/h → 300ms | ≥50 km/h → 500ms | ≥30 km/h → 800ms
 */
void dali_fade_up(float vel_kmh);

/**
 * @brief Fade down para LIGHT_MIN em 4 segundos.
 *        Usado quando veículo sai da zona de detecção.
 */
void dali_fade_down(void);

/* ══════════════════════════════════════════════════════════
   LEITURA DE ESTADO (thread-safe)
   ══════════════════════════════════════════════════════════ */

/**
 * @brief Valor lógico de brilho (destino do fade).
 * @return Percentagem 0-100. Thread-safe via spinlock.
 */
uint8_t dali_get_brightness(void);


#endif /* DALI_MANAGER_H */
