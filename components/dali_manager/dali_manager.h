/* ============================================================
   DALI MANAGER — DECLARAÇÃO v3.1 CORRIGIDA
   @file      dali_manager.h
   @version   3.1  |  2026-05-08
   Projecto  : Poste Inteligente v8
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

   Alterações v3.0 → v3.1:
   ────────────────────────
   - ADICIONADO: dali_test_curve() para validação da curva IEC 62386
   - CORRIGIDO: Curva logarítmica em _pct_to_duty() (privada, não visível aqui)
   
   COMPATIBILIDADE:
   ─────────────────
   ✅ 100% compatível com v3.0 - todas as interfaces públicas mantêm-se
   ✅ Sem alterações em módulos dependentes (fsm_task, state_machine)
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
 * @brief Acende luz ao máximo (LIGHT_MAX).
 *        Atalho para dali_set_brightness(LIGHT_MAX).
 */
void dali_turn_on(void);

/**
 * @brief Apaga luz ao mínimo (LIGHT_MIN).
 *        Atalho para dali_set_brightness(LIGHT_MIN).
 */
void dali_turn_off(void);

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

/**
 * @brief Para fade imediatamente no nível actual.
 *        Congela o brilho onde está durante transição.
 */
void dali_fade_stop(void);


/* ══════════════════════════════════════════════════════════
   LEITURA DE ESTADO (thread-safe)
   ══════════════════════════════════════════════════════════ */

/**
 * @brief Valor lógico de brilho (destino do fade).
 * @return Percentagem 0-100
 * 
 * NOTA: Durante um fade, retorna o valor de DESTINO, não o actual.
 *       Para brilho instantâneo real, usar dali_get_brightness_real().
 * 
 * Thread-safe via spinlock.
 */
uint8_t dali_get_brightness(void);

/**
 * @brief Valor real do hardware LEDC durante o fade.
 * @return Percentagem 0-100 (brilho instantâneo)
 * 
 * Reflecte o brilho actual do LED enquanto fade corre.
 * Usar no system_monitor para barra de progresso no display.
 * 
 * Exemplo durante fade up (500ms):
 *   t=0ms    → 10%  (início)
 *   t=250ms  → 55%  (meio)
 *   t=500ms  → 100% (fim)
 * 
 * Thread-safe - lê directo do periférico LEDC.
 */
uint8_t dali_get_brightness_real(void);


/* ══════════════════════════════════════════════════════════
   DIAGNÓSTICO E TESTE (v3.1)
   ══════════════════════════════════════════════════════════ */

/**
 * @brief Teste completo da curva DALI IEC 62386.
 *        Imprime tabela de validação no log.
 * 
 * Uso:
 *   Chamar de app_main() ou via GDB para diagnóstico.
 *   Valida que 10% → ~9% real, 50% → ~45% real, etc.
 * 
 * Output esperado:
 *   ═══ TESTE CURVA DALI IEC 62386 v3.1 ═══
 *   pct= 10% → duty= 23 → real=9.1%
 *   pct= 50% → duty=114 → real=44.9%
 *   pct=100% → duty=254 → real=100.0%
 *   ════════════════════════════════════════
 */
void dali_test_curve(void);


#endif /* DALI_MANAGER_H */
