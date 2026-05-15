/* ============================================================
   DISPLAY MANAGER — DECLARAÇÃO
   @file      display_manager.h
   @version   5.2  |  2026-04-17
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   Configuração lv_conf.h necessária:
   ------------------------------------
     #define LV_USE_CANVAS          1   (OBRIGATÓRIO v5.0+)
     #define LV_USE_BAR             1
     #define LV_USE_LABEL           1
     #define LV_FONT_MONTSERRAT_10  1
     #define LV_FONT_MONTSERRAT_12  1
     #define LV_FONT_MONTSERRAT_14  1
     #define LV_FONT_MONTSERRAT_18  1

   Dependências directas:
   ----------------------
     radar_manager.h : radar_obj_t, RADAR_MAX_OBJ
     system_config.h : LCD_H_RES, LCD_V_RES, LIGHT_MIN
     hw_config.h     : LCD_PIN_*
     post_config.h   : post_get_name(), post_get_id()
     lvgl            : v8.3.x

   NOTA v5.2 — integração tracking_manager:
   ─────────────────────────────────────────
   display_manager_set_radar() recebe radar_obj_t com os campos:
     x_mm       → posição lateral (consumido pelo interpolador)
     y_mm       → distância frontal (consumido pelo interpolador)
     speed_kmh  → velocidade para canvas
   O display tem interpolador próprio (s_interp[]) que gera o
   rasto internamente a 50Hz com movimento preditivo.

   MUDANÇAS v5.1 → v5.2:
   ─────────────────────────
   - REMOVIDO: #include <inttypes.h> desnecessário.
   - ADICIONADO: nota sobre campos de radar_obj_t consumidos.
============================================================ */

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "radar_manager.h"


/* ============================================================
   CICLO DE VIDA
============================================================ */

/**
 * @brief Inicializa ST7789, LVGL e interface.
 *        Chamar UMA VEZ em app_main() antes de criar tasks.
 */
void display_manager_init(void);

/**
 * @brief Incrementa ticker LVGL.
 * @param ms Milissegundos decorridos (usar período nominal 20ms).
 */
void display_manager_tick(uint32_t ms);

/**
 * @brief Drena fila de mensagens + redesenha canvas radar
 *        + chama lv_timer_handler().
 *        Chamar EXCLUSIVAMENTE a partir da display_task.
 */
void display_manager_task(void);

/**
 * @brief Cria display_task — Core 0, Prio 4, Stack 8192B, 50Hz.
 *        CORRIGIDO v2.0: Core 0 (era Core 1 em conflito com fsm_task).
 */
void display_manager_task_start(void);

/* ============================================================
   ACTUALIZAÇÃO DE ESTADO
   ────────────────────────
   Todas thread-safe: escrevem na fila s_fila (non-blocking).
   Nunca chamar funções LVGL directamente fora da display_task.
============================================================ */

/**
 * @brief Actualiza badge de modo FSM.
 *        Estados: "IDLE", "LIGHT ON", "SAFE MODE", "MASTER", "AUTONOMO".
 */
void display_manager_set_status(const char *status);

/**
 * @brief Actualiza estado Wi-Fi e IP.
 * @param connected true se Wi-Fi ligado com IP.
 * @param ip        String do IP (NULL se desligado).
 */
void display_manager_set_wifi(bool connected, const char *ip);

/**
 * @brief Actualiza estado radar e brilho DALI.
 * @param radar_st   "REAL", "SIM" ou "FAIL".
 * @param radar_ok   true se radar responde.
 * @param brightness Brilho DALI em percentagem (0-100).
 */
void display_manager_set_hardware(const char *radar_st,
                                  bool        radar_ok,
                                  uint8_t     brightness);

/**
 * @brief Actualiza contadores T e Tc na zona de tráfego.
 * @param T  Veículos detectados localmente (0-999).
 * @param Tc Veículos a caminho via UDP (0-999).
 */
void display_manager_set_traffic(int T, int Tc);



/**
 * @brief Actualiza estado dos vizinhos UDP.
 * @param nebL    IP vizinho esquerdo (NULL → "---").
 * @param nebR    IP vizinho direito  (NULL → "---").
 * @param leftOk  true se esquerdo online.
 * @param rightOk true se direito online.
 */
void display_manager_set_neighbors(const char *nebL, const char *nebR,
                                   bool leftOk, bool rightOk);

/**
 * @brief Actualiza canvas radar com objectos detectados.
 *        Campos consumidos de radar_obj_t: x_mm, y_mm, speed_kmh.
 *        Campos ignorados: trail_x[], trail_y[], trail_len.
 *
 * @param objs  Array radar_obj_t (tracking_manager). Campos: x_mm, y_mm, speed_kmh.
 * @param count Número de objectos (0..RADAR_MAX_OBJ).
 */
void display_manager_set_radar(const radar_obj_t *objs, uint8_t count);
/**
 * @brief Actualiza card de velocidade km/h.
 * @param speed Velocidade em km/h (0-300).
 */
void display_manager_set_speed(int speed);



#endif /* DISPLAY_MANAGER_H */
