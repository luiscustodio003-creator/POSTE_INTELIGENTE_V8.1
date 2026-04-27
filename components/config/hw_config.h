/* ============================================================
   HARDWARE CONFIG — PINOS E PERIFÉRICOS v6.2
   ------------------------------------------------------------
   @file      hw_config.h
   @version   6.2  |  2026-04-27
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno

   AUTO-DETECÇÃO DE RESOLUÇÃO (v6.2):
   ────────────────────────────────────
   LCD_V_RES deixou de ser um #define fixo.
   É agora uma variável global uint16_t g_lcd_v_res definida
   em st7789.c e lida em runtime da NVS (namespace "hw_cfg",
   chave "lcd_v_res").

   Valor por omissão: 240 (ecrã 240×240).
   Para configurar num poste com ecrã 240×320, chamar UMA VEZ:
     st7789_set_resolution(320);
   O valor fica gravado na flash e é usado em todos os arranques
   seguintes sem necessidade de nova configuração.

   TODOS OS MÓDULOS que usam LCD_V_RES (display_manager.c,
   display_manager.h, etc.) continuam a usar o macro — nenhuma
   alteração necessária nesses ficheiros. O macro resolve agora
   para a variável global em vez de uma constante de compilação.

   Dependências:
   -------------
   driver/ledc.h  — LEDC_HIGH/LOW_SPEED_MODE
   driver/uart.h  — UART_NUM_2
============================================================ */

#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include <stdint.h>
#include "driver/ledc.h"

/* ============================================================
   DISPLAY TFT ST7789 — SPI
   LCD_H_RES: fixo (sempre 240 — a largura não varia).
   LCD_V_RES: variável global — detectada em runtime via NVS.
============================================================ */
#define LCD_PIN_SCL     18
#define LCD_PIN_SDA     23
#define LCD_PIN_RST     33
#define LCD_PIN_DC      32
#define LCD_PIN_CS       5
#define LCD_PIN_BL      25

#define LCD_H_RES       240     /* Resolução horizontal — fixa em todos os modelos */

/* Variável global definida em st7789.c, inicializada em st7789_init().
   Valor: 240 (ecrã quadrado) ou 320 (ecrã rectangular).
   Lida da NVS — persistente entre arranques. */
extern uint16_t g_lcd_v_res;

/* Macro compatível com todo o código existente.
   Resolve para a variável global em vez de constante. */
#define LCD_V_RES       g_lcd_v_res

/* ============================================================
   RADAR HLK-LD2450 — UART2
============================================================ */
#define RADAR_UART_PORT     UART_NUM_2
#define RADAR_PIN_TX        27
#define RADAR_PIN_RX        22
#define RADAR_BAUD_RATE     256000

/* ============================================================
   LUMINÁRIA — PWM (LEDC / DALI analógico)
============================================================ */
#define LED_PWM_PIN         26
#define LED_PWM_FREQ_HZ     5000

#if CONFIG_IDF_TARGET_ESP32
    #define DALI_LEDC_MODE  LEDC_HIGH_SPEED_MODE
#else
    #define DALI_LEDC_MODE  LEDC_LOW_SPEED_MODE
#endif

#endif /* HW_CONFIG_H */
