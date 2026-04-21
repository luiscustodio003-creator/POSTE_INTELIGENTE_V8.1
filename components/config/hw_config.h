#include <inttypes.h>
/* ============================================================
   HARDWARE CONFIG — PINOS E PERIFÉRICOS v6.0
   ------------------------------------------------------------
   @file      hw_config.h
   @brief     Mapeamento de pinos GPIO e configuração de periféricos.
              FONTE ÚNICA DE VERDADE para LCD_H_RES e LCD_V_RES.
   

   Projecto  : Poste Inteligente v6
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

  

   Dependências:
   -------------
   - driver/ledc.h  : para LEDC_HIGH/LOW_SPEED_MODE
   - driver/uart.h  : para UART_NUM_2
============================================================ */

#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include "driver/ledc.h"   /* Necessário para selecção de DALI_LEDC_MODE */

/* ============================================================
   DISPLAY TFT ST7789 — SPI
   LCD_H_RES e LCD_V_RES definidos APENAS aqui.
   Não definir em system_config.h — fonte única de verdade.
============================================================ */

#define LCD_PIN_SCL     18      /* SCLK — relógio SPI                */
#define LCD_PIN_SDA     23      /* MOSI — dados SPI                  */
#define LCD_PIN_RST     33      /* Reset (activo baixo)              */
#define LCD_PIN_DC      32      /* Data/Command — alto=dados         */
#define LCD_PIN_CS       5      /* Chip Select (activo baixo)        */
#define LCD_PIN_BL      25      /* Backlight (activo alto)           */

#define LCD_H_RES       240     /* Resolução horizontal (pixels)     */
#define LCD_V_RES       240     /* Resolução vertical   (pixels)     */

/* ============================================================
   RADAR HLK-LD2450 — UART2
   NOTA: GPIO22 é o SCL do I2C por convenção no ESP32.
   Este projecto não usa I2C — se necessário no futuro,
   mover RADAR_PIN_RX para outro GPIO disponível (ex: GPIO34).
============================================================ */
#define RADAR_UART_PORT     UART_NUM_2  /* UART2 do ESP32             */
#define RADAR_PIN_TX        27          /* TX ESP32 → RX sensor       */
#define RADAR_PIN_RX        22          /* RX ESP32 ← TX sensor       */
#define RADAR_BAUD_RATE     256000      /* Baud rate default HLK-LD2450 */

/* ============================================================
   LUMINÁRIA — PWM (LEDC / DALI analógico)
============================================================ */
#define LED_PWM_PIN         26          /* Saída PWM para o driver    */

/* Frequência PWM: 5 kHz — acima do limiar auditivo (20 Hz–20 kHz)
   e abaixo da cintilação perceptível. Recomendado Espressif p/ 8-bit. */
#define LED_PWM_FREQ_HZ     5000

/* Selecção automática do modo LEDC por chip alvo:
   - ESP32 (LX6 Xtensa): HIGH_SPEED_MODE com fade por hardware
   - ESP32-S2/S3/C3    : apenas LOW_SPEED_MODE disponível        */
#if CONFIG_IDF_TARGET_ESP32
    #define DALI_LEDC_MODE  LEDC_HIGH_SPEED_MODE
#else
    #define DALI_LEDC_MODE  LEDC_LOW_SPEED_MODE
#endif

#endif /* HW_CONFIG_H */

