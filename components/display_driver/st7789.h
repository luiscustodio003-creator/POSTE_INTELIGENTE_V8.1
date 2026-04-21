/* ============================================================
   ST7789 — DECLARAÇÃO
   @file      st7789.h
   @brief     Driver SPI para display TFT ST7789 240x240/240x320
   @version   4.1  |  2026-04-17
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   CONFIGURAÇÃO NECESSÁRIA em lv_conf.h:
   ────────────────────────────────────────
     #define LV_COLOR_DEPTH      16
     #define LV_COLOR_16_SWAP     1   ← OBRIGATÓRIO para cores correctas

   CONFIGURAÇÃO em hw_config.h:
   ────────────────────────────────────────
     #define LCD_PIN_SDA   23   (MOSI)
     #define LCD_PIN_SCL   18   (SCLK)
     #define LCD_PIN_CS     5   (CS)
     #define LCD_PIN_DC    32   (DC)
     #define LCD_PIN_RST   33   (RST)
     #define LCD_PIN_BL    25   (Backlight)
     #define LCD_H_RES    240
     #define LCD_V_RES    240
============================================================ */

#ifndef ST7789_H
#define ST7789_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
   REGISTOS DO ST7789 (comandos principais)
============================================================ */
#define ST7789_SWRESET   0x01   /* Software Reset              */
#define ST7789_SLPIN     0x10   /* Sleep In                    */
#define ST7789_SLPOUT    0x11   /* Sleep Out                   */
#define ST7789_INVOFF    0x20   /* Display Inversion Off       */
#define ST7789_INVON     0x21   /* Display Inversion On        */
#define ST7789_DISPON    0x29   /* Display On                  */
#define ST7789_CASET     0x2A   /* Column Address Set          */
#define ST7789_RASET     0x2B   /* Row Address Set             */
#define ST7789_RAMWR     0x2C   /* Memory Write                */
#define ST7789_MADCTL    0x36   /* Memory Data Access Control  */
#define ST7789_COLMOD    0x3A   /* Interface Pixel Format      */

/* MADCTL bits */
#define ST7789_MADCTL_MY     0x80   /* Row address order    */
#define ST7789_MADCTL_MX     0x40   /* Column address order */
#define ST7789_MADCTL_MV     0x20   /* Row/Column exchange  */
#define ST7789_MADCTL_ML     0x10   /* Vertical refresh     */
#define ST7789_MADCTL_BGR    0x08   /* BGR filter (0=RGB)   */
#define ST7789_MADCTL_MH     0x04   /* Horizontal refresh   */


/* ============================================================
   API PÚBLICA
============================================================ */

/**
 * @brief Inicializa o display ST7789.
 *        Configura SPI, GPIO, sequência de init e backlight.
 *        Chamar UMA VEZ em display_manager_init().
 *
 *        Configuração aplicada:
 *          MADCTL = 0x00  (RGB, orientação normal)
 *          INVON  = activo (necessário para cores correctas)
 *          LV_COLOR_16_SWAP = 1 em lv_conf.h
 */
void st7789_init(void);

/**
 * @brief Define janela de escrita no display.
 *        Chamada internamente por st7789_draw_bitmap().
 *
 * @param x0  Coluna inicial
 * @param y0  Linha inicial
 * @param x1  Coluna final (inclusive)
 * @param y1  Linha final (inclusive)
 */
void st7789_set_window(uint16_t x0, uint16_t y0,
                       uint16_t x1, uint16_t y1);

/**
 * @brief Envia buffer de pixels para o display.
 *        Chamado pelo flush callback do LVGL (st7789_flush_cb).
 *
 * @param x     Coluna inicial
 * @param y     Linha inicial
 * @param w     Largura em pixels
 * @param h     Altura em pixels
 * @param data  Buffer RGB565 (w × h × 2 bytes)
 */
void st7789_draw_bitmap(uint16_t x, uint16_t y,
                        uint16_t w, uint16_t h,
                        const uint16_t *data);

/**
 * @brief Liga ou desliga o backlight.
 * @param on  true = backlight ligado
 */
void st7789_backlight(bool on);

#endif /* ST7789_H */