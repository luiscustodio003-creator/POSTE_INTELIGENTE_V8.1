/* ============================================================
   ST7789 — DECLARAÇÃO
   @file      st7789.h
   @version   5.0  |  2026-04-27
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno

   NOVIDADE v5.0 — AUTO-DETECÇÃO DE RESOLUÇÃO:
   ─────────────────────────────────────────────
   st7789_init() lê LCD_V_RES da NVS automaticamente.
   Se não encontrar valor gravado, usa 240px por omissão.

   Para configurar um poste com ecrã 240×320:
     st7789_set_resolution(320);   ← chamar UMA VEZ
   O valor fica gravado na NVS (namespace "hw_cfg", chave
   "lcd_v_res") e é restaurado em todos os arranques seguintes.

   Para repor o valor por omissão (240px):
     st7789_set_resolution(240);

   A resolução actual pode ser consultada com:
     st7789_get_resolution();  → devolve 240 ou 320

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
     #define LCD_H_RES    240   (fixo)
     LCD_V_RES             →   variável global g_lcd_v_res (NVS)
============================================================ */

#ifndef ST7789_H
#define ST7789_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
   REGISTOS DO ST7789
============================================================ */
#define ST7789_SWRESET   0x01
#define ST7789_SLPIN     0x10
#define ST7789_SLPOUT    0x11
#define ST7789_INVOFF    0x20
#define ST7789_INVON     0x21
#define ST7789_DISPON    0x29
#define ST7789_CASET     0x2A
#define ST7789_RASET     0x2B
#define ST7789_RAMWR     0x2C
#define ST7789_MADCTL    0x36
#define ST7789_COLMOD    0x3A

#define ST7789_MADCTL_MY     0x80
#define ST7789_MADCTL_MX     0x40
#define ST7789_MADCTL_MV     0x20
#define ST7789_MADCTL_ML     0x10
#define ST7789_MADCTL_BGR    0x08
#define ST7789_MADCTL_MH     0x04


/* ============================================================
   API PÚBLICA
============================================================ */

/**
 * @brief Inicializa o display ST7789.
 *        Lê resolução vertical da NVS (default 240px se não
 *        encontrar valor gravado). Configura SPI, GPIO e backlight.
 *        Chamar UMA VEZ em display_manager_init().
 */
void st7789_init(void);

/**
 * @brief Grava resolução vertical na NVS e actualiza g_lcd_v_res.
 *        Chamar apenas uma vez por poste quando se muda o ecrã.
 *        O valor persiste em todos os arranques seguintes.
 *
 * @param v_res  Resolução vertical: 240 ou 320. Outros valores ignorados.
 */
void st7789_set_resolution(uint16_t v_res);

/**
 * @brief Devolve a resolução vertical actualmente configurada.
 * @return 240 ou 320
 */
uint16_t st7789_get_resolution(void);

/**
 * @brief Define janela de escrita no display.
 *        Chamada internamente por st7789_draw_bitmap().
 */
void st7789_set_window(uint16_t x0, uint16_t y0,
                       uint16_t x1, uint16_t y1);

/**
 * @brief Envia buffer de pixels para o display.
 *        Chamado pelo flush callback do LVGL.
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
