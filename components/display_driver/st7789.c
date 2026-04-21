/* ============================================================
   ST7789 — IMPLEMENTAÇÃO
   @file      st7789.c
   @brief     Driver SPI para display TFT ST7789 240x240/240x320
   @version   4.1  |  2026-04-17
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   MUDANÇAS v4.0 → v4.1:
   ─────────────────────────
   - CORRIGIDO: sequência de inicialização com inversão correcta.
     madctl=0x00 (RGB) + INVON (0x21) + LV_COLOR_16_SWAP=1
     resolve cores trocadas em módulos ST7789 240×320.
   - ADICIONADO: comentário na NVS para orientação futura.
   - REMOVIDO: #include <inttypes.h> desnecessário.
============================================================ */

#include "st7789.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hw_config.h"
#include "system_config.h"

static const char *TAG = "ST7789";

/* Handle SPI */
static spi_device_handle_t spi;

/* ============================================================
   CONTROLO DC
============================================================ */
static inline void dc_cmd(void)
{
    gpio_set_level(LCD_PIN_DC, 0);
}

static inline void dc_data(void)
{
    gpio_set_level(LCD_PIN_DC, 1);
}

/* ============================================================
   ENVIO SPI
============================================================ */
static void write_cmd(uint8_t cmd)
{
    spi_transaction_t t = {0};
    dc_cmd();
    t.length    = 8;
    t.tx_buffer = &cmd;
    spi_device_transmit(spi, &t);
}

static void write_data(const uint8_t *data, int len)
{
    if (len == 0) return;
    spi_transaction_t t = {0};
    dc_data();
    t.length    = len * 8;
    t.tx_buffer = data;
    spi_device_transmit(spi, &t);
}

/* ============================================================
   RESET HARDWARE
============================================================ */
static void reset_display(void)
{
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

/* ============================================================
   st7789_init
   ─────────────
   Sequência de inicialização validada para módulos ST7789
   240×240 e 240×320 com painel RGB.

   Configuração de cor:
     MADCTL = 0x00  → RGB, sem inversão de eixos
     INVON  (0x21)  → inversão de cor activa (necessária
                      nestes painéis para preto=preto)
     LV_COLOR_16_SWAP = 1 em lv_conf.h → swap de bytes
                      RGB565 para ordem correcta do painel.
============================================================ */
void st7789_init(void)
{
    ESP_LOGI(TAG, "A inicializar ST7789...");

    /* ── Configuração GPIO ────────────────────────────────── */
    gpio_set_direction(LCD_PIN_DC,  GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_PIN_BL,  GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_PIN_CS,  GPIO_MODE_OUTPUT);

    /* Backlight OFF durante inicialização */
    gpio_set_level(LCD_PIN_BL, 0);

    /* Reset hardware */
    reset_display();

    /* ── Configuração SPI ─────────────────────────────────── */
    spi_bus_config_t buscfg = {
        .mosi_io_num     = LCD_PIN_SDA,
        .miso_io_num     = -1,
        .sclk_io_num     = LCD_PIN_SCL,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = LCD_H_RES * 40 * 2
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20000000,
        .mode           = 0,
        .spics_io_num   = LCD_PIN_CS,
        .queue_size     = 7,
        .flags          = SPI_DEVICE_HALFDUPLEX
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    /* ── Sequência de inicialização ST7789 ───────────────── */

    /* Sleep out — aguarda 120ms obrigatório */
    write_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));

    /* Interface Pixel Format — RGB565 */
    write_cmd(0x3A);
    uint8_t colmod = 0x55;
    write_data(&colmod, 1);

    /* MADCTL — RGB normal, sem inversão de eixos
     * 0x00 = RGB, orientação normal
     * 0x08 = BGR (não usar — cores trocadas)
     * 0x60 = rotação 90°
     * 0xC0 = rotação 180° */
    write_cmd(0x36);
    uint8_t madctl = 0x00;
    write_data(&madctl, 1);

    /* Inversão de cor — OBRIGATÓRIA para este painel.
     * Sem esta linha: preto=branco, cores invertidas.
     * Com LV_COLOR_16_SWAP=1 no lv_conf.h, as cores
     * ficam correctas. */
    write_cmd(0x21);

    /* Display ON */
    write_cmd(0x29);

    /* Backlight ON */
    gpio_set_level(LCD_PIN_BL, 1);

    ESP_LOGI(TAG, "ST7789 v4.1 pronto | RGB565 | INVON | BL ON");
}

/* ============================================================
   st7789_set_window
   ──────────────────
   Define a janela de escrita (CASET + RASET + RAMWR).
   Chamada antes de st7789_draw_bitmap().
============================================================ */
void st7789_set_window(uint16_t x0, uint16_t y0,
                       uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    /* Colunas (CASET) */
    write_cmd(0x2A);
    data[0] = x0 >> 8;
    data[1] = x0 & 0xFF;
    data[2] = x1 >> 8;
    data[3] = x1 & 0xFF;
    write_data(data, 4);

    /* Linhas (RASET) */
    write_cmd(0x2B);
    data[0] = y0 >> 8;
    data[1] = y0 & 0xFF;
    data[2] = y1 >> 8;
    data[3] = y1 & 0xFF;
    write_data(data, 4);

    /* RAM write (RAMWR) */
    write_cmd(0x2C);
}

/* ============================================================
   st7789_draw_bitmap
   ───────────────────
   Envia buffer de pixels RGB565 para o display.
   Chamado pelo flush callback do LVGL.

   @param x     Coluna inicial
   @param y     Linha inicial
   @param w     Largura em pixels
   @param h     Altura em pixels
   @param data  Buffer RGB565 (w × h × 2 bytes)
============================================================ */
void st7789_draw_bitmap(uint16_t x, uint16_t y,
                        uint16_t w, uint16_t h,
                        const uint16_t *data)
{
    if (!data || w == 0 || h == 0) return;

    st7789_set_window(x, y, x + w - 1, y + h - 1);

    dc_data();

    spi_transaction_t t = {0};
    t.length    = (size_t)w * h * 16;
    t.tx_buffer = data;
    spi_device_transmit(spi, &t);
}

/* ============================================================
   st7789_backlight
   ─────────────────
   Liga ou desliga o backlight via GPIO.
   @param on  true = backlight ligado
============================================================ */
void st7789_backlight(bool on)
{
    gpio_set_level(LCD_PIN_BL, on ? 1 : 0);
}