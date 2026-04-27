/* ============================================================
   ST7789 — IMPLEMENTAÇÃO
   @file      st7789.c
   @version   5.0  |  2026-04-27
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   NOVIDADE v4.1 → v5.0 — AUTO-DETECÇÃO DE RESOLUÇÃO:
   ──────────────────────────────────────────────────────
   g_lcd_v_res: variável global que substitui o #define LCD_V_RES.
   Inicializada em st7789_init() com o valor lido da NVS.
   Se a NVS não tiver valor gravado, usa 240 como default seguro.

   COMO CONFIGURAR UM POSTE COM ECRÃ 240×320:
   ─────────────────────────────────────────────
   1. Flash o firmware normalmente.
   2. No monitor série, chamar st7789_set_resolution(320)
      UMA VEZ (pode ser via comando de debug, menu UART, etc.).
   3. Reiniciar — o poste arranca com LCD_V_RES=320 para sempre.
   4. Para voltar a 240px: st7789_set_resolution(240).

   IMPACTO NO RESTO DO CÓDIGO:
   ─────────────────────────────
   display_manager.c usa LCD_V_RES via macro → resolve para
   g_lcd_v_res em runtime. RADAR_H = (LCD_V_RES - 149) adapta-se
   automaticamente. NENHUM outro ficheiro precisa de alteração.

   MANTIDO de v4.1:
   ─────────────────
   - MADCTL=0x00 (RGB) + INVON (0x21) + LV_COLOR_16_SWAP=1
   - Sequência de init validada para 240×240 e 240×320
============================================================ */

#include "st7789.h"
#include "hw_config.h"
#include "system_config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "ST7789";

/* Handle SPI */
static spi_device_handle_t spi;

/* ============================================================
   VARIÁVEL GLOBAL DE RESOLUÇÃO
   ──────────────────────────────
   Declarada extern em hw_config.h como g_lcd_v_res.
   O macro LCD_V_RES resolve para esta variável.
   Valor inicial 240 — sobrescrito por _ler_resolucao_nvs()
   durante st7789_init() se houver valor gravado na NVS.
============================================================ */
uint16_t g_lcd_v_res = 240;

/* Namespace e chave NVS */
#define NVS_NAMESPACE   "hw_cfg"
#define NVS_KEY_V_RES   "lcd_v_res"


/* ============================================================
   _ler_resolucao_nvs — lê resolução gravada na NVS
   ──────────────────────────────────────────────────
   Chamada no início de st7789_init().
   Se não houver valor gravado ou o valor for inválido,
   mantém g_lcd_v_res = 240 (default seguro).
============================================================ */
static void _ler_resolucao_nvs(void)
{
    /* LCD_V_RES_CONFIG vem de system_config.h — editado por poste.
       Comportamento:
         1. NVS vazia ou inválida → usa LCD_V_RES_CONFIG e grava.
         2. NVS diferente de LCD_V_RES_CONFIG → reflash detectado
            → actualiza NVS com o novo valor de system_config.h.
         3. NVS igual a LCD_V_RES_CONFIG → usa sem alteração.
       Assim basta editar LCD_V_RES_CONFIG e reflashar. */
    const uint16_t cfg = LCD_V_RES_CONFIG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        /* NVS inacessível — usa valor de system_config.h em RAM */
        g_lcd_v_res = (cfg == 240 || cfg == 320) ? cfg : 240;
        ESP_LOGW(TAG, "NVS inacessível — usa %dpx (system_config.h)",
                 g_lcd_v_res);
        return;
    }

    uint16_t v_nvs = 0;
    esp_err_t get_err = nvs_get_u16(h, NVS_KEY_V_RES, &v_nvs);

    if (get_err != ESP_OK || (v_nvs != 240 && v_nvs != 320)) {
        /* NVS vazia ou inválida — grava LCD_V_RES_CONFIG */
        g_lcd_v_res = (cfg == 240 || cfg == 320) ? cfg : 240;
        nvs_set_u16(h, NVS_KEY_V_RES, g_lcd_v_res);
        nvs_commit(h);
        ESP_LOGI(TAG, "NVS: resolução gravada %dpx (system_config.h)",
                 g_lcd_v_res);
    } else if (v_nvs != cfg && (cfg == 240 || cfg == 320)) {
        /* system_config.h foi alterado — actualiza NVS */
        g_lcd_v_res = cfg;
        nvs_set_u16(h, NVS_KEY_V_RES, g_lcd_v_res);
        nvs_commit(h);
        ESP_LOGI(TAG, "NVS: resolução actualizada %d→%dpx (system_config.h)",
                 v_nvs, g_lcd_v_res);
    } else {
        /* NVS igual ao system_config.h — usa directamente */
        g_lcd_v_res = v_nvs;
        ESP_LOGI(TAG, "NVS: resolução %dpx OK", g_lcd_v_res);
    }

    nvs_close(h);
}


/* ============================================================
   CONTROLO DC / ENVIO SPI / RESET
============================================================ */
static inline void dc_cmd(void)  { gpio_set_level(LCD_PIN_DC, 0); }
static inline void dc_data(void) { gpio_set_level(LCD_PIN_DC, 1); }

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

static void reset_display(void)
{
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}


/* ============================================================
   st7789_init — inicialização completa
   ──────────────────────────────────────
   1. Lê resolução da NVS → g_lcd_v_res (240 ou 320)
   2. Configura GPIO e SPI
   3. Sequência de init ST7789 (igual para 240 e 320)

   A sequência de init é idêntica para ambos os tamanhos —
   o ST7789 adapta-se automaticamente à memória de vídeo
   disponível (240 linhas vs 320 linhas) sem comandos extra.
============================================================ */
void st7789_init(void)
{
    /* 1. Lê resolução gravada na NVS */
    _ler_resolucao_nvs();

    ESP_LOGI(TAG, "A inicializar ST7789 %dx%d...", LCD_H_RES, g_lcd_v_res);

    /* 2. GPIO */
    gpio_set_direction(LCD_PIN_DC,  GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_PIN_BL,  GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_PIN_CS,  GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_PIN_BL, 0);  /* backlight OFF durante init */

    reset_display();

    /* 3. SPI */
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

    /* 4. Sequência de inicialização ST7789
          Igual para 240×240 e 240×320 — o chip adapta-se. */
    write_cmd(0x11);                /* Sleep out — aguarda 120ms obrigatório */
    vTaskDelay(pdMS_TO_TICKS(120));

    write_cmd(0x3A);                /* Interface Pixel Format — RGB565 */
    uint8_t colmod = 0x55;
    write_data(&colmod, 1);

    write_cmd(0x36);                /* MADCTL — RGB normal, sem inversão de eixos */
    uint8_t madctl = 0x00;
    write_data(&madctl, 1);

    write_cmd(0x21);                /* INVON — inversão obrigatória nestes painéis */

    write_cmd(0x29);                /* Display ON */

    gpio_set_level(LCD_PIN_BL, 1); /* Backlight ON */

    ESP_LOGI(TAG, "ST7789 v5.0 pronto | %dx%d | RGB565 | INVON",
             LCD_H_RES, g_lcd_v_res);
}


/* ============================================================
   st7789_set_resolution — grava resolução na NVS
   ──────────────────────────────────────────────────
   Chamar UMA VEZ por poste quando se troca o ecrã.
   Reiniciar após chamar para que o display_manager use
   o novo valor de LCD_V_RES.

   @param v_res  240 ou 320. Outros valores são ignorados.
============================================================ */
void st7789_set_resolution(uint16_t v_res)
{
    if (v_res != 240 && v_res != 320) {
        ESP_LOGW(TAG, "st7789_set_resolution: valor inválido %d (aceita 240 ou 320)",
                 v_res);
        return;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open erro %d — resolução NÃO gravada", err);
        return;
    }

    nvs_set_u16(h, NVS_KEY_V_RES, v_res);
    nvs_commit(h);
    nvs_close(h);

    g_lcd_v_res = v_res;

    ESP_LOGI(TAG, "Resolução gravada na NVS: %dpx — reiniciar para aplicar",
             v_res);
}


/* ============================================================
   st7789_get_resolution — devolve resolução actual
============================================================ */
uint16_t st7789_get_resolution(void)
{
    return g_lcd_v_res;
}


/* ============================================================
   st7789_set_window — define janela de escrita (CASET+RASET)
============================================================ */
void st7789_set_window(uint16_t x0, uint16_t y0,
                       uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    write_cmd(0x2A);    /* CASET — colunas */
    data[0] = x0 >> 8; data[1] = x0 & 0xFF;
    data[2] = x1 >> 8; data[3] = x1 & 0xFF;
    write_data(data, 4);

    write_cmd(0x2B);    /* RASET — linhas */
    data[0] = y0 >> 8; data[1] = y0 & 0xFF;
    data[2] = y1 >> 8; data[3] = y1 & 0xFF;
    write_data(data, 4);

    write_cmd(0x2C);    /* RAMWR */
}


/* ============================================================
   st7789_draw_bitmap — envia buffer RGB565 para o display
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
   st7789_backlight — controlo do backlight
============================================================ */
void st7789_backlight(bool on)
{
    gpio_set_level(LCD_PIN_BL, on ? 1 : 0);
}
