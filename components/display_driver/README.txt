================================================================================
MODULO: display_driver (ST7789)
VERSAO: 4.0 | DATA: 2026-03-23
PROJECTO: Poste Inteligente v6
AUTORES: Luis Custodio | Tiago Moreno
================================================================================

DESCRICAO
---------
Driver SPI de baixo nivel para o display TFT ST7789 240x240 pixels.
Fornece inicializacao do controlador, configuracao de janela de escrita
e renderizacao de bitmaps RGB565. Compativel com o flush callback do LVGL.

O QUE FAZ
---------
  - Inicializa o barramento SPI2 (HSPI) do ESP32 a 20 MHz
  - Configura o ST7789: RGB565, inversao de cor, MADCTL (orientacao)
  - Implementa set_window() para definir area de escrita no display
  - Renderiza bitmaps RGB565 via DMA SPI
  - Controla o backlight (GPIO 25)
  - Fornece st7789_flush_cb() compativel com lv_disp_drv_t do LVGL

API PUBLICA
-----------
  void st7789_init(void)
      Inicializa GPIO, SPI e sequencia de arranque do ST7789.
      Deve ser chamada UMA VEZ antes de qualquer render.

  void st7789_set_window(x0, y0, x1, y1)
      Define a janela de escrita (colunas + linhas) no controlador.
      Envia comandos CASET (0x2A) e RASET (0x2B) + RAMWR (0x2C).

  void st7789_draw_bitmap(x, y, w, h, data)
      Escreve bitmap RGB565 na janela definida.
      data: array uint16_t, um word por pixel.
      Usado pelo LVGL como flush_cb.

  void st7789_backlight(bool on)
      Liga/desliga o backlight (GPIO 25).

LIGACOES HARDWARE
-----------------
  ESP32 GPIO  -->  ST7789 Pino
  GPIO 23     -->  SDA (MOSI / dados)
  GPIO 18     -->  SCL (SCLK / clock)
  GPIO  5     -->  CS  (Chip Select)
  GPIO 32     -->  DC  (Data / Command)
  GPIO 33     -->  RST (Reset hardware)
  GPIO 25     -->  BL  (Backlight)
  3.3V        -->  VCC
  GND         -->  GND

  NOTA: o ST7789 trabalha a 3.3V. Nao ligar a 5V.
  Ordem dos pinos no conector tipico: GND VCC SCL SDA RST DC CS BL

CONFIGURACAO DO CONTROLADOR
-----------------------------
  Formato de cor : RGB565 (COLMOD 0x55)
  Orientacao     : MADCTL 0x08 (BGR bit activo para cor correcta)
  Inversao       : INVON (0x21) activa -- necessario para ST7789
  SPI            : SPI2_HOST (HSPI), 20 MHz, modo 0, half-duplex
  DMA            : SPI_DMA_CH_AUTO

  IMPORTANTE SOBRE MADCTL:
    0x00 = RGB normal  -> pode aparecer com cores trocadas
    0x08 = BGR activo  -> corrige a inversao R<->B do ST7789
  Se as cores ainda aparecerem trocadas, testar 0x00.

FORMATO DE COR RGB565
----------------------
  Cada pixel = 16 bits: RRRRRGGGGGGBBBBB
  5 bits R | 6 bits G | 5 bits B
  Nota: o LVGL usa lv_color_t internamente (igual a RGB565 com depth=16).
  lv_color_hex(0xRRGGBB) converte de 24-bit para RGB565 automaticamente.

DEPENDENCIAS
------------
  hw_config.h          -- LCD_PIN_* (todos os pinos do display)
  driver/spi_master.h  -- API SPI do ESP-IDF
  driver/gpio.h        -- GPIO para DC, RST, BL, CS
  freertos/task.h      -- vTaskDelay nos delays de reset

COMPONENTE DE HARDWARE
-----------------------
  Controlador : ST7789V (Sitronix)
  Resolucao   : 240 x 240 pixels
  Interface   : SPI (4 fios + CS + DC + RST + BL)
  Tensao      : 3.3V
  Tamanho     : 1.3" ou 1.54" (tipico)

DATASHEET
---------
  ST7789V Datasheet (Sitronix):
    https://www.waveshare.com/w/upload/a/ae/ST7789_Datasheet.pdf

  ST7789 Application Note:
    https://www.rhydolabz.com/documents/33/ST7789.pdf

REFERENCIAS
-----------
  ESP-IDF SPI Master Driver:
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html

  LVGL Flush Callback (como integrar driver com LVGL):
    https://docs.lvgl.io/8.3/porting/display.html

  GitHub - ESP-IDF SPI Master exemplo:
    https://github.com/espressif/esp-idf/tree/master/examples/peripherals/spi_master

  YouTube - ST7789 com ESP32 (Low Level Learning):
    https://www.youtube.com/@LowLevelLearning

  YouTube - Display SPI ESP32 ESP-IDF:
    https://www.youtube.com/@EspressifSystems
================================================================================
