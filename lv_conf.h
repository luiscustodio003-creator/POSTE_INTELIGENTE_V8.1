/**
 * ============================================================
 * LVGL CONFIGURATION - Poste Inteligente v8
 * ============================================================
 * @file      lv_conf.h
 * @version   8.3.11
 * @platform  ESP32 + ST7789 SPI Display
 * @date      14 Abril 2026
 *
 * CORRECÇÕES 2026-04-15:
 * [FIX-T1] LV_TICK_CUSTOM=1: lv_tick_inc() não é compilada pelo LVGL.
 *          display_manager_tick() corrigida para não chamar lv_tick_inc().
 *          O LVGL lê o tick directamente de esp_timer_get_time()/1000.
 * [FIX-F1] LV_FONT_MONTSERRAT_12 desactivada — nunca usada no código.
 * [FIX-F2] LV_FONT_MONTSERRAT_18 desactivada — nunca usada no código.
 *          Poupança estimada: ~10-16KB de flash.
 *
 * LOCALIZAÇÃO: Raiz do projeto
 * LVGL: components/lvgl/
 * ============================================================
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#define LV_CONF_OK 1

#include <stdint.h>
#include "esp_timer.h"

/* ============================================================
   MEMÓRIA E PERFORMANCE
============================================================ */
#define LV_MEM_CUSTOM           0
#define LV_MEM_SIZE             (128U * 1024U)  /* 128KB para buffers */
#define LV_MEM_BUF_MAX_NUM      32

/* ============================================================
   TICK - ESP32 Timer
   IMPORTANTE: Com LV_TICK_CUSTOM=1, o LVGL chama directamente
   LV_TICK_CUSTOM_SYS_TIME_EXPR para obter o tempo.
   lv_tick_inc() NÃO é compilada — não chamar em código de aplicação.
   display_manager_tick() foi adaptada para respeitar este comportamento.
============================================================ */
#define LV_TICK_CUSTOM              1
#define LV_TICK_CUSTOM_INCLUDE      "esp_timer.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (esp_timer_get_time() / 1000)

/* ============================================================
   DISPLAY E REFRESH
============================================================ */
#define LV_DISP_DEF_REFR_PERIOD     10      /* 100Hz interno — display_task corre a 50Hz (20ms) */
#define LV_INDEV_DEF_READ_PERIOD    30      /* 33Hz input polling */
#define LV_DPI_DEF                  130

/* ============================================================
   COR - ST7789 (RGB565) - CONFIGURAÇÃO CRÍTICA
============================================================ */
#define LV_COLOR_DEPTH              16
#define LV_COLOR_16_SWAP            1      /* ESP32 SPI já envia RGB correcto */
#define LV_COLOR_SCREEN_TRANSP      0
#define LV_COLOR_MIX_ROUND_OFS      0

/* ============================================================
   GPU E ACELERAÇÃO (Desativado para ESP32)
============================================================ */
#define LV_USE_GPU_STM32_DMA2D      0
#define LV_USE_EXTERNAL_RENDERER    0

/* ============================================================
   FONTES - Montserrat (Sans-serif profissional)
   Apenas activar as fontes realmente usadas no código:
     10px → headers dos cards (display_manager.c)
     14px → todos os outros labels (fonte por omissão)
   [FIX-F1] montserrat_12 desactivada — não usada
   [FIX-F2] montserrat_18 desactivada — não usada
============================================================ */
#define LV_FONT_MONTSERRAT_8        0
#define LV_FONT_MONTSERRAT_10       1   /* Headers dos cards de dados */
#define LV_FONT_MONTSERRAT_12       0   /* [FIX-F1] Não usada — desactivada */
#define LV_FONT_MONTSERRAT_14       1   /* Labels principais, badge, WiFi, etc. */
#define LV_FONT_MONTSERRAT_16       0
#define LV_FONT_MONTSERRAT_18       0   /* [FIX-F2] Não usada — desactivada */
#define LV_FONT_MONTSERRAT_20       0
#define LV_FONT_MONTSERRAT_22       0
#define LV_FONT_MONTSERRAT_24       0
#define LV_FONT_MONTSERRAT_26       0
#define LV_FONT_MONTSERRAT_28       0

#define LV_FONT_DEFAULT             &lv_font_montserrat_14

/* ============================================================
   WIDGETS - Apenas os utilizados (otimização)
   Usados: BAR (DALI), CANVAS (Radar), LABEL (texto), OBJ (base)
============================================================ */
#define LV_USE_ARC                  0
#define LV_USE_BAR                  1       /* Barra DALI */
#define LV_USE_BTN                  0
#define LV_USE_BTNMATRIX            0
#define LV_USE_CANVAS               1       /* Canvas Radar */
#define LV_USE_CHECKBOX             0
#define LV_USE_DROPDOWN             0
#define LV_USE_IMG                  1       /* Ícones WiFi */
#define LV_USE_LABEL                1       /* Texto */
#define LV_USE_LINE                 1       /* Linhas Canvas */
#define LV_USE_ROLLER               0
#define LV_USE_SLIDER               0
#define LV_USE_SWITCH               0
#define LV_USE_TEXTAREA             0
#define LV_USE_TABLE                0

/* Widgets Extra - Desativados */
#define LV_USE_ANIMIMG              0
#define LV_USE_CALENDAR             0
#define LV_USE_CHART                0
#define LV_USE_COLORWHEEL           0
#define LV_USE_IMGBTN               0
#define LV_USE_KEYBOARD             0
#define LV_USE_LED                  0
#define LV_USE_LIST                 0
#define LV_USE_MENU                 0
#define LV_USE_METER                0
#define LV_USE_MSGBOX               0
#define LV_USE_SPAN                 0
#define LV_USE_SPINBOX              0
#define LV_USE_SPINNER              0
#define LV_USE_TABVIEW              0
#define LV_USE_TILEVIEW             0
#define LV_USE_WIN                  0

/* ============================================================
   TEMAS - Desativados (UI customizado)
============================================================ */
#define LV_USE_THEME_DEFAULT        0
#define LV_USE_THEME_BASIC          0
#define LV_USE_THEME_MONO           0

/* ============================================================
   LAYOUTS
============================================================ */
#define LV_USE_FLEX                 1
#define LV_USE_GRID                 0

/* ============================================================
   LOGGING E DEBUG
============================================================ */
#define LV_USE_LOG                  0
#define LV_LOG_LEVEL                LV_LOG_LEVEL_WARN
#define LV_LOG_PRINTF               0

/* ============================================================
   ASSERTS - Desativados para produção
============================================================ */
#define LV_USE_ASSERT_NULL          0
#define LV_USE_ASSERT_MALLOC        0
#define LV_USE_ASSERT_STYLE         0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_OBJ           0

/* ============================================================
   OUTRAS CONFIGURAÇÕES
============================================================ */
#define LV_USE_PERF_MONITOR         0
#define LV_USE_MEM_MONITOR          0
#define LV_USE_REFR_DEBUG           0
#define LV_USE_LARGE_COORD          0
#define LV_USE_USER_DATA            1

/* ============================================================
   ANIMAÇÕES
============================================================ */
#define LV_USE_ANIMATION            1
#define LV_ANIM_CACHE_ENABLE        1

/* ============================================================
   FILESYSTEM - Desativado
============================================================ */
#define LV_USE_FS_STDIO             0
#define LV_USE_FS_POSIX             0
#define LV_USE_FS_WIN32             0
#define LV_USE_FS_FATFS             0

/* ============================================================
   IMAGENS - Desativado (ícones embutidos no código)
============================================================ */
#define LV_USE_PNG                  0
#define LV_USE_BMP                  0
#define LV_USE_SJPG                 0
#define LV_USE_GIF                  0
#define LV_USE_QRCODE               0

/* ============================================================
   OUTRAS BIBLIOTECAS - Desativado
============================================================ */
#define LV_USE_FREETYPE             0
#define LV_USE_RLOTTIE              0
#define LV_USE_FFMPEG               0

/* ============================================================
   SNAPSHOT / MONKEY / GRIDNAV / FRAGMENT / IMGFONT / MSG / IME
   - Todos desativados
============================================================ */
#define LV_USE_SNAPSHOT             0
#define LV_USE_MONKEY               0
#define LV_USE_GRIDNAV              0
#define LV_USE_FRAGMENT             0
#define LV_USE_IMGFONT              0
#define LV_USE_MSG                  0
#define LV_USE_IME_PINYIN           0

#endif /* LV_CONF_H */
