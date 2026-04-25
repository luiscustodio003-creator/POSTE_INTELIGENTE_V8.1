/* ============================================================
   DISPLAY MANAGER — IMPLEMENTAÇÃO
   @file      display_manager.c
   @brief     Camada de apresentação LVGL + ST7789 — ecrã 240×240
   @version   6.0  |  2026-04-24
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   Layout do ecrã:
   ───────────────────────────────────────
     y=  0.. 35  → ZONA IDENTIDADE
     y= 36.. 36  → separador
     y= 37.. 92  → ZONA HARDWARE
     y= 93.. 93  → separador
     y= 94..145  → ZONA TRÁFEGO
     y=146..146  → separador
     y=147..239  → ZONA RADAR (canvas 230×90 px)

   MUDANÇA PRINCIPAL v5.7 → v6.0:
   ──────────────────────────────────────────────────────────
   REMOVIDO: interpolador preditivo (radar_interp_t, _interp_update,
   _interp_aplicar_frame).
   O interpolador avançava a posição do objecto com base na
   velocidade entre frames reais — mostrava onde o algoritmo
   ACHAVA que o objecto estava, não onde ele REALMENTE estava.

   SUBSTITUÍDO POR: posição directa (dm_alvo_t).
   O canvas mostra exactamente a posição x_mm / y_mm que vem
   do tracking_manager a cada frame.
   O rasto é construído com posições REAIS anteriores (buffer
   circular preenchido apenas quando chega um frame real),
   nunca com posições simuladas.

   Timeout de visibilidade: ALVO_HOLD_MS (500ms).
   Se não chegar frame real durante esse tempo, o alvo
   desaparece do canvas — sem "fantasma" a mover-se sozinho.

   Dependências:
   ─────────────
     display_manager.h : API pública
     st7789.h          : driver SPI do display físico
     system_config.h   : LCD_H_RES, LCD_V_RES, RADAR_MAX_MM
     hw_config.h       : LCD_PIN_*
     post_config.h     : post_get_name(), post_get_id()
     lvgl              : v8.3.x (LV_USE_CANVAS=1 em lv_conf.h)
============================================================ */

#include "display_manager.h"
#include "state_machine.h"
#include "st7789.h"
#include "system_config.h"
#include "hw_config.h"
#include "post_config.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "lvgl.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "DISP_MGR";

/* ============================================================
   FILA DE MENSAGENS — THREAD SAFETY
   ──────────────────────────────────────────────────────────
   Toda a comunicação entre tasks produtoras (fsm_task, udp_task)
   e a display_task consumidora é feita por esta fila.
   Nunca chamar funções LVGL fora da display_task.
============================================================ */
#define DM_FILA_CAP  24

typedef enum {
    DM_MSG_STATUS = 0,
    DM_MSG_WIFI,
    DM_MSG_HARDWARE,
    DM_MSG_TRAFFIC,
    DM_MSG_SPEED,
    DM_MSG_NEIGHBORS,
    DM_MSG_RADAR,
} dm_msg_tipo_t;

typedef struct {
    dm_msg_tipo_t tipo;
    union {
        struct { char status[16]; } st;
        struct { bool connected; char ip[16]; } wifi;
        struct { char radar_st[8]; bool radar_ok; uint8_t brightness; } hw;
        struct { int T; int Tc; } traf;
        struct { int speed; } spd;
        struct {
            char nebL[16]; char nebR[16];
            bool leftOk;   bool rightOk;
        } neb;
        struct {
            radar_obj_t objs[RADAR_MAX_OBJ];
            uint8_t     count;
        } radar;
    };
} dm_msg_t;

static QueueHandle_t s_fila = NULL;


/* ============================================================
   PALETA DE CORES
============================================================ */
#define COR_BRANCO      0xFFFFFF
#define COR_PRETO       0x000000
#define COR_CINZENTO    0x6B7280
#define COR_CINZ_CLARO  0x9CA3AF
#define COR_VERDE       0x22C55E
#define COR_VERMELHO    0xFF3333
#define COR_AMARELO     0xFF8C00
#define COR_LARANJA     0xFF8C00
#define COR_CIANO       0x00D4FF
#define COR_VIOLETA     0xA855F7
#define COR_SEPARADOR   0x374151
#define COR_FUNDO_CARD  0x010601


/* ============================================================
   DIMENSÕES DO CANVAS DO RADAR
============================================================ */
#define RADAR_W       230
#define RADAR_H        90
#define RADAR_X_OFF     5
#define RADAR_Y_OFF   149

/* Tempo máximo sem frame real antes de apagar o alvo do canvas */
#define ALVO_HOLD_MS  500


/* ============================================================
   PONTEIROS PARA ELEMENTOS VISUAIS
============================================================ */
static lv_obj_t *label_nome;
static lv_obj_t *label_badge;
static lv_obj_t *label_wifi;
static lv_obj_t *label_radar_st;
static lv_obj_t *label_neb_esq;
static lv_obj_t *label_neb_dir;
static lv_obj_t *label_dali;
static lv_obj_t *bar_dali;
static lv_obj_t *label_T_val;
static lv_obj_t *label_Tc_val;
static lv_obj_t *label_vel_val;
static lv_obj_t *card_T;
static lv_obj_t *card_Tc;
static lv_obj_t *card_vel;
static lv_obj_t *canvas_radar;

/* Buffer do canvas — RGB565 */
static lv_color_t radar_buf[RADAR_W * RADAR_H];


/* ============================================================
   ESTADO DOS ALVOS — POSIÇÕES REAIS
   ──────────────────────────────────────────────────────────
   dm_alvo_t armazena a última posição REAL recebida do
   tracking_manager e um buffer circular de posições reais
   anteriores (rasto).

   Não existe nenhum campo de velocidade para movimento
   preditivo — o alvo só se move quando chega um novo frame.
   Se não chegar frame dentro de ALVO_HOLD_MS, desaparece.
============================================================ */
typedef struct {
    float    x_mm;                      /* Posição lateral real (mm)         */
    float    y_mm;                      /* Posição frontal real (mm)         */
    float    speed_kmh;                 /* Velocidade para label (km/h)      */
    bool     activo;                    /* true = visível no canvas          */
    uint32_t ultimo_ms;                 /* Timestamp do último frame real    */
    /* Rasto — posições reais anteriores em buffer circular */
    uint8_t  trail_head;               /* Índice circular (próxima escrita)  */
    float    trail_x[RADAR_TRAIL_MAX]; /* Buffer circular de X reais         */
    float    trail_y[RADAR_TRAIL_MAX]; /* Buffer circular de Y reais         */
    uint8_t  trail_len;                /* Número de entradas válidas         */
} dm_alvo_t;

static dm_alvo_t s_alvos[RADAR_MAX_OBJ];

/* Estado de tráfego actual — para indicador de trânsito no canvas */
static int s_T_actual  = 0;  /* Veículos detectados localmente         */
static int s_Tc_actual = 0;  /* Veículos a caminho (anunciados via UDP) */


/* ============================================================
   FUNÇÕES INTERNAS DE APOIO
============================================================ */

/* Callback de flush SPI para o LVGL */
static void st7789_flush_cb(lv_disp_drv_t  *disp_drv,
                             const lv_area_t *area,
                             lv_color_t      *color_p)
{
    int32_t w = area->x2 - area->x1 + 1;
    int32_t h = area->y2 - area->y1 + 1;
    st7789_draw_bitmap((uint16_t)area->x1, (uint16_t)area->y1,
                       (uint16_t)w, (uint16_t)h,
                       (const uint16_t *)color_p);
    lv_disp_flush_ready(disp_drv);
}

/* Cria linha horizontal de separação de zonas */
static void _separador(lv_obj_t *pai, int y_pos)
{
    lv_obj_t *l = lv_obj_create(pai);
    lv_obj_set_size(l, LCD_H_RES - 10, 1);
    lv_obj_set_pos(l, 5, y_pos);
    lv_obj_set_style_bg_color(l, lv_color_hex(COR_SEPARADOR), 0);
    lv_obj_set_style_bg_opa(l, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(l, 0, 0);
    lv_obj_set_style_radius(l, 0, 0);
    lv_obj_set_style_pad_all(l, 0, 0);
}

/* Cria label LVGL com posição, cor e texto iniciais */
static lv_obj_t *_label_novo(lv_obj_t *pai, int x, int y,
                              uint32_t cor, const char *texto)
{
    lv_obj_t *l = lv_label_create(pai);
    lv_label_set_text(l, texto);
    lv_obj_set_pos(l, x, y);
    lv_obj_set_style_text_color(l, lv_color_hex(cor), 0);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    return l;
}


/* ============================================================
   FUNÇÕES DE DESENHO PIXEL DIRECTO NO BUFFER RADAR
============================================================ */

/* Pinta pixel com blending de alpha — sem escrita fora dos limites */
static inline void _px_blend(int x, int y, lv_color_t cor, uint8_t alpha)
{
    if ((unsigned)x >= RADAR_W || (unsigned)y >= RADAR_H) return;
    if (alpha == 0)   return;
    if (alpha == 255) { radar_buf[y * RADAR_W + x] = cor; return; }
    radar_buf[y * RADAR_W + x] =
        lv_color_mix(cor, radar_buf[y * RADAR_W + x], alpha);
}

/* Preenche linha horizontal completa com cor sólida */
static void _linha_h_px(int y, lv_color_t cor)
{
    if ((unsigned)y >= RADAR_H) return;
    for (int x = 0; x < RADAR_W; x++)
        radar_buf[y * RADAR_W + x] = cor;
}

/* Círculo preenchido com gradiente radial suave */
static void _circulo_px(int cx, int cy, int r, lv_color_t cor)
{
    int r2 = r * r;
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            int d2 = dx*dx + dy*dy;
            if (d2 > r2) continue;
            uint8_t alpha = (uint8_t)(255 - (d2 * 75) / (r2 + 1));
            _px_blend(cx + dx, cy + dy, cor, alpha);
        }
    }
}

/* Anel (halo) com gradiente entre raio interior e exterior */
static void _halo_px(int cx, int cy, int r_int, int r_ext,
                     lv_color_t cor, uint8_t alpha_base)
{
    int ri2  = r_int * r_int;
    int re2  = r_ext * r_ext;
    int span = re2 - ri2 + 1;
    for (int dy = -r_ext; dy <= r_ext; dy++) {
        for (int dx = -r_ext; dx <= r_ext; dx++) {
            int d2 = dx*dx + dy*dy;
            if (d2 < ri2 || d2 > re2) continue;
            uint8_t a = (uint8_t)((uint32_t)alpha_base * (re2 - d2) / span);
            _px_blend(cx + dx, cy + dy, cor, a);
        }
    }
}

/* Converte posição em mm (x_mm, y_mm) para pixel no canvas polar */
static void _radar_mm_to_px(int x_mm, int y_mm,
                             lv_coord_t *px_x, lv_coord_t *px_y)
{
    const int cx_s  = RADAR_W / 2;
    const int cy_s  = RADAR_H - 2;
    const int r_max = RADAR_H - 4;

    float dist_mm = sqrtf((float)x_mm * (float)x_mm +
                          (float)y_mm * (float)y_mm);
    float ang_rad = atan2f((float)x_mm, (float)y_mm);
    float radius  = (dist_mm / (float)RADAR_MAX_MM) * (float)r_max;
    if (radius > (float)r_max) radius = (float)r_max;

    *px_x = (lv_coord_t)(cx_s + radius * sinf(ang_rad));
    *px_y = (lv_coord_t)(cy_s - radius * cosf(ang_rad));

    if (*px_x < 0)        *px_x = 0;
    if (*px_x >= RADAR_W) *px_x = RADAR_W - 1;
    if (*px_y < 0)        *px_y = 0;
    if (*px_y >= RADAR_H) *px_y = RADAR_H - 1;
}


/* ============================================================
   FONTE BITMAP 4×6 — dígitos 0-9 + 'k','m','/','h'
   Cada glifo: 6 bytes, bit7=coluna esquerda
============================================================ */
static const uint8_t _font4x6[][6] = {
    /* 0 */ {0x69, 0x99, 0x96, 0x00, 0x00, 0x00},
    /* 1 */ {0x26, 0x22, 0x27, 0x00, 0x00, 0x00},
    /* 2 */ {0x69, 0x12, 0x4F, 0x00, 0x00, 0x00},
    /* 3 */ {0xF1, 0x21, 0x1F, 0x00, 0x00, 0x00},
    /* 4 */ {0x99, 0xF1, 0x11, 0x00, 0x00, 0x00},
    /* 5 */ {0xF8, 0xE1, 0x1E, 0x00, 0x00, 0x00},
    /* 6 */ {0x69, 0x8E, 0x96, 0x00, 0x00, 0x00},
    /* 7 */ {0xF1, 0x12, 0x24, 0x00, 0x00, 0x00},
    /* 8 */ {0x69, 0x96, 0x96, 0x00, 0x00, 0x00},
    /* 9 */ {0x69, 0x71, 0x16, 0x00, 0x00, 0x00},
    /* k */ {0x89, 0xAC, 0xA9, 0x00, 0x00, 0x00},
    /* m */ {0x00, 0x6A, 0xA9, 0x00, 0x00, 0x00},
    /* / */ {0x12, 0x24, 0x48, 0x00, 0x00, 0x00},
    /* h */ {0x88, 0xE8, 0x89, 0x00, 0x00, 0x00},
};
#define FC_K  10
#define FC_M  11
#define FC_SL 12
#define FC_H  13

/* Desenha um glifo da fonte bitmap na posição (x0, y0) */
static void _glifo_px(int x0, int y0, int idx, lv_color_t cor)
{
    for (int row = 0; row < 6; row++) {
        uint8_t b = _font4x6[idx][row];
        for (int col = 0; col < 4; col++) {
            if (b & (0x80u >> col))
                _px_blend(x0 + col, y0 + row, cor, 220u);
        }
    }
}

/* Desenha "XXXkm/h" a partir de (x0, y0) */
static void _vel_label_px(int x0, int y0, float speed_kmh, lv_color_t cor)
{
    if (fabsf(speed_kmh) < 1.0f) return;
    int v  = (int)(fabsf(speed_kmh) + 0.5f);
    int cx = x0;
    if (v >= 100) { _glifo_px(cx, y0, v / 100,       cor); cx += 5; }
    if (v >= 10)  { _glifo_px(cx, y0, (v / 10) % 10, cor); cx += 5; }
    _glifo_px(cx, y0, v % 10, cor); cx += 5;
    _glifo_px(cx, y0, FC_K,   cor); cx += 5;
    _glifo_px(cx, y0, FC_M,   cor); cx += 5;
    _glifo_px(cx, y0, FC_SL,  cor); cx += 5;
    _glifo_px(cx, y0, FC_H,   cor);
}


/* ============================================================
   _radar_aplicar_frame
   ──────────────────────────────────────────────────────────
   Recebe os objectos REAIS do tracking_manager e actualiza
   o estado de cada alvo (dm_alvo_t) com a posição real.

   Para cada objecto recebido:
     1. Procura o slot activo mais próximo (nearest-neighbour)
     2. Se match: actualiza posição real e acrescenta ao rasto
     3. Se sem match: cria novo slot

   O rasto é preenchido com a posição ANTERIOR ao update,
   ou seja, apenas com posições que o sensor realmente mediu.

   Alvos sem frame há ALVO_HOLD_MS são marcados inactivos.
============================================================ */
static void _radar_aplicar_frame(const radar_obj_t *objs, uint8_t count)
{
    uint32_t agora = (uint32_t)(esp_timer_get_time() / 1000ULL);

    /* Marca quais slots já foram emparelhados neste frame */
    bool usado[RADAR_MAX_OBJ] = {false};

    for (int n = 0; n < count && n < RADAR_MAX_OBJ; n++) {
        const radar_obj_t *novo = &objs[n];
        float nx = (float)novo->x_mm;
        float ny = (float)novo->y_mm;

        /* Procura o slot activo mais próximo */
        int   best_idx  = -1;
        float best_dist = 2000.0f;  /* Raio de associação em mm */

        for (int s = 0; s < RADAR_MAX_OBJ; s++) {
            if (!s_alvos[s].activo || usado[s]) continue;
            float dx   = s_alvos[s].x_mm - nx;
            float dy   = s_alvos[s].y_mm - ny;
            float dist = sqrtf(dx*dx + dy*dy);
            if (dist < best_dist) {
                best_dist = dist;
                best_idx  = s;
            }
        }

        if (best_idx >= 0) {
            /* Match — guarda posição anterior no rasto antes de actualizar */
            dm_alvo_t *a = &s_alvos[best_idx];
            a->trail_x[a->trail_head] = a->x_mm;
            a->trail_y[a->trail_head] = a->y_mm;
            a->trail_head = (a->trail_head + 1) % RADAR_TRAIL_MAX;
            if (a->trail_len < RADAR_TRAIL_MAX) a->trail_len++;

            /* Actualiza para a posição real do frame actual */
            a->x_mm     = nx;
            a->y_mm     = ny;
            a->speed_kmh = novo->speed_kmh;
            a->ultimo_ms = agora;
            usado[best_idx] = true;
        } else {
            /* Sem match — inicializa novo slot com posição real */
            for (int s = 0; s < RADAR_MAX_OBJ; s++) {
                if (!s_alvos[s].activo && !usado[s]) {
                    s_alvos[s].x_mm      = nx;
                    s_alvos[s].y_mm      = ny;
                    s_alvos[s].speed_kmh = novo->speed_kmh;
                    s_alvos[s].activo    = true;
                    s_alvos[s].ultimo_ms = agora;
                    s_alvos[s].trail_len  = 0;
                    s_alvos[s].trail_head = 0;
                    usado[s] = true;
                    break;
                }
            }
        }
    }

    /* Apaga alvos sem frame real há mais de ALVO_HOLD_MS */
    for (int s = 0; s < RADAR_MAX_OBJ; s++) {
        if (!s_alvos[s].activo) continue;
        if ((agora - s_alvos[s].ultimo_ms) > ALVO_HOLD_MS) {
            s_alvos[s].activo    = false;
            s_alvos[s].trail_len = 0;
            s_alvos[s].trail_head = 0;
        }
    }
}


/* ============================================================
   _radar_redraw — redesenha o canvas com posições reais
   ──────────────────────────────────────────────────────────
   Chamado a cada 20ms pela display_task.
   Mostra apenas posições reais vindas do tracking_manager.
   Não avança nenhuma posição — o que é desenhado é o que
   o sensor mediu no último frame válido.
============================================================ */
static void _radar_redraw(void)
{
    if (!canvas_radar) return;

    /* Cores do canvas */
    lv_color_t C_FUNDO  = lv_color_hex(0x010601);
    lv_color_t C_ARCO   = lv_color_hex(0x28A745);
    lv_color_t C_EIXO   = lv_color_hex(0x28A745);
    lv_color_t C_SWEEP  = lv_color_hex(0x22C55E);
    lv_color_t C_VRD    = lv_color_hex(0x22C55E);
    lv_color_t C_VRD_HL = lv_color_hex(0xAAFFAA);

    /* Cores e rastos dos alvos (até 3) */
    static const uint32_t COR_ALVO[3]    = {0xFF3333, 0x00D4FF, 0xF5C542};
    static const uint32_t COR_ALVO_HL[3] = {0xFFAAAA, 0xAAEEFF, 0xFFEEAA};
    static const uint8_t  RASTO_R[3]     = {220,   0, 220};
    static const uint8_t  RASTO_G[3]     = {  0, 180, 180};
    static const uint8_t  RASTO_B[3]     = {  0, 220,   0};

    const int cx_s  = RADAR_W / 2;
    const int cy_s  = RADAR_H - 2;
    const int r_max = RADAR_H - 4;

    /* 1. Limpeza do fundo */
    for (int i = 0; i < RADAR_W * RADAR_H; i++)
        radar_buf[i] = C_FUNDO;

    /* 2. Arcos semicirculares de 1m em 1m (até 6m) */
    for (int mm = 1000; mm <= 6000; mm += 1000) {
        int r = (int)((float)mm / (float)RADAR_MAX_MM * (float)r_max);
        uint8_t opa;
        if (mm == 6000)              opa = 100u;
        else if ((mm / 1000) % 2 == 0) opa = 70u;
        else                         opa = 35u;

        for (int dx = -r; dx <= r; dx++) {
            int dy2 = r*r - dx*dx;
            if (dy2 < 0) continue;
            int dy = (int)sqrtf((float)dy2);
            _px_blend(cx_s + dx, cy_s - dy, C_ARCO, opa);
        }
    }

    /* 3. Raios guia: ±30°, ±60°, centro (0°) */
    const float guia_ang[] = {
        -60.0f * 3.14159f / 180.0f,
        -30.0f * 3.14159f / 180.0f,
         0.0f,
         30.0f * 3.14159f / 180.0f,
         60.0f * 3.14159f / 180.0f,
    };
    for (int ri = 0; ri < 5; ri++) {
        float ang = guia_ang[ri];
        float fdx = sinf(ang);
        float fdy = -cosf(ang);
        uint8_t opa = (ri == 2) ? 70u : 35u;
        for (float t = 2.0f; t < (float)r_max; t += 1.0f) {
            int x = cx_s + (int)(t * fdx);
            int y = cy_s + (int)(t * fdy);
            if ((unsigned)x >= RADAR_W || (unsigned)y >= RADAR_H) break;
            _px_blend(x, y, C_EIXO, opa);
        }
    }

    /* 4. Linha de base do sensor */
    _linha_h_px(RADAR_H - 1, C_EIXO);
    _linha_h_px(RADAR_H - 2, lv_color_hex(0x0A1A0A));

    /* 5. Sweep animado polar (estética radar) */
    static uint16_t s_sweep_tick = 0;
    s_sweep_tick = (uint16_t)((s_sweep_tick + 2u) % 180u);

    float sw_s  = ((float)s_sweep_tick * 3.14159f / 180.0f) - 3.14159f / 2.0f;
    float sw_dx = sinf(sw_s);
    float sw_dy = -cosf(sw_s);

    /* Rastilho de fade do sweep */
    for (int back = 1; back <= 30; back++) {
        int   bd   = ((int)s_sweep_tick - back + 360) % 180;
        float bd_s = ((float)bd * 3.14159f / 180.0f) - 3.14159f / 2.0f;
        float bdx  = sinf(bd_s);
        float bdy  = -cosf(bd_s);
        uint8_t fa = (uint8_t)(38 - back);
        if (fa == 0) break;
        for (float t = 2.0f; t < (float)r_max; t += 1.5f) {
            int x = cx_s + (int)(t * bdx);
            int y = cy_s + (int)(t * bdy);
            if ((unsigned)x >= RADAR_W || (unsigned)y >= RADAR_H) break;
            _px_blend(x, y, C_SWEEP, fa);
        }
    }
    /* Linha principal do sweep */
    for (float t = 2.0f; t < (float)r_max; t += 1.0f) {
        int x = cx_s + (int)(t * sw_dx);
        int y = cy_s + (int)(t * sw_dy);
        if ((unsigned)x >= RADAR_W || (unsigned)y >= RADAR_H) break;
        uint8_t fade = (uint8_t)(210u - (uint8_t)(t * 2.0f));
        if (fade < 50u) fade = 50u;
        _px_blend(x, y, C_SWEEP, fade);
    }

    /* 6. Alvos — posições REAIS do tracking_manager */
    for (int i = 0; i < RADAR_MAX_OBJ; i++) {
        dm_alvo_t *alvo = &s_alvos[i];
        if (!alvo->activo) continue;

        lv_color_t C_ALV    = lv_color_hex(COR_ALVO[i]);
        lv_color_t C_ALV_HL = lv_color_hex(COR_ALVO_HL[i]);

        /* Converte posição real para pixel */
        lv_coord_t px, py;
        _radar_mm_to_px((int)alvo->x_mm, (int)alvo->y_mm, &px, &py);

        /* 6a. Rasto — posições reais anteriores (do mais antigo ao mais recente) */
        if (alvo->trail_len > 0) {
            int len = alvo->trail_len;
            for (int t = 0; t < len; t++) {
                /* O buffer circular: head aponta para próxima escrita.
                   (head - len + t) = ponto mais antigo + t */
                int idx = ((int)alvo->trail_head - len + t
                           + RADAR_TRAIL_MAX) % RADAR_TRAIL_MAX;
                lv_coord_t tx, ty;
                _radar_mm_to_px((int)alvo->trail_x[idx],
                                (int)alvo->trail_y[idx], &tx, &ty);
                /* Raio e alpha crescem do mais antigo para o mais recente */
                int     r_tr = 1 + (t * 2) / (len + 1);
                uint8_t a_tr = (uint8_t)(20u + (uint32_t)t * 80u /
                                         (uint32_t)(len + 1));
                lv_color_t c_tr = lv_color_make(
                    (uint8_t)((uint32_t)RASTO_R[i] * a_tr >> 8),
                    (uint8_t)((uint32_t)RASTO_G[i] * a_tr >> 8),
                    (uint8_t)((uint32_t)RASTO_B[i] * a_tr >> 8));
                _circulo_px((int)tx, (int)ty, r_tr, c_tr);
            }
        }

        /* 6b. Halo duplo em torno do ponto */
        _halo_px((int)px, (int)py, 5, 10, C_ALV, 50u);
        _halo_px((int)px, (int)py, 3,  6, C_ALV, 90u);

        /* 6c. Ponto principal */
        _circulo_px((int)px, (int)py, 3, C_ALV);
        _px_blend((int)px - 1, (int)py - 1, C_ALV_HL, 210u);
        _px_blend((int)px,     (int)py - 1, C_ALV_HL, 160u);
        _px_blend((int)px - 1, (int)py,     C_ALV_HL, 110u);

        /* 6d. Label de velocidade junto ao ponto */
        if (fabsf(alvo->speed_kmh) > 0.5f) {
            int v     = (int)(fabsf(alvo->speed_kmh) + 0.5f);
            int n_dig = (v >= 100) ? 3 : (v >= 10 ? 2 : 1);
            int lbl_w = (n_dig + 4) * 5;
            int lx    = (int)px + 12;
            int ly    = (int)py - 10;

            /* Clamp para não sair do canvas */
            if (lx + lbl_w > RADAR_W - 2) lx = (int)px - lbl_w - 4;
            if (lx < 1)                   lx = 1;
            if (ly < 1)                   ly = 1;
            if (ly > RADAR_H - 8)         ly = RADAR_H - 8;

            _vel_label_px(lx, ly, alvo->speed_kmh, lv_color_hex(COR_ALVO[i]));
        }
    }

    /* 7. Ponto do sensor — centro verde da base */
    _halo_px(cx_s, cy_s, 4, 9, C_VRD, 55u);
    _circulo_px(cx_s, cy_s, 3, C_VRD);
    _px_blend(cx_s - 1, cy_s - 1, C_VRD_HL, 210u);
    _px_blend(cx_s,     cy_s - 1, C_VRD_HL, 150u);

    /* 8. Indicador de tráfego em trânsito
       ─────────────────────────────────────────────────────────
       Quando há veículos contabilizados (T ou Tc > 0) mas o
       radar local não vê nenhum objecto no canvas, mostra
       indicadores visuais nas bordas do canvas:

       T > 0 sem objecto visível:
         Seta amarela no canto DIREITO do canvas → o objecto
         saiu deste poste e está a caminho do seguinte.
         A luz deste poste mantém-se acesa (T=1) até o poste
         seguinte confirmar a chegada.

       Tc > 0 sem objecto visível:
         Seta ciano no canto ESQUERDO do canvas → há um objecto
         a caminho deste poste, anunciado via UDP pelo anterior.

       Quando há objectos visíveis no canvas, os indicadores
       não são necessários — o objecto fala por si.
    ────────────────────────────────────────────────────────── */
    {
        /* Conta alvos actualmente visíveis no canvas */
        int alvos_visiveis = 0;
        for (int i = 0; i < RADAR_MAX_OBJ; i++)
            if (s_alvos[i].activo) alvos_visiveis++;

        /* Tick de piscar — alterna a cada ~10 frames (200ms a 50Hz) */
        static uint16_t s_blink_tick = 0;
        s_blink_tick++;
        bool blink_on = (s_blink_tick % 20) < 10;  /* 50% duty cycle */

        /* T > 0 e sem objecto visível → seta amarela à direita
           Indica: objecto saiu, está entre este poste e o seguinte */
        if (s_T_actual > 0 && alvos_visiveis == 0 && blink_on) {
            lv_color_t C_T = lv_color_hex(0xFF8C00);  /* Amarelo */
            /* Seta a apontar para a direita (→) na borda direita */
            int sx = RADAR_W - 8;
            int sy = RADAR_H / 2;
            /* Corpo da seta */
            for (int i = -3; i <= 3; i++)
                _px_blend(sx - 6 + i, sy, C_T, 200u);
            /* Ponta da seta */
            _px_blend(sx, sy - 1, C_T, 200u);
            _px_blend(sx, sy + 1, C_T, 200u);
            _px_blend(sx - 1, sy - 2, C_T, 160u);
            _px_blend(sx - 1, sy + 2, C_T, 160u);
            /* Label "T" em bitmap */
            char t_str[4];
            snprintf(t_str, sizeof(t_str), "%d", s_T_actual);
            int digit = t_str[0] - '0';
            if (digit >= 0 && digit <= 9)
                _glifo_px(sx - 14, sy - 3, digit, C_T);
        }

        /* Tc > 0 e sem objecto visível → seta ciano à esquerda
           Indica: objecto a caminho deste poste, anunciado via UDP */
        if (s_Tc_actual > 0 && alvos_visiveis == 0 && blink_on) {
            lv_color_t C_Tc = lv_color_hex(0x00D4FF);  /* Ciano */
            /* Seta a apontar para a direita (→) na borda esquerda
               (vem da esquerda, entra pela esquerda do canvas) */
            int sx = 8;
            int sy = RADAR_H / 2;
            /* Corpo da seta */
            for (int i = -3; i <= 3; i++)
                _px_blend(sx + i, sy, C_Tc, 200u);
            /* Ponta da seta */
            _px_blend(sx + 4, sy - 1, C_Tc, 200u);
            _px_blend(sx + 4, sy + 1, C_Tc, 200u);
            _px_blend(sx + 5, sy - 2, C_Tc, 160u);
            _px_blend(sx + 5, sy + 2, C_Tc, 160u);
            /* Label "Tc" */
            char tc_str[4];
            snprintf(tc_str, sizeof(tc_str), "%d", s_Tc_actual);
            int digit = tc_str[0] - '0';
            if (digit >= 0 && digit <= 9)
                _glifo_px(sx + 8, sy - 3, digit, C_Tc);
        }
    }

    lv_obj_invalidate(canvas_radar);
}


/* ============================================================
   ui_create — constrói a interface LVGL
   ──────────────────────────────────────────────────────────
   Layout fixo de 4 zonas verticais:
     IDENTIDADE (0-35) | HARDWARE (37-92) | TRÁFEGO (94-145) | RADAR (147-239)
============================================================ */
static void ui_create(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(COR_PRETO), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    /* ── ZONA IDENTIDADE (y: 0..35) ─────────────────────────── */
    /* Nome do poste e badge de estado FSM */
    label_nome = lv_label_create(scr);
    lv_label_set_text_fmt(label_nome, "> %s", post_get_name());
    lv_obj_set_style_text_color(label_nome, lv_color_hex(COR_BRANCO), 0);
    lv_obj_set_style_text_font(label_nome, &lv_font_montserrat_14, 0);
    lv_obj_align(label_nome, LV_ALIGN_TOP_LEFT, 8, 15);

    /* Badge de estado FSM — canto superior direito */
    label_badge = lv_label_create(scr);
    lv_label_set_text(label_badge, "IDLE");
    lv_obj_set_style_text_color(label_badge, lv_color_hex(COR_CINZENTO), 0);
    lv_obj_set_style_text_font(label_badge, &lv_font_montserrat_14, 0);
    lv_obj_set_style_bg_color(label_badge, lv_color_hex(0x1C1C1C), 0);
    lv_obj_set_style_bg_opa(label_badge, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(label_badge, 5, 0);
    lv_obj_set_style_pad_hor(label_badge, 7, 0);
    lv_obj_set_style_pad_ver(label_badge, 3, 0);
    lv_obj_set_style_border_color(label_badge, lv_color_hex(COR_SEPARADOR), 0);
    lv_obj_set_style_border_width(label_badge, 1, 0);
    lv_obj_align(label_badge, LV_ALIGN_TOP_RIGHT, -8, 10);

    _separador(scr, 36);

    /* ── ZONA HARDWARE (y: 37..92) ──────────────────────────── */
    /* WiFi, estado radar, vizinhos, barra DALI */
    label_wifi     = _label_novo(scr,   8, 39, COR_CINZENTO, "WiFi: ---");
    label_radar_st = _label_novo(scr, 130, 39, COR_CINZENTO, "Radar: ---");

    /* Linha de vizinhos */
    lv_obj_t *div_neb = lv_obj_create(scr);
    lv_obj_set_size(div_neb, LCD_H_RES - 16, 16);
    lv_obj_set_pos(div_neb, 8, 55);
    lv_obj_set_style_bg_opa(div_neb, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(div_neb, 0, 0);
    lv_obj_set_style_pad_all(div_neb, 0, 0);

    label_neb_esq = _label_novo(scr,   8, 57, COR_CINZ_CLARO, "E: ---");
    label_neb_dir = _label_novo(scr, LCD_H_RES/2+4, 57, COR_CINZ_CLARO, "D: ---");
    label_dali    = _label_novo(scr,   8, 76, COR_CINZENTO,   "DALI:  0%");

    /* Barra de brilho DALI */
    bar_dali = lv_bar_create(scr);
    lv_obj_set_size(bar_dali, 88, 7);
    lv_obj_set_pos(bar_dali, 144, 78);
    lv_bar_set_range(bar_dali, 0, 100);
    lv_bar_set_value(bar_dali, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar_dali, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar_dali, lv_color_hex(COR_AMARELO), LV_PART_INDICATOR);
    lv_obj_set_style_radius(bar_dali, 3, LV_PART_MAIN);
    lv_obj_set_style_radius(bar_dali, 3, LV_PART_INDICATOR);

    _separador(scr, 93);

    /* ── ZONA TRÁFEGO (y: 94..145) ──────────────────────────── */
    /* 3 cards: T (aqui), Tc (a caminho), km/h (velocidade) */
    const int CW  = 70, CH = 56, CY = 95, GAP = 5, CX0 = 5;
    const int CX1 = CX0 + CW + GAP;
    const int CX2 = CX1 + CW + GAP;

    const uint32_t COR_CARD[3]  = {COR_AMARELO, COR_CIANO, COR_VIOLETA};
    const char *TITULO_CARD[3]  = {"T", "Tc", "km/h"};
    const char *UNIDADE_CARD[3] = {"carros", "carros", "km/h"};
    const int   CX_ARR[3]       = {CX0, CX1, CX2};
    lv_obj_t  **VAL_LABELS[3]   = {&label_T_val, &label_Tc_val, &label_vel_val};
    lv_obj_t  **CARDS[3]        = {&card_T, &card_Tc, &card_vel};

    for (int i = 0; i < 3; i++) {
        /* Corpo do card */
        lv_obj_t *card = lv_obj_create(scr);
        lv_obj_set_size(card, CW, CH);
        lv_obj_set_pos(card, CX_ARR[i], CY);
        lv_obj_set_style_bg_color(card, lv_color_hex(COR_FUNDO_CARD), 0);
        lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(card, lv_color_hex(COR_SEPARADOR), 0);
        lv_obj_set_style_border_width(card, 1, 0);
        lv_obj_set_style_radius(card, 5, 0);
        lv_obj_set_style_pad_all(card, 0, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
        *CARDS[i] = card;

        /* Header colorido */
        lv_obj_t *hdr = lv_obj_create(card);
        lv_obj_set_size(hdr, CW, 14);
        lv_obj_set_pos(hdr, 0, 0);
        lv_obj_set_style_bg_color(hdr, lv_color_hex(COR_CARD[i]), 0);
        lv_obj_set_style_bg_opa(hdr, 45, 0);
        lv_obj_set_style_radius(hdr, 0, 0);
        lv_obj_set_style_border_width(hdr, 0, 0);
        lv_obj_set_style_pad_all(hdr, 0, 0);

        lv_obj_t *lbl_tit = lv_label_create(hdr);
        lv_label_set_text(lbl_tit, TITULO_CARD[i]);
        lv_obj_set_style_text_font(lbl_tit, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lbl_tit, lv_color_hex(COR_CARD[i]), 0);
        lv_obj_center(lbl_tit);

        /* Valor numérico grande */
        *VAL_LABELS[i] = lv_label_create(card);
        lv_label_set_text(*VAL_LABELS[i], "0");
        lv_obj_set_style_text_color(*VAL_LABELS[i], lv_color_hex(COR_CINZENTO), 0);
        lv_obj_set_style_text_font(*VAL_LABELS[i], &lv_font_montserrat_14, 0);
        lv_obj_align(*VAL_LABELS[i], LV_ALIGN_TOP_MID, 0, 18);

        /* Unidade por baixo */
        lv_obj_t *lbl_unit = lv_label_create(card);
        lv_label_set_text(lbl_unit, UNIDADE_CARD[i]);
        lv_obj_set_style_text_font(lbl_unit, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lbl_unit, lv_color_hex(COR_CINZ_CLARO), 0);
        lv_obj_align(lbl_unit, LV_ALIGN_BOTTOM_MID, 0, -3);
    }

    _separador(scr, 146);

    /* ── ZONA RADAR (y: 147..239) ───────────────────────────── */
    /* Canvas 230×90px — representação polar do sensor HLK-LD2450 */
    lv_obj_t *lbl_radar_tit = lv_label_create(scr);
    lv_label_set_text(lbl_radar_tit, "HLK-LD2450  radar");
    lv_obj_set_style_text_color(lbl_radar_tit, lv_color_hex(COR_SEPARADOR), 0);
    lv_obj_set_style_text_font(lbl_radar_tit, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(lbl_radar_tit, RADAR_X_OFF, 148);

    canvas_radar = lv_canvas_create(scr);
    lv_canvas_set_buffer(canvas_radar, radar_buf,
                         RADAR_W, RADAR_H, LV_IMG_CF_TRUE_COLOR);
    lv_obj_set_pos(canvas_radar, RADAR_X_OFF, RADAR_Y_OFF);
    lv_obj_set_style_border_color(canvas_radar, lv_color_hex(COR_SEPARADOR), 0);
    lv_obj_set_style_border_width(canvas_radar, 1, 0);
    lv_obj_set_style_radius(canvas_radar, 3, 0);

    _radar_redraw();

    ESP_LOGI(TAG, "Interface v6.0 criada | ID=%d | %s",
             post_get_id(), post_get_name());
}


/* ============================================================
   API PÚBLICA — CICLO DE VIDA
============================================================ */

void display_manager_init(void)
{
    ESP_LOGI(TAG, "A inicializar display v6.0 | ID=%d | %s",
             post_get_id(), post_get_name());

    st7789_init();
    lv_init();

    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t         buf[LCD_H_RES * 10];
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LCD_H_RES * 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res  = LCD_H_RES;
    disp_drv.ver_res  = LCD_V_RES;
    disp_drv.flush_cb = st7789_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* Inicializa estado dos alvos e contadores de tráfego a zero */
    memset(s_alvos, 0, sizeof(s_alvos));
    s_T_actual  = 0;
    s_Tc_actual = 0;

    s_fila = xQueueCreate(DM_FILA_CAP, sizeof(dm_msg_t));
    if (!s_fila)
        ESP_LOGE(TAG, "Falha ao criar fila de mensagens!");

    ui_create();

    ESP_LOGI(TAG, "Display v6.0 pronto | posições reais activas");
}

void display_manager_tick(uint32_t ms)
{
    lv_tick_inc(ms);
}


/* ============================================================
   display_manager_reset_radar — limpa estado dos alvos
   Chamar quando radar reinicia ou perde tracking total.
   Thread-safe: s_alvos é lido/escrito apenas na display_task.
============================================================ */
void display_manager_reset_radar(void)
{
    memset(s_alvos, 0, sizeof(s_alvos));
    s_T_actual  = 0;
    s_Tc_actual = 0;
    ESP_LOGI(TAG, "Estado radar limpo");
}


/* ============================================================
   display_manager_task — loop de render (20ms / 50Hz)
   ──────────────────────────────────────────────────────────
   ÚNICA função que toca em objectos LVGL.
   Drena fila de mensagens, redesenha canvas e chama
   lv_timer_handler(). Chamada exclusivamente pela display_task.
============================================================ */
void display_manager_task(void)
{
    if (!s_fila) {
        lv_timer_handler();
        return;
    }

    dm_msg_t msg;
    int msg_count = 0;

    /* Processa até 5 mensagens por ciclo para não bloquear o render */
    while (xQueueReceive(s_fila, &msg, 0) == pdTRUE && msg_count < 5) {
        msg_count++;

        switch (msg.tipo) {

            case DM_MSG_STATUS:
                if (!label_badge) break;
                lv_label_set_text(label_badge, msg.st.status);
                {
                    uint32_t cor_txt = COR_CINZENTO;
                    uint32_t cor_bg  = 0x1C1C1C;
                    uint32_t cor_brd = COR_SEPARADOR;
                    const char *s = msg.st.status;
                    if      (strcmp(s, "LIGHT ON")  == 0) {
                        cor_txt = COR_AMARELO; cor_bg = 0x1E1600; cor_brd = 0x3A2A00;
                    } else if (strcmp(s, "SAFE MODE") == 0) {
                        cor_txt = COR_LARANJA; cor_bg = 0x1E0E00; cor_brd = 0x3A1A00;
                    } else if (strcmp(s, "MASTER")    == 0) {
                        cor_txt = COR_VERDE;   cor_bg = 0x001A08; cor_brd = 0x003A10;
                    } else if (strcmp(s, "AUTONOMO")  == 0) {
                        cor_txt = COR_VERMELHO; cor_bg = 0x1A0000; cor_brd = 0x3A0000;
                    }
                    lv_obj_set_style_text_color(label_badge, lv_color_hex(cor_txt), 0);
                    lv_obj_set_style_bg_color(label_badge, lv_color_hex(cor_bg), 0);
                    lv_obj_set_style_border_color(label_badge, lv_color_hex(cor_brd), 0);
                }
                break;

            case DM_MSG_WIFI:
                if (!label_wifi) break;
                if (msg.wifi.connected) {
                    char buf[36];
                    snprintf(buf, sizeof(buf), "WiFi: %s",
                             msg.wifi.ip[0] ? msg.wifi.ip : "---");
                    lv_label_set_text(label_wifi, buf);
                    lv_obj_set_style_text_color(label_wifi,
                        lv_color_hex(COR_VERDE), 0);
                } else {
                    lv_label_set_text(label_wifi, "WiFi: OFF");
                    lv_obj_set_style_text_color(label_wifi,
                        lv_color_hex(COR_VERMELHO), 0);
                }
                break;

            case DM_MSG_NEIGHBORS:
                if (label_neb_esq) {
                    char buf[32];
                    if (msg.neb.nebL[0] && strcmp(msg.neb.nebL, "---") != 0) {
                        snprintf(buf, sizeof(buf), "E: %s", msg.neb.nebL);
                        lv_obj_set_style_text_color(label_neb_esq,
                            lv_color_hex(msg.neb.leftOk ? COR_VERDE : COR_VERMELHO), 0);
                    } else {
                        snprintf(buf, sizeof(buf), "E: ---");
                        lv_obj_set_style_text_color(label_neb_esq,
                            lv_color_hex(COR_CINZ_CLARO), 0);
                    }
                    lv_label_set_text(label_neb_esq, buf);
                }
                if (label_neb_dir) {
                    char buf[32];
                    if (msg.neb.nebR[0] && strcmp(msg.neb.nebR, "---") != 0) {
                        snprintf(buf, sizeof(buf), "D: %s", msg.neb.nebR);
                        lv_obj_set_style_text_color(label_neb_dir,
                            lv_color_hex(msg.neb.rightOk ? COR_VERDE : COR_VERMELHO), 0);
                    } else {
                        snprintf(buf, sizeof(buf), "D: ---");
                        lv_obj_set_style_text_color(label_neb_dir,
                            lv_color_hex(COR_CINZ_CLARO), 0);
                    }
                    lv_label_set_text(label_neb_dir, buf);
                }
                break;

            case DM_MSG_HARDWARE:
                if (label_radar_st) {
                    char rbuf[20];
                    snprintf(rbuf, sizeof(rbuf), "Radar: %s", msg.hw.radar_st);
                    lv_label_set_text(label_radar_st, rbuf);
                    uint32_t cor = COR_VERMELHO;
                    if (msg.hw.radar_ok)
                        cor = (msg.hw.radar_st[0] == 'S') ? 0xFFAA00 : COR_VERDE;
                    lv_obj_set_style_text_color(label_radar_st,
                        lv_color_hex(cor), 0);
                }
                if (label_dali) {
                    char buf[14];
                    snprintf(buf, sizeof(buf), "DALI: %3d%%", msg.hw.brightness);
                    lv_label_set_text(label_dali, buf);
                    lv_obj_set_style_text_color(label_dali,
                        lv_color_hex(msg.hw.brightness > 10
                                     ? COR_AMARELO : COR_CINZENTO), 0);
                }
                if (bar_dali)
                    lv_bar_set_value(bar_dali, (int)msg.hw.brightness, LV_ANIM_ON);
                break;

            case DM_MSG_TRAFFIC:
                /* Guarda T e Tc para o indicador de trânsito no canvas */
                s_T_actual  = msg.traf.T;
                s_Tc_actual = msg.traf.Tc;
                if (label_T_val) {
                    char buf[8];
                    snprintf(buf, sizeof(buf), "%d", msg.traf.T);
                    lv_label_set_text(label_T_val, buf);
                    lv_obj_set_style_text_color(label_T_val,
                        lv_color_hex(msg.traf.T > 0 ? COR_AMARELO : COR_CINZENTO), 0);
                }
                if (label_Tc_val) {
                    char buf[8];
                    snprintf(buf, sizeof(buf), "%d", msg.traf.Tc);
                    lv_label_set_text(label_Tc_val, buf);
                    lv_obj_set_style_text_color(label_Tc_val,
                        lv_color_hex(msg.traf.Tc > 0 ? COR_CIANO : COR_CINZENTO), 0);
                }
                break;

            case DM_MSG_SPEED:
                if (label_vel_val) {
                    char buf[8];
                    snprintf(buf, sizeof(buf), "%d", msg.spd.speed);
                    lv_label_set_text(label_vel_val, buf);
                    lv_obj_set_style_text_color(label_vel_val,
                        lv_color_hex(msg.spd.speed > 0 ? COR_VIOLETA : COR_CINZENTO), 0);
                }
                break;

            case DM_MSG_RADAR:
                /* Aplica frame REAL — sem simulação de posição */
                if (msg.radar.count > 0) {
                    _radar_aplicar_frame(msg.radar.objs, msg.radar.count);
                } else {
                    /* count=0 → tracking não tem nenhum alvo activo neste ciclo.
                       Limpa canvas imediatamente — sem esperar timeout.
                       Evita que um obstáculo parado fique eternamente
                       no ecrã quando outros veículos passam por cima. */
                    for (int s = 0; s < RADAR_MAX_OBJ; s++) {
                        s_alvos[s].activo    = false;
                        s_alvos[s].trail_len = 0;
                        s_alvos[s].trail_head = 0;
                    }
                }
                break;

            default: break;
        }
    }

    /* Redesenha canvas com posições reais actuais */
    _radar_redraw();

    /* Processa motor gráfico LVGL */
    lv_timer_handler();

    /* Cede tempo ao CPU — evita watchdog */
    vTaskDelay(pdMS_TO_TICKS(10));
}


/* ============================================================
   API PÚBLICA — ACTUALIZAÇÃO DE ESTADO
============================================================ */

void display_manager_set_status(const char *status)
{
    if (!s_fila || !status) return;
    dm_msg_t msg = { .tipo = DM_MSG_STATUS };
    strncpy(msg.st.status, status, sizeof(msg.st.status) - 1);
    xQueueSend(s_fila, &msg, 0);
}

void display_manager_set_leader(bool is_leader)
{
    display_manager_set_status(is_leader ? "MASTER" : "IDLE");
}

void display_manager_set_wifi(bool connected, const char *ip)
{
    if (!s_fila) return;
    dm_msg_t msg = { .tipo = DM_MSG_WIFI };
    msg.wifi.connected = connected;
    if (ip && ip[0])
        strncpy(msg.wifi.ip, ip, sizeof(msg.wifi.ip) - 1);
    else
        msg.wifi.ip[0] = '\0';
    xQueueSend(s_fila, &msg, 0);
}

void display_manager_set_hardware(const char *radar_st,
                                   bool        radar_ok,
                                   uint8_t     brightness)
{
    if (!s_fila) return;
    dm_msg_t msg = { .tipo = DM_MSG_HARDWARE };
    strncpy(msg.hw.radar_st,
            radar_st ? radar_st : "---",
            sizeof(msg.hw.radar_st) - 1);
    msg.hw.radar_st[sizeof(msg.hw.radar_st) - 1] = '\0';
    msg.hw.radar_ok   = radar_ok;
    msg.hw.brightness = brightness;
    xQueueSend(s_fila, &msg, 0);
}

void display_manager_set_traffic(int T, int Tc)
{
    if (!s_fila) return;
    dm_msg_t msg = { .tipo = DM_MSG_TRAFFIC };
    msg.traf.T  = T;
    msg.traf.Tc = Tc;
    xQueueSend(s_fila, &msg, 0);
}

void display_manager_set_speed(int speed)
{
    if (!s_fila) return;
    dm_msg_t msg = { .tipo = DM_MSG_SPEED };
    msg.spd.speed = speed;
    xQueueSend(s_fila, &msg, 0);
}

void display_manager_set_neighbors(const char *nebL, const char *nebR,
                                   bool leftOk, bool rightOk)
{
    if (!s_fila) return;
    dm_msg_t msg = { .tipo = DM_MSG_NEIGHBORS };
    if (nebL && nebL[0])
        strncpy(msg.neb.nebL, nebL, sizeof(msg.neb.nebL) - 1);
    else
        msg.neb.nebL[0] = '\0';
    if (nebR && nebR[0])
        strncpy(msg.neb.nebR, nebR, sizeof(msg.neb.nebR) - 1);
    else
        msg.neb.nebR[0] = '\0';
    msg.neb.leftOk  = leftOk;
    msg.neb.rightOk = rightOk;
    xQueueSend(s_fila, &msg, 0);
}

/* Recebe posições reais do tracking_manager e coloca na fila */
void display_manager_set_radar(const radar_obj_t *objs, uint8_t count)
{
    if (!s_fila) return;
    dm_msg_t msg = { .tipo = DM_MSG_RADAR };
    msg.radar.count = (count > RADAR_MAX_OBJ) ? RADAR_MAX_OBJ : count;
    if (objs && msg.radar.count > 0)
        memcpy(msg.radar.objs, objs, msg.radar.count * sizeof(radar_obj_t));
    /* Se fila cheia, descarta — o próximo frame real substitui */
    xQueueSend(s_fila, &msg, 0);
}


