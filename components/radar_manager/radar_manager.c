/* ============================================================
   RADAR MANAGER — IMPLEMENTAÇÃO
   @file      radar_manager.c
   @version   3.3  |  2026-04-09
   Projecto  : Poste Inteligente v8
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

   Alterações v3.2 → v3.3:
   ─────────────────────────
   - CORRIGIDO: descodificação X, Y e velocidade do HLK-LD2450.
     O sensor usa formato magnitude + bit de sinal (bit 15),
     NÃO complemento a 2. Valores anteriores (~-32000) eram
     artefactos da interpretação errada.
     Formato HLK-LD2450:
       bit 15 = 0 → positivo (magnitude nos bits 0-14)
       bit 15 = 1 → negativo (magnitude nos bits 0-14)
     X: positivo = direita do sensor, negativo = esquerda
     Y: sempre positivo (distância frontal)
     Velocidade: bit 15=1 → a aproximar-se (negativo para FSM)
   - Removido #include <inttypes.h>.
============================================================ */
#include "radar_manager.h"
#include "system_config.h"
#include "hw_config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "RADAR_MGR";

/* ── Frame HLK-LD2450 ─────────────────────────────────────── */
#define FRAME_LEN       30
#define HEADER_0        0xAA
#define HEADER_1        0xFF
#define HEADER_2        0x03
#define HEADER_3        0x00
#define FOOTER_0        0x55
#define FOOTER_1        0xCC
#define UART_BUF_SIZE   256
#define MAX_DIST_MM     (RADAR_MAX_M * 1000 + 500)

/* ── Estado interno ───────────────────────────────────────── */
static radar_mode_t s_mode         = RADAR_MODE_SIMULATED;
static bool         s_last_read_ok = false;
static int          s_no_frame_cnt = 0;

/* Ring buffer UART */
static uint8_t s_ring_buf[FRAME_LEN * 4];
static int     s_ring_len = 0;

/* Cache do último frame — protegida por spinlock */
static radar_data_t      s_last_data  = {0};
static portMUX_TYPE      s_cache_mux  = portMUX_INITIALIZER_UNLOCKED;

/* Contadores de obstáculo estático por slot de alvo */
static uint16_t s_frames_est[MAX_RADAR_TARGETS] = {0};
static int      s_dist_ant[MAX_RADAR_TARGETS]   = {0};

/* ============================================================
   _hlk_decode_signed — descodifica valor HLK-LD2450
   Conforme biblioteca de referência HLK-LD2450.h:
     bit 15 = 1 → valor POSITIVO (magnitude = raw - 0x8000)
     bit 15 = 0 → valor NEGATIVO (magnitude = -raw)
   Para X: positivo = direita, negativo = esquerda
   Para Y: positivo = à frente do sensor
   Para Speed: positivo = a aproximar-se, negativo = a afastar-se
============================================================ */
static int _hlk_decode_signed(uint8_t lo, uint8_t hi)
{
    int16_t raw = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
    if (hi & 0x80)
        return (int)(raw - (int16_t)0x8000);  /* bit 15=1 → positivo */
    else
        return (int)(-raw);                     /* bit 15=0 → negativo */
}

/* ── Rasto visual ─────────────────────────────────────────── */
static void _update_trail(radar_obj_t *obj, int x, int y)
{
    if (obj->trail_len < RADAR_TRAIL_MAX) {
        obj->trail_x[obj->trail_len] = x;
        obj->trail_y[obj->trail_len] = y;
        obj->trail_len++;
    } else {
        memmove(obj->trail_x, obj->trail_x + 1,
                (RADAR_TRAIL_MAX - 1) * sizeof(int));
        memmove(obj->trail_y, obj->trail_y + 1,
                (RADAR_TRAIL_MAX - 1) * sizeof(int));
        obj->trail_x[RADAR_TRAIL_MAX - 1] = x;
        obj->trail_y[RADAR_TRAIL_MAX - 1] = y;
    }
    obj->x_mm = x;
    obj->y_mm = y;
}

/* ============================================================
   radar_init
============================================================ */
void radar_init(radar_mode_t mode)
{
    s_mode = mode;
    if (mode == RADAR_MODE_UART) {
        uart_config_t cfg = {
            .baud_rate  = RADAR_BAUD_RATE,
            .data_bits  = UART_DATA_8_BITS,
            .parity     = UART_PARITY_DISABLE,
            .stop_bits  = UART_STOP_BITS_1,
            .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        ESP_ERROR_CHECK(uart_driver_install(RADAR_UART_PORT,
                        UART_BUF_SIZE * 2, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(RADAR_UART_PORT, &cfg));
        ESP_ERROR_CHECK(uart_set_pin(RADAR_UART_PORT,
                        RADAR_PIN_TX, RADAR_PIN_RX,
                        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        ESP_LOGI(TAG, "Radar UART%d | TX=%d RX=%d | %d baud",
                 RADAR_UART_PORT, RADAR_PIN_TX, RADAR_PIN_RX, RADAR_BAUD_RATE);
    } else {
        ESP_LOGI(TAG, "Radar: modo simulado");
    }
}

/* ============================================================
   radar_flush_rx — limpa backlog UART do arranque
============================================================ */
void radar_flush_rx(void)
{
    if (s_mode == RADAR_MODE_UART) {
        uart_flush_input(RADAR_UART_PORT);
        s_ring_len = 0;
        memset(&s_last_data, 0, sizeof(s_last_data));
        ESP_LOGI(TAG, "UART RX limpa");
    }
}

/* ============================================================
   radar_read_data — lê UART e actualiza cache
   Chamado pela radar_task a cada 100ms.

   CORRIGIDO v3.3: descodificação X/Y/velocidade usa
   _hlk_decode_signed() — formato magnitude + bit de sinal.
============================================================ */
bool radar_read_data(radar_data_t *out_data,
                     radar_simulated_input_t *sim_input)
{
    if (!out_data) return false;
    memset(out_data, 0, sizeof(radar_data_t));

    /* Modo simulado — dados injectados externamente */
    if (s_mode == RADAR_MODE_SIMULATED) {
        if (sim_input && sim_input->active) {
            out_data->count = 1;
            out_data->targets[0].y_mm     = sim_input->distance;
            out_data->targets[0].distance = sim_input->distance / 1000.0f;
            out_data->targets[0].detected = true;
            taskENTER_CRITICAL(&s_cache_mux);
            s_last_data = *out_data;
            taskEXIT_CRITICAL(&s_cache_mux);
            return true;
        }
        return false;
    }

    /* Modo UART — lê bytes do sensor */
    uint8_t tmp[64];
    int len = uart_read_bytes(RADAR_UART_PORT, tmp,
                              sizeof(tmp), pdMS_TO_TICKS(10));
    if (len > 0) {
        /* Gestão de overflow: preserva últimos 29 bytes */
        if (s_ring_len + len > (int)sizeof(s_ring_buf)) {
            int keep = FRAME_LEN - 1;
            memmove(s_ring_buf, s_ring_buf + s_ring_len - keep, keep);
            s_ring_len = keep;
        }
        memcpy(s_ring_buf + s_ring_len, tmp, len);
        s_ring_len += len;
    }

    /* Procura frame válido no ring buffer */
    int found = -1;
    for (int i = 0; i <= s_ring_len - FRAME_LEN; i++) {
        if (s_ring_buf[i]   == HEADER_0 &&
            s_ring_buf[i+1] == HEADER_1 &&
            s_ring_buf[i+2] == HEADER_2 &&
            s_ring_buf[i+3] == HEADER_3 &&
            s_ring_buf[i+FRAME_LEN-2] == FOOTER_0 &&
            s_ring_buf[i+FRAME_LEN-1] == FOOTER_1) {
            found = i;
            break;
        }
    }

    if (found < 0) {
        s_no_frame_cnt++;
        if (s_no_frame_cnt > NO_FRAME_LIMIT) s_last_read_ok = false;
        return false;
    }

    /* Descodifica 3 alvos — formato HLK-LD2450:
     * Cada alvo = 8 bytes: X(2) Y(2) Speed(2) Distance(2)
     * X/Y/Speed: bit 15 = sinal, bits 0-14 = magnitude em mm ou cm/s
     * Resolution: unsigned 16-bit (bytes 6-7) — NÃO é distância.
     * A distância real calcula-se com sqrt(X² + Y²).              */
    uint8_t *f = s_ring_buf + found;
    for (int t = 0; t < MAX_RADAR_TARGETS; t++) {
        uint8_t *b = f + 4 + t * 8;

        /* X, Y, Velocidade: magnitude + bit de sinal (bit 15)
         * CORRIGIDO v3.3: usa _hlk_decode_signed()
         * CORRIGIDO v3.4: distância calculada com sqrt(X²+Y²) */
        int decoded_x   = _hlk_decode_signed(b[0], b[1]);
        int decoded_y   = _hlk_decode_signed(b[2], b[3]);
        int decoded_spd = _hlk_decode_signed(b[4], b[5]);
        uint16_t resolution = (uint16_t)((b[7] << 8) | b[6]);

        /* Alvo vazio: X=0, Y=0, Speed=0 e Resolution=0 */
        if (decoded_x == 0 && decoded_y == 0 && decoded_spd == 0 && resolution == 0)
            continue;

        /* Distância real em mm calculada a partir de X e Y */
        float dx = (float)decoded_x;
        float dy = (float)decoded_y;
        float dist_mm = sqrtf(dx * dx + dy * dy);

        /* Filtra alvos fora do alcance máximo */
        if (dist_mm < 1.0f || dist_mm >= (float)MAX_DIST_MM) continue;

        radar_vehicle_t *tgt = &out_data->targets[out_data->count];
        tgt->x_mm      = decoded_x;
        tgt->y_mm      = decoded_y;
        tgt->distance  = dist_mm / 1000.0f;  /* metros */

        /* Velocidade: sensor reporta em cm/s.
         * Com _hlk_decode_signed():
         *   positivo = a aproximar-se do sensor
         *   negativo = a afastar-se do sensor
         * A FSM usa speed_signed negativo = aproximação.
         * Conversão: cm/s → km/h = * 0.036
         * Invertemos o sinal para manter convenção FSM.       */
        float vel_kmh = (float)decoded_spd * 0.036f;
        tgt->speed_signed = -vel_kmh;  /* invertido: aproximação = negativo */
        tgt->speed = (vel_kmh < 0.0f) ? -vel_kmh : vel_kmh;  /* módulo */

        /* Detector de obstáculo estático */
        int di = (int)dist_mm;
        if (tgt->speed < OBSTACULO_SPEED_MAX_KMH &&
            abs(di - s_dist_ant[t]) < OBSTACULO_DIST_TOL_MM) {
            if (s_frames_est[t] < 0xFFFF) s_frames_est[t]++;
        } else {
            s_frames_est[t] = 0;
        }
        s_dist_ant[t]         = di;
        tgt->frames_estaticos = s_frames_est[t];
        tgt->dist_mm_anterior = s_dist_ant[t];
        tgt->detected         = true;
        out_data->count++;
    }

    /* Consome frame do ring buffer */
    int consumed = found + FRAME_LEN;
    memmove(s_ring_buf, s_ring_buf + consumed, s_ring_len - consumed);
    s_ring_len -= consumed;

    /* Actualiza cache com spinlock — thread-safe */
    taskENTER_CRITICAL(&s_cache_mux);
    s_last_data    = *out_data;
    s_last_read_ok = true;
    s_no_frame_cnt = 0;
    taskEXIT_CRITICAL(&s_cache_mux);

    return true;
}

/* ============================================================
   radar_read_data_cached — lê da cache sem tocar a UART
   Chamado pela fsm_task. Thread-safe via spinlock.
============================================================ */
bool radar_read_data_cached(radar_data_t *out_data)
{
    if (!out_data) return false;
    taskENTER_CRITICAL(&s_cache_mux);
    *out_data = s_last_data;
    taskEXIT_CRITICAL(&s_cache_mux);
    return (out_data->count > 0);
}

/* ============================================================
   radar_manager_get_last_data — exporta último frame completo
============================================================ */
void radar_manager_get_last_data(radar_data_t *out)
{
    if (!out) return;
    taskENTER_CRITICAL(&s_cache_mux);
    *out = s_last_data;
    taskEXIT_CRITICAL(&s_cache_mux);
}

/* ============================================================
   radar_manager_get_objects — objectos com rasto para display
   Lê da cache — não toca UART. Thread-safe via cache_mux.
============================================================ */
uint8_t radar_manager_get_objects(radar_obj_t *objs, uint8_t max)
{
    static radar_obj_t s_trail[MAX_RADAR_TARGETS] = {0};
    radar_data_t data;

    taskENTER_CRITICAL(&s_cache_mux);
    data = s_last_data;
    taskEXIT_CRITICAL(&s_cache_mux);

    if (data.count == 0) {
        for (int i = 0; i < MAX_RADAR_TARGETS; i++) {
            s_trail[i].trail_len = 0;
            s_frames_est[i]      = 0;
            s_dist_ant[i]        = 0;
        }
        return 0;
    }

    uint8_t n = 0;
    for (int i = 0; i < data.count && i < (int)max &&
                    i < MAX_RADAR_TARGETS; i++) {
        _update_trail(&s_trail[i],
                      data.targets[i].x_mm,
                      data.targets[i].y_mm);
        s_trail[i].speed_kmh = data.targets[i].speed;
        objs[n++] = s_trail[i];
    }
    return n;
}

/* ============================================================
   radar_vehicle_in_range — filtra por direcção de movimento
============================================================ */
bool radar_vehicle_in_range(const radar_data_t *data)
{
    if (!data || data->count == 0) return false;
    for (int i = 0; i < data->count; i++)
        if (data->targets[i].speed_signed <= AFASTAR_THRESHOLD_KMH)
            return true;
    return false;
}

/* ============================================================
   radar_get_closest_speed — velocidade do alvo mais próximo
============================================================ */
float radar_get_closest_speed(const radar_data_t *data)
{
    if (!data || data->count == 0) return 0.0f;
    float min_d = 9999.0f, spd = 0.0f;
    for (int i = 0; i < data->count; i++) {
        if (data->targets[i].speed_signed > AFASTAR_THRESHOLD_KMH)
            continue;
        if (data->targets[i].distance < min_d) {
            min_d = data->targets[i].distance;
            spd   = data->targets[i].speed;
        }
    }
    return spd;
}

/* ============================================================
   radar_static_object_present — obstáculo estático ≥ 8s
============================================================ */
bool radar_static_object_present(const radar_data_t *data)
{
    if (!data || data->count == 0) return false;
    for (int i = 0; i < data->count; i++)
        if (data->targets[i].frames_estaticos >= OBSTACULO_MIN_FRAMES)
            return true;
    return false;
}

/* ── Getters de estado ────────────────────────────────────── */
bool radar_is_connected(void)  { return s_last_read_ok; }
radar_mode_t radar_get_mode(void) { return s_mode; }

const char *radar_get_status_str(void)
{
    if (s_mode == RADAR_MODE_SIMULATED) return "SIM";
    return s_last_read_ok ? "REAL" : "FAIL";
}

/* ============================================================
   radar_auto_detect_baud — testa baud rate do HLK-LD2450
============================================================ */
int radar_auto_detect_baud(void)
{
    if (s_mode != RADAR_MODE_UART) return 0;
    const int cand[] = {RADAR_BAUD_RATE};
    const int n = 1;

    for (int c = 0; c < n; c++) {
        uart_set_baudrate(RADAR_UART_PORT, cand[c]);
        uart_flush_input(RADAR_UART_PORT);
        s_ring_len = 0;

        int bytes = 0;
        #define ITER 100
        for (int i = 0; i < ITER; i++) {
            uint8_t tmp[32];
            int r = uart_read_bytes(RADAR_UART_PORT, tmp,
                                    sizeof(tmp), pdMS_TO_TICKS(50));
            if (r > 0) {
                bytes += r;
                if (s_ring_len + r <= (int)sizeof(s_ring_buf)) {
                    memcpy(s_ring_buf + s_ring_len, tmp, r);
                    s_ring_len += r;
                }
                for (int j = 0; j <= s_ring_len - FRAME_LEN; j++) {
                    if (s_ring_buf[j]   == HEADER_0 &&
                        s_ring_buf[j+1] == HEADER_1 &&
                        s_ring_buf[j+2] == HEADER_2 &&
                        s_ring_buf[j+3] == HEADER_3 &&
                        s_ring_buf[j+FRAME_LEN-2] == FOOTER_0 &&
                        s_ring_buf[j+FRAME_LEN-1] == FOOTER_1) {
                        ESP_LOGI(TAG, "Baud detectado: %d", cand[c]);
                        uart_flush_input(RADAR_UART_PORT);
                        s_ring_len = 0;
                        return cand[c];
                    }
                }
            }
        }
        ESP_LOGW(TAG, "Baud %d: %d bytes, sem frame válido", cand[c], bytes);
    }
    ESP_LOGE(TAG, "Auto-detect falhou — fallback %d", RADAR_BAUD_RATE);
    uart_set_baudrate(RADAR_UART_PORT, RADAR_BAUD_RATE);
    uart_flush_input(RADAR_UART_PORT);
    s_ring_len = 0;
    return 0;
}

/* ============================================================
   radar_diagnostic — janela de diagnóstico de 8 segundos
============================================================ */
void radar_diagnostic(void)
{
    if (s_mode != RADAR_MODE_UART) return;
    #define DIAG_S   8
    #define DIAG_MS  100
    ESP_LOGI(TAG, "=== DIAGNÓSTICO RADAR (%ds) ===", DIAG_S);
    int frames_ok = 0, sem_frame = 0;
    for (int i = 0; i < (DIAG_S * 1000 / DIAG_MS); i++) {
        radar_data_t d = {0};
        if (radar_read_data(&d, NULL)) {
            frames_ok++;
            for (int t = 0; t < d.count; t++) {
                const char *dir = (d.targets[t].speed_signed <=
                                   AFASTAR_THRESHOLD_KMH) ? "APROX" : "AFAS";
                ESP_LOGI(TAG, "  Alvo %d | X=%dmm Y=%dmm "
                         "Dist=%.2fm Vel=%.1f km/h [%s]",
                         t+1, d.targets[t].x_mm, d.targets[t].y_mm,
                         d.targets[t].distance, d.targets[t].speed, dir);
            }
        } else {
            if (++sem_frame % 10 == 0)
                ESP_LOGW(TAG, "  %d ciclos sem frame", sem_frame);
        }
        vTaskDelay(pdMS_TO_TICKS(DIAG_MS));
    }
    ESP_LOGI(TAG, "=== FIM DIAGNÓSTICO: %d frames OK ===", frames_ok);
    if (!frames_ok) {
        ESP_LOGE(TAG, "  RADAR FAIL — verificar TX/RX e alimentação 5V");
    }
    uart_flush_input(RADAR_UART_PORT);
    s_ring_len = 0;
    #undef DIAG_S
    #undef DIAG_MS
}
bool radar_get_vehicle_event(radar_data_t *data, float *out_speed)
{
    if (!data || data->count == 0) return false;

    for (int i = 0; i < data->count; i++) {

        float dist = data->targets[i].distance;
        float vel  = data->targets[i].speed;

        if (dist >= RADAR_MIN_DIST_M &&
            vel  >= MIN_DETECT_KMH)
        {
            if (out_speed) *out_speed = vel;
            return true;
        }
    }

    return false;
}