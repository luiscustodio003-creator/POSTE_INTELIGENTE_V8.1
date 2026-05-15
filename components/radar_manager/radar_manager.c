/* ============================================================
   MÓDULO     : radar_manager
   FICHEIRO   : radar_manager.c — Parser HLK-LD2450 e interface de leitura
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)
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

#define FRAME_LEN       30
#define HEADER_0        0xAA
#define HEADER_1        0xFF
#define HEADER_2        0x03
#define HEADER_3        0x00
#define FOOTER_0        0x55
#define FOOTER_1        0xCC
#define UART_BUF_SIZE   256
#define MAX_DIST_MM     (RADAR_MAX_M * 1000 + 500)

static radar_mode_t s_mode         = RADAR_MODE_SIMULATED;
static bool         s_last_read_ok = false;
static int          s_no_frame_cnt = 0;

static uint8_t      s_ring_buf[FRAME_LEN * 4];
static int          s_ring_len = 0;

static radar_data_t s_last_data = {0};
static portMUX_TYPE s_cache_mux = portMUX_INITIALIZER_UNLOCKED;


/* HLK-LD2450: bit15=1 → positivo, bit15=0 → negativo (magnitude nos bits 0-14). */
static int _hlk_decode_signed(uint8_t lo, uint8_t hi)
{
    int16_t raw = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
    if (hi & 0x80)
        return (int)(raw - (int16_t)0x8000);
    else
        return (int)(-raw);
}


/* ── radar_init ───────────────────────────────────────────── */
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


/* ── radar_flush_rx ───────────────────────────────────────── */
void radar_flush_rx(void)
{
    if (s_mode == RADAR_MODE_UART) {
        uart_flush_input(RADAR_UART_PORT);
        s_ring_len = 0;
        memset(&s_last_data, 0, sizeof(s_last_data));
        ESP_LOGI(TAG, "UART RX limpa");
    }
}


/* ── radar_read_data — lê UART e actualiza cache ─────────── */
bool radar_read_data(radar_data_t *out_data,
                     radar_simulated_input_t *sim_input)
{
    if (!out_data) return false;
    memset(out_data, 0, sizeof(radar_data_t));

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

    uint8_t tmp[64];
    int len = uart_read_bytes(RADAR_UART_PORT, tmp,
                              sizeof(tmp), pdMS_TO_TICKS(10));
    if (len > 0) {
        if (s_ring_len + len > (int)sizeof(s_ring_buf)) {
            int keep = FRAME_LEN - 1;
            memmove(s_ring_buf, s_ring_buf + s_ring_len - keep, keep);
            s_ring_len = keep;
        }
        memcpy(s_ring_buf + s_ring_len, tmp, len);
        s_ring_len += len;
    }

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

    /* Cada alvo = 8 bytes: X(2) Y(2) Speed(2) Resolution(2).
       X/Y/Speed: magnitude + bit de sinal. Resolution ignorado. */
    uint8_t *f = s_ring_buf + found;
    for (int t = 0; t < MAX_RADAR_TARGETS; t++) {
        uint8_t *b = f + 4 + t * 8;

        int decoded_x   = _hlk_decode_signed(b[0], b[1]);
        int decoded_y   = _hlk_decode_signed(b[2], b[3]);
        int decoded_spd = _hlk_decode_signed(b[4], b[5]);
        uint16_t resolution = (uint16_t)((b[7] << 8) | b[6]);

        if (decoded_x == 0 && decoded_y == 0 && decoded_spd == 0 && resolution == 0)
            continue;

        float dx = (float)decoded_x;
        float dy = (float)decoded_y;
        float dist_mm = sqrtf(dx * dx + dy * dy);

        if (dist_mm < 1.0f || dist_mm >= (float)MAX_DIST_MM) continue;

        radar_vehicle_t *tgt = &out_data->targets[out_data->count];
        tgt->x_mm      = decoded_x;
        tgt->y_mm      = decoded_y;
        tgt->distance  = dist_mm / 1000.0f;

        /* Sensor reporta cm/s; positivo = aproximação.
           Invertemos: FSM usa negativo = aproximação. */
        float vel_kmh = (float)decoded_spd * 0.036f;
        tgt->speed_signed = -vel_kmh;
        tgt->speed = (vel_kmh < 0.0f) ? -vel_kmh : vel_kmh;

        tgt->detected = true;
        out_data->count++;
    }

    int consumed = found + FRAME_LEN;
    memmove(s_ring_buf, s_ring_buf + consumed, s_ring_len - consumed);
    s_ring_len -= consumed;

    taskENTER_CRITICAL(&s_cache_mux);
    s_last_data    = *out_data;
    s_last_read_ok = true;
    s_no_frame_cnt = 0;
    taskEXIT_CRITICAL(&s_cache_mux);

    return true;
}


/* ── radar_manager_get_last_data ─────────────────────────── */
void radar_manager_get_last_data(radar_data_t *out)
{
    if (!out) return;
    taskENTER_CRITICAL(&s_cache_mux);
    *out = s_last_data;
    taskEXIT_CRITICAL(&s_cache_mux);
}

bool         radar_is_connected(void)    { return s_last_read_ok; }
radar_mode_t radar_get_mode(void)        { return s_mode; }

const char *radar_get_status_str(void)
{
    if (s_mode == RADAR_MODE_SIMULATED) return "SIM";
    return s_last_read_ok ? "REAL" : "FAIL";
}


/* ── radar_auto_detect_baud — diagnóstico de baud rate ────── */
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


/* ── radar_diagnostic — janela de 8s ─────────────────────── */
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
    if (!frames_ok)
        ESP_LOGE(TAG, "  RADAR FAIL — verificar TX/RX e alimentação 5V");
    uart_flush_input(RADAR_UART_PORT);
    s_ring_len = 0;
    #undef DIAG_S
    #undef DIAG_MS
}
