/* ============================================================
   DALI MANAGER — IMPLEMENTACAO v2.1
   @file      dali_manager.c
   @version   2.1  |  2026-04-08
   Projecto  : Poste Inteligente v8
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

   Alterações v2.0 → v2.1:
   ─────────────────────────
   - Corrigido: PRIu32 substituído por %lu + (unsigned long)
     em dali_fade_up() e dali_fade_down().
   - Removidas linhas comentadas duplicadas em dali_fade_down().
   - Removido #include <inttypes.h> (não necessário sem PRIu32).
============================================================ */
#include "dali_manager.h"
#include "hw_config.h"
#include "system_config.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <math.h>

static const char *TAG = "DALI_MGR";

/* DALI_LEDC_MODE vem do hw_config.h — selecção automática por chip */
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_DUTY_RES   LEDC_TIMER_8_BIT
#define FADE_DOWN_MS    4000

static uint8_t      s_brightness     = 0;
static bool         s_fade_installed = false;
static portMUX_TYPE s_mux            = portMUX_INITIALIZER_UNLOCKED;

/* ============================================================
   _pct_to_duty — curva logaritmica DALI IEC 62386
   Converte percentagem (0-100) em duty cycle (0-255).
   Curva: arc = 1 + (253/3) * log10(pct * 10)
   Conforme tabela DALI do anexo E da IEC 62386.
============================================================ */
static uint32_t _pct_to_duty(uint8_t pct)
{
    if (pct == 0)   return 0;
    if (pct >= 100) return 255;
    float arc = 1.0f + (253.0f / 3.0f) * log10f((float)pct * 10.0f);
    if (arc < 0.0f)   arc = 0.0f;
    if (arc > 254.0f) arc = 254.0f;
    return (uint32_t)(arc * 255.0f / 254.0f + 0.5f);
}

/* ============================================================
   _fade_to_pct — fade por hardware LEDC (nao bloqueia CPU)
   Usa o serviço de fade do LEDC para transição suave.
   Se o fade não está instalado, faz set directo.
============================================================ */
static void _fade_to_pct(uint8_t pct, uint32_t time_ms)
{
    if (pct < LIGHT_MIN) pct = LIGHT_MIN;
    if (pct > LIGHT_MAX) pct = LIGHT_MAX;

    uint32_t duty = _pct_to_duty(pct);

    if (s_fade_installed) {
        ledc_set_fade_with_time(DALI_LEDC_MODE, LEDC_CHANNEL,
                                duty, (int)time_ms);
        ledc_fade_start(DALI_LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
    } else {
        ledc_set_duty(DALI_LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(DALI_LEDC_MODE, LEDC_CHANNEL);
    }

    portENTER_CRITICAL(&s_mux);
    s_brightness = pct;
    portEXIT_CRITICAL(&s_mux);
}

/* ============================================================
   dali_init
   Configura timer LEDC, canal PWM e serviço de fade.
   Chamado uma vez pelo system_monitor na inicialização.
============================================================ */
void dali_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = DALI_LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = LED_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch = {
        .speed_mode = DALI_LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = LED_PWM_PIN,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    ESP_ERROR_CHECK(ledc_fade_func_install(0));
    s_fade_installed = true;

    dali_set_brightness(LIGHT_MIN);

    ESP_LOGI(TAG, "DALI v2.1 | GPIO%d | %dHz | %d%% | IEC 62386",
             LED_PWM_PIN, LED_PWM_FREQ_HZ, LIGHT_MIN);
}

/* ============================================================
   dali_set_brightness — instantaneo, sem fade
   Limita ao intervalo [LIGHT_MIN, LIGHT_MAX].
============================================================ */
void dali_set_brightness(uint8_t brightness)
{
    if (brightness < LIGHT_MIN) brightness = LIGHT_MIN;
    if (brightness > LIGHT_MAX) brightness = LIGHT_MAX;

    uint32_t duty = _pct_to_duty(brightness);

    if (s_fade_installed) {
        ledc_set_fade_with_time(DALI_LEDC_MODE, LEDC_CHANNEL, duty, 1);
        ledc_fade_start(DALI_LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
    } else {
        ledc_set_duty(DALI_LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(DALI_LEDC_MODE, LEDC_CHANNEL);
    }

    portENTER_CRITICAL(&s_mux);
    s_brightness = brightness;
    portEXIT_CRITICAL(&s_mux);
}

/* Atalhos de controlo directo */
void dali_turn_on(void)   { dali_set_brightness(LIGHT_MAX); }
void dali_turn_off(void)  { dali_set_brightness(LIGHT_MIN); }
void dali_safe_mode(void) { dali_set_brightness(LIGHT_SAFE_MODE); }

/* ============================================================
   dali_fade_up — subida vel-dependente IEC 62386
   Tempo de fade adaptado à velocidade do veículo:
     >= 80 km/h → 300ms (reacção rápida, autoestrada)
     >= 50 km/h → 500ms (estrada nacional)
     >= 30 km/h → 800ms (zona urbana)
     <  30 km/h → 500ms (default)
   CORRIGIDO v2.1: PRIu32 → %lu + (unsigned long)
============================================================ */
void dali_fade_up(float vel_kmh)
{
    /* Se já está no máximo não faz nada — evita chamadas redundantes ao LEDC */
    portENTER_CRITICAL(&s_mux);
    uint8_t brilho_actual = s_brightness;
    portEXIT_CRITICAL(&s_mux);

    if (brilho_actual >= LIGHT_MAX) {
        ESP_LOGD(TAG, "Fade UP ignorado — já em %d%%", LIGHT_MAX);
        return;
    }

    uint32_t t_ms;
    if      (vel_kmh >= 80.0f) t_ms = 300;
    else if (vel_kmh >= 50.0f) t_ms = 500;
    else if (vel_kmh >= 30.0f) t_ms = 800;
    else                       t_ms = 500;

    ESP_LOGI(TAG, "Fade UP %.0f km/h -> %lums", vel_kmh, (unsigned long)t_ms);
    _fade_to_pct(LIGHT_MAX, t_ms);
}

/* ============================================================
   dali_fade_down — descida 4000ms para LIGHT_MIN
   Transição suave para luminosidade mínima após passagem.
   CORRIGIDO v2.1: PRIu32 → %lu + (unsigned long)
============================================================ */
void dali_fade_down(void)
{
    ESP_LOGI(TAG, "Fade DOWN %lums -> %d%%",
             (unsigned long)FADE_DOWN_MS, (int)LIGHT_MIN);
    _fade_to_pct(LIGHT_MIN, FADE_DOWN_MS);
}

/* ============================================================
   dali_fade_stop — para fade no nivel actual
   Lê o duty corrente do LEDC e congela nesse ponto.
============================================================ */
void dali_fade_stop(void)
{
    if (!s_fade_installed) return;

    uint32_t duty_now = ledc_get_duty(DALI_LEDC_MODE, LEDC_CHANNEL);
    ledc_set_fade_with_time(DALI_LEDC_MODE, LEDC_CHANNEL, duty_now, 1);
    ledc_fade_start(DALI_LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);

    uint8_t pct = (uint8_t)((duty_now * 100U) / 255U);
    if (pct < LIGHT_MIN) pct = LIGHT_MIN;

    portENTER_CRITICAL(&s_mux);
    s_brightness = pct;
    portEXIT_CRITICAL(&s_mux);
}

/* ============================================================
   dali_get_brightness — thread-safe
   Retorna brilho actual protegido por spinlock.
============================================================ */
uint8_t dali_get_brightness(void)
{
    uint8_t val;
    portENTER_CRITICAL(&s_mux);
    val = s_brightness;
    portEXIT_CRITICAL(&s_mux);
    return val;
}
