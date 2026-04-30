/* ============================================================
   DALI MANAGER — IMPLEMENTACAO v3.0
   @file      dali_manager.c
   @version   3.0  |  2026-04-29
   Projecto  : Poste Inteligente v8
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

   Alterações v2.1 → v3.0:
   ─────────────────────────
   - dali_fade_up(): literais 80/50/30 km/h substituídos por
     VEL_FADE_RAPIDO/MEDIO/LENTO_KMH do system_config.h.
     Em MODO_LABORATORIO=1 usa 3/2/1 km/h (mão/pessoa).
     Em MODO_LABORATORIO=0 usa 80/50/30 km/h (veículos).
   - FADE_DOWN_MS: removido define local — vem do system_config.h.
   - ADICIONADO: dali_get_brightness_real() — lê duty actual do
     hardware LEDC durante o fade para o display mostrar em
     tempo real a progressão da barra de brilho.
   - dali_manager.h: #include <inttypes.h> movido para dentro
     do guard. Adicionada declaração dali_get_brightness_real().
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
/* FADE_DOWN_MS vem do system_config.h — não definir aqui */

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

    ESP_LOGI(TAG, "DALI v3.0 | GPIO%d | %dHz | %d%% | IEC 62386 | %s",
             LED_PWM_PIN, LED_PWM_FREQ_HZ, LIGHT_MIN,
             MODO_LABORATORIO ? "LABORATORIO" : "PRODUCAO");
    ESP_LOGI(TAG, "Fade UP: >%.0f=300ms >%.0f=500ms >%.0f=800ms %s",
             (double)VEL_FADE_RAPIDO_KMH,
             (double)VEL_FADE_MEDIO_KMH,
             (double)VEL_FADE_LENTO_KMH,
             MODO_LABORATORIO ? "km/h (mao)" : "km/h (veiculo)");
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
   Tempo de fade adaptado à velocidade do objecto detectado.
   Limiares definidos no system_config.h via MODO_LABORATORIO:

   MODO_LABORATORIO=1 (bancada, mão/pessoa):
     >= VEL_FADE_RAPIDO_KMH (3.0) → FADE_UP_RAPIDO_MS (300ms)
     >= VEL_FADE_MEDIO_KMH  (2.0) → FADE_UP_MEDIO_MS  (500ms)
     >= VEL_FADE_LENTO_KMH  (1.0) → FADE_UP_LENTO_MS  (800ms)
     <  1.0 km/h            → FADE_UP_DEFAULT_MS (500ms)

   MODO_LABORATORIO=0 (produção, veículos):
     >= VEL_FADE_RAPIDO_KMH (80) → FADE_UP_RAPIDO_MS (300ms)
     >= VEL_FADE_MEDIO_KMH  (50) → FADE_UP_MEDIO_MS  (500ms)
     >= VEL_FADE_LENTO_KMH  (30) → FADE_UP_LENTO_MS  (800ms)
     <  30 km/h             → FADE_UP_DEFAULT_MS (500ms)
============================================================ */
void dali_fade_up(float vel_kmh)
{
    uint8_t brilho_actual = s_brightness;

    ESP_LOGI(TAG, "Fade UP chamado | brilho_actual=%d | LIGHT_MAX=%d | vel=%.1f",
             brilho_actual, LIGHT_MAX, vel_kmh);

    if (brilho_actual >= LIGHT_MAX) {
        ESP_LOGD(TAG, "Fade UP ignorado — já em %d%%", LIGHT_MAX);
        return;
    }

    uint32_t t_ms;
    if      (vel_kmh >= VEL_FADE_RAPIDO_KMH) t_ms = FADE_UP_RAPIDO_MS;
    else if (vel_kmh >= VEL_FADE_MEDIO_KMH)  t_ms = FADE_UP_MEDIO_MS;
    else if (vel_kmh >= VEL_FADE_LENTO_KMH)  t_ms = FADE_UP_LENTO_MS;
    else                                       t_ms = FADE_UP_DEFAULT_MS;

    ESP_LOGI(TAG, "Fade UP %.1f km/h → %lums", vel_kmh, (unsigned long)t_ms);
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
   Valor optimista durante fade (já tem destino, ainda a subir).
============================================================ */
uint8_t dali_get_brightness(void)
{
    uint8_t val;
    portENTER_CRITICAL(&s_mux);
    val = s_brightness;
    portEXIT_CRITICAL(&s_mux);
    return val;
}

/* ============================================================
   dali_get_brightness_real — valor real do hardware LEDC
   ──────────────────────────────────────────────────────────
   Lê o duty cycle actual do periférico LEDC e converte para
   percentagem. Reflecte o valor instantâneo durante o fade —
   não o valor de destino optimista de s_brightness.

   Usar no system_monitor para actualizar a barra DALI no
   display em tempo real durante fade up e fade down:
     t=0ms    barra: 10%   (início do fade)
     t=200ms  barra: 32%   (a subir)
     t=400ms  barra: 58%
     t=800ms  barra: 100%  (fim do fade)
============================================================ */
uint8_t dali_get_brightness_real(void)
{
    if (!s_fade_installed) return s_brightness;

    uint32_t duty = ledc_get_duty(DALI_LEDC_MODE, LEDC_CHANNEL);
    uint8_t  pct  = (uint8_t)((duty * 100U + 127U) / 255U);

    if (pct < LIGHT_MIN) pct = LIGHT_MIN;
    if (pct > LIGHT_MAX) pct = LIGHT_MAX;

    return pct;
}
