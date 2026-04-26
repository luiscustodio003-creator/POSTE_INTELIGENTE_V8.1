/* ============================================================
   fsm_sim.c — Simulador físico de veículos
   Compilado sempre — guardado por #if USE_RADAR == 0 internamente.
============================================================ */

#include "fsm_sim.h"
#include "system_config.h"

#if USE_RADAR == 0

#include "fsm_core.h"
#include "fsm_events.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "FSM_SIM";

typedef enum {
    SIM_AGUARDA = 0, SIM_ENTRAR, SIM_EM_VIA, SIM_DETECTADO, SIM_SAIU
} sim_estado_t;

typedef struct {
    float        y_mm;
    float        x_mm;
    float        vy;
    float        vel_kmh;
    sim_estado_t estado;
    uint64_t     t_inicio_ms;
    bool         injectado;
} sim_carro_t;

static sim_carro_t       s_sim       = {0};
static SemaphoreHandle_t s_sim_mutex = NULL;

static const float s_vels[]  = {30.0f, 50.0f, 80.0f, 50.0f};
static int         s_vel_idx = 0;

#define SIM_INTERVALO_MS  ((uint64_t)(TRAFIC_TIMEOUT_MS + 3000))
#define SIM_ZONA_MM       ((float)(RADAR_DETECT_M * 1000))

static void _sim_lancar(float vel, int16_t x_mm)
{
    float dist_extra = (float)((POSTE_DIST_M - RADAR_MAX_M) * 1000);
    if (dist_extra < 0.0f) dist_extra = 0.0f;
    s_sim.x_mm        = (x_mm == 0)
                      ? (float)((int32_t)(esp_random() % 769) - 384)
                      : (float)x_mm;
    s_sim.y_mm        = (float)RADAR_MAX_MM + dist_extra;
    s_sim.vel_kmh     = vel;
    s_sim.vy          = (vel / 3.6f) * 100.0f * 0.25f;
    s_sim.estado      = SIM_ENTRAR;
    s_sim.injectado   = false;
    s_sim.t_inicio_ms = fsm_agora_ms();
    ESP_LOGI(TAG, "[SIM] Carro lançado | pos=%d | %.0f km/h | x=%.0fmm",
             POST_POSITION, vel, s_sim.x_mm);
}

void fsm_sim_init(void)
{
    if (!s_sim_mutex)
        s_sim_mutex = xSemaphoreCreateMutex();
}

void fsm_sim_notificar_chegada(float vel_kmh, int16_t x_mm)
{
    if (!s_sim_mutex) return;
    xSemaphoreTake(s_sim_mutex, portMAX_DELAY);
    if (s_sim.estado == SIM_AGUARDA ||
        s_sim.estado == SIM_SAIU    ||
        s_sim.estado == SIM_ENTRAR)
        _sim_lancar(vel_kmh, x_mm);
    xSemaphoreGive(s_sim_mutex);
}

void fsm_sim_update(void)
{
    if (!s_sim_mutex) return;
    xSemaphoreTake(s_sim_mutex, portMAX_DELAY);
    uint64_t agora = fsm_agora_ms();
    switch (s_sim.estado) {
        case SIM_AGUARDA:
            if (g_fsm_state != STATE_MASTER) break;
            if (!s_sim.t_inicio_ms) s_sim.t_inicio_ms = agora;
            if ((agora - s_sim.t_inicio_ms) >= SIM_INTERVALO_MS) {
                float vel = s_vels[s_vel_idx];
                s_vel_idx = (s_vel_idx + 1) % 4;
                _sim_lancar(vel, 0);
            }
            break;
        case SIM_ENTRAR:
            s_sim.y_mm -= s_sim.vy;
            if (s_sim.y_mm <= (float)RADAR_MAX_MM) {
                s_sim.y_mm   = (float)RADAR_MAX_MM;
                s_sim.estado = SIM_EM_VIA;
            }
            break;
        case SIM_EM_VIA:
        case SIM_DETECTADO:
            s_sim.y_mm -= s_sim.vy;
            if (!s_sim.injectado && s_sim.y_mm <= SIM_ZONA_MM) {
                s_sim.estado    = SIM_DETECTADO;
                s_sim.injectado = true;
                xSemaphoreGive(s_sim_mutex);
                sm_on_radar_detect(s_sim.vel_kmh);
                xSemaphoreTake(s_sim_mutex, portMAX_DELAY);
            }
            if (s_sim.y_mm <= 0.0f) {
                s_sim.y_mm        = 0.0f;
                s_sim.estado      = SIM_SAIU;
                s_sim.t_inicio_ms = agora;
            }
            break;
        case SIM_SAIU:
            if (g_fsm_T == 0 && g_fsm_Tc == 0 &&
                (agora - s_sim.t_inicio_ms) >= (uint64_t)TRAFIC_TIMEOUT_MS) {
                s_sim.estado      = SIM_AGUARDA;
                s_sim.t_inicio_ms = agora;
            }
            break;
    }
    xSemaphoreGive(s_sim_mutex);
}

bool fsm_sim_get_objeto(float *x_mm, float *y_mm)
{
    if (!s_sim_mutex || !x_mm || !y_mm) return false;
    xSemaphoreTake(s_sim_mutex, portMAX_DELAY);
    bool visivel = (s_sim.estado == SIM_EM_VIA ||
                    s_sim.estado == SIM_DETECTADO) &&
                   s_sim.y_mm > 0.0f &&
                   s_sim.y_mm <= (float)RADAR_MAX_MM;
    if (visivel) { *x_mm = s_sim.x_mm; *y_mm = s_sim.y_mm; }
    xSemaphoreGive(s_sim_mutex);
    return visivel;
}

#endif /* USE_RADAR == 0 */