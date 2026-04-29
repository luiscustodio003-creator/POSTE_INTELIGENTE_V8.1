/* ============================================================
   MÓDULO     : tracking_manager
   FICHEIRO   : tracking_manager.c — Implementação
   VERSÃO     : 1.1  |  2026-04-20
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   
============================================================ */

#include "tracking_manager.h"
#include "system_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

#include <stdatomic.h>

static atomic_bool s_radar_frame_ok = false;

/* Função que o radar_manager_task chama e o Linker não encontrava */
void tracking_manager_task_notify_frame(bool ok) {
    atomic_store(&s_radar_frame_ok, ok);
}

/* Função para a FSM ler a saúde do radar */
bool tracking_manager_get_radar_status(void) {
    return atomic_load(&s_radar_frame_ok);
}

static const char *TAG = "TRK";

/* ── Slot interno — estende tracked_vehicle_t com campos privados ── */
typedef struct {
    tracked_vehicle_t pub;
    float    speed_window[TRK_SPEED_WINDOW];
    uint8_t  speed_win_idx;
    uint8_t  speed_win_filled;
    bool     occupied;
    uint64_t last_seen_ms;
} trk_slot_t;

/* ── Estado interno do módulo ─────────────────────────────── */
static trk_slot_t        s_slots[TRK_MAX_VEHICLES];
static tracked_vehicle_t s_pub[TRK_MAX_VEHICLES];
static uint8_t           s_pub_count = 0;
static SemaphoreHandle_t s_mutex     = NULL;
static uint16_t          s_next_id   = 1;
static trk_stats_t       s_stats     = {0};

/* ── Utilitários privados ─────────────────────────────────── */

static uint64_t _agora_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

static float _distancia_alvos(const trk_slot_t *slot,const radar_vehicle_t *target)
{
    float dx = slot->pub.x_mm - (float)target->x_mm;
    float dy = slot->pub.y_mm - (float)target->y_mm;
    return sqrtf(dx * dx + dy * dy);
}

static float _calcular_velocidade_suavizada(trk_slot_t *slot, float vel_raw)
{
    slot->speed_window[slot->speed_win_idx] = vel_raw;
    slot->speed_win_idx = (slot->speed_win_idx + 1) % TRK_SPEED_WINDOW;
    if (slot->speed_win_filled < TRK_SPEED_WINDOW)
        slot->speed_win_filled++;

    /* Guarda contra divisão por zero — não deve acontecer mas protege */
    if (slot->speed_win_filled == 0)
        return vel_raw;

    float soma = 0.0f;
    for (uint8_t i = 0; i < slot->speed_win_filled; i++)
        soma += slot->speed_window[i];
    return soma / (float)slot->speed_win_filled;
}

static uint32_t _calcular_eta(float dist_m, float speed_kmh)
{
    if (speed_kmh < 1.0f || dist_m <= 0.0f) return 0;
    float speed_ms = speed_kmh / 3.6f;
    return (uint32_t)((dist_m / speed_ms) * 1000.0f);
}

static void _limpar_slot(trk_slot_t *slot)
{
    memset(slot, 0, sizeof(trk_slot_t));
}

static void _copiar_para_publico(void)
{
    if (!s_mutex) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_pub_count = 0;
    for (int i = 0; i < TRK_MAX_VEHICLES; i++) {
        if (s_slots[i].occupied) {
            s_pub[s_pub_count] = s_slots[i].pub;
            s_pub_count++;
        }
    }
    xSemaphoreGive(s_mutex);
}


/* ============================================================
   tracking_manager_init
============================================================ */
void tracking_manager_init(void)
{
    if (!s_mutex)
        s_mutex = xSemaphoreCreateMutex();

    memset(s_slots, 0, sizeof(s_slots));

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memset(s_pub, 0, sizeof(s_pub));
    s_pub_count = 0;
    xSemaphoreGive(s_mutex);

    memset(&s_stats, 0, sizeof(s_stats));
    s_next_id = 1;

    ESP_LOGI(TAG, "tracking_manager v1.1 inicializado");
}


/* ============================================================
   tracking_manager_update — pipeline principal a 100ms
============================================================ */
void tracking_manager_update(const radar_data_t *data)
{
    if (!data) return;

    s_stats.frames_processed++;

    bool     slot_associado[TRK_MAX_VEHICLES] = {false};
    uint64_t agora = _agora_ms();


    /* ── PASSO 1: Associação nearest-neighbour ────────────── */

    for (int t = 0; t < data->count; t++) {
        const radar_vehicle_t *alvo = &data->targets[t];

        if (alvo->distance < RADAR_MIN_DIST_M) continue;
        if (alvo->speed    < MIN_DETECT_KMH)   continue;

        float melhor_dist = TRK_ASSOC_RADIUS_MM + 1.0f;
        int   melhor_slot = -1;

        for (int s = 0; s < TRK_MAX_VEHICLES; s++) {
            if (!s_slots[s].occupied)                          continue;
            if (slot_associado[s])                             continue;
            if (s_slots[s].pub.state == TRK_STATE_EXITED)     continue;

            float d = _distancia_alvos(&s_slots[s], alvo);
            if (d < melhor_dist) {
                melhor_dist = d;
                melhor_slot = s;
            }
        }

        if (melhor_slot >= 0) {
            slot_associado[melhor_slot]                  = true;
            s_slots[melhor_slot].last_seen_ms            = agora;
            s_slots[melhor_slot].pub.x_mm                = (float)alvo->x_mm;
            s_slots[melhor_slot].pub.y_mm                = (float)alvo->y_mm;
            s_slots[melhor_slot].pub.distance_m          = alvo->distance;
            s_slots[melhor_slot].pub.speed_signed        = alvo->speed_signed;
            s_slots[melhor_slot].pub.speed_kmh           =
                _calcular_velocidade_suavizada(&s_slots[melhor_slot], alvo->speed);
            s_slots[melhor_slot].pub.lost_frames         = 0;
            s_slots[melhor_slot].pub.total_frames++;
            s_slots[melhor_slot].pub.active              = true;

            /* Actualiza velocidade máxima registada */
            if (s_slots[melhor_slot].pub.speed_kmh >
                s_slots[melhor_slot].pub.speed_kmh_max)
                s_slots[melhor_slot].pub.speed_kmh_max =
                    s_slots[melhor_slot].pub.speed_kmh;

        } else {
            s_stats.association_failures++;
            bool slot_criado = false;

            for (int s = 0; s < TRK_MAX_VEHICLES; s++) {
                if (!s_slots[s].occupied) {
                    _limpar_slot(&s_slots[s]);

                    s_slots[s].occupied           = true;
                    s_slots[s].last_seen_ms       = agora;
                    s_slots[s].pub.state          = TRK_STATE_TENTATIVE;
                    s_slots[s].pub.x_mm           = (float)alvo->x_mm;
                    s_slots[s].pub.y_mm           = (float)alvo->y_mm;
                    s_slots[s].pub.distance_m     = alvo->distance;
                    s_slots[s].pub.speed_signed   = alvo->speed_signed;
                    s_slots[s].pub.confirm_frames = 1;
                    s_slots[s].pub.total_frames   = 1;
                    s_slots[s].pub.active         = true;
                    s_slots[s].pub.speed_kmh      = alvo->speed;
                    s_slots[s].pub.speed_kmh_max  = alvo->speed;
                    s_slots[s].speed_window[0]    = alvo->speed;
                    s_slots[s].speed_win_filled   = 1;
                    s_slots[s].speed_win_idx      = 1;

                    slot_associado[s] = true;
                    slot_criado       = true;

                    ESP_LOGD(TAG, "[NOVO] Tentativo @ (%.0f,%.0f) %.1f km/h",
                             s_slots[s].pub.x_mm, s_slots[s].pub.y_mm,
                             s_slots[s].pub.speed_kmh);
                    break;
                }
            }

            if (!slot_criado)
                ESP_LOGW(TAG, "Sem slots livres — alvo descartado");
        }
    }


    /* ── PASSO 2: Slots não associados → COASTING ─────────── */

    for (int s = 0; s < TRK_MAX_VEHICLES; s++) {
        if (!s_slots[s].occupied)                         continue;
        if (slot_associado[s])                            continue;
        if (s_slots[s].pub.state == TRK_STATE_EXITED)    continue;

        s_slots[s].pub.active = false;
        s_slots[s].pub.lost_frames++;

        if (s_slots[s].pub.state != TRK_STATE_COASTING)
            ESP_LOGD(TAG, "[ID %u] → COASTING (lost=%u)",
                     s_slots[s].pub.id, s_slots[s].pub.lost_frames);

        s_slots[s].pub.state = TRK_STATE_COASTING;
    }


    /* ── PASSO 3: Progressão de estados e eventos ─────────── */

    for (int s = 0; s < TRK_MAX_VEHICLES; s++) {
        if (!s_slots[s].occupied) continue;

        trk_slot_t *sl = &s_slots[s];

        switch (sl->pub.state) {

            case TRK_STATE_TENTATIVE:
                if (slot_associado[s]) sl->pub.confirm_frames++;

                if (sl->pub.confirm_frames >= TRK_CONFIRM_FRAMES) {
                    sl->pub.id    = s_next_id++;
                    sl->pub.state = TRK_STATE_CONFIRMED;
                    sl->pub.event_detected_pending = true;
                    s_stats.total_vehicles_tracked++;

                    ESP_LOGI(TAG, "[ID %u] CONFIRMED @ (%.0f,%.0f) %.1f km/h dist=%.1fm",
                             sl->pub.id, sl->pub.x_mm, sl->pub.y_mm,
                             sl->pub.speed_kmh, sl->pub.distance_m);
                }
                break;

            case TRK_STATE_CONFIRMED:
                /* Detecção de obstáculo: parado na zona após ter tido movimento */
                if (sl->pub.speed_kmh <= OBSTACULO_SPEED_MAX_KMH &&
                    sl->pub.distance_m <= (float)RADAR_DETECT_M  &&
                    sl->pub.speed_kmh_max > MIN_DETECT_KMH) {
                    sl->pub.obstaculo_frames++;
                    if (sl->pub.obstaculo_frames >= OBSTACULO_MIN_FRAMES &&
                        !sl->pub.event_obstaculo_pending) {
                        sl->pub.event_obstaculo_pending = true;
                        ESP_LOGW(TAG, "[ID %u] OBSTACULO — parado %u frames na zona",
                                 sl->pub.id, sl->pub.obstaculo_frames);
                    }
                } else {
                    sl->pub.obstaculo_frames = 0;
                }

                /* Transita para APPROACHING se a aproximar-se */
                if (sl->pub.speed_signed <= AFASTAR_THRESHOLD_KMH) {
                    sl->pub.state = TRK_STATE_APPROACHING;
                    sl->pub.event_approach_pending = true;
                    sl->pub.eta_ms = _calcular_eta(sl->pub.distance_m,sl->pub.speed_kmh);
                    
                    ESP_LOGI(TAG, "[ID %u] APPROACHING vel=%.1f km/h dist=%.1fm ETA=%lums",
                             sl->pub.id, sl->pub.speed_kmh,
                             sl->pub.distance_m, (unsigned long)sl->pub.eta_ms);
                }
                break;

            case TRK_STATE_APPROACHING:
                /* Detecção de obstáculo: parou a aproximar-se mas ficou na zona */
                if (sl->pub.speed_kmh <= OBSTACULO_SPEED_MAX_KMH &&
                    sl->pub.distance_m <= (float)RADAR_DETECT_M  &&
                    sl->pub.speed_kmh_max > MIN_DETECT_KMH) {
                    sl->pub.obstaculo_frames++;
                    if (sl->pub.obstaculo_frames >= OBSTACULO_MIN_FRAMES &&
                        !sl->pub.event_obstaculo_pending) {
                        sl->pub.event_obstaculo_pending = true;
                        ESP_LOGW(TAG, "[ID %u] OBSTACULO — parado %u frames na zona",
                                 sl->pub.id, sl->pub.obstaculo_frames);
                    }
                } else {
                    sl->pub.obstaculo_frames = 0;
                }

                /* Actualiza ETA ou volta a CONFIRMED se inverteu */
                if (sl->pub.speed_signed <= AFASTAR_THRESHOLD_KMH) {
                    sl->pub.eta_ms = _calcular_eta(sl->pub.distance_m,
                                                   sl->pub.speed_kmh);
                } else {
                    sl->pub.state  = TRK_STATE_CONFIRMED;
                    sl->pub.eta_ms = 0;
                    ESP_LOGD(TAG, "[ID %u] Inverteu → CONFIRMED", sl->pub.id);
                }
                break;

            case TRK_STATE_COASTING:
                if (sl->pub.lost_frames >= TRK_LOST_FRAMES) {
                    sl->pub.state  = TRK_STATE_EXITED;
                    sl->pub.eta_ms = 0;
                    if (sl->pub.id > 0)
                        sl->pub.event_passed_pending = true;
                    ESP_LOGI(TAG, "[ID %u] EXITED após %u frames perdidos",
                             sl->pub.id, sl->pub.lost_frames);
                }
                break;

            case TRK_STATE_EXITED:
                ESP_LOGI(TAG, "[ID %u] EXITED check — event_passed=%d occupied=%d agora=%llu last=%llu",
                         sl->pub.id, sl->pub.event_passed_pending,
                         sl->occupied, agora, sl->last_seen_ms);
                if (!sl->pub.event_passed_pending ||
                    (agora - sl->last_seen_ms) > 5000ULL) {
                    if (sl->pub.event_passed_pending)
                        ESP_LOGW(TAG, "[ID %u] Slot libertado por timeout", sl->pub.id);
                    _limpar_slot(sl);
                }
                break;
        }
    }


    /* ── PASSO 4: Estatísticas ────────────────────────────── */

    uint8_t activos = 0;
    for (int s = 0; s < TRK_MAX_VEHICLES; s++) {
        if (s_slots[s].occupied &&
            s_slots[s].pub.state != TRK_STATE_EXITED)
            activos++;
    }
    s_stats.current_active = activos;


    /* ── PASSO 5: Cópia thread-safe para buffer público ───── */

    _copiar_para_publico();
}


/* ============================================================
   tracking_manager_get_vehicles
============================================================ */
bool tracking_manager_get_vehicles(tracked_vehicle_t *out,
                                   uint8_t *out_count)
{
    if (!out || !out_count || !s_mutex) {
        if (out_count) *out_count = 0;
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(out, s_pub, s_pub_count * sizeof(tracked_vehicle_t));
    *out_count = s_pub_count;
    xSemaphoreGive(s_mutex);

    return (*out_count > 0);
}


/* ============================================================
   tracking_manager_clear_events
============================================================ */
void tracking_manager_clear_events(uint16_t vehicle_id)
{
    if (!s_mutex) return;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < s_pub_count; i++) {
        if (s_pub[i].id == vehicle_id) {
            s_pub[i].event_detected_pending  = false;
            s_pub[i].event_approach_pending  = false;
            s_pub[i].event_passed_pending    = false;
            s_pub[i].event_obstaculo_pending = false;
            break;
        }
    }
    xSemaphoreGive(s_mutex);

    for (int s = 0; s < TRK_MAX_VEHICLES; s++) {
        if (s_slots[s].occupied && s_slots[s].pub.id == vehicle_id) {
            s_slots[s].pub.event_detected_pending  = false;
            s_slots[s].pub.event_approach_pending  = false;
            s_slots[s].pub.event_passed_pending    = false;
            s_slots[s].pub.event_obstaculo_pending = false;
            break;
        }
    }
}


/* ============================================================
   tracking_manager_get_stats
============================================================ */
void tracking_manager_get_stats(trk_stats_t *out)
{
    if (!out) return;
    *out = s_stats;
}


/* ============================================================
   tracking_manager_reset
============================================================ */
void tracking_manager_reset(void)
{
    memset(s_slots, 0, sizeof(s_slots));

    if (s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        memset(s_pub, 0, sizeof(s_pub));
        s_pub_count = 0;
        xSemaphoreGive(s_mutex);
    }

    s_stats.current_active = 0;
    ESP_LOGW(TAG, "Tracking resetado — todos os slots limpos");
}


/* ============================================================
   tracking_state_name
============================================================ */
const char *tracking_state_name(trk_state_t state)
{
    switch (state) {
        case TRK_STATE_TENTATIVE:   return "TENTATIVE";
        case TRK_STATE_CONFIRMED:   return "CONFIRMED";
        case TRK_STATE_APPROACHING: return "APPROACHING";
        case TRK_STATE_COASTING:    return "COASTING";
        case TRK_STATE_EXITED:      return "EXITED";
        default:                    return "---";
    }
}