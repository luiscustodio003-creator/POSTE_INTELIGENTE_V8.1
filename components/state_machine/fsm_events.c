/* ============================================================
   MÓDULO     : fsm_events
   FICHEIRO   : fsm_events.c — Callbacks UDP e processamento de eventos de tracking
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)
============================================================ */

#include "fsm_events.h"
#include "fsm_core.h"
#include "fsm_network.h"
#include "comm_manager.h"
#include "system_config.h"
#include "esp_log.h"

static const char *TAG = "FSM_EVT";


/* ── Callbacks UDP ────────────────────────────────────────── */

void on_tc_inc_received(float speed, int16_t x_mm)
{
    g_fsm_apagar_pend    = false;
    g_fsm_last_speed     = speed;
    g_fsm_last_detect_ms = fsm_agora_ms();
    g_fsm_tc_timeout_ms  = fsm_agora_ms() + TC_TIMEOUT_MS;

    if (g_fsm_Tc < MAX_RADAR_TARGETS) {
        g_fsm_Tc++;
    } else {
        ESP_LOGW(TAG, "[UDP] TC_INC ignorado — Tc no máximo (%d)", g_fsm_Tc);
    }

    ESP_LOGI(TAG, "[UDP] TC_INC | vel=%.0f | T=%d Tc=%d", speed, g_fsm_T, g_fsm_Tc);
}

void on_prev_passed_received(float speed)
{
    (void)speed;

    if (g_fsm_enviados_dir == 0) {
        ESP_LOGW(TAG, "[UDP] PASSED tardio ignorado — timeout já actuou (T=%d Tc=%d)",
                 g_fsm_T, g_fsm_Tc);
        return;
    }

    if (g_fsm_enviados_dir > 0) g_fsm_enviados_dir--;
    if (g_fsm_T > 0)            g_fsm_T--;

    if (g_fsm_enviados_dir == 0) {
        g_fsm_tc_timeout_ms = 0;
    }

    ESP_LOGI(TAG, "[UDP] PASSED confirmado | T=%d Tc=%d env_dir=%d",
             g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);

    if (g_fsm_T == 0 && g_fsm_Tc == 0)
        fsm_agendar_apagar();
}

static uint32_t _fade_ms_para_velocidade(float vel_kmh)
{
    if (vel_kmh >= VEL_FADE_RAPIDO_KMH) return FADE_UP_RAPIDO_MS;
    if (vel_kmh >= VEL_FADE_MEDIO_KMH)  return FADE_UP_MEDIO_MS;
    if (vel_kmh >= VEL_FADE_LENTO_KMH)  return FADE_UP_LENTO_MS;
    return FADE_UP_DEFAULT_MS;
}

void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm)
{
    (void)x_mm;
    g_fsm_last_speed = speed;

    uint32_t fade_ms = _fade_ms_para_velocidade(speed);

    if (eta_ms == 0 || eta_ms < fade_ms) {
        /* Sem margem para fade gradual — acender instantâneo ao atingir ETA. */
        g_fsm_acender_instantaneo = true;
        fsm_acender_em_ms_set(fsm_agora_ms() + eta_ms);
        ESP_LOGD(TAG, "[UDP] SPD | vel=%.0f ETA=%" PRIu32 "ms fade=%" PRIu32 "ms → INSTANTÂNEO",
                 speed, eta_ms, fade_ms);
    } else {
        g_fsm_acender_instantaneo = false;
        fsm_acender_em_ms_set(fsm_agora_ms() + eta_ms);
        ESP_LOGD(TAG, "[UDP] SPD | vel=%.0f ETA=%" PRIu32 "ms fade=%" PRIu32 "ms → FADE GRADUAL",
                 speed, eta_ms, fade_ms);
    }
}

void on_master_claim_received_ext(int from_id, int master_id)
{
    fsm_network_master_claim_relay(from_id, master_id);
}

void on_master_claim_received(int from_id)
{
    on_master_claim_received_ext(from_id, from_id);
}


/* ── on_obstaculo_received ────────────────────────────────────
   Cancela TC_TIMEOUT quando vizinho esquerdo notifica obstáculo.
   Sem este callback, B expirava Tc após 8s mesmo com veículo parado em A. */
void on_obstaculo_received(uint16_t vehicle_id, float speed, int16_t x_mm)
{
    ESP_LOGW(TAG, "═══════════════════════════════════════");
    ESP_LOGW(TAG, "  [UDP] OBSTÁCULO recebido de A");
    ESP_LOGW(TAG, "  vehicle_id=%u | vel=%.1f | x=%d",
             vehicle_id, speed, x_mm);
    ESP_LOGW(TAG, "  Estado actual: T=%d Tc=%d env_dir=%d",
             g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);

    g_fsm_last_speed     = speed;
    g_fsm_last_detect_ms = fsm_agora_ms();

    if (g_fsm_tc_timeout_ms > 0) {
        ESP_LOGW(TAG, "  TC_TIMEOUT cancelado (veículo parado em A)");
        g_fsm_tc_timeout_ms = 0;
    } else {
        ESP_LOGD(TAG, "  TC_TIMEOUT já estava inactivo");
    }

    ESP_LOGW(TAG, "  Tc mantém-se=%d (aguarda PASSED quando sair)", g_fsm_Tc);
    ESP_LOGW(TAG, "  Luz mantém ACESA até obstáculo sair");
    ESP_LOGW(TAG, "═══════════════════════════════════════");
}


/* ── Gestão de vizinhos ───────────────────────────────────── */

void sm_on_right_neighbor_offline(void)
{
    if (!g_fsm_right_online) return;
    g_fsm_right_online        = false;
    fsm_acender_em_ms_set(0);
    g_fsm_acender_instantaneo = false;

    if (g_fsm_Tc > 0) {
        g_fsm_Tc = 0;
        ESP_LOGW(TAG, "Vizinho dir. OFFLINE — Tc resetado");
    }
    if (g_fsm_enviados_dir > 0) {
        g_fsm_enviados_dir = 0;
        ESP_LOGW(TAG, "Vizinho dir. OFFLINE — env_dir resetado");
    }
    g_fsm_tc_last_vehicle_id = 0;

    fsm_agendar_apagar();
}

void sm_on_right_neighbor_online(void)
{
    if (g_fsm_right_online) return;
    g_fsm_right_online = true;
    ESP_LOGI(TAG, "Vizinho dir. ONLINE");
}


/* ── sm_process_event — ponto central de eventos ─────────── */
void sm_process_event(sm_event_type_t type, uint16_t vehicle_id,
                      float vel, uint32_t eta_ms, int16_t x_mm)
{
    switch (type) {

        case SM_EVT_VEHICLE_DETECTED:
            ESP_LOGI(TAG, "[DETECÇÃO] ID=%u | %.1f km/h", vehicle_id,
                     vel > 0.3f ? vel : 0.3f);

            if (g_fsm_state == STATE_AUTONOMO &&
                (g_fsm_right_online || comm_left_online())) {
                g_fsm_state = STATE_IDLE;
            }

            g_fsm_apagar_pend    = false;
            g_fsm_last_detect_ms = fsm_agora_ms();
            g_fsm_last_speed     = vel;
            break;


        case SM_EVT_VEHICLE_APPROACHING:
            g_fsm_apagar_pend    = false;
            g_fsm_last_speed     = vel;
            g_fsm_last_detect_ms = fsm_agora_ms();
            break;


        case SM_EVT_VEHICLE_LOCAL:
            ESP_LOGI(TAG, "[LUZ ON] ID=%u | %.1f km/h | T=%d Tc=%d",
                    vehicle_id, vel, g_fsm_T + 1, g_fsm_Tc);

            fsm_acender_em_ms_set(0);
            g_fsm_acender_instantaneo = false;  /* carro físico presente — fade normal */
            g_fsm_apagar_pend         = false;
            g_fsm_last_speed     = vel;
            g_fsm_last_detect_ms = fsm_agora_ms();

            if (g_fsm_Tc > 0) {
                g_fsm_Tc--;
                comm_notify_prev_passed(vel);
                ESP_LOGI(TAG, "[T/Tc] ID=%u vindo da esq. — PASSED enviado", vehicle_id);
            } else {
                ESP_LOGI(TAG, "[T/Tc] ID=%u local directo — sem PASSED", vehicle_id);
            }

            if (g_fsm_T < MAX_RADAR_TARGETS) g_fsm_T++;

            if (g_fsm_state != STATE_LIGHT_ON &&
                g_fsm_state != STATE_OBSTACULO) {
                g_fsm_state = STATE_LIGHT_ON;
            }

            if (g_fsm_right_online) {
                if (vehicle_id != g_fsm_tc_last_vehicle_id ||
                    g_fsm_enviados_dir == 0) {
                    comm_send_tc_inc(vel, x_mm);
                    comm_send_spd(vel, x_mm);
                    g_fsm_enviados_dir++;
                    g_fsm_tc_last_vehicle_id = vehicle_id;
                    g_fsm_tc_timeout_ms      = fsm_agora_ms() + TC_TIMEOUT_MS;
                    ESP_LOGI(TAG, "[T/Tc] TC_INC → B | ID=%u env_dir=%d",
                            vehicle_id, g_fsm_enviados_dir);
                } else {
                    ESP_LOGD(TAG, "[T/Tc] TC_INC suprimido — ID=%u ainda em transito (env_dir=%d)",
                            vehicle_id, g_fsm_enviados_dir);
                }
            }
            break;


        case SM_EVT_VEHICLE_PASSED:
            ESP_LOGI(TAG, "[SAÍDA] ID=%u | T=%d Tc=%d env_dir=%d",
                    vehicle_id, g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);

            g_fsm_last_detect_ms = fsm_agora_ms();
            fsm_acender_em_ms_set(0);

            if (vehicle_id == g_fsm_tc_last_vehicle_id) {
                g_fsm_tc_last_vehicle_id = 0;
            }

            if (g_fsm_right_online && g_fsm_enviados_dir > 0) {
                comm_send_spd(vel, x_mm);
            } else {
                if (g_fsm_T > 0) g_fsm_T--;
                if (g_fsm_right_online) comm_send_spd(vel, x_mm);
                if (g_fsm_T == 0 && g_fsm_Tc == 0)
                    fsm_agendar_apagar();
            }
            break;


        case SM_EVT_VEHICLE_OBSTACULO:
            ESP_LOGW(TAG, "═══════════════════════════════════════");
            ESP_LOGW(TAG, "  MODO OBSTÁCULO ACTIVADO");
            ESP_LOGW(TAG, "  ID=%u | vel=%.1f km/h | x=%dmm",
                     vehicle_id, vel, x_mm);
            ESP_LOGW(TAG, "  Estado ANTES: T=%d Tc=%d env_dir=%d",
                     g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);

            g_fsm_obstaculo_last_ms = fsm_agora_ms();
            g_fsm_apagar_pend       = false;
            g_fsm_last_speed        = vel;
            g_fsm_last_detect_ms    = fsm_agora_ms();

            bool is_new_vehicle = (vehicle_id != g_fsm_tc_last_vehicle_id);
            if (is_new_vehicle) {
                if (g_fsm_T < MAX_RADAR_TARGETS) {
                    g_fsm_T++;
                    ESP_LOGW(TAG, "  T++ → %d (veículo parado contado)", g_fsm_T);
                }
                g_fsm_tc_last_vehicle_id = vehicle_id;
            } else {
                ESP_LOGD(TAG, "  T mantém-se (ID=%u já contado)", vehicle_id);
            }

            if (g_fsm_Tc > 0) {
                g_fsm_Tc--;
                comm_notify_prev_passed(vel);
                ESP_LOGW(TAG, "  Tc-- → %d | PASSED enviado à esquerda", g_fsm_Tc);
            } else {
                ESP_LOGW(TAG, "  Tc=0 (obstáculo local, não veio da esq.)");
            }

            if (g_fsm_right_online) {
                if (is_new_vehicle || g_fsm_enviados_dir == 0) {
                    comm_send_tc_inc(vel, x_mm);
                    g_fsm_enviados_dir++;
                    g_fsm_tc_timeout_ms = fsm_agora_ms() + TC_TIMEOUT_MS;
                    ESP_LOGW(TAG, "  TC_INC enviado → B (env_dir=%d)", g_fsm_enviados_dir);

                    comm_send_obstaculo(vehicle_id, vel, x_mm);
                    ESP_LOGW(TAG, "  OBSTACULO notificado → B");
                } else {
                    ESP_LOGD(TAG, "  TC_INC suprimido (ID=%u em trânsito)", vehicle_id);
                }
            } else {
                ESP_LOGW(TAG, "  Sem vizinho direito — sem notificação UDP");
            }

            if (g_fsm_state != STATE_OBSTACULO) {
                g_fsm_state = STATE_OBSTACULO;
                ESP_LOGW(TAG, "  Estado → OBSTACULO (luz 100%%)");
            }

            ESP_LOGW(TAG, "  Estado DEPOIS: T=%d Tc=%d env_dir=%d",
                     g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);
            ESP_LOGW(TAG, "═══════════════════════════════════════");
            break;


        default:
            break;
    }
}
