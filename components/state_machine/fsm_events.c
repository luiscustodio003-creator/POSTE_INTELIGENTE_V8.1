/* ============================================================
   MÓDULO     : fsm_events
   FICHEIRO   : fsm_events.c — Processamento de eventos da FSM
   VERSÃO     : 1.0  |  2026-04-26
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   - sm_process_event(): ponto central de entrada de eventos
   - Callbacks UDP: on_tc_inc_received, on_prev_passed_received,
     on_spd_received, on_master_claim_received
   - Gestão de vizinhos: offline/online do vizinho direito
   - Compatibilidade: sm_on_radar_detect, sm_inject_test_car
============================================================ */

#include "fsm_events.h"
#include "fsm_core.h"
#include "fsm_sim.h"
#include "comm_manager.h"
#include "dali_manager.h"
#include "system_config.h"
#include "esp_log.h"

static const char *TAG = "FSM_EVT";


/* ============================================================
   CALLBACKS UDP
============================================================ */

/* Recebe TC_INC do poste esquerdo — incrementa Tc.
   Não reenvia — propagação feita pelo EVT_LOCAL. */
void on_tc_inc_received(float speed, int16_t x_mm)
{
    g_fsm_apagar_pend    = false;
    g_fsm_last_speed     = speed;
    g_fsm_Tc++;
    g_fsm_last_detect_ms = fsm_agora_ms();
    g_fsm_tc_timeout_ms  = fsm_agora_ms() + TC_TIMEOUT_MS;

    ESP_LOGI(TAG, "[TC_INC] %.0f km/h | x=%dmm | T=%d Tc=%d",
             speed, (int)x_mm, g_fsm_T, g_fsm_Tc);

#if USE_RADAR == 0
    fsm_sim_notificar_chegada(speed, x_mm);
#endif
}

/* Recebe PASSED do poste direito — decrementa T.
   Sem propagação — cada poste gere o seu próprio T. */
void on_prev_passed_received(void)
{
    if (g_fsm_T > 0) g_fsm_T--;
    g_fsm_acender_em_ms = 0;
    ESP_LOGI(TAG, "[PASSED recebido] T=%d Tc=%d", g_fsm_T, g_fsm_Tc);
    /* Sem propagação — cada poste gere o seu próprio T. */
    fsm_agendar_apagar();
}

/* Recebe SPD — actualiza ETA para pré-acendimento */
void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm)
{
    g_fsm_last_speed = speed;
    if (g_fsm_Tc <= 0) return;

    if (eta_ms > MARGEM_ACENDER_MS)
        g_fsm_acender_em_ms = fsm_agora_ms() + (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
    else
        g_fsm_acender_em_ms = fsm_agora_ms();

    ESP_LOGI(TAG, "[SPD] %.0f km/h | ETA=%lums | x=%dmm",
             speed, (unsigned long)eta_ms, (int)x_mm);

#if USE_RADAR == 0
    fsm_sim_notificar_chegada(speed, x_mm);
#endif
}

/* Recebe MASTER_CLAIM — log apenas */
void on_master_claim_received(int from_id)
{
    ESP_LOGI(TAG, "[MASTER_CLAIM] de ID=%d", from_id);
}


/* ============================================================
   GESTÃO DE VIZINHOS
============================================================ */

/* Vizinho direito ficou offline — cancela Tc e agenda apagamento */
void sm_on_right_neighbor_offline(void)
{
    if (!g_fsm_right_online) return;
    g_fsm_right_online  = false;
    g_fsm_acender_em_ms = 0;
    if (g_fsm_Tc > 0) {
        g_fsm_Tc = 0;
        ESP_LOGW(TAG, "Vizinho dir. OFFLINE — Tc=0");
    }
    fsm_agendar_apagar();
}

/* Vizinho direito voltou online */
void sm_on_right_neighbor_online(void)
{
    if (g_fsm_right_online) return;
    g_fsm_right_online = true;
    ESP_LOGI(TAG, "Vizinho direito voltou online");
}


/* ============================================================
   sm_process_event — ponto central de entrada de eventos
   Chamado pela fsm_task após ler eventos do tracking_manager.

   DETECTED    → só prepara ETA (T++ é do EVT_LOCAL)
   APPROACHING → pré-acendimento local, sem propagação UDP
   PASSED      → T--, SPD ao vizinho direito, agenda apagar
   LOCAL       → Tc--, T++, luz ON, PASSED ao esq., TC_INC ao dir.
   OBSTACULO   → STATE_OBSTACULO, PASSED ao dir., luz máxima
============================================================ */
void sm_process_event(sm_event_type_t type, uint16_t vehicle_id,
                      float vel, uint32_t eta_ms, int16_t x_mm)
{
    switch (type) {

        /* ── Veículo confirmado pelo tracking_manager ────────── */
        case SM_EVT_VEHICLE_DETECTED:
            ESP_LOGI(TAG, "[EVT] DETECTED id=%u vel=%.1f km/h eta=%lums",
                    vehicle_id, vel, (unsigned long)eta_ms);

            /* T++ e acende luz — no pipeline real não há EVT_LOCAL separado */
            if (g_fsm_T < 3) g_fsm_T++;
            g_fsm_apagar_pend    = false;
            g_fsm_last_detect_ms = fsm_agora_ms();
            g_fsm_last_speed     = vel;

            if (eta_ms > MARGEM_ACENDER_MS)
                g_fsm_acender_em_ms = fsm_agora_ms() + (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
            else
                g_fsm_acender_em_ms = fsm_agora_ms();

            /* Acende luz e regista se vinha de AUTONOMO */
            if (g_fsm_state == STATE_IDLE   ||
                g_fsm_state == STATE_MASTER ||
                g_fsm_state == STATE_AUTONOMO) {
                g_fsm_era_autonomo = (g_fsm_state == STATE_AUTONOMO);
                g_fsm_state = STATE_LIGHT_ON;
                dali_fade_up(vel);
            }

            /* Propaga para vizinho direito */
            if (g_fsm_right_online) {
                comm_send_tc_inc(vel, x_mm);
                comm_send_spd(vel, x_mm);
            }
            break;

        /* ── Veículo em aproximação activa ──────────────────── */
        case SM_EVT_VEHICLE_APPROACHING:
            ESP_LOGI(TAG, "[EVT] APPROACHING id=%u vel=%.1f km/h eta=%lums",
                     vehicle_id, vel, (unsigned long)eta_ms);
            if (eta_ms <= MARGEM_ACENDER_MS || eta_ms == 0) {
                if (g_fsm_state == STATE_IDLE   ||
                    g_fsm_state == STATE_MASTER ||
                    g_fsm_state == STATE_AUTONOMO) {
                    g_fsm_state = STATE_LIGHT_ON;
                }
                g_fsm_acender_em_ms = 0;
            } else {
                g_fsm_acender_em_ms = fsm_agora_ms() + (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
            }
            g_fsm_last_speed     = vel;
            g_fsm_last_detect_ms = fsm_agora_ms();
            g_fsm_apagar_pend    = false;
            /* Sem propagação UDP — só agenda pré-acendimento local.
               TC_INC é enviado exclusivamente por EVT_LOCAL. */
            break;

        /* ── Veículo saiu da zona do radar ─────────────────── */
        case SM_EVT_VEHICLE_PASSED:
            ESP_LOGI(TAG, "[EVT] PASSED id=%u vel=%.1f km/h | T=%d Tc=%d",
                    vehicle_id, vel, g_fsm_T, g_fsm_Tc);
            g_fsm_acender_em_ms = 0;
            if (g_fsm_T > 0) {
                g_fsm_T--;
            } else {
                ESP_LOGW(TAG, "[EVT] PASSED sem DETECTED prévio id=%u", vehicle_id);
            }
            /* Notifica poste esquerdo apenas se existir vizinho online.
            Em AUTONOMO não há vizinho — a condição protege ambos os casos. */
            if (comm_left_online())
                comm_notify_prev_passed(vel);

            if (g_fsm_right_online)
                comm_send_spd(vel, x_mm);
            fsm_agendar_apagar();
            break;

        /* ── Veículo detectado localmente pelo radar ─────────── */
        case SM_EVT_VEHICLE_LOCAL:
            ESP_LOGI(TAG, "[EVT] LOCAL id=%u vel=%.1f km/h T=%d Tc=%d",
                     vehicle_id, vel, g_fsm_T, g_fsm_Tc);
            g_fsm_acender_em_ms  = 0;
            if (g_fsm_Tc > 0) g_fsm_Tc--;
            if (g_fsm_T  < 3) g_fsm_T++;
            g_fsm_last_speed     = vel;
            g_fsm_last_detect_ms = fsm_agora_ms();
            g_fsm_apagar_pend    = false;
            /* Regista se vinha de AUTONOMO antes de acender */
            if (g_fsm_state == STATE_IDLE   ||
                g_fsm_state == STATE_MASTER ||
                g_fsm_state == STATE_AUTONOMO) {
                g_fsm_era_autonomo = (g_fsm_state == STATE_AUTONOMO);
                g_fsm_state = STATE_LIGHT_ON;
                dali_fade_up(vel);
            }
            /* Confirma ao poste esquerdo que o veículo chegou */
            comm_notify_prev_passed(vel);
            /* Propaga para vizinho direito — 1 TC_INC por veículo */
            if (g_fsm_right_online) {
                comm_send_tc_inc(vel, x_mm);
                comm_send_spd(vel, x_mm);
            }
            break;

        /* ── Obstáculo estático detectado ───────────────────── */
        case SM_EVT_VEHICLE_OBSTACULO:
            ESP_LOGW(TAG, "[EVT] OBSTACULO id=%u vel=%.1f km/h",
                     vehicle_id, vel);
            g_fsm_obstaculo_last_ms = fsm_agora_ms();
            /* Cancela Tc no vizinho direito — veículo não vai chegar.
               Só na primeira activação — evita PASSED duplicados. */
            if (g_fsm_state != STATE_OBSTACULO && g_fsm_right_online)
                comm_notify_prev_passed(vel);
            if (g_fsm_state != STATE_OBSTACULO &&
                g_fsm_state != STATE_SAFE_MODE) {
                g_fsm_state = STATE_OBSTACULO;
                dali_set_brightness(LIGHT_MAX);
                ESP_LOGW(TAG, "[OBSTACULO] CONFIRMADO id=%u — luz máxima",
                         vehicle_id);
            }
            break;

        default:
            ESP_LOGW(TAG, "[EVT] Tipo desconhecido: %d", type);
            break;
    }
}


/* ============================================================
   COMPATIBILIDADE E INJECÇÃO DE TESTE
============================================================ */

/* Shim v3.x — delega para SM_EVT_VEHICLE_LOCAL */
void sm_on_radar_detect(float vel)
{
    ESP_LOGD(TAG, "[SHIM] sm_on_radar_detect → SM_EVT_VEHICLE_LOCAL");
#if USE_RADAR == 0
    float fx = 0.0f, fy = 0.0f;
    fsm_sim_get_objeto(&fx, &fy);
    int16_t x_mm = (int16_t)fx;
#else
    int16_t x_mm = 0;
#endif
    sm_process_event(SM_EVT_VEHICLE_LOCAL, 0, vel, 0, x_mm);
}

/* Injeta carro de teste via debugger */
void sm_inject_test_car(float vel)
{
    ESP_LOGI(TAG, "[TEST] Injecção de carro a %.0f km/h", vel);
    sm_process_event(SM_EVT_VEHICLE_LOCAL, 0, vel, 0, 0);
}
