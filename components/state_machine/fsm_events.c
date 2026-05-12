/* ============================================================
   MÓDULO     : fsm_events
   FICHEIRO   : fsm_events.c — VERSÃO FINAL CORRIGIDA
   VERSÃO     : 3.6  |  2026-05-12
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÕES v3.5 → v3.6 (CALLBACK UDP OBSTÁCULO):
   ───────────────────────────────────────────────────────────
   🔴 BUG UDP CORRIGIDO — Falta comunicação de obstáculo

   - ADICIONADO: on_obstaculo_received(vehicle_id, speed, x_mm)
     Callback chamado quando vizinho esquerdo notifica obstáculo.
     Cancela TC_TIMEOUT e mantém Tc (veículo ainda na linha).

   - MODIFICADO: SM_EVT_VEHICLE_OBSTACULO agora também envia
     comm_send_obstaculo() além de TC_INC.

   CORRECÇÕES INCLUÍDAS:
   ─────────────────────
   ✅ Bug #1: T++ em modo obstáculo (v3.5)
   ✅ Bug UDP: Comunicação de obstáculo entre postes (v3.6)
============================================================ */

#include "fsm_events.h"
#include "fsm_core.h"
#include "fsm_network.h"
#include "comm_manager.h"
#include "system_config.h"
#include "esp_log.h"

static const char *TAG = "FSM_EVT";


/* ============================================================
   CALLBACKS UDP
============================================================ */

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

void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm)
{
    (void)eta_ms;
    (void)x_mm;
    g_fsm_last_speed = speed;
    ESP_LOGD(TAG, "[UDP] SPD | vel=%.0f (guardado para fade)", speed);
}

void on_master_claim_received_ext(int from_id, int master_id)
{
    fsm_network_master_claim_relay(from_id, master_id);
}

void on_master_claim_received(int from_id)
{
    on_master_claim_received_ext(from_id, from_id);
}


/* ============================================================
   on_obstaculo_received  (NOVO v3.6)
   ──────────────────────────────────────────────────────────
   Callback chamado quando recebe UDP "OBSTACULO:..." do
   vizinho esquerdo.

   PROBLEMA QUE RESOLVE:
   Sem este callback, quando Poste A detecta obstáculo,
   Poste B assume TC_TIMEOUT após 8s e apaga luz prematuramente,
   mesmo que veículo ainda esteja parado em A.

   SOLUÇÃO:
   Cancela TC_TIMEOUT quando recebe notificação de obstáculo.
   Mantém Tc inalterado — veículo ainda está "a caminho"
   tecnicamente (presente na linha). Luz fica acesa até
   receber PASSED real quando obstáculo sair.

   @param vehicle_id ID do veículo parado (para logs/tracking futuro)
   @param speed      Velocidade quando parou (para logs)
   @param x_mm       Posição lateral (para logs)
============================================================ */
void on_obstaculo_received(uint16_t vehicle_id, float speed, int16_t x_mm)
{
    (void)x_mm;  /* Pode usar para tracking futuro */
    
    ESP_LOGW(TAG, "═══════════════════════════════════════");
    ESP_LOGW(TAG, "  [UDP] OBSTÁCULO recebido de A");
    ESP_LOGW(TAG, "  vehicle_id=%u | vel=%.1f | x=%d", 
             vehicle_id, speed, x_mm);
    ESP_LOGW(TAG, "  Estado actual: T=%d Tc=%d env_dir=%d",
             g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);
    
    /* ── Actualiza estado geral ─────────────────────────── */
    g_fsm_last_speed     = speed;
    g_fsm_last_detect_ms = fsm_agora_ms();
    
    /* ── CRÍTICO: Cancela timeout UDP ───────────────────────
       O veículo não "desapareceu" — está parado em A.
       Mantém Tc (veículo ainda presente na linha).
       Luz fica acesa até A enviar PASSED (quando obstáculo sair).
    ──────────────────────────────────────────────────────── */
    if (g_fsm_tc_timeout_ms > 0) {
        ESP_LOGW(TAG, "  TC_TIMEOUT cancelado (veículo parado em A)");
        g_fsm_tc_timeout_ms = 0;
    } else {
        ESP_LOGD(TAG, "  TC_TIMEOUT já estava inactivo");
    }
    
    /* ── Mantém Tc — veículo ainda "a caminho" ───────────── */
    ESP_LOGW(TAG, "  Tc mantém-se=%d (aguarda PASSED quando sair)", g_fsm_Tc);
    ESP_LOGW(TAG, "  Luz mantém ACESA até obstáculo sair");
    ESP_LOGW(TAG, "═══════════════════════════════════════");
}


/* ============================================================
   GESTÃO DE VIZINHOS
============================================================ */

void sm_on_right_neighbor_offline(void)
{
    if (!g_fsm_right_online) return;
    g_fsm_right_online  = false;
    g_fsm_acender_em_ms = 0;

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


/* ============================================================
   sm_process_event — ponto central de eventos do tracking
============================================================ */
void sm_process_event(sm_event_type_t type, uint16_t vehicle_id,
                      float vel, uint32_t eta_ms, int16_t x_mm)
{
    switch (type) {

        /* ── DETECTED — primeiro avistamento pelo radar ──────── */
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


        /* ── APPROACHING — veículo a aproximar-se ────────────── */
        case SM_EVT_VEHICLE_APPROACHING:
            g_fsm_apagar_pend    = false;
            g_fsm_last_speed     = vel;
            g_fsm_last_detect_ms = fsm_agora_ms();
            break;


       /* ── LOCAL — veículo confirmado fisicamente no radar ─── */
        case SM_EVT_VEHICLE_LOCAL:
            ESP_LOGI(TAG, "[LUZ ON] ID=%u | %.1f km/h | T=%d Tc=%d",
                    vehicle_id, vel, g_fsm_T + 1, g_fsm_Tc);

            g_fsm_acender_em_ms  = 0;
            g_fsm_apagar_pend    = false;
            g_fsm_last_speed     = vel;
            g_fsm_last_detect_ms = fsm_agora_ms();

            /* Handover T/Tc */
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

            /* Controlo de TC_INC por ID */
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


        /* ── PASSED — veículo saiu do radar ─────────────────── */
        case SM_EVT_VEHICLE_PASSED:
            ESP_LOGI(TAG, "[SAÍDA] ID=%u | T=%d Tc=%d env_dir=%d",
                    vehicle_id, g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);

            g_fsm_last_detect_ms = fsm_agora_ms();
            g_fsm_acender_em_ms  = 0;

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


        /* ═══════════════════════════════════════════════════════
           OBSTACULO — veículo parado
           VERSÃO FINAL: Bug #1 + Bug UDP corrigidos
        ═══════════════════════════════════════════════════════ */
        case SM_EVT_VEHICLE_OBSTACULO:
            ESP_LOGW(TAG, "═══════════════════════════════════════");
            ESP_LOGW(TAG, "  MODO OBSTÁCULO ACTIVADO");
            ESP_LOGW(TAG, "  ID=%u | vel=%.1f km/h | x=%dmm", 
                     vehicle_id, vel, x_mm);
            ESP_LOGW(TAG, "  Estado ANTES: T=%d Tc=%d env_dir=%d",
                     g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);
            
            /* ── Actualiza estado geral ─────────────────────── */
            g_fsm_obstaculo_last_ms = fsm_agora_ms();
            g_fsm_apagar_pend       = false;
            g_fsm_last_speed        = vel;
            g_fsm_last_detect_ms    = fsm_agora_ms();
            
            /* ── T++: Conta veículo parado como presente ─────── */
            if (vehicle_id != g_fsm_tc_last_vehicle_id) {
                if (g_fsm_T < MAX_RADAR_TARGETS) {
                    g_fsm_T++;
                    ESP_LOGW(TAG, "  T++ → %d (veículo parado contado)", g_fsm_T);
                }
                g_fsm_tc_last_vehicle_id = vehicle_id;
            } else {
                ESP_LOGD(TAG, "  T mantém-se (ID=%u já contado)", vehicle_id);
            }
            
            /* ── Tc--: Handover se veio da esquerda ──────────── */
            if (g_fsm_Tc > 0) {
                g_fsm_Tc--;
                comm_notify_prev_passed(vel);
                ESP_LOGW(TAG, "  Tc-- → %d | PASSED enviado à esquerda", g_fsm_Tc);
            } else {
                ESP_LOGW(TAG, "  Tc=0 (obstáculo local, não veio da esq.)");
            }
            
            /* ── COMUNICAÇÃO UDP: TC_INC + OBSTACULO ────────────
               CORRIGIDO v3.6: Envia AMBOS!
               - TC_INC: anúncio normal (compatibilidade)
               - OBSTACULO: notificação específica (cancela timeout)
            ──────────────────────────────────────────────────── */
            if (g_fsm_right_online) {
                if (vehicle_id != g_fsm_tc_last_vehicle_id ||
                    g_fsm_enviados_dir == 0) {
                    
                    /* Envia TC_INC normal */
                    comm_send_tc_inc(vel, x_mm);
                    g_fsm_enviados_dir++;
                    g_fsm_tc_timeout_ms = fsm_agora_ms() + TC_TIMEOUT_MS;
                    ESP_LOGW(TAG, "  TC_INC enviado → B (env_dir=%d)", g_fsm_enviados_dir);
                    
                    /* NOVO v3.6: Envia notificação específica de obstáculo */
                    comm_send_obstaculo(vehicle_id, vel, x_mm);
                    ESP_LOGW(TAG, "  OBSTACULO notificado → B");
                    
                } else {
                    ESP_LOGD(TAG, "  TC_INC suprimido (ID=%u em trânsito)", vehicle_id);
                }
            } else {
                ESP_LOGW(TAG, "  Sem vizinho direito — sem notificação UDP");
            }
            
            /* ── Muda para STATE_OBSTACULO ────────────────────── */
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


/* ============================================================
   COMPATIBILIDADE
============================================================ */

void sm_on_radar_detect(float vel)
{
    sm_process_event(SM_EVT_VEHICLE_LOCAL, 0, vel, 0, 0);
}

void sm_inject_test_car(float vel)
{
    sm_process_event(SM_EVT_VEHICLE_LOCAL, 0, vel, 0, 0);
}
