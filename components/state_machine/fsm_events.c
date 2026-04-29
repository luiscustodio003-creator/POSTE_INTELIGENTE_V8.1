/* ============================================================
   MÓDULO     : fsm_events
   FICHEIRO   : fsm_events.c — Processamento de Eventos (Produção)
   VERSÃO     : 2.0  |  2026-04-29
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   - sm_process_event(): Ponto central de entrada de eventos do tracking real.
   - Callbacks UDP: on_tc_inc_received, on_prev_passed_received, etc.
   - Gestão de vizinhos: Sincronização de estado online/offline.
   - Removida toda a lógica de simulação e shims de compatibilidade.
============================================================ */

#include "fsm_events.h"
#include "fsm_core.h"
#include "comm_manager.h"
#include "dali_manager.h"
#include "system_config.h"
#include "esp_log.h"

static const char *TAG = "FSM_EVT";

/* ============================================================
   CALLBACKS UDP (Mensagens vindas da rede)
============================================================ */

/* ============================================================
   on_tc_inc_received — Callback UDP
   Proteção contra incrementos fantasmas e saturação de rede.
============================================================ */
void on_tc_inc_received(float speed, int16_t x_mm)
{
    g_fsm_apagar_pend = false;
    g_fsm_last_speed  = speed;
    
    // CORREÇÃO: Em vez de "if (g_fsm_Tc < 5)", usa "if (g_fsm_Tc == 0)"
    // Se já estivermos à espera de um carro (Tc=1), ignoramos novos incrementos
    // vindos da rede no curto espaço de tempo (evita o "double trigger").
    if (g_fsm_Tc == 0) {
        g_fsm_Tc = 1;
    } else {
        // Se o Tc já for > 0, apenas atualizamos os timers, mas NÃO incrementamos.
        ESP_LOGW(TAG, "[UDP] TC_INC Ignorado (já existe carro em trânsito)");
    }

    g_fsm_last_detect_ms = fsm_agora_ms();
    
    // Atualiza o timeout (usa o define que adicionámos ao system_config.h)
    g_fsm_tc_timeout_ms  = fsm_agora_ms() + TC_TIMEOUT_MS; 

    ESP_LOGI(TAG, "[UDP] TC_INC Processado: Vel=%.0f | T=%d Tc=%d", speed, g_fsm_T, g_fsm_Tc);

    if (g_fsm_state == STATE_IDLE || g_fsm_state == STATE_MASTER || g_fsm_state == STATE_AUTONOMO) {
        g_fsm_state = STATE_LIGHT_ON;
        dali_fade_up(speed);
    }
}

void on_prev_passed_received(float speed) {
    if (g_fsm_T > 0) {
        g_fsm_T--;
    } else {
        g_fsm_T = 0; // Proteção contra pacotes duplicados
    }
    
    // Se tudo estiver limpo, agenda o desligamento
    if (g_fsm_T == 0 && g_fsm_Tc == 0) {
        fsm_agendar_apagar(); 
    }
}

void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm)
{
    g_fsm_last_speed = speed;
    if (g_fsm_Tc <= 0) return;

    if (eta_ms > MARGEM_ACENDER_MS)
        g_fsm_acender_em_ms = fsm_agora_ms() + (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
    else
        g_fsm_acender_em_ms = fsm_agora_ms();

    ESP_LOGD(TAG, "[UDP] SPD: %.0f km/h | ETA=%lums", speed, (unsigned long)eta_ms);
}

void on_master_claim_received(int from_id)
{
    ESP_LOGI(TAG, "[UDP] MASTER_CLAIM: Poste ID=%d detetado", from_id);
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
    fsm_agendar_apagar();
}

void sm_on_right_neighbor_online(void)
{
    if (g_fsm_right_online) return;
    g_fsm_right_online = true;
    ESP_LOGI(TAG, "Vizinho direito ONLINE");
}

/* ============================================================
   PONTO DE ENTRADA PRINCIPAL (Eventos do Tracking Manager)
============================================================ */

/* ============================================================
   sm_process_event
   ────────────────────────────────────────────────────────────
   @brief Ponto central de decisão da FSM para eventos de radar.
   @note  Versão 4.2: Corrigida transição AUTONOMO -> IDLE e
          proteção rigorosa de contadores T/Tc.
============================================================ */
void sm_process_event(sm_event_type_t type, uint16_t vehicle_id, 
                      float vel, uint32_t eta_ms, int16_t x_mm)
{
    switch (type) {

        case SM_EVT_VEHICLE_DETECTED:
            ESP_LOGI(TAG, "[REAL] DETECTED id=%u vel=%.1f km/h", vehicle_id, vel);
            
            // 1. RECUPERAÇÃO DE REDE: Se o poste estava isolado (AUTONOMO) mas
            // detetou um veículo e sabe que tem vizinhos, tenta voltar ao IDLE.
            if (g_fsm_state == STATE_AUTONOMO) {
                if (g_fsm_right_online || comm_left_online()) {
                    ESP_LOGI(TAG, "Rede detetada durante evento físico. Saindo de AUTONOMO.");
                    g_fsm_state = STATE_IDLE;
                }
            }

            // Incrementa tráfego local (Máximo 3 para evitar fantasmas)
            if (g_fsm_T < 3) g_fsm_T++;
            
            g_fsm_apagar_pend    = false;
            g_fsm_last_detect_ms = fsm_agora_ms();
            g_fsm_last_speed     = vel;

            // Ativa iluminação se estiver em repouso
            if (g_fsm_state == STATE_IDLE || g_fsm_state == STATE_MASTER || g_fsm_state == STATE_AUTONOMO) {
                g_fsm_state = STATE_LIGHT_ON;
                dali_fade_up(vel);
            }

            // Notifica o próximo poste (se existir) que o carro vai chegar
            if (g_fsm_right_online) {
                comm_send_tc_inc(vel, x_mm);
                comm_send_spd(vel, x_mm);
            }
            break;

        case SM_EVT_VEHICLE_LOCAL:
            ESP_LOGI(TAG, "[REAL] LOCAL id=%u T=%d Tc=%d", vehicle_id, g_fsm_T, g_fsm_Tc);
            
            g_fsm_acender_em_ms = 0;
            
            // 2. LÓGICA DE HANDOVER (Tc -> T):
            // Se o carro chegou ao "centro" do radar, ele deixa de ser esperado (Tc)
            // e passa a ser contado como local (T).
            if (g_fsm_Tc > 0) g_fsm_Tc--;
            else g_fsm_Tc = 0;

            if (g_fsm_T < 3) g_fsm_T++;
            
            g_fsm_last_speed     = vel;
            g_fsm_last_detect_ms = fsm_agora_ms();
            g_fsm_apagar_pend    = false;

            // Garante luz ligada
            if (g_fsm_state != STATE_LIGHT_ON && g_fsm_state != STATE_OBSTACULO) {
                g_fsm_state = STATE_LIGHT_ON;
                dali_fade_up(vel);
            }

            // Avisa o poste anterior que o carro já passou (limpa o T do vizinho)
            comm_notify_prev_passed(vel);

            // Mantém o poste seguinte em alerta
            if (g_fsm_right_online) {
                comm_send_tc_inc(vel, x_mm);
                comm_send_spd(vel, x_mm);
            }
            break;

        case SM_EVT_VEHICLE_PASSED:
            ESP_LOGI(TAG, "[REAL] PASSED id=%u | T=%d", vehicle_id, g_fsm_T);
            g_fsm_acender_em_ms = 0;
            
            // Decrementa tráfego local
            if (g_fsm_T > 0) g_fsm_T--;
            else g_fsm_T = 0;

            // Notifica vizinho esquerdo (limpeza de rasto UDP)
            if (comm_left_online()) {
                comm_notify_prev_passed(vel);
            }

            // Notifica vizinho direito (última atualização de velocidade)
            if (g_fsm_right_online) {
                comm_send_spd(vel, x_mm);
            }
            
            // Tenta apagar a luz se não houver mais ninguém
            fsm_agendar_apagar();
            break;

        case SM_EVT_VEHICLE_OBSTACULO:
            g_fsm_obstaculo_last_ms = fsm_agora_ms();
            if (g_fsm_state != STATE_OBSTACULO) {
                // Se detetar obstáculo, avisa o poste anterior para não esperar por um carro em movimento
                comm_notify_prev_passed(vel);
                
                g_fsm_state = STATE_OBSTACULO;
                dali_set_brightness(LIGHT_MAX);
                ESP_LOGW(TAG, "[ALERTA] Obstáculo estático no radar!");
            }
            break;

        default:
            break;
    }
}