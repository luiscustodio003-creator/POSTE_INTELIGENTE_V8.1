/* ============================================================
   MÓDULO     : fsm_events
   FICHEIRO   : fsm_events.c — Processamento de eventos da FSM
   VERSÃO     : 3.3  |  2026-05-02
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Ponto central de entrada de eventos do pipeline tracking→FSM
   e dos callbacks UDP recebidos pelo udp_manager.

   REGRAS DO PROTOCOLO T/Tc:
   ──────────────────────────
   DETECTED  → guarda ETA, muda estado. SEM T++. SEM TC_INC.
   LOCAL     → T++, muda estado, TC_INC+SPD→dir.
               SE Tc>0: Tc--, PASSED→esq (veículo vindo da esq.)
               SE Tc=0: sem PASSED (veículo entrou directamente aqui)
   PASSED    → SE vizinho dir: só SPD, aguarda PASSED de confirmação
               SE fim de linha: T--, apaga se T==0 e Tc==0.
   OBSTACULO → STATE_OBSTACULO, PASSED→esq se Tc>0.

   ALTERAÇÕES v3.2 → v3.3:
   ─────────────────────────
   - CORRIGIDO: EVT_LOCAL só envia PASSED ao poste esq. se Tc>0.
     Antes enviava sempre — causava T-- indevido em A quando um
     veículo entrava directamente no radar de B sem passar por A.

   - CORRIGIDO: EVT_PASSED não decrementa T quando há vizinho dir.
     T-- passa a ser feito em on_prev_passed_received() quando B
     confirma a chegada do veículo. Sem vizinho (fim de linha),
     decrementa directamente pois não há confirmação possível.

   ALTERAÇÕES v3.1 → v3.2:
   ─────────────────────────
   - ADICIONADO: on_master_claim_received_ext(from_id, master_id)
     Implementa o callback declarado no udp_manager.h v5.2.
     Delega para fsm_network_master_claim_relay() que faz o relay
     completo em cadeia preservando o master_id original.

   - ALTERADO: on_master_claim_received(from_id)
     Agora delega para on_master_claim_received_ext() em vez de
     apenas fazer log. Mantido para compatibilidade com postes
     que ainda enviem o formato antigo (v5.1).
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
   ──────────────
   Chamados pelo udp_manager no contexto da udp_task (Core 0).
   NÃO chamam dali_* — correm em Core 0, o dali corre em Core 1.
   A transição de estado é detectada por fsm_aplicar_luz() no
   próximo ciclo de 100ms da fsm_task.
============================================================ */

/* ── on_tc_inc_received — veículo anunciado pelo poste esquerdo ── */
void on_tc_inc_received(float speed, int16_t x_mm)
{
    g_fsm_apagar_pend    = false;
    g_fsm_last_speed     = speed;
    g_fsm_last_detect_ms = fsm_agora_ms();
    g_fsm_tc_timeout_ms  = fsm_agora_ms() + TC_TIMEOUT_MS;

    /* Protecção anti double-trigger: só incrementa se Tc==0 */
    if (g_fsm_Tc == 0) {
        g_fsm_Tc = 1;
    } else {
        ESP_LOGW(TAG, "[UDP] TC_INC ignorado — carro ja em transito (Tc=%d)",
                 g_fsm_Tc);
    }

    ESP_LOGI(TAG, "[UDP] TC_INC | vel=%.0f | T=%d Tc=%d", speed, g_fsm_T, g_fsm_Tc);

    if (g_fsm_state == STATE_IDLE   ||
        g_fsm_state == STATE_MASTER ||
        g_fsm_state == STATE_AUTONOMO) {
        g_fsm_state = STATE_LIGHT_ON;
    }
}

/* ── on_prev_passed_received — poste direito confirmou chegada ── */
void on_prev_passed_received(float speed)
{
    (void)speed;

    /* B confirmou que recebeu o veículo que A anunciou via TC_INC */
    if (g_fsm_enviados_dir > 0) g_fsm_enviados_dir--;
    if (g_fsm_T > 0)            g_fsm_T--;

    ESP_LOGI(TAG, "[UDP] PASSED recebido | T=%d Tc=%d env_dir=%d",
             g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);

    if (g_fsm_T == 0 && g_fsm_Tc == 0)
        fsm_agendar_apagar();
}

/* ── on_spd_received — actualiza ETA de pré-acendimento ── */
void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm)
{
    (void)x_mm;
    g_fsm_last_speed = speed;

    if (g_fsm_Tc <= 0) return;

    if (eta_ms > MARGEM_ACENDER_MS)
        g_fsm_acender_em_ms = fsm_agora_ms() + (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
    else
        g_fsm_acender_em_ms = fsm_agora_ms();

    ESP_LOGD(TAG, "[UDP] SPD | vel=%.0f eta=%lums", speed, (unsigned long)eta_ms);
}

/* ── on_master_claim_received_ext — callback novo v5.2 ─────────
   Delega para fsm_network_master_claim_relay() que:
     1. Regista o master_id real
     2. Cede STATE_MASTER se éramos temporários
     3. Propaga ao vizinho direito (relay em cadeia)

   FIX BUG 1: substitui o simples ESP_LOGI anterior.
─────────────────────────────────────────────────────────────── */
void on_master_claim_received_ext(int from_id, int master_id)
{
    fsm_network_master_claim_relay(from_id, master_id);
}

/* ── on_master_claim_received — callback legado (compatibilidade) ──
   Chamado por postes com firmware v5.1 que enviam formato antigo.
   Delega para a versão ext com from_id == master_id.
─────────────────────────────────────────────────────────────────── */
void on_master_claim_received(int from_id)
{
    on_master_claim_received_ext(from_id, from_id);
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
   ──────────────────────────────────────────────────────────
   Chamado pela fsm_task para cada evento pendente.
   Nenhum case chama dali_* directamente.
   A aplicação de brilho é feita por fsm_aplicar_luz() em
   fsm_task.c após este ciclo de eventos.
============================================================ */
void sm_process_event(sm_event_type_t type, uint16_t vehicle_id,
                      float vel, uint32_t eta_ms, int16_t x_mm)
{
    (void)vehicle_id;

    switch (type) {

        /* ── DETECTED — primeiro avistamento pelo radar ──────── */
        case SM_EVT_VEHICLE_DETECTED:
            ESP_LOGI(TAG, "[DETECÇÃO] Objecto | %.1f km/h",
                     vel > 0.3f ? vel : 0.3f);

            if (g_fsm_state == STATE_AUTONOMO &&
                (g_fsm_right_online || comm_left_online())) {
                g_fsm_state = STATE_IDLE;
            }

            g_fsm_apagar_pend    = false;
            g_fsm_last_detect_ms = fsm_agora_ms();
            g_fsm_last_speed     = vel;

            if (eta_ms > MARGEM_ACENDER_MS)
                g_fsm_acender_em_ms = fsm_agora_ms() +
                                      (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
            else
                g_fsm_acender_em_ms = fsm_agora_ms();

            if (g_fsm_state == STATE_IDLE   ||
                g_fsm_state == STATE_MASTER ||
                g_fsm_state == STATE_AUTONOMO) {
                g_fsm_state = STATE_LIGHT_ON;
            }
            break;


        /* ── LOCAL — veículo confirmado fisicamente no radar ─── */
        case SM_EVT_VEHICLE_LOCAL:
            ESP_LOGI(TAG, "[LUZ ON] Objecto confirmado | %.1f km/h | T=%d Tc=%d",
                     vel, g_fsm_T + 1, g_fsm_Tc);

            g_fsm_acender_em_ms  = 0;
            g_fsm_apagar_pend    = false;
            g_fsm_last_speed     = vel;
            g_fsm_last_detect_ms = fsm_agora_ms();

            /* ── Handover T/Tc ────────────────────────────────────
               Só envia PASSED ao poste esquerdo SE havia Tc>0,
               ou seja, se o veículo veio anunciado pelo poste esq.
               Se Tc==0 o veículo entrou directamente aqui — não
               há nada para confirmar à esquerda.                  */
            if (g_fsm_Tc > 0) {
                g_fsm_Tc--;
                comm_notify_prev_passed(vel);
                ESP_LOGI(TAG, "[T/Tc] Veículo vindo da esq. — PASSED enviado");
            } else {
                ESP_LOGI(TAG, "[T/Tc] Veículo local — sem PASSED ao poste esq.");
            }

            if (g_fsm_T < MAX_RADAR_TARGETS) g_fsm_T++;

            if (g_fsm_state != STATE_LIGHT_ON &&
                g_fsm_state != STATE_OBSTACULO) {
                g_fsm_state = STATE_LIGHT_ON;
            }

            /* Propaga sempre para a direita — independente da origem */
            if (g_fsm_right_online) {
                comm_send_tc_inc(vel, x_mm);
                comm_send_spd(vel, x_mm);
                g_fsm_enviados_dir++;
                /* Define timeout para o caso de B nunca confirmar
                   (veículo não chegou a B) — resolvido no passo 9 */
                g_fsm_tc_timeout_ms = fsm_agora_ms() + TC_TIMEOUT_MS;
                ESP_LOGI(TAG, "[T/Tc] TC_INC → B | env_dir=%d timeout=%llums",
                         g_fsm_enviados_dir, (unsigned long long)TC_TIMEOUT_MS);
            }
            break;


        /* ── PASSED — veículo saiu do radar ─────────────────── */
        case SM_EVT_VEHICLE_PASSED:
            ESP_LOGI(TAG, "[SAÍDA] Objecto saiu | T=%d Tc=%d env_dir=%d",
                     g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);

            g_fsm_acender_em_ms = 0;

            if (g_fsm_right_online && g_fsm_enviados_dir > 0) {
                /* Há um TC_INC pendente enviado a B — aguarda PASSED de B.
                   T-- virá via on_prev_passed_received() quando B confirmar. */
                comm_send_spd(vel, x_mm);
            } else {
                /* Sem TC_INC pendente ou sem vizinho direito:
                   não há confirmação possível — decrementa directamente. */
                if (g_fsm_T > 0) g_fsm_T--;
                if (g_fsm_right_online) comm_send_spd(vel, x_mm);
                if (g_fsm_T == 0 && g_fsm_Tc == 0)
                    fsm_agendar_apagar();
            }
            break;


        /* ── OBSTACULO — veículo parado ──────────────────────── */
        case SM_EVT_VEHICLE_OBSTACULO:
            ESP_LOGW(TAG, "[OBSTÁCULO] Objecto parado | LUZ MÁXIMA");

            g_fsm_obstaculo_last_ms = fsm_agora_ms();

            if (g_fsm_state != STATE_OBSTACULO) {
                comm_notify_prev_passed(vel);
                g_fsm_state = STATE_OBSTACULO;
            }
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
