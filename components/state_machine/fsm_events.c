/* ============================================================
   MÓDULO     : fsm_events
   FICHEIRO   : fsm_events.c
   VERSÃO     : 2.0  |  2026-04-27
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno

   ALTERAÇÕES v1.0 → v2.0:
   ─────────────────────────
   1. on_radar_fail_received(from_left):
      - from_left=true  (viz.ESQ perdeu radar):
        Este poste entra em AUTONOMO. Não receberá mais TC_INC
        desse lado. Opera pelo radar local até RADAR_OK.
      - from_left=false (viz.DIR perdeu radar):
        Este poste assume MASTER imediatamente. Viz.dir não
        conseguirá enviar PASSED de volta.
        Cancela Tc (viz.dir não vai confirmar chegada).

   2. on_radar_ok_received(from_left):
      - from_left=true  (viz.ESQ recuperou):
        Sai de AUTONOMO → IDLE (ou MASTER se pos=0).
      - from_left=false (viz.DIR recuperou):
        Cede MASTER → IDLE se pos>0.
        Repõe g_fsm_right_radar_fail = false.

   3. on_tc_inc_received: speed<0 → redirige para
      on_prev_passed_received (sinal PASSED via velocidade negativa).

   4. on_tc_inc_received: agenda g_fsm_acender_em_ms com ETA
      estimado — não espera pelo SPD para pré-acender.

   5. on_spd_received: se ETA<=MARGEM, acende imediatamente.

   6. sm_process_event — EVT_LOCAL:
      - NÃO chama comm_notify_prev_passed aqui.
        PASSED só é enviado em EVT_PASSED (única fonte).
      - Propaga TC_INC+SPD ao viz.dir apenas se online E
        viz.dir não está em RADAR_FAIL.

   7. sm_process_event — EVT_PASSED:
      - ÚNICA fonte de comm_notify_prev_passed ao viz.esq.
      - Guard comm_left_online() protege AUTONOMO e MASTER P0.

   8. EVT_DETECTED: não propaga TC_INC — só EVT_LOCAL propaga.

   REGRA DE PASSED (v2.0):
   ────────────────────────
   Só EVT_PASSED envia PASSED ao esquerdo. EVT_LOCAL não envia
   (veículo chegou mas ainda não saiu). Isto elimina o PASSED
   duplicado que causava T negativo e fade down prematuro.

   MODO AUTONOMO nos eventos:
   ────────────────────────────
   Em AUTONOMO o poste opera pelo radar local.
   EVT_LOCAL → T++, luz ON. Sem TC_INC (g_fsm_right_online=false
   ou g_fsm_right_radar_fail=true).
   EVT_PASSED → T--, sem PASSED (comm_left_online()=false).
   Fade down quando T=0 + TRAFIC_TIMEOUT.
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
   _acender_agora — auxiliar interno
   ──────────────────────────────────
   Guard: não repete se já LIGHT_ON.
   Regista se vinha de AUTONOMO (g_fsm_era_autonomo).
============================================================ */
static void _acender_agora(float vel)
{
    if (g_fsm_state == STATE_LIGHT_ON) return;

    if (g_fsm_state == STATE_IDLE    ||
        g_fsm_state == STATE_MASTER  ||
        g_fsm_state == STATE_AUTONOMO) {
        g_fsm_era_autonomo = (g_fsm_state == STATE_AUTONOMO);
        g_fsm_state        = STATE_LIGHT_ON;
        dali_fade_up(vel);
        ESP_LOGI(TAG, "Acendimento %.0f km/h → LIGHT_ON", vel);
    }
}


/* ============================================================
   CALLBACKS UDP
   ─────────────
   Chamados pela udp_task (Core 0) ao receber mensagens UDP.
   Escrevem variáveis escalares — seguro sem mutex.
============================================================ */

/* ── on_tc_inc_received ────────────────────────────────────
   Viz.esq detectou veículo → Tc++, agenda pré-acendimento.

   speed < 0 → sinal PASSED (comm_notify_prev_passed envia
   velocidade negativa) → redirige para on_prev_passed_received.
   speed > 0 → Tc++, calcula ETA estimado.
   NÃO propaga aqui — propagação exclusiva de EVT_LOCAL.
============================================================ */
void on_tc_inc_received(float speed, int16_t x_mm)
{
    /* Velocidade negativa = sinal PASSED codificado no TC_INC */
    if (speed < 0.0f) {
        on_prev_passed_received();
        return;
    }

    /* Em AUTONOMO ou SAFE_MODE não processamos TC_INC —
       não há viz.esq válido a enviar-nos informação fiável */
    if (g_fsm_state == STATE_AUTONOMO ||
        g_fsm_state == STATE_SAFE_MODE) {
        ESP_LOGD(TAG, "[TC_INC] ignorado — modo %s",
                 state_machine_get_state_name());
        return;
    }

    g_fsm_apagar_pend    = false;
    g_fsm_last_speed     = speed;
    g_fsm_Tc++;
    g_fsm_last_detect_ms = fsm_agora_ms();
    g_fsm_tc_timeout_ms  = fsm_agora_ms() + TC_TIMEOUT_MS;

    ESP_LOGI(TAG, "[TC_INC] %.0f km/h | x=%dmm | T=%d Tc=%d",
             speed, (int)x_mm, g_fsm_T, g_fsm_Tc);

    /* ETA estimado — SPD refinará este valor quando chegar */
    float    dist_m  = (float)(POSTE_DIST_M - RADAR_DETECT_M);
    float    spd_ms  = (speed > 1.0f) ? (speed / 3.6f) : 1.0f;
    uint32_t eta_est = (uint32_t)((dist_m / spd_ms) * 1000.0f);

    if (eta_est > MARGEM_ACENDER_MS) {
        g_fsm_acender_em_ms = fsm_agora_ms() +
                              (uint64_t)(eta_est - MARGEM_ACENDER_MS);
        ESP_LOGI(TAG, "[TC_INC] ETA_est=%lums → acende em %lums",
                 (unsigned long)eta_est,
                 (unsigned long)(eta_est - MARGEM_ACENDER_MS));
    } else {
        /* ETA muito curto — acende já */
        g_fsm_acender_em_ms = 0;
        _acender_agora(speed);
    }

#if USE_RADAR == 0
    fsm_sim_notificar_chegada(speed, x_mm);
#endif
}

/* ── on_prev_passed_received ───────────────────────────────
   Viz.dir confirmou que o veículo chegou lá → T-- aqui.
   Agenda apagamento se T=0 e Tc=0 após TRAFIC_TIMEOUT.
   Chamado também via on_tc_inc_received quando speed<0.
============================================================ */
void on_prev_passed_received(void)
{
    if (g_fsm_T > 0) g_fsm_T--;
    g_fsm_acender_em_ms = 0;
    ESP_LOGI(TAG, "[PASSED recebido] T=%d Tc=%d", g_fsm_T, g_fsm_Tc);
    fsm_agendar_apagar();
}

/* ── on_spd_received ───────────────────────────────────────
   Viz.esq envia ETA real → refina g_fsm_acender_em_ms.
   Guard: só age se Tc>0. Se ETA<=MARGEM acende imediatamente.
============================================================ */
void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm)
{
    if (g_fsm_Tc <= 0) {
        ESP_LOGD(TAG, "[SPD] descartado — Tc=0");
        return;
    }

    g_fsm_last_speed = speed;

    if (eta_ms == 0 || eta_ms <= MARGEM_ACENDER_MS) {
        g_fsm_acender_em_ms = 0;
        _acender_agora(speed);
        ESP_LOGI(TAG, "[SPD] ETA expirado → acende agora");
    } else {
        g_fsm_acender_em_ms = fsm_agora_ms() +
                              (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
        ESP_LOGI(TAG, "[SPD] %.0f km/h ETA=%lums acende em %lums",
                 speed, (unsigned long)eta_ms,
                 (unsigned long)(eta_ms - MARGEM_ACENDER_MS));
    }

#if USE_RADAR == 0
    fsm_sim_notificar_chegada(speed, x_mm);
#endif
}

/* ── on_master_claim_received ─────────────────────────────── */
void on_master_claim_received(int from_id)
{
    ESP_LOGI(TAG, "[MASTER_CLAIM] de ID=%d", from_id);
}

/* ── on_radar_fail_received ────────────────────────────────
   Vizinho notificou que o seu radar falhou.

   from_left=true  (viz.ESQ perdeu radar — ex: Poste B para A):
     Este poste (A) entra em AUTONOMO.
     B não vai enviar mais TC_INC nem receber PASSED de A.
     A opera só pelo seu radar local.

   from_left=false (viz.DIR perdeu radar — ex: Poste B para C):
     Este poste (C) assume MASTER imediatamente.
     B não vai enviar TC_INC a C → C não receberá Tc.
     Cancela Tc pendente (não vai ser confirmado).
     C gere o seu T pelo radar e propaga a D...N.
============================================================ */
void on_radar_fail_received(bool from_left)
{
    if (from_left) {
        /* viz.ESQ perdeu radar → entra em AUTONOMO */
        g_fsm_left_radar_fail = true;
        g_fsm_Tc = 0;               /* Tc pendente não vai ser confirmado */
        g_fsm_acender_em_ms = 0;    /* cancela pré-acendimento            */

        if (g_fsm_state != STATE_AUTONOMO  &&
            g_fsm_state != STATE_SAFE_MODE &&
            g_fsm_state != STATE_LIGHT_ON  &&
            g_fsm_state != STATE_OBSTACULO) {
            g_fsm_state = STATE_AUTONOMO;
            ESP_LOGW(TAG, "[RADAR_FAIL viz.esq] → AUTONOMO");
        } else {
            ESP_LOGW(TAG, "[RADAR_FAIL viz.esq] Tc=0 (estado=%s)",
                     state_machine_get_state_name());
        }
    } else {
        /* viz.DIR perdeu radar → assume MASTER imediatamente */
        g_fsm_right_radar_fail = true;

        if (g_fsm_state == STATE_IDLE) {
            g_fsm_state = STATE_MASTER;
            ESP_LOGW(TAG, "[RADAR_FAIL viz.dir] → MASTER imediato");
        } else {
            ESP_LOGW(TAG, "[RADAR_FAIL viz.dir] estado=%s (sem mudança)",
                     state_machine_get_state_name());
        }
    }
}

/* ── on_radar_ok_received ──────────────────────────────────
   Vizinho notificou que o seu radar recuperou.

   from_left=true  (viz.ESQ recuperou):
     Sai de AUTONOMO → IDLE (retoma modo normal).
     Viz.esq voltará a enviar TC_INC.

   from_left=false (viz.DIR recuperou):
     Se este poste era MASTER temporário → cede → IDLE.
     Viz.dir voltará a enviar PASSED.
============================================================ */
void on_radar_ok_received(bool from_left)
{
    if (from_left) {
        g_fsm_left_radar_fail = false;

        if (g_fsm_state == STATE_AUTONOMO) {
            /* Repõe modo normal — viz.esq voltou */
            g_fsm_era_autonomo = false;
            g_fsm_state = (POST_POSITION == 0) ? STATE_MASTER : STATE_IDLE;
            ESP_LOGI(TAG, "[RADAR_OK viz.esq] AUTONOMO → %s",
                     (POST_POSITION == 0) ? "MASTER" : "IDLE");
        } else {
            ESP_LOGI(TAG, "[RADAR_OK viz.esq] estado=%s (sem mudança)",
                     state_machine_get_state_name());
        }
    } else {
        g_fsm_right_radar_fail = false;

        if (g_fsm_state == STATE_MASTER && POST_POSITION > 0) {
            /* Era MASTER temporário por RADAR_FAIL do dir → cede */
            g_fsm_state = STATE_IDLE;
            ESP_LOGI(TAG, "[RADAR_OK viz.dir] cede MASTER → IDLE");
        } else {
            ESP_LOGI(TAG, "[RADAR_OK viz.dir] estado=%s (sem mudança)",
                     state_machine_get_state_name());
        }
    }
}


/* ============================================================
   GESTÃO DE VIZINHOS
   ─────────────────────
   Chamadas por fsm_network_vizinhos() quando detecta mudança.
============================================================ */

void sm_on_right_neighbor_offline(void)
{
    if (!g_fsm_right_online) return;
    g_fsm_right_online  = false;
    g_fsm_acender_em_ms = 0;
    if (g_fsm_Tc > 0) {
        g_fsm_Tc = 0;
        ESP_LOGW(TAG, "Viz. dir. OFFLINE — Tc=0");
    }
    fsm_agendar_apagar();
}

void sm_on_right_neighbor_online(void)
{
    if (g_fsm_right_online) return;
    g_fsm_right_online = true;
    ESP_LOGI(TAG, "Viz. dir. voltou online");
}


/* ============================================================
   sm_process_event — ponto central de eventos do radar
   ──────────────────────────────────────────────────────────
   REGRA ÚNICA DE PASSED:
     Só EVT_PASSED envia comm_notify_prev_passed ao viz.esq.
     EVT_LOCAL NÃO envia — veículo chegou mas ainda não saiu.

   PROPAGAÇÃO TC_INC:
     Só EVT_LOCAL e EVT_DETECTED propagam TC_INC+SPD ao dir.
     Guard: g_fsm_right_online && !g_fsm_right_radar_fail.

   AUTONOMO nos eventos:
     EVT_LOCAL → T++, luz ON, sem TC_INC (dir indisponível).
     EVT_PASSED → T--, sem PASSED (esq indisponível).
============================================================ */
void sm_process_event(sm_event_type_t type, uint16_t vehicle_id,
                      float vel, uint32_t eta_ms, int16_t x_mm)
{
    /* Guard: dir disponível para propagar? */
    bool dir_disponivel = g_fsm_right_online && !g_fsm_right_radar_fail;

    switch (type) {

        /* ── DETECTED — tracking confirmou veículo ────────────── */
        case SM_EVT_VEHICLE_DETECTED:
            ESP_LOGI(TAG, "[EVT] DETECTED id=%u vel=%.1f eta=%lums",
                     vehicle_id, vel, (unsigned long)eta_ms);

            if (g_fsm_T < 3) g_fsm_T++;
            g_fsm_apagar_pend    = false;
            g_fsm_last_detect_ms = fsm_agora_ms();
            g_fsm_last_speed     = vel;

            if (eta_ms > MARGEM_ACENDER_MS)
                g_fsm_acender_em_ms = fsm_agora_ms() +
                                      (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
            else {
                g_fsm_acender_em_ms = 0;
                _acender_agora(vel);
            }

            /* Propaga ao dir se disponível */
            if (dir_disponivel) {
                comm_send_tc_inc(vel, x_mm);
                comm_send_spd(vel, x_mm);
            }
            break;

        /* ── APPROACHING — veículo em aproximação ─────────────── */
        case SM_EVT_VEHICLE_APPROACHING:
            ESP_LOGI(TAG, "[EVT] APPROACHING id=%u vel=%.1f eta=%lums",
                     vehicle_id, vel, (unsigned long)eta_ms);

            g_fsm_last_speed     = vel;
            g_fsm_last_detect_ms = fsm_agora_ms();
            g_fsm_apagar_pend    = false;

            if (eta_ms == 0 || eta_ms <= MARGEM_ACENDER_MS) {
                g_fsm_acender_em_ms = 0;
                _acender_agora(vel);
            } else {
                g_fsm_acender_em_ms = fsm_agora_ms() +
                                      (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
            }
            /* Sem propagação — apenas actualiza ETA local */
            break;

        /* ── PASSED — veículo saiu da zona do radar ───────────── */
        case SM_EVT_VEHICLE_PASSED:
            ESP_LOGI(TAG, "[EVT] PASSED id=%u vel=%.1f | T=%d Tc=%d",
                     vehicle_id, vel, g_fsm_T, g_fsm_Tc);

            g_fsm_acender_em_ms = 0;
            if (g_fsm_T > 0) g_fsm_T--;
            else ESP_LOGW(TAG, "[EVT] PASSED mas T já era 0 id=%u", vehicle_id);

            /* NÃO envia PASSED aqui — já foi enviado em EVT_LOCAL
               quando o radar confirmou a detecção.
               T_A decrementou nesse momento. Enviar novamente
               causaria T_A negativo. */

            fsm_agendar_apagar();
            break;

        /* ── LOCAL — radar local confirmou veículo ────────────── */
        case SM_EVT_VEHICLE_LOCAL:
            ESP_LOGI(TAG, "[EVT] LOCAL id=%u vel=%.1f | T=%d Tc=%d",
                     vehicle_id, vel, g_fsm_T, g_fsm_Tc);

            g_fsm_acender_em_ms  = 0;
            if (g_fsm_Tc > 0) g_fsm_Tc--;  /* veículo anunciado chegou */
            if (g_fsm_T  < 3) g_fsm_T++;
            g_fsm_last_speed     = vel;
            g_fsm_last_detect_ms = fsm_agora_ms();
            g_fsm_apagar_pend    = false;

            /* Acende — veículo está na zona agora */
            _acender_agora(vel);

            /* Notifica viz.esq que o veículo chegou → T-- no viz.esq.
               T_A decrementa quando o radar de B confirma a detecção,
               não quando o veículo sai da zona de B.
               Guard: sem viz.esq (MASTER P0, AUTONOMO, RADAR_FAIL) não envia. */
            if (comm_left_online() && !g_fsm_left_radar_fail)
                comm_notify_prev_passed(vel);

            /* Propaga ao dir se disponível (fim de linha: não propaga) */
            if (dir_disponivel) {
                comm_send_tc_inc(vel, x_mm);
                comm_send_spd(vel, x_mm);
            }
            break;

        /* ── OBSTACULO — veículo parado ───────────────────────── */
        case SM_EVT_VEHICLE_OBSTACULO:
            ESP_LOGW(TAG, "[EVT] OBSTACULO id=%u vel=%.1f",
                     vehicle_id, vel);

            g_fsm_obstaculo_last_ms = fsm_agora_ms();

            /* Cancela Tc no dir — veículo não vai continuar */
            if (g_fsm_state != STATE_OBSTACULO && dir_disponivel)
                comm_notify_prev_passed(vel);

            if (g_fsm_state != STATE_OBSTACULO &&
                g_fsm_state != STATE_SAFE_MODE) {
                g_fsm_era_autonomo = false;
                g_fsm_state        = STATE_OBSTACULO;
                dali_set_brightness(LIGHT_MAX);
                ESP_LOGW(TAG, "[OBSTACULO] id=%u — luz máxima", vehicle_id);
            }
            break;

        default:
            ESP_LOGW(TAG, "[EVT] Tipo desconhecido: %d", type);
            break;
    }
}


/* ============================================================
   COMPATIBILIDADE E TESTE
============================================================ */
void sm_on_radar_detect(float vel)
{
    ESP_LOGD(TAG, "[SHIM] → SM_EVT_VEHICLE_LOCAL");
#if USE_RADAR == 0
    float fx = 0.0f, fy = 0.0f;
    fsm_sim_get_objeto(&fx, &fy);
    int16_t x_mm = (int16_t)fx;
#else
    int16_t x_mm = 0;
#endif
    sm_process_event(SM_EVT_VEHICLE_LOCAL, 0, vel, 0, x_mm);
}

void sm_inject_test_car(float vel)
{
    ESP_LOGI(TAG, "[TEST] %.0f km/h", vel);
    sm_process_event(SM_EVT_VEHICLE_LOCAL, 0, vel, 0, 0);
}
