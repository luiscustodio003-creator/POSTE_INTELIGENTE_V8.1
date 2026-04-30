/* ============================================================
   MÁQUINA DE ESTADOS — PROCESSAMENTO DE EVENTOS
   @file      fsm_events.c
   @version   3.1  |  2026-04-30
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Ponto central de entrada de eventos do pipeline tracking→FSM
   e dos callbacks UDP recebidos pelo udp_manager.

   REGRAS DO PROTOCOLO T/Tc (implementadas aqui):
   ────────────────────────────────────────────────
   DETECTED  → guarda ETA, muda estado. SEM T++. SEM TC_INC.
   LOCAL     → T++, Tc--, muda estado, PASSED→esq, TC_INC+SPD→dir.
   PASSED    → T--, SPD→dir, apaga se T==0 e Tc==0.
   OBSTACULO → STATE_OBSTACULO, PASSED→dir (cancela Tc).

   CORRECÇÕES v3.0 → v3.1:
   ─────────────────────────
   - REMOVIDO: todas as chamadas dali_* (dali_fade_up, dali_set_brightness).
     Este módulo NÃO chama hardware directamente.
     A aplicação de brilho é da exclusiva responsabilidade de
     fsm_aplicar_luz() em fsm_task.c, chamada após cada ciclo de eventos.
   - REMOVIDO: #include "dali_manager.h" — já não necessário.
   - MANTIDO: toda a lógica de estado g_fsm_* — sem alterações.
   - MANTIDO: protocolo T/Tc — sem alterações.
============================================================ */

#include "fsm_events.h"
#include "fsm_core.h"
#include "comm_manager.h"
#include "system_config.h"
#include "esp_log.h"

static const char *TAG = "FSM_EVT";


/* ============================================================
   CALLBACKS UDP
   ──────────────
   Chamados pelo udp_manager no contexto da udp_task (Core 0).
   Acesso a g_fsm_T / g_fsm_Tc: escrita atómica em Xtensa LX6
   (32 bits alinhados) — sem mutex necessário para int simples.
   NOTA: NÃO chamam dali_* — correm em Core 0, o dali corre
   em Core 1. A transição de estado é detectada por
   fsm_aplicar_luz() no próximo ciclo de 100ms da fsm_task.
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
        ESP_LOGW(TAG, "[UDP] TC_INC ignorado — carro ja em transito (Tc=%d)", g_fsm_Tc);
    }

    ESP_LOGI(TAG, "[UDP] TC_INC | vel=%.0f | T=%d Tc=%d", speed, g_fsm_T, g_fsm_Tc);

    /* Muda estado — fsm_aplicar_luz() aplica dali no ciclo seguinte */
    if (g_fsm_state == STATE_IDLE   ||
        g_fsm_state == STATE_MASTER ||
        g_fsm_state == STATE_AUTONOMO) {
        g_fsm_state = STATE_LIGHT_ON;
    }
}

/* ── on_prev_passed_received — poste direito confirmou chegada ── */
void on_prev_passed_received(float speed)
{
    if (g_fsm_T > 0) g_fsm_T--;

    ESP_LOGI(TAG, "[UDP] PASSED recebido | T=%d Tc=%d", g_fsm_T, g_fsm_Tc);

    if (g_fsm_T == 0 && g_fsm_Tc == 0)
        fsm_agendar_apagar();
}

/* ── on_spd_received — actualiza ETA de pré-acendimento ── */
void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm)
{
    g_fsm_last_speed = speed;

    /* Só agenda se há um veículo anunciado */
    if (g_fsm_Tc <= 0) return;

    if (eta_ms > MARGEM_ACENDER_MS)
        g_fsm_acender_em_ms = fsm_agora_ms() + (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
    else
        g_fsm_acender_em_ms = fsm_agora_ms();

    ESP_LOGD(TAG, "[UDP] SPD | vel=%.0f eta=%lums", speed, (unsigned long)eta_ms);
}

/* ── on_master_claim_received — regista MASTER na cadeia ── */
void on_master_claim_received(int from_id)
{
    ESP_LOGI(TAG, "[UDP] MASTER_CLAIM de ID=%d", from_id);
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
    ESP_LOGI(TAG, "Vizinho dir. ONLINE");
}


/* ============================================================
   sm_process_event — ponto central de eventos do tracking
   ──────────────────────────────────────────────────────────
   Chamado pela fsm_task para cada evento pendente.
   Cada veículo gera no máximo: DETECTED → LOCAL → PASSED.
   Nunca LOCAL sem DETECTED antes (tracking_manager garante ordem).

   NOTA: nenhum case chama dali_* directamente.
   Todos os cases apenas actualizam g_fsm_state e variáveis
   de contagem. A aplicação de brilho é feita por
   fsm_aplicar_luz() em fsm_task.c após este ciclo de eventos.
============================================================ */
void sm_process_event(sm_event_type_t type, uint16_t vehicle_id,
                      float vel, uint32_t eta_ms, int16_t x_mm)
{
    switch (type) {

        /* ── DETECTED — primeiro avistamento pelo radar ────────
           Responsabilidade: preparar ETA e mudar estado.
           NÃO faz T++ — o veículo ainda não está confirmado.
           NÃO envia TC_INC — só EVT_LOCAL propaga na cadeia.
           NÃO chama dali — fsm_aplicar_luz() trata no ciclo seguinte. */
        case SM_EVT_VEHICLE_DETECTED:
            ESP_LOGI(TAG, "[EVT] DETECTED id=%u vel=%.0f", vehicle_id, vel);

            /* Recuperação de modo AUTONOMO se rede voltou */
            if (g_fsm_state == STATE_AUTONOMO &&
                (g_fsm_right_online || comm_left_online())) {
                g_fsm_state = STATE_IDLE;
                ESP_LOGI(TAG, "Rede OK durante deteccao — saiu de AUTONOMO");
            }

            g_fsm_apagar_pend    = false;
            g_fsm_last_detect_ms = fsm_agora_ms();
            g_fsm_last_speed     = vel;

            /* Agenda pré-acendimento baseado no ETA */
            if (eta_ms > MARGEM_ACENDER_MS)
                g_fsm_acender_em_ms = fsm_agora_ms() + (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
            else
                g_fsm_acender_em_ms = fsm_agora_ms();

            /* Muda estado — fsm_aplicar_luz() aplica dali no ciclo seguinte */
            if (g_fsm_state == STATE_IDLE   ||
                g_fsm_state == STATE_MASTER ||
                g_fsm_state == STATE_AUTONOMO) {
                g_fsm_state = STATE_LIGHT_ON;
            }
            /* SEM T++ aqui. SEM TC_INC aqui. SEM dali aqui. */
            break;


        /* ── LOCAL — veículo confirmado fisicamente no radar ───
           Responsabilidade: handover T/Tc, propagar na cadeia.
           É aqui que T++ acontece — UMA VEZ por veículo.
           TC_INC é enviado UMA VEZ — para o vizinho direito.
           NÃO chama dali — fsm_aplicar_luz() trata no ciclo seguinte. */
        case SM_EVT_VEHICLE_LOCAL:
            ESP_LOGI(TAG, "[EVT] LOCAL id=%u vel=%.0f T=%d Tc=%d",
                     vehicle_id, vel, g_fsm_T, g_fsm_Tc);

            g_fsm_acender_em_ms  = 0;
            g_fsm_apagar_pend    = false;
            g_fsm_last_speed     = vel;
            g_fsm_last_detect_ms = fsm_agora_ms();

            /* Handover: veículo deixa de ser "esperado" (Tc) e passa a "local" (T) */
            if (g_fsm_Tc > 0) g_fsm_Tc--;
            if (g_fsm_T < MAX_RADAR_TARGETS) g_fsm_T++;

            /* Muda estado — fsm_aplicar_luz() aplica dali no ciclo seguinte */
            if (g_fsm_state != STATE_LIGHT_ON &&
                g_fsm_state != STATE_OBSTACULO) {
                g_fsm_state = STATE_LIGHT_ON;
            }

            /* Confirma ao poste esquerdo que o veículo chegou → T-- no viz.esq. */
            comm_notify_prev_passed(vel);

            /* Propaga para vizinho direito — 1 TC_INC por veículo */
            if (g_fsm_right_online) {
                comm_send_tc_inc(vel, x_mm);
                comm_send_spd(vel, x_mm);
            }
            break;


        /* ── PASSED — veículo saiu da zona do radar ────────────
           Responsabilidade: decrementar T, avisar vizinhos, apagar.
           Apagamento só acontece quando T==0 AND Tc==0.          */
        case SM_EVT_VEHICLE_PASSED:
            ESP_LOGI(TAG, "[EVT] PASSED id=%u | T=%d Tc=%d",
                     vehicle_id, g_fsm_T, g_fsm_Tc);

            g_fsm_acender_em_ms = 0;

            if (g_fsm_T > 0) g_fsm_T--;

            /* Envia SPD ao vizinho direito para afinar ETA (sem TC_INC) */
            if (g_fsm_right_online)
                comm_send_spd(vel, x_mm);

            /* Agenda apagamento — fsm_timer só actua quando T==0 e Tc==0 */
            if (g_fsm_T == 0 && g_fsm_Tc == 0)
                fsm_agendar_apagar();
            break;


        /* ── OBSTACULO — veículo parado na zona do radar ───────
           Responsabilidade: mudar estado, cancelar Tc no vizinho.
           NÃO chama dali — fsm_aplicar_luz() trata no ciclo seguinte. */
        case SM_EVT_VEHICLE_OBSTACULO:
            ESP_LOGW(TAG, "[EVT] OBSTACULO id=%u vel=%.0f", vehicle_id, vel);

            g_fsm_obstaculo_last_ms = fsm_agora_ms();

            if (g_fsm_state != STATE_OBSTACULO) {
                /* Cancela Tc no vizinho direito — este veículo não vai chegar */
                comm_notify_prev_passed(vel);
                g_fsm_state = STATE_OBSTACULO;
            }
            break;


        default:
            break;
    }
}
