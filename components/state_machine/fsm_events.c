/* ============================================================
   MÓDULO     : fsm_events
   FICHEIRO   : fsm_events.c — Processamento de eventos da FSM
   VERSÃO     : 3.4  |  2026-05-04
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
   LOCAL     → T++, muda estado, TC_INC+SPD→dir (só se ID novo).
               SE Tc>0: Tc--, PASSED→esq (veículo vindo da esq.)
               SE Tc=0: sem PASSED (veículo entrou directamente aqui)
   PASSING   → SE vizinho dir e env_dir>0: só SPD, aguarda PASSED de B
               SE fim de linha ou sem env_dir: T-- directo.
   OBSTACULO → STATE_OBSTACULO, PASSED→esq se Tc>0.

   ALTERAÇÕES v3.3 → v3.4:
   ─────────────────────────

   BUG 1 CORRIGIDO — on_prev_passed_received():
     Situação: o passo 9 do fsm_timer (TC_TIMEOUT_MS) podia expirar
     antes de B enviar o PASSED, fazendo T-- e env_dir=0 pelo timeout.
     Quando o PASSED de B chegava depois, on_prev_passed_received()
     fazia outro T--, tornando T negativo.
     Correcção: se env_dir==0 quando o PASSED chega, o timeout já
     actuou — o PASSED é tardio e deve ser ignorado para não fazer
     T-- duplo. Adicionado guard no início da função.
     Adicionalmente, quando o PASSED chega a tempo, cancela o
     timeout (g_fsm_tc_timeout_ms=0) para evitar que dispare depois.

   BUG 2 CORRIGIDO — EVT_LOCAL em sm_process_event():
     Situação: a FSM recebia o vehicle_id do tracking_manager mas
     descartava-o com (void)vehicle_id. Para proteger contra retries
     UDP, on_tc_inc_received() usava "if (Tc==0) Tc=1", o que impedia
     Tc de reflectir dois veículos reais em simultâneo (Tc ficava
     sempre 1 mesmo com dois carros em trânsito).
     O tracking_manager já atribui IDs únicos e estáveis a cada
     objecto físico — esse ID é a solução correcta.
     Correcção: em EVT_LOCAL, só envia TC_INC se vehicle_id for
     diferente do último ID registado em g_fsm_tc_last_vehicle_id.
     O mesmo veículo com EVT_LOCAL repetido → TC_INC suprimido.
     Dois veículos distintos → dois TC_INC → Tc=2 em B → correcto.
     on_tc_inc_received() passa a fazer Tc++ directamente, sem guard.

   ALTERAÇÕES v3.2 → v3.3:
   ─────────────────────────
   - CORRIGIDO: EVT_LOCAL só envia PASSED ao poste esq. se Tc>0.
   - CORRIGIDO: EVT_PASSED não decrementa T quando há vizinho dir.

   ALTERAÇÕES v3.1 → v3.2:
   ─────────────────────────
   - ADICIONADO: on_master_claim_received_ext(from_id, master_id)
   - ALTERADO: on_master_claim_received() delega para _ext.
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
    /* ── BUG 2 (corrigido aqui no receptor B) ────────────────────
       Antes: "if (Tc==0) Tc=1" — protegia contra retries UDP mas
       impedia Tc=2 quando dois carros reais chegavam de A.
       Agora: a protecção contra duplicados vive em A (EVT_LOCAL),
       que usa o vehicle_id para só enviar TC_INC uma vez por carro.
       B passa a confiar que cada TC_INC que recebe é de um veículo
       diferente — faz Tc++ directamente, sem guard.
       Resultado: com dois carros, Tc sobe para 2 em B. ✓
    ──────────────────────────────────────────────────────────── */
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

   /* if (g_fsm_state == STATE_IDLE   ||
        g_fsm_state == STATE_MASTER ||
        g_fsm_state == STATE_AUTONOMO) {
        g_fsm_state = STATE_LIGHT_ON;
    }*/
}

/* ── on_prev_passed_received — poste direito confirmou chegada ── */
void on_prev_passed_received(float speed)
{
    (void)speed;

    /* ── BUG 1 (corrigido aqui) ──────────────────────────────────
       Situação: o passo 9 (TC_TIMEOUT_MS) pode expirar antes de B
       enviar o PASSED — nesse caso já fez T-- e env_dir=0.
       Se o PASSED de B chega depois do timeout, env_dir já é 0.
       Sem este guard, T-- era feito duas vezes: uma pelo timeout,
       outra aqui — tornando T negativo ou apagando a luz cedo.
       Solução: se env_dir==0 o timeout já actuou — ignorar PASSED.
    ──────────────────────────────────────────────────────────── */
    if (g_fsm_enviados_dir == 0) {
        ESP_LOGW(TAG, "[UDP] PASSED tardio ignorado — timeout já actuou (T=%d Tc=%d)",
                 g_fsm_T, g_fsm_Tc);
        return;
    }

    /* B confirmou que recebeu o veículo que A anunciou via TC_INC.
       Cancela o timeout — a confirmação chegou a tempo. */
    if (g_fsm_enviados_dir > 0) g_fsm_enviados_dir--;
    if (g_fsm_T > 0)            g_fsm_T--;

    /* Cancela o timeout: B confirmou, já não é necessário guardar. */
    if (g_fsm_enviados_dir == 0) {
        g_fsm_tc_timeout_ms = 0;
    }

    ESP_LOGI(TAG, "[UDP] PASSED confirmado | T=%d Tc=%d env_dir=%d",
             g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);

    if (g_fsm_T == 0 && g_fsm_Tc == 0)
        fsm_agendar_apagar();
}

/* ── on_spd_received — actualiza ETA de pré-acendimento ── */
/*void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm)
{
    (void)x_mm;
    g_fsm_last_speed = speed;

    if (g_fsm_Tc <= 0) return;

    if (eta_ms > MARGEM_ACENDER_MS)
        g_fsm_acender_em_ms = fsm_agora_ms() + (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
    else
        g_fsm_acender_em_ms = fsm_agora_ms();

    ESP_LOGD(TAG, "[UDP] SPD | vel=%.0f eta=%lums", speed, (unsigned long)eta_ms);
}*/

void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm)
{
    (void)eta_ms;
    (void)x_mm;
    g_fsm_last_speed = speed; /* só guarda velocidade para o fade quando B detectar localmente */
    ESP_LOGD(TAG, "[UDP] SPD | vel=%.0f (guardado para fade)", speed);
}

/* ── on_master_claim_received_ext — callback novo v5.2 ─────────
   Delega para fsm_network_master_claim_relay() que:
     1. Regista o master_id real
     2. Cede STATE_MASTER se éramos temporários
     3. Propaga ao vizinho direito (relay em cadeia)
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
    /* Reseta também o ID de controlo para não bloquear o próximo veículo */
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
   ──────────────────────────────────────────────────────────
   Chamado pela fsm_task para cada evento pendente.
   Nenhum case chama dali_* directamente.
   A aplicação de brilho é feita por fsm_aplicar_luz() em
   fsm_task.c após este ciclo de eventos.
============================================================ */
void sm_process_event(sm_event_type_t type, uint16_t vehicle_id,
                      float vel, uint32_t eta_ms, int16_t x_mm)
{
    /* vehicle_id deixa de ser descartado — usado em EVT_LOCAL (Bug 2) */

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

           /* if (eta_ms > MARGEM_ACENDER_MS)
                g_fsm_acender_em_ms = fsm_agora_ms() +
                                      (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
            else
                g_fsm_acender_em_ms = fsm_agora_ms();

            if (g_fsm_state == STATE_IDLE   ||
                g_fsm_state == STATE_MASTER ||
                g_fsm_state == STATE_AUTONOMO) {
                g_fsm_state = STATE_LIGHT_ON;
            }*/
            break;


        /* ── APPROACHING — veículo a aproximar-se ────────────── */
        case SM_EVT_VEHICLE_APPROACHING:
            g_fsm_apagar_pend    = false;
            g_fsm_last_speed     = vel;
            g_fsm_last_detect_ms = fsm_agora_ms();

            /* Só actualiza ETA — SEM T++, SEM TC_INC */
            /*if (eta_ms > MARGEM_ACENDER_MS)
                g_fsm_acender_em_ms = fsm_agora_ms() +
                                      (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
            else
                g_fsm_acender_em_ms = fsm_agora_ms();

            ESP_LOGD(TAG, "[APROX] ID=%u | %.1f km/h | ETA=%lums",
                     vehicle_id, vel, (unsigned long)eta_ms);
            */
            break;


       /* ── LOCAL — veículo confirmado fisicamente no radar ─── */
        case SM_EVT_VEHICLE_LOCAL:
            ESP_LOGI(TAG, "[LUZ ON] ID=%u | %.1f km/h | T=%d Tc=%d",
                    vehicle_id, vel, g_fsm_T + 1, g_fsm_Tc);

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
                ESP_LOGI(TAG, "[T/Tc] ID=%u vindo da esq. — PASSED enviado", vehicle_id);
            } else {
                ESP_LOGI(TAG, "[T/Tc] ID=%u local directo — sem PASSED", vehicle_id);
            }

            if (g_fsm_T < MAX_RADAR_TARGETS) g_fsm_T++;

            if (g_fsm_state != STATE_LIGHT_ON &&
                g_fsm_state != STATE_OBSTACULO) {
                g_fsm_state = STATE_LIGHT_ON;
            }

            /* ── Controlo de TC_INC por ID e confirmação pendente ────
            Suprime TC_INC apenas se o mesmo veículo ainda está em
            trânsito para B (env_dir > 0).
            Se env_dir == 0 significa que B já confirmou — aceita o
            mesmo ID como nova passagem legítima.

            Cenário 1 — mesmo ID, ainda em trânsito (env_dir > 0):
                → TC_INC suprimido — evita duplicado UDP.
            Cenário 2 — mesmo ID, B já confirmou (env_dir == 0):
                → TC_INC enviado — nova passagem do mesmo objecto.
            Cenário 3 — ID diferente:
                → TC_INC enviado — veículo diferente.
            ──────────────────────────────────────────────────────── */
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

            /* Actualiza timestamp — o timeout de apagamento conta a partir daqui */
            g_fsm_last_detect_ms = fsm_agora_ms();
            g_fsm_acender_em_ms  = 0;

            if (vehicle_id == g_fsm_tc_last_vehicle_id) {
                g_fsm_tc_last_vehicle_id = 0;
            }

            if (g_fsm_right_online && g_fsm_enviados_dir > 0) {
                /* Tem vizinho C — aguarda PASSED de C para T-- */
                comm_send_spd(vel, x_mm);
            } else {
                /* Fim de linha ou sem env_dir pendente — decrementa directamente */
                if (g_fsm_T > 0) g_fsm_T--;
                if (g_fsm_right_online) comm_send_spd(vel, x_mm);
                if (g_fsm_T == 0 && g_fsm_Tc == 0)
                    fsm_agendar_apagar();   /* passo 7 aguarda TRAFIC_TIMEOUT_MS */
            }
            break;


        /* ── OBSTACULO — veículo parado ──────────────────────── */
        case SM_EVT_VEHICLE_OBSTACULO:
            ESP_LOGW(TAG, "[OBSTÁCULO] ID=%u parado | LUZ MÁXIMA", vehicle_id);

            g_fsm_obstaculo_last_ms = fsm_agora_ms();

            if (g_fsm_state != STATE_OBSTACULO) {
                /* Não envia PASSED aqui: o veículo ainda está no radar.
                   O modo obstáculo mantém T e Tc inalterados — a luz
                   apaga apenas quando o obstáculo desaparece (passo 8). */
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
