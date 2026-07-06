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
#include "udp_manager.h"
#include "system_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <stdio.h>
#include <stdarg.h>

static const char *TAG = "FSM_EVT";


/* ── _box_* — desenho de caixa auto-alinhada ─────────────────
   Em vez de alinhar espaços à mão (quebra sempre que um número
   muda de nº de dígitos), cada linha é formatada para um buffer
   e depois preenchida com %-*s até à largura fixa BOX_W — a
   borda direita fica sempre alinhada, seja qual for o conteúdo. */
#define BOX_W 48  /* largura interior da caixa, entre as duas bordas "│" */

static void _box_top(void)
{
    printf("\n┌");
    for (int i = 0; i < BOX_W; i++) printf("─");
    printf("┐\n");
}

static void _box_bottom(void)
{
    printf("└");
    for (int i = 0; i < BOX_W; i++) printf("─");
    printf("┘\n\n");
}

static void _box_line(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    printf("│%-*s│\n", BOX_W, buf);
}


/* ── _print_deteccao_box ─────────────────────────────────────
   Destaque visual no log sempre que o radar confirma um carro
   localmente (EVT_LOCAL): mostra T/Tc deste poste e se o TC_INC
   foi enviado ao poste seguinte (que fará Tc++ ao recebê-lo).
   Puramente informativo — não afecta a lógica da FSM. */
static void _print_deteccao_box(uint16_t vehicle_id, float vel, int T, int Tc,
                                 bool tc_inc_enviado, const char *ip_dir)
{
    _box_top();
    _box_line(" CARRO DETECTADO  ID=%-4u %.1f km/h", vehicle_id, vel);
    _box_line(" Este poste  P%-2d   T=%-2d  Tc=%-2d", POST_POSITION, T, Tc);
    if (tc_inc_enviado)
        _box_line(" TC_INC -> P%-2d (%s) -> Tc=1 la", POST_POSITION + 1, ip_dir);
    else
        _box_line(" Sem vizinho direito -- TC_INC nao enviado");
    _box_bottom();
}

/* ── _print_tc_inc_box ───────────────────────────────────────
   Destaque visual no log sempre que este poste recebe um
   TC_INC do vizinho esquerdo (Tc++). Ajuda a confirmar se o
   Tc chega mesmo a subir e por quanto tempo se mantém, antes
   de o radar local confirmar o alvo (EVT_LOCAL, Tc--). */
static void _print_tc_inc_box(uint16_t vehicle_id, int T, int Tc)
{
    _box_top();
    _box_line(" TC_INC RECEBIDO  ID=%-4u  P%-2d", vehicle_id, POST_POSITION);
    _box_line(" Anunciado pelo poste anterior");
    _box_line(" T=%-2d  Tc=%-2d", T, Tc);
    _box_bottom();
}

/* ── _print_saida_box ────────────────────────────────────────
   Destaque visual no log sempre que o T deste poste desce
   (EVT_PASSED — radar local perdeu o alvo). Mostra se ainda há
   um TC_INC por confirmar (env_dir>0) ou um Tc pendente, para
   se perceber de imediato se falta alguma coisa a resolver. */
static void _print_saida_box(uint16_t vehicle_id, int T, int Tc, int env_dir)
{
    _box_top();
    _box_line(" T DECREMENTADO   ID=%-4u  P%-2d", vehicle_id, POST_POSITION);
    _box_line(" Radar local perdeu o alvo");
    _box_line(" T=%-2d  Tc=%-2d  env_dir=%-2d", T, Tc, env_dir);
    if (env_dir > 0)
        _box_line(" A aguardar confirmacao do poste seguinte");
    else if (Tc > 0)
        _box_line(" Tc pendente (carro anunciado a chegar)");
    else
        _box_line(" Tudo limpo (T=0 Tc=0)");
    _box_bottom();
}


/* ── _print_passed_enviado_box ───────────────────────────────
   Destaque visual no log sempre que este poste envia a
   confirmação PASSED ao poste anterior (o carro chegou aqui,
   vindo da esquerda). Simétrico com _print_confirmacao_box,
   que mostra o mesmo evento do lado de quem recebe. */
static void _print_passed_enviado_box(uint16_t vehicle_id, int T, int Tc)
{
    _box_top();
    _box_line(" CONFIRMACAO ENVIADA   ID=%-4u  P%-2d", vehicle_id, POST_POSITION);
    _box_line(" Carro chegou -> PASSED -> poste anterior");
    _box_line(" T=%-2d  Tc=%-2d", T, Tc);
    _box_bottom();
}


/* ── _print_confirmacao_box ──────────────────────────────────
   Destaque visual no log sempre que este poste recebe a
   confirmação PASSED do poste seguinte (o carro chegou lá).
   Mostra se ainda falta confirmar mais algum carro (env_dir>0)
   ou se já está tudo pronto para agendar o apagar. */
static void _print_confirmacao_box(int T, int Tc, int env_dir, bool apagar_agendado)
{
    _box_top();
    _box_line(" CONFIRMACAO RECEBIDA   P%-2d", POST_POSITION);
    _box_line(" Poste seguinte confirmou a chegada");
    _box_line(" T=%-2d  Tc=%-2d  env_dir=%-2d", T, Tc, env_dir);
    if (apagar_agendado)
        _box_line(" Tudo confirmado -> a apagar a luz");
    else
        _box_line(" Ainda falta confirmar outro carro");
    _box_bottom();
}


/* ── Callbacks UDP ────────────────────────────────────────── */

/* Dedup do double-send: comm_send_tc_inc() envia a mesma mensagem 2x seguidas
   (sem atraso deliberado) para tolerar perda de pacote UDP. O "vehicle_id"
   aqui recebido é, na prática, o POSTE_ID fixo de quem envia (ver
   udp_manager.c: "TC_INC:<POSTE_ID>:<vel>:<x>") — nunca um identificador por
   veículo. Por isso o dedup NÃO PODE comparar só o ID (ficaria preso para
   sempre no primeiro carro do mesmo vizinho, ignorando todos os seguintes) —
   tem de exigir também que a repetição chegue dentro de uma janela curta,
   coerente com o double-send (que chega em poucos ms), não com o intervalo
   real entre veículos distintos (segundos, no mínimo). */
#define TC_INC_DEDUP_WINDOW_MS 500ULL

static uint16_t s_last_tc_inc_id = 0;
static uint64_t s_last_tc_inc_ms = 0;

void on_tc_inc_received(uint16_t vehicle_id, float speed, int16_t x_mm)
{
    uint64_t agora = fsm_agora_ms();
    if (vehicle_id != 0 && vehicle_id == s_last_tc_inc_id &&
        (agora - s_last_tc_inc_ms) < TC_INC_DEDUP_WINDOW_MS) {
        ESP_LOGD(TAG, "[UDP] TC_INC dedup — ID=%u repetido em <%llums (double-send)",
                 vehicle_id, (unsigned long long)TC_INC_DEDUP_WINDOW_MS);
        return;
    }
    s_last_tc_inc_id = vehicle_id;
    s_last_tc_inc_ms = agora;

    g_fsm_apagar_pend    = false;
    g_fsm_last_speed     = speed;
    g_fsm_last_detect_ms = fsm_agora_ms();
    fsm_tc_timeout_ms_set(fsm_agora_ms() + TC_TIMEOUT_MS);
    fsm_spd_fallback_ms_set(fsm_agora_ms() + SPD_FALLBACK_MS);

    portENTER_CRITICAL(&g_fsm_counters_mux);
    bool tc_overflow = (g_fsm_Tc >= MAX_RADAR_TARGETS);
    if (!tc_overflow) g_fsm_Tc++;
    portEXIT_CRITICAL(&g_fsm_counters_mux);

    if (tc_overflow) {
        ESP_LOGW(TAG, "[UDP] TC_INC ignorado — Tc no máximo (%d)", g_fsm_Tc);
    } else {
        _print_tc_inc_box(vehicle_id, g_fsm_T, g_fsm_Tc);
    }
    ESP_LOGI(TAG, "[UDP] TC_INC | ID=%u vel=%.0f | T=%d Tc=%d", vehicle_id, speed, g_fsm_T, g_fsm_Tc);
}

void on_prev_passed_received(float speed)
{
    (void)speed;

    /* T-- já foi feito em EVT_PASSED. Aqui apenas gere enviados_dir e
       verifica all_clear para agenda de apagamento. */
    portENTER_CRITICAL(&g_fsm_counters_mux);
    bool tardio    = (g_fsm_enviados_dir == 0);
    bool env_zero  = false;
    bool all_clear = false;
    if (!tardio) {
        g_fsm_enviados_dir--;
        env_zero  = (g_fsm_enviados_dir == 0);
        /* env_zero incluído: só agenda apagar quando também não há mais
           nenhum TC_INC por confirmar (consistente com EVT_PASSED/passo8). */
        all_clear = (g_fsm_T == 0 && g_fsm_Tc == 0 && env_zero);
    }
    portEXIT_CRITICAL(&g_fsm_counters_mux);

    if (tardio) {
        ESP_LOGW(TAG, "[UDP] PASSED tardio ignorado (T=%d Tc=%d)", g_fsm_T, g_fsm_Tc);
        return;
    }
    if (env_zero) fsm_tc_timeout_ms_set(0);
    ESP_LOGI(TAG, "[UDP] PASSED confirmado | T=%d Tc=%d env_dir=%d",
             g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);
    if (all_clear) fsm_agendar_apagar();

    _print_confirmacao_box(g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir, all_clear);
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
    fsm_spd_fallback_ms_set(0);   /* SPD chegou — cancela fallback */
    g_fsm_last_speed = speed;

    /* Pré-acendimento por ETA só se justifica quando a velocidade é alta
       o suficiente para a deteção local (EVT_LOCAL, T=1) não ter tempo de
       reagir antes do veículo chegar. Para velocidades normais/baixas há
       tempo de sobra — a luz espera pela deteção local real, em vez de
       acender antecipadamente com base numa previsão que pode nunca se
       confirmar (carro que sai da via antes de chegar). */
    if (speed < VEL_FADE_RAPIDO_KMH) {
        ESP_LOGD(TAG, "[UDP] SPD | vel=%.0f km/h < %.0f — sem pré-acendimento, aguarda deteção local (T=1)",
                 speed, VEL_FADE_RAPIDO_KMH);
        return;
    }

    uint32_t fade_ms = _fade_ms_para_velocidade(speed);

    if (eta_ms == 0 || eta_ms <= fade_ms) {
        /* ETA demasiado curto para fade gradual — acende instantaneamente */
        g_fsm_acender_instantaneo = true;
        fsm_acender_em_ms_set(fsm_agora_ms() + eta_ms);
        ESP_LOGD(TAG, "[UDP] SPD | vel=%.0f ETA=%" PRIu32 "ms fade=%" PRIu32 "ms → INSTANTÂNEO",
                 speed, eta_ms, fade_ms);
    } else {
        /* Inicia fade com fade_ms de antecedência: completa exactamente na chegada */
        g_fsm_acender_instantaneo = false;
        uint32_t arranque_ms = eta_ms - fade_ms;
        fsm_acender_em_ms_set(fsm_agora_ms() + arranque_ms);
        ESP_LOGD(TAG, "[UDP] SPD | vel=%.0f ETA=%" PRIu32 "ms fade=%" PRIu32 "ms arranque=%" PRIu32 "ms → FADE GRADUAL",
                 speed, eta_ms, fade_ms, arranque_ms);
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
   Estende TC_TIMEOUT quando vizinho esquerdo notifica obstáculo.
   Sem este callback, B expirava Tc prematuramente mesmo com veículo parado em A.
   TC_TIMEOUT mantém-se activo (não é cancelado) para que, se o veículo
   desaparecer sem chegar a B, o timer expire e limpe Tc correctamente.
   O heartbeat periódico em fsm_timer._passo11b mantém o timeout renovado
   enquanto o obstáculo persistir em A. */
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

    /* Estende TC_TIMEOUT (não cancela) — se o veículo recuar/desaparecer sem chegar
       a este poste, o timeout estendido limpa Tc após TC_TIMEOUT_MS.
       Heartbeat periódico em fsm_timer.c mantém o timeout vivo enquanto o
       obstáculo persistir no poste anterior. */
    fsm_tc_timeout_ms_set(fsm_agora_ms() + TC_TIMEOUT_MS);
    ESP_LOGW(TAG, "  TC_TIMEOUT estendido +%llus (veículo parado em A)",
             (unsigned long long)(TC_TIMEOUT_MS / 1000ULL));

    ESP_LOGW(TAG, "  Tc mantém-se=%d (aguarda chegada ou TC_TIMEOUT)", g_fsm_Tc);
    ESP_LOGW(TAG, "  Luz ACESA — apaga quando veículo chegar ou timeout expirar");
    ESP_LOGW(TAG, "═══════════════════════════════════════");
}


/* ── Gestão de vizinhos ───────────────────────────────────── */

void sm_on_right_neighbor_offline(void)
{
    if (!g_fsm_right_online) return;
    g_fsm_right_online        = false;
    fsm_acender_em_ms_set(0);
    g_fsm_acender_instantaneo = false;

    portENTER_CRITICAL(&g_fsm_counters_mux);
    bool had_tc   = (g_fsm_Tc > 0);
    bool had_env  = (g_fsm_enviados_dir > 0);
    int  snap_env = g_fsm_enviados_dir;
    g_fsm_Tc           = 0;
    g_fsm_enviados_dir = 0;
    /* Decrementa T pelo nº de TC_INC sem confirmação: veículos que saíram da zona
       local mas o vizinho direito não confirmou (PASSED nunca chegou).
       Sem isto, _passo9 vê env_dir=0 e não decrementa T → luz acesa indefinidamente. */
    if (had_env) {
        if (g_fsm_T >= snap_env) g_fsm_T -= snap_env;
        else                     g_fsm_T  = 0;
    }
    portEXIT_CRITICAL(&g_fsm_counters_mux);

    if (had_tc)  ESP_LOGW(TAG, "Vizinho dir. OFFLINE — Tc resetado");
    if (had_env) ESP_LOGW(TAG, "Vizinho dir. OFFLINE — env_dir resetado (T-=%d)", snap_env);
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

            portENTER_CRITICAL(&g_fsm_counters_mux);
            bool local_tc_dec = (g_fsm_Tc > 0);
            if (local_tc_dec) g_fsm_Tc--;
            if (g_fsm_T < MAX_RADAR_TARGETS) g_fsm_T++;
            portEXIT_CRITICAL(&g_fsm_counters_mux);

            if (local_tc_dec) {
                comm_notify_prev_passed(vel);
                ESP_LOGI(TAG, "[T/Tc] ID=%u vindo da esq. — PASSED enviado", vehicle_id);
                _print_passed_enviado_box(vehicle_id, g_fsm_T, g_fsm_Tc);
            } else {
                ESP_LOGI(TAG, "[T/Tc] ID=%u local directo — sem PASSED", vehicle_id);
            }

            if (g_fsm_state != STATE_LIGHT_ON &&
                g_fsm_state != STATE_OBSTACULO) {
                g_fsm_state = STATE_LIGHT_ON;
            }

            bool tc_inc_enviado = false;
            if (g_fsm_right_online) {
                if ((vehicle_id != g_fsm_tc_last_vehicle_id ||
                     g_fsm_enviados_dir == 0) &&
                    g_fsm_enviados_dir < MAX_RADAR_TARGETS) {
                    /* env_dir nunca deve exceder MAX_RADAR_TARGETS: o vizinho
                       direito rejeita Tc além desse limite (Tc no máximo,
                       ver on_tc_inc_received) — sem este limite aqui, env_dir
                       crescia sem parar e o valor mostrado no LCD (que usa
                       max(T, env_dir) para reter o "1" até confirmação)
                       mostrava um número maior do que o vizinho alguma vez
                       poderia confirmar. */
                    comm_send_tc_inc(vel, x_mm);
                    comm_send_spd(vel, x_mm);
                    portENTER_CRITICAL(&g_fsm_counters_mux);
                    g_fsm_enviados_dir++;
                    portEXIT_CRITICAL(&g_fsm_counters_mux);
                    g_fsm_tc_last_vehicle_id = vehicle_id;
                    fsm_tc_timeout_ms_set(fsm_agora_ms() + TC_TIMEOUT_MS);
                    ESP_LOGI(TAG, "[T/Tc] TC_INC → B | ID=%u env_dir=%d",
                            vehicle_id, g_fsm_enviados_dir);
                    tc_inc_enviado = true;
                } else {
                    ESP_LOGD(TAG, "[T/Tc] TC_INC suprimido — ID=%u ainda em transito (env_dir=%d)",
                            vehicle_id, g_fsm_enviados_dir);
                }
            }

            char nL_ip[MAX_IP_LEN] = {0}, nR_ip[MAX_IP_LEN] = {0};
            udp_manager_get_neighbors(nL_ip, nR_ip);
            _print_deteccao_box(vehicle_id, vel, g_fsm_T, g_fsm_Tc, tc_inc_enviado, nR_ip);
            break;


        case SM_EVT_VEHICLE_PASSED:
            ESP_LOGI(TAG, "[SAÍDA] ID=%u | T=%d Tc=%d env_dir=%d",
                    vehicle_id, g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);

            g_fsm_last_detect_ms = fsm_agora_ms();
            fsm_acender_em_ms_set(0);

            if (vehicle_id == g_fsm_tc_last_vehicle_id) {
                g_fsm_tc_last_vehicle_id = 0;
            }

            /* T-- imediato em todos os casos — evita T preso se PASSED do vizinho
               direito nunca chegar (Tc=0 no direito quando veículo chega rápido).
               on_prev_passed_received gere enviados_dir mas já não faz T--. */
            portENTER_CRITICAL(&g_fsm_counters_mux);
            if (g_fsm_T > 0) g_fsm_T--;
            /* Só agenda apagar quando T=0, Tc=0 E não há TC_INC por confirmar
               (env_dir=0). Decisão explícita: sem esta última condição, a luz
               apagava mesmo sem o vizinho direito ter confirmado a chegada do
               veículo — agora só apaga com confirmação real (PASSED) ou se
               nunca chegou a enviar-se nada (env_dir já era 0). */
            bool passed_all_clear = (g_fsm_T == 0 && g_fsm_Tc == 0 && g_fsm_enviados_dir == 0);
            portEXIT_CRITICAL(&g_fsm_counters_mux);

            if (g_fsm_right_online) comm_send_spd(vel, x_mm);
            if (passed_all_clear) fsm_agendar_apagar();

            _print_saida_box(vehicle_id, g_fsm_T, g_fsm_Tc, g_fsm_enviados_dir);
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

            portENTER_CRITICAL(&g_fsm_counters_mux);
            if (is_new_vehicle && g_fsm_T < MAX_RADAR_TARGETS) g_fsm_T++;
            bool obs_tc_dec = (g_fsm_Tc > 0);
            if (obs_tc_dec) g_fsm_Tc--;
            portEXIT_CRITICAL(&g_fsm_counters_mux);

            if (is_new_vehicle) {
                g_fsm_tc_last_vehicle_id = vehicle_id;
                ESP_LOGW(TAG, "  T++ → %d (veículo parado contado)", g_fsm_T);
            } else {
                ESP_LOGD(TAG, "  T mantém-se (ID=%u já contado)", vehicle_id);
            }

            if (obs_tc_dec) {
                comm_notify_prev_passed(vel);
                ESP_LOGW(TAG, "  Tc-- → %d | PASSED enviado à esquerda", g_fsm_Tc);
            } else {
                ESP_LOGW(TAG, "  Tc=0 (obstáculo local, não veio da esq.)");
            }

            if (g_fsm_right_online) {
                if ((is_new_vehicle || g_fsm_enviados_dir == 0) &&
                    g_fsm_enviados_dir < MAX_RADAR_TARGETS) {
                    comm_send_tc_inc(vel, x_mm);
                    portENTER_CRITICAL(&g_fsm_counters_mux);
                    g_fsm_enviados_dir++;
                    portEXIT_CRITICAL(&g_fsm_counters_mux);
                    fsm_tc_timeout_ms_set(fsm_agora_ms() + TC_TIMEOUT_MS);
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
