/* ============================================================
   MÓDULO     : state_machine
   FICHEIRO   : state_machine.c — Implementação
   VERSÃO     : 4.0  |  2026-04-17
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   MUDANÇA ARQUITECTURAL v3.1 → v4.0:
   ─────────────────────────────────────
   A FSM deixa de ler o radar directamente.
   Passa a ser puramente event-driven: consome system_event_t
   da queue do event_manager.

   ANTES (v3.x):
     state_machine_update() → radar_read_data_cached() → decide
     sm_on_radar_detect()   → chamada directa de radar_manager

   DEPOIS (v4.0):
     sm_process_event(VEHICLE_DETECTED, vel, id, eta)  → reage
     sm_process_event(VEHICLE_APPROACHING, vel, id, eta) → reage
     sm_process_event(VEHICLE_PASSED, vel, id)         → reage

   COMPATIBILIDADE:
   ─────────────────
   - sm_on_radar_detect() mantém-se como shim de compatibilidade
     durante a migração, chamando sm_process_event() internamente.
   - on_tc_inc_received(), on_prev_passed_received(), on_spd_received()
     mantêm-se como callbacks UDP (não mudam nesta fase).
   - A leitura de radar, filtros de distância/velocidade e detecção
     de obstáculo foram REMOVIDOS deste ficheiro — passam a ser
     responsabilidade de tracking_manager e event_manager.

   PIPELINE NOVO:
   ──────────────
     radar_manager → tracking_manager → event_manager → [queue]
                                                            │
                                          state_machine ←──┘
                                          (sm_process_event)
============================================================ */

#include "state_machine.h"
#include "comm_manager.h"
#include "dali_manager.h"
#include "system_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_random.h"
#include <string.h>

static const char *TAG = "FSM";

/* ============================================================
   CONSTANTES DE TEMPORIZAÇÃO
============================================================ */

/* Timeout para forçar T=0 quando o vizinho esquerdo está offline */
#define T_STUCK_TIMEOUT_MS    (TRAFIC_TIMEOUT_MS * 3)

/* Timeout de segurança de Tc: veículo anunciado não chegou */
#define TC_TIMEOUT_MS         (TRAFIC_TIMEOUT_MS * 2)

/* Nº de ciclos (100ms) sem frame do radar para declarar falha */
#define RADAR_FAIL_COUNT      50

/* Nº de ciclos consecutivos com frame para declarar radar recuperado */
#define RADAR_OK_COUNT         3

/* Intervalo do heartbeat MASTER_CLAIM em ms (30 segundos) */
#define MASTER_CLAIM_HB_MS    30000ULL

#define AUTONOMO_DELAY_MS       10000ULL
static uint64_t s_sem_vizinho_desde_ms = 0;
/* ============================================================
   ESTADO INTERNO DA FSM
   ─────────────────────
   Variáveis escalares — acesso exclusivo pela fsm_task,
   excepto onde indicado (callbacks UDP).
============================================================ */

static system_state_t s_state           = STATE_IDLE;
static int            s_T               = 0;   /* veículos detectados localmente */
static int            s_Tc              = 0;   /* veículos a caminho (UDP)       */
static float          s_last_speed      = 0.0f;
static bool           s_apagar_pend     = false;
static bool           s_radar_ok        = true;
static int            s_radar_fail_cnt  = 0;
static int            s_radar_ok_cnt    = 0;
static bool           s_right_online    = true;
static bool           s_radar_degradado = false;

static uint64_t s_last_detect_ms   = 0;
static uint64_t s_left_offline_ms  = 0;
static uint64_t s_tc_timeout_ms    = 0;
static bool     s_left_was_offline = false;
static uint64_t s_acender_em_ms    = 0;
static uint64_t s_master_claim_ms  = 0;


/* ============================================================
   SIMULADOR FÍSICO — USE_RADAR == 0
   ────────────────────────────────────
   O simulador mantém-se aqui na v4.0 por compatibilidade.
   Na v5.0 (após event_manager) será migrado para injectar
   eventos na queue em vez de chamar sm_on_radar_detect().
============================================================ */
#if USE_RADAR == 0

typedef enum {
    SIM_AGUARDA = 0, SIM_ENTRAR, SIM_EM_VIA, SIM_DETECTADO, SIM_SAIU
} sim_estado_t;

typedef struct {
    float        y_mm;
    float        x_mm;
    float        vy;
    float        vel_kmh;
    sim_estado_t estado;
    uint64_t     t_inicio_ms;
    bool         injectado;
} sim_carro_t;

static sim_carro_t       s_sim       = {0};
static SemaphoreHandle_t s_sim_mutex = NULL;

static const float s_vels[] = {30.0f, 50.0f, 80.0f, 50.0f};
static int         s_vel_idx = 0;

#define SIM_INTERVALO_MS  ((uint64_t)(TRAFIC_TIMEOUT_MS + 3000))
#define SIM_ZONA_MM       ((float)(RADAR_DETECT_M * 1000))

static uint64_t _agora_ms(void);  /* declaração antecipada */

void sim_init_mutex(void)
{
    if (!s_sim_mutex)
        s_sim_mutex = xSemaphoreCreateMutex();
}

static void _sim_lancar(float vel, int16_t x_mm)
{
    float dist_extra = (float)((POSTE_DIST_M - RADAR_MAX_M) * 1000);
    if (dist_extra < 0.0f) dist_extra = 0.0f;

    s_sim.x_mm    = (x_mm == 0)
                  ? (float)((int32_t)(esp_random() % 769) - 384)
                  : (float)x_mm;
    s_sim.y_mm        = (float)RADAR_MAX_MM + dist_extra;
    s_sim.vel_kmh     = vel;
    s_sim.vy          = (vel / 3.6f) * 100.0f * 0.25f;
    s_sim.estado      = SIM_ENTRAR;
    s_sim.injectado   = false;
    s_sim.t_inicio_ms = _agora_ms();
    ESP_LOGI(TAG, "[SIM] Carro lançado | pos=%d | %.0f km/h | x=%.0fmm",
             POST_POSITION, vel, s_sim.x_mm);
}

void sim_notificar_chegada(float vel_kmh, int16_t x_mm)
{
    if (!s_sim_mutex) return;
    xSemaphoreTake(s_sim_mutex, portMAX_DELAY);
    if (s_sim.estado == SIM_AGUARDA ||
        s_sim.estado == SIM_SAIU    ||
        s_sim.estado == SIM_ENTRAR)
        _sim_lancar(vel_kmh, x_mm);
    xSemaphoreGive(s_sim_mutex);
}

void _sim_update(void)
{
    if (!s_sim_mutex) return;
    xSemaphoreTake(s_sim_mutex, portMAX_DELAY);
    uint64_t agora = _agora_ms();

    switch (s_sim.estado) {
        case SIM_AGUARDA:
            if (s_state != STATE_MASTER) break;
            if (!s_sim.t_inicio_ms) s_sim.t_inicio_ms = agora;
            if ((agora - s_sim.t_inicio_ms) >= SIM_INTERVALO_MS) {
                float vel = s_vels[s_vel_idx];
                s_vel_idx = (s_vel_idx + 1) % 4;
                _sim_lancar(vel, 0);
            }
            break;

        case SIM_ENTRAR:
            s_sim.y_mm -= s_sim.vy;
            if (s_sim.y_mm <= (float)RADAR_MAX_MM) {
                s_sim.y_mm   = (float)RADAR_MAX_MM;
                s_sim.estado = SIM_EM_VIA;
            }
            break;

        case SIM_EM_VIA:
        case SIM_DETECTADO:
            s_sim.y_mm -= s_sim.vy;
            if (!s_sim.injectado && s_sim.y_mm <= SIM_ZONA_MM) {
                s_sim.estado    = SIM_DETECTADO;
                s_sim.injectado = true;
                xSemaphoreGive(s_sim_mutex);
                /* NOTA v4.0: Na v5.0 isto vai injectar na queue
                   em vez de chamar sm_on_radar_detect() directamente */
                sm_on_radar_detect(s_sim.vel_kmh);
                xSemaphoreTake(s_sim_mutex, portMAX_DELAY);
            }
            if (s_sim.y_mm <= 0.0f) {
                s_sim.y_mm        = 0.0f;
                s_sim.estado      = SIM_SAIU;
                s_sim.t_inicio_ms = agora;
            }
            break;

        case SIM_SAIU:
            if (s_T == 0 && s_Tc == 0 &&
                (agora - s_sim.t_inicio_ms) >= (uint64_t)TRAFIC_TIMEOUT_MS) {
                s_sim.estado      = SIM_AGUARDA;
                s_sim.t_inicio_ms = agora;
            }
            break;
    }
    xSemaphoreGive(s_sim_mutex);
}

bool sim_get_objeto(float *x_mm, float *y_mm)
{
    if (!s_sim_mutex || !x_mm || !y_mm) return false;
    xSemaphoreTake(s_sim_mutex, portMAX_DELAY);
    bool visivel = (s_sim.estado == SIM_EM_VIA ||
                    s_sim.estado == SIM_DETECTADO) &&
                   s_sim.y_mm > 0.0f &&
                   s_sim.y_mm <= (float)RADAR_MAX_MM;
    if (visivel) { *x_mm = s_sim.x_mm; *y_mm = s_sim.y_mm; }
    xSemaphoreGive(s_sim_mutex);
    return visivel;
}

#endif /* USE_RADAR == 0 */


/* ── Utilitário de tempo ──────────────────────────────────── */
static uint64_t _agora_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

/* ── Agenda apagamento após TRAFIC_TIMEOUT ────────────────── */
static void _agendar_apagar(void)
{
    s_apagar_pend    = true;
    s_last_detect_ms = _agora_ms();
}


/* ============================================================
   CALLBACKS UDP
   ─────────────
   Mantidos intactos da v3.1. Substituem versões weak do
   comm_manager. Chamados a partir do contexto UDP.
============================================================ */

void on_tc_inc_received(float speed, int16_t x_mm)
{
    s_apagar_pend    = false;
    s_last_speed     = speed;
    s_Tc++;
    s_last_detect_ms = _agora_ms();
    s_tc_timeout_ms  = _agora_ms() + TC_TIMEOUT_MS;

    ESP_LOGI(TAG, "[TC_INC] %.0f km/h | x=%dmm | T=%d Tc=%d",
             speed, (int)x_mm, s_T, s_Tc);

#if USE_RADAR == 0
    sim_notificar_chegada(speed, x_mm);
#endif

    if (s_right_online) {
        comm_send_tc_inc(speed, x_mm);
        comm_send_spd(speed, x_mm);
    }
}

void on_prev_passed_received(void)
{
    if (s_T  > 0) s_T--;
    if (s_Tc > 0) s_Tc--;
    s_acender_em_ms = 0;
    ESP_LOGI(TAG, "[PASSED] T=%d Tc=%d", s_T, s_Tc);
    comm_notify_prev_passed(s_last_speed);
    _agendar_apagar();
}

void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm)
{
    s_last_speed = speed;
    if (s_Tc <= 0) return;

    if (eta_ms > MARGEM_ACENDER_MS)
        s_acender_em_ms = _agora_ms() + (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
    else
        s_acender_em_ms = _agora_ms();

    ESP_LOGI(TAG, "[SPD] %.0f km/h | ETA=%lums | x=%dmm",
             speed, (unsigned long)eta_ms, (int)x_mm);

#if USE_RADAR == 0
    sim_notificar_chegada(speed, x_mm);
#endif
}

void on_master_claim_received(int from_id)
{
    ESP_LOGI(TAG, "[MASTER_CLAIM] de ID=%d", from_id);
}


/* ============================================================
   sm_process_event  ← NOVO em v4.0
   ──────────────────────────────────
   Ponto central de entrada de eventos do pipeline.
   Chamado pelo event_manager após desencapsular um system_event_t
   da queue FreeRTOS.

   Substitui a leitura directa do radar pela FSM.
   Cada evento transporta: tipo, ID do veículo, velocidade, ETA.

   @param type      Tipo de evento (SM_EVT_*)
   @param vehicle_id ID estável do veículo (do tracking_manager)
   @param vel        Velocidade suavizada em km/h
   @param eta_ms     ETA calculado pelo tracking_manager (ms)
   @param x_mm       Posição lateral do veículo em mm
============================================================ */
void sm_process_event(sm_event_type_t type,
                      uint16_t vehicle_id,
                      float vel,
                      uint32_t eta_ms,
                      int16_t x_mm)
{
    switch (type) {

        /* ── Veículo confirmado pelo tracking_manager ──────── */
        case SM_EVT_VEHICLE_DETECTED:
            ESP_LOGI(TAG, "[EVT] DETECTED id=%u vel=%.1f km/h eta=%lums",
                     vehicle_id, vel, (unsigned long)eta_ms);

            if (s_T < 3) s_T++;
            s_apagar_pend    = false;
            s_last_detect_ms = _agora_ms();
            s_last_speed     = vel;

            /* Calcula pré-acendimento com base no ETA */
            if (eta_ms > MARGEM_ACENDER_MS)
                s_acender_em_ms = _agora_ms() + (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
            else
                s_acender_em_ms = _agora_ms();

            /* Fade up apenas na transição — não repete se já em LIGHT_ON */
            if (s_state == STATE_IDLE   ||
                s_state == STATE_MASTER ||
                s_state == STATE_AUTONOMO) {
                s_state = STATE_LIGHT_ON;
                dali_fade_up(vel);
            }
            break;

        /* ── Veículo em aproximação activa ─────────────────── */
        case SM_EVT_VEHICLE_APPROACHING:
            ESP_LOGI(TAG, "[EVT] APPROACHING id=%u vel=%.1f km/h eta=%lums",
                     vehicle_id, vel, (unsigned long)eta_ms);

            /* Se ETA já passou ou é inferior à margem — acende já */
            if (eta_ms <= MARGEM_ACENDER_MS || eta_ms == 0) {
                if (s_state == STATE_IDLE   ||
                    s_state == STATE_MASTER ||
                    s_state == STATE_AUTONOMO) {
                    s_state = STATE_LIGHT_ON;
                    dali_fade_up(vel);
                }
                s_acender_em_ms = 0;
            } else {
                /* Agenda pré-acendimento com margem de antecipação */
                s_acender_em_ms = _agora_ms() + (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
            }

            s_last_speed     = vel;
            s_last_detect_ms = _agora_ms();
            s_apagar_pend    = false;

            /* Propaga para vizinho direito se disponível */
            if (s_right_online) {
                comm_send_tc_inc(vel, x_mm);
                comm_send_spd(vel, x_mm);
            }
            break;

        /* ── Veículo saiu da zona ───────────────────────────── */
        case SM_EVT_VEHICLE_PASSED:
            ESP_LOGI(TAG, "[EVT] PASSED id=%u vel=%.1f km/h",
                     vehicle_id, vel);

            if (s_T  > 0) s_T--;
            if (s_Tc > 0) s_Tc--;
            s_acender_em_ms = 0;

            /* Avisa vizinho esquerdo que o objecto passou */
            comm_notify_prev_passed(vel);
            /* Agenda apagamento após TRAFIC_TIMEOUT_MS */
            _agendar_apagar();
            break;

        /* ── Veículo detectado localmente pelo radar ─────────── */
        case SM_EVT_VEHICLE_LOCAL:
            s_acender_em_ms  = 0;
            if (s_Tc > 0) s_Tc--;
            if (s_T  < 3) s_T++;
            s_last_speed     = vel;
            s_last_detect_ms = _agora_ms();
            s_apagar_pend    = false;

            /* Fade up apenas na transição — não repete se já em LIGHT_ON */
            if (s_state == STATE_IDLE   ||
                s_state == STATE_MASTER ||
                s_state == STATE_AUTONOMO) {
                s_state = STATE_LIGHT_ON;
                dali_fade_up(vel);
            }

            ESP_LOGI(TAG, "[EVT] LOCAL vel=%.1f km/h T=%d Tc=%d",
                     vel, s_T, s_Tc);

            /* Propaga para vizinho direito se disponível
               Em AUTONOMO s_right_online=false — não propaga */
            if (s_right_online) {
                comm_send_tc_inc(vel, x_mm);
                comm_send_spd(vel, x_mm);
            }
            /* NOTA: comm_notify_prev_passed() NÃO é chamado aqui
               — só é chamado em PASSED quando o objecto sai */
            break;

        /* ── Obstáculo estático detectado ───────────────────── */
        case SM_EVT_VEHICLE_OBSTACULO:
            ESP_LOGW(TAG, "[EVT] OBSTACULO id=%u vel=%.1f km/h",
                     vehicle_id, vel);

            /* Activa apenas se não estiver já em OBSTACULO ou SAFE MODE */
            if (s_state != STATE_OBSTACULO &&
                s_state != STATE_SAFE_MODE) {
                s_state = STATE_OBSTACULO;
                dali_set_brightness(LIGHT_MAX);
                ESP_LOGW(TAG, "STATE_OBSTACULO — luz maxima");
            }
            break;

        default:
            ESP_LOGW(TAG, "[EVT] Tipo desconhecido: %d", type);
            break;
    }
}


/* ============================================================
   sm_on_radar_detect  — SHIM de compatibilidade v3.x → v4.0
   ─────────────────────────────────────────────────────────────
   Mantido para compatibilidade durante a migração.
   Na v5.0 (com event_manager completo), esta função deixa
   de ser chamada directamente e pode ser removida.

   Delega para sm_process_event(SM_EVT_VEHICLE_LOCAL, ...).
============================================================ */
void sm_on_radar_detect(float vel)
{
    ESP_LOGD(TAG, "[SHIM] sm_on_radar_detect → SM_EVT_VEHICLE_LOCAL");

#if USE_RADAR == 0
    float fx = 0.0f, fy = 0.0f;
    sim_get_objeto(&fx, &fy);
    int16_t x_mm = (int16_t)fx;
#else
    int16_t x_mm = 0;
#endif

    sm_process_event(SM_EVT_VEHICLE_LOCAL, 0, vel, 0, x_mm);
}


/* ============================================================
   Gestão de vizinhos — mantida igual à v3.1
============================================================ */

void sm_on_right_neighbor_offline(void)
{
    if (!s_right_online) return;
    s_right_online  = false;
    s_acender_em_ms = 0;
    if (s_Tc > 0) {
        s_Tc = 0;
        ESP_LOGW(TAG, "Dir. OFFLINE — Tc=0");
    }
    _agendar_apagar();
}

void sm_on_right_neighbor_online(void)
{
    if (s_right_online) return;
    s_right_online = true;
    ESP_LOGI(TAG, "Vizinho direito voltou online");
}


/* ============================================================
   _verificar_radar
   ─────────────────
   Monitoriza saúde do radar com debounce bidirecional.
   NOTA v4.0: Em vez de receber `teve_frame` directamente do radar,
   recebe um bool de saúde do tracking_manager (que sabe se teve
   frames válidos recentemente). A lógica interna mantém-se igual.
============================================================ */
static void _verificar_radar(bool teve_frame, bool comm_ok)
{
    if (teve_frame) {
        s_radar_fail_cnt = 0;
        s_radar_ok_cnt++;
        if (!s_radar_ok && s_radar_ok_cnt >= RADAR_OK_COUNT) {
            s_radar_ok        = true;
            s_radar_degradado = false;
            ESP_LOGI(TAG, "Radar recuperado");
            if (s_state == STATE_SAFE_MODE)
                s_state = STATE_IDLE;
        }
    } else {
        s_radar_ok_cnt = 0;
        s_radar_fail_cnt++;
        if (s_radar_ok && s_radar_fail_cnt >= RADAR_FAIL_COUNT) {
            s_radar_ok        = false;
            s_radar_degradado = true;
            ESP_LOGW(TAG, "Radar FAIL após %d leituras — PASSO 10 decide estado", RADAR_FAIL_COUNT);
        }       
    }
}


/* ============================================================
   state_machine_init
============================================================ */
void state_machine_init(void)
{
    s_state           = STATE_IDLE;
    s_T = s_Tc        = 0;
    s_last_speed      = 0.0f;
    s_apagar_pend     = false;
    s_radar_ok        = true;
    s_right_online    = true;
    s_acender_em_ms   = 0;
    s_master_claim_ms = 0;

#if USE_RADAR == 0
    sim_init_mutex();
#endif

    ESP_LOGI(TAG, "FSM v4.0 inicializada — estado IDLE (event-driven)");
}


/* ============================================================
   state_machine_update  — ciclo de manutenção a 100ms
============================================================ */

void state_machine_update(bool comm_ok, bool is_master,bool radar_teve_frame)
{
    uint64_t agora = _agora_ms();

    /* ── PASSO 1: Simulador (modo sem radar físico) ───────── */
#if USE_RADAR == 0
    _sim_update();
    radar_teve_frame = true; /* Simulador nunca perde frames */
#endif

    /* ── PASSO 2: Saúde do radar ──────────────────────────── */
    _verificar_radar(radar_teve_frame, comm_ok);

    /* ── PASSO 3: Conectividade do vizinho direito ──────────── */
    bool dir_ok = comm_right_online();
    if (!dir_ok &&  s_right_online) sm_on_right_neighbor_offline();
    if ( dir_ok && !s_right_online) sm_on_right_neighbor_online();

    /* ── PASSO 4: Vizinho esquerdo offline → MASTER ─────────── */
    bool esq_ok = comm_left_online();
    if (!esq_ok && !s_left_was_offline) {
        s_left_was_offline = true;
        s_left_offline_ms  = agora;
    }
    if (esq_ok && s_left_was_offline) {
        s_left_was_offline = false;
        if (s_state == STATE_MASTER && POST_POSITION > 0) {
            s_state = STATE_IDLE;
            comm_send_master_claim();
        }
    }

    /* ── PASSO 5: T preso com esquerdo offline ──────────────── */
    if (s_left_was_offline && s_T > 0 &&
        (agora - s_left_offline_ms) > T_STUCK_TIMEOUT_MS) {
        s_T = 0;
        ESP_LOGW(TAG, "T forçado a 0 (viz. esq. offline)");
    }

    /* ── PASSO 6: Timer de pré-acendimento (ETA) ──────────── */
    if (s_acender_em_ms > 0 && agora >= s_acender_em_ms) {
        s_acender_em_ms = 0;
        if (s_Tc > 0) {
            dali_fade_up(s_last_speed);
            if (s_state == STATE_IDLE   ||
                s_state == STATE_MASTER ||
                s_state == STATE_AUTONOMO)
                s_state = STATE_LIGHT_ON;
            ESP_LOGI(TAG, "Pré-acendimento: luz ON (ETA atingido)");
        }
    }

/* ── PASSO 7: Timeout para apagar ─────────────────────── */
    if (s_apagar_pend) {
        ESP_LOGI(TAG, "APAGAR PEND: T=%d Tc=%d elapsed=%llums",
                 s_T, s_Tc, (agora - s_last_detect_ms));
    }
    if (s_apagar_pend && s_T == 0 && s_Tc == 0 &&
        (agora - s_last_detect_ms) >= TRAFIC_TIMEOUT_MS) {
        s_apagar_pend   = false;
        s_last_speed    = 0.0f;
        s_acender_em_ms = 0;
        if (s_state == STATE_LIGHT_ON  ||
            s_state == STATE_AUTONOMO  ||
            s_state == STATE_OBSTACULO) {
            bool era_autonomo = (s_state == STATE_AUTONOMO);
            if (is_master)
                s_state = STATE_MASTER;
            else if (era_autonomo)
                s_state = STATE_AUTONOMO;
            else
                s_state = STATE_IDLE;
            dali_fade_down();
            ESP_LOGI(TAG, "Apagamento — volta a %s",
                     is_master    ? "MASTER"   :
                     era_autonomo ? "AUTONOMO" : "IDLE");
        }
    }

    /* ── PASSO 8: Timeout de segurança Tc ────────────────────── */
    if (s_Tc > 0 && s_tc_timeout_ms > 0 && agora > s_tc_timeout_ms) {
        ESP_LOGW(TAG, "Timeout Tc — carro não chegou (Tc=%d→0)", s_Tc);
        s_Tc = 0; s_tc_timeout_ms = 0;
        _agendar_apagar();
    }

    /* ── PASSO 9: Gestão do papel MASTER ─────────────────────── */
    if ( is_master && s_state == STATE_IDLE) s_state = STATE_MASTER;
    if (!is_master && s_state == STATE_MASTER) {
        s_state = STATE_IDLE;
        ESP_LOGI(TAG, "MASTER cedido");
    }

    /* Viz. esq. offline há mais de AUTONOMO_DELAY_MS → assume MASTER temporário
       Cobre cenário 5: Poste A falha, Poste B assume liderança da linha */
    if (!is_master && s_left_was_offline && s_radar_ok &&
        (agora - s_left_offline_ms) > AUTONOMO_DELAY_MS &&
        s_state == STATE_IDLE) {
        s_state = STATE_MASTER;
        ESP_LOGW(TAG, "MASTER temporário — viz. esq. offline há %lus",(unsigned long)(AUTONOMO_DELAY_MS / 1000));
    }

    /* Viz. esq. voltou online — cede MASTER se não for o poste 0 */
    if (!is_master && !s_left_was_offline &&
        s_state == STATE_MASTER && POST_POSITION > 0) {
        s_state = STATE_IDLE;
        ESP_LOGI(TAG, "Viz. esq. voltou — cede MASTER → IDLE");
    }

    /* ── PASSO 10: Gestão de estados degradados ─────────────── */
     bool tem_vizinho_esq = comm_left_online();

        /* SAFE MODE — cego total:
        Sem radar E (sem WiFi OU sem vizinho esquerdo) */
        bool cego_total = !s_radar_ok && (!comm_ok || !tem_vizinho_esq);
        if (cego_total &&
            s_state != STATE_SAFE_MODE &&
            s_state != STATE_AUTONOMO  &&
            s_state != STATE_LIGHT_ON  &&
            s_state != STATE_OBSTACULO) {
            s_state = STATE_SAFE_MODE;
            dali_set_brightness(LIGHT_SAFE_MODE);
            ESP_LOGW(TAG, "SAFE MODE — radar KO e sem info (wifi=%d viz_esq=%d)",comm_ok, tem_vizinho_esq);
        }

        /* Sai de SAFE MODE quando recupera radar OU (WiFi + vizinho esquerdo) */
        if (s_state == STATE_SAFE_MODE &&
            (s_radar_ok || (comm_ok && tem_vizinho_esq))) {
            s_state = STATE_IDLE;
            ESP_LOGI(TAG, "Saiu de SAFE MODE → IDLE");
        }

    /* AUTONOMO — só após tempo suficiente para DISCOVER completar */
        bool tem_vizinho = comm_left_online() || comm_right_online();

        if (s_radar_ok && comm_ok && !tem_vizinho) {
            if (s_sem_vizinho_desde_ms == 0)
                s_sem_vizinho_desde_ms = agora;
            if ((agora - s_sem_vizinho_desde_ms) > AUTONOMO_DELAY_MS &&
                s_state != STATE_SAFE_MODE &&
                s_state != STATE_LIGHT_ON  &&
                s_state != STATE_OBSTACULO)
            {
                s_state = STATE_AUTONOMO;
                ESP_LOGW(TAG, "AUTONOMO — sem vizinhos há %lus",(unsigned long)(AUTONOMO_DELAY_MS / 1000));
            }
        } else {
            s_sem_vizinho_desde_ms = 0;
        }

        /* Sai de AUTONOMO quando WiFi volta E tem vizinhos */
        if (s_state == STATE_AUTONOMO && comm_ok && tem_vizinho) {
            s_sem_vizinho_desde_ms = 0;
            s_state = is_master ? STATE_MASTER : STATE_IDLE;
        }

        /* Sem WiFi + radar OK → AUTONOMO imediato */
        if (!comm_ok && s_radar_ok && s_state != STATE_SAFE_MODE)
            s_state = STATE_AUTONOMO;

        /* ── PASSO 11: Heartbeat MASTER_CLAIM ───────────────────── */
        if (is_master && POST_POSITION == 0 &&
            (agora - s_master_claim_ms) >= MASTER_CLAIM_HB_MS) {
            s_master_claim_ms = agora;
            comm_send_master_claim();
        }
    }


/* ── Getters ──────────────────────────────────────────────── */
system_state_t state_machine_get_state(void)     { return s_state; }
int            state_machine_get_T(void)          { return s_T; }
int            state_machine_get_Tc(void)         { return s_Tc; }
float          state_machine_get_last_speed(void) { return s_last_speed; }
bool           state_machine_radar_ok(void)       { return s_radar_ok; }
bool           sm_is_obstaculo(void) { return s_state == STATE_OBSTACULO; }

void sm_inject_test_car(float vel)
{
    ESP_LOGI(TAG, "[TEST] Injecção de carro a %.0f km/h", vel);
    sm_process_event(SM_EVT_VEHICLE_LOCAL, 0, vel, 0, 0);
}

const char *state_machine_get_state_name(void)
{
    switch (s_state) {
        case STATE_IDLE:      return "IDLE";
        case STATE_LIGHT_ON:  return "LIGHT ON";
        case STATE_SAFE_MODE: return "SAFE MODE";
        case STATE_MASTER:    return "MASTER";
        case STATE_AUTONOMO:  return "AUTONOMO";
        case STATE_OBSTACULO: return "OBSTACULO";
        default:              return "---";
    }
}
