/* ============================================================
   MÓDULO     : state_machine
   FICHEIRO   : state_machine.c — Implementação
   VERSÃO     : 5.0  |  2026-04-24
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

  
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
   CONSTANTES LOCAIS DE TEMPORIZAÇÃO
   ──────────────────────────────────────────────────────────
   TC_TIMEOUT_MS, T_STUCK_TIMEOUT_MS, AUTONOMO_DELAY_MS e
   OBSTACULO_REMOVE_MS estão definidos em system_config.h.
   Aqui apenas constantes de debounce do radar.
============================================================ */

/* Ciclos (100ms) sem frame do radar para declarar FAIL */
#define RADAR_FAIL_COUNT    50

/* Ciclos consecutivos com frame para declarar radar recuperado */
#define RADAR_OK_COUNT       3

/* Intervalo do heartbeat MASTER_CLAIM (ms) */
#define MASTER_CLAIM_HB_MS  30000ULL


/* ============================================================
   ESTADO INTERNO DA FSM
   ─────────────────────────────────────────────────────────
   Todas as variáveis são locais a este módulo.
   Acesso exclusivo pela fsm_task, excepto callbacks UDP
   (on_tc_inc_received, on_prev_passed_received) que podem
   ser chamados do contexto da udp_task.
   NOTA: callbacks UDP são chamados com frequência baixa
   (1/evento), logo a ausência de mutex é aceitável no ESP32.
============================================================ */

static system_state_t s_state          = STATE_IDLE;
static int            s_T              = 0;    /* veículos detectados localmente */
static int            s_Tc             = 0;    /* veículos a caminho (UDP)        */
static float          s_last_speed     = 0.0f;
static bool           s_apagar_pend    = false;
static bool           s_radar_ok       = true;
static int            s_radar_fail_cnt = 0;
static int            s_radar_ok_cnt   = 0;
static bool           s_right_online   = true;

static uint64_t s_last_detect_ms    = 0;  /* timestamp da última detecção    */
static uint64_t s_left_offline_ms   = 0;  /* timestamp desde viz. esq. off   */
static uint64_t s_tc_timeout_ms     = 0;  /* deadline de segurança Tc        */
static bool     s_left_was_offline  = false;
static uint64_t s_acender_em_ms     = 0;  /* timestamp de pré-acendimento    */
static uint64_t s_master_claim_ms   = 0;  /* timestamp do último MASTER_CLAIM*/
static uint64_t s_sem_vizinho_ms    = 0;  /* timestamp desde sem vizinhos    */
static uint64_t s_obstaculo_last_ms = 0;  /* timestamp da última detecção de obstáculo */


/* ============================================================
   SIMULADOR FÍSICO — USE_RADAR == 0
   ─────────────────────────────────────────────────────────
   O simulador gera carros virtuais com velocidades realistas
   para testes em bancada sem hardware de radar físico.
   Ciclo: AGUARDA → ENTRAR → EM_VIA → DETECTADO → SAIU → AGUARDA
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

/* Velocidades de rotação para variar os testes */
static const float s_vels[]  = {30.0f, 50.0f, 80.0f, 50.0f};
static int         s_vel_idx = 0;

/* Intervalo entre carros simulados: TRAFIC_TIMEOUT + 3s */
#define SIM_INTERVALO_MS  ((uint64_t)(TRAFIC_TIMEOUT_MS + 3000))
/* Distância de detecção simulada em mm */
#define SIM_ZONA_MM       ((float)(RADAR_DETECT_M * 1000))

static uint64_t _agora_ms(void);  /* declaração antecipada */

/* Inicializa mutex do simulador — chamar uma vez */
void sim_init_mutex(void)
{
    if (!s_sim_mutex)
        s_sim_mutex = xSemaphoreCreateMutex();
}

/* Lança carro simulado a vel km/h com posição lateral x_mm */
static void _sim_lancar(float vel, int16_t x_mm)
{
    float dist_extra = (float)((POSTE_DIST_M - RADAR_MAX_M) * 1000);
    if (dist_extra < 0.0f) dist_extra = 0.0f;

    /* Posição lateral aleatória se x_mm=0, ou usa o valor fornecido */
    s_sim.x_mm      = (x_mm == 0)
                    ? (float)((int32_t)(esp_random() % 769) - 384)
                    : (float)x_mm;
    s_sim.y_mm        = (float)RADAR_MAX_MM + dist_extra;
    s_sim.vel_kmh     = vel;
    s_sim.vy          = (vel / 3.6f) * 100.0f * 0.25f;  /* pixels/ciclo */
    s_sim.estado      = SIM_ENTRAR;
    s_sim.injectado   = false;
    s_sim.t_inicio_ms = _agora_ms();
    ESP_LOGI(TAG, "[SIM] Carro lançado | pos=%d | %.0f km/h | x=%.0fmm",
             POST_POSITION, vel, s_sim.x_mm);
}

/* Notifica o simulador que um carro está a chegar via UDP */
void sim_notificar_chegada(float vel_kmh, int16_t x_mm)
{
    if (!s_sim_mutex) return;
    xSemaphoreTake(s_sim_mutex, portMAX_DELAY);
    /* Só lança se não houver carro já em voo */
    if (s_sim.estado == SIM_AGUARDA ||
        s_sim.estado == SIM_SAIU    ||
        s_sim.estado == SIM_ENTRAR)
        _sim_lancar(vel_kmh, x_mm);
    xSemaphoreGive(s_sim_mutex);
}

/* Actualiza posição do carro simulado — chamar a cada 100ms */
void _sim_update(void)
{
    if (!s_sim_mutex) return;
    xSemaphoreTake(s_sim_mutex, portMAX_DELAY);
    uint64_t agora = _agora_ms();

    switch (s_sim.estado) {
        case SIM_AGUARDA:
            /* Só lança auto-carro se for MASTER */
            if (s_state != STATE_MASTER) break;
            if (!s_sim.t_inicio_ms) s_sim.t_inicio_ms = agora;
            if ((agora - s_sim.t_inicio_ms) >= SIM_INTERVALO_MS) {
                float vel = s_vels[s_vel_idx];
                s_vel_idx = (s_vel_idx + 1) % 4;
                _sim_lancar(vel, 0);
            }
            break;

        case SIM_ENTRAR:
            /* Carro a aproximar-se, ainda fora do alcance do radar */
            s_sim.y_mm -= s_sim.vy;
            if (s_sim.y_mm <= (float)RADAR_MAX_MM) {
                s_sim.y_mm   = (float)RADAR_MAX_MM;
                s_sim.estado = SIM_EM_VIA;
            }
            break;

        case SIM_EM_VIA:
        case SIM_DETECTADO:
            /* Carro dentro do alcance — avança e injeta evento quando na zona */
            s_sim.y_mm -= s_sim.vy;
            if (!s_sim.injectado && s_sim.y_mm <= SIM_ZONA_MM) {
                s_sim.estado    = SIM_DETECTADO;
                s_sim.injectado = true;
                xSemaphoreGive(s_sim_mutex);
                sm_on_radar_detect(s_sim.vel_kmh);
                xSemaphoreTake(s_sim_mutex, portMAX_DELAY);
            }
            /* Carro saiu do campo */
            if (s_sim.y_mm <= 0.0f) {
                s_sim.y_mm        = 0.0f;
                s_sim.estado      = SIM_SAIU;
                s_sim.t_inicio_ms = agora;
            }
            break;

        case SIM_SAIU:
            /* Aguarda apagamento antes de reiniciar ciclo */
            if (s_T == 0 && s_Tc == 0 &&
                (agora - s_sim.t_inicio_ms) >= (uint64_t)TRAFIC_TIMEOUT_MS) {
                s_sim.estado      = SIM_AGUARDA;
                s_sim.t_inicio_ms = agora;
            }
            break;
    }
    xSemaphoreGive(s_sim_mutex);
}

/* Copia posição do carro simulado para o display */
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


/* ============================================================
   UTILITÁRIOS INTERNOS
============================================================ */

/* Retorna tempo actual em milissegundos */
static uint64_t _agora_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

/* Agenda apagamento após TRAFIC_TIMEOUT — redefine o timer */
static void _agendar_apagar(void)
{
    s_apagar_pend    = true;
    s_last_detect_ms = _agora_ms();
}

/* Monitoriza saúde do radar com debounce bidirecional.
   Evita flapping: exige RADAR_FAIL_COUNT ciclos para declarar falha
   e RADAR_OK_COUNT ciclos para declarar recuperação. */
static void _verificar_radar(bool teve_frame, bool comm_ok)
{
    if (teve_frame) {
        s_radar_fail_cnt = 0;
        s_radar_ok_cnt++;
        /* Radar recuperado após RADAR_OK_COUNT frames consecutivos */
        if (!s_radar_ok && s_radar_ok_cnt >= RADAR_OK_COUNT) {
            s_radar_ok = true;
            ESP_LOGI(TAG, "Radar recuperado após %d frames", RADAR_OK_COUNT);
            /* Sai de SAFE MODE imediatamente quando radar volta */
            if (s_state == STATE_SAFE_MODE)
                s_state = STATE_IDLE;
        }
    } else {
        s_radar_ok_cnt = 0;
        s_radar_fail_cnt++;
        /* Radar em falha após RADAR_FAIL_COUNT ciclos sem frame */
        if (s_radar_ok && s_radar_fail_cnt >= RADAR_FAIL_COUNT) {
            s_radar_ok = false;
            ESP_LOGW(TAG, "Radar FAIL após %d ciclos — Passo 10 decide estado",
                     RADAR_FAIL_COUNT);
        }
    }
    (void)comm_ok; /* Reservado para uso futuro */
}


/* ============================================================
   CALLBACKS UDP
   ─────────────────────────────────────────────────────────
   Substituem versões weak do udp_manager.
   Chamados a partir do contexto da udp_task (Core 0).
   Escrevem em variáveis escalares — thread-safe no ESP32
   para escritas de 32 bits alinhadas em Xtensa LX6.
============================================================ */

/* Recebe anúncio de veículo a caminho — incrementa Tc */
void on_tc_inc_received(float speed, int16_t x_mm)
{
    s_apagar_pend    = false;
    s_last_speed     = speed;
    s_Tc++;
    s_last_detect_ms = _agora_ms();
    /* Timeout de segurança: se o veículo não chegar, Tc reseta */
    s_tc_timeout_ms  = _agora_ms() + TC_TIMEOUT_MS;

    ESP_LOGI(TAG, "[TC_INC] %.0f km/h | x=%dmm | T=%d Tc=%d",
             speed, (int)x_mm, s_T, s_Tc);

#if USE_RADAR == 0
    /* Em modo simulado, lança carro no simulador */
    sim_notificar_chegada(speed, x_mm);
#endif

    /* Propaga para vizinho direito se disponível */
    if (s_right_online) {
        comm_send_tc_inc(speed, x_mm);
        comm_send_spd(speed, x_mm);
    }
}

/* Recebe confirmação do vizinho direito que o objecto chegou lá.
   PROTOCOLO T/Tc CORRECTO:
   - Este callback é chamado no Poste A quando o Poste B detectou
     o objecto localmente e enviou PASSED de volta.
   - Decrementa T (o objecto já não está neste poste).
   - NÃO decrementa Tc — o Tc já foi decrementado quando B detectou
     localmente (SM_EVT_VEHICLE_LOCAL faz Tc--).
   - Propaga PASSED para o Poste anterior a este (cadeia). */
void on_prev_passed_received(void)
{
    if (s_T > 0) s_T--;
    s_acender_em_ms = 0;
    ESP_LOGI(TAG, "[PASSED recebido] T=%d Tc=%d — propaga para viz. esq.", s_T, s_Tc);
    /* Propaga a confirmação de passagem para o poste ainda mais à esquerda
       (para cadeia com mais de 2 postes). */
    comm_notify_prev_passed(s_last_speed);
    _agendar_apagar();
}

/* Recebe actualização de velocidade e ETA para pré-acendimento */
void on_spd_received(float speed, uint32_t eta_ms, int16_t x_mm)
{
    s_last_speed = speed;
    if (s_Tc <= 0) return;  /* Ignora se não há veículos anunciados */

    /* Agenda pré-acendimento com margem de antecipação */
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

/* Recebe anúncio de novo MASTER na cadeia */
void on_master_claim_received(int from_id)
{
    ESP_LOGI(TAG, "[MASTER_CLAIM] de ID=%d — cedemos liderança", from_id);
    /* A gestão efectiva é feita no Passo 9 do update */
}


/* ============================================================
   sm_process_event — processamento de eventos do pipeline
   ─────────────────────────────────────────────────────────
   Ponto central de entrada event-driven da FSM.
   Cada evento transporta: tipo, ID do veículo, velocidade, ETA.

   Eventos:
     DETECTED    → T++, luz ON, agenda ETA
     APPROACHING → pré-acendimento, propaga para vizinho direito
     PASSED      → T--, Tc--, avisa vizinho esquerdo, agenda apagar
     LOCAL       → Tc--, T++, luz ON, propaga se direito online
     OBSTACULO   → STATE_OBSTACULO, luz LIGHT_MAX
============================================================ */
void sm_process_event(sm_event_type_t type,
                      uint16_t vehicle_id,
                      float vel,
                      uint32_t eta_ms,
                      int16_t x_mm)
{
    switch (type) {

        /* ── Veículo confirmado pelo tracking_manager ────────── */
        case SM_EVT_VEHICLE_DETECTED:
            ESP_LOGI(TAG, "[EVT] DETECTED id=%u vel=%.1f km/h eta=%lums",
                     vehicle_id, vel, (unsigned long)eta_ms);

            if (s_T < 3) s_T++;
            s_apagar_pend    = false;
            s_last_detect_ms = _agora_ms();
            s_last_speed     = vel;

            /* Calcula timestamp de pré-acendimento baseado no ETA */
            if (eta_ms > MARGEM_ACENDER_MS)
                s_acender_em_ms = _agora_ms() + (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
            else
                s_acender_em_ms = _agora_ms();

            /* Transição para LIGHT_ON apenas se estava em repouso */
            if (s_state == STATE_IDLE   ||
                s_state == STATE_MASTER ||
                s_state == STATE_AUTONOMO) {
                s_state = STATE_LIGHT_ON;
                dali_fade_up(vel);
            }
            break;

       /* ── Veículo em aproximação activa ──────────────────── */
        case SM_EVT_VEHICLE_APPROACHING:
            ESP_LOGI(TAG, "[EVT] APPROACHING id=%u vel=%.1f km/h eta=%lums",
                     vehicle_id, vel, (unsigned long)eta_ms);

            /* ETA muito pequeno ou zero → acende imediatamente */
            if (eta_ms <= MARGEM_ACENDER_MS || eta_ms == 0) {
                if (s_state == STATE_IDLE   ||
                    s_state == STATE_MASTER ||
                    s_state == STATE_AUTONOMO) {
                    s_state = STATE_LIGHT_ON;
                }
                s_acender_em_ms = 0;
            } else {
                /* Agenda pré-acendimento com margem */
                s_acender_em_ms = _agora_ms() + (uint64_t)(eta_ms - MARGEM_ACENDER_MS);
            }

            s_last_speed     = vel;
            s_last_detect_ms = _agora_ms();
            s_apagar_pend    = false;
            /* Sem propagação UDP — APPROACHING só agenda pré-acendimento local.
               TC_INC é enviado exclusivamente por EVT_LOCAL e EVT_DETECTED. */
            
            break;

        /* ── Veículo saiu da zona do radar ─────────────────── */
        case SM_EVT_VEHICLE_PASSED:
            /* PROTOCOLO T/Tc CORRECTO:
               Poste A NÃO decrementa o seu próprio T aqui.
               O objecto saiu do sensor mas ainda está entre postes.
               T mantém-se a 1 — luz fica acesa — até o Poste B
               confirmar a chegada via on_prev_passed_received().
               Só então T é decrementado neste poste. */
            ESP_LOGI(TAG, "[EVT] PASSED id=%u vel=%.1f km/h | T=%d mantido até confirm. viz. dir.",
                     vehicle_id, vel, s_T);

            s_acender_em_ms = 0;

            /* Avisa vizinho direito que o objecto está a caminho.
               Quando o vizinho o detectar localmente, enviará PASSED
               de volta → on_prev_passed_received() decrementa T aqui. */
            if (s_right_online)
                comm_send_tc_inc(vel, x_mm);

            /* Agenda apagamento — só actua quando T=0 e Tc=0.
               Como T ainda é 1, o apagamento só acontece após
               on_prev_passed_received() decrementar T. */
            _agendar_apagar();
            break;

        /* ── Veículo detectado localmente pelo radar ─────────── */
        case SM_EVT_VEHICLE_LOCAL:
            /* PROTOCOLO T/Tc CORRECTO:
               O Poste B detectou o objecto localmente.
               T++ neste poste (objecto aqui).
               Tc-- se havia Tc pendente (o anunciado chegou).
               OBRIGATÓRIO: envia PASSED ao Poste A (viz. esquerdo)
               para que o Poste A decremente o seu T.
               Sem este PASSED, o T do Poste A ficaria preso a 1
               indefinidamente até ao TC_TIMEOUT. */
            s_acender_em_ms  = 0;
            if (s_Tc > 0) s_Tc--;
            if (s_T  < 3) s_T++;
            s_last_speed     = vel;
            s_last_detect_ms = _agora_ms();
            s_apagar_pend    = false;

            /* Transição para LIGHT_ON apenas se estava em repouso */
            if (s_state == STATE_IDLE   ||
                s_state == STATE_MASTER ||
                s_state == STATE_AUTONOMO) {
                s_state = STATE_LIGHT_ON;
                dali_fade_up(vel);
            }

            ESP_LOGI(TAG, "[EVT] LOCAL vel=%.1f km/h T=%d Tc=%d",
                     vel, s_T, s_Tc);

            /* Confirma ao Poste A que o objecto chegou (T-- no A) */
            comm_notify_prev_passed(vel);

            /* Propaga para vizinho direito se disponível */
            if (s_right_online) {
                comm_send_tc_inc(vel, x_mm);
                comm_send_spd(vel, x_mm);
            }
            break;

        /* ── Obstáculo estático detectado ───────────────────── */
        case SM_EVT_VEHICLE_OBSTACULO:
            ESP_LOGW(TAG, "[EVT] OBSTACULO id=%u vel=%.1f km/h", vehicle_id, vel);

            /* Regista timestamp para timeout de remoção automática */
            s_obstaculo_last_ms = _agora_ms();

            /* Activa apenas se não estiver já em OBSTACULO ou SAFE MODE */
            if (s_state != STATE_OBSTACULO &&
                s_state != STATE_SAFE_MODE) {
                s_state = STATE_OBSTACULO;
                dali_set_brightness(LIGHT_MAX);
                ESP_LOGW(TAG, "[OBSTACULO] CONFIRMADO id=%u — luz máxima", vehicle_id);
            }
            break;

        default:
            ESP_LOGW(TAG, "[EVT] Tipo desconhecido: %d", type);
            break;
    }
}


/* ============================================================
   sm_on_radar_detect — shim de compatibilidade v3.x
   ─────────────────────────────────────────────────────────
   Mantido por retrocompatibilidade com o simulador USE_RADAR=0.
   Delega para sm_process_event(SM_EVT_VEHICLE_LOCAL).
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
   GESTÃO DE VIZINHOS
============================================================ */

/* Vizinho direito ficou offline — cancela Tc e agenda apagamento */
void sm_on_right_neighbor_offline(void)
{
    if (!s_right_online) return;
    s_right_online  = false;
    s_acender_em_ms = 0;
    if (s_Tc > 0) {
        s_Tc = 0;
        ESP_LOGW(TAG, "Vizinho dir. OFFLINE — Tc=0");
    }
    _agendar_apagar();
}

/* Vizinho direito voltou online */
void sm_on_right_neighbor_online(void)
{
    if (s_right_online) return;
    s_right_online = true;
    ESP_LOGI(TAG, "Vizinho direito voltou online");
}


/* ============================================================
   state_machine_init — inicialização
============================================================ */
void state_machine_init(void)
{
    s_state             = STATE_IDLE;
    s_T                 = 0;
    s_Tc                = 0;
    s_last_speed        = 0.0f;
    s_apagar_pend       = false;
    s_radar_ok          = true;
    s_radar_fail_cnt    = 0;
    s_radar_ok_cnt      = 0;
    s_right_online      = true;
    s_acender_em_ms     = 0;
    s_master_claim_ms   = 0;
    s_sem_vizinho_ms    = 0;
    s_obstaculo_last_ms = 0;
    s_left_was_offline  = false;
    s_left_offline_ms   = 0;
    s_tc_timeout_ms     = 0;
    s_last_detect_ms    = 0;

#if USE_RADAR == 0
    sim_init_mutex();
#endif

    ESP_LOGI(TAG, "FSM v5.0 inicializada — estado IDLE (event-driven)");
}


/* ============================================================
   state_machine_update — ciclo de manutenção a 100ms
   ─────────────────────────────────────────────────────────
   Executa passos de manutenção periódica independentes dos
   eventos de detecção: timers, saúde do radar, gestão
   de vizinhos e transições de estado de rede.

   NÃO recebe dados de radar directamente.
   O estado de saúde do radar vem como parâmetro externo
   (escrito atomicamente pela radar_task).
============================================================ */
void state_machine_update(bool comm_ok, bool is_master, bool radar_teve_frame)
{
    uint64_t agora = _agora_ms();

    /* ── Passo 1: Simulador físico (modo sem radar físico) ────── */
#if USE_RADAR == 0
    _sim_update();
    radar_teve_frame = true;  /* Simulador nunca perde frames */
#endif

    /* ── Passo 2: Saúde do radar ──────────────────────────────── */
    _verificar_radar(radar_teve_frame, comm_ok);

    /* ── Passo 3: Conectividade do vizinho direito ────────────── */
    bool dir_ok = comm_right_online();
    if (!dir_ok &&  s_right_online) sm_on_right_neighbor_offline();
    if ( dir_ok && !s_right_online) sm_on_right_neighbor_online();

    /* ── Passo 4: Vizinho esquerdo — detecção de offline ──────── */
    bool esq_ok = comm_left_online();
    if (!esq_ok && !s_left_was_offline) {
        /* Esquerdo acabou de ficar offline */
        s_left_was_offline = true;
        s_left_offline_ms  = agora;
    }
    if (esq_ok && s_left_was_offline) {
        /* Esquerdo voltou — cede MASTER se não for o poste 0 */
        s_left_was_offline = false;
        if (s_state == STATE_MASTER && POST_POSITION > 0) {
            s_state = STATE_IDLE;
            comm_send_master_claim();
        }
    }

    /* ── Passo 5: T preso quando vizinho esquerdo está offline ─── */
    /* Evita T infinito quando não há quem envie PASSED */
    if (s_left_was_offline && s_T > 0 &&
        (agora - s_left_offline_ms) > T_STUCK_TIMEOUT_MS) {
        s_T = 0;
        ESP_LOGW(TAG, "T forçado a 0 — viz. esq. offline há %llus",
                 (unsigned long long)(T_STUCK_TIMEOUT_MS / 1000));
    }

    /* ── Passo 6: Timer de pré-acendimento (ETA) ──────────────── */
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

    /* ── Passo 7: Timeout de apagamento ───────────────────────── */
    /* Apaga apenas quando T=0, Tc=0 e passou o timeout de tráfego */
    if (s_apagar_pend && s_T == 0 && s_Tc == 0 &&
        (agora - s_last_detect_ms) >= TRAFIC_TIMEOUT_MS) {
        s_apagar_pend   = false;
        s_last_speed    = 0.0f;
        s_acender_em_ms = 0;
        /* Volta ao estado de repouso correcto */
        if (s_state == STATE_LIGHT_ON  ||
            s_state == STATE_AUTONOMO  ||
            s_state == STATE_OBSTACULO) {
            bool era_autonomo = (s_state == STATE_AUTONOMO);
            if (is_master)          s_state = STATE_MASTER;
            else if (era_autonomo)  s_state = STATE_AUTONOMO;
            else                    s_state = STATE_IDLE;
            dali_fade_down();
            ESP_LOGI(TAG, "Apagamento — volta a %s",
                     is_master    ? "MASTER"   :
                     era_autonomo ? "AUTONOMO" : "IDLE");
        }
    }

    /* ── Passo 8: Remoção automática de OBSTÁCULO ─────────────── */
    /* Sem detecções por OBSTACULO_REMOVE_MS → obstáculo desapareceu */
    if (s_state == STATE_OBSTACULO && s_obstaculo_last_ms > 0 &&
        (agora - s_obstaculo_last_ms) >= OBSTACULO_REMOVE_MS) {
        s_state             = is_master ? STATE_MASTER : STATE_IDLE;
        s_obstaculo_last_ms = 0;
        dali_fade_down();
        ESP_LOGW(TAG, "[OBSTACULO] REMOVIDO — sem detecção há %us",
                 OBSTACULO_REMOVE_MS / 1000);
    }

    /* ── Passo 9: Timeout de segurança Tc ─────────────────────── */
    /* Se o veículo anunciado não chegou dentro do prazo, reset Tc */
    if (s_Tc > 0 && s_tc_timeout_ms > 0 && agora > s_tc_timeout_ms) {
        ESP_LOGW(TAG, "Timeout Tc — veículo não chegou (Tc=%d→0)", s_Tc);
        s_Tc = 0; s_tc_timeout_ms = 0;
        _agendar_apagar();
    }

    /* ── Passo 10: Gestão do papel MASTER ──────────────────────── */
    /* Transições IDLE ↔ MASTER baseadas no papel actual */
    if ( is_master && s_state == STATE_IDLE)   s_state = STATE_MASTER;
    if (!is_master && s_state == STATE_MASTER) {
        s_state = STATE_IDLE;
        ESP_LOGI(TAG, "MASTER cedido");
    }

    /* Vizinho esq. offline há AUTONOMO_DELAY_MS → assume MASTER temporário */
    if (!is_master && s_left_was_offline && s_radar_ok &&
        (agora - s_left_offline_ms) > AUTONOMO_DELAY_MS &&
        s_state == STATE_IDLE) {
        s_state = STATE_MASTER;
        ESP_LOGW(TAG, "MASTER temporário — viz. esq. offline");
    }

    /* Vizinho esq. voltou — cede MASTER se não for o poste 0 */
    if (!is_master && !s_left_was_offline &&
        s_state == STATE_MASTER && POST_POSITION > 0) {
        s_state = STATE_IDLE;
        ESP_LOGI(TAG, "Viz. esq. voltou — cede MASTER → IDLE");
    }

    /* ── Passo 11: Gestão de estados degradados ────────────────── */
    bool tem_vizinho_esq = comm_left_online();

    /* SAFE MODE: sem radar E (sem WiFi OU sem vizinho esquerdo)
       Prioridade máxima — luz a 50% para segurança dos utentes */
    bool cego_total = !s_radar_ok && (!comm_ok || !tem_vizinho_esq);
    if (cego_total &&
        s_state != STATE_SAFE_MODE &&
        s_state != STATE_AUTONOMO  &&
        s_state != STATE_LIGHT_ON  &&
        s_state != STATE_OBSTACULO) {
        s_state = STATE_SAFE_MODE;
        dali_set_brightness(LIGHT_SAFE_MODE);
        ESP_LOGW(TAG, "SAFE MODE — radar KO (wifi=%d viz_esq=%d)",
                 comm_ok, tem_vizinho_esq);
    }

    /* Sai de SAFE MODE quando recupera radar OU (WiFi + vizinho esq.) */
    if (s_state == STATE_SAFE_MODE &&
        (s_radar_ok || (comm_ok && tem_vizinho_esq))) {
        s_state = STATE_IDLE;
        ESP_LOGI(TAG, "Saiu de SAFE MODE → IDLE");
    }

    /* AUTONOMO: radar OK, WiFi OK mas sem vizinhos após AUTONOMO_DELAY_MS */
    bool tem_vizinho = comm_left_online() || comm_right_online();
    if (s_radar_ok && comm_ok && !tem_vizinho) {
        if (s_sem_vizinho_ms == 0) s_sem_vizinho_ms = agora;
        if ((agora - s_sem_vizinho_ms) > AUTONOMO_DELAY_MS &&
            s_state != STATE_SAFE_MODE &&
            s_state != STATE_LIGHT_ON  &&
            s_state != STATE_OBSTACULO) {
            s_state = STATE_AUTONOMO;
            ESP_LOGW(TAG, "AUTONOMO — sem vizinhos");
        }
    } else {
        s_sem_vizinho_ms = 0;
    }

    /* Sai de AUTONOMO quando WiFi volta E tem vizinhos */
    if (s_state == STATE_AUTONOMO && comm_ok && tem_vizinho) {
        s_sem_vizinho_ms = 0;
        s_state = is_master ? STATE_MASTER : STATE_IDLE;
    }

    /* Sem WiFi + radar OK → AUTONOMO imediato (não SAFE MODE) */
    if (!comm_ok && s_radar_ok && s_state != STATE_SAFE_MODE &&
        s_state != STATE_LIGHT_ON && s_state != STATE_OBSTACULO)
        s_state = STATE_AUTONOMO;

    /* ── Passo 12: Heartbeat MASTER_CLAIM ─────────────────────── */
    /* Poste 0 anuncia periodicamente a sua liderança */
    if (is_master && POST_POSITION == 0 &&
        (agora - s_master_claim_ms) >= MASTER_CLAIM_HB_MS) {
        s_master_claim_ms = agora;
        comm_send_master_claim();
    }
}


/* ============================================================
   GETTERS
============================================================ */

system_state_t state_machine_get_state(void)     { return s_state; }
int            state_machine_get_T(void)          { return s_T; }
int            state_machine_get_Tc(void)         { return s_Tc; }
float          state_machine_get_last_speed(void) { return s_last_speed; }
bool           state_machine_radar_ok(void)       { return s_radar_ok; }
bool           sm_is_obstaculo(void)              { return s_state == STATE_OBSTACULO; }

/* Injeta carro de teste via debugger (JTAG/GDB) */
void sm_inject_test_car(float vel)
{
    ESP_LOGI(TAG, "[TEST] Injecção de carro a %.0f km/h", vel);
    sm_process_event(SM_EVT_VEHICLE_LOCAL, 0, vel, 0, 0);
}

/* Converte estado da FSM para string legível */
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

