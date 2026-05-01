/* ============================================================
   MÁQUINA DE ESTADOS — TASK PRINCIPAL
   @file      fsm_task.c
   @version   5.1  |  2026-04-30
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Task principal da FSM. Corre no Core 1 a 100ms.

   CICLO DE EXECUÇÃO:
   ───────────────────
     1. Lê estado atómico do radar (escrito pela radar_task)
     2. Chama state_machine_update() — timeouts e transições
     3. Consome eventos pendentes do tracking_manager
     3.5 Aplica brilho DALI com base no novo estado
     4. Actualiza display com alvos confirmados pelo radar
     5. Envia heartbeat ao system_monitor

   SEPARAÇÃO DE RESPONSABILIDADES:
   ─────────────────────────────────
     Esta task NÃO lê o radar directamente.
     Esta task NÃO chama tracking_manager_update().
     Esses passos são da exclusiva responsabilidade da radar_task
     (Core 0, Prio 5). A comunicação entre tasks é feita via
     atomic_bool (tracking_manager_task_notify_frame).

   HARDWARE DALI — PONTO ÚNICO DE CONTROLO:
   ──────────────────────────────────────────
     fsm_aplicar_luz() é o ÚNICO sítio em todo o sistema que
     chama dali_fade_up(), dali_fade_down(), dali_set_brightness()
     e dali_safe_mode(). Todos os outros módulos (fsm_events,
     fsm_timer, fsm_network) apenas mudam g_fsm_state — nunca
     chamam dali directamente.

   SINCRONIZAÇÃO ENTRE TASKS:
   ────────────────────────────
     radar_task (Core 0, 100ms) → tracking_manager_update()
                                → tracking_manager_task_notify_frame() [escrita atómica]
     fsm_task   (Core 1, 100ms) → lê atomic_bool → state_machine_update()
                                → tracking_manager_get_vehicles()
                                → sm_process_event() por evento
                                → fsm_aplicar_luz()

   DISPLAY — DADOS REAIS APENAS:
   ──────────────────────────────
     _atualiza_radar_display() envia ao display apenas alvos
     com estado TRK_STATE_CONFIRMED ou TRK_STATE_APPROACHING.
     Alvos TENTATIVE (instáveis) e COASTING (perdidos) são filtrados.
     Em modo OBSTACULO, a velocidade enviada é 0 — posição fixa.

   ARRANQUE:
   ──────────
     Aguarda 6 segundos com heartbeat contínuo para estabilização
     do HLK-LD2450. O sensor demora até 5s a inicializar o UART.
     Após o delay, limpa o buffer UART antes de processar frames.

   MUDANÇAS v5.0 → v5.1:
   ──────────────────────
     - ADICIONADO: fsm_aplicar_luz() — ponto único de controlo DALI.
     - ADICIONADO: s_ultimo_estado — guarda de transição de estado.
     - REMOVIDO: #include "radar_manager.h" duplicado.
     - CORRIGIDO: ordem dos blocos — fsm_aplicar_luz() definida
       antes de fsm_task() para evitar dependência da declaração
       antecipada.
============================================================ */

#include "fsm_core.h"
#include "fsm_events.h"
#include "state_machine.h"
#include "tracking_manager.h"
#include "display_manager.h"
#include "comm_manager.h"
#include "radar_manager.h"
#include "dali_manager.h"
#include "system_monitor.h"
#include "system_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdatomic.h>
#include <string.h>

static const char *TAG = "FSM_TASK";


/* ============================================================
   VARIÁVEL ATÓMICA PARTILHADA ENTRE TASKS
   ─────────────────────────────────────────
   Escrita pela radar_task a cada frame (válido ou inválido).
   Lida pela fsm_task para determinar saúde do radar.

   Uso de _Atomic bool (C11 / stdatomic.h) garante visibilidade
   imediata entre os dois cores do ESP32 (Xtensa LX6) sem mutex.
   atomic_store / atomic_exchange asseguram a barreira de memória.
============================================================ */
static _Atomic bool s_radar_teve_frame = false;




/* ============================================================
   _atualiza_radar_display
   ────────────────────────
   @brief Envia alvos confirmados pelo radar ao display_manager.

   FILTRAGEM:
     Só são enviados alvos com estado TRK_STATE_CONFIRMED ou
     TRK_STATE_APPROACHING. Alvos TENTATIVE (ruído instável) e
     COASTING (perdidos, apenas estimativa) são ignorados.
     Isto garante que o display reflecte apenas detecções reais.

   MODO OBSTÁCULO:
     Quando sm_is_obstaculo() é true, a velocidade é forçada a 0.
     O display_manager interpreta speed=0 como posição fixa —
     sem movimento preditivo. A posição real do obstáculo
     continua a ser actualizada a cada frame do radar.

   THREAD-SAFETY:
     tracking_manager_get_vehicles() usa mutex interno.
     display_manager_set_radar() é não-bloqueante (fila LVGL).
     Se a fila estiver cheia, o frame é descartado — aceitável,
     pois o próximo frame real chega em 100ms.
============================================================ */
static void _atualiza_radar_display(void)
{
    tracked_vehicle_t veiculos[TRK_MAX_VEHICLES];
    uint8_t count = 0;

    if (!tracking_manager_get_vehicles(veiculos, &count) || count == 0) {
        /* Sem veículos activos — limpa display */
        display_manager_set_radar(NULL, 0);
        return;
    }

    radar_obj_t alvos[RADAR_MAX_OBJ];
    uint8_t n_alvos      = 0;
    bool    em_obstaculo = sm_is_obstaculo();

    for (uint8_t i = 0; i < count && n_alvos < RADAR_MAX_OBJ; i++) {
        tracked_vehicle_t *v = &veiculos[i];

        /* Keepalive: mantém obstáculo vivo enquanto radar o detecta */
        if (em_obstaculo && v->obstaculo_frames >= OBSTACULO_MIN_FRAMES) {
            fsm_obstaculo_keepalive();
        }

        /* Só estados com detecção física confirmada */
        if (v->state != TRK_STATE_CONFIRMED &&
            v->state != TRK_STATE_APPROACHING) {
            continue;
        }

        alvos[n_alvos].x_mm      = (int)v->x_mm;
        alvos[n_alvos].y_mm      = (int)v->y_mm;
        /* Em modo obstáculo: velocidade=0 → posição fixa no display */
        alvos[n_alvos].speed_kmh = em_obstaculo ? 0.0f : v->speed_kmh;
        n_alvos++;
    }

    display_manager_set_radar(n_alvos > 0 ? alvos : NULL, n_alvos);
}


/* ============================================================
   _processa_eventos_tracking
   ───────────────────────────
   @brief Itera os veículos activos e injeta eventos pendentes na FSM.

   Cada veículo pode ter até 4 flags de evento activas simultaneamente.
   A ordem de processamento respeita a prioridade lógica:
     1. DETECTED   — primeiro avistamento (sem luz ainda)
     2. LOCAL      — veículo confirmado local (T++, muda estado, propaga)
     3. PASSING    — a aproximar-se (agenda pré-acendimento ETA)
     4. PASSED     — saiu do radar (T--, SPD ao vizinho direito)
     5. OBSTACULO  — veículo parado (STATE_OBSTACULO)

   Após consumir todos os eventos, chama tracking_manager_clear_events()
   para limpar as flags. A limpeza é feita APÓS todos os eventos para
   evitar perda de flags no caso de múltiplos eventos simultâneos.
============================================================ */
static void _processa_eventos_tracking(void)
{
    tracked_vehicle_t veiculos[TRK_MAX_VEHICLES];
    uint8_t count = 0;

    if (!tracking_manager_get_vehicles(veiculos, &count)) {
        return;
    }

    for (uint8_t i = 0; i < count; i++) {
        tracked_vehicle_t *v = &veiculos[i];

        /* Nenhum evento pendente — passa para o próximo */
        if (!v->event_detected_pending   &&
            !v->event_approach_pending   &&
            !v->event_passed_pending     &&
            !v->event_obstaculo_pending) {
            continue;
        }

        /* 1. Primeiro avistamento — prepara ETA, sem alterar T */
        if (v->event_detected_pending) {
            sm_process_event(SM_EVT_VEHICLE_DETECTED,
                             v->id, v->speed_kmh, v->eta_ms, (int16_t)v->x_mm);
        }

        /* 2. Veículo confirmado local — T++, muda estado, TC_INC + SPD ao vizinho */
        if (v->event_approach_pending) {
            sm_process_event(SM_EVT_VEHICLE_LOCAL,
                             v->id, v->speed_kmh, v->eta_ms, (int16_t)v->x_mm);
        }

        /* 3. Veículo saiu — T--, agenda apagamento, SPD ao vizinho direito */
        if (v->event_passed_pending) {
            sm_process_event(SM_EVT_VEHICLE_PASSED,
                             v->id, v->speed_kmh, 0, (int16_t)v->x_mm);
        }

        /* 4. Veículo parado — entra em STATE_OBSTACULO */
        if (v->event_obstaculo_pending) {
            sm_process_event(SM_EVT_VEHICLE_OBSTACULO,
                             v->id, v->speed_kmh, 0, (int16_t)v->x_mm);
        }

        /* Limpa flags APÓS consumir todos os eventos deste veículo */
        tracking_manager_clear_events(v->id);
    }
}


/* ============================================================
   fsm_aplicar_luz
   ──────────────────────────────────────────────────────────
   @brief  Aplica o nível de brilho correcto ao dali_manager
           com base no estado actual da FSM.

   Chamada UMA VEZ por ciclo de 100ms, após _processa_eventos_tracking().
   É o ÚNICO sítio em todo o sistema que chama dali_fade_up(),
   dali_fade_down(), dali_set_brightness() e dali_safe_mode().

   Só actua em mudanças de estado — s_ultimo_estado evita que
   dali_fade_up() seja reiniciado a cada 100ms enquanto o
   estado se mantém LIGHT_ON.

   PRIVADA — não exposta no .h. Uso exclusivo da fsm_task.
============================================================ */
static system_state_t s_ultimo_estado = STATE_IDLE;

static void fsm_aplicar_luz(void)
{
    system_state_t estado_actual = g_fsm_state;

    /* Só actua em mudanças de estado */
    if (estado_actual == s_ultimo_estado) return;

    switch (estado_actual) {

        case STATE_LIGHT_ON:
            /* Veículo em movimento — fade suave proporcional à velocidade */
            dali_fade_up(g_fsm_last_speed);
            break;

        case STATE_OBSTACULO:
            /* Obstáculo parado — luz máxima instantânea, sem fade */
            dali_set_brightness(LIGHT_MAX);
            break;

        case STATE_SAFE_MODE:
            /* Radar em falha — luz fixa a LIGHT_SAFE_MODE (50%) */
            dali_safe_mode();
            break;

        case STATE_IDLE:
        case STATE_MASTER:
        case STATE_AUTONOMO:
            /* Sem tráfego — fade down suave para LIGHT_MIN */
            dali_fade_down();
            break;

        default:
            break;
    }

    s_ultimo_estado = estado_actual;
    ESP_LOGD(TAG, "Luz: %s", state_machine_get_state_name());
}


/* ============================================================
   fsm_task — Core 1, Prioridade 6, Stack 6144B
   ──────────────────────────────────────────────
   Loop principal a 100ms.

   CICLO:
     1. Lê atomic_bool do radar (escrito pela radar_task)
     2. Chama state_machine_update() — timeouts e transições de estado
     3. Processa eventos do tracking_manager → FSM
     3.5 Aplica brilho DALI com base no novo estado
     4. Actualiza display com alvos confirmados
     5. Envia heartbeat ao system_monitor
     6. Aguarda 100ms
============================================================ */
static void fsm_task(void *arg)
{
    ESP_LOGI(TAG, "fsm_task | Core %d | Prio 6 | a aguardar estabilização do radar (6s)...",
             xPortGetCoreID());

    /* ── Arranque: aguarda estabilização do HLK-LD2450 ──────────
       O sensor demora até 5s a inicializar a comunicação UART.
       Dividido em 30 blocos de 200ms com heartbeat contínuo para
       evitar falsos alarmes "FSM sem heartbeat" no system_monitor. */
    for (int i = 0; i < 30; i++) {
        system_monitor_heartbeat(MOD_FSM);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* Limpa backlog UART acumulado durante o arranque */
    radar_flush_rx();
    g_fsm_radar_fail_cnt = 0;
    g_fsm_radar_ok_cnt   = 0;
    g_fsm_radar_ok       = true;   /* presume OK até prova em contrário */
    if (g_fsm_state == STATE_SAFE_MODE)
        g_fsm_state = STATE_IDLE;
    
    ESP_LOGI(TAG, "fsm_task v5.1 activa — pipeline radar real");

    while (1) {

        /* ── 1. Lê estado de saúde do radar ───────────────────── */
        /* atomic_exchange repõe a false — detecção de ausência
           de frames no próximo ciclo (→ SAFE_MODE após RADAR_FAIL_COUNT) */
        bool radar_frame = atomic_exchange(&s_radar_teve_frame, false);

        /* ── 2. Ciclo de manutenção da FSM ────────────────────── */
        /* Gere timeouts, transições de estado, MASTER_CLAIM, etc. */
        bool comm_ok   = comm_status_ok();
        bool is_master = comm_is_master();
        state_machine_update(comm_ok, is_master, radar_frame);

        /* ── 3. Eventos do tracking → FSM ─────────────────────── */
        _processa_eventos_tracking();

        /* ── 3.5 Aplica brilho com base no novo estado ────────── */
        /* Ponto único de controlo DALI — só actua em transições. */
        fsm_aplicar_luz();

        /* ── 4. Actualiza display com dados reais do radar ───────*/
        _atualiza_radar_display();

        /* ── 5. Heartbeat ao system_monitor ───────────────────── */
        system_monitor_heartbeat(MOD_FSM);

        /* ── 6. Aguarda próximo ciclo de 100ms ────────────────── */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/* ============================================================
   state_machine_task_start
   ─────────────────────────
   @brief Inicializa o tracking_manager e cria a fsm_task.

   Ordem obrigatória:
     1. tracking_manager_init() — antes da radar_task arrancar
     2. xTaskCreatePinnedToCore() — Core 1, Prio 6

   NOTA: tracking_manager_init() é chamado AQUI e não no
   system_monitor para garantir que o tracking está pronto
   antes do primeiro frame da radar_task. A radar_task é
   criada depois em system_monitor_start().
============================================================ */
void state_machine_task_start(void)
{
    /* Inicializa o tracking antes de qualquer frame do radar */
    tracking_manager_init();
    ESP_LOGI(TAG, "tracking_manager inicializado");

    xTaskCreatePinnedToCore(
        fsm_task,
        "fsm_task",
        6144,
        NULL,
        6,
        NULL,
        1   /* Core 1 — APP_CPU */
    );

    ESP_LOGI(TAG, "fsm_task v5.1 | Core 1 | Prio 6 | Stack 6144B");
}
