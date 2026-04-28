/* ============================================================
   STATE MACHINE — TASK INTERNA
   @file      state_machine_task.c
   @version   4.0  |  2026-04-17
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE DESTA TASK (fsm_task):
   ─────────────────────────────────────────
   - Chamar state_machine_update() a cada 100ms (Core 1, Prio 6).
   - Ler o estado de saúde do radar (s_radar_teve_frame) escrito
     pela radar_task via atomic — sem acesso directo à UART.
   - Actualizar o display com dados do tracking_manager.
   - Enviar heartbeat ao system_monitor.
   - Suportar injecção de carro de teste via debugger (JTAG/GDB).

   MUDANÇAS v3.1 → v4.0:
   ─────────────────────────
   - REMOVIDO: state_machine_update() deixou de ler radar interno.
     Assinatura actualizada: recebe radar_teve_frame como parâmetro.
   - ADICIONADO: s_radar_teve_frame — variável atómica partilhada
     entre radar_task (escrita) e fsm_task (leitura).
   - ADICIONADO: tracking_manager_task_notify_frame() —
     função chamada pela radar_task a cada frame válido do sensor.
   - ADICIONADO: _atualiza_radar_display() usa tracking_manager
     no modo USE_RADAR=1 em vez de radar_manager_get_objects().
   - MANTIDO: modo simulado USE_RADAR=0 via sim_get_objeto().
   - MANTIDO: delay de 6s com heartbeats durante arranque.
   - MANTIDO: injecção de carro via g_test_car / g_test_car_speed.

   SINCRONIZAÇÃO ENTRE TASKS:
   ────────────────────────────
   radar_task (Core 0, 10ms)  escrita → s_radar_teve_frame
   fsm_task   (Core 1, 100ms) leitura → s_radar_teve_frame

   ESP32 dual-core: acessos a bool são atómicos em Xtensa LX6
   (alinhado em 32 bits). Não é necessário mutex para este valor.
   Para garantir memória visível entre cores, usa-se
   atomic_store / atomic_load via <stdatomic.h>.

   LATÊNCIA TOTAL DO PIPELINE:
   ─────────────────────────────
   radar_task (10ms) → tracking_manager → atomic_bool
   fsm_task   (100ms) → state_machine_update → dali_manager
   Total máximo: 10ms + 100ms = 110ms (dentro do limite de 100ms
   para a FSM, mas o radar tem resolução de 10ms própria).
============================================================ */

#include "state_machine.h"
#include "tracking_manager.h"
#include "system_monitor.h"
#include "comm_manager.h"
#include "radar_manager.h"
#include "dali_manager.h"
#include "display_manager.h"
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
   Escrita pela radar_task a cada frame válido do radar.
   Lida pela fsm_task em state_machine_update().

   Uso de _Atomic bool (C11/stdatomic.h) garante visibilidade
   entre os dois cores do ESP32 sem necessidade de mutex.
============================================================ */
static _Atomic bool s_radar_teve_frame = false;

/* ============================================================
   VARIÁVEIS DE TESTE VIA DEBUGGER (JTAG/GDB)
   ─────────────────────────────────────────────
   Alterar em tempo real via GDB para injectar carro de teste:
     (gdb) set g_test_car = true
     (gdb) set g_test_car_speed = 80.0
============================================================ */
volatile bool  g_test_car       = false;
volatile float g_test_car_speed = 50.0f;


/* ============================================================
   tracking_manager_task_notify_frame
   ────────────────────────────────────
   Chamada pela radar_task após cada frame válido do radar.
   Regista que o radar está activo para a fsm_task.

   Deve ser chamada EM TODOS OS FRAMES — mesmo quando count=0 —
   para que a FSM consiga distinguir "radar sem alvos"
   de "radar completamente silencioso/morto".

   @param teve_dados  true se o frame tinha pelo menos 1 alvo válido.
                      false se o frame foi válido mas sem alvos.
============================================================ */
void tracking_manager_task_notify_frame(bool teve_dados)
{
    /* Escreve atomicamente — visível na fsm_task no Core 1 */
    atomic_store(&s_radar_teve_frame, teve_dados);
}


/* ============================================================
   _atualiza_radar_display
   ────────────────────────
   Lê dados de posição e envia ao display_manager (não-bloqueante).

   MODO USE_RADAR == 0 (simulado):
     Usa sim_get_objeto() da state_machine — compatível com v3.x.

   MODO USE_RADAR == 1 (real):
     Usa tracking_manager_get_vehicles() em vez de
     radar_manager_get_objects() — fornece IDs estáveis e
     velocidade suavizada, mais adequado para o display.
     Converte tracked_vehicle_t → radar_obj_t para o display_manager.

   NOTA: display_manager_set_radar() é não-bloqueante (fila interna).
   Se a fila estiver cheia, o frame é descartado — aceitável.
============================================================ */
static void _atualiza_radar_display(void)
{
#if USE_RADAR == 0
    /* Modo simulado: usa posição do carro virtual */
    float x = 0.0f, y = 0.0f;
    if (sim_get_objeto(&x, &y)) {
        radar_obj_t obj = {0};
        obj.x_mm      = (int)x;
        obj.y_mm      = (int)y;
        obj.speed_kmh = state_machine_get_last_speed();
        display_manager_set_radar(&obj, 1);
    } else {
        display_manager_set_radar(NULL, 0);
    }

#else
    /* Modo real v4.0: usa tracking_manager (IDs estáveis, vel. suavizada) */
    tracked_vehicle_t veiculos[TRK_MAX_VEHICLES];
    uint8_t count = 0;

    if (tracking_manager_get_vehicles(veiculos, &count) && count > 0) {
        radar_obj_t objs[TRK_MAX_VEHICLES];
        uint8_t n_objs = 0;
        bool em_obstaculo = sm_is_obstaculo();

        for (uint8_t i = 0; i < count && i < TRK_MAX_VEHICLES; i++) {
            tracked_vehicle_t *v = &veiculos[i];

            /* Ignora TENTATIVE — pode ser ruído */
            if (v->state == TRK_STATE_TENTATIVE) continue;

            if (em_obstaculo) {
                /* Obstáculo activo — mostra posição fixa sem movimento preditivo.
                   speed=0 garante que o interpolador do display não move o ponto. */
                if (v->state == TRK_STATE_CONFIRMED  ||
                    v->state == TRK_STATE_APPROACHING) {
                    objs[n_objs].x_mm      = (int)v->x_mm;
                    objs[n_objs].y_mm      = (int)v->y_mm;
                    objs[n_objs].speed_kmh = 0.0f;
                    n_objs++;
                }
            } else {
                /* Modo normal — mostra veículos em movimento */
                if (v->state == TRK_STATE_CONFIRMED  ||
                    v->state == TRK_STATE_APPROACHING) {
                    objs[n_objs].x_mm      = (int)v->x_mm;
                    objs[n_objs].y_mm      = (int)v->y_mm;
                    objs[n_objs].speed_kmh = v->speed_kmh;
                    n_objs++;
                }
            }
        }

        display_manager_set_radar(n_objs > 0 ? objs : NULL, n_objs);
    } else {
        display_manager_set_radar(NULL, 0);
    }
#endif
}


/* ============================================================
   fsm_task — Core 1, Prioridade 6, Stack 6144B
   ──────────────────────────────────────────────
   Ciclo principal a 100ms. Responsável por:
     1. Injecção de carro de teste (via debugger)
     2. Chamar state_machine_update() com parâmetros actuais
     3. Actualizar display com estado FSM e dados de tracking
     4. Enviar heartbeat ao system_monitor

   NÃO faz leitura de radar — isso é responsabilidade da radar_task.
   NÃO chama tracking_manager_update() — isso é da radar_task.
   Lê apenas o atomic_bool s_radar_teve_frame para saúde do radar.
============================================================ */
static void fsm_task(void *arg)
{
    ESP_LOGI(TAG, "fsm_task | Core %d | Prio 6 | aguarda 6s (radar)",
             xPortGetCoreID());

    /* ── Arranque: aguarda estabilização do HLK-LD2450 ─────────
       6s divididos em blocos de 200ms com heartbeat para evitar
       falsos warnings "FSM sem heartbeat" no system_monitor.
       (mantido igual à v3.1 — lógica correcta) */
    for (int i = 0; i < 30; i++) {
        system_monitor_heartbeat(MOD_FSM);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* Limpa buffer UART do radar antes de começar a processar */
    radar_flush_rx();
    ESP_LOGI(TAG, "fsm_task v4.0 activa — event-driven pipeline");

    while (1) {

        /* ── 1. Injecção de carro de teste via debugger ──────── */
        if (g_test_car) {
            g_test_car = false;
            /* v4.0: sm_inject_test_car delega para SM_EVT_VEHICLE_LOCAL */
            sm_inject_test_car(g_test_car_speed);
        }

        /* ── 2. Lê estado atómico do radar ───────────────────── */
        /* Escrito pela radar_task a cada frame (válido ou não).
           Repõe a false após leitura para detectar ausência de frames
           no próximo ciclo de 100ms da FSM. */
        bool radar_frame = atomic_exchange(&s_radar_teve_frame, false);

        /* ── 3. Ciclo da FSM ──────────────────────────────────── */
        /* v4.0: recebe radar_teve_frame como parâmetro externo.
           Não lê radar internamente. */
        state_machine_update(
            comm_status_ok(),
            comm_is_master(),
            radar_frame           /* ← NOVO parâmetro v4.0 */
        );
        /* ── 3.1 Lê eventos do tracking_manager → FSM ─────────── */
        tracked_vehicle_t veiculos[TRK_MAX_VEHICLES];
        uint8_t count = 0;
        if (tracking_manager_get_vehicles(veiculos, &count)) {
            for (uint8_t i = 0; i < count; i++) {
                tracked_vehicle_t *v = &veiculos[i];

                if (v->event_detected_pending)
                    sm_process_event(SM_EVT_VEHICLE_DETECTED,
                        v->id, v->speed_kmh, v->eta_ms, (int16_t)v->x_mm);

                if (v->event_approach_pending)
                    sm_process_event(SM_EVT_VEHICLE_APPROACHING,
                        v->id, v->speed_kmh, v->eta_ms, (int16_t)v->x_mm);

                if (v->event_passed_pending)
                    sm_process_event(SM_EVT_VEHICLE_PASSED,
                        v->id, v->speed_kmh, 0, (int16_t)v->x_mm);

                if (v->event_obstaculo_pending)
                    sm_process_event(SM_EVT_VEHICLE_OBSTACULO,
                        v->id, v->speed_kmh, 0, (int16_t)v->x_mm);

                /* Limpa todas as flags só depois de processar todos os eventos */
                tracking_manager_clear_events(v->id);
                
            }
        }

        /* ── 4. Sincroniza display ────────────────────────────── */
        /* Estado textual da FSM */
        display_manager_set_status(state_machine_get_state_name());

        /* Contadores T e Tc */
        display_manager_set_traffic(state_machine_get_T(),
                                    state_machine_get_Tc());

        /* Velocidade do último veículo detectado */
        display_manager_set_speed((int)state_machine_get_last_speed());

        /* Estado do hardware (radar + DALI) */
        display_manager_set_hardware(radar_get_status_str(),
                                     radar_is_connected(),
                                     (uint8_t)dali_get_brightness());

        /* Posições dos veículos rastreados (tracking_manager ou sim) */
        _atualiza_radar_display();

        /* ── 5. Heartbeat ao monitor ──────────────────────────── */
        system_monitor_heartbeat(MOD_FSM);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/* ============================================================
   state_machine_task_start — Core 1, Prio 6
   ───────────────────────────────────────────
   Inicializa o tracking_manager e cria a fsm_task.
   Deve ser chamada após radar_manager_task_start() para garantir
   que o tracking_manager é inicializado antes do primeiro frame.
============================================================ */
void state_machine_task_start(void)
{
    /* Inicializa o tracking_manager antes de qualquer frame */
    tracking_manager_init();

    xTaskCreatePinnedToCore(fsm_task, "fsm_task",
                            6144, NULL, 6, NULL, 1);

    ESP_LOGI(TAG, "fsm_task v4.0 criada | Core 1 | Prio 6 | Stack 6144");
}
