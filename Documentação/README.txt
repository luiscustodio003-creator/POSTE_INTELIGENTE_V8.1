===============================================================================
   POSTE INTELIGENTE v8 — COMPONENTE state_machine
   README.txt  |  Versão 5.0  |  2026-04-29
   Autores: Luis Custódio | Tiago Moreno
===============================================================================


===============================================================================
DESCRIÇÃO
===============================================================================

  Máquina de estados principal do Poste Inteligente.
  Gere o protocolo T/Tc, o controlo de brilho DALI, a liderança
  dinâmica (MASTER) e os estados especiais (SAFE_MODE, AUTONOMO,
  OBSTACULO) com base em eventos reais do radar HLK-LD2450.

  NOTA v5.0 — SIMULAÇÃO COMPLETAMENTE REMOVIDA:
    Os ficheiros fsm_sim.c e fsm_sim.h foram eliminados do projecto.
    O sistema opera exclusivamente com dados reais do radar.
    A flag USE_RADAR e todos os blocos #if USE_RADAR foram removidos
    deste componente. O componente radar_manager inicializa sempre
    em RADAR_MODE_UART.


===============================================================================
FICHEIROS
===============================================================================

  state_machine.h  — Interface pública (única a incluir externamente)
  fsm_core.c/.h    — Variáveis de estado, getters, saúde do radar
  fsm_events.c/.h  — Eventos do tracking e callbacks UDP
  fsm_network.c/.h — Lógica de vizinhos, MASTER, SAFE_MODE, AUTONOMO
  fsm_timer.c/.h   — Gestão de timeouts e agendamentos
  fsm_task.c       — Loop principal 100ms (Core 1, Prio 6)
  CMakeLists.txt   — Build ESP-IDF (5 ficheiros .c, sem fsm_sim)
  README.txt       — Este ficheiro


===============================================================================
INTERFACE PÚBLICA (state_machine.h)
===============================================================================

  Ciclo de vida:
    state_machine_init()
    state_machine_update(comm_ok, is_master, radar_teve_frame)
    state_machine_task_start()

  Processamento de eventos:
    sm_process_event(type, vehicle_id, vel, eta_ms, x_mm)

  Callbacks UDP (implementados em fsm_events.c):
    on_tc_inc_received(speed, x_mm)
    on_prev_passed_received(speed)
    on_spd_received(speed, eta_ms, x_mm)
    on_master_claim_received(from_id)

  Getters:
    state_machine_get_state()       → system_state_t
    state_machine_get_state_name()  → "IDLE" | "LIGHT_ON" | ...
    state_machine_get_T()           → int (veículos locais)
    state_machine_get_Tc()          → int (veículos a caminho via UDP)
    state_machine_get_last_speed()  → float km/h
    state_machine_radar_ok()        → bool
    sm_is_obstaculo()               → bool

  Gestão de vizinhos:
    sm_on_right_neighbor_offline()
    sm_on_right_neighbor_online()


===============================================================================
DEPENDÊNCIAS (REQUIRES no CMakeLists.txt)
===============================================================================

  dali_manager      — controlo de brilho DALI
  comm_manager      — interface de comunicação UDP entre postes
  radar_manager     — leitura UART HLK-LD2450
  display_manager   — display ST7789 via LVGL
  tracking_manager  — rastreamento multi-veículo
  system_monitor    — heartbeat e watchdog
  config            — system_config.h (constantes)
  freertos          — tasks, semáforos, atomic
  esp_timer         — esp_timer_get_time()
  esp_system        — utilitários ESP-IDF
  log               — ESP_LOGI / LOGW / LOGE


===============================================================================
PIPELINE DE DADOS — RADAR → FSM → DISPLAY
===============================================================================

  radar_task (Core 0, 100ms)
      └─→ radar_read_data(&dados)           [parse UART HLK-LD2450]
      └─→ tracking_manager_update(&dados)   [associação nearest-neighbour]
      └─→ tracking_manager_task_notify_frame(ok)  [atomic_bool]

  fsm_task (Core 1, 100ms)
      └─→ atomic_exchange(s_radar_teve_frame)  [saúde do radar]
      └─→ state_machine_update()               [timeouts, transições]
      └─→ _processa_eventos_tracking()         [EVT_LOCAL, EVT_PASSED, ...]
      └─→ _atualiza_radar_display()            [alvos confirmados → display]
      └─→ system_monitor_heartbeat(MOD_FSM)


===============================================================================
ESTADOS DA FSM
===============================================================================

  IDLE       — Repouso. T=0 e Tc=0. Luz apagada ou em mínimo.
  LIGHT_ON   — Veículo(s) detectado(s). Luz acesa.
  SAFE_MODE  — Radar em falha (RADAR_FAIL_COUNT ciclos sem frame).
               Luz a 50% por segurança. Nunca apaga sozinho.
  MASTER     — Líder da cadeia. Primeiro poste activo à esquerda.
               Gera TC_INC quando detecta o primeiro veículo.
  AUTONOMO   — WiFi em falha mas radar OK.
               Opera localmente sem propagação UDP.
  OBSTACULO  — Objecto parado (vel ≤ 5 km/h durante OBSTACULO_MIN_FRAMES).
               Luz a 100%. Posição fixa no display. T e Tc não avançam.


===============================================================================
PROTOCOLO T/Tc — REGRAS
===============================================================================

  T  = veículos detectados pelo radar local deste poste
  Tc = veículos anunciados via UDP pelo poste à esquerda

  1. TC_INC enviado UMA VEZ por veículo — em EVT_LOCAL (detecção física)
  2. on_tc_inc_received() só faz Tc++ — nunca reenvia TC_INC
  3. EVT_APPROACHING não envia TC_INC — só agenda pré-acendimento por ETA
  4. EVT_PASSED envia SPD (ETA) — nunca TC_INC
  5. EVT_OBSTACULO envia PASSED ao vizinho direito — cancela Tc do veículo
  6. Luz apaga APENAS quando: T == 0 AND Tc == 0 AND elapsed >= TRAFIC_TIMEOUT_MS
  7. Sem radar → SAFE_MODE, independentemente do WiFi
  8. Sem vizinho esquerdo por AUTONOMO_DELAY_MS → assume MASTER


===============================================================================
EVENTOS DO TRACKING_MANAGER
===============================================================================

  SM_EVT_VEHICLE_DETECTED   — TRK_STATE_TENTATIVE → CONFIRMED
                               Primeiro avistamento. Sem alteração de T.
                               Liga luz se ETA < APPROACHING_THRESHOLD.

  SM_EVT_VEHICLE_LOCAL      — TRK_STATE_CONFIRMED, a aproximar-se
  (mapeado de event_approach_pending)
                               T++. Luz ON. Envia TC_INC + SPD ao vizinho direito.
                               Envia PASSED ao vizinho esquerdo (T-- em esq.).

  SM_EVT_VEHICLE_PASSED     — TRK_STATE_EXITED
                               T--. Envia SPD ao vizinho direito.
                               Agenda apagamento se T==0 e Tc==0.

  SM_EVT_VEHICLE_OBSTACULO  — vel ≤ 5 km/h durante OBSTACULO_MIN_FRAMES
                               Transição para STATE_OBSTACULO.
                               Luz 100%. Posição fixa no display.


===============================================================================
SINCRONIZAÇÃO ENTRE TASKS
===============================================================================

  Partilha de dados entre radar_task (Core 0) e fsm_task (Core 1):

  1. atomic_bool s_radar_teve_frame
       radar_task escreve via atomic_store()
       fsm_task lê e repõe via atomic_exchange()
       Garante visibilidade entre cores sem mutex.

  2. tracking_manager (mutex interno)
       radar_task: tracking_manager_update() — exclusivo Core 0
       fsm_task:   tracking_manager_get_vehicles() — thread-safe via mutex

  3. Callbacks UDP (on_tc_inc_received, on_prev_passed_received, ...)
       Chamados pela udp_task (Core 0) no contexto da recepção UDP.
       Acedendo a g_fsm_T, g_fsm_Tc sem mutex — aceitável pois
       são escritas atómicas em Xtensa LX6 (32 bits alinhados).
       A lógica de guarda em fsm_events.c previne double-trigger.


===============================================================================
FIM DO DOCUMENTO
===============================================================================
