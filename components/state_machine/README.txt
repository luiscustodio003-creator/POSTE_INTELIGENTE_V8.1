===============================================================================
   MÓDULO     : state_machine (reestruturado)
   VERSÃO     : 1.0  |  2026-04-26
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
===============================================================================

DESCRIÇÃO
─────────
Máquina de estados finita (FSM) central do Poste Inteligente v8.
Reestruturada de um único ficheiro de 798 linhas em 6 módulos
independentes com responsabilidades claras.

A lógica é exactamente a mesma — apenas reorganizada para facilitar
manutenção, depuração e adição de novos cenários.

===============================================================================
ESTRUTURA DOS FICHEIROS
===============================================================================

  fsm_core.c / fsm_core.h
  ────────────────────────
  Núcleo da FSM. Variáveis de estado globais (g_fsm_*), utilitários
  internos (fsm_agora_ms, fsm_agendar_apagar, fsm_verificar_radar),
  inicialização e ponto de entrada state_machine_update().
  Getters públicos: get_state, get_T, get_Tc, get_last_speed, etc.

  fsm_events.c / fsm_events.h
  ────────────────────────────
  Processamento de eventos do pipeline tracking → FSM.
  sm_process_event() com os 5 casos: DETECTED, APPROACHING,
  PASSED, LOCAL, OBSTACULO.
  Callbacks UDP: on_tc_inc_received, on_prev_passed_received,
  on_spd_received, on_master_claim_received.
  Gestão de vizinhos: sm_on_right_neighbor_offline/online.
  Compatibilidade: sm_on_radar_detect, sm_inject_test_car.

  fsm_network.c / fsm_network.h
  ──────────────────────────────
  Gestão de rede e estados degradados.
  fsm_network_vizinhos()           — Passos 3 e 4
  fsm_network_master()             — Passo 10
  fsm_network_estados_degradados() — Passo 11 (SAFE_MODE, AUTONOMO)

  fsm_timer.c / fsm_timer.h
  ──────────────────────────
  Timers e timeouts do ciclo 100ms.
  fsm_timer_update() chama internamente:
    _passo5_t_preso()          — T preso quando viz. esq. offline
    _passo6_pre_acendimento()  — pré-acendimento por ETA
    _passo7_apagamento()       — apagamento quando T=0, Tc=0
    _passo8_obstaculo()        — remoção automática de obstáculo
    _passo9_tc_timeout()       — timeout de segurança Tc
    _passo12_master_claim()    — heartbeat MASTER_CLAIM

  fsm_sim.c / fsm_sim.h
  ──────────────────────
  Simulador físico de veículos — compilado APENAS quando USE_RADAR=0.
  Ciclo: AGUARDA → ENTRAR → EM_VIA → DETECTADO → SAIU → AGUARDA
  Velocidades de rotação: 30, 50, 80, 50 km/h.
  fsm_sim_init(), fsm_sim_update(), fsm_sim_get_objeto(),
  fsm_sim_notificar_chegada().
  Aliases de compatibilidade: sim_init_mutex, sim_notificar_chegada,
  sim_get_objeto, _sim_update.

  fsm_task.c
  ──────────
  Loop principal a 100ms (Core 1, Prio 6).
  tracking_manager_task_notify_frame() — notificação atómica do radar.
  _atualiza_radar_display() — sincronização do display.
  fsm_task() — ciclo: injecção teste → radar → FSM → eventos → display.
  state_machine_task_start() — cria a task no Core 1.

  CMakeLists.txt
  ──────────────
  Regista os 6 ficheiros .c no sistema de build ESP-IDF.

===============================================================================
INTERFACE PÚBLICA (state_machine.h — não alterado)
===============================================================================

  Ciclo de vida:
    state_machine_init()
    state_machine_update(comm_ok, is_master, radar_teve_frame)
    state_machine_task_start()

  Processamento de eventos:
    sm_process_event(type, vehicle_id, vel, eta_ms, x_mm)

  Callbacks UDP (chamados pelo udp_manager):
    on_tc_inc_received(speed, x_mm)
    on_prev_passed_received()
    on_spd_received(speed, eta_ms, x_mm)
    on_master_claim_received(from_id)

  Getters:
    state_machine_get_state()
    state_machine_get_state_name()
    state_machine_get_T()
    state_machine_get_Tc()
    state_machine_get_last_speed()
    state_machine_radar_ok()
    sm_is_obstaculo()

  Gestão de vizinhos:
    sm_on_right_neighbor_offline()
    sm_on_right_neighbor_online()
    sm_inject_test_car(vel)
    sm_on_radar_detect(vel)   [deprecated — usar sm_process_event]

===============================================================================
DEPENDÊNCIAS
===============================================================================

  Requer (REQUIRES no CMakeLists.txt):
    dali_manager      — controlo de brilho DALI
    comm_manager      — comunicação UDP entre postes
    radar_manager     — leitura do sensor HLK-LD2450
    display_manager   — display ST7789 via LVGL
    tracking_manager  — rastreamento de veículos
    system_monitor    — heartbeat e watchdog
    config            — system_config.h (constantes)
    freertos          — tasks, semáforos, atomic
    esp_timer         — esp_timer_get_time()
    esp_system        — esp_random()
    log               — ESP_LOGI/LOGW/LOGE

===============================================================================
PROTOCOLO T/Tc — RESUMO
===============================================================================

  T  = veículos detectados localmente pelo radar
  Tc = veículos anunciados via UDP (a caminho)

  Regras fundamentais:
  1. TC_INC enviado UMA VEZ por veículo — em EVT_LOCAL
  2. on_tc_inc_received apenas faz Tc++ — sem reenvio
  3. EVT_APPROACHING não envia TC_INC — só agenda ETA local
  4. EVT_PASSED envia SPD (não TC_INC) e faz T--
  5. EVT_OBSTACULO envia PASSED ao vizinho direito (1ª vez)
  6. Sem radar = SAFE_MODE sempre, independente do WiFi
  7. Sem vizinho esquerdo = assume MASTER após AUTONOMO_DELAY_MS
  8. Último poste activo apaga após TRAFIC_TIMEOUT_MS

===============================================================================
VARIÁVEIS DE ESTADO GLOBAIS (prefixo g_fsm_)
===============================================================================

  Definidas em fsm_core.c, declaradas extern em fsm_core.h.
  Acessíveis a todos os sub-módulos sem passar por parâmetros.

  g_fsm_state          — estado actual da FSM
  g_fsm_T              — contador local de veículos
  g_fsm_Tc             — contador UDP de veículos a caminho
  g_fsm_last_speed     — última velocidade detectada (km/h)
  g_fsm_apagar_pend    — apagamento agendado
  g_fsm_radar_ok       — saúde do radar
  g_fsm_right_online   — vizinho direito online
  g_fsm_era_autonomo   — flag: acendeu em modo AUTONOMO
  g_fsm_last_detect_ms — timestamp da última detecção
  g_fsm_acender_em_ms  — timestamp de pré-acendimento
  g_fsm_tc_timeout_ms  — deadline de segurança Tc
  g_fsm_left_was_offline — vizinho esquerdo estava offline
  g_fsm_left_offline_ms  — timestamp desde viz. esq. offline
  g_fsm_master_claim_ms  — timestamp do último MASTER_CLAIM
  g_fsm_sem_vizinho_ms   — timestamp desde sem vizinhos
  g_fsm_obstaculo_last_ms — timestamp da última detecção obstáculo

===============================================================================
FIM DO README
===============================================================================
