/* ============================================================
   SYSTEM CONFIG — PARÂMETROS GLOBAIS (OPTIMIZADO)
   @file      system_config.h
   @version   4.1  |  2026-05-07
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÕES v4.0 → v4.1:
   ───────────────────────────────────────────────────────────
   • ADICIONADO: Perfis de timeout (CONSERVADOR/BALANCEADO/AGRESSIVO)
   • OPTIMIZADO: Tempos de failover de master (70s → 20s → 13s)
   • OPTIMIZADO: Detecção de vizinho offline (8s → 3s → 2s)
   • ADICIONADO: Validação automática de dependências entre timeouts
   • DOCUMENTAÇÃO: Análise de race conditions e margem de segurança

   ESCOLHA DO PERFIL:
   ──────────────────
   #define TIMEOUT_PROFILE  PROFILE_CONSERVADOR  ← seguro, testado
   #define TIMEOUT_PROFILE  PROFILE_BALANCEADO   ← recomendado
   #define TIMEOUT_PROFILE  PROFILE_AGRESSIVO    ← rápido, arriscado
============================================================ */
#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H


/* ============================================================
   PERFIL DE TIMEOUTS — ESCOLHER UM
   ──────────────────────────────────────────────────────────
   CONSERVADOR: Original, testado, failover ~70s
   BALANCEADO:  Optimizado, failover ~20s (RECOMENDADO)
   AGRESSIVO:   Máxima velocidade, failover ~13s (risco)
============================================================ */
#define PROFILE_CONSERVADOR  0
#define PROFILE_BALANCEADO   1
#define PROFILE_AGRESSIVO    2

#define TIMEOUT_PROFILE  PROFILE_BALANCEADO  /* ← EDITAR AQUI */


/* ============================================================
   MODO DE OPERAÇÃO
============================================================ */
#define MODO_LABORATORIO      1   /* 1 = bancada | 0 = produção */


/* ============================================================
   IDENTIDADE DO POSTE
============================================================ */
#define POSTE_ID              1
#define POSTE_NAME            "POSTE 01"
#define POST_POSITION         0


/* ============================================================
   DISPLAY — RESOLUÇÃO VERTICAL
============================================================ */
#define LCD_V_RES_CONFIG   240


/* ============================================================
   WI-FI
============================================================ */
#define WIFI_SSID             "wifi"
#define WIFI_PASS             "password"
#define WIFI_AP_CHANNEL        1
#define WIFI_RETRY_ATTEMPTS   5
#define WIFI_RECONNECT_MS     30000

#define WIFI_AP_IP_1        192
#define WIFI_AP_IP_2        168
#define WIFI_AP_IP_3          4
#define WIFI_AP_GW_LAST       1

#define POSTE_IP_LAST_OCTET  (POST_POSITION + 1)


/* ============================================================
   PROTOCOLO UDP — TIMEOUTS OPTIMIZADOS POR PERFIL
   ──────────────────────────────────────────────────────────
   DISCOVER_INTERVAL_MS:
     Frequência de broadcast DISCOVER.
     Mais frequente = descoberta mais rápida de vizinhos.
     Margem de segurança: deve ser < NEIGHBOR_TIMEOUT_MS / 2
   
   NEIGHBOR_TIMEOUT_MS:
     Tempo sem resposta para marcar vizinho OFFLINE.
     Impacto directo na detecção de falhas.
     Margem: deve ser > 2 × DISCOVER_INTERVAL_MS
   
   DISCOVER_RETRY_MS:
     Reenvio de DISCOVER quando sem resposta.
     Apenas para descoberta inicial, não afecta failover.
============================================================ */
#define UDP_PORT              5005
#define MAX_NEIGHBORS         4
#define MAX_IP_LEN            16

#if TIMEOUT_PROFILE == PROFILE_CONSERVADOR
  /* ── CONSERVADOR: Original, testado ──────────────────── */
  #define DISCOVER_INTERVAL_MS   2000
  #define NEIGHBOR_TIMEOUT_MS    8000
  #define DISCOVER_RETRY_MS       500
  
#elif TIMEOUT_PROFILE == PROFILE_BALANCEADO
  /* ── BALANCEADO: Optimizado, recomendado ─────────────── */
  #define DISCOVER_INTERVAL_MS   1000   // 2x mais rápido
  #define NEIGHBOR_TIMEOUT_MS    3000   // Detecta falha em 3s
  #define DISCOVER_RETRY_MS       300
  
#elif TIMEOUT_PROFILE == PROFILE_AGRESSIVO
  /* ── AGRESSIVO: Máxima velocidade ────────────────────── */
  #define DISCOVER_INTERVAL_MS    500   // 4x mais rápido
  #define NEIGHBOR_TIMEOUT_MS    2000   // Detecta falha em 2s
  #define DISCOVER_RETRY_MS       200
  
#else
  #error "TIMEOUT_PROFILE inválido! Use CONSERVADOR, BALANCEADO ou AGRESSIVO"
#endif


/* ============================================================
   HEARTBEAT DE MASTER — OPTIMIZADO POR PERFIL
   ──────────────────────────────────────────────────────────
   MASTER_CLAIM_HB_MS:
     Intervalo entre broadcasts de MASTER_CLAIM.
     Impacto directo no failover de master.
     Margem: MASTER_CLAIM_TIMEOUT deve ser > 2× este valor
   
   MASTER_CLAIM_TIMEOUT_MS:
     Tempo sem MASTER_CLAIM para considerar master offline.
     Usado em fsm_network.c para verificação antes de promover.
     Margem: deve ser > 2 × MASTER_CLAIM_HB_MS
============================================================ */
#if TIMEOUT_PROFILE == PROFILE_CONSERVADOR
  #define MASTER_CLAIM_HB_MS      30000  // Heartbeat a cada 30s
  #define MASTER_CLAIM_TIMEOUT_MS 60000  // Timeout 60s
  
#elif TIMEOUT_PROFILE == PROFILE_BALANCEADO
  #define MASTER_CLAIM_HB_MS       5000  // Heartbeat a cada 5s
  #define MASTER_CLAIM_TIMEOUT_MS 15000  // Timeout 15s
  
#elif TIMEOUT_PROFILE == PROFILE_AGRESSIVO
  #define MASTER_CLAIM_HB_MS       3000  // Heartbeat a cada 3s
  #define MASTER_CLAIM_TIMEOUT_MS 10000  // Timeout 10s
#endif


/* ============================================================
   ELEIÇÃO DE MASTER — OPTIMIZADO POR PERFIL
   ──────────────────────────────────────────────────────────
   AUTONOMO_DELAY_MS:
     Tempo após vizinho esq. offline antes de promover a master.
     Impacto: failover de master.
     
     ANÁLISE DE RACE CONDITIONS:
     ───────────────────────────
     Cenário crítico: A (master) ← B (offline) ← C
     
     C detecta B offline em NEIGHBOR_TIMEOUT_MS
     C aguarda AUTONOMO_DELAY_MS antes de promover
     C verifica se recebeu MASTER_CLAIM nos últimos 
       MASTER_CLAIM_TIMEOUT_MS
     
     Para evitar promoção indevida:
       AUTONOMO_DELAY_MS ≥ NEIGHBOR_TIMEOUT_MS + margem
     
     Margem de segurança: 2s (cobre jitter de rede)
============================================================ */
#if TIMEOUT_PROFILE == PROFILE_CONSERVADOR
  #define AUTONOMO_DELAY_MS  10000ULL  // 10s após detectar offline
  
#elif TIMEOUT_PROFILE == PROFILE_BALANCEADO
  #define AUTONOMO_DELAY_MS   5000ULL  // 5s (3s timeout + 2s margem)
  
#elif TIMEOUT_PROFILE == PROFILE_AGRESSIVO
  #define AUTONOMO_DELAY_MS   3000ULL  // 3s (2s timeout + 1s margem)
#endif


/* ============================================================
   PARÂMETROS FÍSICOS DA INSTALAÇÃO
============================================================ */
#define POSTE_DIST_M          2
#define RADAR_MAX_M           2
#define RADAR_MAX_MM          (RADAR_MAX_M * 1000)
#define RADAR_DETECT_M        1


/* ============================================================
   HARDWARE DO RADAR
============================================================ */
#define USE_RADAR             1
#define NO_FRAME_LIMIT        20
#define MAX_RADAR_TARGETS     3
#define RADAR_MAX_OBJ         MAX_RADAR_TARGETS
#define RADAR_TRAIL_MAX       8


/* ============================================================
   RADAR — PARÂMETROS DE DETECÇÃO
============================================================ */
#if MODO_LABORATORIO
  #define RADAR_MIN_DIST_M        0.2f
  #define MIN_DETECT_KMH          0.3f
  #define AFASTAR_THRESHOLD_KMH   8.0f
  #define OBSTACULO_MIN_FRAMES    30
  #define OBSTACULO_SPEED_MAX_KMH 1.0f
#else
  #define RADAR_MIN_DIST_M        0.5f
  #define MIN_DETECT_KMH          3.0f
  #define AFASTAR_THRESHOLD_KMH   2.0f
  #define OBSTACULO_MIN_FRAMES    80
  #define OBSTACULO_SPEED_MAX_KMH 3.0f
#endif
#define OBSTACULO_DIST_TOL_MM   300


/* ============================================================
   ILUMINAÇÃO DALI/PWM
============================================================ */
#define LIGHT_MIN             2
#define LIGHT_MAX             100
#define LIGHT_SAFE_MODE       50


/* ============================================================
   FADE DALI — LIMIARES DE VELOCIDADE
============================================================ */
#if MODO_LABORATORIO
  #define VEL_FADE_RAPIDO_KMH   3.0f
  #define VEL_FADE_MEDIO_KMH    2.0f
  #define VEL_FADE_LENTO_KMH    1.0f
#else
  #define VEL_FADE_RAPIDO_KMH  80.0f
  #define VEL_FADE_MEDIO_KMH   50.0f
  #define VEL_FADE_LENTO_KMH   30.0f
#endif

#define FADE_UP_RAPIDO_MS     300
#define FADE_UP_MEDIO_MS      500
#define FADE_UP_LENTO_MS      800
#define FADE_UP_DEFAULT_MS    500
#define FADE_DOWN_MS          4000


/* ============================================================
   TEMPORIZAÇÃO PRINCIPAL
============================================================ */
#define TRAFIC_TIMEOUT_MS       5000
#define LIGHT_ON_TIMEOUT_MS     5000
#define DETECTION_TIMEOUT_MS    1000
#define MARGEM_ACENDER_MS        500

/* ── TC_TIMEOUT_MS: depende do perfil e modo ─────────────
   Laboratório: sempre 60s (veículos lentos)
   Produção: 2× TRAFIC_TIMEOUT_MS ou baseado no perfil
──────────────────────────────────────────────────────────── */
#if MODO_LABORATORIO
  #define TC_TIMEOUT_MS  60000ULL
#else
  #if TIMEOUT_PROFILE == PROFILE_CONSERVADOR
    #define TC_TIMEOUT_MS  (TRAFIC_TIMEOUT_MS * 2)  // 10s
  #elif TIMEOUT_PROFILE == PROFILE_BALANCEADO
    #define TC_TIMEOUT_MS  8000ULL                   // 8s
  #else
    #define TC_TIMEOUT_MS  6000ULL                   // 6s
  #endif
#endif

#define T_STUCK_TIMEOUT_MS      (TRAFIC_TIMEOUT_MS * 3)
#define OBSTACULO_REMOVE_MS     8000

/* Saúde do radar */
#define RADAR_OK_COUNT          3
#define RADAR_FAIL_COUNT        80

/* Watchdog */
#define SYSTEM_WDT_TIMEOUT_S    30


/* ============================================================
   VALIDAÇÃO AUTOMÁTICA DE TIMEOUTS
   ──────────────────────────────────────────────────────────
   Verifica dependências críticas em tempo de compilação.
   Previne configurações que causariam race conditions.
============================================================ */

/* Regra 1: NEIGHBOR_TIMEOUT > 2 × DISCOVER_INTERVAL */
#if NEIGHBOR_TIMEOUT_MS <= (DISCOVER_INTERVAL_MS * 2)
  #error "NEIGHBOR_TIMEOUT_MS deve ser > 2 × DISCOVER_INTERVAL_MS"
#endif

/* Regra 2: AUTONOMO_DELAY ≥ NEIGHBOR_TIMEOUT + margem */
#if AUTONOMO_DELAY_MS < (NEIGHBOR_TIMEOUT_MS + 1000ULL)
  #warning "AUTONOMO_DELAY_MS muito curto — risco de race condition!"
#endif

/* Regra 3: MASTER_CLAIM_TIMEOUT > 2 × MASTER_CLAIM_HB */
#if MASTER_CLAIM_TIMEOUT_MS <= (MASTER_CLAIM_HB_MS * 2)
  #error "MASTER_CLAIM_TIMEOUT_MS deve ser > 2 × MASTER_CLAIM_HB_MS"
#endif

/* Regra 4: TC_TIMEOUT razoável para tráfego */
#if !MODO_LABORATORIO && TC_TIMEOUT_MS < 5000ULL
  #warning "TC_TIMEOUT_MS muito curto para veículos reais!"
#endif


/* ============================================================
   VALIDAÇÃO DE PARÂMETROS BÁSICOS
============================================================ */
#if POSTE_ID < 1 || POSTE_ID > 255
  #error "POSTE_ID deve estar entre 1 e 255"
#endif

#if POST_POSITION < 0 || POST_POSITION > 15
  #error "POST_POSITION deve estar entre 0 e 15"
#endif

#if RADAR_MAX_M < 1 || RADAR_MAX_M > 30
  #error "RADAR_MAX_M deve estar entre 1m e 30m"
#endif

#if POSTE_DIST_M <= RADAR_DETECT_M
  #error "POSTE_DIST_M deve ser maior que RADAR_DETECT_M"
#endif

#if LIGHT_MIN >= LIGHT_MAX
  #error "LIGHT_MIN deve ser menor que LIGHT_MAX"
#endif

#if UDP_PORT < 1024 || UDP_PORT > 65535
  #error "UDP_PORT deve estar entre 1024 e 65535"
#endif

#if MAX_NEIGHBORS < 2 || MAX_NEIGHBORS > 8
  #error "MAX_NEIGHBORS deve estar entre 2 e 8"
#endif

#if MODO_LABORATORIO != 0 && MODO_LABORATORIO != 1
  #error "MODO_LABORATORIO deve ser 0 (producao) ou 1 (laboratorio)"
#endif


/* ============================================================
   MENSAGEM DE COMPILAÇÃO — PERFIL ACTIVO
============================================================ */
#if TIMEOUT_PROFILE == PROFILE_CONSERVADOR
  #pragma message "TIMEOUT_PROFILE: CONSERVADOR (failover ~70s)"
#elif TIMEOUT_PROFILE == PROFILE_BALANCEADO
  #pragma message "TIMEOUT_PROFILE: BALANCEADO (failover ~20s) ← RECOMENDADO"
#elif TIMEOUT_PROFILE == PROFILE_AGRESSIVO
  #pragma message "TIMEOUT_PROFILE: AGRESSIVO (failover ~13s) — TESTE ANTES!"
#endif


#endif /* SYSTEM_CONFIG_H */
