/* system_config.h — v4.1 | 2026-05-07 | Poste Inteligente v8
   Parâmetros globais. Editar TIMEOUT_PROFILE e MODO_LABORATORIO conforme ambiente. */

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H


/* ── Perfil de timeouts ───────────────────────────────────── */
#define PROFILE_CONSERVADOR  0   /* failover ~70s — seguro, testado       */
#define PROFILE_BALANCEADO   1   /* failover ~20s — recomendado           */
#define PROFILE_AGRESSIVO    2   /* failover ~13s — risco de race cond.   */

#define TIMEOUT_PROFILE  PROFILE_BALANCEADO


/* ── Modo de operação ─────────────────────────────────────── */
#define MODO_LABORATORIO      1   /* 1 = bancada | 0 = produção */


/* ── Identidade do poste ──────────────────────────────────── */
#define POSTE_ID              3
#define POSTE_NAME            "POSTE 03"
#define POST_POSITION         2


/* ── Display ──────────────────────────────────────────────── */
#define LCD_V_RES_CONFIG   240


/* ── Wi-Fi ────────────────────────────────────────────────── */
#define WIFI_SSID             "wifi"
#define WIFI_PASS             "password"
#define WIFI_AP_CHANNEL        1

/* Tentativas e pausa entre rondas de ligação STA
   Anti-colisão AP: 2×PROMOTE_BASE > ATTEMPTS×1s + RECONNECT_MS         */
#if MODO_LABORATORIO
  #define WIFI_RETRY_ATTEMPTS   3       /* 3×~500ms ≈ 1.5s antes de pausa  */
  #define WIFI_RECONNECT_MS     5000    /* 5s — maior ganho na recuperação  */
#else
  #define WIFI_RETRY_ATTEMPTS   5
  #define WIFI_RECONNECT_MS     10000   /* 10s entre rondas em produção     */
#endif

/* ── Failover de AP (eleição dinâmica quando poste 0 cai) ─ */
#if MODO_LABORATORIO
  #define WIFI_AP_PROMOTE_BASE_MS   10000ULL  /* 10s × POST_POSITION em lab */
  #define WIFI_AP_SCAN_INTERVAL_MS  15000ULL  /* cada 15s tenta demoção    */
#else
  #define WIFI_AP_PROMOTE_BASE_MS   30000ULL  /* 30s × POST_POSITION em prod */
  #define WIFI_AP_SCAN_INTERVAL_MS  60000ULL  /* cada 60s tenta demoção    */
#endif
#define WIFI_DEMOTE_RETRIES  2   /* tentativas rápidas de demoção (minimiza disrupção) */

#define WIFI_AP_IP_1        192
#define WIFI_AP_IP_2        168
#define WIFI_AP_IP_3          4
#define WIFI_AP_GW_LAST       1

#define POSTE_IP_LAST_OCTET  (POST_POSITION + 1)


/* ── Protocolo UDP ────────────────────────────────────────── */
#define UDP_PORT              5005
#define MAX_NEIGHBORS         4
#define MAX_IP_LEN            16

#if TIMEOUT_PROFILE == PROFILE_CONSERVADOR
  #define DISCOVER_INTERVAL_MS   2000
  #define NEIGHBOR_TIMEOUT_MS    8000
  #define DISCOVER_RETRY_MS       500

#elif TIMEOUT_PROFILE == PROFILE_BALANCEADO
  #define DISCOVER_INTERVAL_MS   1000
  #define NEIGHBOR_TIMEOUT_MS    3000
  #define DISCOVER_RETRY_MS       300

#elif TIMEOUT_PROFILE == PROFILE_AGRESSIVO
  #define DISCOVER_INTERVAL_MS    500
  #define NEIGHBOR_TIMEOUT_MS    2000
  #define DISCOVER_RETRY_MS       200

#else
  #error "TIMEOUT_PROFILE inválido! Use CONSERVADOR, BALANCEADO ou AGRESSIVO"
#endif


/* ── Heartbeat de master ──────────────────────────────────── */
#if TIMEOUT_PROFILE == PROFILE_CONSERVADOR
  #define MASTER_CLAIM_HB_MS      30000
  #define MASTER_CLAIM_TIMEOUT_MS 60000

#elif TIMEOUT_PROFILE == PROFILE_BALANCEADO
  #define MASTER_CLAIM_HB_MS       5000
  #define MASTER_CLAIM_TIMEOUT_MS 15000

#elif TIMEOUT_PROFILE == PROFILE_AGRESSIVO
  #define MASTER_CLAIM_HB_MS       3000
  #define MASTER_CLAIM_TIMEOUT_MS 10000
#endif


/* ── Eleição de master ────────────────────────────────────── */
/* AUTONOMO_DELAY ≥ NEIGHBOR_TIMEOUT + 2s evita race condition
   no cenário A←B(off)←C quando A ainda envia MASTER_CLAIM. */
#if TIMEOUT_PROFILE == PROFILE_CONSERVADOR
  #define AUTONOMO_DELAY_MS  10000ULL

#elif TIMEOUT_PROFILE == PROFILE_BALANCEADO
  #define AUTONOMO_DELAY_MS   6000ULL   /* NEIGHBOR_TIMEOUT(3s) + 3s margem */

#elif TIMEOUT_PROFILE == PROFILE_AGRESSIVO
  #define AUTONOMO_DELAY_MS   3000ULL
#endif


/* ── Parâmetros físicos da instalação ────────────────────── */
#define POSTE_DIST_M          2
#define RADAR_MAX_M           2
#define RADAR_MAX_MM          (RADAR_MAX_M * 1000)
#define RADAR_DETECT_M        1


/* ── Hardware do radar ────────────────────────────────────── */
#define USE_RADAR             1
#define NO_FRAME_LIMIT        20
#define MAX_RADAR_TARGETS     100
#define RADAR_MAX_OBJ         MAX_RADAR_TARGETS
#define RADAR_TRAIL_MAX       8


/* ── Parâmetros de detecção ───────────────────────────────── */
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


/* ── Iluminação DALI/PWM ──────────────────────────────────── */
#define LIGHT_MIN             2
#define LIGHT_MAX             100
#define LIGHT_SAFE_MODE       50


/* ── Fade DALI ────────────────────────────────────────────── */
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


/* ── Temporização principal ───────────────────────────────── */
#define TRAFIC_TIMEOUT_MS       5000
#define LIGHT_ON_TIMEOUT_MS     5000
#define DETECTION_TIMEOUT_MS    1000
#define MARGEM_ACENDER_MS        500

#if MODO_LABORATORIO
  #define TC_TIMEOUT_MS  60000ULL
#else
  #if TIMEOUT_PROFILE == PROFILE_CONSERVADOR
    #define TC_TIMEOUT_MS  (TRAFIC_TIMEOUT_MS * 2)
  #elif TIMEOUT_PROFILE == PROFILE_BALANCEADO
    #define TC_TIMEOUT_MS  8000ULL
  #else
    #define TC_TIMEOUT_MS  6000ULL
  #endif
#endif

#define T_STUCK_TIMEOUT_MS      (TRAFIC_TIMEOUT_MS * 3)
#define OBSTACULO_REMOVE_MS     8000

#define RADAR_OK_COUNT          3
#define RADAR_FAIL_COUNT        80

#define SYSTEM_WDT_TIMEOUT_S    30


/* ── Energia e interface web ──────────────────────────────── */
#define LED_POWER_WATT    50
#define NIGHT_START_HOUR  20
#define NIGHT_END_HOUR     7
#define POSTE_TIMEZONE    "WET0WEST,M3.5.0/1,M10.5.0"


/* ── Ajustes de laboratório ───────────────────────────────── */
/* A sondagem de demoção WiFi (wifi_manager._try_demote_to_sta) pára
   o Wi-Fi ~4s. Com NEIGHBOR_TIMEOUT_MS=3s (BALANCEADO) esse corte
   dispara falsos OFFLINE → AUTONOMO_DELAY → promoção espúria de master.
   Em lab, aumentamos os dois valores para absorver o corte de 4s.    */
#if MODO_LABORATORIO && TIMEOUT_PROFILE == PROFILE_BALANCEADO
  #undef  NEIGHBOR_TIMEOUT_MS
  #define NEIGHBOR_TIMEOUT_MS   6000     /* > 4s do probe de demoção  */
  #undef  AUTONOMO_DELAY_MS
  #define AUTONOMO_DELAY_MS     8000ULL  /* NEIGHBOR_TIMEOUT(6s) + 2s */
#endif


/* ── Validação de timeouts em tempo de compilação ─────────── */

#if NEIGHBOR_TIMEOUT_MS <= (DISCOVER_INTERVAL_MS * 2)
  #error "NEIGHBOR_TIMEOUT_MS deve ser > 2 × DISCOVER_INTERVAL_MS"
#endif

#if AUTONOMO_DELAY_MS < (NEIGHBOR_TIMEOUT_MS + 1000ULL)
  #warning "AUTONOMO_DELAY_MS muito curto — risco de race condition!"
#endif

#if MASTER_CLAIM_TIMEOUT_MS <= (MASTER_CLAIM_HB_MS * 2)
  #error "MASTER_CLAIM_TIMEOUT_MS deve ser > 2 × MASTER_CLAIM_HB_MS"
#endif

#if !MODO_LABORATORIO && TC_TIMEOUT_MS < 5000ULL
  #warning "TC_TIMEOUT_MS muito curto para veículos reais!"
#endif

/* Anti-colisão AP: garante que P(n) encontra AP de P(n-1) antes de se promover.
   Condição: 2×PROMOTE_BASE > ATTEMPTS×1000 + RECONNECT_MS               */
#if (2 * WIFI_AP_PROMOTE_BASE_MS) <= (WIFI_RETRY_ATTEMPTS * 1000ULL + WIFI_RECONNECT_MS)
  #error "Anti-colisão WiFi violada: aumentar PROMOTE_BASE ou reduzir RECONNECT_MS/RETRY_ATTEMPTS"
#endif


/* ── Validação de parâmetros básicos ─────────────────────── */

#if POSTE_ID < 1 || POSTE_ID > 255
  #error "POSTE_ID deve estar entre 1 e 255"
#endif

#if POST_POSITION < 0 || POST_POSITION > 252
  #error "POST_POSITION deve estar entre 0 e 252 (IP = POST_POSITION+1, max .253)"
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


#if TIMEOUT_PROFILE == PROFILE_CONSERVADOR
  #pragma message "TIMEOUT_PROFILE: CONSERVADOR (failover ~70s)"
#elif TIMEOUT_PROFILE == PROFILE_BALANCEADO
  #pragma message "TIMEOUT_PROFILE: BALANCEADO (failover ~20s)"
#elif TIMEOUT_PROFILE == PROFILE_AGRESSIVO
  #pragma message "TIMEOUT_PROFILE: AGRESSIVO (failover ~13s)"
#endif


#endif /* SYSTEM_CONFIG_H */
