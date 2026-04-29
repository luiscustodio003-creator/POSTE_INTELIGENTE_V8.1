/* ============================================================
   SYSTEM CONFIG — PARÂMETROS GLOBAIS
   @file      system_config.h
   @version   4.0  |  2026-04-24
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   FONTE ÚNICA DE VERDADE para todas as constantes do sistema.
   Qualquer alteração aqui propaga-se automaticamente a todos
   os módulos que incluem este ficheiro.

   ┌─────────────────────────────────────────────────────────────┐
   │  EDITAR POR POSTE  : POSTE_ID, POSTE_NAME, POST_POSITION   │
   │  EDITAR POR MODO   : Secção RADAR — MODO TESTE / PRODUÇÃO  │
   │  NÃO EDITAR        : Constantes derivadas (#define calculado)│
   └─────────────────────────────────────────────────────────────┘

   MELHORIAS v3.3 → v4.0:
   ──────────────────────────────────────────────────
   - Adicionadas constantes OBSTÁCULO (timeout de remoção)
   - Adicionada constante TC_TIMEOUT_FACTOR para ajuste de Tc
   - Adicionadas constantes SAFE_MODE_TIMEOUT_MS e AUTONOMO_DELAY_MS
     (antes espalhadas em literais no state_machine.c)
   - Adicionado DISCOVER_RETRY_MS (retry de descoberta UDP)
   - Adicionada validação de parâmetros em tempo de compilação
   - Comentários melhorados em todas as secções
   - #include <inttypes.h> removido (não necessário neste header)
============================================================ */
#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H


/* ============================================================
   IDENTIDADE DO POSTE
   ──────────────────────────────────────────────────────────
   Editar antes de flashar cada ESP32:
     POSTE_ID       → identificador numérico único (1, 2, 3...)
     POSTE_NAME     → nome apresentado no display e nos logs
     POST_POSITION  → posição na cadeia (0 = MASTER, 1, 2, ...)

   ATENÇÃO: POST_POSITION deve ser único e sequencial.
            POSTE_ID     deve ser único em toda a rede.
============================================================ */
#define POSTE_ID              1
#define POSTE_NAME            "POSTE 01"
#define POST_POSITION         0


/* ============================================================
   DISPLAY — RESOLUÇÃO VERTICAL
============================================================ */
#define LCD_V_RES_CONFIG      /* 240 = ecrã 240×240 | 320 = ecrã 240×320 */


/* ============================================================
   WI-FI
   ──────────────────────────────────────────────────────────
   Credenciais da rede local onde os postes comunicam via UDP.
   WIFI_RETRY_ATTEMPTS → tentativas antes de pausar reconexão
   WIFI_RECONNECT_MS   → pausa entre ciclos de reconexão (ms)
============================================================ */
#define WIFI_SSID             "wifi"
#define WIFI_PASS             "password"
#define WIFI_RETRY_ATTEMPTS   5
#define WIFI_RECONNECT_MS     30000


/* ============================================================
   PROTOCOLO UDP
   ──────────────────────────────────────────────────────────
   UDP_PORT           → porto de escuta/envio (igual em todos)
   MAX_NEIGHBORS      → máximo de postes vizinhos na tabela
   MAX_IP_LEN         → tamanho máximo de string IP (chars)
   DISCOVER_INTERVAL  → intervalo de broadcast DISCOVER (ms)
   DISCOVER_RETRY_MS  → reenvio DISCOVER quando sem resposta
   NEIGHBOR_TIMEOUT   → tempo sem resposta para marcar OFFLINE
============================================================ */
#define UDP_PORT              5005
#define MAX_NEIGHBORS         4
#define MAX_IP_LEN            16
#define DISCOVER_INTERVAL_MS  2000
#define DISCOVER_RETRY_MS     500
#define NEIGHBOR_TIMEOUT_MS   8000


/* ============================================================
   PARÂMETROS FÍSICOS DA INSTALAÇÃO
   ──────────────────────────────────────────────────────────
   POSTE_DIST_M   → distância entre postes consecutivos (m)
   RADAR_MAX_M    → alcance máximo do radar configurado (m)
                    Teste: 2m | Produção: 10-15m
   RADAR_MAX_MM   → idem em milímetros (calculado)
   RADAR_DETECT_M → distância a que se considera "detecção local"
                    usada para calcular ETA para o próximo poste
============================================================ */
#define POSTE_DIST_M          2         /* Distancia entre postes  */
#define RADAR_MAX_M           2         /* TESTE: 2m | PRODUÇÃO: 10 */
#define RADAR_MAX_MM          (RADAR_MAX_M * 1000)
#define RADAR_DETECT_M        1


/* ============================================================
   HARDWARE DO RADAR
   ──────────────────────────────────────────────────────────
   USE_RADAR          → 1=radar físico HLK-LD2450 | 0=simulador
   NO_FRAME_LIMIT     → ciclos sem frame para declarar radar FAIL
   MAX_RADAR_TARGETS  → máximo de alvos simultâneos do HLK-LD2450
   RADAR_MAX_OBJ      → alias para o display_manager
   RADAR_TRAIL_MAX    → pontos de rasto por alvo no canvas radar
============================================================ */
#define USE_RADAR             1
#define NO_FRAME_LIMIT        20
#define MAX_RADAR_TARGETS     3
#define RADAR_MAX_OBJ         MAX_RADAR_TARGETS
#define RADAR_TRAIL_MAX       8


/* ============================================================
   RADAR — PARÂMETROS DE DETECÇÃO
   ──────────────────────────────────────────────────────────
   Ajustar conforme o modo de operação:

   MODO TESTE (bancada, mão/pessoa a ~1m):
     RADAR_MIN_DIST_M      0.2   aceita objectos a partir de 20cm
     MIN_DETECT_KMH        0.5   aceita movimento muito lento
     AFASTAR_THRESHOLD_KMH 8.0   aceita qualquer direcção
     OBSTACULO_MIN_FRAMES  500   desactiva detecção de obstáculo

   MODO PRODUÇÃO (poste real, veículos):
     RADAR_MIN_DIST_M      0.5   ignora reflexões próximas
     MIN_DETECT_KMH        3.0   ignora objectos estáticos
     AFASTAR_THRESHOLD_KMH 2.0   só alvos a aproximar-se
     OBSTACULO_MIN_FRAMES  80    obstáculo após ~8 segundos
   ──────────────────────────────────────────────────────────── */
#define RADAR_MIN_DIST_M        0.2f  /* Distância mínima de detecção (metros)   */
#define MIN_DETECT_KMH          0.5f  /* Velocidade mínima de detecção (km/h)    */
#define AFASTAR_THRESHOLD_KMH   8.0f  /* Limiar de afastamento (km/h)            */
#define OBSTACULO_MIN_FRAMES    500   /* Frames parado para activar OBSTÁCULO    */
#define OBSTACULO_SPEED_MAX_KMH 3.0f  /* Velocidade máxima de obstáculo (km/h)   */
#define OBSTACULO_DIST_TOL_MM   300   /* Tolerância de posição para obstáculo(mm)*/


/* ============================================================
   ILUMINAÇÃO DALI/PWM
   ──────────────────────────────────────────────────────────
   LIGHT_MIN       → brilho mínimo em repouso (%)
   LIGHT_MAX       → brilho máximo ao detectar veículo (%)
   LIGHT_SAFE_MODE → brilho fixo em SAFE MODE (%)
============================================================ */
#define LIGHT_MIN             10
#define LIGHT_MAX             100
#define LIGHT_SAFE_MODE       50


/* ============================================================
   TEMPORIZAÇÃO PRINCIPAL
   ──────────────────────────────────────────────────────────
   TRAFIC_TIMEOUT_MS    → tempo sem veículos antes de apagar (ms)
   DETECTION_TIMEOUT_MS → timeout de detecção local (ms)
   MARGEM_ACENDER_MS    → antecipação do acendimento face ao ETA (ms)

   TIMEOUTS DERIVADOS (calculados — não editar):
   TC_TIMEOUT_MS        → timeout de segurança Tc (2×TRAFIC)
   T_STUCK_TIMEOUT_MS   → timeout T preso sem viz. esq. (3×TRAFIC)
   OBSTACULO_REMOVE_MS  → tempo sem detecção para remover obstáculo
   AUTONOMO_DELAY_MS    → atraso antes de assumir modo AUTONOMO
============================================================ */
#define TRAFIC_TIMEOUT_MS       5000
#define LIGHT_ON_TIMEOUT_MS     5000
#define DETECTION_TIMEOUT_MS    1000
#define MARGEM_ACENDER_MS        500

/* Timeouts derivados — não editar directamente */
#define TC_TIMEOUT_MS           (TRAFIC_TIMEOUT_MS * 2)
#define T_STUCK_TIMEOUT_MS      (TRAFIC_TIMEOUT_MS * 3)
#define OBSTACULO_REMOVE_MS     8000
#define AUTONOMO_DELAY_MS       10000ULL






/* ============================================================
   VALIDAÇÃO DE PARÂMETROS EM TEMPO DE COMPILAÇÃO
   ──────────────────────────────────────────────────────────
   Detecta configurações impossíveis antes de flashar.
   Gera erro de compilação descritivo com mensagem clara.
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


#endif /* SYSTEM_CONFIG_H */
