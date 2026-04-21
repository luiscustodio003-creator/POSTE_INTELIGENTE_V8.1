/* ============================================================
   SYSTEM CONFIG — PARÂMETROS GLOBAIS
   @file      system_config.h
   @version   3.3  |  2026-04-20
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x / PlatformIO)

   FONTE ÚNICA DE VERDADE para todas as constantes do sistema.

   ┌─────────────────────────────────────────────────────────┐
   │  EDITAR POR POSTE : POSTE_ID, POSTE_NAME, POST_POSITION │
   │  EDITAR POR MODO  : Secção RADAR DETECÇÃO               │
   └─────────────────────────────────────────────────────────┘
============================================================ */
#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H


/* ============================================================
   IDENTIDADE DO POSTE
   ──────────────────────────────────────────────────────────
   Editar antes de flashar cada ESP32:
     POSTE_ID       → identificador numérico único (1, 2, 3...)
     POSTE_NAME     → nome apresentado no display
     POST_POSITION  → posição na cadeia (0=MASTER, 1, 2...)
============================================================ */
#define POSTE_ID              2
#define POSTE_NAME            "POSTE 02"
#define POST_POSITION         1


/* ============================================================
   WI-FI
   ──────────────────────────────────────────────────────────
   Credenciais da rede local onde os postes comunicam via UDP.
   WIFI_RETRY_ATTEMPTS → tentativas antes de pausar
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
   NEIGHBOR_TIMEOUT   → tempo sem resposta para marcar OFFLINE
============================================================ */
#define UDP_PORT              5005
#define MAX_NEIGHBORS         4
#define MAX_IP_LEN            16
#define DISCOVER_INTERVAL_MS  2000
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
#define POSTE_DIST_M          50
#define RADAR_MAX_M           2        /* TESTE: 2m | PRODUÇÃO: 10 */
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


#define RADAR_MIN_DIST_M        0.2f /* Distância mínima de detecção (metros) — ignora fantasmas */
#define MIN_DETECT_KMH          0.5f /* Velocidade mínima de detecção (km/h) — ignora objectos parados */
#define AFASTAR_THRESHOLD_KMH   8.0f /* Limiar de afastamento (km/h) — abaixo disto = a aproximar-se */
#define OBSTACULO_MIN_FRAMES    500  /* Frames consecutivos parado para activar modo OBSTÁCULO */
#define OBSTACULO_SPEED_MAX_KMH 3.0f /* Velocidade máxima para considerar obstáculo estático (km/h) */
#define OBSTACULO_DIST_TOL_MM   300  /* Tolerância de posição para obstáculo estático (mm) */


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
   TEMPORIZAÇÃO
   ──────────────────────────────────────────────────────────
   TRAFIC_TIMEOUT_MS    → tempo sem veículos antes de apagar (ms)
   DETECTION_TIMEOUT_MS → timeout de detecção local (ms)
   MARGEM_ACENDER_MS    → antecipação do acendimento face ao ETA (ms)
============================================================ */
#define TRAFIC_TIMEOUT_MS     5000
#define DETECTION_TIMEOUT_MS  1000
#define MARGEM_ACENDER_MS     500


/* ============================================================
   DISPLAY
   ──────────────────────────────────────────────────────────
   Resolução do display TFT ST7789.
   LCD_H_RES → largura em pixels
   LCD_V_RES → altura em pixels
   NOTA: para display 240×320 alterar LCD_V_RES para 320.
============================================================ */
#define LCD_H_RES             240
#define LCD_V_RES             240


#endif 