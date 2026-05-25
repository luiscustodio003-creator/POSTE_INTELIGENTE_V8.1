/* ============================================================
   SYSTEM MONITOR — DECLARAÇÃO
   @file      system_monitor.h
   @version   2.1  |  2026-05-14
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Supervisor central do sistema. Responsável por:
   - Inicializar todos os módulos em ordem correcta e segura
   - Criar e fixar todas as tasks nos cores correctos
   - Alimentar o hardware watchdog do ESP-IDF
   - Monitorizar heartbeats periódicos de cada módulo
   - Iniciar UDP quando Wi-Fi fica disponível
   - Actualizar display com estado de rede a cada 200ms

   DISTRIBUIÇÃO DUAL-CORE ESP32:
   ──────────────────────────────
   Core 0 (PRO_CPU):
     radar_task    prio 6  4096B  100ms  ← leitura UART
     display_task  prio 4  8192B   20ms  ← LVGL 50 Hz
     udp_task      prio 5  4096B  ~10ms  ← tráfego de rede
     (Wi-Fi stack  prio 22-23 — gerido pelo ESP-IDF)

   Core 1 (APP_CPU):
     monitor_task  prio 7  4096B  200ms  ← alimenta WDT
     fsm_task      prio 6  6144B  100ms  ← controlo principal

   HARDWARE WATCHDOG:
   ──────────────────
   Apenas monitor_task registada no WDT.
   Timeout: SYSTEM_WDT_TIMEOUT_S (system_config.h) → panic + reboot.
   monitor_task verifica heartbeats dos outros módulos
   e regista aviso LOGW se algum exceder o timeout.

   
   ──────────────────────────────────────────────────────────
   - CORRIGIDO: tabela de cores — radar/display no Core 0.
   - CORRIGIDO: timeout DISPLAY 500ms → 2000ms (falso alarme no arranque).
   - REMOVIDO: system_monitor_is_alive() — não implementada, nunca chamada.
   - REMOVIDO: #define SYSTEM_WDT_TIMEOUT_S comentado (valor era 10; real é 30).
============================================================ */
#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include <stdint.h>
#include <stdbool.h>


/* ============================================================
   IDENTIFICADORES DE MÓDULO
============================================================ */
typedef enum {
    MOD_FSM     = 0,  /* Máquina de estados (fsm_task)   */
    MOD_RADAR   = 1,  /* Leitura UART HLK-LD2450          */
    MOD_DISPLAY = 2,  /* LVGL + ST7789 (display_task)    */
    MOD_UDP     = 3,  /* Protocolo UDP (udp_task)         */
    MOD_COUNT   = 4
} monitor_module_t;


/* ============================================================
   TIMEOUTS DE HEARTBEAT POR MÓDULO
   ──────────────────────────────────────────────────────────
   FSM / RADAR : 500ms — ciclos de 100ms, margem de 5 ciclos
   DISPLAY     : 2000ms — render LVGL pode demorar no arranque
   UDP         : 500ms — ciclo de 10ms, 50 ciclos de margem
   Justificativa: cada módulo envia heartbeat no seu ciclo.
   Timeout = período × margem de segurança.
   monitor_task inicializa timestamps no arranque (sem falsos alarmes).
============================================================ */
#define MOD_FSM_TIMEOUT_MS      500
#define MOD_RADAR_TIMEOUT_MS    500
#define MOD_DISPLAY_TIMEOUT_MS  2000
#define MOD_UDP_TIMEOUT_MS      500

/* Escalão crítico: módulo parado > N × timeout normal → LOGE */
#define MOD_HEARTBEAT_CRITICAL_MULT  5


/* ============================================================
   SUPERVISÃO DE ESTADOS FSM
   ──────────────────────────────────────────────────────────
   Limiares para alertas periódicos do supervisor passivo.
   Nenhuma acção altera a FSM — só log + re-init comm quando seguro.
   Condição de segurança: sem tráfego (T=0, Tc=0, não LIGHT_ON/OBSTACULO).
============================================================ */
#define SUP_AUTONOMO_MS   30000ULL  /* AUTONOMO com WiFi OK > 30s → re-init comm  */
#define SUP_SAFE_MS       60000ULL  /* SAFE_MODE > 60s → alerta radar prolongado   */
#define SUP_WIFI_MS       30000ULL  /* WiFi offline (não SAFE_MODE) > 30s → alerta */


/* ============================================================
   API PÚBLICA
============================================================ */

/**
 * @brief Ponto de entrada único do sistema.
 *        Chamar de app_main() após NVS e infraestrutura de rede.
 *        Inicializa todos os módulos, cria todas as tasks,
 *        configura watchdog. NÃO retorna.
 */
void system_monitor_start(void);

/**
 * @brief Regista heartbeat de um módulo (thread-safe).
 *        Chamar a cada ciclo de cada task.
 * @param mod Identificador do módulo (MOD_*)
 */
void system_monitor_heartbeat(monitor_module_t mod);


#endif /* SYSTEM_MONITOR_H */
