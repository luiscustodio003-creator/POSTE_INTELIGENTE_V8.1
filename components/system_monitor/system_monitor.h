/* ============================================================
   SYSTEM MONITOR — DECLARAÇÃO
   @file      system_monitor.h
   @version   2.0  |  2026-04-24
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
     udp_task      prio 5  4096B  ~10ms  ← tráfego de rede
     (Wi-Fi stack  prio 22-23 — gerido pelo ESP-IDF)

   Core 1 (APP_CPU):
     monitor_task  prio 7  3072B  200ms  ← alimenta WDT
     fsm_task      prio 6  6144B  100ms  ← controlo principal
     radar_task    prio 5  4096B  100ms  ← leitura UART
     display_task  prio 4  8192B   20ms  ← LVGL 50 Hz

   HARDWARE WATCHDOG:
   ──────────────────
   Apenas monitor_task registada no WDT.
   Timeout: SYSTEM_WDT_TIMEOUT_S → panic + reboot.
   monitor_task verifica heartbeats dos outros módulos
   e regista aviso LOGW se algum exceder o timeout.

   MELHORIAS v1.0 → v2.0:
   ──────────────────────────────────────────────────────────
   1. #include <inttypes.h> removido (não necessário neste header).
   2. Timeouts de heartbeat documentados com justificativa.
   3. Distribuição de cores documentada por task.
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
   DISPLAY     : 200ms — ciclos de 20ms, margem de 10 ciclos
   UDP         : 100ms — ciclo de 10ms, 10 ciclos de margem
   Justificativa: cada módulo envia heartbeat no seu ciclo.
   Timeout = período × margem de segurança (5-10×).
============================================================ */
/* Timeouts de heartbeat — em operação normal.
   Durante os primeiros segundos de arranque, o monitor_task
   inicializa todos os timestamps para evitar falsos alarmes.
   O timeout do DISPLAY é generoso (2s) porque o render LVGL
   pode demorar no arranque dependendo da inicialização SPI. */
#define MOD_FSM_TIMEOUT_MS      500
#define MOD_RADAR_TIMEOUT_MS    500
#define MOD_DISPLAY_TIMEOUT_MS  2000
#define MOD_UDP_TIMEOUT_MS      500


/* ============================================================
   HARDWARE WATCHDOG
   ──────────────────────────────────────────────────────────
   10 segundos: cobre o pior caso de inicialização (WiFi + DHCP).
   Após inicialização, monitor_task alimenta WDT a cada 200ms.
============================================================ */
//#define SYSTEM_WDT_TIMEOUT_S    10


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

/**
 * @brief Verifica se módulo está vivo (heartbeat dentro do timeout).
 *        Usado para degradar comportamento em falha parcial.
 * @param mod Identificador do módulo
 * @return true se activo, false se timeout expirado
 */
bool system_monitor_is_alive(monitor_module_t mod);


#endif /* SYSTEM_MONITOR_H */
