#include <inttypes.h>
/* ============================================================
   SYSTEM MONITOR — DECLARAÇÃO
   @file      system_monitor.h
   @version   1.0  |  2026-04-07
   Projecto  : Poste Inteligente v8
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

   Supervisor central do sistema. Responsável por:
   - Inicializar todos os módulos em ordem correcta
   - Criar e fixar todas as tasks nos cores correctos
   - Alimentar o hardware watchdog do ESP-IDF
   - Monitorizar heartbeats periódicos de cada módulo
   - Iniciar UDP quando Wi-Fi fica disponível
   - Actualizar display com estado de rede (200ms)

   Distribuição dual-core ESP32:
   ─────────────────────────────
   Core 0 (PRO_CPU): udp_task (prio 5, 4096B)
     Partilha core com Wi-Fi stack (prio 22-23, ESP-IDF interno)
     Isola tráfego de rede do processamento de controlo.

   Core 1 (APP_CPU):
     monitor_task  prio 7  3072B  200ms  ← alimenta WDT
     fsm_task      prio 6  6144B  100ms  ← controlo principal
     radar_task    prio 5  4096B  100ms  ← leitura UART
     display_task  prio 4  8192B   20ms  ← LVGL 50 Hz

   Hardware Watchdog:
   ──────────────────
   Apenas monitor_task registada no WDT.
   Timeout: SYSTEM_WDT_TIMEOUT_S (10s) → panic + reboot.
   A monitor_task verifica heartbeats dos outros módulos
   e regista aviso LOGW se algum exceder o timeout.
============================================================ */
#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

/* ── Identificadores de módulo ────────────────────────────── */
typedef enum {
    MOD_FSM     = 0,
    MOD_RADAR   = 1,
    MOD_DISPLAY = 2,
    MOD_UDP     = 3,
    MOD_COUNT   = 4
} monitor_module_t;

/* ── Timeouts de heartbeat por módulo ─────────────────────── */
#define MOD_FSM_TIMEOUT_MS      500
#define MOD_RADAR_TIMEOUT_MS    500
#define MOD_DISPLAY_TIMEOUT_MS  200
#define MOD_UDP_TIMEOUT_MS      100

/* ── Hardware watchdog ────────────────────────────────────── */
#define SYSTEM_WDT_TIMEOUT_S    10

/* ============================================================
   API PÚBLICA
============================================================ */

/**
 * @brief Ponto de entrada único do sistema.
 *        Chamar de app_main() após NVS e infraestrutura de rede.
 *        Inicializa todos os módulos, cria todas as tasks,
 *        configura watchdog. Não retorna.
 */
void system_monitor_start(void);

/**
 * @brief Regista heartbeat de um módulo (thread-safe).
 *        Chamar a cada ciclo de cada task.
 * @param mod Identificador do módulo
 */
void system_monitor_heartbeat(monitor_module_t mod);

/**
 * @brief Verifica se módulo está vivo (heartbeat dentro do timeout).
 *        Usado pela FSM para degradar comportamento em falha parcial.
 * @param mod Identificador do módulo
 * @return true se activo, false se timeout expirado
 */
bool system_monitor_is_alive(monitor_module_t mod);



#endif /* SYSTEM_MONITOR_H */

