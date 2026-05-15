/* ============================================================
   WIFI MANAGER — DECLARAÇÃO
   @file      wifi_manager.h
   @version   2.0  |  2026-05-07
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÕES v2.0 → v2.1:
   ─────────────────────────
   • REMOVIDAS: wifi_manager_assume_ap(), wifi_manager_reset_retry(),
     wifi_manager_is_ap_mode() — nunca chamadas.
   • CORRIGIDO: duplo esp_wifi_connect() em wifi_manager_enable().

   ALTERAÇÕES v1.5 → v2.0:
   ─────────────────────────
   • REMOVIDA lógica de mudança STA↔AP em runtime
   • ADICIONADA wifi_manager_init_ap() para pos=0
   
   MODELO v2.0:
   • POST_POSITION == 0 → AP permanente (192.168.4.1)
   • POST_POSITION > 0  → STA permanente (192.168.4.X)
   • IP FIXO nunca muda
   • Papel master/slave é lógico (fsm_network), não físico (WiFi)

   PRÉ-REQUISITOS:
   ───────────────
   esp_netif_init() e esp_event_loop_create_default()
   devem ser chamados ANTES de wifi_manager_init*().
============================================================ */
#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>


/* ============================================================
   INICIALIZAÇÃO
============================================================ */

/**
 * @brief Inicializa WiFi em modo STA com IP fixo.
 * 
 * Usado por postes com POST_POSITION > 0.
 * 
 * Configuração:
 * - Modo: WIFI_MODE_STA (cliente)
 * - IP: 192.168.4.(POST_POSITION + 1) — FIXO, não DHCP
 * - Gateway: 192.168.4.1 (AP do master pos=0)
 * - SSID/Password: definidos em system_config.h
 * 
 * Reconexão automática:
 * - Até WIFI_RETRY_ATTEMPTS tentativas imediatas
 * - Depois pausa WIFI_RECONNECT_MS e recomeça
 */
void wifi_manager_init(void);

/**
 * @brief Inicializa WiFi em modo AP.
 * 
 * Usado por poste com POST_POSITION == 0 (master físico).
 * 
 * Configuração:
 * - Modo: WIFI_MODE_AP (access point)
 * - IP: 192.168.4.1 — FIXO (automático ESP-IDF)
 * - SSID/Password: definidos em system_config.h
 * - Canal: WIFI_AP_CHANNEL
 * - Máx. clientes: 10
 */
void wifi_manager_init_ap(void);

/**
 * @brief Escolhe AP ou STA automaticamente por POST_POSITION.
 * 
 * Chamado no arranque do sistema (main.c).
 * 
 * Lógica:
 * - Se POST_POSITION == 0 → wifi_manager_init_ap()
 * - Se POST_POSITION > 0  → wifi_manager_init()
 * 
 * RECOMENDADO: Usar esta função em vez de init() ou init_ap() directo.
 */
void wifi_manager_init_auto(void);


/* ============================================================
   QUERY DE ESTADO
============================================================ */

/**
 * @brief Verifica se WiFi está ligado.
 * 
 * Para STA: TRUE quando conectado ao AP.
 * Para AP:  TRUE sempre (AP não "conecta", está sempre activo).
 * 
 * @return TRUE se conectado/activo
 */
bool wifi_manager_is_connected(void);

/**
 * @brief Retorna IP actual como string.
 * 
 * Formato: "x.x.x.x" ou "---" se desligado.
 * 
 * Thread-safe: Leitura sem lock (aceitável para display).
 * 
 * @return Ponteiro para string interna (não libertar!)
 */
const char *wifi_manager_get_ip(void);

/* ============================================================
   CONTROLO DE ACTIVAÇÃO/DESACTIVAÇÃO (SAFE MODE)
============================================================ */

/**
 * @brief Desliga WiFi completamente (safe mode).
 * 
 * Usado quando radar falha para evitar propagação de TC
 * sem detecção de veículos.
 * 
 * Efeito:
 * - Para envio/recepção UDP
 * - Desconecta de AP (se STA)
 * - Para AP (se modo AP)
 * - Poste fica isolado da rede
 * 
 * Chamado por: fsm_network.c quando radar offline
 */
void wifi_manager_disable(void);

/**
 * @brief Religa WiFi após safe mode.
 * 
 * Restaura conectividade quando radar recupera.
 * 
 * Efeito:
 * - Reinicia WiFi no modo original (STA ou AP)
 * - IP mantém-se fixo (não muda)
 * - Reconecta automaticamente
 * 
 * Chamado por: fsm_network.c quando radar volta online
 */
void wifi_manager_enable(void);

/**
 * @brief Verifica se WiFi está activo.
 *
 * @return true se WiFi ligado, false se desligado (safe mode)
 */
bool wifi_manager_is_enabled(void);


/* ============================================================
   FAILOVER DE AP
============================================================ */

/**
 * @brief Motor de failover — deve ser chamado periodicamente (~200ms).
 *
 * Gere a promoção STA→AP e a demoção AP→STA de forma autónoma:
 * - Se STA offline há POST_POSITION × WIFI_AP_PROMOTE_BASE_MS → promove a AP.
 * - Se AP promovido → tenta periodicamente voltar a STA (quando AP original regressa).
 * - POST_POSITION=0 é imune: nunca promovido, nunca demovido.
 *
 * Chamado por: system_monitor (_monitor_task, Core 1, Prio 7).
 */
void wifi_manager_tick(void);

/**
 * @brief Indica se este poste está em modo AP por failover (não é o AP original).
 *
 * @return true se é AP promovido, false se é AP original (pos=0) ou STA
 */
bool wifi_manager_is_promoted_ap(void);


#endif /* WIFI_MANAGER_H */
