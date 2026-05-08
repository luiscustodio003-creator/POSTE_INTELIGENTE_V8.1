/* ============================================================
   WIFI MANAGER — DECLARAÇÃO
   @file      wifi_manager.h
   @version   2.0  |  2026-05-07
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   ALTERAÇÕES v1.5 → v2.0:
   ─────────────────────────
   • REMOVIDA lógica de mudança STA↔AP em runtime
   • ADICIONADA wifi_manager_init_ap() para pos=0
   • ADICIONADA wifi_manager_is_ap_mode() getter
   • wifi_manager_assume_ap() OBSOLETA (mantida por compatibilidade)
   
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

/**
 * @brief Verifica se está em modo AP.
 * 
 * Usado por comm_manager para saber se deve enviar broadcast
 * ou unicast (embora v2.0 sempre use broadcast para MASTER_CLAIM).
 * 
 * @return TRUE se modo AP, FALSE se modo STA
 */
bool wifi_manager_is_ap_mode(void);


/* ============================================================
   CONTROLO DE RECONEXÃO
============================================================ */

/**
 * @brief Reinicia contador de tentativas de reconexão.
 * 
 * Usado para forçar nova série de tentativas imediatas
 * após pausa de WIFI_RECONNECT_MS.
 */
void wifi_manager_reset_retry(void);


/* ============================================================
   FUNÇÕES OBSOLETAS (compatibilidade v1.x)
============================================================ */

/**
 * @brief OBSOLETA — Não faz nada em v2.0!
 * 
 * PROBLEMA v1.x: Mudava STA→AP em runtime quando assumia master,
 * causando perda de IP e split-brain.
 * 
 * SOLUÇÃO v2.0: Modo WiFi é FIXO no arranque.
 * - pos=0 sempre AP
 * - pos>0 sempre STA
 * - Papel master/slave é LÓGICO (gerido por fsm_network)
 * 
 * Esta função APENAS loga warning e não faz nada.
 * Mantida por compatibilidade com código existente que
 * ainda possa chamar assume_ap() ao promover-se a master.
 * 
 * @deprecated Use wifi_manager_init_auto() no arranque.
 */
void wifi_manager_assume_ap(void);


#endif /* WIFI_MANAGER_H */
