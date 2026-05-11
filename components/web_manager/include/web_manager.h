/**
 * @file web_manager.h
 * @brief Gestor de Interface Web HTTP para Monitorização Remota
 * 
 * DESCRIÇÃO:
 * -----------
 * Este módulo implementa um servidor HTTP leve para acesso via browser aos
 * dados dos postes inteligentes. Permite monitorização em tempo real do estado,
 * consumo energético, topologia da rede e estatísticas de tráfego.
 * 
 * FUNCIONALIDADES:
 * ----------------
 * ✅ Servidor HTTP na porta 80
 * ✅ Interface web responsiva (mobile + desktop)
 * ✅ API JSON para dados dinâmicos
 * ✅ Dashboard de linha completa
 * ✅ Página individual por poste
 * ✅ Estatísticas de tempo/energia
 * ✅ Visualização de topologia (master/slave)
 * ✅ Limite de 1 cliente simultâneo (economia de RAM)
 * 
 * RESPONSABILIDADES:
 * ------------------
 * ✅ Iniciar/parar servidor HTTP
 * ✅ Routing de URLs (/dashboard, /poste/N, /api/...)
 * ✅ Servir HTML embebido
 * ✅ Responder a pedidos JSON
 * 
 * NÃO FAZ:
 * --------
 * ❌ Lógica de negócio (delega para fsm_core)
 * ❌ Comunicação UDP (delega para udp_manager)
 * ❌ Eleição de líder (delega para network_coordinator)
 * 
 * DEPENDÊNCIAS:
 * -------------
 * - ESP-IDF HTTP Server (esp_http_server)
 * - web_data_provider (fonte de dados)
 * - wifi_manager (rede activa)
 * 
 * RECURSOS CONSUMIDOS:
 * --------------------
 * - Flash: ~45KB (código + HTML embebido)
 * - RAM: ~20KB (1 cliente activo)
 * - CPU: ~3% quando activo / ~0.5% em idle
 * - Stack: 3KB por tarefa HTTP
 * 
 * PRIORIDADE FREERTOS: 3 (baixa - não crítico)
 * 
 * ACESSO VIA BROWSER:
 * -------------------
 * - http://192.168.1.100/          → Dashboard (se poste #0)
 * - http://192.168.1.101/          → Dashboard (se poste #1)
 * - http://192.168.1.100/poste/0   → Detalhes do poste #0
 * - http://192.168.1.100/api/line  → JSON com estado completo
 * 
 * @author Luis Custodio | Tiago Moreno
 * @date 2026-05-09
 * @version 1.0
 */

#ifndef WEB_MANAGER_H
#define WEB_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_err.h>
#include <esp_http_server.h>
#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// CONFIGURAÇÃO
// ============================================================================

/**
 * @brief Porta do servidor HTTP
 * @note Porta padrão HTTP (sem SSL)
 */
#define WEB_SERVER_PORT         80

/**
 * @brief Número máximo de clientes simultâneos
 * @note Limitado a 1 para economizar RAM (~20KB por cliente)
 */
#define WEB_MAX_CONNECTIONS     1

/**
 * @brief Tamanho do buffer de recepção HTTP
 * @note Suficiente para pedidos GET simples
 */
#define WEB_RX_BUFFER_SIZE      512

/**
 * @brief Tamanho do buffer de envio HTTP
 * @note Deve acomodar JSON máximo (~2KB) + headers
 */
#define WEB_TX_BUFFER_SIZE      4096

/**
 * @brief Timeout de leitura de socket (ms)
 */
#define WEB_SOCKET_TIMEOUT_MS   5000

// ============================================================================
// ESTRUTURAS
// ============================================================================

/**
 * @brief Estatísticas do servidor web
 */
typedef struct {
    uint32_t total_requests;        ///< Total de pedidos recebidos
    uint32_t active_connections;    ///< Conexões activas no momento
    uint32_t failed_requests;       ///< Pedidos que falharam
    uint32_t uptime_seconds;        ///< Tempo de actividade do servidor
} web_stats_t;

// ============================================================================
// API PÚBLICA
// ============================================================================

/**
 * @brief Inicializa o servidor HTTP
 * 
 * Esta função deve ser chamada APÓS wifi_manager_init() estar completo
 * e o poste ter um IP atribuído.
 * 
 * SEQUÊNCIA DE ARRANQUE:
 * 1. wifi_manager_init() → IP fixo atribuído
 * 2. web_data_provider_init() → Agregador de dados pronto
 * 3. web_manager_init() → Servidor HTTP activo
 * 
 * @return ESP_OK se servidor iniciado com sucesso
 * @return ESP_FAIL se falha (ex: porta ocupada, sem memória)
 * 
 * @note Servidor fica a escutar na porta 80
 * @note Limite de 1 cliente simultâneo
 * 
 * @example
 * ```c
 * // No main.c, após WiFi conectado:
 * ESP_ERROR_CHECK(web_manager_init());
 * ESP_LOGI(TAG, "Servidor web activo em http://%s/", my_ip);
 * ```
 */
esp_err_t web_manager_init(void);

/**
 * @brief Para o servidor HTTP e liberta recursos
 * 
 * Fecha todas as conexões activas e liberta memória.
 * Útil para debug ou reinicializações.
 * 
 * @note Após chamar isto, web_manager_init() pode ser chamado novamente
 */
void web_manager_stop(void);

/**
 * @brief Verifica se servidor está activo
 * 
 * @return true se servidor HTTP está a correr
 * @return false se parado ou não inicializado
 */
bool web_manager_is_running(void);

/**
 * @brief Obtém número de clientes conectados no momento
 * 
 * @return Número de conexões activas (0 ou 1 nesta implementação)
 */
uint8_t web_manager_get_active_sessions(void);

/**
 * @brief Obtém estatísticas do servidor
 * 
 * @param[out] stats Estrutura para preencher com estatísticas
 * 
 * @example
 * ```c
 * web_stats_t stats;
 * web_manager_get_stats(&stats);
 * printf("Total pedidos: %lu\n", stats.total_requests);
 * ```
 */
void web_manager_get_stats(web_stats_t* stats);

/**
 * @brief Reset das estatísticas (útil para testes)
 */
void web_manager_reset_stats(void);

#ifdef __cplusplus
}
#endif

#endif // WEB_MANAGER_H
