/**
 * @file web_data_provider.h
 * @brief Agregador de Dados para API JSON
 * 
 * DESCRIÇÃO:
 * -----------
 * Este módulo agrega informação de TODOS os outros módulos do sistema
 * (fsm_core, network_coordinator, tracking_manager, light_controller)
 * e converte para formato JSON minificado para envio via HTTP.
 * 
 * FUNCIONALIDADES:
 * ----------------
 * ✅ Recolher dados de estado actual (T, Tc, duty_cycle)
 * ✅ Calcular distribuição de tempo (SAVE/MIN/ON)
 * ✅ Estimar consumo energético acumulado
 * ✅ Obter topologia da rede (master/slave, vizinhos)
 * ✅ Converter tudo para JSON minificado
 * 
 * RESPONSABILIDADES:
 * ------------------
 * ✅ Interface única para web_manager
 * ✅ Cálculos de estatísticas
 * ✅ Formatação JSON (sem pretty-print)
 * 
 * NÃO FAZ:
 * --------
 * ❌ Comunicação HTTP (delega para web_manager)
 * ❌ Lógica de negócio (apenas lê dados)
 * 
 * DEPENDÊNCIAS:
 * -------------
 * - fsm_core (estados, T, Tc)
 * - network_coordinator (papel, vizinhos)
 * - tracking_manager (contagem de veículos)
 * - light_controller (duty_cycle actual)
 * - cJSON (construção de JSON)
 * 
 * @author Luis Custodio | Tiago Moreno
 * @date 2026-05-09
 * @version 1.0
 */

#ifndef WEB_DATA_PROVIDER_H
#define WEB_DATA_PROVIDER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <cJSON.h>

// ============================================================================
// ESTRUTURAS
// ============================================================================

/**
 * @brief Distribuição de tempo por estado de luz
 * 
 * Contabiliza quanto tempo o poste esteve em cada modo de operação
 * desde o arranque (ou desde o último reset).
 */
typedef struct {
    uint32_t save_seconds;      ///< Tempo em SAVE_MODE (duty 10%)
    uint32_t min_seconds;       ///< Tempo em LIGHT_MIN (duty 50%)
    uint32_t on_seconds;        ///< Tempo em LIGHT_ON (duty 100%)
    float save_percent;         ///< Percentagem em SAVE_MODE
    float min_percent;          ///< Percentagem em LIGHT_MIN
    float on_percent;           ///< Percentagem em LIGHT_ON
} time_distribution_t;

/**
 * @brief Estatísticas de consumo energético
 * 
 * Estimativas baseadas em:
 * - Potência do LED: 50W
 * - Duty cycles: 10% / 50% / 100%
 * - Tempo acumulado em cada modo
 */
typedef struct {
    float consumed_kwh;         ///< Consumo real acumulado (kWh)
    float full_on_kwh;          ///< Consumo se sempre 100% (kWh)
    float saved_kwh;            ///< Energia poupada (kWh)
    float saved_percent;        ///< Percentagem de poupança
    uint16_t power_w;           ///< Potência instalada (W)
} energy_stats_t;

/**
 * @brief Informação de um vizinho na rede
 */
typedef struct {
    uint8_t position;           ///< Posição física (0-N)
    char ip[16];                ///< IP fixo (ex: "192.168.1.101")
    bool is_alive;              ///< true se recebeu heartbeat recente
    uint32_t last_seen_ts;      ///< Timestamp último heartbeat
} neighbor_info_t;

// ============================================================================
// API PÚBLICA
// ============================================================================

/**
 * @brief Inicializa o módulo de agregação de dados
 * 
 * IMPORTANTE: Chamar DEPOIS de todos os módulos dependentes estarem prontos:
 * - fsm_core_init()
 * - network_coordinator_init()
 * - tracking_manager_init()
 * - light_controller_init()
 * 
 * @note Não aloca memória dinâmica
 * @note Thread-safe (usa mutex interno)
 */
void web_data_provider_init(void);

/**
 * @brief Obtém JSON com estado completo da linha de postes
 * 
 * FORMATO DE RETORNO:
 * ```json
 * {
 *   "system": {
 *     "uptime_s": 86400,
 *     "cpu_usage_percent": 23.5,
 *     "free_heap_kb": 180
 *   },
 *   "topology": {
 *     "master_position": 0,
 *     "active_slaves": 3,
 *     "last_election_ts": 1683456789
 *   },
 *   "postes": [
 *     {
 *       "position": 0,
 *       "ip": "192.168.1.100",
 *       "is_online": true,
 *       "role": "MASTER",
 *       "state": "TRAFIC",
 *       "T": 2,
 *       "Tc": 1,
 *       "duty_cycle": 100
 *     }
 *   ],
 *   "stats": {
 *     "total_vehicles": 245,
 *     "total_energy": 12.5,
 *     "energy_saved_percent": 47.9
 *   }
 * }
 * ```
 * 
 * @return Objecto cJSON* (DEVE SER LIBERTADO com cJSON_Delete após uso!)
 * @return NULL se erro
 * 
 * @warning Chamar cJSON_Delete() no ponteiro retornado!
 * 
 * @example
 * ```c
 * cJSON* json = web_data_get_line_status();
 * if (json) {
 *     char* str = cJSON_PrintUnformatted(json);  // Minificado
 *     httpd_resp_sendstr(req, str);
 *     free(str);
 *     cJSON_Delete(json);
 * }
 * ```
 */
cJSON* web_data_get_line_status(void);

/**
 * @brief Obtém JSON com dados de um poste específico
 * 
 * FORMATO DE RETORNO:
 * ```json
 * {
 *   "position": 0,
 *   "ip": "192.168.1.100",
 *   "role": "MASTER",
 *   "state": "TRAFIC",
 *   "T": 2,
 *   "Tc": 1,
 *   "duty_cycle": 100,
 *   "neighbors": [
 *     {"position": 1, "ip": "192.168.1.101", "is_alive": true}
 *   ],
 *   "time_stats": {
 *     "save_seconds": 23400,
 *     "save_percent": 65.0,
 *     "min_seconds": 9000,
 *     "min_percent": 25.0,
 *     "on_seconds": 3600,
 *     "on_percent": 10.0
 *   },
 *   "energy": {
 *     "consumed_kwh": 1.85,
 *     "full_on_kwh": 3.6,
 *     "saved_kwh": 1.75,
 *     "saved_percent": 48.6,
 *     "power_w": 50
 *   }
 * }
 * ```
 * 
 * @param position Posição física do poste (0-N)
 * @return cJSON* (DEVE SER LIBERTADO!)
 * @return NULL se posição inválida
 */
cJSON* web_data_get_poste_status(uint8_t position);

/**
 * @brief Obtém distribuição de tempo num período
 * 
 * @param start_ts Timestamp inicial (Unix epoch)
 * @param end_ts Timestamp final
 * @param[out] out Estrutura para preencher
 * 
 * @note Se start_ts == end_ts == 0, retorna desde arranque
 */
void web_data_get_time_distribution(uint32_t start_ts, uint32_t end_ts,
                                     time_distribution_t* out);

/**
 * @brief Obtém estatísticas de energia num período
 * 
 * @param start_ts Timestamp inicial
 * @param end_ts Timestamp final
 * @param[out] out Estrutura para preencher
 * 
 * @note Cálculo: consumed_kwh = (save_h * 10% + min_h * 50% + on_h * 100%) * 50W / 1000
 */
void web_data_get_energy_stats(uint32_t start_ts, uint32_t end_ts,
                                energy_stats_t* out);

/**
 * @brief Obtém lista de vizinhos activos
 * 
 * @param[out] neighbors Array para preencher (tamanho mínimo: 10)
 * @param[in] max_neighbors Tamanho do array
 * @return Número de vizinhos encontrados
 * 
 * @example
 * ```c
 * neighbor_info_t neighbors[10];
 * uint8_t count = web_data_get_neighbors(neighbors, 10);
 * for (uint8_t i = 0; i < count; i++) {
 *     printf("Vizinho #%d: %s (%s)\n", 
 *            neighbors[i].position,
 *            neighbors[i].ip,
 *            neighbors[i].is_alive ? "ONLINE" : "OFFLINE");
 * }
 * ```
 */
uint8_t web_data_get_neighbors(neighbor_info_t* neighbors, uint8_t max_neighbors);

/**
 * @brief Reset de estatísticas acumuladas (útil para testes)
 *
 * Zera contadores de tempo e energia, mantendo estado actual.
 */
void web_data_reset_stats(void);


// ============================================================================
// ESTATÍSTICAS NOCTURNAS (20h - 7h)
// ============================================================================

/** Número de horas no período nocturno (20h, 21h, 22h, 23h, 0h, 1h, 2h, 3h, 4h, 5h, 6h) */
#define NIGHT_BUCKETS  11

/**
 * @brief Dados acumulados numa hora do período nocturno
 */
typedef struct {
    uint8_t  hour;        ///< Hora real 0-23
    char     label[5];    ///< "20h", "0h", etc.
    uint32_t vehicles;    ///< Veículos detectados nesta hora
    float    energy_wh;   ///< Energia consumida (Wh), incluindo fade
} night_bucket_t;

/**
 * @brief Obtém JSON com estatísticas nocturnas horárias
 *
 * Período: 20:00 → 07:00. Reinicia automaticamente ao início de cada noite.
 * A energia inclui períodos de fade-up e fade-down (amostrado a 10s).
 *
 * FORMATO:
 * {
 *   "synced": true,
 *   "hour": 22,
 *   "buckets": [{"h":20,"l":"20h","v":12,"e":18.5}, ...],
 *   "veh": 45,
 *   "wh": 120.3
 * }
 *
 * @return cJSON* (chamar cJSON_Delete após uso). NULL se erro.
 */
cJSON *web_data_get_night_stats(void);

/**
 * @brief Obtém JSON com estado actual do poste (tempo real)
 *
 * FORMATO:
 * {
 *   "name": "POSTE 03",
 *   "state": "IDLE",
 *   "duty": 5,
 *   "T": 0, "Tc": 0,
 *   "radar": "REAL",
 *   "ip": "192.168.4.3",
 *   "role": "SLAVE",
 *   "neb_l_ip": "192.168.4.2", "neb_l_ok": true,
 *   "neb_r_ip": "---",         "neb_r_ok": false,
 *   "uptime_s": 3600
 * }
 *
 * @return cJSON* (chamar cJSON_Delete após uso). NULL se erro.
 */
cJSON *web_data_get_status(void);

#ifdef __cplusplus
}
#endif

#endif // WEB_DATA_PROVIDER_H
