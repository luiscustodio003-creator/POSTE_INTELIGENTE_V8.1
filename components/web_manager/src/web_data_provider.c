/**
 * @file web_data_provider.c
 * @brief Implementação do Agregador de Dados
 * 
 * NOTA IMPORTANTE:
 * ----------------
 * Este ficheiro assume que existem funções externas nos módulos dependentes.
 * Se os teus módulos tiverem nomes diferentes, ajusta as chamadas!
 * 
 * FUNÇÕES EXTERNAS ESPERADAS:
 * - fsm_core_get_state() → retorna estado actual
 * - fsm_core_get_T() → retorna contador T
 * - fsm_core_get_Tc() → retorna contador Tc
 * - fsm_core_get_duty_cycle() → retorna duty actual (0-100)
 * - network_coordinator_get_role() → retorna MASTER/SLAVE
 * - network_coordinator_get_neighbors() → lista vizinhos
 * - tracking_manager_get_vehicle_count() → total veículos
 * 
 * @author Luis Custodio | Tiago Moreno
 * @date 2026-05-09
 */

#include "web_data_provider.h"
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <sys/time.h>
#include <string.h>

// ============================================================================
// CONFIGURAÇÃO
// ============================================================================

static const char* TAG = "WEB_DATA";

/// Potência do LED em Watts
#define LED_POWER_W         50.0f

/// Duty cycles
#define DUTY_SAVE_PERCENT   0.10f  // 10%
#define DUTY_MIN_PERCENT    0.50f  // 50%
#define DUTY_ON_PERCENT     1.00f  // 100%

// ============================================================================
// VARIÁVEIS PRIVADAS
// ============================================================================

/// Contadores de tempo acumulado (segundos)
static uint32_t time_in_save_s = 0;
static uint32_t time_in_min_s = 0;
static uint32_t time_in_on_s = 0;

/// Timestamp da última actualização
static uint32_t last_update_ts = 0;

/// Duty cycle anterior (para detectar mudanças)
static uint8_t last_duty = 0;

// ============================================================================
// FUNÇÕES AUXILIARES PRIVADAS
// ============================================================================

/**
 * @brief Actualiza contadores de tempo baseado no duty actual
 * 
 * Chamado internamente antes de gerar JSON para garantir dados frescos.
 */
static void update_time_counters(void) {
    uint32_t now = esp_timer_get_time() / 1000000; // segundos
    
    if (last_update_ts == 0) {
        last_update_ts = now;
        return;
    }
    
    uint32_t delta = now - last_update_ts;
    if (delta == 0) return; // Sem mudança
    
    // Classificar duty actual
    // NOTA: Ajusta estes valores se os teus duty cycles forem diferentes!
    if (last_duty <= 15) {
        time_in_save_s += delta;
    } else if (last_duty <= 60) {
        time_in_min_s += delta;
    } else {
        time_in_on_s += delta;
    }
    
    last_update_ts = now;
}

/**
 * @brief Obtém string do estado actual
 * 
 * NOTA: Ajusta conforme os nomes dos teus estados!
 */
static const char* get_state_string(void) {
    // TODO: Chamar fsm_core_get_state() aqui
    // Por agora retorna placeholder
    return "IDLE";  // Valores possíveis: "IDLE", "TRAFIC", "OBSTACULO"
}

/**
 * @brief Obtém papel na rede (MASTER/SLAVE)
 */
static const char* get_role_string(void) {
    // TODO: Chamar network_coordinator_get_role()
    return "SLAVE";  // Placeholder
}

/**
 * @brief Obtém IP do poste baseado na posição
 * 
 * Assume IP fixo: 192.168.1.(100 + position)
 */
static void get_ip_from_position(uint8_t position, char* ip_out) {
    snprintf(ip_out, 16, "192.168.1.%d", 100 + position);
}

// ============================================================================
// IMPLEMENTAÇÃO DA API PÚBLICA
// ============================================================================

void web_data_provider_init(void) {
    ESP_LOGI(TAG, "A iniciar agregador de dados...");
    
    // Inicializar timestamp
    last_update_ts = esp_timer_get_time() / 1000000;
    
    // TODO: Aqui poderias validar que os módulos dependentes estão activos
    // Exemplo: if (!fsm_core_is_initialized()) { ESP_LOGE(...); return; }
    
    ESP_LOGI(TAG, "✅ Agregador de dados pronto");
}

cJSON* web_data_get_line_status(void) {
    update_time_counters();
    
    cJSON* root = cJSON_CreateObject();
    if (!root) {
        ESP_LOGE(TAG, "Erro ao criar JSON raiz");
        return NULL;
    }
    
    // ========== SYSTEM ==========
    cJSON* system = cJSON_CreateObject();
    cJSON_AddNumberToObject(system, "uptime_s", esp_timer_get_time() / 1000000);
    cJSON_AddNumberToObject(system, "free_heap_kb", esp_get_free_heap_size() / 1024);
    cJSON_AddItemToObject(root, "system", system);
    
    // ========== TOPOLOGY ==========
    cJSON* topology = cJSON_CreateObject();
    cJSON_AddNumberToObject(topology, "master_position", 0);  // TODO: obter dinamicamente
    cJSON_AddNumberToObject(topology, "active_slaves", 3);    // TODO: contar vizinhos vivos
    cJSON_AddNumberToObject(topology, "last_election_ts", last_update_ts);
    cJSON_AddItemToObject(root, "topology", topology);
    
    // ========== POSTES ==========
    cJSON* postes = cJSON_CreateArray();
    
    // Exemplo: Adicionar este poste (position = 0)
    // TODO: Obter posição actual do sistema
    cJSON* poste = cJSON_CreateObject();
    cJSON_AddNumberToObject(poste, "position", 0);
    cJSON_AddStringToObject(poste, "ip", "192.168.1.100");
    cJSON_AddBoolToObject(poste, "is_online", true);
    cJSON_AddStringToObject(poste, "role", get_role_string());
    cJSON_AddStringToObject(poste, "state", get_state_string());
    cJSON_AddNumberToObject(poste, "T", 0);  // TODO: fsm_core_get_T()
    cJSON_AddNumberToObject(poste, "Tc", 0); // TODO: fsm_core_get_Tc()
    cJSON_AddNumberToObject(poste, "duty_cycle", 10); // TODO: fsm_core_get_duty_cycle()
    cJSON_AddItemToArray(postes, poste);
    
    cJSON_AddItemToObject(root, "postes", postes);
    
    // ========== STATS ==========
    cJSON* stats_obj = cJSON_CreateObject();
    cJSON_AddNumberToObject(stats_obj, "total_vehicles", 0); // TODO: tracking_manager_get_count()
    
    // Calcular energia
    energy_stats_t energy;
    web_data_get_energy_stats(0, 0, &energy);
    cJSON_AddNumberToObject(stats_obj, "total_energy", energy.consumed_kwh);
    cJSON_AddNumberToObject(stats_obj, "energy_saved_percent", energy.saved_percent);
    
    cJSON_AddItemToObject(root, "stats", stats_obj);
    
    return root;
}

cJSON* web_data_get_poste_status(uint8_t position) {
    if (position > 9) {
        ESP_LOGE(TAG, "Posição inválida: %d", position);
        return NULL;
    }
    
    update_time_counters();
    
    cJSON* root = cJSON_CreateObject();
    if (!root) return NULL;
    
    // ========== DADOS BÁSICOS ==========
    char ip[16];
    get_ip_from_position(position, ip);
    
    cJSON_AddNumberToObject(root, "position", position);
    cJSON_AddStringToObject(root, "ip", ip);
    cJSON_AddStringToObject(root, "role", get_role_string());
    cJSON_AddStringToObject(root, "state", get_state_string());
    cJSON_AddNumberToObject(root, "T", 0);  // TODO: obter real
    cJSON_AddNumberToObject(root, "Tc", 0);
    cJSON_AddNumberToObject(root, "duty_cycle", 10);
    
    // ========== VIZINHOS ==========
    cJSON* neighbors = cJSON_CreateArray();
    
    // Exemplo: vizinho fictício
    cJSON* neighbor = cJSON_CreateObject();
    cJSON_AddNumberToObject(neighbor, "position", 1);
    cJSON_AddStringToObject(neighbor, "ip", "192.168.1.101");
    cJSON_AddBoolToObject(neighbor, "is_alive", true);
    cJSON_AddItemToArray(neighbors, neighbor);
    
    cJSON_AddItemToObject(root, "neighbors", neighbors);
    
    // ========== TIME STATS ==========
    time_distribution_t time_dist;
    web_data_get_time_distribution(0, 0, &time_dist);
    
    cJSON* time_stats = cJSON_CreateObject();
    cJSON_AddNumberToObject(time_stats, "save_seconds", time_dist.save_seconds);
    cJSON_AddNumberToObject(time_stats, "save_percent", time_dist.save_percent);
    cJSON_AddNumberToObject(time_stats, "min_seconds", time_dist.min_seconds);
    cJSON_AddNumberToObject(time_stats, "min_percent", time_dist.min_percent);
    cJSON_AddNumberToObject(time_stats, "on_seconds", time_dist.on_seconds);
    cJSON_AddNumberToObject(time_stats, "on_percent", time_dist.on_percent);
    cJSON_AddItemToObject(root, "time_stats", time_stats);
    
    // ========== ENERGY ==========
    energy_stats_t energy;
    web_data_get_energy_stats(0, 0, &energy);
    
    cJSON* energy_obj = cJSON_CreateObject();
    cJSON_AddNumberToObject(energy_obj, "consumed_kwh", energy.consumed_kwh);
    cJSON_AddNumberToObject(energy_obj, "full_on_kwh", energy.full_on_kwh);
    cJSON_AddNumberToObject(energy_obj, "saved_kwh", energy.saved_kwh);
    cJSON_AddNumberToObject(energy_obj, "saved_percent", energy.saved_percent);
    cJSON_AddNumberToObject(energy_obj, "power_w", energy.power_w);
    cJSON_AddItemToObject(root, "energy", energy_obj);
    
    return root;
}

void web_data_get_time_distribution(uint32_t start_ts, uint32_t end_ts,
                                     time_distribution_t* out) {
    if (!out) return;
    
    update_time_counters();
    
    // Se timestamps == 0, retorna desde arranque
    out->save_seconds = time_in_save_s;
    out->min_seconds = time_in_min_s;
    out->on_seconds = time_in_on_s;
    
    uint32_t total = time_in_save_s + time_in_min_s + time_in_on_s;
    
    if (total > 0) {
        out->save_percent = (time_in_save_s * 100.0f) / total;
        out->min_percent = (time_in_min_s * 100.0f) / total;
        out->on_percent = (time_in_on_s * 100.0f) / total;
    } else {
        out->save_percent = 0.0f;
        out->min_percent = 0.0f;
        out->on_percent = 0.0f;
    }
}

void web_data_get_energy_stats(uint32_t start_ts, uint32_t end_ts,
                                energy_stats_t* out) {
    if (!out) return;
    
    update_time_counters();
    
    // Converter segundos para horas
    float save_h = time_in_save_s / 3600.0f;
    float min_h = time_in_min_s / 3600.0f;
    float on_h = time_in_on_s / 3600.0f;
    
    // Consumo real (kWh)
    float energy_save = LED_POWER_W * DUTY_SAVE_PERCENT * save_h / 1000.0f;
    float energy_min = LED_POWER_W * DUTY_MIN_PERCENT * min_h / 1000.0f;
    float energy_on = LED_POWER_W * DUTY_ON_PERCENT * on_h / 1000.0f;
    
    out->consumed_kwh = energy_save + energy_min + energy_on;
    
    // Consumo se sempre 100%
    float total_h = save_h + min_h + on_h;
    out->full_on_kwh = LED_POWER_W * total_h / 1000.0f;
    
    // Poupança
    out->saved_kwh = out->full_on_kwh - out->consumed_kwh;
    
    if (out->full_on_kwh > 0.001f) {
        out->saved_percent = (out->saved_kwh / out->full_on_kwh) * 100.0f;
    } else {
        out->saved_percent = 0.0f;
    }
    
    out->power_w = (uint16_t)LED_POWER_W;
}

uint8_t web_data_get_neighbors(neighbor_info_t* neighbors, uint8_t max_neighbors) {
    if (!neighbors || max_neighbors == 0) return 0;
    
    // TODO: Chamar network_coordinator_get_neighbors() aqui
    // Por agora retorna exemplo fixo
    
    neighbors[0].position = 1;
    strcpy(neighbors[0].ip, "192.168.1.101");
    neighbors[0].is_alive = true;
    neighbors[0].last_seen_ts = last_update_ts;
    
    return 1; // Retornou 1 vizinho
}

void web_data_reset_stats(void) {
    time_in_save_s = 0;
    time_in_min_s = 0;
    time_in_on_s = 0;
    last_update_ts = esp_timer_get_time() / 1000000;
    
    ESP_LOGI(TAG, "Estatísticas reiniciadas");
}
