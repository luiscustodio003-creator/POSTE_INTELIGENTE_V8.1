/* ============================================================
   PATCH FINAL: web_data_provider.c COMPLETO E INTEGRADO
   @file      web_data_provider_FINAL.c
   @version   1.1  |  2026-05-11
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custodio | Tiago Moreno
   
   INSTRUÇÕES:
   ───────────
   SUBSTITUIR o ficheiro web_data_provider.c actual por este.
   
   ALTERAÇÕES APLICADAS:
   ─────────────────────
   ✅ Includes adicionados (state_machine.h, fsm_core.h, etc.)
   ✅ get_state_string() integrada com FSM
   ✅ get_role_string() integrada com comm_manager
   ✅ get_ip_from_position() corrigida (192.168.4.X)
   ✅ update_time_counters() usa duty real
   ✅ web_data_get_line_status() valores dinâmicos (T/Tc/duty)
   ✅ web_data_get_poste_status() valores dinâmicos
   
   DEPENDÊNCIAS EXTERNAS:
   ──────────────────────
   - state_machine_get_state_name() → state_machine.h
   - state_machine_get_T/Tc() → state_machine.h
   - fsm_core_get_duty_cycle() → fsm_core.h (ADICIONAR!)
   - comm_is_master() → comm_manager.h
   - comm_left_online() → comm_manager.h
   - POST_POSITION → system_config.h
   
   NOTA CRÍTICA:
   ─────────────
   Verificar que fsm_core.h TEM a declaração:
   uint8_t fsm_core_get_duty_cycle(void);
   
   Se NÃO tiver, adicionar conforme PATCH_fsm_getters_webmanager.h
============================================================ */

#include "web_data_provider.h"
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <sys/time.h>
#include <string.h>

/* ── INCLUDES ADICIONADOS PARA INTEGRAÇÃO ─────────────────── */
#include "state_machine.h"      /* state_machine_get_state_name(), get_T/Tc() */
#include "fsm_core.h"            /* fsm_core_get_duty_cycle() */
#include "comm_manager.h"        /* comm_is_master(), comm_left_online() */
#include "system_config.h"       /* POST_POSITION */

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
 * ✅ INTEGRADO: Usa fsm_core_get_duty_cycle() para duty real
 */
static void update_time_counters(void) {
    uint32_t now = esp_timer_get_time() / 1000000; // segundos
    
    if (last_update_ts == 0) {
        last_update_ts = now;
        return;
    }
    
    uint32_t delta = now - last_update_ts;
    if (delta == 0) return; // Sem mudança
    
    /* ✅ INTEGRADO: Obter duty actual da FSM */
    uint8_t current_duty = fsm_core_get_duty_cycle();
    
    /* Classificar e acumular tempo */
    if (current_duty <= 15) {
        time_in_save_s += delta;
    } else if (current_duty <= 60) {
        time_in_min_s += delta;
    } else {
        time_in_on_s += delta;
    }
    
    last_duty = current_duty;  /* Guardar para próximo ciclo */
    last_update_ts = now;
}

/**
 * @brief Obtém string do estado actual
 * 
 * ✅ INTEGRADO: Usa state_machine_get_state_name()
 */
static const char* get_state_string(void) {
    /* state_machine.h já expõe esta função (linha 144) */
    return state_machine_get_state_name();
}

/**
 * @brief Obtém papel na rede (MASTER/SLAVE)
 * 
 * ✅ INTEGRADO: Usa comm_is_master()
 */
static const char* get_role_string(void) {
    /* comm_manager.h expõe comm_is_master() */
    return comm_is_master() ? "MASTER" : "SLAVE";
}

/**
 * @brief Obtém IP do poste baseado na posição
 * 
 * ✅ CORRIGIDO: Usa 192.168.4.X (alinhado com wifi_manager v2.0)
 * 
 * Esquema IP fixo:
 * - pos=0 → 192.168.4.1 (AP do master)
 * - pos>0 → 192.168.4.(pos+1) (STA)
 */
static void get_ip_from_position(uint8_t position, char* ip_out) {
    if (position == 0) {
        snprintf(ip_out, 16, "192.168.4.1");  /* AP do master */
    } else {
        snprintf(ip_out, 16, "192.168.4.%d", position + 1);  /* STA */
    }
}

// ============================================================================
// IMPLEMENTAÇÃO DA API PÚBLICA
// ============================================================================

void web_data_provider_init(void) {
    ESP_LOGI(TAG, "A iniciar agregador de dados...");
    
    // Inicializar timestamp
    last_update_ts = esp_timer_get_time() / 1000000;
    
    /* Validação opcional: verificar que módulos dependentes estão activos */
    /* Comentado porque nem todos os módulos expõem is_initialized() */
    // if (!comm_status_ok()) {
    //     ESP_LOGW(TAG, "Comm_manager ainda não está pronto");
    // }
    
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
    
    /* ✅ INTEGRADO: master_position dinâmico */
    cJSON_AddNumberToObject(topology, "master_position", 
                            comm_is_master() ? POST_POSITION : 0);
    
    /* ✅ INTEGRADO: contar vizinhos vivos */
    /* Conta vizinho esquerdo + direito (se online) */
    extern bool g_fsm_right_online;  /* Declarado em fsm_core.c */
    uint8_t active_neighbors = (comm_left_online() ? 1 : 0) + 
                               (g_fsm_right_online ? 1 : 0);
    cJSON_AddNumberToObject(topology, "active_slaves", active_neighbors);
    
    cJSON_AddNumberToObject(topology, "last_election_ts", last_update_ts);
    cJSON_AddItemToObject(root, "topology", topology);
    
    // ========== POSTES ==========
    cJSON* postes = cJSON_CreateArray();
    
    /* Adicionar este poste (POST_POSITION) */
    cJSON* poste = cJSON_CreateObject();
    
    cJSON_AddNumberToObject(poste, "position", POST_POSITION);
    
    /* ✅ CORRIGIDO: IP baseado em esquema 192.168.4.X */
    char ip[16];
    get_ip_from_position(POST_POSITION, ip);
    cJSON_AddStringToObject(poste, "ip", ip);
    
    cJSON_AddBoolToObject(poste, "is_online", true);  /* Este poste está sempre online */
    
    /* ✅ INTEGRADO: Papel dinâmico */
    cJSON_AddStringToObject(poste, "role", get_role_string());
    
    /* ✅ INTEGRADO: Estado dinâmico */
    cJSON_AddStringToObject(poste, "state", get_state_string());
    
    /* ✅ INTEGRADO: Contadores T/Tc reais */
    cJSON_AddNumberToObject(poste, "T", state_machine_get_T());
    cJSON_AddNumberToObject(poste, "Tc", state_machine_get_Tc());
    
    /* ✅ INTEGRADO: Duty cycle real */
    cJSON_AddNumberToObject(poste, "duty_cycle", fsm_core_get_duty_cycle());
    
    cJSON_AddItemToArray(postes, poste);
    
    cJSON_AddItemToObject(root, "postes", postes);
    
    // ========== STATS ==========
    cJSON* stats_obj = cJSON_CreateObject();
    
    /* ✅ INTEGRADO: Total de veículos (T + Tc como aproximação) */
    /* NOTA: Se implementares tracking_manager_get_vehicle_count(), usar aqui */
    cJSON_AddNumberToObject(stats_obj, "total_vehicles", 
                            state_machine_get_T() + state_machine_get_Tc());
    
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
    
    /* NOTA: Esta versão só retorna dados do PRÓPRIO poste (POST_POSITION)
       Para suportar múltiplos postes, seria necessário protocolo UDP adicional
       para trocar dados de status entre postes. Por agora, apenas validamos
       que a posição solicitada é a nossa. */
    
    if (position != POST_POSITION) {
        ESP_LOGW(TAG, "Pedido de posição %d mas só temos dados de %d", 
                 position, POST_POSITION);
        /* Poderíamos retornar NULL aqui, mas por compatibilidade com HTML
           que pode chamar /api/poste/X sem saber a posição actual,
           retornamos os nossos dados mesmo assim. */
    }
    
    update_time_counters();
    
    cJSON* root = cJSON_CreateObject();
    if (!root) return NULL;
    
    // ========== DADOS BÁSICOS ==========
    char ip[16];
    get_ip_from_position(POST_POSITION, ip);  /* Usar posição real, não pedida */
    
    cJSON_AddNumberToObject(root, "position", POST_POSITION);
    cJSON_AddStringToObject(root, "ip", ip);
    
    /* ✅ INTEGRADO: Valores reais */
    cJSON_AddStringToObject(root, "role", get_role_string());
    cJSON_AddStringToObject(root, "state", get_state_string());
    cJSON_AddNumberToObject(root, "T", state_machine_get_T());
    cJSON_AddNumberToObject(root, "Tc", state_machine_get_Tc());
    cJSON_AddNumberToObject(root, "duty_cycle", fsm_core_get_duty_cycle());
    
    // ========== VIZINHOS ==========
    cJSON* neighbors = cJSON_CreateArray();
    
    /* Adicionar vizinho esquerdo (pos-1) se online */
    if (POST_POSITION > 0 && comm_left_online()) {
        cJSON* left_neighbor = cJSON_CreateObject();
        cJSON_AddNumberToObject(left_neighbor, "position", POST_POSITION - 1);
        get_ip_from_position(POST_POSITION - 1, ip);
        cJSON_AddStringToObject(left_neighbor, "ip", ip);
        cJSON_AddBoolToObject(left_neighbor, "is_alive", true);
        cJSON_AddItemToArray(neighbors, left_neighbor);
    }
    
    /* Adicionar vizinho direito (pos+1) se online */
    extern bool g_fsm_right_online;
    if (g_fsm_right_online) {
        cJSON* right_neighbor = cJSON_CreateObject();
        cJSON_AddNumberToObject(right_neighbor, "position", POST_POSITION + 1);
        get_ip_from_position(POST_POSITION + 1, ip);
        cJSON_AddStringToObject(right_neighbor, "ip", ip);
        cJSON_AddBoolToObject(right_neighbor, "is_alive", true);
        cJSON_AddItemToArray(neighbors, right_neighbor);
    }
    
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
    
    (void)start_ts;  /* Não usado nesta versão */
    (void)end_ts;    /* Não usado nesta versão */
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
    
    (void)start_ts;  /* Não usado nesta versão */
    (void)end_ts;    /* Não usado nesta versão */
}

uint8_t web_data_get_neighbors(neighbor_info_t* neighbors, uint8_t max_neighbors) {
    if (!neighbors || max_neighbors == 0) return 0;
    
    uint8_t count = 0;
    
    /* Vizinho esquerdo (pos-1) */
    if (POST_POSITION > 0 && comm_left_online() && count < max_neighbors) {
        neighbors[count].position = POST_POSITION - 1;
        get_ip_from_position(POST_POSITION - 1, neighbors[count].ip);
        neighbors[count].is_alive = true;
        neighbors[count].last_seen_ts = last_update_ts;
        count++;
    }
    
    /* Vizinho direito (pos+1) */
    extern bool g_fsm_right_online;
    if (g_fsm_right_online && count < max_neighbors) {
        neighbors[count].position = POST_POSITION + 1;
        get_ip_from_position(POST_POSITION + 1, neighbors[count].ip);
        neighbors[count].is_alive = true;
        neighbors[count].last_seen_ts = last_update_ts;
        count++;
    }
    
    return count;
}

void web_data_reset_stats(void) {
    time_in_save_s = 0;
    time_in_min_s = 0;
    time_in_on_s = 0;
    last_update_ts = esp_timer_get_time() / 1000000;
    
    ESP_LOGI(TAG, "Estatísticas reiniciadas");
}


/* ════════════════════════════════════════════════════════════
   FIM DO FICHEIRO INTEGRADO
   ════════════════════════════════════════════════════════════
   
   DEPENDÊNCIAS EXTERNAS NECESSÁRIAS:
   ──────────────────────────────────────────────────────────
   
   1. fsm_core.h → Adicionar:
      uint8_t fsm_core_get_duty_cycle(void);
      uint16_t fsm_core_get_last_vehicle_id(void);
   
   2. fsm_core.c → Implementar (ver PATCH_fsm_getters_implementation.c)
   
   3. comm_manager.h → Já tem:
      bool comm_is_master(void);
      bool comm_left_online(void);
   
   4. state_machine.h → Já tem:
      const char* state_machine_get_state_name(void);
      int state_machine_get_T(void);
      int state_machine_get_Tc(void);
   
   5. system_config.h → Já tem:
      POST_POSITION
   
   
   COMPILAÇÃO:
   ───────────
   $ idf.py build
   
   
   TESTE:
   ──────
   $ curl http://192.168.4.1/api/line | jq
   
   Verificar:
   {
     "postes": [{
       "state": "LIGHT_ON",    ← Não mais sempre "IDLE"
       "role": "MASTER",       ← Dinâmico
       "T": 2,                 ← Não mais sempre 0
       "Tc": 1,                ← Não mais sempre 0
       "duty_cycle": 100       ← Não mais sempre 10
     }]
   }
   
============================================================ */
