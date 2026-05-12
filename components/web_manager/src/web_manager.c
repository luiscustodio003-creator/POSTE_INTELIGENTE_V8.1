/**
 * @file web_manager.c
 * @brief Implementação do Servidor HTTP
 * 
 * @author Luis Custodio | Tiago Moreno
 * @date 2026-05-09
 */

#include "web_manager.h"
#include "web_data_provider.h"
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <string.h>

// ============================================================================
// CONSTANTES E VARIÁVEIS PRIVADAS
// ============================================================================

static const char* TAG = "WEB_MGR";

/// Handle do servidor HTTP (NULL se não activo)
static httpd_handle_t server = NULL;

/// Contador de clientes activos
static uint8_t active_clients = 0;

/// Estatísticas do servidor
static web_stats_t stats = {0};

/// Timestamp de inicialização
static uint32_t init_timestamp = 0;

// ============================================================================
// DECLARAÇÃO DE PÁGINAS HTML (implementadas em web_html_pages.c)
// ============================================================================

extern const char* get_dashboard_html(void);
extern const char* get_poste_detail_html(void);

// ============================================================================
// HANDLERS DE URLs
// ============================================================================

/**
 * @brief Handler para GET /
 * Redireciona para /dashboard
 */
static esp_err_t handler_root(httpd_req_t *req) {
    stats.total_requests++;
    
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/dashboard");
    httpd_resp_send(req, NULL, 0);
    
    return ESP_OK;
}

/**
 * @brief Handler para GET /dashboard
 * Serve página principal com lista de postes
 */
static esp_err_t handler_dashboard(httpd_req_t *req) {
    stats.total_requests++;
    
    // Verificar limite de clientes
    if (active_clients >= WEB_MAX_CONNECTIONS) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Servidor ocupado. Tenta novamente em breve.");
        stats.failed_requests++;
        return ESP_FAIL;
    }
    
    active_clients++;
    
    // Obter HTML
    const char* html = get_dashboard_html();
    
    // Enviar resposta
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, html, strlen(html));
    
    active_clients--;
    
    ESP_LOGI(TAG, "Dashboard servido (clientes: %d)", active_clients);
    return ESP_OK;
}

/**
 * @brief Handler para GET /poste/{position}
 * Serve página de detalhes de um poste específico
 */
static esp_err_t handler_poste_detail(httpd_req_t *req) {
    stats.total_requests++;
    
    // Verificar limite
    if (active_clients >= WEB_MAX_CONNECTIONS) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Servidor ocupado.");
        stats.failed_requests++;
        return ESP_FAIL;
    }
    
    active_clients++;
    
    // Extrair posição da URL (ex: /poste/0)
    char uri[32];
    strncpy(uri, req->uri, sizeof(uri) - 1);
    
    // Obter HTML
    const char* html = get_poste_detail_html();
    
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, html, strlen(html));
    
    active_clients--;
    
    ESP_LOGI(TAG, "Poste detail servido: %s", uri);
    return ESP_OK;
}

/**
 * @brief Handler para GET /api/line
 * Retorna JSON com estado completo da linha
 */
static esp_err_t handler_api_line(httpd_req_t *req) {
    stats.total_requests++;
    
    // Obter JSON
    cJSON* json = web_data_get_line_status();
    if (!json) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Erro ao gerar dados");
        stats.failed_requests++;
        return ESP_FAIL;
    }
    
    // Converter para string MINIFICADA (sem espaços)
    char* json_str = cJSON_PrintUnformatted(json);
    
    // Enviar resposta
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"); // CORS
    httpd_resp_send(req, json_str, strlen(json_str));
    
    // Libertar memória
    size_t json_size = strlen(json_str);  // ← ADICIONAR
    free(json_str);
    cJSON_Delete(json);
    ESP_LOGI(TAG, "API /line servido (%.1f KB)", json_size / 1024.0f);  // ✅   
    return ESP_OK;
}

/**
 * @brief Handler para GET /api/poste/{position}
 * Retorna JSON com dados de um poste específico
 */
static esp_err_t handler_api_poste(httpd_req_t *req) {
    stats.total_requests++;
    
    // Extrair posição da URL
    const char* uri = req->uri;
    uint8_t position = 0;
    
    // Procurar último '/' e converter número seguinte
    const char* last_slash = strrchr(uri, '/');
    if (last_slash && *(last_slash + 1) >= '0' && *(last_slash + 1) <= '9') {
        position = atoi(last_slash + 1);
    }
    
    // Validar posição (assumindo max 10 postes)
    if (position > 9) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Posição inválida");
        stats.failed_requests++;
        return ESP_FAIL;
    }
    
    // Obter JSON
    cJSON* json = web_data_get_poste_status(position);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Poste não encontrado");
        stats.failed_requests++;
        return ESP_FAIL;
    }
    
    // Converter e enviar
    char* json_str = cJSON_PrintUnformatted(json);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json_str, strlen(json_str));
    
    free(json_str);
    cJSON_Delete(json);
    
    ESP_LOGI(TAG, "API /poste/%d servido", position);
    return ESP_OK;
}

// ============================================================================
// CONFIGURAÇÃO DE ROTAS
// ============================================================================

static const httpd_uri_t uri_root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = handler_root,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_dashboard = {
    .uri       = "/dashboard",
    .method    = HTTP_GET,
    .handler   = handler_dashboard,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_poste_detail = {
    .uri       = "/poste/*",  // Wildcard para /poste/0, /poste/1, etc.
    .method    = HTTP_GET,
    .handler   = handler_poste_detail,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_api_line = {
    .uri       = "/api/line",
    .method    = HTTP_GET,
    .handler   = handler_api_line,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_api_poste = {
    .uri       = "/api/poste/*",
    .method    = HTTP_GET,
    .handler   = handler_api_poste,
    .user_ctx  = NULL
};

// ============================================================================
// IMPLEMENTAÇÃO DA API PÚBLICA
// ============================================================================

esp_err_t web_manager_init(void) {
    if (server != NULL) {
        ESP_LOGW(TAG, "Servidor já está activo");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "A iniciar servidor HTTP...");
    
    // Configuração do servidor
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WEB_SERVER_PORT;
    config.max_open_sockets = WEB_MAX_CONNECTIONS + 1; // +1 para margem
    config.lru_purge_enable = true;
    config.recv_wait_timeout = WEB_SOCKET_TIMEOUT_MS / 1000;
    config.send_wait_timeout = WEB_SOCKET_TIMEOUT_MS / 1000;
    
    // Iniciar servidor
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao iniciar servidor: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Registar rotas
    httpd_register_uri_handler(server, &uri_root);
    httpd_register_uri_handler(server, &uri_dashboard);
    httpd_register_uri_handler(server, &uri_poste_detail);
    httpd_register_uri_handler(server, &uri_api_line);
    httpd_register_uri_handler(server, &uri_api_poste);
    
    // Guardar timestamp de arranque
    init_timestamp = esp_timer_get_time() / 1000000;
    
    ESP_LOGI(TAG, "✅ Servidor HTTP activo na porta %d", WEB_SERVER_PORT);
    ESP_LOGI(TAG, "   Acesso: http://<IP_DO_POSTE>/");
    ESP_LOGI(TAG, "   Limite: %d cliente(s) simultâneo(s)", WEB_MAX_CONNECTIONS);
    
    return ESP_OK;
}

void web_manager_stop(void) {
    if (server == NULL) {
        ESP_LOGW(TAG, "Servidor já está parado");
        return;
    }
    
    ESP_LOGI(TAG, "A parar servidor HTTP...");
    
    httpd_stop(server);
    server = NULL;
    active_clients = 0;
    
    ESP_LOGI(TAG, "✅ Servidor HTTP parado");
}

bool web_manager_is_running(void) {
    return (server != NULL);
}

uint8_t web_manager_get_active_sessions(void) {
    return active_clients;
}

void web_manager_get_stats(web_stats_t* out_stats) {
    if (!out_stats) return;
    
    memcpy(out_stats, &stats, sizeof(web_stats_t));
    
    // Actualizar uptime
    if (init_timestamp > 0) {
        uint32_t now = esp_timer_get_time() / 1000000;
        out_stats->uptime_seconds = now - init_timestamp;
    }
}

void web_manager_reset_stats(void) {
    stats.total_requests = 0;
    stats.failed_requests = 0;
    stats.active_connections = 0;
    
    ESP_LOGI(TAG, "Estatísticas reiniciadas");
}
