# ============================================================================
# CMakeLists.txt - Componente Web Manager
# Projecto: Poste Inteligente v8
# Autores: Luis Custodio | Tiago Moreno
# Data: 2026-05-09
# ============================================================================

idf_component_register(
    SRCS 
        "src/web_manager.c"
        "src/web_data_provider.c"
        "src/web_html_pages.c"
    
    INCLUDE_DIRS 
        "include"
    
    REQUIRES 
        esp_http_server
        json
        nvs_flash
        esp_wifi
        esp_timer 
)

# Optimizações de compilação
target_compile_options(${COMPONENT_LIB} PRIVATE 
    -Os              # Optimizar para tamanho
    -ffunction-sections
    -fdata-sections
)
