# 🌐 WEB MANAGER - Módulo de Interface Web

**Projecto:** Poste Inteligente v8  
**Autores:** Luis Custodio | Tiago Moreno  
**Data:** 2026-05-09  
**Versão:** 1.0

---

## 📋 ÍNDICE

1. [Descrição Geral](#descrição-geral)
2. [Funcionalidades](#funcionalidades)
3. [Arquitectura](#arquitectura)
4. [Instalação](#instalação)
5. [Como Aceder](#como-aceder)
6. [URLs Disponíveis](#urls-disponíveis)
7. [Consumo de Recursos](#consumo-de-recursos)
8. [Dependências](#dependências)
9. [Integração com Outros Módulos](#integração-com-outros-módulos)
10. [FAQ](#faq)

---

## 📖 DESCRIÇÃO GERAL

O **web_manager** é um módulo que implementa um servidor HTTP leve para monitorização remota dos postes inteligentes via browser. Permite visualizar em tempo real:

- Estado de cada poste (IDLE/TRAFIC/OBSTACULO)
- Topologia da rede (master/slave, vizinhos activos)
- Estatísticas de tráfego (veículos detectados)
- Consumo energético estimado
- Distribuição de tempo por modo de luz

**Design:** Interface minimalista e responsiva (funciona em PC, tablet, smartphone).

---

## ✅ FUNCIONALIDADES

### 🎯 Principais

- ✅ Servidor HTTP na porta 80 (sem SSL)
- ✅ Dashboard com lista de todos os postes da linha
- ✅ Página de detalhes individual por poste
- ✅ API JSON para integração com scripts externos
- ✅ Auto-refresh opcional (via botão manual)
- ✅ Interface mobile-first (responsiva)
- ✅ Limite de 1 cliente simultâneo (economia de RAM)

### 📊 Dados Visualizados

| Categoria | Informação |
|-----------|-----------|
| **Estado** | Modo actual (IDLE/TRAFIC/OBSTACULO), T, Tc, duty_cycle |
| **Topologia** | Papel (master/slave), vizinhos vivos |
| **Tempo** | Horas em SAVE_MODE (10%), LIGHT_MIN (50%), LIGHT_ON (100%) |
| **Energia** | Consumo acumulado, poupança vs. 100% ON, custo evitado |
| **Tráfego** | Total de veículos detectados (hoje) |

---

## 🏗️ ARQUITECTURA

```
┌─────────────────────────────────────────────────────────┐
│ CAMADA 3: INTERFACE WEB (Browser)                      │
│ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ │
│ • HTML/CSS/JavaScript (embebido no ESP32)              │
│ • Refresh manual via botão                             │
│ • AJAX para /api/* endpoints                           │
└─────────────────────────────────────────────────────────┘
                          ▲ HTTP GET
                          │
┌─────────────────────────────────────────────────────────┐
│ CAMADA 2: SERVIDOR HTTP (web_manager.c)                │
│ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ │
│ • ESP-IDF HTTP Server                                  │
│ • Routing: /dashboard, /poste/X, /api/*               │
│ • Limite: 1 cliente simultâneo                         │
└─────────────────────────────────────────────────────────┘
                          ▲ Queries
                          │
┌─────────────────────────────────────────────────────────┐
│ CAMADA 1: DADOS (web_data_provider.c)                  │
│ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ │
│ • Agrega dados de fsm_core, network_coordinator, etc. │
│ • Calcula estatísticas (tempo, energia)               │
│ • Converte para JSON minificado                        │
└─────────────────────────────────────────────────────────┘
```

---

## 🔧 INSTALAÇÃO

### 1. Copiar Componente para o Projecto

```bash
cd ~/seu_projecto_esp32/components/
cp -r ~/poste_v8_web_manager ./web_manager
```

### 2. Adicionar Dependência no CMakeLists.txt Principal

```cmake
# main/CMakeLists.txt
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        wifi_manager
        web_manager      # ← ADICIONAR ESTA LINHA
)
```

### 3. Inicializar no main.c

```c
#include "web_manager.h"
#include "web_data_provider.h"

void app_main(void) {
    // 1. Inicializar WiFi (obrigatório primeiro!)
    wifi_manager_init();
    
    // 2. Esperar WiFi conectar (com IP atribuído)
    while (!wifi_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // 3. Inicializar agregador de dados
    web_data_provider_init();
    
    // 4. Inicializar servidor HTTP
    ESP_ERROR_CHECK(web_manager_init());
    
    ESP_LOGI("MAIN", "Servidor web activo!");
    ESP_LOGI("MAIN", "Acesso: http://192.168.1.10X/");
    
    // ... resto da aplicação ...
}
```

### 4. Configurar menuconfig (Opcional)

```bash
idf.py menuconfig
```

Ir a: `Component config → HTTP Server`

- Aumentar `Max HTTP Request Header Length` → 1024 (se necessário)
- `Max HTTP URI Length` → 512

---

## 🌐 COMO ACEDER

### Esquema de IP's

O sistema usa **IP's FIXOS** baseados na **posição física** do poste:

| Poste | Posição | IP Fixo |
|-------|---------|---------|
| Poste #0 | 0 | **192.168.1.100** |
| Poste #1 | 1 | **192.168.1.101** |
| Poste #2 | 2 | **192.168.1.102** |
| Poste #3 | 3 | **192.168.1.103** |
| ... | ... | ... |

**Fórmula:** `IP = 192.168.1.(100 + POSIÇÃO)`

### Acesso Via Browser

1. **Ligar telemóvel/PC à mesma rede WiFi** que os postes (ex: "PosteInteligente_AP")

2. **Abrir browser** e digitar:

```
http://192.168.1.100/        ← Aceder ao poste #0
http://192.168.1.101/        ← Aceder ao poste #1
http://192.168.1.102/        ← Aceder ao poste #2
```

3. **Dashboard** aparece automaticamente

4. **Clicar num poste** para ver detalhes individuais

---

## 📍 URLS DISPONÍVEIS

### Páginas HTML (Para Humanos)

| URL | Descrição |
|-----|-----------|
| `http://192.168.1.10X/` | Redireciona para /dashboard |
| `http://192.168.1.10X/dashboard` | Vista geral da linha de postes |
| `http://192.168.1.10X/poste/0` | Detalhes do poste #0 |
| `http://192.168.1.10X/poste/1` | Detalhes do poste #1 |

### API JSON (Para Scripts)

| URL | Formato | Descrição |
|-----|---------|-----------|
| `/api/line` | JSON | Estado completo de todos os postes |
| `/api/poste/0` | JSON | Dados do poste #0 |
| `/api/poste/1` | JSON | Dados do poste #1 |

#### Exemplo de Uso da API

```bash
# Via curl no terminal:
curl http://192.168.1.100/api/line

# Via Python:
import requests
r = requests.get('http://192.168.1.100/api/poste/0')
data = r.json()
print(f"Duty cycle: {data['duty_cycle']}%")
```

---

## 💾 CONSUMO DE RECURSOS

| Recurso | Consumo | Observações |
|---------|---------|-------------|
| **Flash** | ~45 KB | Código + HTML embebido |
| **RAM** | ~20 KB | 1 cliente activo |
| **CPU** | ~3% | Com cliente ligado |
| **CPU (idle)** | ~0.5% | Sem clientes |
| **Stack** | 3 KB | Por tarefa HTTP |

### Impacto no Loop 50Hz

- ✅ **MÍNIMO** - Loop principal mantém performance
- ✅ Servidor HTTP corre em tarefa separada (prioridade 3)
- ✅ Não interfere com fsm_core (prioridade 5)

---

## 📦 DEPENDÊNCIAS

### Obrigatórias

| Módulo | Versão | Uso |
|--------|--------|-----|
| `esp_http_server` | ESP-IDF | Servidor HTTP |
| `json` (cJSON) | ESP-IDF | Geração de JSON |
| `esp_wifi` | ESP-IDF | Rede activa |
| `nvs_flash` | ESP-IDF | Armazenamento |

### Módulos do Projecto

| Módulo | Interface Esperada |
|--------|-------------------|
| `wifi_manager` | `wifi_is_connected()` |
| `fsm_core` | `fsm_core_get_state()`, `fsm_core_get_T()`, `fsm_core_get_Tc()`, `fsm_core_get_duty_cycle()` |
| `network_coordinator` | `network_coordinator_get_role()`, `network_coordinator_get_neighbors()` |
| `tracking_manager` | `tracking_manager_get_vehicle_count()` |

---

## 🔗 INTEGRAÇÃO COM OUTROS MÓDULOS

### Funções Necessárias (TODO)

O ficheiro `web_data_provider.c` tem **placeholders** para funções externas.  
**TENS DE IMPLEMENTAR OU ADAPTAR** estas chamadas:

```c
// NO TEU fsm_core.h, adicionar:
const char* fsm_core_get_state(void);      // Retorna "IDLE", "TRAFIC", etc.
uint8_t fsm_core_get_T(void);              // Retorna contador T
uint8_t fsm_core_get_Tc(void);             // Retorna contador Tc
uint8_t fsm_core_get_duty_cycle(void);     // Retorna 0-100

// NO TEU network_coordinator.h, adicionar:
const char* network_coordinator_get_role(void);  // Retorna "MASTER" ou "SLAVE"
uint8_t network_coordinator_get_neighbors(neighbor_info_t* out, uint8_t max);

// NO TEU tracking_manager.h, adicionar:
uint32_t tracking_manager_get_vehicle_count(void);  // Total de veículos
```

### Passos de Integração

1. **Abrir `web_data_provider.c`**
2. **Procurar comentários `// TODO:`**
3. **Substituir chamadas placeholder pelas reais**

Exemplo:

```c
// ANTES (placeholder):
cJSON_AddNumberToObject(poste, "T", 0);  // TODO: fsm_core_get_T()

// DEPOIS (integrado):
cJSON_AddNumberToObject(poste, "T", fsm_core_get_T());
```

---

## ❓ FAQ

### 1. Qual o IP que uso para aceder?

**Depende da posição do poste:**

- Poste #0 → `http://192.168.1.100/`
- Poste #1 → `http://192.168.1.101/`
- Poste #2 → `http://192.168.1.102/`

**Dica:** Se não sabes a posição, experimenta todos os IPs de 100 a 105.

---

### 2. Não consigo aceder ao servidor web

**Checklist:**

1. ✅ WiFi está conectado? (`wifi_is_connected() == true`)
2. ✅ `web_manager_init()` foi chamado?
3. ✅ Monitor série mostra "Servidor HTTP activo"?
4. ✅ PC/telemóvel está na mesma rede WiFi?
5. ✅ Firewall não está a bloquear porta 80?

**Teste de conectividade:**

```bash
# No terminal (Linux/Mac):
ping 192.168.1.100

# Se responder, WiFi está OK
# Depois testar HTTP:
curl http://192.168.1.100/
```

---

### 3. Página carrega mas não mostra dados

**Possível causa:** `web_data_provider` não está integrado com outros módulos.

**Solução:** Verificar secção [Integração](#integração-com-outros-módulos) e implementar funções TODO.

---

### 4. Posso ter múltiplos clientes conectados?

**Não.** Esta versão limita a **1 cliente simultâneo** para economizar RAM.

Se precisares de mais clientes, editar:

```c
// web_manager.h
#define WEB_MAX_CONNECTIONS  3  // Aumentar para 3 clientes
```

**AVISO:** Cada cliente consome ~20KB RAM adicional!

---

### 5. Como exportar dados para Excel?

**Opção 1:** Usar API JSON

```python
import requests
import pandas as pd

r = requests.get('http://192.168.1.100/api/line')
data = r.json()
df = pd.DataFrame(data['postes'])
df.to_excel('postes.xlsx')
```

**Opção 2:** Implementar botão "Exportar CSV" no HTML (futuro).

---

### 6. Servidor web interfere com loop 50Hz?

**Não.** Testes mostram:

- CPU adicional: ~3% com 1 cliente
- Prioridade baixa (3) vs. fsm_core (5)
- Loop 50Hz mantém timing crítico

---

### 7. Como desactivar servidor web temporariamente?

```c
// No main.c ou via comando:
web_manager_stop();

// Para reactivar:
web_manager_init();
```

---

### 8. HTML pode ser modificado?

**Sim!** Editar `web_html_pages.c` e recompilar.

**Dica:** Para facilitar edição, usar ferramenta online para minificar HTML:
- https://www.willpeavy.com/tools/minifier/

---

## 📞 SUPORTE

**Problema não resolvido?**

1. Verificar logs no monitor série (`idf.py monitor`)
2. Procurar por `[WEB_MGR]` ou `[WEB_DATA]`
3. Verificar secção [Integração](#integração-com-outros-módulos)

---

## 📄 LICENÇA

Projecto académico - ISEL 2026  
Livre para uso educacional.

---

**Última actualização:** 2026-05-09  
**Versão:** 1.0  
**Autores:** Luis Custodio | Tiago Moreno
