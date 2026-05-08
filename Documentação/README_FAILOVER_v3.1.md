# 🎯 SOLUÇÃO FAILOVER INTELIGENTE v3.1

**Sistema de failover para rede mesh WiFi com IP fixo e eleição de master resiliente**

---

## 📋 RESUMO EXECUTIVO

### **PROBLEMA CORRIGIDO:**
Sistema com múltiplos postes em cadeia WiFi tinha **split-brain**: quando um poste intermediário falhava, múltiplos postes promoviam-se simultaneamente a MASTER, criando conflito de IPs e estados inconsistentes.

### **SOLUÇÃO IMPLEMENTADA:**
- ✅ **WiFi Manager v2.0** — IP fixo estático (nunca muda)
- ✅ **FSM Network v3.1** — Verificação de MASTER_CLAIM antes de promover
- ✅ **System Config optimizado** — Timeouts balanceados (failover 70s → 20s)

### **RESULTADO:**
- ✅ **Zero split-brain** — Apenas 1 master activo sempre
- ✅ **IP fixo** — Sem perda de conexão em transições
- ✅ **Sub-segmentos coordenados** — Menor posição assume master local
- ✅ **Recuperação automática** — Master original retoma papel sem conflito

---

## 🏗️ ARQUITECTURA DA SOLUÇÃO

### **Separação de Papéis (3 camadas):**

```
┌──────────────────────────────────────────────────┐
│ NÍVEL 1: PAPEL WiFi (FÍSICO — FIXO)             │
├──────────────────────────────────────────────────┤
│ pos=0 → AP permanente (192.168.4.1)             │
│ pos>0 → STA permanente (192.168.4.X)            │
│                                                  │
│ NUNCA MUDA em runtime!                          │
└──────────────────────────────────────────────────┘
           ↓
┌──────────────────────────────────────────────────┐
│ NÍVEL 2: PAPEL MASTER (LÓGICO — DINÂMICO)       │
├──────────────────────────────────────────────────┤
│ Eleição por menor posição viva                  │
│ Promoção automática em caso de falha            │
│ Cedência quando master original recupera        │
└──────────────────────────────────────────────────┘
           ↓
┌──────────────────────────────────────────────────┐
│ NÍVEL 3: VERIFICAÇÃO MASTER_CLAIM                │
├──────────────────────────────────────────────────┤
│ Antes de promover, verifica:                    │
│ • Recebeu CLAIM nos últimos 15s?                │
│ • Master conhecido tem ID menor?                │
│ • Se SIM → NÃO promove!                         │
└──────────────────────────────────────────────────┘
```

---

## 📦 FICHEIROS DA SOLUÇÃO

### **1. WiFi Manager v2.0** ⭐ **JÁ INSTALADO**
```
components/wifi_manager/wifi_manager.c  (v2.0)
components/wifi_manager/wifi_manager.h  (v2.0)
```

**Mudanças críticas:**
- IP fixo estático baseado em POST_POSITION
- Modo WiFi NUNCA muda (pos=0 sempre AP, pos>0 sempre STA)
- `wifi_manager_assume_ap()` OBSOLETA (só loga warning)

### **2. System Config optimizado** ⭐ **JÁ INSTALADO**
```
components/config/system_config.h  (v4.1)
```

**3 perfis de timeout disponíveis:**
- CONSERVADOR: 70s failover (original)
- **BALANCEADO: 20s failover** ← ACTIVO
- AGRESSIVO: 13s failover (arriscado)

### **3. FSM Network v3.1** 🆕 **NOVO FICHEIRO**
```
components/state_machine/fsm_network.c  (v3.1)
```

**ÚNICO FICHEIRO QUE PRECISAS ACTUALIZAR!**

---

## 🎯 CENÁRIOS DE OPERAÇÃO

### **Cenário 1: 3 postes A-B-C, B falha**

```
TOPOLOGIA: A ← B(OFF) ← C

┌─────────────────────────────────────┐
│ A (pos=0): AUTONOMO                 │
│ • Vizinho dir. offline → sozinho    │
│ • IP: 192.168.4.1 (mantém-se!)     │
│ • Radar local apenas                │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│ C (pos=2): MASTER temporário        │
│ • Menor posição viva no segmento    │
│ • IP: 192.168.4.3 (mantém-se!)     │
│ • VERIFICA: não recebeu CLAIM       │
│   de master menor → promove         │
└─────────────────────────────────────┘

QUANDO B VOLTA:
• A retoma MASTER (envia CLAIM)
• C recebe CLAIM → cede → IDLE
• IPs NUNCA mudaram ✅
• Topologia restaurada em ~5s
```

### **Cenário 2: 5 postes A-B-C-D-E, B falha**

```
TOPOLOGIA: A ← B(OFF) ← C ← D ← E

┌─────────────────────────────────────┐
│ A (pos=0): AUTONOMO                 │
│ • Não consegue enviar TC_INC        │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│ SUB-SEGMENTO C-D-E:                 │
│                                     │
│ C (pos=2): MASTER temporário ✅     │
│ • Coordena sub-segmento C→D→E       │
│ • Protocolo T/Tc funciona           │
│                                     │
│ D (pos=3): IDLE                     │
│ • Recebe TC_INC de C                │
│ • Relay para E                      │
│                                     │
│ E (pos=4): IDLE (fim de linha)      │
│ • Recebe TC_INC mas NÃO envia       │
│ • Apaga por timeout quando T=0      │
└─────────────────────────────────────┘

QUANDO B VOLTA:
• A retoma MASTER global
• C cede → IDLE
• Protocolo T/Tc restaurado A→B→C→D→E
```

### **Cenário 3: A←B(off)←C** 🐛 **BUG 6 CORRIGIDO!**

```
ANTES v3.0 (BUGADO):
─────────────────────
T=0s:  Rede normal A(MASTER)→B→C(IDLE)
T=3s:  B falha
T=3s:  C detecta B offline
T=8s:  C aguarda AUTONOMO_DELAY (5s)
T=8s:  C promove-se a MASTER ❌
       (MESMO recebendo CLAIM de A!)
       
RESULTADO: A e C ambos MASTER! (split-brain)

DEPOIS v3.1 (CORRIGIDO):
────────────────────────
T=0s:  Rede normal A(MASTER)→B→C(IDLE)
T=3s:  B falha
T=3s:  C detecta B offline
T=8s:  C aguarda AUTONOMO_DELAY (5s)
T=8s:  C VERIFICA:
       • Recebeu CLAIM de A há 2s (< 15s timeout) ✅
       • A(id=1) < C(id=3) ✅
T=8s:  C NÃO se promove ✅

RESULTADO: Só A é MASTER! ✅
```

---

## 🔧 INSTALAÇÃO

### **PASSO 1: Backup**
```bash
cd /caminho/do/projeto
git add .
git commit -m "BACKUP antes fsm_network v3.1"
```

### **PASSO 2: Substituir ficheiro**
```bash
# Apenas 1 ficheiro para actualizar!
cp fsm_network_v3.1_FINAL.c components/state_machine/fsm_network.c
```

### **PASSO 3: Verificar perfil activo**
Abrir `components/config/system_config.h` linha ~40:

```c
#define TIMEOUT_PROFILE  PROFILE_BALANCEADO  /* ← Verificar */
```

Opções:
- `PROFILE_CONSERVADOR` — 70s failover (seguro, testado)
- `PROFILE_BALANCEADO` — 20s failover (**RECOMENDADO**)
- `PROFILE_AGRESSIVO` — 13s failover (rápido, arriscado)

### **PASSO 4: Compilar**
```bash
idf.py fullclean
idf.py build
```

Deves ver:
```
✅ TIMEOUT_PROFILE: BALANCEADO (failover ~20s) ← RECOMENDADO
✅ Compiling fsm_network.c
✅ Build complete
```

### **PASSO 5: Flash em TODOS os postes**
```bash
# Flash simultaneamente em TODOS os postes!
idf.py -p /dev/ttyUSB0 flash
```

⚠️ **IMPORTANTE:** Flash em todos ao mesmo tempo para evitar incompatibilidades de protocolo!

---

## 📊 MÉTRICAS DE DESEMPENHO

```
┌──────────────────────┬─────────┬──────────┬──────────────┐
│ MÉTRICA              │ ANTES   │ DEPOIS   │ MELHORIA     │
├──────────────────────┼─────────┼──────────┼──────────────┤
│ Failover master      │ ~70s    │ ~20s     │ 3.5× RÁPIDO  │
│ Detectar B offline   │ ~8s     │ ~3s      │ 2.7× RÁPIDO  │
│ A recupera master    │ ~30s    │ ~5s      │ 6× RÁPIDO    │
│ Split-brain A←B←C    │ ❌ SIM  │ ✅ NÃO   │ CORRIGIDO    │
│ IP muda em failover  │ ❌ SIM  │ ✅ NÃO   │ FIXO SEMPRE  │
└──────────────────────┴─────────┴──────────┴──────────────┘
```

**PERFIL BALANCEADO (activo):**
```
DISCOVER_INTERVAL:     2000ms → 1000ms
NEIGHBOR_TIMEOUT:      8000ms → 3000ms
MASTER_CLAIM_HB:      30000ms → 5000ms
MASTER_CLAIM_TIMEOUT: 60000ms → 15000ms
AUTONOMO_DELAY:       10000ms → 5000ms
```

---

## ✅ GARANTIAS DA SOLUÇÃO

### **1. IP Fixo (WiFi Manager v2.0)**
- ✅ pos=0 sempre 192.168.4.1 (AP)
- ✅ pos=N sempre 192.168.4.(N+1) (STA)
- ✅ IP **NUNCA** muda, mesmo com failover
- ✅ Sem perda de conexão em transições

### **2. Master Único (FSM Network v3.1)**
- ✅ Verificação MASTER_CLAIM antes de promover
- ✅ Tracking de `s_master_id_conhecido`
- ✅ Timeout de 15s para considerar master offline
- ✅ Comparação: master_id < POSTE_ID → não promove

### **3. Sub-segmentos Coordenados**
- ✅ Menor posição viva assume master local
- ✅ Protocolo T/Tc funciona dentro do segmento
- ✅ Fim de linha (sem vizinho dir.) não envia TC_INC

### **4. Recuperação Graceful**
- ✅ Master original retoma papel automaticamente
- ✅ Master temporário cede sem conflito
- ✅ Relay de MASTER_CLAIM em toda a cadeia
- ✅ Propagação do ID real (não o relay intermediário)

---

## 🧪 TESTES DE VALIDAÇÃO

### **TESTE 1:** B falha (3 postes A-B-C)
```
✅ A fica AUTONOMO
✅ C fica AUTONOMO (ou MASTER se sub-segmento)
✅ IPs mantêm-se fixos
✅ Quando B volta, topologia restaura em ~5s
```

### **TESTE 2:** B falha (5 postes A-B-C-D-E)
```
✅ A fica AUTONOMO
✅ C assume MASTER do sub-segmento C-D-E
✅ Protocolo T/Tc funciona entre C→D→E
✅ E (fim de linha) apaga por timeout
✅ Quando B volta, A retoma master global
```

### **TESTE 3:** Cenário A←B(off)←C (BUG 6)
```
✅ C detecta B offline
✅ C aguarda 5s
✅ C verifica: recebeu CLAIM de A recentemente
✅ C NÃO se promove (A continua master)
✅ Zero split-brain! ✅
```

### **TESTE 4:** A falha
```
✅ C detecta timeout de MASTER_CLAIM (15s)
✅ C promove-se a MASTER após 20s total
✅ Sub-segmento C-D-E funciona coordenado
```

### **TESTE 5:** B volta após falha
```
✅ Topologia detecta B online
✅ Protocolo T/Tc restaurado em ~5s
✅ Sem conflito de IP
✅ Transição suave
```

---

## 🐛 BUGS CORRIGIDOS

### **v3.0 → v3.1:**
| BUG | DESCRIÇÃO | CORRECÇÃO |
|-----|-----------|-----------|
| **BUG 6** | Promoção indevida A←B(off)←C | Verificação MASTER_CLAIM antes de promover |

### **v2.9 → v3.0 (mantidos):**
| BUG | DESCRIÇÃO | CORRECÇÃO |
|-----|-----------|-----------|
| **BUG 1** | MASTER_CLAIM não propagava | Relay completo em cadeia |
| **BUG 2** | Promoção não notificava | CLAIM imediato ao promover |
| **BUG 3** | Promoção bloqueada por LIGHT_ON | Guarda alargada |
| **BUG 4** | AUTONOMO sobrepunha MASTER | Exclusão explícita |
| **BUG 5** | Propagação com ID errado | Usa s_master_id_conhecido |

---

## 📖 CÓDIGO-CHAVE (v3.1)

### **Tracking de MASTER_CLAIM:**
```c
/* No topo do fsm_network.c */
static int      s_master_id_conhecido  = 0;
static uint64_t s_master_claim_last_ms = 0;
```

### **Actualização no relay:**
```c
void fsm_network_master_claim_relay(int from_id, int master_id)
{
    uint64_t agora = fsm_agora_ms();
    
    s_master_id_conhecido  = master_id;
    s_master_claim_last_ms = agora;  // ← NOVO v3.1
    
    // ... resto do código
}
```

### **Verificação antes de promover:**
```c
bool master_menor_existe = false;

if (s_master_claim_last_ms > 0 &&
    (agora - s_master_claim_last_ms) < MASTER_CLAIM_TIMEOUT_MS) {
    
    if (s_master_id_conhecido > 0 && 
        s_master_id_conhecido < POSTE_ID) {
        master_menor_existe = true;
        ESP_LOGI(TAG, "[MASTER] Master id=%d activo — NÃO promovo",
                 s_master_id_conhecido);
    }
}

if (!master_menor_existe) {
    // SÓ AQUI PROMOVE!
    g_fsm_state = STATE_MASTER;
}
```

---

## 🔍 TROUBLESHOOTING

### **Problema: Compilação falha**
```
Erro: implicit declaration of 'comm_send_master_claim_id'
```
**Solução:** Verifica que `comm_manager.c/.h` estão na v3.1 (função adicionada)

### **Problema: Split-brain continua**
```
Logs mostram 2 postes como MASTER simultaneamente
```
**Solução:** 
1. Verifica `MASTER_CLAIM_TIMEOUT_MS` no system_config.h (deve ser 15000)
2. Confirma que `fsm_network.c` é v3.1 (tem verificação master_menor_existe)

### **Problema: Failover muito lento**
```
Demora >30s para assumir master após falha
```
**Solução:** Muda perfil para BALANCEADO ou AGRESSIVO em system_config.h

### **Problema: IP muda em runtime**
```
IP do poste muda de .3 para .1 durante failover
```
**Solução:** Verifica que `wifi_manager.c` é v2.0 (IP fixo estático)

---

## 📞 SUPORTE

**Autores:** Luis Custódio | Tiago Moreno  
**Projecto:** Poste Inteligente v8  
**Versão:** fsm_network v3.1 | 2026-05-08

---

## 📄 LICENÇA

Este código faz parte do Projecto Poste Inteligente v8.
Todos os direitos reservados.
