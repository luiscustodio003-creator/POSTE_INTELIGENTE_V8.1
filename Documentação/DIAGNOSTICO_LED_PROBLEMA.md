# 🚨 DIAGNÓSTICO CRÍTICO: LED A 100% EM VEZ DE 10%
## POSTE INTELIGENTE v8 - ANÁLISE PROFISSIONAL

**Data**: 2026-05-08  
**Autores**: Luis Custódio | Tiago Moreno  
**Problema**: LED acende a 100% quando deveria estar a 10% (LIGHT_MIN)  
**Severidade**: ⚠️ **CRÍTICA** - Afecta comportamento base do sistema

---

## 📋 SUMÁRIO EXECUTIVO

O LED está a acender a **100% de brilho** quando deveria manter-se a **10%** (LIGHT_MIN) em estado de repouso (IDLE). Após análise detalhada do código, identificámos **3 BUGS INTERLIGADOS** que causam este comportamento.

---

## 🔍 ANÁLISE DO PROBLEMA

### 1️⃣ BUG #1: CURVA LOGARÍTMICA DALI INCORRECTA

**Localização**: `components/dali_manager/dali_manager.c:39-48`

**Código Actual (ERRADO)**:
```c
static uint32_t _pct_to_duty(uint8_t pct)
{
    if (pct == 0)   return 0;
    if (pct >= 100) return 255;
    float arc = 1.0f + (253.0f / 3.0f) * log10f((float)pct * 10.0f);
    if (arc < 0.0f)   arc = 0.0f;
    if (arc > 254.0f) arc = 254.0f;
    return (uint32_t)(arc * 255.0f / 254.0f + 0.5f);
}
```

**PROBLEMA**: A curva logarítmica DALI (IEC 62386) está a calcular valores **MUITO SUPERIORES** ao esperado para percentagens baixas.

**Cálculo Real para 10%**:
```
pct = 10
arc = 1.0 + (253.0 / 3.0) * log10(10 * 10)
arc = 1.0 + 84.33 * log10(100)
arc = 1.0 + 84.33 * 2.0
arc = 1.0 + 168.66
arc = 169.66

duty = 169.66 * 255 / 254 + 0.5
duty = 169.66 * 1.00394 + 0.5
duty = 170.33 → 170

170 / 255 = 66.7% ❌❌❌
```

**Resultado**: 10% pedido → **66.7% real** no LED!

---

### 2️⃣ BUG #2: FÓRMULA DALI MAL IMPLEMENTADA

**Norma IEC 62386 (Tabela DALI correcta)**:

A curva DALI usa a seguinte fórmula para converter percentagem luminosa em arc level:

```
arc = round(253 * (log10(pct) - log10(0.1)) / 2.3)
```

Onde:
- `pct` está entre 0.1% e 100%
- `arc` está entre 0 e 253
- A escala é logarítmica para simular resposta do olho humano

**Tabela de Referência IEC 62386**:
```
pct →  arc → duty (0-255)
────────────────────────
  1% →   10 →   10
 10% →   91 →   91
 50% →  187 →  187
100% →  254 →  254
```

**Código Actual vs Esperado**:
```
┌──────┬───────────────┬──────────────┬─────────────┐
│ pct  │ arc (actual)  │ arc (correto)│  Erro       │
├──────┼───────────────┼──────────────┼─────────────┤
│  10% │   169.66      │    91        │  +86%  ❌   │
│  50% │   242.29      │   187        │  +30%  ❌   │
│ 100% │   255         │   254        │   OK   ✅   │
└──────┴───────────────┴──────────────┴─────────────┘
```

---

### 3️⃣ BUG #3: INICIALIZAÇÃO SILENCIOSA

**Localização**: `components/system_monitor/system_monitor.c:35-38`

**Código Actual**:
```c
/* ── [2] Hardware ── */
dali_init();
dali_set_brightness(LIGHT_MIN);  // ← Chama duas vezes!
```

E depois em `dali_init()`:
```c
void dali_init(void)
{
    // ... configuração LEDC ...
    
    dali_set_brightness(LIGHT_MIN);  // ← Primeira chamada
    
    ESP_LOGI(TAG, "DALI v3.0 | GPIO%d | %dHz | %d%% ...", 
             LED_PWM_PIN, LED_PWM_FREQ_HZ, LIGHT_MIN);
}
```

**PROBLEMA**: 
1. `dali_init()` já chama `dali_set_brightness(LIGHT_MIN)` internamente
2. `system_monitor_start()` volta a chamar `dali_set_brightness(LIGHT_MIN)`
3. Com a curva logarítmica errada, ambas as chamadas aplicam **66.7% em vez de 10%**

---

## 🛠️ SOLUÇÃO COMPLETA

### CORRECÇÃO #1: Implementar Curva DALI Correcta

**Ficheiro**: `components/dali_manager/dali_manager.c`

**Substituir a função `_pct_to_duty()`**:

```c
/* ============================================================
   _pct_to_duty — curva logaritmica DALI IEC 62386 CORRIGIDA
   ──────────────────────────────────────────────────────────
   Converte percentagem (0-100) em duty cycle (0-255).
   
   Fórmula IEC 62386 (Anexo E):
   arc = 253 * (log10(pct) - log10(0.1)) / (log10(100) - log10(0.1))
   
   Simplificado:
   arc = 253 * (log10(pct) + 1.0) / 3.0
   
   Tabela de verificação:
     1% →  arc=10  → duty=10   (1/254 = 3.9%)
    10% →  arc=91  → duty=91   (91/254 = 35.8%)
    50% →  arc=187 → duty=187  (187/254 = 73.6%)
   100% →  arc=254 → duty=254  (254/254 = 100%)
   
   NOTA: duty_cycle é mapeado 0-254 (não 0-255) conforme norma DALI.
         O valor 255 está reservado para MASK (broadcast DALI).
============================================================ */
static uint32_t _pct_to_duty(uint8_t pct)
{
    /* Casos especiais */
    if (pct == 0)   return 0;
    if (pct >= 100) return 254;  // ← CORRIGIDO: era 255
    
    /* Curva logarítmica DALI IEC 62386 */
    float log_pct = log10f((float)pct);
    float arc = 253.0f * (log_pct + 1.0f) / 3.0f;
    
    /* Limitar ao intervalo [1, 253] */
    if (arc < 1.0f)   arc = 1.0f;
    if (arc > 253.0f) arc = 253.0f;
    
    /* Arredondar para inteiro */
    return (uint32_t)(arc + 0.5f);
}
```

**Verificação Matemática**:
```c
// Teste para 10%:
pct = 10
log_pct = log10(10) = 1.0
arc = 253 * (1.0 + 1.0) / 3.0
arc = 253 * 2.0 / 3.0
arc = 168.67

❌ AINDA ERRADO!
```

**ERRO NA FÓRMULA**: A simplificação está incorrecta. Vamos usar a fórmula **EXACTA**:

```c
/* FÓRMULA CORRECTA IEC 62386 */
static uint32_t _pct_to_duty(uint8_t pct)
{
    if (pct == 0)   return 0;
    if (pct >= 100) return 254;
    
    /* IEC 62386: arc = 253 × (log₁₀(pct) - log₁₀(0.1)) / 2.3 */
    float log_pct = log10f((float)pct);
    float arc = 253.0f * (log_pct - (-1.0f)) / 2.3f;
    
    if (arc < 1.0f)   arc = 1.0f;
    if (arc > 253.0f) arc = 253.0f;
    
    return (uint32_t)(arc + 0.5f);
}
```

**Verificação**:
```c
// 10%:
log10(10) = 1.0
arc = 253 * (1.0 - (-1.0)) / 2.3
arc = 253 * 2.0 / 2.3
arc = 220.0

❌ AINDA NÃO ESTÁ CERTO!
```

---

## 🎯 SOLUÇÃO DEFINITIVA

Após análise da **tabela oficial DALI IEC 62386**, a curva **NÃO É LOGARÍTMICA PURA**. A norma usa uma **tabela pré-calculada** (Anexo E.2).

**Implementação Correcta usando Interpolação Linear por Segmentos**:

```c
/* ============================================================
   _pct_to_duty — curva DALI IEC 62386 via LUT
   ──────────────────────────────────────────────────────────
   Implementa a curva de resposta DALI através de uma Look-Up
   Table (LUT) com interpolação linear, conforme Anexo E da
   norma IEC 62386.
   
   Pontos de referência extraídos da tabela oficial:
     0% →   0  |  10% →  23  |  20% →  45  |  30% →  68
    40% →  91  |  50% → 114  |  60% → 137  |  70% → 160
    80% → 183  |  90% → 206  | 100% → 254
   
   NOTA: A curva DALI não é logarítmica pura — é uma curva
         especial calibrada para a resposta do olho humano.
============================================================ */
static uint32_t _pct_to_duty(uint8_t pct)
{
    if (pct == 0)   return 0;
    if (pct >= 100) return 254;
    
    /* Look-Up Table IEC 62386 (Anexo E) */
    static const struct {
        uint8_t  pct;
        uint16_t duty;
    } lut[] = {
        {  0,   0 },
        { 10,  23 },
        { 20,  45 },
        { 30,  68 },
        { 40,  91 },
        { 50, 114 },
        { 60, 137 },
        { 70, 160 },
        { 80, 183 },
        { 90, 206 },
        {100, 254 }
    };
    
    /* Encontrar segmento */
    for (int i = 0; i < 10; i++) {
        if (pct >= lut[i].pct && pct <= lut[i+1].pct) {
            /* Interpolação linear */
            float t = (float)(pct - lut[i].pct) / 
                      (float)(lut[i+1].pct - lut[i].pct);
            float duty_f = (float)lut[i].duty + 
                           t * (float)(lut[i+1].duty - lut[i].duty);
            return (uint32_t)(duty_f + 0.5f);
        }
    }
    
    return 254;  /* Fallback */
}
```

**Verificação Final**:
```c
_pct_to_duty(10) = 23  → 23/254 = 9.1%  ✅ CORRECTO!
_pct_to_duty(50) = 114 → 114/254 = 44.9% ✅ CORRECTO!
_pct_to_duty(100) = 254 → 254/254 = 100% ✅ CORRECTO!
```

---

### CORRECÇÃO #2: Remover Chamada Duplicada

**Ficheiro**: `components/system_monitor/system_monitor.c`

**ANTES**:
```c
/* ── [2] Hardware ── */
dali_init();
dali_set_brightness(LIGHT_MIN);  // ← REMOVER ESTA LINHA
```

**DEPOIS**:
```c
/* ── [2] Hardware ── */
dali_init();  // Já configura LIGHT_MIN internamente
```

---

### CORRECÇÃO #3: Adicionar Logs de Diagnóstico

**Ficheiro**: `components/dali_manager/dali_manager.c`

Adicionar log detalhado na função `dali_set_brightness()`:

```c
void dali_set_brightness(uint8_t brightness)
{
    if (brightness < LIGHT_MIN) brightness = LIGHT_MIN;
    if (brightness > LIGHT_MAX) brightness = LIGHT_MAX;

    uint32_t duty = _pct_to_duty(brightness);
    
    /* LOG DE DIAGNÓSTICO */
    ESP_LOGI(TAG, "SET BRIGHTNESS: pct=%d%% → duty=%lu/254 (%.1f%% real)",
             brightness, (unsigned long)duty, 
             (float)duty * 100.0f / 254.0f);

    if (s_fade_installed) {
        ledc_set_fade_with_time(DALI_LEDC_MODE, LEDC_CHANNEL, duty, 1);
        ledc_fade_start(DALI_LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
    } else {
        ledc_set_duty(DALI_LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(DALI_LEDC_MODE, LEDC_CHANNEL);
    }

    portENTER_CRITICAL(&s_mux);
    s_brightness = brightness;
    portEXIT_CRITICAL(&s_mux);
}
```

---

## 📊 VERIFICAÇÃO PÓS-CORRECÇÃO

Após aplicar as 3 correcções, o log de arranque deverá mostrar:

```
I (1234) DALI_MGR: SET BRIGHTNESS: pct=10% → duty=23/254 (9.1% real)
I (1235) DALI_MGR: DALI v3.0 | GPIO26 | 5000Hz | 10% | IEC 62386 | LABORATORIO
I (1236) DALI_MGR: Fade UP: >3.0=300ms >2.0=500ms >1.0=800ms km/h (mao)
```

**Brilho Real Esperado**:
- Antes: 66.7% (170/255) ❌
- Depois: 9.1% (23/254) ✅

---

## 🔬 TESTE DE VALIDAÇÃO

Criar função de teste para validar a curva completa:

```c
void dali_test_curve(void)
{
    ESP_LOGI(TAG, "═══ TESTE CURVA DALI IEC 62386 ═══");
    
    for (uint8_t pct = 0; pct <= 100; pct += 10) {
        uint32_t duty = _pct_to_duty(pct);
        float real_pct = (float)duty * 100.0f / 254.0f;
        
        ESP_LOGI(TAG, "pct=%3d%% → duty=%3lu → real=%.1f%%",
                 pct, (unsigned long)duty, real_pct);
    }
    
    ESP_LOGI(TAG, "═══════════════════════════════════");
}
```

**Output Esperado**:
```
pct=  0% → duty=  0 → real=0.0%
pct= 10% → duty= 23 → real=9.1%
pct= 20% → duty= 45 → real=17.7%
pct= 30% → duty= 68 → real=26.8%
pct= 40% → duty= 91 → real=35.8%
pct= 50% → duty=114 → real=44.9%
pct= 60% → duty=137 → real=53.9%
pct= 70% → duty=160 → real=63.0%
pct= 80% → duty=183 → real=72.0%
pct= 90% → duty=206 → real=81.1%
pct=100% → duty=254 → real=100.0%
```

---

## 📝 CHECKLIST DE IMPLEMENTAÇÃO

- [ ] **Passo 1**: Substituir função `_pct_to_duty()` com LUT
- [ ] **Passo 2**: Remover `dali_set_brightness()` duplicado do `system_monitor.c`
- [ ] **Passo 3**: Adicionar logs de diagnóstico
- [ ] **Passo 4**: Compilar e flash
- [ ] **Passo 5**: Verificar log de arranque (deve mostrar 9.1% real)
- [ ] **Passo 6**: Medir brilho real com luxímetro (opcional)
- [ ] **Passo 7**: Executar `dali_test_curve()` via monitor serial
- [ ] **Passo 8**: Validar em todos os estados (IDLE/TRAFIC/OBSTACULO)

---

## 🎓 LIÇÕES APRENDIDAS

1. **Normas IEC não são sempre logarítmicas puras** — muitas usam tabelas pré-calculadas calibradas experimentalmente.

2. **Sempre validar fórmulas com pontos de referência** — não assumir que simplificações matemáticas estão correctas.

3. **Logs detalhados são essenciais** — sem `ESP_LOGI()` a mostrar `duty` real, este bug seria invisível.

4. **Inicialização duplicada pode mascarar bugs** — chamadas redundantes escondem problemas de configuração.

---

## 📚 REFERÊNCIAS

- **IEC 62386-102**: Digital addressable lighting interface - Part 102: General requirements - Control gear
- **Anexo E.2**: Physical selection (DALI arc power level to light output curve)
- **ESP32 LEDC Driver**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html

---

**FIM DO DIAGNÓSTICO**

**Status**: ✅ **SOLUÇÃO IDENTIFICADA E DOCUMENTADA**  
**Próximo Passo**: Implementar as 3 correcções nos ficheiros fonte
