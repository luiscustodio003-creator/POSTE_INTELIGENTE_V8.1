# CORRECÇÃO DO BUG DO LED A 100%
## POSTE INTELIGENTE v8 - MÓDULO DALI

**Data**: 2026-05-08  
**Autores**: Luis Custódio | Tiago Moreno  
**Versão**: 3.1 (CORRIGIDA)

---

## 📋 FICHEIROS FORNECIDOS

1. **DIAGNOSTICO_LED_PROBLEMA.md** - Análise completa do problema
2. **dali_manager_v3.1_CORRIGIDO.c** - Código corrigido
3. **README_CORRECAO.txt** - Este ficheiro (instruções)

---

## 🔧 INSTRUÇÕES DE INSTALAÇÃO

### PASSO 1: Backup do código actual

```bash
cd seu_projecto/components/dali_manager
cp dali_manager.c dali_manager.c.BACKUP_v3.0
```

### PASSO 2: Substituir ficheiro

```bash
# Copiar ficheiro corrigido
cp /caminho/para/dali_manager_v3.1_CORRIGIDO.c dali_manager.c
```

### PASSO 3: Corrigir system_monitor.c

Editar `components/system_monitor/system_monitor.c`:

**ANTES (linha ~35)**:
```c
/* ── [2] Hardware ── */
dali_init();
dali_set_brightness(LIGHT_MIN);  // ← APAGAR ESTA LINHA
```

**DEPOIS**:
```c
/* ── [2] Hardware ── */
dali_init();  // Já configura LIGHT_MIN internamente
```

### PASSO 4: Compilar

```bash
cd seu_projecto
idf.py build
```

### PASSO 5: Flash

```bash
idf.py flash monitor
```

---

## ✅ VALIDAÇÃO

Após flash, o log de arranque deve mostrar:

```
I (1234) DALI_MGR: SET: pct=10% → duty=23/254 → 9.1% real
I (1235) DALI_MGR: DALI v3.1 | GPIO26 | 5000Hz | 10% | IEC 62386 v3.1 | LABORATORIO
```

**ANTES**: `→ 66.7% real` ❌  
**DEPOIS**: `→ 9.1% real` ✅

---

## 🧪 TESTE OPCIONAL

Para validar a curva completa, adicionar ao `app_main()`:

```c
#include "dali_manager.h"

void app_main(void)
{
    // ... inicializações ...
    
    dali_test_curve();  // ← Imprimir tabela completa
    
    // ... resto do código ...
}
```

---

## 📊 O QUE FOI CORRIGIDO

### Bug #1: Curva Logarítmica Errada
- **Antes**: Fórmula genérica `arc = 1 + (253/3) * log10(pct*10)`
- **Depois**: Look-Up Table IEC 62386 Anexo E.2 com interpolação

### Bug #2: Duty Máximo Incorreto
- **Antes**: `return 255;`
- **Depois**: `return 254;` (conforme norma DALI)

### Bug #3: Chamada Duplicada
- **Antes**: `dali_init()` + `dali_set_brightness()` no monitor
- **Depois**: Só `dali_init()` (já configura internamente)

---

## 🔍 DEPENDÊNCIAS

Este módulo **NÃO ALTERA** as interfaces públicas:
- Todos os `.h` mantêm-se iguais
- Todos os módulos que chamam DALI continuam a funcionar
- **COMPATIBILIDADE TOTAL** com v3.0

Módulos afectados:
- ✅ `fsm_task.c` - Sem alterações necessárias
- ✅ `state_machine.c` - Sem alterações necessárias
- ✅ `system_monitor.c` - Só remover linha duplicada

---

## 📈 RESULTADOS ESPERADOS

| Estado        | Antes (v3.0) | Depois (v3.1) |
|---------------|--------------|---------------|
| IDLE          | 66.7%  ❌    | 9.1%  ✅      |
| LIGHT_ON      | 100%   ✅    | 100%  ✅      |
| OBSTACULO     | 100%   ✅    | 100%  ✅      |
| SAFE_MODE     | ~80%   ❌    | 50%   ✅      |

---

## ⚠️ NOTAS IMPORTANTES

1. **Não alterar outras funções** - Só `_pct_to_duty()` foi modificada
2. **Fade continua igual** - Tempos e transições mantêm-se
3. **Performance** - LUT é mais rápida que log10f()
4. **Precisão** - Interpolação linear garante curva suave

---

## 📚 REFERÊNCIAS TÉCNICAS

- **IEC 62386-102**: Digital addressable lighting interface
- **Anexo E.2**: DALI arc power level to light output curve
- **Tabela Oficial**: 11 pontos de 0% a 100%

---

## 🆘 TROUBLESHOOTING

### Problema: LED continua a 100%
**Solução**: Verificar se removeu `dali_set_brightness()` do `system_monitor.c`

### Problema: LED não acende
**Solução**: Verificar GPIO26 no `hw_config.h`

### Problema: Curva não suave
**Solução**: Executar `dali_test_curve()` e verificar interpolação

---

## 📞 SUPORTE

Em caso de dúvidas:
1. Ler `DIAGNOSTICO_LED_PROBLEMA.md` completo
2. Verificar logs com `idf.py monitor`
3. Executar `dali_test_curve()` para validação

---

**FIM DAS INSTRUÇÕES**

**Status**: ✅ Pronto para implementação  
**Tempo Estimado**: 10 minutos  
**Risco**: Baixo (só 1 função alterada)
