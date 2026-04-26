/* ============================================================
   MÓDULO     : fsm_sim
   FICHEIRO   : fsm_sim.h — Simulador físico de veículos
   VERSÃO     : 1.0  |  2026-04-26
   PROJECTO   : Poste Inteligente v8
   AUTORES    : Luis Custódio | Tiago Moreno
   PLATAFORMA : ESP32 (ESP-IDF v5.x)

   RESPONSABILIDADE:
   ─────────────────
   Simulação física de veículos para testes em bancada
   sem hardware de radar físico (USE_RADAR == 0).

   Ciclo: AGUARDA → ENTRAR → EM_VIA → DETECTADO → SAIU → AGUARDA

   DEPENDÊNCIAS:
   ─────────────
   fsm_core.h, system_config.h
   Compilado APENAS quando USE_RADAR == 0.
============================================================ */

#ifndef FSM_SIM_H
#define FSM_SIM_H

#include <stdbool.h>
#include <stdint.h>

#if USE_RADAR == 0

/** Inicializa o mutex do simulador — chamar uma vez */
void fsm_sim_init(void);

/** Actualiza posição do carro simulado — chamar a cada 100ms */
void fsm_sim_update(void);

/** Notifica o simulador que um carro está a chegar via UDP */
void fsm_sim_notificar_chegada(float vel_kmh, int16_t x_mm);

/** Copia posição do carro simulado para o display */
bool fsm_sim_get_objeto(float *x_mm, float *y_mm);

#else
/* Stubs vazios para modo real — sem código gerado */
static inline void fsm_sim_init(void) {}
static inline void fsm_sim_update(void) {}
static inline void fsm_sim_notificar_chegada(float v, int16_t x) { (void)v; (void)x; }
static inline bool fsm_sim_get_objeto(float *x, float *y) { (void)x; (void)y; return false; }
#endif

/* Aliases de compatibilidade com o código existente */
#define sim_init_mutex()              fsm_sim_init()
#define sim_notificar_chegada(v,x)    fsm_sim_notificar_chegada(v,x)
#define sim_get_objeto(x,y)           fsm_sim_get_objeto(x,y)
#define _sim_update()                 fsm_sim_update()

#endif /* FSM_SIM_H */
