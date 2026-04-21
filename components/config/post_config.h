#include <inttypes.h>
/* ============================================================
   POST CONFIG — DECLARAÇÃO v6.0
   ------------------------------------------------------------
   @file      post_config.h
   @brief     Gestão da configuração persistente do poste na NVS.
   @version   6.0
   @date      2026-04-03

   Projecto  : Poste Inteligente v6
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

   Descrição:
   ----------
   Gere a configuração persistente do poste na flash NVS.
   No primeiro arranque após flash, escreve os valores de
   POSTE_ID e POSTE_NAME de system_config.h na NVS.
   Em arranques subsequentes, os valores lidos da NVS
   sobrevivem a cortes de energia.

   Pré-requisito:
   --------------
   nvs_flash_init() deve ser chamado em app_main() ANTES
   de post_config_init(). Este módulo não inicializa a NVS.

   Dependências:
   -------------
   - system_config.h : POSTE_ID, POSTE_NAME (valores por omissão)
   - nvs_flash        : inicializada externamente
   - esp_log
============================================================ */

#ifndef POST_CONFIG_H
#define POST_CONFIG_H

#include <stdint.h>

/* Comprimento máximo do nome do poste (inclui '\0') */
#define POST_NAME_MAX_LEN   33

/* Estrutura de configuração do poste */
typedef struct {
    uint8_t id;
    char    name[POST_NAME_MAX_LEN];
} post_config_t;

/**
 * @brief Inicializa e persiste a configuração do poste na NVS.
 *        Usa valores de system_config.h como fonte de verdade.
 *        Pré-requisito: nvs_flash_init() já chamado.
 */
void        post_config_init(void);

/** @brief Retorna o ID actual do poste. */
uint8_t     post_get_id(void);

/** @brief Retorna o nome actual do poste (ponteiro estático). */
const char *post_get_name(void);

/** @brief Define novo ID e persiste na NVS. */
void        post_set_id(uint8_t id);

/** @brief Define novo nome e persiste na NVS. */
void        post_set_name(const char *name);

#endif /* POST_CONFIG_H */

