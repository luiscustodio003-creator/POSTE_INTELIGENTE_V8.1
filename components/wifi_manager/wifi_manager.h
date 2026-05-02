#include <inttypes.h>
/* ============================================================
   WIFI MANAGER — DECLARAÇÃO
   @file      wifi_manager.h
   @version   1.5  |  2026-04-07
   Projecto  : Poste Inteligente v8
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

   Gestão da ligação Wi-Fi em modo STA com DHCP.
   Reconexão automática em background. Notifica o
   display_manager quando o estado muda.

   Pré-requisito:
     esp_netif_init() e esp_event_loop_create_default()
     devem ser chamados ANTES de wifi_manager_init().
============================================================ */
#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>

/** Inicializa Wi-Fi STA e tenta ligar. */
void wifi_manager_init(void);

/** true se ligado e com IP obtido. */
bool wifi_manager_is_connected(void);

/** IP actual como string "x.x.x.x" ou "---" se desligado. */
const char *wifi_manager_get_ip(void);

/** Reinicia contador de tentativas (para forçar nova série). */
void wifi_manager_reset_retry(void);

/** Inicia em modo AP (MASTER) ou STA (IDLE) conforme posição */
void wifi_manager_init_auto(void);

/** Muda de STA para AP quando assume liderança */
void wifi_manager_assume_ap(void);

#endif /* WIFI_MANAGER_H */

