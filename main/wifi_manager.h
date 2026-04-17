// ============================================================
// wifi_manager.h  —  Interfaz pública del módulo WiFi
// ============================================================
// Expone solo lo que otros módulos necesitan.
// Los detalles internos (event group, reintentos, etc.)
// permanecen ocultos en wifi_manager.c.
// ============================================================

#pragma once

#include "esp_err.h"
#include <stdbool.h>

// ── Credenciales WiFi ───────────────────────────────────────
// ¡CAMBIA ESTOS VALORES antes de compilar!
#define WIFI_SSID      "LA_OCULTA"
#define WIFI_PASSWORD  "Makala123"

// Número máximo de reintentos antes de declarar error
#define WIFI_MAX_RETRIES  10

// ── Funciones públicas ──────────────────────────────────────

/**
 * @brief Inicializa el stack WiFi y conecta a la red configurada.
 *
 * Esta función es BLOQUEANTE: no retorna hasta que la
 * conexión sea exitosa o se alcance el límite de reintentos.
 *
 * También crea el bucle de eventos de ESP-IDF (esp_event_loop)
 * que es necesario para WiFi, MQTT y otros componentes.
 *
 * @return ESP_OK si se obtuvo IP, ESP_FAIL si falló.
 */
esp_err_t wifi_manager_init(void);

/**
 * @brief Consulta si actualmente hay conexión WiFi con IP asignada.
 *
 * @return true si hay conexión activa, false si no.
 */
bool wifi_manager_is_connected(void);
