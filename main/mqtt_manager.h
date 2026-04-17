// ============================================================
// mqtt_manager.h  —  Interfaz pública del módulo MQTT
// ============================================================
// Abstrae toda la complejidad de la conexión TLS y el
// protocolo MQTT detrás de tres funciones simples:
//   • mqtt_manager_init()       → conectar al broker
//   • mqtt_manager_publish()    → publicar datos
//   • mqtt_manager_set_callback() → recibir mensajes
// ============================================================

#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include <stddef.h>

// ── Configuración del broker HiveMQ Cloud ──────────────────
// ¡CAMBIA ESTOS VALORES antes de compilar!
//
// La URL con prefijo "mqtts://" le indica a esp-mqtt que
// debe usar TLS (puerto 8883).  Sin TLS sería "mqtt://".
#define MQTT_BROKER_URI  "mqtts://725e0dc5cdcd455aa54bbfe8f84fe245.s1.eu.hivemq.cloud:8883"
#define MQTT_USERNAME    "juandavid"
#define MQTT_PASSWORD    "FXHf0612"
#define MQTT_CLIENT_ID   "esp32c6_client_01"

// ── Topics MQTT ─────────────────────────────────────────────
#define MQTT_TOPIC_PUBLISH    "dcdc/salida/data"
#define MQTT_TOPIC_SUBSCRIBE  "dcdc/control/setpoint"

// ── Tipo de función callback para mensajes recibidos ────────
// Cuando llegue un mensaje MQTT, el módulo llamará a la función
// registrada con este tipo.
//
// Parámetros que recibirá el callback:
//   topic    → string con el topic del mensaje (ej: "esp32/rgb/data")
//   data     → contenido del mensaje (NO es null-terminated por defecto)
//   data_len → longitud del contenido en bytes
typedef void (*mqtt_message_cb_t)(const char *topic,
                                   const char *data,
                                   int         data_len);

// ── Funciones públicas ──────────────────────────────────────

/**
 * @brief Registra la función que se llamará al recibir mensajes MQTT.
 *
 * Debe llamarse ANTES de mqtt_manager_init().
 *
 * @param callback Puntero a la función callback.
 */
void mqtt_manager_set_callback(mqtt_message_cb_t callback);

/**
 * @brief Inicializa y conecta el cliente MQTT al broker.
 *
 * Usa TLS con el bundle de certificados integrado en ESP-IDF,
 * que incluye los CA de Let's Encrypt y otras autoridades
 * comunes (HiveMQ Cloud usa Let's Encrypt).
 *
 * @return ESP_OK si el cliente arrancó correctamente.
 */
esp_err_t mqtt_manager_init(void);

/**
 * @brief Publica un mensaje en el topic de sensores.
 *
 * @param payload   String con el mensaje a publicar.
 * @return  msg_id >= 0 si se encoló correctamente, -1 si hubo error.
 */
int mqtt_manager_publish(const char *payload);

/**
 * @brief Consulta si el cliente está actualmente conectado al broker.
 */
bool mqtt_manager_is_connected(void);
