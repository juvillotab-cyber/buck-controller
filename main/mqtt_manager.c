// ============================================================
// mqtt_manager.c  —  Cliente MQTT con TLS para HiveMQ Cloud
// ============================================================
//
// ¿Por qué TLS es necesario para HiveMQ Cloud?
//   HiveMQ Cloud solo acepta conexiones cifradas en el
//   puerto 8883 (MQTT sobre TLS, también llamado MQTTS).
//   Sin TLS, el broker rechaza la conexión.
//
// ¿Cómo funciona TLS aquí?
//   Para cifrar la conexión, el ESP32 debe verificar el
//   certificado del servidor (el broker).  Esa verificación
//   requiere conocer qué Autoridad Certificadora (CA) firmó
//   el certificado.  HiveMQ Cloud usa Let's Encrypt como CA.
//
//   En lugar de embutir manualmente el certificado, usamos
//   esp_crt_bundle: un paquete de ~135 CAs de confianza que
//   ESP-IDF incluye por defecto (similar al de un navegador).
//   Esto cubre Let's Encrypt y casi cualquier CA comercial.
//
// Flujo de eventos MQTT:
//   MQTT_EVENT_CONNECTED    → suscribirse a topics
//   MQTT_EVENT_DATA         → llamar al callback con el payload
//   MQTT_EVENT_DISCONNECTED → el cliente reintenta automáticamente
//   MQTT_EVENT_ERROR        → loguear el error TLS/red
// ============================================================

#include "mqtt_manager.h"
#include "mqtt_client.h"          // API de esp-mqtt
#include "esp_crt_bundle.h"       // Bundle de certificados CA
#include "esp_log.h"
#include <string.h>
#include <stdbool.h>

static const char *TAG = "mqtt_manager";

// Estado interno del módulo
static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static mqtt_message_cb_t        s_message_callback = NULL;
static bool                     s_is_connected = false;

// ────────────────────────────────────────────────────────────
// Handler de eventos MQTT
//
// esp-mqtt llama a esta función en el contexto de su propia
// tarea interna cuando ocurre cualquier evento MQTT.
// ────────────────────────────────────────────────────────────
static void mqtt_event_handler(void *handler_args,
                                esp_event_base_t base,
                                int32_t event_id,
                                void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {

        // ── Conexión establecida con el broker ──────────────
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Conectado al broker MQTT");
            s_is_connected = true;

            // Suscribirse al topic de control RGB
            // QoS 1 = "al menos una entrega" (garantía moderada)
            int sub_id = esp_mqtt_client_subscribe(
                s_mqtt_client, MQTT_TOPIC_SUBSCRIBE, 1);
            if (sub_id < 0) {
                ESP_LOGE(TAG, "Error al suscribirse a '%s'", MQTT_TOPIC_SUBSCRIBE);
            } else {
                ESP_LOGI(TAG, "Suscrito a '%s' (msg_id=%d)",
                         MQTT_TOPIC_SUBSCRIBE, sub_id);
            }
            break;

        // ── Desconexión del broker ──────────────────────────
        // esp-mqtt intentará reconectarse automáticamente
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Desconectado del broker MQTT. Reconectando...");
            s_is_connected = false;
            break;

        // ── Suscripción confirmada por el broker ────────────
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "Suscripción confirmada (msg_id=%d)", event->msg_id);
            break;

        // ── Mensaje recibido en un topic suscrito ───────────
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "Mensaje recibido → Topic: %.*s  Payload: %.*s",
                     event->topic_len, event->topic,
                     event->data_len,  event->data);

            // Llamar al callback externo si fue registrado
            // Creamos strings null-terminated a partir de los buffers
            // porque event->topic y event->data NO tienen '\0' final.
            if (s_message_callback != NULL) {
                // Buffer temporal para el topic (máx 128 chars)
                char topic_buf[128] = {0};
                int  topic_len = (event->topic_len < 127) ? event->topic_len : 127;
                strncpy(topic_buf, event->topic, topic_len);

                s_message_callback(topic_buf, event->data, event->data_len);
            }
            break;

        // ── Error de conexión / TLS ─────────────────────────
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "Error MQTT");
            // Detalla el error TLS si está disponible
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "  ESP-TLS error: 0x%x",
                         event->error_handle->esp_tls_last_esp_err);
                ESP_LOGE(TAG, "  TLS stack error: 0x%x",
                         event->error_handle->esp_tls_stack_err);
            }
            break;

        default:
            // Eventos menos relevantes (PUBLISHED, BEFORE_CONNECT, etc.)
            ESP_LOGD(TAG, "Evento MQTT ignorado: %d", event_id);
            break;
    }
}

// ────────────────────────────────────────────────────────────
// mqtt_manager_set_callback()
// ────────────────────────────────────────────────────────────
void mqtt_manager_set_callback(mqtt_message_cb_t callback)
{
    s_message_callback = callback;
}

// ────────────────────────────────────────────────────────────
// mqtt_manager_init()
// ────────────────────────────────────────────────────────────
esp_err_t mqtt_manager_init(void)
{
    // Configuración del cliente MQTT
    // En ESP-IDF v5.x la estructura usa campos anidados.
    esp_mqtt_client_config_t mqtt_cfg = {

        // Configuración del broker (URL + TLS)
        .broker = {
            .address = {
                .uri = MQTT_BROKER_URI,
                // El prefijo "mqtts://" ya activa TLS automáticamente
            },
            .verification = {
                // Usa el bundle de CAs integrado en ESP-IDF.
                // Esto verifica que el certificado del broker
                // esté firmado por una CA conocida (ej: Let's Encrypt).
                .crt_bundle_attach = esp_crt_bundle_attach,
            },
        },

        // Credenciales de autenticación
        .credentials = {
            .client_id = MQTT_CLIENT_ID,
            .username  = MQTT_USERNAME,
            .authentication = {
                .password = MQTT_PASSWORD,
            },
        },

        // Configuración de sesión
        .session = {
            .keepalive        = 60,    // Ping cada 60s para mantener la sesión
            .disable_clean_session = false,  // Sesión limpia en cada conexión
        },

        // Configuración de red (reintentos)
        .network = {
            .reconnect_timeout_ms = 5000,   // Espera 5s entre reintentos
            .timeout_ms           = 10000,  // Timeout de conexión: 10s
        },
    };

    // Crear la instancia del cliente MQTT
    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Error al crear el cliente MQTT");
        return ESP_FAIL;
    }

    // Registrar el handler de eventos MQTT
    // ESP_EVENT_ANY_ID = manejar todos los eventos del cliente
    esp_err_t err = esp_mqtt_client_register_event(
        s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al registrar eventos MQTT: %s", esp_err_to_name(err));
        return err;
    }

    // Iniciar el cliente (comienza el proceso de conexión en background)
    err = esp_mqtt_client_start(s_mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al iniciar el cliente MQTT: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Cliente MQTT iniciado → conectando a %s", MQTT_BROKER_URI);
    return ESP_OK;
}

// ────────────────────────────────────────────────────────────
// mqtt_manager_publish()
// ────────────────────────────────────────────────────────────
int mqtt_manager_publish(const char *payload)
{
    if (!s_is_connected) {
        ESP_LOGW(TAG, "Publish cancelado: MQTT no está conectado");
        return -1;
    }

    // esp_mqtt_client_publish() envía el mensaje y retorna el msg_id.
    // Parámetros: cliente, topic, payload, longitud (0=autodetect),
    //             QoS, retain_flag
    int msg_id = esp_mqtt_client_publish(
        s_mqtt_client,
        MQTT_TOPIC_PUBLISH,
        payload,
        0,    // Longitud automática (strlen)
        0,    // QoS 0: menor latencia para telemetría en tiempo real
        0     // retain=0: el broker no guarda el último mensaje
    );

    if (msg_id >= 0) {
        ESP_LOGD(TAG, "Publicado en '%s': %s", MQTT_TOPIC_PUBLISH, payload);
    } else {
        ESP_LOGE(TAG, "Error al publicar en '%s'", MQTT_TOPIC_PUBLISH);
    }

    return msg_id;
}

// ────────────────────────────────────────────────────────────
// mqtt_manager_is_connected()
// ────────────────────────────────────────────────────────────
bool mqtt_manager_is_connected(void)
{
    return s_is_connected;
}
