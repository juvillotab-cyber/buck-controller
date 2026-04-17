// ============================================================
// wifi_manager.c  —  Gestión de la conexión WiFi
// ============================================================
//
// Arquitectura basada en eventos:
//
//   esp_wifi (driver)
//        │  genera eventos
//        ▼
//   esp_event_loop  ──► wifi_event_handler()  (esta función)
//        │
//        ▼
//   FreeRTOS EventGroup  ──► wifi_manager_init() desbloquea
//
// Flujo de conexión:
//   1. esp_wifi_start()  →  evento WIFI_EVENT_STA_START
//   2. esp_wifi_connect() → evento IP_EVENT_STA_GOT_IP (éxito)
//                         → evento WIFI_EVENT_STA_DISCONNECTED (fallo)
//   3. Si hay desconexión → reintento automático
//
// Un EventGroup (FreeRTOS) funciona como semáforo de bits:
//   BIT0 = "conexión exitosa"
//   BIT1 = "conexión fallida definitivamente"
// wifi_manager_init() queda bloqueado esperando uno de estos bits.
// ============================================================

#include "wifi_manager.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <string.h>

static const char *TAG = "wifi_manager";

// Bits del EventGroup
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

// Estado interno del módulo (static = privado a este archivo)
static EventGroupHandle_t s_wifi_event_group = NULL;
static int  s_retry_count = 0;
static bool s_is_connected = false;

// ────────────────────────────────────────────────────────────
// Handler de eventos WiFi e IP
//
// ESP-IDF llama a esta función automáticamente cuando ocurre
// un evento de red. El parámetro event_base distingue
// la "categoría" (WIFI_EVENT o IP_EVENT) y event_id es el
// evento específico dentro de esa categoría.
// ────────────────────────────────────────────────────────────
static void wifi_event_handler(void *arg,
                                esp_event_base_t event_base,
                                int32_t event_id,
                                void *event_data)
{
    // ── El driver WiFi terminó de inicializarse → intentar conexión
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi iniciado, conectando a '%s'...", WIFI_SSID);
        esp_wifi_connect();
    }

    // ── Se perdió la conexión WiFi → reconectar o declarar fallo
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_is_connected = false;

        if (s_retry_count < WIFI_MAX_RETRIES) {
            s_retry_count++;
            ESP_LOGW(TAG, "Desconectado. Reintento %d/%d...", s_retry_count, WIFI_MAX_RETRIES);
            vTaskDelay(pdMS_TO_TICKS(1000));  // Espera 1 s antes de reintentar
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "Se agotaron los reintentos WiFi.");
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    }

    // ── El servidor DHCP asignó una dirección IP → conexión exitosa
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Conectado! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_count = 0;
        s_is_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ────────────────────────────────────────────────────────────
// wifi_manager_init()
// ────────────────────────────────────────────────────────────
esp_err_t wifi_manager_init(void)
{
    // 1. Crea el EventGroup que usaremos para sincronización
    s_wifi_event_group = xEventGroupCreate();

    // 2. Inicializa la capa de red (TCP/IP stack de LwIP)
    //    esp_netif DEBE inicializarse antes que esp_wifi
    ESP_ERROR_CHECK(esp_netif_init());

    // 3. Crea el bucle de eventos por defecto de ESP-IDF.
    //    WiFi, MQTT y otros componentes publican sus eventos aquí.
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 4. Crea la interfaz de red WiFi en modo Station (cliente)
    //    esp_netif_create_default_wifi_sta() configura internamente
    //    la interfaz de red con DHCP habilitado.
    esp_netif_create_default_wifi_sta();

    // 5. Inicializa el driver WiFi con la configuración por defecto
    wifi_init_config_t driver_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&driver_cfg));

    // 6. Registra el handler para eventos WiFi (desconexión, inicio, etc.)
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,
        &wifi_event_handler, NULL, NULL));

    // 7. Registra el handler para el evento de IP asignada
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP,
        &wifi_event_handler, NULL, NULL));

    // 8. Configura las credenciales WiFi
    wifi_config_t wifi_config = {
        .sta = {
            // Las credenciales se copian al struct en tiempo de compilación
            .ssid     = WIFI_SSID,
            .password = WIFI_PASSWORD,
            // Requiere al menos WPA2 (más seguro que permitir WEP/WPA)
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            // Permite PMF (Protected Management Frames) si el AP lo soporta
            .pmf_cfg.capable  = true,
            .pmf_cfg.required = false,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // 9. Arranca el driver WiFi → disparará WIFI_EVENT_STA_START
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Esperando conexión WiFi...");

    // 10. Bloquea aquí hasta que el EventGroup reciba CONNECTED o FAIL
    //     portMAX_DELAY = espera indefinidamente
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,       // No limpia los bits al salir
        pdFALSE,       // Espera CUALQUIERA de los dos bits (no ambos)
        portMAX_DELAY
    );

    if (bits & WIFI_CONNECTED_BIT) {
        return ESP_OK;
    }

    ESP_LOGE(TAG, "No se pudo conectar a WiFi.");
    return ESP_FAIL;
}

// ────────────────────────────────────────────────────────────
// wifi_manager_is_connected()
// ────────────────────────────────────────────────────────────
bool wifi_manager_is_connected(void)
{
    return s_is_connected;
}
