#include <stdio.h>
#include <stdlib.h>                  
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "buck_pwm.h"
#include "buck_adc.h"
#include "ctrl_3p3z.h"
#include "wifi_manager.h"            
#include "mqtt_manager.h"            
#include "nvs_flash.h"             

static const char *TAG = "main";

static QueueHandle_t cola_publish = NULL;

typedef struct {
    float  voltaje;
    float  duty_pct;
    float  count;
} monitor_data_t;

// ─────────────────────────────────────────────────────────────────────────────
//  Variables compartidas
// ─────────────────────────────────────────────────────────────────────────────
static SemaphoreHandle_t sem_adc_listo = NULL;
static buck_pwm_t        buck          = {0};
static ctrl_3p3z_t       ctrl          = {0};

// ── Referencia de voltaje de salida ──────────────────────────────────────────
//
//  volatile: el compilador no cachea esta variable en un registro.
//  Necesario porque se escribe desde la tarea MQTT y se lee desde
//  tarea_control — contextos de ejecución distintos.
//
//  En ESP32 (Xtensa LX7, 32-bit) las escrituras a un float alineado
//  son atómicas a nivel de bus — no se necesita mutex para una sola
//  variable de 4 bytes. volatile es suficiente.
//
//  Valor inicial: setpoint de arranque. Actualizable en tiempo real por MQTT.
// ─────────────────────────────────────────────────────────────────────────────
volatile float vref_voltios = 5.0f;  // ← ajusta el default a tu diseño

// ─────────────────────────────────────────────────────────────────────────────
//  Callback MQTT — llamado por la tarea interna de esp-mqtt cuando llega
//  un mensaje en el topic suscrito "dcdc/control/setpoint".
//
//  Flujo:
//    HiveMQ publica "3.3"  →  atof()  →  clamp  →  vref_volts = 3.3f
//    En el próximo ciclo de 100 µs, tarea_control lee el nuevo valor.
//
//  Seguridad:
//    • atof() retorna 0.0 si el payload no es numérico — el clamp lo
//      descarta (0 V queda fuera del rango mínimo).
//    • el payload ya llega null-terminated desde mqtt_manager.
// ─────────────────────────────────────────────────────────────────────────────
static void on_mqtt_setpoint(const char *topic, const char *data, int data_len)
{
    if (strncmp(topic, MQTT_TOPIC_SUBSCRIBE, strlen(MQTT_TOPIC_SUBSCRIBE)) != 0) {
        return;  // mensaje de otro topic — ignorar
    }

    float nuevo_vref = atof(data);  // ej: "3.3" → 3.3f

    // ── Clamp de seguridad ────────────────────────────────────────────
    // Rechaza valores fuera del rango físico del convertidor.
    // ¡AJUSTA estos límites a los de tu buck real!
    const float VREF_MIN = 0.2f;
    const float VREF_MAX = 30.0f;

    if (nuevo_vref < VREF_MIN || nuevo_vref > VREF_MAX) {
        ESP_LOGW(TAG, "Setpoint rechazado: %.2f V (rango: %.1f-%.1f V)",
                 nuevo_vref, VREF_MIN, VREF_MAX);
        return;
    }

    vref_voltios = nuevo_vref;
    ESP_LOGI(TAG, "Nuevo setpoint: %.2f V", vref_voltios);
}

// ─────────────────────────────────────────────────────────────────────────────
//  ISR del ADC — SIN CAMBIOS
//  WiFi/MQTT corren en tareas FreeRTOS y NUNCA interrumpen esta ISR.
// ─────────────────────────────────────────────────────────────────────────────
static bool IRAM_ATTR adc_conversion_done_cb(adc_continuous_handle_t handle,
                                              const adc_continuous_evt_data_t *edata,
                                              void *user_data) {
    BaseType_t high_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(sem_adc_listo, &high_task_woken);
    return high_task_woken == pdTRUE;
}

// ─────────────────────────────────────────────────────────────────────────────
//  tarea_control — SIN CAMBIOS
//  Prioridad 5: siempre preempta a WiFi (4), MQTT (3) y monitor (1).
// ─────────────────────────────────────────────────────────────────────────────
static void tarea_control(void *pvParameters) {
    float duty = 0.0f;
    int counts = 0;

    ESP_LOGI(TAG, "Tarea de control iniciada, esperando primer ciclo PWM...");

    while (1) {
        xSemaphoreTake(sem_adc_listo, portMAX_DELAY);

        counts = buck_adc_leer_counts();
        if (counts < 0) continue;

        // Pasar la referencia actual al controlador antes de cada ciclo.
        // vref_volts puede haber cambiado por MQTT — se toma snapshot
        // local para que el valor sea consistente dentro del ciclo.
        //float ref = vref_volts;
        //ctrl_3p3z_set_ref(&ctrl, ref);   // actualiza la referencia interna
        duty = ctrl_3p3z_update(&ctrl, counts);

        monitor_data_t datos = {
            .voltaje  = ((float)counts / 4095.0f) * 3.1f * BUCK_ADC_DIVISOR,
            .count    = (float)counts,
            .duty_pct = duty * 0.1f,
        };
        xQueueOverwrite(cola_publish, &datos);

        buck_pwm_set_duty_ticks(&buck, (uint32_t)duty);
    }

    vTaskDelete(NULL);
}

// ─────────────────────────────────────────────────────────────────────────────
//  tarea_monitor — MODIFICADA MÍNIMAMENTE
//
//  Solo se añadieron 4 líneas: construcción del payload JSON y publish.
//  El resto es idéntico. La tarea sigue en prioridad 1.
//
//  Notas de seguridad:
//  • mqtt_manager_publish() encola internamente — no bloquea el CPU
//    ni crea sección crítica visible para el scheduler de FreeRTOS.
//  • Si MQTT no está conectado, la función retorna -1 sin hacer nada.
//  • La tarea sigue durmiendo 500 ms entre ciclos: el timing de control
//    no tiene ninguna relación con este código.
// ─────────────────────────────────────────────────────────────────────────────
static void tarea_monitor(void *pvParameters) {
    monitor_data_t datos = {0};
    uint32_t ciclo = 0;

    xQueueReset(cola_publish);
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));

        if (xQueueReceive(cola_publish, &datos, 0) == pdTRUE) {
            ciclo++;
            /* ESP_LOGI(TAG, "[%lu] Bits = %lub | Vout = %.1fV | duty = %lu%%",
                     ciclo,
                     (uint32_t)datos.count,
                     datos.voltaje,
                     (uint32_t)datos.duty_pct); */

            // ── AÑADIDO: publicar por MQTT cada 500 ms ─────────────
            // Formato JSON compatible con ThingsBoard y HiveMQ.
            // snprintf es seguro aquí porque esta tarea tiene stack propio.
            char payload[64];
            snprintf(payload, sizeof(payload),
                     "{\"vout\":%.2f,\"duty\":%lu}",
                     datos.voltaje,
                     (uint32_t)datos.duty_pct);
            mqtt_manager_publish(payload);
            // ──────────────────────────────────────────────────────
        }

        taskYIELD();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  app_main — MODIFICADA: WiFi y MQTT se inician ANTES de arrancar el PWM
//
//  Orden crítico:
//  1. Semáforo, controlador, PWM init, queue    ← sin cambios
//  2. Crear tareas                              ← sin cambios
//  3. *** wifi_manager_init()  ***              ← AÑADIDO (bloqueante ~2-5s)
//  4. *** mqtt_manager_init()  ***              ← AÑADIDO (no bloqueante)
//  5. ADC init + start, PWM start              ← sin cambios

void app_main(void) {

    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || 
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // La partición NVS está corrupta o es de otra versión
        // Borrarla y reinicializar es el manejo estándar de ESP-IDF
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    // 1. Semáforo binario
    sem_adc_listo = xSemaphoreCreateBinary();
    configASSERT(sem_adc_listo != NULL);

    // 2. Controlador
    ctrl_3p3z_init(&ctrl);

    // 3. PWM (configura hardware, NO arranca todavía)
    ESP_ERROR_CHECK(buck_pwm_init(&buck));

    // 4. Queue del los datos para publicar
    cola_publish = xQueueCreate(1, sizeof(monitor_data_t));
    configASSERT(cola_publish != NULL);

    // 5. Crear tarea de control (prioridad 5)
    BaseType_t result = xTaskCreate(
        tarea_control, "ctrl_buck", 4096, NULL, 5, NULL);
    configASSERT(result == pdPASS);

    // 6. Crear tarea monitor (prioridad 1)
    result = xTaskCreate(
        tarea_monitor, "monitor", 2048, NULL, 1, NULL);
    configASSERT(result == pdPASS);

    // ── AÑADIDO: conectar WiFi ────────────────────────────────────────
    // Bloqueante. El PWM todavía no está corriendo, así que no hay
    // lazo de control activo aquí — es el momento seguro para esperar.
    ESP_LOGI(TAG, "Conectando a WiFi...");
    ESP_ERROR_CHECK(wifi_manager_init());
    // ─────────────────────────────────────────────────────────────────

    // ── AÑADIDO: iniciar cliente MQTT ─────────────────────────────────
    // No bloqueante: arranca el cliente en background y retorna
    // inmediatamente. La conexión TLS ocurre en la tarea interna de
    // esp-mqtt. mqtt_manager_publish() revisa s_is_connected antes
    // de intentar publicar, así que no hay riesgo de publish prematuro.
    // ── Registrar callback de setpoint ANTES de init ─────────────────
    // mqtt_manager_set_callback() debe llamarse antes de _init() para
    // que el callback esté listo cuando llegue el primer mensaje.
    mqtt_manager_set_callback(on_mqtt_setpoint);

    ESP_LOGI(TAG, "Iniciando cliente MQTT...");
    ESP_ERROR_CHECK(mqtt_manager_init());
    // ─────────────────────────────────────────────────────────────────

    // 7. ADC init + start
    ESP_ERROR_CHECK(buck_adc_init(adc_conversion_done_cb));
    ESP_ERROR_CHECK(buck_adc_start());

    // 8. Estabilización y arranque del PWM
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(buck_pwm_start(&buck));

    ESP_LOGI(TAG, "Sistema iniciado. Lazo de control activo a 10 kHz.");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
