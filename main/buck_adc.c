#include "buck_adc.h"
#include "esp_log.h"

static const char *TAG = "buck_adc";

#define BUCK_ADC_FRAME_SIZE (SOC_ADC_DIGI_DATA_BYTES_PER_CONV * 1)

// ─────────────────────────────────────────────────────────────────────────────
//  Estado estático del módulo
//  Se añaden cali_handle y cali_enabled para leer_counts().
//  leer_voltaje() no los usa.
// ─────────────────────────────────────────────────────────────────────────────
static adc_continuous_handle_t adc_handle     = NULL;
static uint8_t                 raw_buf[BUCK_ADC_FRAME_SIZE];
static volatile float          ultimo_voltaje = 0.0f;

static adc_cali_handle_t       cali_handle    = NULL;
static bool                    cali_enabled   = false;

// ─────────────────────────────────────────────────────────────────────────────
//  _init_calibration  (privada)
//
//  El ESP32-C6 usa el esquema CURVE_FITTING. El IDF lee de la eFuse los
//  coeficientes de un polinomio de 3er grado que corrige la no linealidad:
//
//      V_real ≈ a0 + a1·raw + a2·raw² + a3·raw³
//
//  Necesita saber canal, atenuación y bitwidth para elegir los coeficientes
//  correctos de la tabla de la eFuse.
//
//  Si la eFuse no tiene datos válidos devuelve ESP_ERR_NOT_SUPPORTED y
//  cali_enabled queda en false — leer_counts() usará fallback lineal.
// ─────────────────────────────────────────────────────────────────────────────
static void _init_calibration(void)
{
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = BUCK_ADC_CHANNEL,
        .atten    = BUCK_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };

    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cfg, &cali_handle);

    if (ret == ESP_OK) {
        cali_enabled = true;
        ESP_LOGI(TAG, "Calibración por curva (eFuse) activa");
    } else {
        cali_enabled = false;
        ESP_LOGW(TAG, "Calibración no disponible (%s), leer_counts usará fallback lineal",
                 esp_err_to_name(ret));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  buck_adc_init  — idéntica al original + llama _init_calibration al final
// ─────────────────────────────────────────────────────────────────────────────
esp_err_t buck_adc_init(buck_adc_cb_t cb)
{
    esp_err_t ret;

    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = BUCK_ADC_FRAME_SIZE * 4,
        .conv_frame_size    = BUCK_ADC_FRAME_SIZE,
    };
    ret = adc_continuous_new_handle(&handle_cfg, &adc_handle);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Handle: %s", esp_err_to_name(ret)); return ret; }

    adc_digi_pattern_config_t pattern = {
        .atten     = BUCK_ADC_ATTEN,
        .channel   = BUCK_ADC_CHANNEL,
        .unit      = ADC_UNIT_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };

    adc_continuous_config_t adc_cfg = {
        .sample_freq_hz = BUCK_ADC_SAMPLE_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .pattern_num    = 1,
        .adc_pattern    = &pattern,
    };
    ret = adc_continuous_config(adc_handle, &adc_cfg);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Config: %s", esp_err_to_name(ret)); return ret; }

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = cb,
        .on_pool_ovf  = NULL,
    };
    ret = adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Callbacks: %s", esp_err_to_name(ret)); return ret; }

    // NUEVO: inicializa calibración para leer_counts()
    _init_calibration();

    ESP_LOGI(TAG, "ADC inicializado. Canal=%d, Freq=%d Hz", BUCK_ADC_CHANNEL, BUCK_ADC_SAMPLE_HZ);
    return ESP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
//  buck_adc_start  — idéntica al original
// ─────────────────────────────────────────────────────────────────────────────
esp_err_t buck_adc_start(void)
{
    esp_err_t ret = adc_continuous_start(adc_handle);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Start: %s", esp_err_to_name(ret));
    return ret;
}

// ─────────────────────────────────────────────────────────────────────────────
//  buck_adc_leer_voltaje  — IDÉNTICA AL ORIGINAL, sin ningún cambio
// ─────────────────────────────────────────────────────────────────────────────
float buck_adc_leer_voltaje(void)
{
    uint32_t leidos = 0;

    esp_err_t ret = adc_continuous_read(adc_handle,
                                         raw_buf,
                                         BUCK_ADC_FRAME_SIZE,
                                         &leidos,
                                         0);

    if (ret != ESP_OK || leidos == 0) {
        return ultimo_voltaje;
    }

    adc_digi_output_data_t *dato = (adc_digi_output_data_t *)raw_buf;

    if (dato->type2.channel != BUCK_ADC_CHANNEL) {
        return ultimo_voltaje;
    }

    int raw = (int)dato->type2.data;

    if (raw < BUCK_ADC_COUNTS_MIN) raw = BUCK_ADC_COUNTS_MIN;
    if (raw > BUCK_ADC_COUNTS_MAX) raw = BUCK_ADC_COUNTS_MAX;

    float v_pin = ((float)raw / BUCK_ADC_RESOLUTION) * BUCK_ADC_VMAX;
    float vout  = v_pin * BUCK_ADC_DIVISOR;

    ultimo_voltaje = vout;
    return vout;
}

// ─────────────────────────────────────────────────────────────────────────────
//  buck_adc_leer_counts
//
//  La lectura y validación del canal son idénticas al original.
//  La corrección del raw tiene ahora dos capas:
//
//  CAPA 1 — Calibración por curva (eFuse):
//    Reemplaza el 1.372f hardcodeado. En lugar de escalar linealmente,
//    usa el polinomio del chip para obtener mV corregidos y luego los
//    convierte de vuelta a counts:
//
//      mV  = adc_cali_raw_to_voltage(raw)
//      counts_cal = (mV / 1000.0) / VMAX * RESOLUTION
//
//    Si la eFuse no tiene datos (cali_enabled = false), hace exactamente
//    lo mismo que el original sin el 1.372f — devuelve el raw sin escalar.
//
//  CAPA 2 — Corrección de dos puntos (opcional, BUCK_CAL_2P_ENABLED):
//    Corrige el error residual que queda después de la eFuse, causado por
//    las tolerancias de las resistencias del divisor. Aplica una recta
//    entre dos puntos medidos con multímetro:
//
//      corrected = (counts_cal - RAW_1) * (REF_2 - REF_1)
//                  / (RAW_2 - RAW_1) + REF_1
//
//    Mide RAW_1/RAW_2 con BUCK_CAL_2P_ENABLED=0 (solo eFuse activa),
//    luego activa con BUCK_CAL_2P_ENABLED=1.
// ─────────────────────────────────────────────────────────────────────────────
int buck_adc_leer_counts(void)
{
    uint32_t leidos = 0;

    esp_err_t ret = adc_continuous_read(adc_handle,
                                         raw_buf,
                                         BUCK_ADC_FRAME_SIZE,
                                         &leidos,
                                         0);

    if (ret != ESP_OK || leidos == 0) return -1;

    adc_digi_output_data_t *dato = (adc_digi_output_data_t *)raw_buf;

    if (dato->type2.channel != BUCK_ADC_CHANNEL) return -1;

    int raw = (int)dato->type2.data;

    // ── CAPA 1: calibración por curva (reemplaza el 1.372f) ──────────────────
    float counts_cal;
    if (cali_enabled) {
        // adc_cali_raw_to_voltage devuelve mV corregidos por el polinomio
        // de la eFuse. Lo reconvertimos a counts para mantener las unidades
        // que espera el controlador 3p3z.
        int mv = 0;
        adc_cali_raw_to_voltage(cali_handle, raw, &mv);
        counts_cal = ((float)mv / 1000.0f) / BUCK_ADC_VMAX * BUCK_ADC_RESOLUTION;
    } else {
        // Fallback: sin escalar, igual que el original sin el 1.372f
        counts_cal = (float)raw;
    }

    // ── CAPA 2: corrección de dos puntos (opcional) ───────────────────────────
#if BUCK_CAL_2P_ENABLED
    // Interpola linealmente entre los dos puntos medidos con multímetro.
    // RAW_1/RAW_2 son lo que devolvía leer_counts() con solo la eFuse activa.
    // REF_1/REF_2 son los counts ideales calculados del voltaje real medido.
    counts_cal = ((counts_cal - BUCK_CAL_RAW_1) *
                  (float)(BUCK_CAL_REF_2 - BUCK_CAL_REF_1) /
                  (float)(BUCK_CAL_RAW_2 - BUCK_CAL_RAW_1))
                 + BUCK_CAL_REF_1;
#endif
    // ─────────────────────────────────────────────────────────────────────────

    float aux = 1.04 * counts_cal;
    int corrected = (int)aux;

    if (corrected < BUCK_ADC_COUNTS_MIN) corrected = BUCK_ADC_COUNTS_MIN;
    if (corrected > BUCK_ADC_COUNTS_MAX) corrected = BUCK_ADC_COUNTS_MAX;

    return corrected;
}

// ─────────────────────────────────────────────────────────────────────────────
//  buck_adc_deinit  — libera ADC y calibración
// ─────────────────────────────────────────────────────────────────────────────
void buck_adc_deinit(void)
{
    if (cali_enabled && cali_handle) {
        adc_cali_delete_scheme_curve_fitting(cali_handle);
        cali_handle  = NULL;
        cali_enabled = false;
    }
    if (adc_handle) {
        adc_continuous_stop(adc_handle);
        adc_continuous_deinit(adc_handle);
        adc_handle = NULL;
    }
    ESP_LOGI(TAG, "ADC liberado");
}