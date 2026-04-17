#pragma once

#include "esp_err.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// ─────────────────────────────────────────────
//  Configuración del ADC  (sin cambios)
// ─────────────────────────────────────────────
#define BUCK_ADC_CHANNEL     ADC_CHANNEL_0
#define BUCK_ADC_ATTEN       ADC_ATTEN_DB_12
#define BUCK_ADC_SAMPLE_HZ   10000
#define BUCK_ADC_VMAX        3.1f
#define BUCK_ADC_RESOLUTION  4095.0f
#define BUCK_ADC_COUNTS_MIN  0
#define BUCK_ADC_COUNTS_MAX  4095
#define BUCK_ADC_DIVISOR     4.27f

// ─────────────────────────────────────────────────────────────────────────────
//  Calibración de dos puntos — capa adicional encima de la eFuse
//
//  buck_adc_leer_counts() aplica dos correcciones en cascada:
//
//    1. Calibración por curva (eFuse del chip):
//       raw  →  adc_cali_raw_to_voltage()  →  mV corregido
//       →  counts_cal = mV / 1000 / VMAX * RESOLUTION
//       Esta corrección la hace el IDF automáticamente si el chip tiene
//       datos en la eFuse. Si no, se cae al fallback lineal.
//
//    2. Corrección de dos puntos (tu divisor resistivo):
//       Compensa el error que queda del divisor y tolerancias de resistencias.
//       Se activa con BUCK_CAL_2P_ENABLED = 1.
//
//  Cómo medir los cuatro valores:
//
//  1. Pon BUCK_CAL_2P_ENABLED en 0 (solo calibración eFuse activa).
//
//  2. Aplica dos tensiones conocidas en la salida del buck.
//     Mide cada una con el multímetro → V_real.
//
//  3. Lee lo que devuelve buck_adc_leer_counts() en cada punto → RAW_1, RAW_2.
//     (En este punto "raw" ya viene corregido por la eFuse, por eso se llaman
//     RAW_1/RAW_2 y no los counts absolutos del ADC.)
//
//  4. Calcula los counts ideales para ese voltaje real:
//       ref = (V_real / BUCK_ADC_DIVISOR) / BUCK_ADC_VMAX * 4095
//     → REF_1, REF_2.
//
//  5. Rellena los defines y pon BUCK_CAL_2P_ENABLED en 1.
// ─────────────────────────────────────────────────────────────────────────────

#define BUCK_CAL_2P_ENABLED  0      // 1 = activa corrección de dos puntos

#define BUCK_CAL_RAW_1       695    // leer_counts() (con eFuse, sin 2p) en punto bajo
#define BUCK_CAL_REF_1       951    // count ideal en punto bajo
#define BUCK_CAL_RAW_2       2034   // leer_counts() (con eFuse, sin 2p) en punto alto
#define BUCK_CAL_REF_2       2779   // count ideal en punto alto

// ─────────────────────────────────────────────
//  Tipo del callback de la ISR  (sin cambios)
// ─────────────────────────────────────────────
typedef bool (*buck_adc_cb_t)(adc_continuous_handle_t handle,
                               const adc_continuous_evt_data_t *edata,
                               void *user_data);

// ─────────────────────────────────────────────
//  API pública  (sin cambios)
// ─────────────────────────────────────────────
esp_err_t buck_adc_init(buck_adc_cb_t cb);
esp_err_t buck_adc_start(void);
float     buck_adc_leer_voltaje(void);  // sin cambios
int       buck_adc_leer_counts(void);   // calibración aplicada aquí
void      buck_adc_deinit(void);