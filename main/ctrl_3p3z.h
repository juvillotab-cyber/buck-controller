#pragma once

#include "esp_err.h"

// ─────────────────────────────────────────────────────────────────────────────
//  Coeficientes generados por Biricha WDS
//  Cópialos exactamente como los entrega el programa.
//
//  La ecuación implementada es:
//  u[n] = K × (B0·e[n] + B1·e[n-1] + B2·e[n-2] + B3·e[n-3] + A1·u[n-1] + A2·u[n-2] + A3·u[n-3])
// ─────────────────────────────────────────────────────────────────────────────
#define B0 (+0.083093164800f)
#define B1 (-0.041639647575f)
#define B2 (-0.078608305110f)
#define B3 (+0.046124507266f)
#define A1 (+0.555938118593f)
#define A2 (+0.394764142777f)
#define A3 (+0.049297738630f)
#define K (+3.488574917146f)
#define REF (1550)
#define DUTY_MIN (0)
#define DUTY_MAX (800)

extern volatile float vref_voltios;
// ─────────────────────────────────────────────────────────────────────────────
//  Estado interno del controlador
//  Toda la memoria del 3p3z vive aquí.
// ─────────────────────────────────────────────────────────────────────────────
typedef struct {
    float e1;   // error ciclo n-1
    float e2;   // error ciclo n-2
    float e3;   // error ciclo n-3
    float u1;   // duty ciclo n-1 (en ticks)
    float u2;   // duty ciclo n-2
    float u3;   // duty ciclo n-3
} ctrl_3p3z_t;

// ─────────────────────────────────────────────────────────────────────────────
//  API pública
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Inicializa la memoria del controlador a cero.
 *        Llama esto una vez antes de arrancar el lazo.
 */
void ctrl_3p3z_init(ctrl_3p3z_t *ctx);

/**
 * @brief Ejecuta un paso del controlador 3p3z.
 *
 * @param ctx        Estado interno (se actualiza en cada llamada)
 * @param adc_counts Lectura cruda del ADC (0 a 4095)
 * @return           Nuevo duty cycle en ticks (DUTY_MIN a DUTY_MAX)
 *
 * Llama esta función desde la tarea de control, una vez por ciclo PWM.
 * El valor retornado va directo a buck_pwm_set_duty_ticks().
 */
float ctrl_3p3z_update(ctrl_3p3z_t *ctx, int adc_counts);