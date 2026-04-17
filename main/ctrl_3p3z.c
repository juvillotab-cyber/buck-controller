#include "ctrl_3p3z.h"
#include "buck_adc.h"

void ctrl_3p3z_init(ctrl_3p3z_t *ctx) {
    // Limpia toda la memoria antes de arrancar.
    // Si no se hace esto, los primeros ciclos calculan
    // con basura y puede haber un spike en la salida.
    ctx->e1 = 0.0f;
    ctx->e2 = 0.0f;
    ctx->e3 = 0.0f;
    ctx->u1 = 0.0f;
    ctx->u2 = 0.0f;
    ctx->u3 = 0.0f;
}

float ctrl_3p3z_update(ctrl_3p3z_t *ctx, int adc_counts) {

    // ── PASO 1: error en counts ───────────────────────────────────────
    // La referencia y la medición están en las mismas unidades (counts).
    // Si VOUT está por debajo de REF → error positivo → sube el duty.
    // Si VOUT está por encima de REF → error negativo → baja el duty.
    float vref_count = (4095*vref_voltios)/(3.1 * BUCK_ADC_DIVISOR);
    float e0 = (float)(vref_count - adc_counts);

    // ── PASO 2: ecuación en diferencias 3p3z ─────────────────────────
    // K multiplica toda la parte feedforward (los B).
    // Los A son la parte recursiva — realimentan la salida anterior.
    // Los signos negativos de A1, A2, A3 ya están en la ecuación
    // estándar — Biricha entrega los coeficientes con ese convenio.
    float u0 = K * (B0 * e0
                       + B1 * ctx->e1
                       + B2 * ctx->e2
                       + B3 * ctx->e3)
                       + A1 * ctx->u1
                       + A2 * ctx->u2
                       + A3 * ctx->u3;
    //printf("e0=%.1f u0_antes=%.1f u1=%.1f u2=%.1f u3=%.1f\n",e0, u0, ctx->u1, ctx->u2, ctx->u3);

    // ── PASO 3: saturación en ticks (anti-windup básico) ─────────────
    // Limita el duty al rango físico del timer MCPWM.
    // Cuando u0 está saturado, u1/u2/u3 se quedan en el límite,
    // lo que evita que el integrador acumule error excesivo.
    if (u0 > (float)DUTY_MAX) u0 = (float)DUTY_MAX;
    if (u0 < (float)DUTY_MIN) u0 = (float)DUTY_MIN;

    // ── PASO 4: desplazar la memoria ─────────────────────────────────
    // Lo que era n-2 pasa a ser n-3, n-1 pasa a n-2, y el valor
    // actual pasa a ser n-1 para el próximo ciclo.
    ctx->e3 = ctx->e2;
    ctx->e2 = ctx->e1;
    ctx->e1 = e0;

    ctx->u3 = ctx->u2;
    ctx->u2 = ctx->u1;
    ctx->u1 = u0;

    return u0;
}