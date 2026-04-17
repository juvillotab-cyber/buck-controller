#pragma once

#include "esp_err.h"
#include "driver/mcpwm_prelude.h"

// ─────────────────────────────────────────────
//  Parámetros del buck síncrono
//  Ajusta estos valores a tu diseño de hardware
// ─────────────────────────────────────────────

// Pines GPIO donde están conectadas las compuertas de Q1 y Q2
#define BUCK_GPIO_HIGH_SIDE   6    // GPIO para Q1 (high-side)
#define BUCK_GPIO_LOW_SIDE    7    // GPIO para Q2 (low-side)

// Frecuencia de conmutación en Hz
// Valores típicos: 100 kHz para conversores pequeños de potencia
#define BUCK_FREQ_HZ          10000

// Resolución del timer en ticks.
// A 80 MHz (reloj interno), con resolución 1 MHz:
//   período = 1 000 000 / 100 000 = 10 ticks → resolución de duty = 10 pasos
// Con 10 MHz de resolución: período = 100 ticks → más pasos de duty cycle
#define BUCK_TIMER_RESOLUTION_HZ  10000000   // 10 MHz

// Dead time: tiempo en nanosegundos durante el cual AMBAS señales están en LOW
// Esto protege contra el shoot-through (cortocircuito momentáneo).
// Valor mínimo depende del gate driver y los MOSFETs usados.
// 200 ns es un valor conservador para empezar.
#define BUCK_DEAD_TIME_NS    3000

// ─────────────────────────────────────────────
//  Handle del módulo (estado interno opaco)
// ─────────────────────────────────────────────

// Agrupamos todos los handles de MCPWM en una sola estructura.
// El main sólo necesita pasar un puntero a esta estructura.
typedef struct {
    mcpwm_timer_handle_t    timer;
    mcpwm_oper_handle_t     operator;
    mcpwm_cmpr_handle_t     comparator;   // Un comparador controla el duty de ambos generadores
    mcpwm_gen_handle_t      gen_high;     // Generador para Q1 (high-side)
    mcpwm_gen_handle_t      gen_low;      // Generador para Q2 (low-side)
} buck_pwm_t;

// ─────────────────────────────────────────────
//  API pública
// ─────────────────────────────────────────────

/**
 * @brief  Inicializa el periférico MCPWM para un buck síncrono.
 *
 * Configura timer, operador, comparador y dos generadores con dead time.
 * Las señales PWM NO arrancan hasta llamar buck_pwm_start().
 *
 * @param  ctx   Puntero a una estructura buck_pwm_t (debe estar en memoria válida)
 * @return ESP_OK si todo salió bien, error de ESP-IDF en caso contrario
 */
esp_err_t buck_pwm_init(buck_pwm_t *ctx);

/**
 * @brief  Arranca las señales PWM (el timer comienza a contar).
 */
esp_err_t buck_pwm_start(buck_pwm_t *ctx);

/**
 * @brief  Detiene las señales PWM (ambas salidas quedan en LOW).
 */
esp_err_t buck_pwm_stop(buck_pwm_t *ctx);

/**
 * @brief  Actualiza el duty cycle en tiempo real.
 *
 * @param  ctx         Handle del módulo
 * @param  duty_pct    Duty cycle de 0.0 a 100.0 (porcentaje)
 *
 * Internamente convierte el porcentaje a ticks según la resolución configurada.
 * Puedes llamar esta función desde una tarea de control sin reinicializar nada.
 */
esp_err_t buck_pwm_set_duty(buck_pwm_t *ctx, float duty_pct);

esp_err_t buck_pwm_set_duty_ticks(buck_pwm_t *ctx, uint32_t ticks);