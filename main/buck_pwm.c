#include "buck_pwm.h"
#include "esp_log.h"
#include "ctrl_3p3z.h"

static const char *TAG = "buck_pwm";

// ─────────────────────────────────────────────────────────────────────────────
//  Función auxiliar: calcula cuántos ticks corresponde a un duty cycle (%)
//
//  El comparador de MCPWM trabaja en "ticks", no en porcentaje.
//  Cuando el contador del timer llega al valor del comparador, cambia la salida.
//
//  Fórmula:
//    ticks_por_período = BUCK_TIMER_RESOLUTION_HZ / BUCK_FREQ_HZ
//    ticks_de_duty     = ticks_por_período * (duty_pct / 100.0)
//
//  Ejemplo con los valores por defecto:
//    ticks_por_período = 10 000 000 / 100 000 = 100 ticks
//    duty 50% → 50 ticks
// ─────────────────────────────────────────────────────────────────────────────
static uint32_t duty_pct_to_ticks(float duty_pct) {
    uint32_t period_ticks = BUCK_TIMER_RESOLUTION_HZ / BUCK_FREQ_HZ;
    return (uint32_t)(period_ticks * duty_pct / 100.0f);
}

// ─────────────────────────────────────────────────────────────────────────────
//  PASO 1: Inicializar el TIMER
//
//  El timer es el "corazón" del MCPWM. Define:
//  - A qué velocidad cuenta (resolution_hz)
//  - Cuántos ticks dura cada período (period_ticks = resolution / freq)
//  - En qué dirección cuenta (UP = de 0 a period, luego vuelve a 0)
// ─────────────────────────────────────────────────────────────────────────────
static esp_err_t init_timer(buck_pwm_t *ctx) {
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,                          // Hay dos grupos MCPWM (0 y 1). Usamos el 0.
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT, // Usa el reloj por defecto del sistema
        .resolution_hz = BUCK_TIMER_RESOLUTION_HZ,  // Resolución: 10 MHz → 1 tick = 100 ns
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,  // Cuenta de 0 hasta period_ticks, luego reset
        .period_ticks  = BUCK_TIMER_RESOLUTION_HZ / BUCK_FREQ_HZ, // Número de ticks por ciclo
    };

    esp_err_t ret = mcpwm_new_timer(&timer_cfg, &ctx->timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error creando timer: %s", esp_err_to_name(ret));
    }
    return ret;
}

// ─────────────────────────────────────────────────────────────────────────────
//  PASO 2: Inicializar el OPERADOR y conectarlo al timer
//
//  El operador es el intermediario: recibe los ticks del timer y los conecta
//  con los comparadores y generadores.
//  Un timer puede tener hasta 3 operadores conectados (útil para 3 fases).
//  Nosotros sólo necesitamos uno.
// ─────────────────────────────────────────────────────────────────────────────
static esp_err_t init_operator(buck_pwm_t *ctx) {
    mcpwm_operator_config_t oper_cfg = {
        .group_id = 0,   // Debe coincidir con el group_id del timer
    };

    esp_err_t ret = mcpwm_new_operator(&oper_cfg, &ctx->operator);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error creando operador: %s", esp_err_to_name(ret));
        return ret;
    }

    // Conectar el operador al timer que creamos antes.
    // A partir de aquí, el operador "sabe" cuánto dura un período.
    ret = mcpwm_operator_connect_timer(ctx->operator, ctx->timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error conectando operador al timer: %s", esp_err_to_name(ret));
    }
    return ret;
}

// ─────────────────────────────────────────────────────────────────────────────
//  PASO 3: Inicializar el COMPARADOR
//
//  El comparador mira el valor actual del contador del timer y lo compara
//  con un valor de referencia (el "threshold").
//
//  Cuando contador == threshold → dispara un evento → el generador cambia de estado
//
//  Aquí iniciamos con duty = 0% (threshold = 0 ticks).
//  Después podremos cambiarlo con buck_pwm_set_duty() sin reiniciar nada.
// ─────────────────────────────────────────────────────────────────────────────
static esp_err_t init_comparator(buck_pwm_t *ctx) {
    mcpwm_comparator_config_t cmpr_cfg = {
        .flags.update_cmp_on_tez = true, // Actualiza el threshold cuando el timer llega a 0
                                          // Esto evita glitches si cambias el duty a mitad de ciclo
    };

    esp_err_t ret = mcpwm_new_comparator(ctx->operator, &cmpr_cfg, &ctx->comparator);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error creando comparador: %s", esp_err_to_name(ret));
        return ret;
    }

    // Inicializar el duty en 0%
    ret = mcpwm_comparator_set_compare_value(ctx->comparator, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando comparador a 0: %s", esp_err_to_name(ret));
    }
    return ret;
}

// ─────────────────────────────────────────────────────────────────────────────
//  PASO 4: Inicializar los GENERADORES con dead time
//
//  Un generador convierte los eventos del comparador en niveles HIGH/LOW
//  en un pin físico del ESP32-C6.
//
//  La lógica de los generadores es declarativa: le dices
//  "cuando pase el evento X, pon el pin en Y".
//
//  Para el buck síncrono necesitamos dos generadores:
//
//  gen_high (Q1, high-side):
//    - Cuando timer llega a 0 (TEZERO) → HIGH
//    - Cuando comparador dispara (TCOMPA) → LOW
//    Resultado: pulso positivo de duración = duty_ticks
//
//  gen_low (Q2, low-side) con dead time:
//    - Es la señal complementaria de gen_high, pero con un retardo
//      en los flancos (dead time) para evitar shoot-through.
//    - El hardware de dead time maneja esto automáticamente.
// ─────────────────────────────────────────────────────────────────────────────
static esp_err_t init_generators(buck_pwm_t *ctx) {
    // --- Generador high-side (Q1) ---
    mcpwm_generator_config_t gen_cfg_high = {
        .gen_gpio_num = BUCK_GPIO_HIGH_SIDE,
    };
    esp_err_t ret = mcpwm_new_generator(ctx->operator, &gen_cfg_high, &ctx->gen_high);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error creando gen_high: %s", esp_err_to_name(ret));
        return ret;
    }

    // --- Generador low-side (Q2) ---
    mcpwm_generator_config_t gen_cfg_low = {
        .gen_gpio_num = BUCK_GPIO_LOW_SIDE,
    };
    ret = mcpwm_new_generator(ctx->operator, &gen_cfg_low, &ctx->gen_low);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error creando gen_low: %s", esp_err_to_name(ret));
        return ret;
    }

    // --- Acciones del generador high-side ---
    //
    // mcpwm_gen_timer_event_action_t define QUÉ hacer en CUÁNDO
    //   .direction = MCPWM_TIMER_DIRECTION_UP → sólo aplica cuando el timer cuenta hacia arriba
    //   .event     = MCPWM_TIMER_EVENT_EMPTY   → cuando el contador llega a 0 (Timer-EZ = Timer Empty)
    //   .action    = MCPWM_GEN_ACTION_HIGH      → poner el pin en HIGH
    //
    // MCPWM_GEN_TIMER_EVENT_ACTION_END() es un macro centinela que marca el fin de la lista.
    ret = mcpwm_generator_set_actions_on_timer_event(ctx->gen_high,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END());
    if (ret != ESP_OK) { ESP_LOGE(TAG, "gen_high timer action: %s", esp_err_to_name(ret)); return ret; }

    // Cuando el comparador dispara (contador == duty_ticks) → LOW
    ret = mcpwm_generator_set_actions_on_compare_event(ctx->gen_high,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       ctx->comparator,
                                       MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    if (ret != ESP_OK) { ESP_LOGE(TAG, "gen_high compare action: %s", esp_err_to_name(ret)); return ret; }

    // --- Acciones del generador low-side (misma lógica, luego invertimos con dead time) ---
    ret = mcpwm_generator_set_actions_on_timer_event(ctx->gen_low,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END());
    if (ret != ESP_OK) { ESP_LOGE(TAG, "gen_low timer action: %s", esp_err_to_name(ret)); return ret; }

    ret = mcpwm_generator_set_actions_on_compare_event(ctx->gen_low,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       ctx->comparator,
                                       MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    if (ret != ESP_OK) { ESP_LOGE(TAG, "gen_low compare action: %s", esp_err_to_name(ret)); return ret; }

    // ─────────────────────────────────────────────────────────────────────────
    //  DEAD TIME
    //
    //  El bloque de dead time se aplica DESPUÉS de que los generadores
    //  calculan sus salidas. Lo que hace:
    //
    //  gen_high → aplica un retardo en el flanco de BAJADA (rising edge del complemento)
    //  gen_low  → invierte gen_high y aplica un retardo en el flanco de SUBIDA
    //
    //  Resultado: entre el fin del pulso de Q1 y el inicio del pulso de Q2
    //  (y viceversa) existe un tiempo muerto = BUCK_DEAD_TIME_NS
    //
    //  El dead time se mide en ticks del timer.
    //  Ticks = (dead_time_ns / 1e9) * BUCK_TIMER_RESOLUTION_HZ
    //
    //  Ejemplo: 200 ns * 10 MHz = 2 ticks
    // ─────────────────────────────────────────────────────────────────────────
    uint32_t dead_time_ticks = (uint32_t)((float)BUCK_DEAD_TIME_NS / 1e9f * BUCK_TIMER_RESOLUTION_HZ);
    ESP_LOGI(TAG, "Dead time: %u ticks (%u ns)", dead_time_ticks, BUCK_DEAD_TIME_NS);

    mcpwm_dead_time_config_t dt_cfg_high = {
        .posedge_delay_ticks = dead_time_ticks, // Retardo en flanco de subida de gen_high
        .negedge_delay_ticks = 0,
    };
    ret = mcpwm_generator_set_dead_time(ctx->gen_high, ctx->gen_high, &dt_cfg_high);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Dead time gen_high: %s", esp_err_to_name(ret)); return ret; }

    mcpwm_dead_time_config_t dt_cfg_low = {
        .posedge_delay_ticks = 0,
        .negedge_delay_ticks = dead_time_ticks, // Retardo en flanco de bajada de gen_low
        .flags.invert_output = true,             // Invertir: gen_low es el complemento de gen_high
    };
    ret = mcpwm_generator_set_dead_time(ctx->gen_low, ctx->gen_low, &dt_cfg_low);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Dead time gen_low: %s", esp_err_to_name(ret)); return ret; }

    return ESP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Funciones públicas
// ─────────────────────────────────────────────────────────────────────────────

esp_err_t buck_pwm_init(buck_pwm_t *ctx) {
    esp_err_t ret;

    ret = init_timer(ctx);
    if (ret != ESP_OK) return ret;

    ret = init_operator(ctx);
    if (ret != ESP_OK) return ret;

    ret = init_comparator(ctx);
    if (ret != ESP_OK) return ret;

    ret = init_generators(ctx);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "Buck PWM inicializado. Freq=%d kHz, Res=%d MHz, DT=%d ns",
             BUCK_FREQ_HZ/1000, BUCK_TIMER_RESOLUTION_HZ/1000000, BUCK_DEAD_TIME_NS);
    return ESP_OK;
}

esp_err_t buck_pwm_start(buck_pwm_t *ctx) {
    // Habilitar el timer: empieza a contar → las señales aparecen en los GPIO
    esp_err_t ret = mcpwm_timer_enable(ctx->timer);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Error habilitando timer: %s", esp_err_to_name(ret)); return ret; }

    ret = mcpwm_timer_start_stop(ctx->timer, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Error arrancando timer: %s", esp_err_to_name(ret)); }

    ESP_LOGI(TAG, "PWM iniciado");
    return ret;
}

esp_err_t buck_pwm_stop(buck_pwm_t *ctx) {
    // Detener el timer: las salidas van a LOW según la acción configurada
    esp_err_t ret = mcpwm_timer_start_stop(ctx->timer, MCPWM_TIMER_STOP_EMPTY);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Error deteniendo timer: %s", esp_err_to_name(ret)); return ret; }

    ret = mcpwm_timer_disable(ctx->timer);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Error deshabilitando timer: %s", esp_err_to_name(ret)); }

    ESP_LOGI(TAG, "PWM detenido");
    return ret;
}

esp_err_t buck_pwm_set_duty(buck_pwm_t *ctx, float duty_pct) {
    // Validar rango
    if (duty_pct < 0.0f)   duty_pct = 0.0f;
    if (duty_pct > 100.0f) duty_pct = 100.0f;

    uint32_t ticks = duty_pct_to_ticks(duty_pct);

    // Esta llamada es segura en tiempo real: el driver actualiza el comparador
    // al final del ciclo actual (gracias a .flags.update_cmp_on_tez = true)
    esp_err_t ret = mcpwm_comparator_set_compare_value(ctx->comparator, ticks);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error actualizando duty: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t buck_pwm_set_duty_ticks(buck_pwm_t *ctx, uint32_t ticks) {
    if (ticks > DUTY_MAX) ticks = DUTY_MAX;
    return mcpwm_comparator_set_compare_value(ctx->comparator, ticks);
}
