# Buck Controller — Convertidor DC-DC síncrono con ESP32-C6

Control digital para un convertidor buck síncrono usando el módulo MCPWM del ESP32-C6, con lazo de control **3p3z** (3 polos, 3 zeros) ejecutado a 10 kHz, telemetría y setpoint remoto vía MQTT/TLS.

---

## Arquitectura del sistema

```
                    ┌─────────────────────────────────────────────────┐
                    │                 ESP32-C6                        │
                    │                                                 │
  Vout ──┬──┤├─── GPIO 0 (ADC)                                        │
         │            │                                               │
         │     ┌──────▼──────┐    ┌──────────────┐    ┌────────────┐  │
         │     │  buck_adc   │    │  ctrl_3p3z   │    │  buck_pwm  │  │
         │     │  (ADC cont.)│───►│  (control)   │───►│  (MCPWM)   │──┤──► GPIO 6 (HS)
         │     └─────────────┘    └──────────────┘    └────────────┘  │──► GPIO 7 (LS)
         │                                                            │
         │     ┌──────────────────────────────────────────────────┐   |
         │     │              wifi_manager                        │   |
         │     │  WiFi Station → EventGroup → conexión síncrona   │   │
         │     └──────────────────────┬───────────────────────────┘   │
         │                            │                               │
         │     ┌──────────────────────▼───────────────────────────┐   │
         │     │               mqtt_manager                       │   │
         │     │  esp-mqtt + TLS (HiveMQ Cloud)                   │   │
         │     │  Subscribe: dcdc/control/setpoint                |   │
         │     │  Publish:   dcdc/salida/data                     │   │
         │     └──────────────────────────────────────────────────┘   │
         │                                                  │
         └──────────────────────────────────────────────────┘
                              │
                              ▼
                        HiveMQ Cloud
                     (Broker MQTT con TLS)
```

### Flujo de datos

1. **ADC** (`buck_adc.c`) — Lee el voltaje de salida del buck por GPIO 0, usando ADC continuo con calibración por eFuse (curva de 3er grado). Dispara una ISR cada muestra.

2. **Controlador** (`ctrl_3p3z.c`) — Tarea de máxima prioridad. Se despierta con la ISR del ADC, calcula el error (`vref - medicion`), ejecuta la ecuación en diferencias 3p3z y produce el nuevo duty en ticks.

3. **PWM** (`buck_pwm.c`) — Recibe los ticks de duty y actualiza el comparador del MCPWM. El hardware genera las señales complementarias en GPIO 6 (high-side) y GPIO 7 (low-side) con dead time configurable.

4. **WiFi** (`wifi_manager.c`) — Conexión WiFi en modo Station con reintentos. Bloquea el inicio del sistema hasta tener IP.

5. **MQTT** (`mqtt_manager.c`) — Cliente MQTT con TLS (certificate bundle de Let's Encrypt). Publica `{"vout":..., "duty":...}` cada 500 ms en `dcdc/salida/data` y recibe setpoint en `dcdc/control/setpoint`.

---

## Pines (conexiones al buck)

| Pin ESP32-C6 | Señal       | Conecta a                              |
|-------------|-------------|----------------------------------------|
| GPIO 0      | ADC_IN      | Divisor resistivo de Vout              |
| GPIO 6      | PWM_HIGH    | Compuerta MOSFET Q1 (high-side)        |
| GPIO 7      | PWM_LOW     | Compuerta MOSFET Q2 (low-side)         |

### Circuito de potencia (buck síncrono)

```
         Vin
          │
          │
     ┌────┤──── Q1 (high-side, canal N)
     │    │    GPIO 6 ──► Gate driver ──► Gate Q1
     │    │
     │    ├──── L ────┬──── Vout
     │    │           │
     │    │ Q2 (low-side, canal N)
     │    │ GPIO 7 ──► Gate driver ──► Gate Q2
     │    │           │
     │   GND         C │
     │               GND
     │
     └────────────────┴──── Divisor resistivo ──── GPIO 0 (ADC)
                              (R1 + R2)
```

### Divisor resistivo para ADC

El ADC mide con atenuación de 12 dB (rango 0–3.1 V). Usa un divisor que mapee el voltaje máximo de salida a ~3.1 V:

```
Vout_max = 3.1 V × BUCK_ADC_DIVISOR
```

`BUCK_ADC_DIVISOR = 4.27` → Vout_max ≈ 13.2 V.

Para cambiar el divisor, ajusta `BUCK_ADC_DIVISOR` en `buck_adc.h`.

---

## Configuración del proyecto

### Requisitos
- ESP-IDF v5.5.4+
- Target: esp32c6

### Compilar

```bash
cd buck-controller
. $HOME/.espressif/v5.5.4/esp-idf/export.sh
idf.py set-target esp32c6
idf.py build
```

### Configurar credenciales WiFi y MQTT

Edita los archivos de configuración:

| Archivo            | Macros                         | Descripción                          |
|--------------------|--------------------------------|--------------------------------------|
| `wifi_manager.h`   | `WIFI_SSID`, `WIFI_PASSWORD`   | Red WiFi                             |
| `mqtt_manager.h`   | `MQTT_BROKER_URI`              | URL del broker (mqtts://...)         |
| `mqtt_manager.h`   | `MQTT_USERNAME`, `MQTT_PASSWORD` | Credenciales MQTT                  |
| `mqtt_manager.h`   | `MQTT_CLIENT_ID`               | ID único del cliente                 |
| `mqtt_manager.h`   | `MQTT_TOPIC_PUBLISH`           | Topic para publicar telemetría       |
| `mqtt_manager.h`   | `MQTT_TOPIC_SUBSCRIBE`         | Topic para recibir setpoint          |

### Ajustar controlador

Los coeficientes del 3p3z se generan con **Biricha WDS** y se copian en `ctrl_3p3z.h`:

```c
#define B0 (+0.083093164800f)
#define B1 (-0.041639647575f)
...
#define K  (+3.488574917146f)
#define REF (1550)           // Setpoint por defecto en counts
#define DUTY_MIN (0)         // Duty mínimo en ticks
#define DUTY_MAX (800)       // Duty máximo en ticks
```

### Calibrar ADC

1. En `buck_adc.h`, pon `BUCK_CAL_2P_ENABLED = 0`.
2. Mide dos voltajes de salida reales con multímetro.
3. Lee los counts que reporta el ADC en cada punto.
4. Calcula los counts ideales y completa `BUCK_CAL_RAW_1/2` y `BUCK_CAL_REF_1/2`.
5. Pon `BUCK_CAL_2P_ENABLED = 1`.

---

## Estructura del código

```
buck-controller/
├── CMakeLists.txt              # Proyecto ESP-IDF
├── main/
│   ├── CMakeLists.txt          # Registro de componentes
│   ├── main.c                  # Punto de entrada (app_main)
│   ├── buck_pwm.c / .h        # MCPWM, dead time, duty
│   ├── buck_adc.c / .h        # ADC continuo, calibración
│   ├── ctrl_3p3z.c / .h       # Controlador 3p3z
│   ├── wifi_manager.c / .h    # Conexión WiFi
│   └── mqtt_manager.c / .h    # Cliente MQTT/TLS
├── dashboard2.html             # Dashboard web alternativo
├── dashboard3.html             # Dashboard web principal
└── sdkconfig                   # Generado por set-target (no trackear)
```

### Prioridad de tareas FreeRTOS

| Tarea           | Prioridad | Stack  | Función                     |
|-----------------|-----------|--------|-----------------------------|
| `ctrl_buck`     | 5         | 4096   | Lazo de control (10 kHz)    |
| WiFi (interna)  | 4         | —      | Gestión WiFi (ESP-IDF)      |
| MQTT (interna)  | 3         | —      | Cliente MQTT (esp-mqtt)     |
| `monitor`       | 1         | 2048   | Publicar telemetría (500 ms) |

---

## Tópicos MQTT

| Topic                     | Dirección   | Formato                              |
|---------------------------|-------------|--------------------------------------|
| `dcdc/control/setpoint`   | Subscribe   | `"12.5"` (voltaje deseado en V)     |
| `dcdc/salida/data`        | Publish     | `{"vout":12.3,"duty":65}`           |

El setpoint se limpia al rango `[VREF_MIN, VREF_MAX]` definido en `main.c` (por defecto 0.2–30.0 V).

---

## Troubleshooting

**Error: `app partition is too small`** → Ejecutar `idf.py menuconfig` → `Partition Table` → `Single factory app, large`.

**Error: `cannot open source file "esp_err.h"`** en VS Code → Recargar ventana (`Ctrl+Shift+P` → `Developer: Reload Window`). El `compile_commands.json` ya debe existir en `build/`.
