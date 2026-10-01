# Firmware del CenfoBot en C — Vision Rover Challenge

Firmware **100 % en C** (ESP-IDF) para los dos CenfoBots. Toda la lógica de
decisión corre **dentro del rover**, como exige el reglamento: la computadora
de la organización solo publica telemetría, no decide nada.

Esta carpeta es independiente del resto del repositorio. Los ejemplos en
CircuitPython de `codigos/` siguen intactos y sirven de referencia.

---

## 1. Qué hace el rover

```
  Cámara + sistema oficial de visión (PC)
                │  TCP 2026, NDJSON, 20 Hz, unidireccional
                ▼
  ┌──────────────────────────────────────────────┐
  │  CenfoBot (ESP32 IdeaBoard)                  │
  │                                              │
  │  vision_client.c  →  world_t (último estado) │
  │                          │                   │
  │  esp_link.c  ◄──────────►│  reparto de cubos │  ESP-NOW
  │  (otro rover)            │                   │  ◄────────►
  │                          ▼                   │
  │  mission.c   máquina de estados de la ronda  │
  │                          │                   │
  │                          ▼                   │
  │  control.c   rumbo en lazo cerrado           │
  │                          │                   │
  │                          ▼                   │
  │  motors.c    PWM → puente H                  │
  └──────────────────────────────────────────────┘
```

Ciclo de una entrega:

1. **SELECT** — elige el cubo con menor coste (distancia de acercamiento +
   distancia de acarreo) que el otro rover no esté atendiendo.
2. **GO_STAGE** — va a un punto *detrás* del cubo, sobre la recta
   cubo → depósito. Si el camino pasaría por encima del cubo, lo rodea.
3. **ALIGN** — se orienta hacia el centro del depósito.
4. **PUSH** — avanza atravesando el cubo, corrigiendo el rumbo con la cámara.
5. **VERIFY** — comprueba que el cubo quedó completamente dentro y pasa al
   siguiente. Si no, reintenta.

---

## 2. Archivos

| Archivo | Qué resuelve |
|---|---|
| `src/config.h` | **Todo lo que hay que ajustar.** Red, IDs, MACs, calibración, ganancias. |
| `src/pins.h` | Mapa de pines, con la fuente de cada valor. |
| `src/world.h` | Modelo del mundo, calcado del contrato de visión v2. |
| `src/geom.h` | Geometría del sistema de coordenadas (col/row/theta). |
| `src/motors.c` | PWM por LEDC, banda muerta, ganancia de calibración. |
| `src/imu.c` | LSM6DS3TRC por I2C (0x6B), giroscopio Z sin sesgo. |
| `src/sensors.c` | Ultrasónico HC-SR04, 4 infrarrojos, botón BOOT. |
| `src/status_led.c` | NeoPixel como indicador de estado. |
| `src/vision_client.c` | Wi-Fi, socket TCP, parseo NDJSON, "último valor gana". |
| `src/esp_link.c` | ESP-NOW: reparto de cubos y prevención de colisiones. |
| `src/control.c` | Control de rumbo: cámara para lo absoluto, giroscopio para amortiguar. |
| `src/mission.c` | Máquina de estados de la ronda. |
| `src/main.c` | Arranque y pruebas de puesta a punto. |

---

## 3. Instalación del entorno

**Una sola vez, en la computadora de desarrollo.**

1. Instalar [Visual Studio Code](https://code.visualstudio.com/).
2. Instalar la extensión **PlatformIO IDE** (`platformio.platformio-ide`)
   desde el panel de extensiones.
3. Reiniciar VS Code. PlatformIO descargará la cadena de compilación de
   ESP32 y ESP-IDF la primera vez que se compile (tarda varios minutos).
4. Abrir **esta carpeta** (`firmware-c/`) como carpeta de trabajo.

> Si se prefiere la línea de comandos: `pip install platformio` y usar `pio`
> desde una terminal. Los comandos de este README funcionan igual.

**Importante:** al subir este firmware se borra CircuitPython de la IdeaBoard.
Para volver a CircuitPython hay que reflashear con el
[IdeaBoard Flasher](https://crcibernetica.github.io/ideaboard-terminal/),
como explica `programacion/README.md`.

---

## 4. Configuración antes de la primera compilación

Editar **`src/config.h`**:

```c
#define WIFI_SSID     "nombre_de_la_red"
#define WIFI_PASSWORD "clave_de_la_red"
#define VISION_HOST   "192.168.1.47"   // IP de la PC del sistema de visión
```

La IP de la PC de visión se obtiene en esa misma PC con `ipconfig` (Windows)
o `ip addr` (Linux). Los dos rovers y la PC tienen que estar en la misma red.

Las MAC de los rovers se rellenan en el paso 5.2.

---

## 5. Puesta a punto, paso a paso

Hay cinco pruebas. **Hacerlas en orden.** Cada una valida una capa y evita
depurar tres problemas a la vez.

Para cada prueba:

```powershell
pio run -e test1_motores -t upload -t monitor
```

(o desde VS Code: barra inferior de PlatformIO → elegir el entorno → Upload).

### 5.1 — Prueba 1: sentido de los motores

```powershell
pio run -e test1_motores -t upload -t monitor
```

**Poner el robot sobre un soporte, con las ruedas al aire.** Verificar que:

| Mensaje en pantalla | Lo que debe pasar |
|---|---|
| ADELANTE | las dos ruedas giran hacia adelante |
| ATRAS | las dos giran hacia atrás |
| SOLO MOTOR 1 | gira solo la rueda izquierda, hacia adelante |
| SOLO MOTOR 2 | gira solo la derecha, hacia adelante |
| GIRO EN EL SITIO | giran en sentidos opuestos |

**Si una rueda gira al revés:** lo más limpio es intercambiar los cables 1 y 2
(motor 1) o 3 y 4 (motor 2) en la bornera, como indica `conexiones/README.md`.
Alternativa por software: poner `MOTOR_LEFT_INVERT` o `MOTOR_RIGHT_INVERT` a 1
en `config.h`.

**Si no se mueve nada:** revisar el jumper entre **SELECT** y **Vin**. Sin él
los motores no reciben alimentación.

**Si zumban pero no giran:** subir `MOTOR_DEADBAND` (p. ej. de 0.18 a 0.25).

### 5.2 — Anotar las MAC

Con cualquier firmware que levante Wi-Fi (pruebas 4 o 5), el monitor imprime:

```
MAC propia: 24:6F:28:12:34:56  <-- usar como PEER_MAC en el otro rover
```

Anotar la de **cada** rover y rellenar en `config.h`:

```c
#define PEER_MAC_ROVER_10 {0x24, 0x6F, 0x28, 0x12, 0x34, 0x56}  // MAC del rover 10
#define PEER_MAC_ROVER_11 {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF}  // MAC del rover 11
```

Los dos rovers se compilan con el **mismo** `config.h`; cada entorno elige la
MAC contraria automáticamente.

### 5.3 — Prueba 2: línea recta (calibración de motores)

```powershell
pio run -e test2_recta -t upload -t monitor
```

**En el suelo, con 1 m libre por delante.** El robot avanza 2 s y reporta
cuánto se desvió:

```
desvio tras 2 s: 8.4 grados
si es positivo, BAJAR MOTOR_RIGHT_GAIN ~0.03
```

Ajustar `MOTOR_RIGHT_GAIN` en `config.h`, recompilar y repetir hasta que el
desvío quede **por debajo de 5 grados**. Normalmente bastan 2 o 3 iteraciones.

> Equivale a lo que hace `codigos/motor_calibration.py`, pero en C.

### 5.4 — Prueba 3: sensores

```powershell
pio run -e test3_sensores -t upload -t monitor
```

Verificar:

- **ultra**: acercar la mano → la distancia baja. `-1.0` significa sin eco.
  Requiere el jumper SELECT–Vin.
- **IR**: tapar cada sensor → su valor cambia.
- **gyroZ**: quieto ≈ 0; al girar el robot a mano, cambia de signo según el
  sentido. **Si siempre marca 0, la IMU no responde:** revisar el cable Qwiic
  y confirmar los pines I2C en `pins.h`.
- **boton**: pasa a 1 al pulsar BOOT.

### 5.5 — Prueba 4: telemetría de visión

**Arrancar primero el sistema de visión en la PC** (ver `vision-system/`).
Para probar sin cámara sirve el simulador:

```powershell
python vision-system/contrato/mock_publisher.py
```

Luego:

```powershell
pio run -e test4_vision -t upload -t monitor
```

Salida esperada:

```
seq=1043 phase=RUNNING grid=43x43 cubos=3 depositos=3 edad=38 ms
  yo (10): col=12.40 row=30.10 theta=271.5 age=0
  cubo red   col=20.10 row=15.30 entregado=0
```

| Problema | Causa habitual |
|---|---|
| `sin telemetria (conectado=0)` | IP incorrecta en `VISION_HOST`, firewall de Windows bloqueando el puerto 2026, o rover en otra red |
| Conecta pero `la camara NO ve el marcador 10` | marcador ArUco mal pegado, tapado o con el ID equivocado |
| `edad` sube de 500 ms | red saturada o publicador detenido |

**No pasar de aquí hasta que esta prueba funcione.** Sin telemetría fiable el
rover no puede navegar.

### 5.6 — Prueba 5: giro en lazo cerrado

```powershell
pio run -e test5_giro -t upload -t monitor
```

El robot debe girar en el sitio hasta mirar a 0°, 90°, 180° y 270°
(referidos al sistema de la cámara) y detenerse en cada uno.

**Si gira alejándose del objetivo y da vueltas sin parar:** el signo del giro
está invertido. Cambiar en `config.h`:

```c
#define TURN_CCW_SIGN (-1)
```

y repetir. Este es el ajuste que más fallos de navegación causa si queda mal.

**Si oscila alrededor del objetivo sin asentarse:** bajar `TURN_KP`
(p. ej. 0.009 → 0.006) o subir `TURN_TOLERANCE_DEG`.

---

## 6. Compilar y subir la ronda

Con las cinco pruebas en verde:

```powershell
# Rover con marcador ArUco 10
pio run -e rover10 -t upload

# Rover con marcador ArUco 11
pio run -e rover11 -t upload
```

`ROVER_ID` **tiene que coincidir con el número del marcador ArUco** pegado en
ese robot físico. Es el único dato que distingue a los dos rovers.

---

## 7. Operación durante una ronda

1. Encender los dos rovers **quietos** sobre la superficie. Durante los
   primeros 2 s calibran el giroscopio: no tocarlos.
2. El LED pasa por: rojo (parado) → amarillo (conectando / esperando) →
   verde (ejecutando).
3. El rover arranca solo cuando la telemetría reporta `phase = RUNNING`,
   o si se pulsa el botón **BOOT** (permitido por el reglamento, sección 9).
4. A partir de ahí no se toca nada.

Código de colores del LED:

| Color | Significado |
|---|---|
| Rojo | parado: sin telemetría o la cámara no lo ve |
| Amarillo | conectando o esperando el inicio |
| Verde | navegando hacia un cubo |
| Azul | alineando o empujando |
| Magenta | cediendo el paso al otro rover |

---

## 8. Decisiones de diseño (y por qué)

**El rumbo absoluto viene de la cámara, no del giroscopio.**
`turn_angle.py` integra la velocidad angular para estimar el giro. Funciona
para un giro aislado, pero el error se acumula: tras varias maniobras el robot
ya no sabe hacia dónde mira. Aquí `theta` sale del contrato de visión en cada
frame y no acumula deriva. El giroscopio solo aporta el término derivativo,
que necesita ancho de banda alto (20 Hz de cámara no bastan para amortiguar)
pero no exactitud absoluta.

**El control no bloquea.**
Las funciones de `control.c` se llaman una vez por ciclo y devuelven si ya
llegaron. Un bucle interno tipo `while (...) { mover; sleep; }` dejaría al
robot ciego a la telemetría mientras se mueve, y sin forma de abortar.

**El reparto de cubos no negocia.**
Los dos rovers ejecutan el mismo criterio sobre la misma información global,
así que llegan a la misma conclusión por separado. El desempate es el ID menor
— determinista, sin turnos ni confirmaciones que puedan perderse.

**Se ignoran las observaciones viejas.**
El contrato expone `age_ms` justamente porque un cubo puede quedar oculto tras
un rover. Planificar sobre una posición de hace dos segundos manda al robot a
un sitio donde el objeto ya no está.

**Un solo mensaje en memoria.**
El contrato avisa: "último valor gana". Procesar una cola de mensajes viejos
haría que el rover reaccione a un mundo que ya cambió.

---

## 9. Ajustes finos

Todo en `src/config.h`.

| Síntoma | Qué tocar |
|---|---|
| Da vueltas alrededor del cubo sin engancharlo | subir `STAGE_DISTANCE_CELLS` |
| Empuja el cubo de lado y lo pierde | bajar `PUSH_SPEED`, bajar `ARRIVE_TOLERANCE_CELLS` |
| Zigzaguea al avanzar | bajar `HEADING_KP`, subir `HEADING_KD` |
| Tarda mucho en arrancar el giro | subir `TURN_MIN_SPEED` |
| Se pasa de largo en los giros | bajar `TURN_MAX_SPEED` o subir `HEADING_KD` |
| Los dos rovers se estorban | subir `PEER_AVOID_CELLS` |
| Se para todo el tiempo por el ultrasónico | bajar `ULTRA_EMERGENCY_CM` |
| Abandona tareas antes de terminarlas | subir `TASK_TIMEOUT_MS` |

---

## 10. Pendientes de verificar en hardware

Estos valores no están escritos en el repositorio; se dedujeron de la
plataforma IdeaBoard y **hay que confirmarlos con las pruebas**:

- `PIN_NEOPIXEL = 2` — si el LED no enciende, probar otro GPIO. No es crítico.
- `PIN_I2C_SDA = 21`, `PIN_I2C_SCL = 22` — si la prueba 3 muestra
  `gyroZ` siempre en 0, son estos pines.
- `MOTOR_DEADBAND`, `MOTOR_RIGHT_GAIN`, `TURN_CCW_SIGN` — se calibran en las
  pruebas 1, 2 y 5. Los valores por defecto son solo un punto de partida.

---

## 11. Robustez — Planner y fallback para D=1 máxima

La organización genera escenarios con **dificultad D ∈ [0,1]**:
- D=0: cubos cercanos, sin interferencias
- D=1: cubos lejanos (~42 celdas), hasta 3 trayectorias que se cruzan

**Tu planner es óptimo y no falla,** pero hemos agregado herramientas para
validar y monitorear:

### 11.1 — Algoritmo fallback greedy

Si algo impide que el planner enumerativo funcione (ej: pérdida de rovers por
telemetría stale), un algoritmo greedy simple asigna cubos por cercanía:

```c
#include "planner_fallback.h"

cube_color_t my_cube, peer_cube;
if (!planner_solve(...)) {
    planner_greedy_assign(&world, ROVER_ID, PEER_ID, &my_cube, &peer_cube);
}
```

El fallback nunca se activa en ejecución normal (la replanificación cada 20 ms
absorbe todo). Está para robustez extrema.

**Leer:** `FALLBACK_INTEGRATION.md`

### 11.2 — Validador de diagnóstico

Detecta escenarios D≈1 y registra alertas:

```c
#include "planner_validation.h"

plan_validity_t diag = {0};
planner_validate_result(&world, &plan, &diag);

if (diag.is_high_difficulty) {
    ESP_LOGI(TAG, "D≈1: dist=%.1f, interference=%d, makespan=%.1f s",
             diag.avg_distance_cells, diag.interference_count, 
             diag.makespan_s);
}
```

**Leer:** `PLANNER_D1_ANALYSIS.md`

### 11.3 — Suite Python: generar escenarios como la organización

La organización usa el generador de `docs/` para crear escenarios aleatorios.
Incluimos una réplica Python que genera JSON compatibles:

```bash
cd tools/
python scenario_validator.py --difficulty 1.0 --count 10 --save-json
```

Genera 10 escenarios con D=1.0 máxima y los guarda en `scenarios/`.
Úsalos para validar que tu planner no falla contra la dificultad esperada.

**Leer:** `tools/scenario_validator.py` y `PLANNER_D1_ANALYSIS.md`

### 11.4 — Números: por qué no hay que preocuparse

| Métrica | Valor |
|---------|-------|
| Planes a enumerar (n=3 cubos) | 24 (exacto) |
| Tiempo de enumeración | ~100 µs |
| Makespan típico a D=1 | 420–540 s (7–9 min de 10 disponibles) |
| Replanificación automática | cada 20 ms |
| Fallback disponible | sí, pero raro de activar |

El planner enumerativo es óptimo y no falla por computación. El riesgo real es
**ejecución imperfecta** (colisiones, patinaje, cubos que se mueven), no
planificación.

---

## 12. Resumen: próximos pasos

1. ✅ Prueba 1 (motores) — lista
2. ⏳ Pruebas 2–5 — en marcha (tests 2 y 5 dan constantes críticas)
3. 🔜 Test D=1 simulado — ejecuta planner contra escenarios generados
4. 🔜 Test D=1 real — con cámara real en competencia

El planner no te va a dejar en la estacada.
