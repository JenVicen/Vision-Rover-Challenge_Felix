# Firmware de competencia — Equipo Felix

Firmware ESP-IDF para los CenfoBots 10 y 11 del equipo **Felix**.

**Integrantes:** Jennifer Vicentes Valle y Alejandro Espinoza Morales.

La decisión de movimiento se ejecuta dentro de cada rover. La computadora de visión únicamente publica telemetría global mediante TCP/NDJSON.

## Documentos

- [GUIA_COMPETENCIA.md](GUIA_COMPETENCIA.md): procedimiento de compilación, carga y revisión previa a una ronda.
- [ESPECIFICACION_TECNICA.md](ESPECIFICACION_TECNICA.md): descripción de la arquitectura y de los algoritmos.
- [SETUP_NUEVA_PC.md](SETUP_NUEVA_PC.md): instalación del entorno en Windows.

## Estructura

| Archivo | Responsabilidad |
|---|---|
| `src/config.h` | Identidad, red, calibraciones, ganancias y límites de seguridad. |
| `src/world.h` | Modelo local del contrato de visión v3. |
| `src/vision_client.c` | Wi-Fi, TCP, NDJSON y publicación del último estado válido. |
| `src/planner.c` | Asignación y secuenciación exacta de cubos. |
| `src/mission.c` | Máquina de estados y coordinación física. |
| `src/control.c` | Giro y avance en lazo cerrado. |
| `src/motors.c` | PWM y adaptación al puente H. |
| `src/esp_link.c` | Estado del rover par mediante ESP-NOW. |
| `src/imu.c` | Velocidad angular del giroscopio. |
| `src/sensors.c` | Ultrasónico, infrarrojos y botón. |
| `src/status_led.c` | Indicador WS2812. |
| `src/main.c` | Inicialización y modos de prueba. |

## Entornos de PlatformIO

| Entorno | Uso |
|---|---|
| `rover10` | Firmware normal, marcador ArUco 10. |
| `rover11` | Firmware normal, marcador ArUco 11. |
| `test1_motores` / `test1_motores_rover11` | Sentido de motores. |
| `test2_recta` / `test2_recta_rover11` | Ajuste de avance recto. |
| `test3_sensores` / `test3_sensores_rover11` | Lectura de sensores. |
| `test4_vision` / `test4_vision_rover11` | Red, contrato y marcadores. |
| `test5_giro` / `test5_giro_rover11` | Giro a rumbos absolutos. |

Los perfiles sin sufijo corresponden al rover 10.

## Compilación rápida

El proyecto se compila desde `C:\cenfobot` para evitar espacios en la ruta y archivos temporales dentro de OneDrive.

```powershell
pio run -d C:\cenfobot -e rover10
pio run -d C:\cenfobot -e rover11
```

Carga con puerto explícito:

```powershell
pio run -d C:\cenfobot -e rover10 -t upload --upload-port COM7
pio run -d C:\cenfobot -e rover11 -t upload --upload-port COM6
```

Los puertos son ejemplos. Deben identificarse cada vez que se conectan los equipos.

## Operación

1. Inicializar motores en cero.
2. Calibrar la IMU durante dos segundos con el rover inmóvil.
3. Conectar Wi-Fi y ESP-NOW.
4. Conectar al publicador de visión en el puerto 2026.
5. Esperar `phase=RUNNING` o el botón BOOT.
6. Ejecutar la misión a 50 Hz.
7. Detenerse ante telemetría vencida o al recibir `phase=FINISHED`.

La confirmación de entrega utiliza exclusivamente `in_depot` del contrato v3.
