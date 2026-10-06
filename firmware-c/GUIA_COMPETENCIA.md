# Guía de competencia — Equipo Felix

**Integrantes:** Jennifer Vicentes Valle y Alejandro Espinoza Morales  
**Firmware normal:** `rover10` y `rover11`

Esta guía cubre la preparación final, carga y verificación de los dos CenfoBots. Los modos `test1_*` a `test5_*` son diagnósticos y no deben quedar instalados para la ronda.

## 1. Material y software

- computadora con Visual Studio Code, PlatformIO y el repositorio actualizado;
- ruta `C:\cenfobot` enlazada al directorio `firmware-c`;
- dos cables USB de datos;
- baterías cargadas;
- red Wi-Fi de 2.4 GHz compartida por la PC y los rovers;
- sistema de visión en protocolo v3 y puerto TCP 2026.

## 2. Revisión de configuración

Antes de compilar, revisar en `src/config.h`:

- `WIFI_SSID` y `WIFI_PASSWORD`;
- `VISION_HOST` con la IPv4 de la PC que publica visión;
- `VISION_PORT` igual a 2026;
- MAC del rover 10 terminada en `BC:10`;
- MAC del rover 11 terminada en `BC:7C`.

No cambiar calibraciones de motores o control el día de competencia sin repetir las pruebas correspondientes.

## 3. Identificar los puertos COM

Los puertos pueden cambiar al reconectar los equipos.

1. Desconectar ambos rovers.
2. Conectar únicamente el rover 10.
3. Ejecutar:

```powershell
pio device list
```

4. Anotar su puerto.
5. Desconectar el rover 10, conectar únicamente el rover 11 y repetir.

Si existe duda, abrir el monitor a 115200 baudios y confirmar `ROVER_ID` y la MAC del arranque.

## 4. Compilar los dos firmwares

```powershell
pio run -d C:\cenfobot -e rover10
pio run -d C:\cenfobot -e rover11
```

Ambos comandos deben terminar con `SUCCESS`.

La advertencia sobre una imagen configurada para 4 MB en un dispositivo detectado con 2 MB ya se ha observado en estas placas. Solo se acepta si la carga concluye correctamente y el firmware inicia; un error de escritura o de arranque no se debe ignorar.

## 5. Cargar el rover 10

Conectar solamente el rover 10 y sustituir `COM7` por el puerto detectado:

```powershell
pio run -d C:\cenfobot -e rover10 -t upload --upload-port COM7
pio device monitor -p COM7 -b 115200
```

Confirmar en el monitor:

```text
ROVER_ID = 10   (peer = 11)   SELFTEST = 0
IP obtenida:
ESP-NOW listo. Peer (rover 11):
conectado a la vision en <IP>:2026
esperando phase=RUNNING o el boton BOOT
```

Cerrar el monitor con `Ctrl+C` antes de usar el puerto en otro proceso.

## 6. Cargar el rover 11

Desconectar el rover 10. Conectar solamente el rover 11 y sustituir `COM6` por el puerto detectado:

```powershell
pio run -d C:\cenfobot -e rover11 -t upload --upload-port COM6
pio device monitor -p COM6 -b 115200
```

Confirmar:

```text
ROVER_ID = 11   (peer = 10)   SELFTEST = 0
IP obtenida:
ESP-NOW listo. Peer (rover 10):
conectado a la vision en <IP>:2026
esperando phase=RUNNING o el boton BOOT
```

## 7. Verificación conjunta

1. Colocar ambos rovers dentro de la vista de la cámara.
2. Encenderlos sin moverlos durante los primeros dos segundos; en ese intervalo se calibra el giroscopio.
3. Comprobar en visión:
   - marcador 10 visible;
   - marcador 11 visible;
   - orientación coherente al girar cada rover a mano;
   - tres cubos y tres depósitos con colores correctos;
   - `age_ms` estable y normalmente menor a 500 ms;
   - fase distinta de `FINISHED` antes de la prueba.
4. Confirmar que cada rover reporta al otro mediante ESP-NOW.
5. Ejecutar una prueba controlada de inicio y detenerla antes de reorganizar la cancha.
6. Reiniciar el publicador y los rovers antes de la ronda oficial si la organización lo solicita.

## 8. Secuencia de inicio de ronda

1. Iniciar el sistema de visión y verificar la imagen completa.
2. Colocar los cubos según la configuración oficial.
3. Colocar los rovers con sus marcadores visibles.
4. Encender ambos rovers y no tocarlos durante la calibración de la IMU.
5. Verificar que permanecen detenidos mientras esperan.
6. Iniciar la ronda con `phase=RUNNING`. El botón BOOT queda como inicio manual permitido por el firmware.
7. No intervenir durante la ejecución salvo por seguridad.

El LED indica:

| Color | Estado |
|---|---|
| Rojo | Detenido o sin telemetría utilizable. |
| Amarillo | Conectando o esperando inicio. |
| Verde | Navegación o verificación. |
| Azul | Alineación o empuje. |
| Magenta | Cesión de paso. |

## 9. Pruebas de diagnóstico

Ejecutarlas únicamente si aparece una falla concreta. Para el rover 11 se usa el entorno con sufijo `_rover11`.

| Capa | Rover 10 | Rover 11 |
|---|---|---|
| Motores | `test1_motores` | `test1_motores_rover11` |
| Línea recta | `test2_recta` | `test2_recta_rover11` |
| Sensores | `test3_sensores` | `test3_sensores_rover11` |
| Visión | `test4_vision` | `test4_vision_rover11` |
| Giro | `test5_giro` | `test5_giro_rover11` |

Patrón de carga:

```powershell
pio run -d C:\cenfobot -e NOMBRE_ENTORNO -t upload --upload-port COMx
```

Después de cualquier diagnóstico es obligatorio volver a cargar `rover10` o `rover11` y confirmar `SELFTEST = 0`.

## 10. Recuperación rápida

| Síntoma | Acción |
|---|---|
| Un rover no aparece por USB | Cambiar cable o puerto, revisar el controlador y ejecutar `pio device list`. |
| Arranca con ID incorrecto | Cargar el entorno normal correspondiente al marcador físico. |
| No obtiene IP | Revisar red de 2.4 GHz, credenciales y cobertura. |
| Obtiene IP pero no recibe visión | Revisar `VISION_HOST`, firewall de Windows y publicador en el puerto 2026. |
| El otro rover no aparece por ESP-NOW | Revisar MAC, canal Wi-Fi compartido y reiniciar ambos equipos. |
| La cámara pierde un marcador | Corregir iluminación, enfoque, altura u oclusión antes de reiniciar. |
| Movimiento inseguro | Cortar alimentación. No sujetar las ruedas energizadas. Revisar el modo cargado y la orientación del marcador. |
| Compilación bloqueada | Confirmar que se usa `C:\cenfobot` y que los objetos van a `C:\cenfobot-build`. |

## 11. Lista final

- [ ] Baterías cargadas y conectores firmes.
- [ ] Ruedas limpias y libres.
- [ ] Marcadores 10 y 11 planos, visibles y orientados correctamente.
- [ ] Cámara enfocada y cancha completa dentro del encuadre.
- [ ] Red de 2.4 GHz estable.
- [ ] IP de visión actualizada.
- [ ] Ambos entornos compilan con `SUCCESS`.
- [ ] Rover 10 cargado con `rover10` y `SELFTEST = 0`.
- [ ] Rover 11 cargado con `rover11` y `SELFTEST = 0`.
- [ ] TCP de visión conectado en ambos rovers.
- [ ] ESP-NOW activo en ambos sentidos.
- [ ] Cubos, depósitos y `in_depot` visibles en telemetría.
- [ ] Prueba conjunta corta completada sin intervención.
