# Guía de instalación desde cero en una computadora nueva

> **Para quién es esta guía:** para cualquier persona, aunque no sepa de
> programación. Solo tienes que **copiar cada comando, pegarlo en la terminal
> y presionar Enter**. Los comandos van en bloques grises; cópialos completos.
>
> **Lo único que necesitas tener instalado de antemano:** Visual Studio Code.
> Todo lo demás se instala siguiendo esta guía.
>
> **Sistema operativo:** Windows. Todos los comandos son para **PowerShell**
> (la terminal de Windows).

---

## Índice

1. [Cómo abrir la terminal en VS Code](#1-cómo-abrir-la-terminal-en-vs-code)
2. [Instalar los programas necesarios](#2-instalar-los-programas-necesarios)
3. [Descargar el proyecto (clonar el repositorio)](#3-descargar-el-proyecto-clonar-el-repositorio)
4. [Crear el enlace C:\cenfobot (paso obligatorio)](#4-crear-el-enlace-ccenfobot-paso-obligatorio)
5. [Preparar el entorno de Python](#5-preparar-el-entorno-de-python)
6. [Conectar el rover y encontrar su puerto COM](#6-conectar-el-rover-y-encontrar-su-puerto-com)
7. [Configurar la red en config.h](#7-configurar-la-red-en-configh)
8. [Prueba 1 — Motores](#8-prueba-1--motores)
9. [Prueba 2 — Línea recta](#9-prueba-2--línea-recta)
10. [Prueba 3 — Sensores](#10-prueba-3--sensores)
11. [Prueba 4 — Telemetría de visión](#11-prueba-4--telemetría-de-visión)
12. [Prueba 5 — Giro](#12-prueba-5--giro)
13. [Pruebas con cámara real](#13-pruebas-con-cámara-real)
14. [Compilar la ronda final](#14-compilar-la-ronda-final)
15. [Solución de problemas de red](#15-solución-de-problemas-de-red)

---

## 0. Dos reglas de oro (léelas antes de empezar)

Estas dos cosas causan casi todos los errores. No te preocupes por entenderlas
a fondo; la guía ya las resuelve por ti.

1. **El proyecto se compila desde una carpeta llamada `C:\cenfobot`**, no desde
   donde lo descargas. Es porque la herramienta de compilación no acepta rutas
   con espacios (y "OneDrive - Intel Corporation" tiene espacios). El paso 4
   crea ese atajo.
2. **Los comandos que empiezan con `pio` se corren SIEMPRE desde `C:\cenfobot`.**
   Los comandos de Python (visión) se corren desde la carpeta del proyecto.
   La guía te dirá en cuál estás en cada momento.

---

## 1. Cómo abrir la terminal en VS Code

1. Abre **Visual Studio Code**.
2. En el menú de arriba, haz clic en **Terminal → New Terminal**
   (o presiona las teclas `Ctrl` + `ñ`, o `Ctrl` + `` ` ``).
3. Abajo aparecerá una ventana negra/azul: esa es la **terminal**.
4. Asegúrate de que diga **PowerShell** en la esquina superior derecha de esa
   ventana. Si dice otra cosa, haz clic en la flechita `˅` al lado del `+` y
   elige **PowerShell**.

> A partir de aquí, cada vez que esta guía diga "en la terminal", es esta
> ventana. **Copia el comando, pégalo con `Ctrl` + `V` y presiona `Enter`.**

---

## 2. Instalar los programas necesarios

Necesitas instalar 4 cosas. Hazlo una sola vez.

### 2.1 — Git (para descargar el proyecto)

1. Abre tu navegador y ve a: https://git-scm.com/download/win
2. La descarga empieza sola. Abre el archivo descargado.
3. En el instalador, haz clic en **Next** en todas las pantallas (los valores
   por defecto están bien) y al final en **Install**.

### 2.2 — Python (para el sistema de visión)

1. Ve a: https://www.python.org/downloads/
2. Haz clic en el botón amarillo **"Download Python 3.x.x"**.
3. Abre el archivo descargado.
4. **MUY IMPORTANTE:** en la primera pantalla, marca la casilla de abajo que
   dice **"Add python.exe to PATH"** (añadir Python al PATH). Si no la marcas,
   nada funcionará.
5. Haz clic en **"Install Now"** y espera a que termine.

### 2.3 — Extensión PlatformIO (para programar el rover)

1. En VS Code, haz clic en el icono de **Extensiones** en la barra izquierda
   (son 4 cuadritos), o presiona `Ctrl` + `Shift` + `X`.
2. En la barra de búsqueda escribe: `PlatformIO IDE`
3. En el primer resultado (que dice **PlatformIO IDE**), haz clic en
   **Install**.
4. Espera. Puede tardar varios minutos (descarga herramientas grandes).
5. Cuando termine, **cierra y vuelve a abrir VS Code**.

### 2.4 — Driver USB del rover

El rover usa un chip USB (CP210x o CH340). Si al conectarlo Windows no le
asigna un puerto COM, instala el driver:

- CP210x: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
- CH340: busca en Google "CH340 driver Windows" y descarga de sparkfun o wch.cn

> Normalmente Windows 11 lo instala solo. Si el paso 6 encuentra el COM, puedes
> saltarte esto.

### 2.5 — Comprobar que todo quedó instalado

Cierra y abre VS Code una vez más. Abre la terminal (paso 1) y pega estos dos
comandos, uno por uno:

```powershell
git --version
```

```powershell
python --version
```

Cada uno debe responder con un número de versión (por ejemplo `git version
2.43.0` y `Python 3.12.1`). Si dice "no se reconoce como comando", vuelve a
instalar el programa correspondiente asegurándote de marcar "Add to PATH".

---

## 3. Descargar el proyecto (clonar el repositorio)

Vamos a descargar el proyecto en una carpeta corta y sencilla: `C:\proyectos`.

### 3.1 — Crear la carpeta y entrar en ella

Pega estos comandos uno por uno:

```powershell
mkdir C:\proyectos
```

```powershell
cd C:\proyectos
```

> Si `mkdir` dice que la carpeta ya existe, no pasa nada, continúa.

### 3.2 — Descargar el proyecto

```powershell
git clone https://github.com/JenVicen/Vision-Rover-Challenge_Felix.git
```

Esto crea la carpeta `C:\proyectos\Vision-Rover-Challenge_Felix` con todo el
proyecto. Puede tardar un minuto.

### 3.3 — Entrar en la carpeta del proyecto

```powershell
cd C:\proyectos\Vision-Rover-Challenge_Felix
```

---

## 4. Crear el enlace C:\cenfobot (paso obligatorio)

Este paso crea el atajo corto que necesita la herramienta de compilación.

### 4.1 — Abrir PowerShell como Administrador

1. Presiona la tecla de **Windows**.
2. Escribe: `PowerShell`
3. En el resultado "Windows PowerShell", haz clic derecho y elige
   **"Ejecutar como administrador"**.
4. Si Windows pregunta "¿Permitir que esta app haga cambios?", di **Sí**.

> Esta ventana nueva es distinta a la de VS Code. Solo la usarás para este paso.

### 4.2 — Crear el enlace

Pega este comando (es una sola línea):

```powershell
cmd /c mklink /J C:\cenfobot "C:\proyectos\Vision-Rover-Challenge_Felix\firmware-c"
```

Debe responder: `Union creada para C:\cenfobot <<===>> ...`

### 4.3 — Comprobar que funcionó

```powershell
cd C:\cenfobot
```

```powershell
dir
```

Debes ver una lista que incluye `platformio.ini` y la carpeta `src`. Si la ves,
el enlace quedó bien. **Ya puedes cerrar esta ventana de administrador.**

---

## 5. Preparar el entorno de Python

Esto instala las piezas que necesita el sistema de visión. Vuelve a la
**terminal de VS Code** (la del paso 1).

### 5.1 — Ir a la carpeta del proyecto

```powershell
cd C:\proyectos\Vision-Rover-Challenge_Felix
```

### 5.2 — Crear el entorno virtual de Python

```powershell
python -m venv .venv
```

Esto crea una carpeta `.venv`. Tarda unos segundos.

### 5.3 — Activar el entorno virtual

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy RemoteSigned
```

```powershell
.\.venv\Scripts\Activate.ps1
```

Ahora al inicio de la línea de la terminal debe aparecer `(.venv)`. Eso
significa que el entorno está activo.

> **Importante:** cada vez que abras una terminal nueva para correr el
> simulador o la visión, tendrás que volver a activar el entorno con el
> comando del paso 5.3 (`.\.venv\Scripts\Activate.ps1`).

### 5.4 — Instalar las dependencias

```powershell
pip install -r vision-system/contrato/requirements.txt
```

```powershell
pip install -r vision-system/vision/requirements.txt
```

Cada uno descarga e instala varias librerías. Espera a que terminen.

---

## 6. Conectar el rover y encontrar su puerto COM

### 6.1 — Conectar el rover

Conecta el rover a la computadora con el cable USB. Enciéndelo si tiene
interruptor.

### 6.2 — Encontrar el puerto COM

En la terminal de VS Code, ve al enlace y lista los puertos:

```powershell
cd C:\cenfobot
```

```powershell
pio device list
```

Busca una línea tipo `COM7` o `COM4`. **Anota ese número**; es el puerto de tu
rover. En esta guía usaremos `COM7` como ejemplo; si el tuyo es distinto,
cámbialo en los comandos.

> Si no aparece ningún COM: revisa el cable USB (que no sea de solo carga),
> prueba otro puerto USB, o instala el driver del paso 2.4.

---

## 7. Configurar la red en config.h

El rover necesita saber a qué WiFi conectarse y la IP de esta computadora.

> **REQUISITO DE LA RED:** debe ser una red **WiFi de 2.4 GHz** y **sin
> aislamiento de clientes** (AP isolation). Las redes de operadores (Claro,
> Kölbi, etc.) suelen tener aislamiento y **no sirven** para probar en casa.
> Lo más seguro es un **hotspot de un teléfono Android forzado a 2.4 GHz**,
> o conectar esta PC por **cable de red (UTP)** al router.

### 7.1 — Averiguar la IP de esta computadora

```powershell
ipconfig
```

Busca la sección de tu adaptador (Wi-Fi o Ethernet) y anota el valor de
**"Dirección IPv4"**. Será algo como `192.168.1.143` o `192.168.43.5`.

### 7.2 — Abrir el archivo config.h

En VS Code: menú **File → Open Folder**, elige la carpeta
`C:\proyectos\Vision-Rover-Challenge_Felix`. Luego, en el explorador de la
izquierda, abre: `firmware-c` → `src` → `config.h`.

### 7.3 — Cambiar 3 líneas

Busca estas líneas (cerca de la línea 39) y cámbialas por los datos de tu red:

```c
#define WIFI_SSID     "EL_NOMBRE_DE_TU_RED"
#define WIFI_PASSWORD "LA_CLAVE_DE_TU_RED"
#define VISION_HOST   "LA_IP_QUE_ANOTASTE"
```

**Ejemplo** (reemplaza por lo tuyo):

```c
#define WIFI_SSID     "MiHotspot24G"
#define WIFI_PASSWORD "clave12345"
#define VISION_HOST   "192.168.43.5"
```

Guarda el archivo con `Ctrl` + `S`.

> **Importante sobre el nombre de la red:** usa el **nombre exacto** que
> aparece en la lista de WiFi (respeta mayúsculas, minúsculas y espacios). No
> uses el "nombre del perfil" de Windows, que a veces lleva un número extra.

### 7.4 — Valores que YA están calibrados (NO los toques)

Estos ya fueron medidos y ajustados. Déjalos como están:

| Línea en config.h | Valor correcto | Qué es |
|---|---|---|
| `MOTOR_RIGHT_INVERT` | `1` | el motor derecho va invertido |
| `MOTOR_DEADBAND` | `0.05f` | mínimo para que arranquen los motores |

---

## 8. Prueba 1 — Motores

**Objetivo:** confirmar que los motores giran en el sentido correcto.

**Preparación física:** pon el rover **sobre un soporte con las ruedas al
aire** (una caja, un vaso) para que no se caiga de la mesa.

En la terminal de VS Code:

```powershell
cd C:\cenfobot
```

```powershell
pio run -e test1_motores -t upload -t monitor
```

La primera vez tarda **varios minutos** (compila ESP-IDF entero). Las
siguientes son más rápidas.

Cuando termine de subir, mira los mensajes. Debe pasar esto:

| Mensaje en pantalla | Lo que deben hacer las ruedas |
|---|---|
| ADELANTE | las dos giran hacia adelante |
| ATRAS | las dos giran hacia atrás |
| SOLO MOTOR 1 | gira solo la izquierda, hacia adelante |
| SOLO MOTOR 2 | gira solo la derecha, hacia adelante |
| GIRO EN EL SITIO | giran en sentidos opuestos |

**Para salir del monitor:** presiona `Ctrl` + `C`.

> Si una rueda gira al revés, avísale a quien te ayuda con el firmware; se
> arregla con `MOTOR_LEFT_INVERT` o `MOTOR_RIGHT_INVERT` en config.h.

---

## 9. Prueba 2 — Línea recta

**Objetivo:** que el rover avance derecho.

**Preparación física:** ponlo **en el suelo** con 1 metro libre por delante.

```powershell
cd C:\cenfobot
```

```powershell
pio run -e test2_recta -t upload -t monitor
```

El rover avanza 2 segundos y reporta:

```
desvio tras 2 s: X grados
```

- Si el desvío es **menor a 5 grados**, está bien. Continúa.
- Si es mayor, hay que ajustar `MOTOR_RIGHT_GAIN` en config.h y repetir.

Sal con `Ctrl` + `C`.

---

## 10. Prueba 3 — Sensores

**Objetivo:** confirmar que los sensores responden.

```powershell
cd C:\cenfobot
```

```powershell
pio run -e test3_sensores -t upload -t monitor
```

Verifica, mirando los números en pantalla:

- **ultra:** acerca la mano al sensor delantero → el número baja.
- **IR:** tapa cada sensor de abajo → su número cambia.
- **gyroZ:** quieto marca casi 0; si giras el rover a mano, cambia.
- **boton:** pasa a 1 cuando presionas el botón BOOT.

Sal con `Ctrl` + `C`.

---

## 11. Prueba 4 — Telemetría de visión

**Objetivo:** que el rover reciba datos del sistema de visión por WiFi.

Esta prueba usa **DOS terminales** al mismo tiempo.

### 11.1 — Terminal A: arrancar el simulador

Abre una terminal nueva en VS Code (menú **Terminal → New Terminal**). En ella:

```powershell
cd C:\proyectos\Vision-Rover-Challenge_Felix
```

```powershell
.\.venv\Scripts\Activate.ps1
```

```powershell
python vision-system/contrato/mock_publisher.py
```

Debe quedarse escribiendo líneas como:

```
[estado] fase=IDLE seq=98 clientes=0 pisados=0
```

**No cierres esta terminal.** Déjala corriendo.

### 11.2 — Terminal B: subir el firmware al rover

Abre **otra** terminal nueva (deja la anterior abierta). En la nueva:

```powershell
cd C:\cenfobot
```

```powershell
pio run -e test4_vision -t upload -t monitor
```

### 11.3 — Qué debe pasar (éxito)

- En la **Terminal A** (simulador), el número de `clientes` pasa de `0` a `1`.
- En la **Terminal B** (rover), aparecen líneas como:

```
seq=1043 phase=IDLE grid=43x43 cubos=3 depositos=3 edad=38 ms
  yo (10): col=12.40 row=30.10 theta=271.5 age=0
```

Si ves eso, **¡la prueba 4 pasó!**

### 11.4 — Si dice "sin telemetria (conectado=0)"

Significa que el rover no se pudo conectar. Ve a la sección
[15. Solución de problemas de red](#15-solución-de-problemas-de-red).

**No sigas a la prueba 5 hasta que la 4 funcione.**

---

## 12. Prueba 5 — Giro

**Objetivo:** que el rover gire a ángulos exactos.

**Preparación física:** en el suelo, con espacio para girar.

```powershell
cd C:\cenfobot
```

```powershell
pio run -e test5_giro -t upload -t monitor
```

El rover debe girar hasta mirar a 0°, 90°, 180° y 270°, y detenerse en cada
uno.

- Si **gira sin parar y se aleja del objetivo:** hay que cambiar
  `TURN_CCW_SIGN` a `(-1)` en config.h y repetir.
- Si **tiembla alrededor del objetivo sin asentarse:** hay que bajar `TURN_KP`.

Sal con `Ctrl` + `C`.

---

## 13. Pruebas con cámara real

Cuando las 5 pruebas estén en verde, se pasa a la cámara real (en vez del
simulador).

1. Conecta la **cámara USB** a esta computadora.
2. Sigue las guías específicas del sistema de visión:
   - `vision-system/PUESTA_A_PUNTO.md` (calibración de la cámara).
   - `vision-system/OPERACION.md` (cómo arrancar el sistema real).
3. El procedimiento es **igual que la prueba 4**, pero en la **Terminal A**, en
   lugar del simulador (`mock_publisher.py`), arrancas el **sistema de visión
   real** (ver esas guías). El rover se conecta exactamente igual.

---

## 14. Compilar la ronda final

Con todo probado, para subir el firmware de competencia a cada rover:

**Rover con marcador ArUco número 10:**

```powershell
cd C:\cenfobot
```

```powershell
pio run -e rover10 -t upload
```

**Rover con marcador ArUco número 11:**

```powershell
cd C:\cenfobot
```

```powershell
pio run -e rover11 -t upload
```

---

## 15. Solución de problemas de red

Si la prueba 4 muestra `sin telemetria (conectado=0)`, sigue estos pasos **en
orden**. Usa una terminal de VS Code (no hace falta el entorno de Python).

### 15.1 — ¿El simulador está corriendo?

En la Terminal A debe seguir escribiendo `[estado] fase=... clientes=...`. Si
se cerró, vuelve a arrancarlo (paso 11.1).

Comprueba que esté "escuchando":

```powershell
Get-NetTCPConnection -LocalPort 2026 -State Listen
```

Debe mostrar una línea con `2026`. Si no muestra nada, el simulador no está
corriendo.

### 15.2 — Permitir el puerto en el Firewall de Windows

Abre **PowerShell como administrador** (paso 4.1) y pega:

```powershell
New-NetFirewallRule -DisplayName "VisionRover 2026" -Direction Inbound -Protocol TCP -LocalPort 2026 -Action Allow -Profile Any
```

Debe responder sin errores.

### 15.3 — Averiguar la IP del rover

Mira el monitor del rover (Terminal B) en el arranque. Busca la línea:

```
IP obtenida: 192.168.X.X
```

Anota esa IP.

### 15.4 — ¿Esta PC puede "ver" al rover?

Reemplaza la IP por la del rover y pega:

```powershell
ping 192.168.X.X
```

- Si responde **"Reply from ..."** → la red está bien; el problema es otra cosa
  (revisa que `VISION_HOST` en config.h sea la IP correcta de esta PC).
- Si responde **"Destination host unreachable"** o **"Request timed out"** →
  **esa red tiene aislamiento de clientes (AP isolation)**. El rover y la PC no
  se pueden ver. **Solución: cambia de red.** Usa un hotspot de Android en
  2.4 GHz, o conecta esta PC por cable de red al router.

### 15.5 — La IP de esta PC cambió

Las redes a veces cambian la IP de la computadora. Si dejó de funcionar,
vuelve a correr `ipconfig`, y si la IP cambió, actualiza `VISION_HOST` en
config.h con la nueva, guarda, y vuelve a subir el firmware (paso 11.2).

---

## Resumen de comandos más usados

| Qué quiero hacer | Comando |
|---|---|
| Ir a la carpeta de compilación | `cd C:\cenfobot` |
| Ir a la carpeta del proyecto | `cd C:\proyectos\Vision-Rover-Challenge_Felix` |
| Activar Python | `.\.venv\Scripts\Activate.ps1` |
| Arrancar el simulador | `python vision-system/contrato/mock_publisher.py` |
| Prueba 1 (motores) | `pio run -e test1_motores -t upload -t monitor` |
| Prueba 2 (recta) | `pio run -e test2_recta -t upload -t monitor` |
| Prueba 3 (sensores) | `pio run -e test3_sensores -t upload -t monitor` |
| Prueba 4 (visión) | `pio run -e test4_vision -t upload -t monitor` |
| Prueba 5 (giro) | `pio run -e test5_giro -t upload -t monitor` |
| Subir ronda rover 10 | `pio run -e rover10 -t upload` |
| Subir ronda rover 11 | `pio run -e rover11 -t upload` |
| Salir del monitor | tecla `Ctrl` + `C` |
| Ver puerto del rover | `pio device list` |
| Ver mi IP | `ipconfig` |
