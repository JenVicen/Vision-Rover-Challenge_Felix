# Instalación del entorno en Windows

Procedimiento para preparar una computadora de desarrollo del equipo **Felix**.

## 1. Requisitos

Instalar:

- Git;
- Python 3;
- Visual Studio Code;
- extensión PlatformIO IDE;
- controlador USB CP210x o CH340, según el adaptador detectado por Windows.

Comprobar desde PowerShell:

```powershell
git --version
python --version
pio --version
```

Si `pio` no está disponible después de instalar la extensión, abrir una terminal de PlatformIO o instalar la interfaz de línea de comandos:

```powershell
python -m pip install platformio
```

## 2. Obtener el repositorio

```powershell
New-Item -ItemType Directory -Force C:\proyectos | Out-Null
Set-Location C:\proyectos
git clone https://github.com/JenVicen/Vision-Rover-Challenge_Felix.git
```

Si el repositorio ya existe, actualizarlo con el flujo de Git acordado por el equipo en vez de clonarlo de nuevo.

## 3. Crear la ruta de compilación

ESP-IDF no admite la ruta de OneDrive con espacios y OneDrive puede bloquear archivos temporales. El proyecto usa un junction corto y un directorio de compilación externo.

```powershell
$Destino = "C:\proyectos\Vision-Rover-Challenge_Felix\firmware-c"
New-Item -ItemType Junction -Path C:\cenfobot -Target $Destino
Get-Item C:\cenfobot
```

Si `C:\cenfobot` ya existe, verificar que apunte al directorio correcto. No crear una copia independiente del firmware.

`platformio.ini` envía los objetos de compilación a `C:\cenfobot-build`.

## 4. Configurar la red

Editar `src/config.h` y definir:

```c
#define WIFI_SSID "NOMBRE_RED_2_4_GHZ"
#define WIFI_PASSWORD "CLAVE_RED"
#define VISION_HOST "IP_DE_LA_PC_DE_VISION"
```

La PC de visión y los dos rovers deben estar en la misma red de 2.4 GHz. La red no debe usar aislamiento entre clientes.

No publicar credenciales reales en capturas, documentos o registros de terminal.

## 5. Preparar el sistema de visión

Desde la raíz del repositorio:

```powershell
python -m venv .venv
Set-ExecutionPolicy -Scope Process -ExecutionPolicy RemoteSigned
.\.venv\Scripts\Activate.ps1
python -m pip install -r vision-system\contrato\requirements.txt
python -m pip install -r vision-system\vision\requirements.txt
```

La documentación de ejecución se encuentra en `vision-system/README.md`.

## 6. Identificar el puerto USB

Conectar un solo rover y ejecutar:

```powershell
pio device list
```

Anotar el puerto COM. Repetir con el otro rover por separado. Windows puede asignar un puerto distinto después de reconectar el dispositivo.

La identidad se confirma en el arranque:

```text
ROVER_ID = 10
```

También se puede comprobar la MAC:

- rover 10 termina en `BC:10`;
- rover 11 termina en `BC:7C`.

## 7. Compilar

```powershell
pio run -d C:\cenfobot -e rover10
pio run -d C:\cenfobot -e rover11
```

## 8. Cargar y abrir el monitor

Sustituir los puertos por los detectados:

```powershell
pio run -d C:\cenfobot -e rover10 -t upload --upload-port COM7
pio device monitor -p COM7 -b 115200
```

```powershell
pio run -d C:\cenfobot -e rover11 -t upload --upload-port COM6
pio device monitor -p COM6 -b 115200
```

La primera compilación descarga dependencias y puede tardar varios minutos.

## 9. Diagnóstico

| Problema | Revisión |
|---|---|
| ESP-IDF rechaza la ruta | Compilar con `-d C:\cenfobot`. |
| `Access is denied` durante el build | Confirmar `build_dir = C:/cenfobot-build` y cerrar procesos que usen esa carpeta. |
| No aparece el puerto COM | Probar otro cable USB y revisar el controlador CP210x/CH340. |
| No conecta a visión | Revisar `VISION_HOST`, firewall, puerto 2026 y red de 2.4 GHz. |
| No hay telemetría válida | Confirmar protocolo v3 y marcador ArUco correcto. |
| Se cargó un modo de prueba | Volver a cargar `rover10` o `rover11`. |

Para el procedimiento de campo usar `GUIA_COMPETENCIA.md`.
