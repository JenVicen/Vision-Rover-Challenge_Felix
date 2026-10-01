#include "vision_client.h"

#include "config.h"
#include "geom.h"

#include "cJSON.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <string.h>

static const char *TAG = "vision";

/* Un mensaje del contrato con 2 rovers, 3 cubos y 3 depositos ronda los 700 B.
 * 4 KB deja margen de sobra para obstaculos futuros sin arriesgar la pila. */
#define LINE_BUFFER_SIZE 4096
#define RX_CHUNK_SIZE 1024

#define WIFI_CONNECTED_BIT BIT0

static EventGroupHandle_t s_wifi_events;
static SemaphoreHandle_t s_world_mutex;
static world_t s_world;
static volatile bool s_connected = false;

/* =====================================================================
 * Utilidades del modelo de mundo
 * ===================================================================== */

const char *color_name(cube_color_t c)
{
    switch (c)
    {
    case COLOR_RED:
        return "red";
    case COLOR_GREEN:
        return "green";
    case COLOR_BLUE:
        return "blue";
    default:
        return "none";
    }
}

cube_color_t color_from_name(const char *name)
{
    if (name == NULL)
        return COLOR_NONE;
    if (strcmp(name, "red") == 0)
        return COLOR_RED;
    if (strcmp(name, "green") == 0)
        return COLOR_GREEN;
    if (strcmp(name, "blue") == 0)
        return COLOR_BLUE;
    return COLOR_NONE;
}

const char *phase_name(phase_t p)
{
    switch (p)
    {
    case PHASE_IDLE:
        return "IDLE";
    case PHASE_READY:
        return "READY";
    case PHASE_RUNNING:
        return "RUNNING";
    case PHASE_FINISHED:
        return "FINISHED";
    default:
        return "UNKNOWN";
    }
}

static phase_t phase_from_name(const char *name)
{
    if (name == NULL)
        return PHASE_UNKNOWN;
    if (strcmp(name, "IDLE") == 0)
        return PHASE_IDLE;
    if (strcmp(name, "READY") == 0)
        return PHASE_READY;
    if (strcmp(name, "RUNNING") == 0)
        return PHASE_RUNNING;
    if (strcmp(name, "FINISHED") == 0)
        return PHASE_FINISHED;
    return PHASE_UNKNOWN;
}

const rover_obs_t *world_find_rover(const world_t *w, int id)
{
    /* El contrato advierte que el orden del arreglo no esta definido:
     * hay que buscar por id, nunca indexar por posicion. */
    for (int i = 0; i < w->rover_count; ++i)
    {
        if (w->rovers[i].present && w->rovers[i].id == id)
        {
            return &w->rovers[i];
        }
    }
    return NULL;
}

const cube_obs_t *world_find_cube(const world_t *w, cube_color_t color)
{
    for (int i = 0; i < w->cube_count; ++i)
    {
        if (w->cubes[i].present && w->cubes[i].color == color)
        {
            return &w->cubes[i];
        }
    }
    return NULL;
}

const depot_t *world_find_depot(const world_t *w, cube_color_t color)
{
    for (int i = 0; i < w->depot_count; ++i)
    {
        if (w->depots[i].present && w->depots[i].color == color)
        {
            return &w->depots[i];
        }
    }
    return NULL;
}

bool world_cube_delivered(const world_t *w, const cube_obs_t *cube)
{
    if (cube == NULL || !cube->present)
    {
        return false;
    }

    const depot_t *depot = world_find_depot(w, cube->color);
    if (depot == NULL)
    {
        return false;
    }

    /* Criterio conservador del contrato: el cubo cuenta como dentro solo si
     * su media diagonal cabe dentro del rectangulo del deposito, sea cual sea
     * su orientacion (la orientacion final del cubo no importa). */
    const float cube_half_diag = (w->cube_side * 1.41421356f) * 0.5f;
    const float margin = cube_half_diag + DELIVERY_MARGIN_CELLS;

    const float half_len = w->depot_length * 0.5f;
    const float half_depth = w->depot_depth * 0.5f;

    const float dc = fabsf(cube->col - depot->col);
    const float dr = fabsf(cube->row - depot->row);

    /* No sabemos a que borde esta pegado cada deposito, asi que aceptamos
     * cualquiera de las dos orientaciones del rectangulo. */
    const bool fits_wide = (dc + margin <= half_len) && (dr + margin <= half_depth);
    const bool fits_tall = (dc + margin <= half_depth) && (dr + margin <= half_len);

    return fits_wide || fits_tall;
}

/* =====================================================================
 * Wi-Fi
 * ===================================================================== */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;
    (void)data;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "Wi-Fi caido, reconectando");
        xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "IP obtenida: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

bool vision_wifi_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }

    s_wifi_events = xEventGroupCreate();
    s_world_mutex = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASSWORD,
            sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    /* Sin ahorro de energia: con PS activo el ESP32 duerme entre balizas y
     * pierde paquetes ESP-NOW y frames de telemetria. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "conectando a '%s'...", WIFI_SSID);

    const EventBits_t bits = xEventGroupWaitBits(
        s_wifi_events, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(30000));

    if ((bits & WIFI_CONNECTED_BIT) == 0)
    {
        ESP_LOGE(TAG, "no se pudo conectar a Wi-Fi en 30 s");
        return false;
    }

    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "MAC propia: %02X:%02X:%02X:%02X:%02X:%02X  <-- usar como PEER_MAC en el otro rover",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return true;
}

/* =====================================================================
 * Parseo del contrato
 * ===================================================================== */

static float json_float(const cJSON *obj, const char *key, float fallback)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    return cJSON_IsNumber(item) ? (float)item->valuedouble : fallback;
}

static int json_int(const cJSON *obj, const char *key, int fallback)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    return cJSON_IsNumber(item) ? item->valueint : fallback;
}

static const char *json_string(const cJSON *obj, const char *key)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    return cJSON_IsString(item) ? item->valuestring : NULL;
}

static bool parse_message(const char *line, world_t *out)
{
    cJSON *root = cJSON_Parse(line);
    if (root == NULL)
    {
        return false;
    }

    memset(out, 0, sizeof(*out));

    out->protocol_version = json_int(root, "v", 0);
    if (out->protocol_version != 2)
    {
        /* El contrato indica rechazar versiones desconocidas en lugar de
         * interpretar campos que podrian haber cambiado de significado. */
        ESP_LOGW(TAG, "version de protocolo no soportada: %d", out->protocol_version);
        cJSON_Delete(root);
        return false;
    }

    out->seq = (uint32_t)json_int(root, "seq", 0);
    out->ts_ms = (int64_t)json_float(root, "ts_ms", 0.0f);
    out->phase = phase_from_name(json_string(root, "phase"));

    const cJSON *clock = cJSON_GetObjectItemCaseSensitive(root, "clock");
    if (cJSON_IsObject(clock))
    {
        out->clock_elapsed_ms = json_int(clock, "elapsed_ms", 0);
        out->clock_remaining_ms = json_int(clock, "remaining_ms", 0);
        out->clock_total_ms = json_int(clock, "total_ms", 0);
    }

    const cJSON *grid = cJSON_GetObjectItemCaseSensitive(root, "grid");
    if (cJSON_IsObject(grid))
    {
        out->grid_cols = json_int(grid, "cols", 43);
        out->grid_rows = json_int(grid, "rows", 43);
        out->cell_mm = json_float(grid, "cell_mm", CELL_MM);
    }

    const cJSON *item = NULL;

    const cJSON *rovers = cJSON_GetObjectItemCaseSensitive(root, "rovers");
    cJSON_ArrayForEach(item, rovers)
    {
        if (out->rover_count >= MAX_ROVERS)
            break;
        rover_obs_t *r = &out->rovers[out->rover_count++];
        r->id = json_int(item, "id", -1);
        r->col = json_float(item, "col", 0.0f);
        r->row = json_float(item, "row", 0.0f);
        r->theta = wrap360(json_float(item, "theta", 0.0f));
        r->age_ms = (uint32_t)json_int(item, "age_ms", 0);
        r->present = (r->id >= 0);
    }

    const cJSON *cubes = cJSON_GetObjectItemCaseSensitive(root, "cubes");
    cJSON_ArrayForEach(item, cubes)
    {
        if (out->cube_count >= MAX_CUBES)
            break;
        cube_obs_t *c = &out->cubes[out->cube_count++];
        c->color = color_from_name(json_string(item, "color"));
        c->col = json_float(item, "col", 0.0f);
        c->row = json_float(item, "row", 0.0f);
        c->age_ms = (uint32_t)json_int(item, "age_ms", 0);
        c->present = (c->color != COLOR_NONE);
    }

    const cJSON *obstacles = cJSON_GetObjectItemCaseSensitive(root, "obstacles");
    cJSON_ArrayForEach(item, obstacles)
    {
        if (out->obstacle_count >= MAX_OBSTACLES)
            break;
        obstacle_obs_t *o = &out->obstacles[out->obstacle_count++];
        o->col = json_float(item, "col", 0.0f);
        o->row = json_float(item, "row", 0.0f);
        o->age_ms = (uint32_t)json_int(item, "age_ms", 0);
        o->present = true;
    }

    const cJSON *depots = cJSON_GetObjectItemCaseSensitive(root, "depots");
    cJSON_ArrayForEach(item, depots)
    {
        if (out->depot_count >= MAX_DEPOTS)
            break;
        depot_t *d = &out->depots[out->depot_count++];
        d->color = color_from_name(json_string(item, "color"));
        d->col = json_float(item, "col", 0.0f);
        d->row = json_float(item, "row", 0.0f);
        d->present = (d->color != COLOR_NONE);
    }

    const cJSON *start = cJSON_GetObjectItemCaseSensitive(root, "start");
    if (cJSON_IsObject(start))
    {
        out->start_col = json_float(start, "col", 0.0f);
        out->start_row = json_float(start, "row", 0.0f);
    }

    const cJSON *depot_size = cJSON_GetObjectItemCaseSensitive(root, "depot_size");
    if (cJSON_IsObject(depot_size))
    {
        out->depot_length = json_float(depot_size, "length", 10.0f);
        out->depot_depth = json_float(depot_size, "depth", 7.5f);
    }

    out->cube_side = json_float(root, "cube_side", 3.0f);
    out->rx_time_ms = esp_timer_get_time() / 1000;
    out->valid = true;

    cJSON_Delete(root);
    return true;
}

/* =====================================================================
 * Tarea de recepcion
 * ===================================================================== */

bool vision_get_world(world_t *out)
{
    if (s_world_mutex == NULL)
    {
        return false;
    }

    bool ok = false;
    if (xSemaphoreTake(s_world_mutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        if (s_world.valid)
        {
            *out = s_world;
            ok = true;
        }
        xSemaphoreGive(s_world_mutex);
    }
    return ok;
}

int32_t vision_age_ms(void)
{
    int32_t age = INT32_MAX;

    if (s_world_mutex != NULL &&
        xSemaphoreTake(s_world_mutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        if (s_world.valid)
        {
            age = (int32_t)((esp_timer_get_time() / 1000) - s_world.rx_time_ms);
        }
        xSemaphoreGive(s_world_mutex);
    }
    return age;
}

bool vision_is_connected(void)
{
    return s_connected;
}

static void publish_world(const world_t *parsed)
{
    if (xSemaphoreTake(s_world_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_world = *parsed; /* ultimo valor gana */
        xSemaphoreGive(s_world_mutex);
    }
}

static int connect_to_vision(void)
{
    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port = htons(VISION_PORT),
    };
    dest.sin_addr.s_addr = inet_addr(VISION_HOST);

    const int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "no se pudo crear el socket");
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&dest, sizeof(dest)) != 0)
    {
        ESP_LOGW(TAG, "conexion rechazada por %s:%d", VISION_HOST, VISION_PORT);
        close(sock);
        return -1;
    }

    /* Igual que el publicador: sin Nagle, para no acumular latencia. */
    int one = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    /* Timeout de lectura: si la vision deja de publicar, salimos del recv()
     * y reconectamos en lugar de quedarnos bloqueados para siempre. */
    struct timeval tv = {.tv_sec = 2, .tv_usec = 0};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    ESP_LOGI(TAG, "conectado a la vision en %s:%d", VISION_HOST, VISION_PORT);
    return sock;
}

static void vision_task(void *arg)
{
    (void)arg;

    static char line[LINE_BUFFER_SIZE];
    static char chunk[RX_CHUNK_SIZE];
    static world_t parsed;

    size_t line_len = 0;

    for (;;)
    {
        const int sock = connect_to_vision();
        if (sock < 0)
        {
            s_connected = false;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        s_connected = true;
        line_len = 0;

        for (;;)
        {
            const int received = recv(sock, chunk, sizeof(chunk), 0);

            if (received == 0)
            {
                ESP_LOGW(TAG, "la vision cerro la conexion");
                break;
            }
            if (received < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                    ESP_LOGW(TAG, "sin telemetria durante 2 s, reconectando");
                }
                break;
            }

            /* NDJSON: los mensajes pueden llegar partidos o pegados.
             * Hay que acumular y cortar por '\n'. */
            for (int i = 0; i < received; ++i)
            {
                const char ch = chunk[i];

                if (ch == '\n')
                {
                    line[line_len] = '\0';
                    if (line_len > 0 && parse_message(line, &parsed))
                    {
                        publish_world(&parsed);
                    }
                    line_len = 0;
                }
                else if (line_len < LINE_BUFFER_SIZE - 1)
                {
                    line[line_len++] = ch;
                }
                else
                {
                    /* Linea imposiblemente larga: descartarla entera en lugar
                     * de entregar JSON truncado al parser. */
                    ESP_LOGW(TAG, "linea descartada por exceso de longitud");
                    line_len = 0;
                }
            }
        }

        s_connected = false;
        close(sock);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void vision_client_start(void)
{
    xTaskCreatePinnedToCore(vision_task, "vision", 8192, NULL, 5, NULL, 0);
}
