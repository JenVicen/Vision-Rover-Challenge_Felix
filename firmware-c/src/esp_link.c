#include "esp_link.h"

#include "config.h"

#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <string.h>

static const char *TAG = "link";

/* Se considera caducado un anuncio del otro rover despues de este tiempo.
 * Con LINK_PERIOD_MS = 150 ms esto tolera perder 5 paquetes seguidos. */
#define PEER_STALE_MS 800

/* Paquete binario de tamano fijo. Binario y no texto para que quepa holgado
 * en los 250 B de ESP-NOW y no haya que parsear nada en el receptor. */
typedef struct __attribute__((packed))
{
    uint8_t magic; /* 0x5A: descarta basura de otras redes ESP-NOW */
    uint8_t version;
    int16_t rover_id;
    uint8_t claimed_color;
    uint8_t pushing;
    float distance_to_target;
    float col;
    float row;
} link_packet_t;

#define LINK_MAGIC 0x5A
#define LINK_VERSION 1

static uint8_t s_peer_mac[6];
static peer_status_t s_peer;
static int64_t s_peer_rx_ms = 0;
static SemaphoreHandle_t s_peer_mutex;

static void on_receive(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    (void)info;

    if (len != (int)sizeof(link_packet_t))
    {
        return;
    }

    link_packet_t packet;
    memcpy(&packet, data, sizeof(packet));

    if (packet.magic != LINK_MAGIC || packet.version != LINK_VERSION)
    {
        return;
    }
    if (packet.rover_id == ROVER_ID)
    {
        return; /* eco de nosotros mismos */
    }

    if (xSemaphoreTake(s_peer_mutex, 0) == pdTRUE)
    {
        s_peer.rover_id = packet.rover_id;
        s_peer.claimed_color = (cube_color_t)packet.claimed_color;
        s_peer.distance_to_target = packet.distance_to_target;
        s_peer.col = packet.col;
        s_peer.row = packet.row;
        s_peer.pushing = (packet.pushing != 0);
        s_peer.valid = true;
        s_peer_rx_ms = esp_timer_get_time() / 1000;
        xSemaphoreGive(s_peer_mutex);
    }
}

bool esp_link_start(void)
{
    s_peer_mutex = xSemaphoreCreateMutex();
    memset(&s_peer, 0, sizeof(s_peer));

#if ROVER_ID == 10
    const uint8_t peer[6] = PEER_MAC_ROVER_11;
#else
    const uint8_t peer[6] = PEER_MAC_ROVER_10;
#endif
    memcpy(s_peer_mac, peer, sizeof(s_peer_mac));

    if (esp_now_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_now_init fallo");
        return false;
    }

    esp_now_register_recv_cb(on_receive);

    esp_now_peer_info_t info = {0};
    memcpy(info.peer_addr, s_peer_mac, sizeof(s_peer_mac));
    /* channel = 0 -> usa el canal actual de la estacion Wi-Fi. Fijar un canal
     * distinto al de la red rompe ESP-NOW cuando el Wi-Fi esta conectado. */
    info.channel = 0;
    info.ifidx = WIFI_IF_STA;
    info.encrypt = false;

    if (esp_now_add_peer(&info) != ESP_OK)
    {
        ESP_LOGE(TAG, "no se pudo registrar el peer");
        return false;
    }

    ESP_LOGI(TAG, "ESP-NOW listo. Peer (rover %d): %02X:%02X:%02X:%02X:%02X:%02X",
             PEER_ID, s_peer_mac[0], s_peer_mac[1], s_peer_mac[2],
             s_peer_mac[3], s_peer_mac[4], s_peer_mac[5]);
    return true;
}

void esp_link_announce(cube_color_t claimed, float distance, float col, float row,
                       bool pushing)
{
    const link_packet_t packet = {
        .magic = LINK_MAGIC,
        .version = LINK_VERSION,
        .rover_id = ROVER_ID,
        .claimed_color = (uint8_t)claimed,
        .pushing = pushing ? 1 : 0,
        .distance_to_target = distance,
        .col = col,
        .row = row,
    };

    esp_now_send(s_peer_mac, (const uint8_t *)&packet, sizeof(packet));
}

peer_status_t esp_link_peer(void)
{
    peer_status_t copy = {0};

    if (xSemaphoreTake(s_peer_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        copy = s_peer;
        if (copy.valid)
        {
            copy.age_ms = (int32_t)((esp_timer_get_time() / 1000) - s_peer_rx_ms);
            if (copy.age_ms > PEER_STALE_MS)
            {
                copy.valid = false;
            }
        }
        xSemaphoreGive(s_peer_mutex);
    }
    return copy;
}
