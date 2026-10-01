/*
 * esp_link.h - Enlace directo entre los dos rovers por ESP-NOW.
 *
 * Sirve para repartir el trabajo sin que ninguna computadora externa
 * intervenga (reglamento, secciones 6 y 7).
 *
 * Reglas de reparto implementadas:
 *   - Cada rover anuncia que color esta atendiendo y su distancia al cubo.
 *   - Si ambos reclaman el mismo color, gana el que esta mas cerca.
 *   - A igualdad de distancia, gana el ID menor (desempate deterministico:
 *     los dos rovers llegan a la misma conclusion sin negociar).
 *
 * ESP-NOW comparte la radio con el Wi-Fi: el peer se registra con channel = 0
 * para que use el canal en el que ya esta la estacion. Por eso hay que
 * inicializar el Wi-Fi ANTES que esto.
 */
#ifndef ESP_LINK_H
#define ESP_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "world.h"

typedef struct
{
    int rover_id;
    cube_color_t claimed_color;
    float distance_to_target; /* en celdas */
    float col;
    float row;
    bool pushing;   /* true si ya esta empujando el cubo */
    int32_t age_ms; /* antiguedad local del anuncio */
    bool valid;
} peer_status_t;

bool esp_link_start(void);

/* Publica el estado propio. Se llama periodicamente desde la tarea de enlace. */
void esp_link_announce(cube_color_t claimed, float distance, float col, float row,
                       bool pushing);

/* Ultimo estado conocido del otro rover. valid = false si nunca llego nada
 * o si el anuncio ya es demasiado viejo. */
peer_status_t esp_link_peer(void);

#endif /* ESP_LINK_H */
