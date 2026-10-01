/*
 * vision_client.h - Cliente del sistema oficial de vision.
 *
 * Conecta por Wi-Fi, abre un socket TCP contra VISION_HOST:VISION_PORT y lee
 * NDJSON (un objeto JSON por linea, terminado en '\n').
 *
 * Politica de "ultimo valor gana": se guarda SOLO el mensaje mas reciente.
 * El contrato y el reglamento (seccion 7) exigen decidir con el estado mas
 * reciente y nunca con una cola de estados viejos.
 */
#ifndef VISION_CLIENT_H
#define VISION_CLIENT_H

#include <stdbool.h>

#include "world.h"

/* Levanta Wi-Fi en modo estacion. Bloquea hasta obtener IP. */
bool vision_wifi_start(void);

/* Lanza la tarea que mantiene la conexion TCP y actualiza el mundo. */
void vision_client_start(void);

/* Copia atomica del ultimo mundo conocido. Devuelve false si aun no hay datos. */
bool vision_get_world(world_t *out);

/* Antiguedad en ms del ultimo mensaje recibido. INT32_MAX si nunca llego uno. */
int32_t vision_age_ms(void);

bool vision_is_connected(void);

#endif /* VISION_CLIENT_H */
