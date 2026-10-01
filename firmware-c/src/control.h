/*
 * control.h - Control de movimiento en lazo cerrado sobre la vision.
 *
 * Idea central: la orientacion ABSOLUTA viene de la camara (campo theta,
 * 20 Hz, sin deriva). El giroscopio solo aporta el termino derivativo, que
 * necesita ancho de banda alto pero no exactitud absoluta.
 *
 * Esto evita el problema clasico de integrar el giroscopio (turn_angle.py):
 * el error se acumula y, tras varios giros, el robot ya no sabe donde mira.
 *
 * Todas las funciones son NO BLOQUEANTES: se llaman una vez por ciclo de la
 * tarea de mision y devuelven si el objetivo ya se alcanzo. Asi el robot
 * nunca queda atrapado en un bucle interno sin atender la telemetria.
 */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>

#include "world.h"

/* Gira en el sitio hasta mirar a target_theta (grados absolutos).
 * Devuelve true cuando el error cae por debajo de TURN_TOLERANCE_DEG. */
bool control_face_heading(const rover_obs_t *me, float target_theta);

/* Avanza hacia (col,row) corrigiendo el rumbo. Si el error angular es grande
 * gira primero en el sitio. Devuelve true al llegar dentro de la tolerancia.
 *
 * arrive_tolerance: en celdas.
 * speed: velocidad de crucero, 0..1. */
bool control_drive_to(const rover_obs_t *me, float col, float row,
                      float arrive_tolerance, float speed);

/* Igual que control_drive_to pero sin frenar al llegar: se usa al empujar,
 * donde detenerse encima del cubo lo dejaria corto. */
bool control_push_through(const rover_obs_t *me, float col, float row,
                          float arrive_tolerance, float speed);

/* Reinicia el estado interno del controlador (al cambiar de objetivo). */
void control_reset(void);

#endif /* CONTROL_H */
