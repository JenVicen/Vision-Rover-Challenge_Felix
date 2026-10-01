/*
 * sensors.h - Sensores de a bordo: ultrasonico, infrarrojos y boton BOOT.
 *
 * Estos sensores son la red de seguridad local. Las decisiones de navegacion
 * se toman con la telemetria de vision; estos sirven para reaccionar mas
 * rapido que la camara ante un choque inminente o el borde del campo.
 */
#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>

void sensors_init(void);

/* Distancia frontal en cm. Devuelve -1.0 si no hubo eco (fuera de rango). */
float ultrasonic_read_cm(void);

/* Lecturas crudas del ADC (0..4095) de los cuatro infrarrojos. */
void ir_read_all(int *front_left, int *front_right, int *back_left, int *back_right);

/* true mientras el boton BOOT esta presionado (activo en bajo). */
bool button_pressed(void);

#endif /* SENSORS_H */
