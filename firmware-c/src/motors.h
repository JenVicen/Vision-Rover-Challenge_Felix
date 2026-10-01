/*
 * motors.h - Locomocion diferencial del CenfoBot.
 *
 * Los throttles van de -1.0 (atras) a +1.0 (adelante).
 * "left" es el motor 1 (IO12/IO14), "right" es el motor 2 (IO13/IO15),
 * igual que en codigos/ideaboard.py.
 */
#ifndef MOTORS_H
#define MOTORS_H

void motors_init(void);

/* Aplica ganancia de calibracion, banda muerta e inversiones, y satura. */
void motors_set(float left, float right);

/* Frenado inmediato. Seguro de llamar desde cualquier tarea. */
void motors_stop(void);

#endif /* MOTORS_H */
