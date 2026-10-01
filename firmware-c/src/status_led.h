/*
 * status_led.h - NeoPixel integrado, usado solo como indicador de estado.
 * Nunca es critico: si el pin esta mal, el robot sigue funcionando.
 */
#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdint.h>

void status_led_init(void);
void status_led_set(uint8_t r, uint8_t g, uint8_t b);

/* Codigo de colores usado por el firmware:
 *   rojo      -> detenido / sin telemetria
 *   amarillo  -> conectando o esperando el inicio de la ronda
 *   verde     -> ejecutando una tarea
 *   azul      -> empujando un cubo
 *   magenta   -> cediendo el paso al otro rover
 */
#define LED_STOPPED() status_led_set(60, 0, 0)
#define LED_WAITING() status_led_set(50, 35, 0)
#define LED_RUNNING() status_led_set(0, 60, 0)
#define LED_PUSHING() status_led_set(0, 20, 60)
#define LED_YIELDING() status_led_set(50, 0, 50)
#define LED_OFF() status_led_set(0, 0, 0)

#endif /* STATUS_LED_H */
