/*
 * pins.h - Mapa de pines del CenfoBot (IdeaBoard ESP32).
 *
 * Fuente de cada valor (verificado en este repositorio):
 *   motores      -> codigos/ideaboard.py  (IO12/IO14 = motor 1, IO13/IO15 = motor 2)
 *   ultrasonico  -> codigos/code_ultrasonic.py (TRIG = IO25, ECHO = IO26)
 *   infrarrojos  -> codigos/code_4IR.py (IO36, IO39, IO34, IO35)
 *   sensor color -> codigos/color_detect.py (LED iluminador IO4, fotosensor IO39)
 *   boton BOOT   -> codigos/color_detect.py (IO0)
 *
 * OJO: el fotosensor de color y el infrarrojo 2 comparten IO39.
 *      No se pueden usar los dos a la vez.
 *
 * NEOPIXEL_GPIO y los pines I2C no aparecen escritos en el repositorio
 * (CircuitPython los resuelve con board.NEOPIXEL / board.I2C). Los valores
 * de abajo son los de la IdeaBoard. Verificarlos con la prueba de hardware
 * descrita en el README antes de confiar en ellos.
 */
#ifndef PINS_H
#define PINS_H

/* Motores (puente H). Requiere el jumper SELECT-Vin colocado. */
#define PIN_M1_A 12
#define PIN_M1_B 14
#define PIN_M2_A 13
#define PIN_M2_B 15

/* NeoPixel integrado (solo estado visual, no critico). */
#define PIN_NEOPIXEL 2

/* I2C (conector Qwiic) -> IMU LSM6DS3TRC. */
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

/* Ultrasonico HC-SR04. Requiere el jumper SELECT-Vin colocado. */
#define PIN_ULTRA_TRIG 25
#define PIN_ULTRA_ECHO 26

/* Infrarrojos analogicos. */
#define PIN_IR_FRONT_LEFT 36
#define PIN_IR_FRONT_RIGHT 39
#define PIN_IR_BACK_LEFT 34
#define PIN_IR_BACK_RIGHT 35

/* Boton BOOT de la IdeaBoard (activo en bajo). */
#define PIN_BUTTON 0

#endif /* PINS_H */
