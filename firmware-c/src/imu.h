/*
 * imu.h - LSM6DS3TRC (acelerometro + giroscopio) por I2C, direccion 0x6B.
 *
 * Solo se usa el eje Z del giroscopio: aporta el termino derivativo del
 * control de rumbo entre frames de camara. La orientacion absoluta viene
 * siempre de la vision, nunca de integrar el giroscopio (eso derivaria).
 */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>

bool imu_init(void);

/* Calibra el sesgo del giroscopio. El robot debe estar COMPLETAMENTE quieto. */
void imu_calibrate_drift(int milliseconds);

/* Velocidad angular en grados/s alrededor del eje vertical, ya sin sesgo.
 * Devuelve 0 si la IMU no esta disponible. */
float imu_gyro_z_dps(void);

bool imu_available(void);

#endif /* IMU_H */
