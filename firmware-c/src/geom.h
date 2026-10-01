/*
 * geom.h - Geometria en el sistema de coordenadas del contrato de vision.
 *
 * Convenio (CONTRATO.md):
 *   col crece a la DERECHA
 *   row crece hacia ABAJO
 *   theta en grados, 0 = +col, ANTIHORARIO positivo, rango [0, 360)
 *
 * Consecuencia importante: como row apunta hacia abajo y los angulos son
 * antihorarios en pantalla, el vector unitario de avance es
 *
 *     (dcol, drow) = (cos(theta), -sin(theta))
 *
 * y el rumbo hacia un punto es atan2(-drow, dcol).
 * El signo negativo de row es el error mas facil de cometer aqui.
 */
#ifndef GEOM_H
#define GEOM_H

#include <math.h>

#define DEG_TO_RAD_F 0.01745329252f
#define RAD_TO_DEG_F 57.2957795131f

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

/* Lleva un angulo al rango [0, 360). */
static inline float wrap360(float deg)
{
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f)
        deg += 360.0f;
    return deg;
}

/* Lleva un angulo al rango (-180, 180]. Usar SIEMPRE para errores angulares. */
static inline float wrap180(float deg)
{
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0.0f)
        deg += 360.0f;
    return deg - 180.0f;
}

static inline float dist_cells(float col_a, float row_a, float col_b, float row_b)
{
    const float dc = col_b - col_a;
    const float dr = row_b - row_a;
    return sqrtf(dc * dc + dr * dr);
}

/* Rumbo, en grados [0,360), desde (col_a,row_a) hacia (col_b,row_b). */
static inline float bearing_deg(float col_a, float row_a, float col_b, float row_b)
{
    const float dc = col_b - col_a;
    const float dr = row_b - row_a;
    return wrap360(atan2f(-dr, dc) * RAD_TO_DEG_F);
}

/* Vector unitario de avance para un theta dado. */
static inline void heading_vector(float theta_deg, float *out_dcol, float *out_drow)
{
    const float r = theta_deg * DEG_TO_RAD_F;
    *out_dcol = cosf(r);
    *out_drow = -sinf(r);
}

#endif /* GEOM_H */
