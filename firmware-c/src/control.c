#include "control.h"

#include "config.h"
#include "geom.h"
#include "imu.h"
#include "motors.h"

static bool s_turning_in_place = false;

void control_reset(void)
{
    s_turning_in_place = false;
}

/*
 * Correccion diferencial para mantener un rumbo.
 *
 * error_deg > 0 significa "hay que girar en sentido antihorario".
 * TURN_CCW_SIGN traduce ese sentido geometrico al cableado real de los
 * motores, que depende de como quedaron conectados (ver conexiones/README).
 */
static float heading_correction(float error_deg)
{
    const float rate_dps = imu_gyro_z_dps();

    /* El termino derivativo se opone a la velocidad angular actual: amortigua
     * el sobreimpulso en el hueco de 50 ms entre frames de camara. */
    const float correction = HEADING_KP * error_deg - HEADING_KD * rate_dps;

    return clampf(correction, -HEADING_MAX_CORR, HEADING_MAX_CORR) * TURN_CCW_SIGN;
}

bool control_face_heading(const rover_obs_t *me, float target_theta)
{
    const float error = wrap180(target_theta - me->theta);
    const float rate_dps = imu_gyro_z_dps();

    /* El giro termina con error angular y velocidad angular dentro de rango. */
    if (fabsf(error) <= TURN_TOLERANCE_DEG &&
        fabsf(rate_dps) <= TURN_SETTLED_DPS)
    {
        motors_stop();
        s_turning_in_place = false;
        return true;
    }

    /* Velocidad proporcional al error, con un minimo para vencer la friccion
     * estatica y un maximo para no pasarse de largo. El termino derivativo se
     * opone a la velocidad angular actual para amortiguar el sobreimpulso. */
    float speed = TURN_KP * fabsf(error);
    speed = clampf(speed, TURN_MIN_SPEED, TURN_MAX_SPEED);

    const float direction = (error > 0.0f) ? 1.0f : -1.0f;
    float signed_speed = speed * direction * TURN_CCW_SIGN;

    /* Amortiguacion: restar una fraccion de la velocidad angular medida frena
     * el giro cuando ya estamos rotando rapido hacia el objetivo. */
    signed_speed -= (TURN_KD * rate_dps) * TURN_CCW_SIGN;
    signed_speed = clampf(signed_speed, -TURN_MAX_SPEED, TURN_MAX_SPEED);

    /* Giro sobre el propio eje: ruedas en sentidos opuestos. */
    motors_set(-signed_speed, signed_speed);

    s_turning_in_place = true;
    return false;
}

static bool drive_common(const rover_obs_t *me, float col, float row,
                         float arrive_tolerance, float speed,
                         float approach_speed, float slowdown_radius)
{
    const float distance = dist_cells(me->col, me->row, col, row);

    if (distance <= arrive_tolerance)
    {
        motors_stop();
        s_turning_in_place = false;
        return true;
    }

    const float desired = bearing_deg(me->col, me->row, col, row);
    const float error = wrap180(desired - me->theta);

    /* Si estamos muy desalineados, avanzar solo empeora la trayectoria.
     * Histeresis: una vez girando, seguimos hasta quedar bien alineados,
     * para no oscilar entre "girar" y "avanzar" en el umbral. */
    const float enter_threshold = REALIGN_THRESHOLD_DEG;
    const float exit_threshold = REALIGN_THRESHOLD_DEG * 0.4f;

    if (fabsf(error) > enter_threshold ||
        (s_turning_in_place && fabsf(error) > exit_threshold))
    {
        control_face_heading(me, desired);
        return false;
    }

    s_turning_in_place = false;

    /* Frenado progresivo en la aproximacion final. */
    float base = speed;
    if (distance < slowdown_radius)
    {
        const float ratio = distance / slowdown_radius;
        base = approach_speed + (speed - approach_speed) * ratio;
    }

    const float correction = heading_correction(error);

    motors_set(base - correction, base + correction);
    return false;
}

bool control_drive_to(const rover_obs_t *me, float col, float row,
                      float arrive_tolerance, float speed)
{
    return drive_common(me, col, row, arrive_tolerance, speed,
                        APPROACH_SPEED, SLOWDOWN_RADIUS_CELLS);
}

bool control_push_to(const rover_obs_t *me, float col, float row,
                     float arrive_tolerance, float speed)
{
    return drive_common(me, col, row, arrive_tolerance, speed,
                        PUSH_APPROACH_SPEED, PUSH_SLOWDOWN_RADIUS_CELLS);
}
