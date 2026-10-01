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

    if (fabsf(error) <= TURN_TOLERANCE_DEG)
    {
        motors_stop();
        s_turning_in_place = false;
        return true;
    }

    /* Velocidad proporcional al error, con un minimo para vencer la friccion
     * estatica y un maximo para no pasarse de largo. */
    float speed = TURN_KP * fabsf(error);
    speed = clampf(speed, TURN_MIN_SPEED, TURN_MAX_SPEED);

    const float direction = (error > 0.0f) ? 1.0f : -1.0f;
    const float signed_speed = speed * direction * TURN_CCW_SIGN;

    /* Giro sobre el propio eje: ruedas en sentidos opuestos. */
    motors_set(-signed_speed, signed_speed);

    s_turning_in_place = true;
    return false;
}

static bool drive_common(const rover_obs_t *me, float col, float row,
                         float arrive_tolerance, float speed, bool brake_on_arrival)
{
    const float distance = dist_cells(me->col, me->row, col, row);

    if (distance <= arrive_tolerance)
    {
        if (brake_on_arrival)
        {
            motors_stop();
        }
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
    if (distance < SLOWDOWN_RADIUS_CELLS)
    {
        const float ratio = distance / SLOWDOWN_RADIUS_CELLS;
        base = APPROACH_SPEED + (speed - APPROACH_SPEED) * ratio;
    }

    const float correction = heading_correction(error);

    motors_set(base - correction, base + correction);
    return false;
}

bool control_drive_to(const rover_obs_t *me, float col, float row,
                      float arrive_tolerance, float speed)
{
    return drive_common(me, col, row, arrive_tolerance, speed, true);
}

bool control_push_through(const rover_obs_t *me, float col, float row,
                          float arrive_tolerance, float speed)
{
    return drive_common(me, col, row, arrive_tolerance, speed, false);
}
