#include "control.h"

#include "config.h"
#include "geom.h"
#include "imu.h"
#include "motors.h"

#include "esp_timer.h"

static bool s_turning_in_place = false;

/* Momento (ms) en que el rumbo entro en la ventana de tolerancia; 0 si todavia
 * esta fuera. Sirve para exigir que el giro se ASIENTE antes de darlo por bueno
 * (ver control_face_heading). */
static int64_t s_in_tolerance_since_ms = 0;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

void control_reset(void)
{
    s_turning_in_place = false;
    s_in_tolerance_since_ms = 0;
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

    /* Llegada CONFIRMADA. No basta con estar dentro de la ventana: hay que
     * estarlo sin seguir girando, o la inercia nos lleva de largo y el giro
     * oscila (entra, se pasa, vuelve a entrar). Replica el criterio del codigo
     * base de la organizacion: error pequeno Y velocidad angular baja.
     *
     * Como salvaguarda para no quedar nunca atascados si el giroscopio tuviera
     * ruido o sesgo residual, tambien aceptamos tras permanecer TURN_SETTLE_MS
     * dentro de la ventana aunque el rate no haya bajado del umbral. Asi el
     * criterio solo puede RETRASAR un poco la llegada, nunca impedirla. */
    if (fabsf(error) <= TURN_TOLERANCE_DEG)
    {
        if (s_in_tolerance_since_ms == 0)
        {
            s_in_tolerance_since_ms = now_ms();
        }

        const bool settled = fabsf(rate_dps) <= TURN_SETTLED_DPS;
        const bool dwelled = (now_ms() - s_in_tolerance_since_ms) >= TURN_SETTLE_MS;

        if (settled || dwelled)
        {
            motors_stop();
            s_turning_in_place = false;
            s_in_tolerance_since_ms = 0;
            return true;
        }
        /* Dentro de la ventana pero todavia girando: seguimos, el lazo de abajo
         * reduce la velocidad al bajar el error y el robot frena hasta asentar. */
    }
    else
    {
        /* Nos salimos de la ventana (sobreimpulso): reiniciar el cronometro. */
        s_in_tolerance_since_ms = 0;
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
