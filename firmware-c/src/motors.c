#include "motors.h"

#include "config.h"
#include "geom.h"
#include "pins.h"

#include "driver/ledc.h"

/* Frecuencia verificada para conservar par util en el puente H de IdeaBoard. */
#define PWM_FREQ_HZ 50
#define PWM_RES LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY 1023

#define CH_M1_A LEDC_CHANNEL_0
#define CH_M1_B LEDC_CHANNEL_1
#define CH_M2_A LEDC_CHANNEL_2
#define CH_M2_B LEDC_CHANNEL_3

static void channel_init(ledc_channel_t channel, int gpio)
{
    ledc_channel_config_t cfg = {
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&cfg);
}

void motors_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    channel_init(CH_M1_A, PIN_M1_A);
    channel_init(CH_M1_B, PIN_M1_B);
    channel_init(CH_M2_A, PIN_M2_A);
    channel_init(CH_M2_B, PIN_M2_B);

    motors_stop();
}

static void write_duty(ledc_channel_t channel, int duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, (uint32_t)duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

/* Un motor = dos salidas complementarias. Adelante: A con PWM, B a cero. */
static void drive_one(ledc_channel_t a, ledc_channel_t b, float throttle)
{
    const int duty = (int)(fabsf(throttle) * PWM_MAX_DUTY + 0.5f);

    if (throttle > 0.0f)
    {
        write_duty(a, duty);
        write_duty(b, 0);
    }
    else if (throttle < 0.0f)
    {
        write_duty(a, 0);
        write_duty(b, duty);
    }
    else
    {
        write_duty(a, 0);
        write_duty(b, 0);
    }
}

/* Por debajo de la banda muerta el motor no se mueve; por encima, reescalamos
 * para que el rango util [deadband, 1] se recorra de forma continua desde 0. */
static float apply_deadband(float throttle)
{
    if (throttle == 0.0f)
    {
        return 0.0f;
    }

    const float sign = (throttle > 0.0f) ? 1.0f : -1.0f;
    const float mag = fabsf(throttle);

    return sign * (MOTOR_DEADBAND + mag * (1.0f - MOTOR_DEADBAND));
}

void motors_set(float left, float right)
{
    left = clampf(left, -1.0f, 1.0f);
    right = clampf(right, -1.0f, 1.0f) * MOTOR_RIGHT_GAIN;

    left = clampf(apply_deadband(left), -1.0f, 1.0f);
    right = clampf(apply_deadband(right), -1.0f, 1.0f);

#if MOTOR_LEFT_INVERT
    left = -left;
#endif
#if MOTOR_RIGHT_INVERT
    right = -right;
#endif

    drive_one(CH_M1_A, CH_M1_B, left);
    drive_one(CH_M2_A, CH_M2_B, right);
}

void motors_stop(void)
{
    /* El frenado activo puede energizar una rueda en esta IdeaBoard. La parada
     * segura mantiene en cero las cuatro entradas del puente H. */
    write_duty(CH_M1_A, 0);
    write_duty(CH_M1_B, 0);
    write_duty(CH_M2_A, 0);
    write_duty(CH_M2_B, 0);
}
