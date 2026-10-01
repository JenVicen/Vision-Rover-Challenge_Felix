#include "sensors.h"

#include "pins.h"

#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sensors";

/* Mapeo pin -> canal de ADC1 en el ESP32 (fijo por hardware). */
#define ADC_CH_IR_FRONT_LEFT ADC_CHANNEL_0  /* IO36 */
#define ADC_CH_IR_FRONT_RIGHT ADC_CHANNEL_3 /* IO39 */
#define ADC_CH_IR_BACK_LEFT ADC_CHANNEL_6   /* IO34 */
#define ADC_CH_IR_BACK_RIGHT ADC_CHANNEL_7  /* IO35 */

/* Handle del ADC1. El driver antiguo (driver/adc.h) se elimino en ESP-IDF 6.0;
 * el actual es esp_adc/adc_oneshot.h y trabaja con handles explicitos. */
static adc_oneshot_unit_handle_t s_adc1 = NULL;

/* Velocidad del sonido: 343 m/s -> 58.0 us por cm ida y vuelta. */
#define US_PER_CM 58.0f

/* Un eco de mas de 25 ms equivale a >4 m: fuera del campo de 1 m. */
#define ECHO_TIMEOUT_US 25000

void sensors_init(void)
{
    /* Ultrasonico */
    gpio_config_t trig = {
        .pin_bit_mask = 1ULL << PIN_ULTRA_TRIG,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&trig);
    gpio_set_level(PIN_ULTRA_TRIG, 0);

    gpio_config_t echo = {
        .pin_bit_mask = 1ULL << PIN_ULTRA_ECHO,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&echo);

    /* Infrarrojos. ADC_ATTEN_DB_12 cubre el rango completo 0-3.3 V
     * (es el nuevo nombre del antiguo ADC_ATTEN_DB_11). */
    const adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    if (adc_oneshot_new_unit(&unit_cfg, &s_adc1) != ESP_OK)
    {
        ESP_LOGE(TAG, "no se pudo inicializar ADC1; los IR leeran -1");
        s_adc1 = NULL;
    }
    else
    {
        const adc_oneshot_chan_cfg_t ch_cfg = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        adc_oneshot_config_channel(s_adc1, ADC_CH_IR_FRONT_LEFT, &ch_cfg);
        adc_oneshot_config_channel(s_adc1, ADC_CH_IR_FRONT_RIGHT, &ch_cfg);
        adc_oneshot_config_channel(s_adc1, ADC_CH_IR_BACK_LEFT, &ch_cfg);
        adc_oneshot_config_channel(s_adc1, ADC_CH_IR_BACK_RIGHT, &ch_cfg);
    }

    /* Boton BOOT */
    gpio_config_t btn = {
        .pin_bit_mask = 1ULL << PIN_BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);
}

/* Espera a que el pin llegue a 'level'. Devuelve el instante, o 0 si expira. */
static int64_t wait_for_level(int level, int64_t deadline_us)
{
    while (gpio_get_level(PIN_ULTRA_ECHO) != level)
    {
        if (esp_timer_get_time() > deadline_us)
        {
            return 0;
        }
    }
    return esp_timer_get_time();
}

float ultrasonic_read_cm(void)
{
    gpio_set_level(PIN_ULTRA_TRIG, 0);
    esp_rom_delay_us(4);
    gpio_set_level(PIN_ULTRA_TRIG, 1);
    esp_rom_delay_us(10);
    gpio_set_level(PIN_ULTRA_TRIG, 0);

    const int64_t deadline = esp_timer_get_time() + ECHO_TIMEOUT_US;

    const int64_t rise = wait_for_level(1, deadline);
    if (rise == 0)
    {
        return -1.0f;
    }

    const int64_t fall = wait_for_level(0, deadline);
    if (fall == 0)
    {
        return -1.0f;
    }

    return (float)(fall - rise) / US_PER_CM;
}

static int adc_read_channel(adc_channel_t ch)
{
    int raw = -1;
    if (s_adc1 != NULL && adc_oneshot_read(s_adc1, ch, &raw) != ESP_OK)
    {
        raw = -1;
    }
    return raw;
}

void ir_read_all(int *front_left, int *front_right, int *back_left, int *back_right)
{
    if (front_left)
        *front_left = adc_read_channel(ADC_CH_IR_FRONT_LEFT);
    if (front_right)
        *front_right = adc_read_channel(ADC_CH_IR_FRONT_RIGHT);
    if (back_left)
        *back_left = adc_read_channel(ADC_CH_IR_BACK_LEFT);
    if (back_right)
        *back_right = adc_read_channel(ADC_CH_IR_BACK_RIGHT);
}

bool button_pressed(void)
{
    return gpio_get_level(PIN_BUTTON) == 0;
}
