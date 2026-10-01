#include "status_led.h"

#include "pins.h"

#include "driver/gpio.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * WS2812 por bit-banging con el contador de ciclos de CPU.
 *
 * Se eligio bit-banging en lugar del driver RMT porque la API de RMT cambio
 * entre versiones de ESP-IDF; esto compila igual en todas y el LED solo se
 * actualiza en cambios de estado, asi que el tiempo con interrupciones
 * deshabilitadas es despreciable.
 *
 * Tiempos WS2812B: bit 0 -> 0.35us alto, 0.9us bajo
 *                  bit 1 -> 0.70us alto, 0.6us bajo
 */
#define CPU_MHZ 240

#define T0H_CYCLES (CPU_MHZ * 350 / 1000)
#define T1H_CYCLES (CPU_MHZ * 700 / 1000)
#define BIT_CYCLES (CPU_MHZ * 1250 / 1000)

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_ready = false;

static inline void wait_until(uint32_t start, uint32_t cycles)
{
    while ((esp_cpu_get_cycle_count() - start) < cycles)
    {
        /* espera activa */
    }
}

void status_led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_NEOPIXEL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(PIN_NEOPIXEL, 0);
    s_ready = true;

    status_led_set(0, 0, 0);
}

void status_led_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_ready)
    {
        return;
    }

    /* WS2812 espera los bytes en orden G, R, B, bit mas significativo primero. */
    const uint8_t bytes[3] = {g, r, b};

    taskENTER_CRITICAL(&s_mux);
    for (int i = 0; i < 3; ++i)
    {
        for (int bit = 7; bit >= 0; --bit)
        {
            const uint32_t high = (bytes[i] & (1u << bit)) ? T1H_CYCLES : T0H_CYCLES;
            const uint32_t start = esp_cpu_get_cycle_count();

            gpio_set_level(PIN_NEOPIXEL, 1);
            wait_until(start, high);
            gpio_set_level(PIN_NEOPIXEL, 0);
            wait_until(start, BIT_CYCLES);
        }
    }
    taskEXIT_CRITICAL(&s_mux);

    /* Latch: la linea debe quedar baja >50us. */
    esp_rom_delay_us(60);
}
