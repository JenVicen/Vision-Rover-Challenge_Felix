#include "imu.h"

#include "pins.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

static const char *TAG = "imu";

#define I2C_PORT I2C_NUM_0
#define I2C_FREQ_HZ 400000
#define LSM6DS3_ADDR 0x6B

/* Registros del LSM6DS3TRC. */
#define REG_WHO_AM_I 0x0F
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G 0x11
#define REG_CTRL3_C 0x12
#define REG_OUTZ_L_G 0x26

#define WHO_AM_I_VALUE 0x6A

/* CTRL2_G = 0x5C -> ODR 208 Hz, escala 2000 dps.
 * A 2000 dps la sensibilidad es 70 mdps/LSB. */
#define GYRO_SENS_DPS_PER_LSB 0.070f

static bool s_available = false;
static float s_drift_dps = 0.0f;

static esp_err_t reg_write(uint8_t reg, uint8_t value)
{
    const uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(I2C_PORT, LSM6DS3_ADDR, buf, sizeof(buf),
                                      pdMS_TO_TICKS(50));
}

static esp_err_t reg_read(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, LSM6DS3_ADDR, &reg, 1, out, len,
                                        pdMS_TO_TICKS(50));
}

bool imu_init(void)
{
    const i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    if (i2c_param_config(I2C_PORT, &cfg) != ESP_OK ||
        i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK)
    {
        ESP_LOGE(TAG, "no se pudo inicializar I2C");
        return false;
    }

    uint8_t who = 0;
    if (reg_read(REG_WHO_AM_I, &who, 1) != ESP_OK || who != WHO_AM_I_VALUE)
    {
        ESP_LOGE(TAG, "IMU no responde (WHO_AM_I=0x%02X, esperado 0x%02X)",
                 who, WHO_AM_I_VALUE);
        return false;
    }

    /* BDU=1 (no mezclar bytes de muestras distintas) + IF_INC=1 (autoincremento). */
    reg_write(REG_CTRL3_C, 0x44);
    /* Acelerometro: 208 Hz, +-8 g. */
    reg_write(REG_CTRL1_XL, 0x5C);
    /* Giroscopio: 208 Hz, +-2000 dps. */
    reg_write(REG_CTRL2_G, 0x5C);

    vTaskDelay(pdMS_TO_TICKS(50));

    s_available = true;
    ESP_LOGI(TAG, "LSM6DS3TRC lista");
    return true;
}

bool imu_available(void)
{
    return s_available;
}

static float read_raw_gyro_z_dps(void)
{
    uint8_t buf[2] = {0, 0};

    if (reg_read(REG_OUTZ_L_G, buf, sizeof(buf)) != ESP_OK)
    {
        return 0.0f;
    }

    const int16_t raw = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    return (float)raw * GYRO_SENS_DPS_PER_LSB;
}

void imu_calibrate_drift(int milliseconds)
{
    if (!s_available)
    {
        return;
    }

    double total = 0.0;
    int samples = 0;
    TickType_t start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(milliseconds))
    {
        const float z = read_raw_gyro_z_dps();

        /* Con el robot quieto cualquier lectura grande es ruido o un golpe:
         * descartarla evita envenenar el promedio. 20 dps es muy generoso. */
        if (fabsf(z) < 20.0f)
        {
            total += z;
            samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    s_drift_dps = (samples > 0) ? (float)(total / samples) : 0.0f;
    ESP_LOGI(TAG, "sesgo del giroscopio: %.3f dps (%d muestras)", s_drift_dps, samples);
}

float imu_gyro_z_dps(void)
{
    if (!s_available)
    {
        return 0.0f;
    }
    return read_raw_gyro_z_dps() - s_drift_dps;
}
