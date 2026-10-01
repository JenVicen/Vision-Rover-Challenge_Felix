/*
 * main.c - Arranque del firmware del CenfoBot.
 *
 * Orden de inicializacion (importa):
 *   1. Motores parados. Lo primero, siempre: si algo falla despues, el robot
 *      ya esta en un estado seguro.
 *   2. LED y sensores.
 *   3. IMU + calibracion del giroscopio (el robot debe estar quieto).
 *   4. Wi-Fi. Tiene que ir ANTES de ESP-NOW: el peer se registra con
 *      channel = 0, es decir "el canal en el que ya esta la estacion".
 *   5. ESP-NOW.
 *   6. Cliente de vision.
 *   7. Tarea de mision.
 *
 * Modos de prueba: compilar con -DSELFTEST=n para las verificaciones de
 * puesta a punto descritas en el README. SELFTEST=0 (por defecto) ejecuta
 * la ronda normal.
 */
#include "config.h"
#include "control.h"
#include "esp_link.h"
#include "geom.h"
#include "imu.h"
#include "mission.h"
#include "motors.h"
#include "pins.h"
#include "sensors.h"
#include "status_led.h"
#include "vision_client.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

#ifndef SELFTEST
#define SELFTEST 0
#endif

/* ====================================================================
 * Pruebas de puesta a punto
 * ==================================================================== */
#if SELFTEST == 1
/* Prueba 1: sentido de los motores.
 * Esperado: adelante, atras, giro a un lado, giro al otro. */
static void selftest_motors(void)
{
    ESP_LOGI(TAG, "PRUEBA 1: sentido de los motores");

    const struct
    {
        const char *label;
        float l, r;
    } steps[] = {
        {"ADELANTE (los dos motores)", 0.50f, 0.50f},
        {"ATRAS", -0.50f, -0.50f},
        {"SOLO MOTOR 1 (izquierdo)", 0.50f, 0.00f},
        {"SOLO MOTOR 2 (derecho)", 0.00f, 0.50f},
        {"GIRO EN EL SITIO", 0.60f, -0.60f}, // Subido a 0.60 para girar bajo carga
    };

    for (int i = 0; i < 5; ++i)
    {
        ESP_LOGI(TAG, "  %s", steps[i].label);
        motors_set(steps[i].l, steps[i].r);
        vTaskDelay(pdMS_TO_TICKS(1500));
        motors_stop();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

#if SELFTEST == 2
/* Prueba 2: calibracion de la ganancia del motor derecho.
 * El robot avanza en recta; el giroscopio mide cuanto se desvio.
 * Si se desvia, ajustar MOTOR_RIGHT_GAIN en config.h y repetir. */
static void selftest_straight(void)
{
    ESP_LOGI(TAG, "PRUEBA 2: linea recta. Dejar 1 m libre delante.");
    vTaskDelay(pdMS_TO_TICKS(3000));

    float heading = 0.0f;
    int64_t last = esp_timer_get_time();

    motors_set(0.40f, 0.40f);
    const int64_t end = esp_timer_get_time() + 2000000;

    while (esp_timer_get_time() < end)
    {
        const int64_t now = esp_timer_get_time();
        const float dt = (float)(now - last) / 1000000.0f;
        last = now;
        heading += imu_gyro_z_dps() * dt;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    motors_stop();

    ESP_LOGI(TAG, "  desvio tras 2 s: %.1f grados", heading);
    ESP_LOGI(TAG, "  si es positivo, BAJAR MOTOR_RIGHT_GAIN ~0.03");
    ESP_LOGI(TAG, "  si es negativo, SUBIR MOTOR_RIGHT_GAIN ~0.03");
    ESP_LOGI(TAG, "  objetivo: menos de 5 grados");
}
#endif

#if SELFTEST == 3
/* Prueba 3: lectura de sensores. */
static void selftest_sensors(void)
{
    ESP_LOGI(TAG, "PRUEBA 3: sensores (Ctrl+C para salir)");

    for (;;)
    {
        int fl, fr, bl, br;
        ir_read_all(&fl, &fr, &bl, &br);
        ESP_LOGI(TAG, "ultra=%.1f cm | IR: %4d %4d %4d %4d | gyroZ=%.2f dps | boton=%d",
                 ultrasonic_read_cm(), fl, fr, bl, br,
                 imu_gyro_z_dps(), button_pressed());
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif

#if SELFTEST == 4
/* Prueba 4: telemetria de vision. Verifica red, contrato y marcador ArUco. */
static void selftest_vision(void)
{
    ESP_LOGI(TAG, "PRUEBA 4: telemetria de vision");

    for (;;)
    {
        world_t w;
        if (!vision_get_world(&w))
        {
            ESP_LOGW(TAG, "sin telemetria (conectado=%d)", vision_is_connected());
        }
        else
        {
            const rover_obs_t *me = world_find_rover(&w, ROVER_ID);
            ESP_LOGI(TAG, "seq=%lu phase=%s grid=%dx%d cubos=%d depositos=%d edad=%ld ms",
                     (unsigned long)w.seq, phase_name(w.phase),
                     w.grid_cols, w.grid_rows, w.cube_count, w.depot_count,
                     (long)vision_age_ms());

            if (me == NULL)
            {
                ESP_LOGW(TAG, "  la camara NO ve el marcador %d", ROVER_ID);
            }
            else
            {
                ESP_LOGI(TAG, "  yo (%d): col=%.2f row=%.2f theta=%.1f age=%lu",
                         me->id, me->col, me->row, me->theta,
                         (unsigned long)me->age_ms);
            }
            for (int i = 0; i < w.cube_count; ++i)
            {
                ESP_LOGI(TAG, "  cubo %-5s col=%.2f row=%.2f entregado=%d",
                         color_name(w.cubes[i].color), w.cubes[i].col,
                         w.cubes[i].row, world_cube_delivered(&w, &w.cubes[i]));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

#if SELFTEST == 5
/* Prueba 5: giro en lazo cerrado sobre theta de la vision.
 * Verifica TURN_CCW_SIGN: si el robot gira alejandose del objetivo, invertirlo. */
static void selftest_turn(void)
{
    ESP_LOGI(TAG, "PRUEBA 5: giro a rumbos absolutos usando la vision");

    const float targets[] = {0.0f, 90.0f, 180.0f, 270.0f};
    int index = 0;

    for (;;)
    {
        world_t w;
        if (!vision_get_world(&w))
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        const rover_obs_t *me = world_find_rover(&w, ROVER_ID);
        if (me == NULL)
        {
            motors_stop();
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (control_face_heading(me, targets[index]))
        {
            ESP_LOGI(TAG, "  rumbo %.0f alcanzado (theta real %.1f)",
                     targets[index], me->theta);
            vTaskDelay(pdMS_TO_TICKS(1500));
            index = (index + 1) % 4;
            control_reset();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
#endif

/* ====================================================================
 * Punto de entrada
 * ==================================================================== */

void app_main(void)
{
    /* 1. Estado seguro antes que nada. */
    motors_init();
    motors_stop();

    status_led_init();
    LED_STOPPED();

    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, " CenfoBot - Vision Rover Challenge");
    ESP_LOGI(TAG, " ROVER_ID = %d   (peer = %d)   SELFTEST = %d",
             ROVER_ID, PEER_ID, SELFTEST);
    ESP_LOGI(TAG, "================================================");

    /* 2. Sensores. */
    sensors_init();

    /* 3. IMU. El robot debe estar quieto durante la calibracion. */
    LED_WAITING();
    if (imu_init())
    {
        ESP_LOGI(TAG, "calibrando giroscopio: NO MOVER EL ROBOT (2 s)");
        imu_calibrate_drift(2000);
    }
    else
    {
        /* Sin IMU se pierde el termino derivativo del control, pero el rumbo
         * absoluto sigue llegando de la camara: el robot puede operar. */
        ESP_LOGW(TAG, "sin IMU: el control seguira, solo con termino proporcional");
    }

#if SELFTEST == 1
    selftest_motors();
    ESP_LOGI(TAG, "prueba terminada");
    for (;;)
        vTaskDelay(portMAX_DELAY);
#elif SELFTEST == 2
    selftest_straight();
    ESP_LOGI(TAG, "prueba terminada");
    for (;;)
        vTaskDelay(portMAX_DELAY);
#elif SELFTEST == 3
    selftest_sensors();
#else
    /* 4. Wi-Fi (antes de ESP-NOW, para fijar el canal). */
    if (!vision_wifi_start())
    {
        ESP_LOGE(TAG, "sin Wi-Fi: el rover no puede recibir telemetria");
        LED_STOPPED();
        for (;;)
            vTaskDelay(portMAX_DELAY);
    }

    /* 5. Enlace directo con el otro rover. */
    if (!esp_link_start())
    {
        ESP_LOGW(TAG, "sin ESP-NOW: se operara sin coordinacion directa");
    }

    /* 6. Cliente de vision. */
    vision_client_start();

#if SELFTEST == 4
    selftest_vision();
#elif SELFTEST == 5
    selftest_turn();
#else
    /* 7. Ronda. */
    ESP_LOGI(TAG, "esperando phase=RUNNING o el boton BOOT");
    mission_start();
#endif
#endif

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
