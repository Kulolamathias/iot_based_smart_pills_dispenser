/**
 * @file main.c
 * @brief Stepper motor test – verifies rotation and direction.
 * 
 * This test assumes:
 * - A4988 driver connected as per pin definitions below.
 * - Motor power supply is on.
 * - No other components are needed.
 * 
 * After flashing, the motor will rotate clockwise for one revolution,
 * pause, then counter‑clockwise for one revolution, and repeat.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "stepper_motor.h"

static const char *TAG = "MAIN";

/* ============================================================
 * Hardware Configuration (adjust to your wiring)
 * ============================================================ */
#define STEP_PIN      GPIO_NUM_3
#define DIR_PIN       GPIO_NUM_4
#define ENABLE_PIN    GPIO_NUM_5
#define MS1_PIN       GPIO_NUM_9
#define MS2_PIN       GPIO_NUM_10
#define MS3_PIN       GPIO_NUM_11

/* Stepper motor parameters */
#define MOTOR_FULL_STEPS_PER_REV     48
#define MICROSTEP_MODE               STEP_SIXTEENTH   /* 1/16 microstepping */
#define STEP_DELAY_US                1000             /* 1000 µs = 500 steps/sec */

/* Total steps per revolution = full steps * microstep factor */
static inline uint32_t get_steps_per_rev(void)
{
    uint32_t factor = 1;
    switch (MICROSTEP_MODE) {
        case STEP_FULL:      factor = 1;   break;
        case STEP_HALF:      factor = 2;   break;
        case STEP_QUARTER:   factor = 4;   break;
        case STEP_EIGHTH:    factor = 8;   break;
        case STEP_SIXTEENTH: factor = 16;  break;
        default:             factor = 1;   break;
    }
    return MOTOR_FULL_STEPS_PER_REV * factor;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Stepper Motor Test");

    /* 1. Configure the motor */
    stepper_config_t cfg = {
        .step_pin = STEP_PIN,
        .dir_pin = DIR_PIN,
        .enable_pin = ENABLE_PIN,
        .ms1_pin = MS1_PIN,
        .ms2_pin = MS2_PIN,
        .ms3_pin = MS3_PIN,
        .microstep = MICROSTEP_MODE,
        .step_delay_us = STEP_DELAY_US,
        .enable_on_init = true   /* enable immediately */
    };

    esp_err_t ret = stepper_motor_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Motor init failed: %s", esp_err_to_name(ret));
        return;
    }

    uint32_t steps_per_rev = get_steps_per_rev();
    ESP_LOGI(TAG, "Steps per revolution: %lu", (unsigned long)steps_per_rev);

    /* 2. Test loop: CW, pause, CCW, pause */
    while (1) {
        ESP_LOGI(TAG, "Rotating CLOCKWISE...");
        stepper_motor_set_direction(true);
        stepper_motor_rotate_steps(steps_per_rev);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Rotating COUNTER-CLOCKWISE...");
        stepper_motor_set_direction(false);
        stepper_motor_rotate_steps(steps_per_rev);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}