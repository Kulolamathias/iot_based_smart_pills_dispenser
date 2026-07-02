/**
 * @file main_stepper_motor_tst.c
 * @brief NEMA17 full-step motor test for the A4988 driver.
 *
 * This optional standalone test assumes:
 * - STEP=GPIO5, DIR=GPIO4, ENABLE=GPIO21.
 * - A4988 MS1/MS2/MS3 are disconnected or tied to GND for full-step mode.
 * - External motor power is connected and the A4988 current limit is set.
 *
 * This file is not included by the normal component CMakeLists.txt.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "stepper_motor.h"

static const char *TAG = "STEPPER_TEST";

#define STEP_PIN                    GPIO_NUM_5
#define DIR_PIN                     GPIO_NUM_4
#define ENABLE_PIN                  GPIO_NUM_21
#define MOTOR_FULL_STEPS_PER_REV    200
#define STEP_DELAY_US               2500

void app_main(void)
{
    ESP_LOGI(TAG, "Starting NEMA17 full-step test");

    stepper_config_t cfg = {
        .step_pin = STEP_PIN,
        .dir_pin = DIR_PIN,
        .enable_pin = ENABLE_PIN,
        .step_delay_us = STEP_DELAY_US,
        .enable_on_init = false
    };

    esp_err_t ret = stepper_motor_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Motor init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Full steps per revolution: %d", MOTOR_FULL_STEPS_PER_REV);

    while (1) {
        ESP_LOGI(TAG, "Rotating clockwise one revolution");
        stepper_motor_set_direction(true);
        stepper_motor_rotate_steps(MOTOR_FULL_STEPS_PER_REV);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Rotating counter-clockwise one revolution");
        stepper_motor_set_direction(false);
        stepper_motor_rotate_steps(MOTOR_FULL_STEPS_PER_REV);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
