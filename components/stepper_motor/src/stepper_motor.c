/**
 * @file stepper_motor.c
 * @brief A4988 Stepper Motor Driver – with proper enable/disable during movement.
 * This component provides a simple interface to control a stepper motor using an
 * A4988 driver. It supports direction, full-step movement, and enable control.
 * 
 * @author Matthithyahu
 * @date 2026-05-20
 */

#include "stepper_motor.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"

static const char *TAG = "STEPPER";

typedef struct {
    bool initialized;
    gpio_num_t step_pin;
    gpio_num_t dir_pin;
    gpio_num_t enable_pin;
    uint32_t step_delay_us;
    bool enabled;        /* current enable state */
} stepper_ctx_t;

static stepper_ctx_t s_ctx = { .initialized = false };

static void set_enable_pin(bool enable)
{
    /* Active low: low = enabled */
    gpio_set_level(s_ctx.enable_pin, enable ? 0 : 1);
    s_ctx.enabled = enable;
}

esp_err_t stepper_motor_init(const stepper_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (s_ctx.initialized) return ESP_ERR_INVALID_STATE;

    s_ctx.step_pin = cfg->step_pin;
    s_ctx.dir_pin = cfg->dir_pin;
    s_ctx.enable_pin = cfg->enable_pin;
    s_ctx.step_delay_us = cfg->step_delay_us;

    /* Configure STEP/DIR/ENABLE only. MS1/MS2/MS3 stay unused for A4988 full-step mode. */
    uint64_t mask = (1ULL << s_ctx.step_pin) | (1ULL << s_ctx.dir_pin) |
                    (1ULL << s_ctx.enable_pin);
    gpio_config_t io_cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_cfg);
    if (ret != ESP_OK) return ret;

    /* Set initial levels */
    gpio_set_level(s_ctx.step_pin, 0);
    gpio_set_level(s_ctx.dir_pin, 0);
    set_enable_pin(false);            /* start disabled (ENABLE pin high) */

    s_ctx.initialized = true;

    /* Enable now only if explicitly requested */
    if (cfg->enable_on_init) {
        set_enable_pin(true);
    }

    ESP_LOGI(TAG, "Stepper init: full-step, delay=%lu us, initially %s",
             (unsigned long)s_ctx.step_delay_us,
             cfg->enable_on_init ? "ENABLED" : "DISABLED");
    return ESP_OK;
}

void stepper_motor_deinit(void)
{
    if (!s_ctx.initialized) return;
    stepper_motor_disable();
    gpio_reset_pin(s_ctx.step_pin);
    gpio_reset_pin(s_ctx.dir_pin);
    gpio_reset_pin(s_ctx.enable_pin);
    s_ctx.initialized = false;
    ESP_LOGI(TAG, "Stepper deinit");
}

esp_err_t stepper_motor_enable(void)
{
    if (!s_ctx.initialized) return ESP_ERR_INVALID_STATE;
    set_enable_pin(true);
    ESP_LOGD(TAG, "Motor enabled");
    return ESP_OK;
}

esp_err_t stepper_motor_disable(void)
{
    if (!s_ctx.initialized) return ESP_ERR_INVALID_STATE;
    set_enable_pin(false);
    ESP_LOGD(TAG, "Motor disabled");
    return ESP_OK;
}

esp_err_t stepper_motor_set_direction(bool clockwise)
{
    if (!s_ctx.initialized) return ESP_ERR_INVALID_STATE;
    gpio_set_level(s_ctx.dir_pin, clockwise ? 1 : 0);
    esp_rom_delay_us(5);
    return ESP_OK;
}

esp_err_t stepper_motor_rotate_steps(uint32_t steps)
{
    if (!s_ctx.initialized) return ESP_ERR_INVALID_STATE;
    if (steps == 0) return ESP_OK;

    /* Remember previous enable state */
    bool was_enabled = s_ctx.enabled;
    if (!was_enabled) {
        stepper_motor_enable();
        /* Allow the A4988 driver to energize the coils before the first step. */
        esp_rom_delay_us(5000);
    }

    /* Generate step pulses */
    for (uint32_t i = 0; i < steps; i++) {
        gpio_set_level(s_ctx.step_pin, 1);
        esp_rom_delay_us(5);
        gpio_set_level(s_ctx.step_pin, 0);
        esp_rom_delay_us(s_ctx.step_delay_us);
    }

    /* Restore previous enable state (disable if it was disabled before) */
    if (!was_enabled) {
        /* Let the last full step settle before removing holding current. */
        esp_rom_delay_us(100000);
        stepper_motor_disable();
    }

    ESP_LOGD(TAG, "Rotated %lu steps", (unsigned long)steps);
    return ESP_OK;
}
