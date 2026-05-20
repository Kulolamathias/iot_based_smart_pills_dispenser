/**
 * @file stepper_motor.c
 * @brief A4988 Stepper Motor Driver - Implementation
 * 
 * Uses ESP-IDF GPIO API. Delays are implemented with esp_rom_delay_us
 * which blocks the CPU. For production, consider using a timer or
 * separate task if non-blocking stepping is required.
 * 
 * @author System Architect
 * @date 2026-05-20
 */

#include "stepper_motor.h"
#include "esp_log.h"
#include "esp_rom_sys.h"   // for esp_rom_delay_us
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "STEPPER";

/* Static context - holds the current configuration and state */
typedef struct {
    bool initialized;
    gpio_num_t step_pin;
    gpio_num_t dir_pin;
    gpio_num_t enable_pin;
    gpio_num_t ms1_pin;
    gpio_num_t ms2_pin;
    gpio_num_t ms3_pin;
    microstep_mode_t microstep;
    uint32_t step_delay_us;
    bool enabled;           /* true if driver is enabled (ENABLE pin low) */
} stepper_ctx_t;

static stepper_ctx_t s_ctx = { .initialized = false };

/* Forward declarations */
static void set_microstep_pins(microstep_mode_t mode);
static void set_step_pin(bool high);
static void set_dir_pin(bool clockwise);
static void set_enable_pin(bool enable);   /* true = enable (pin low) */

/* ------------------------------------------------------------------------- */
/* Private helpers                                                          */
/* ------------------------------------------------------------------------- */

static void set_microstep_pins(microstep_mode_t mode)
{
    switch (mode) {
        case STEP_FULL:
            gpio_set_level(s_ctx.ms1_pin, 0);
            gpio_set_level(s_ctx.ms2_pin, 0);
            gpio_set_level(s_ctx.ms3_pin, 0);
            break;
        case STEP_HALF:
            gpio_set_level(s_ctx.ms1_pin, 1);
            gpio_set_level(s_ctx.ms2_pin, 0);
            gpio_set_level(s_ctx.ms3_pin, 0);
            break;
        case STEP_QUARTER:
            gpio_set_level(s_ctx.ms1_pin, 0);
            gpio_set_level(s_ctx.ms2_pin, 1);
            gpio_set_level(s_ctx.ms3_pin, 0);
            break;
        case STEP_EIGHTH:
            gpio_set_level(s_ctx.ms1_pin, 1);
            gpio_set_level(s_ctx.ms2_pin, 1);
            gpio_set_level(s_ctx.ms3_pin, 0);
            break;
        case STEP_SIXTEENTH:
            gpio_set_level(s_ctx.ms1_pin, 1);
            gpio_set_level(s_ctx.ms2_pin, 1);
            gpio_set_level(s_ctx.ms3_pin, 1);
            break;
        default:
            ESP_LOGW(TAG, "Unknown microstep mode, default to full step");
            gpio_set_level(s_ctx.ms1_pin, 0);
            gpio_set_level(s_ctx.ms2_pin, 0);
            gpio_set_level(s_ctx.ms3_pin, 0);
            break;
    }
}

static void set_step_pin(bool high)
{
    gpio_set_level(s_ctx.step_pin, high ? 1 : 0);
}

static void set_dir_pin(bool clockwise)
{
    /* High = clockwise, low = counter-clockwise (typical) */
    gpio_set_level(s_ctx.dir_pin, clockwise ? 1 : 0);
}

static void set_enable_pin(bool enable)
{
    /* Enable is active low: low level = driver enabled */
    gpio_set_level(s_ctx.enable_pin, enable ? 0 : 1);
    s_ctx.enabled = enable;
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

esp_err_t stepper_motor_init(const stepper_config_t *cfg)
{
    if (!cfg) {
        ESP_LOGE(TAG, "Null configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }

    /* Check that pins are valid (basic sanity) */
    if (cfg->step_pin == GPIO_NUM_NC || cfg->dir_pin == GPIO_NUM_NC ||
        cfg->enable_pin == GPIO_NUM_NC || cfg->ms1_pin == GPIO_NUM_NC ||
        cfg->ms2_pin == GPIO_NUM_NC || cfg->ms3_pin == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "Invalid GPIO pin(s)");
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg->step_delay_us == 0 || cfg->step_delay_us > 1000000) {
        ESP_LOGE(TAG, "Step delay out of range (1..1,000,000 us)");
        return ESP_ERR_INVALID_ARG;
    }

    /* Fill context */
    s_ctx.step_pin = cfg->step_pin;
    s_ctx.dir_pin = cfg->dir_pin;
    s_ctx.enable_pin = cfg->enable_pin;
    s_ctx.ms1_pin = cfg->ms1_pin;
    s_ctx.ms2_pin = cfg->ms2_pin;
    s_ctx.ms3_pin = cfg->ms3_pin;
    s_ctx.microstep = cfg->microstep;
    s_ctx.step_delay_us = cfg->step_delay_us;
    s_ctx.initialized = false;

    /* Configure all pins as outputs, initial low */
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    uint64_t pin_mask = (1ULL << s_ctx.step_pin) |
                        (1ULL << s_ctx.dir_pin) |
                        (1ULL << s_ctx.enable_pin) |
                        (1ULL << s_ctx.ms1_pin) |
                        (1ULL << s_ctx.ms2_pin) |
                        (1ULL << s_ctx.ms3_pin);
    io_conf.pin_bit_mask = pin_mask;
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIOs: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set initial levels: step low, direction low, enable high (disabled) */
    set_step_pin(false);
    set_dir_pin(false);    /* default direction */
    set_enable_pin(false); /* disabled (high) */
    
    /* Set microstepping pins */
    set_microstep_pins(s_ctx.microstep);

    s_ctx.initialized = true;

    if (cfg->enable_on_init) {
        stepper_motor_enable();
    }

    ESP_LOGI(TAG, "Stepper motor initialized: mode=%s, delay=%lu us",
             stepper_motor_mode_to_string(s_ctx.microstep),
             (unsigned long)s_ctx.step_delay_us);
    return ESP_OK;
}

void stepper_motor_deinit(void)
{
    if (!s_ctx.initialized) return;

    /* Disable driver and reset pins to default */
    stepper_motor_disable();
    gpio_reset_pin(s_ctx.step_pin);
    gpio_reset_pin(s_ctx.dir_pin);
    gpio_reset_pin(s_ctx.enable_pin);
    gpio_reset_pin(s_ctx.ms1_pin);
    gpio_reset_pin(s_ctx.ms2_pin);
    gpio_reset_pin(s_ctx.ms3_pin);
    s_ctx.initialized = false;
    ESP_LOGI(TAG, "Stepper motor deinitialized");
}

esp_err_t stepper_motor_enable(void)
{
    if (!s_ctx.initialized) return ESP_ERR_INVALID_STATE;
    set_enable_pin(true);   /* active low */
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
    set_dir_pin(clockwise);
    ESP_LOGD(TAG, "Direction set to %s", clockwise ? "CW" : "CCW");
    return ESP_OK;
}

esp_err_t stepper_motor_rotate_steps(uint32_t steps)
{
    if (!s_ctx.initialized) return ESP_ERR_INVALID_STATE;
    if (steps == 0) return ESP_OK;

    /* Ensure motor is enabled */
    bool was_enabled = s_ctx.enabled;
    if (!was_enabled) {
        stepper_motor_enable();
    }

    /* Generate step pulses */
    for (uint32_t i = 0; i < steps; i++) {
        set_step_pin(true);
        esp_rom_delay_us(s_ctx.step_delay_us / 2);
        set_step_pin(false);
        esp_rom_delay_us(s_ctx.step_delay_us / 2);
    }

    /* Restore previous enable state */
    if (!was_enabled) {
        stepper_motor_disable();
    }

    ESP_LOGD(TAG, "Rotated %lu steps", (unsigned long)steps);
    return ESP_OK;
}

const char* stepper_motor_mode_to_string(microstep_mode_t mode)
{
    switch (mode) {
        case STEP_FULL:      return "FULL";
        case STEP_HALF:      return "HALF";
        case STEP_QUARTER:   return "QUARTER";
        case STEP_EIGHTH:    return "EIGHTH";
        case STEP_SIXTEENTH: return "SIXTEENTH";
        default:             return "UNKNOWN";
    }
}