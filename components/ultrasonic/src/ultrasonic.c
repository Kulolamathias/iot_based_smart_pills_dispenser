/**
 * @file ultrasonic.c
 * @brief HC-SR04 driver – blocking measurement using polling.
 */

#include "ultrasonic.h"
#include "esp_log.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static const char *TAG = "ULTRASONIC";

static gpio_num_t s_trig_pin = GPIO_NUM_NC;
static gpio_num_t s_echo_pin = GPIO_NUM_NC;
static uint32_t s_timeout_us = 30000;
static bool s_initialized = false;

esp_err_t ultrasonic_init(gpio_num_t trig_pin, gpio_num_t echo_pin, uint32_t timeout_us)
{
    if (s_initialized) return ESP_ERR_INVALID_STATE;

    s_trig_pin = trig_pin;
    s_echo_pin = echo_pin;
    s_timeout_us = timeout_us;

    /* Configure TRIG as output */
    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << s_trig_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&trig_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TRIG pin config failed");
        return ret;
    }

    /* Configure ECHO as input (no interrupt) */
    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << s_echo_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&echo_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ECHO pin config failed");
        return ret;
    }

    /* Ensure TRIG is low */
    gpio_set_level(s_trig_pin, 0);

    s_initialized = true;
    ESP_LOGI(TAG, "Ultrasonic initialized (blocking): TRIG=%d, ECHO=%d, timeout=%lu us",
             s_trig_pin, s_echo_pin, s_timeout_us);
    return ESP_OK;
}

float ultrasonic_measure_blocking(void)
{
    if (!s_initialized) return -1.0f;

    /* Send trigger pulse (10 µs high) */
    gpio_set_level(s_trig_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(s_trig_pin, 0);

    /* Wait for echo pin to go high (start of pulse) */
    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(s_echo_pin) == 0) {
        if ((esp_timer_get_time() - start_time) > s_timeout_us) {
            ESP_LOGW(TAG, "Echo timeout (no rising edge)");
            return 0.0f;
        }
    }

    /* Measure pulse width */
    int64_t pulse_start = esp_timer_get_time();
    while (gpio_get_level(s_echo_pin) == 1) {
        if ((esp_timer_get_time() - pulse_start) > s_timeout_us) {
            ESP_LOGW(TAG, "Echo timeout (pulse too long)");
            return 0.0f;
        }
    }
    int64_t pulse_end = esp_timer_get_time();
    int64_t pulse_width_us = pulse_end - pulse_start;

    /* Convert to distance (cm) */
    float distance = (pulse_width_us * 1715.0f) / 100000.0f;
    return distance;
}

void ultrasonic_deinit(void)
{
    if (!s_initialized) return;
    gpio_reset_pin(s_trig_pin);
    gpio_reset_pin(s_echo_pin);
    s_initialized = false;
    ESP_LOGI(TAG, "Ultrasonic deinitialized");
}