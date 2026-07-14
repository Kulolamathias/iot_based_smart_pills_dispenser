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
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"

#define STEPPER_RMT_RESOLUTION_HZ 1000000U
#define STEPPER_PULSE_HIGH_US     5U

static const char *TAG = "STEPPER";

typedef struct {
    bool initialized;
    gpio_num_t step_pin;
    gpio_num_t dir_pin;
    gpio_num_t enable_pin;
    uint32_t step_delay_us;
    bool enabled;        /* current enable state */
    rmt_channel_handle_t step_channel;
    rmt_encoder_handle_t step_encoder;
    pcnt_unit_handle_t pulse_counter;
    pcnt_channel_handle_t pulse_counter_channel;
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

    /* Configure DIR/ENABLE only. RMT owns STEP; MS1/MS2/MS3 remain unused. */
    uint64_t mask = (1ULL << s_ctx.dir_pin) | (1ULL << s_ctx.enable_pin);
    gpio_config_t io_cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_cfg);
    if (ret != ESP_OK) return ret;

    /* Set initial levels before attaching the STEP pin to RMT. */
    gpio_reset_pin(s_ctx.step_pin);
    gpio_set_direction(s_ctx.step_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(s_ctx.step_pin, 0);
    gpio_set_level(s_ctx.dir_pin, 0);
    set_enable_pin(false);            /* start disabled (ENABLE pin high) */

    rmt_tx_channel_config_t tx_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = s_ctx.step_pin,
        .mem_block_symbols = 64,
        .resolution_hz = STEPPER_RMT_RESOLUTION_HZ,
        .trans_queue_depth = 1,
    };
    ret = rmt_new_tx_channel(&tx_config, &s_ctx.step_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create STEP RMT channel: %s", esp_err_to_name(ret));
        return ret;
    }

    rmt_copy_encoder_config_t encoder_config = {};
    ret = rmt_new_copy_encoder(&encoder_config, &s_ctx.step_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create STEP RMT encoder: %s", esp_err_to_name(ret));
        rmt_del_channel(s_ctx.step_channel);
        s_ctx.step_channel = NULL;
        return ret;
    }

    ret = rmt_enable(s_ctx.step_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable STEP RMT channel: %s", esp_err_to_name(ret));
        rmt_del_encoder(s_ctx.step_encoder);
        rmt_del_channel(s_ctx.step_channel);
        s_ctx.step_encoder = NULL;
        s_ctx.step_channel = NULL;
        return ret;
    }

    pcnt_unit_config_t counter_config = {
        .high_limit = 32767,
        .low_limit = -1,
    };
    ret = pcnt_new_unit(&counter_config, &s_ctx.pulse_counter);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create STEP pulse counter: %s", esp_err_to_name(ret));
        rmt_disable(s_ctx.step_channel);
        rmt_del_encoder(s_ctx.step_encoder);
        rmt_del_channel(s_ctx.step_channel);
        s_ctx.step_encoder = NULL;
        s_ctx.step_channel = NULL;
        return ret;
    }

    pcnt_chan_config_t counter_channel_config = {
        .edge_gpio_num = s_ctx.step_pin,
        .level_gpio_num = -1,
    };
    ret = pcnt_new_channel(s_ctx.pulse_counter, &counter_channel_config,
                           &s_ctx.pulse_counter_channel);
    if (ret == ESP_OK) {
        ret = pcnt_channel_set_edge_action(s_ctx.pulse_counter_channel,
                                           PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                           PCNT_CHANNEL_EDGE_ACTION_HOLD);
    }
    if (ret == ESP_OK) {
        ret = pcnt_unit_enable(s_ctx.pulse_counter);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure STEP pulse counter: %s", esp_err_to_name(ret));
        if (s_ctx.pulse_counter_channel) {
            pcnt_del_channel(s_ctx.pulse_counter_channel);
        }
        pcnt_del_unit(s_ctx.pulse_counter);
        rmt_disable(s_ctx.step_channel);
        rmt_del_encoder(s_ctx.step_encoder);
        rmt_del_channel(s_ctx.step_channel);
        s_ctx.pulse_counter_channel = NULL;
        s_ctx.pulse_counter = NULL;
        s_ctx.step_encoder = NULL;
        s_ctx.step_channel = NULL;
        return ret;
    }

    s_ctx.initialized = true;

    /* Enable now only if explicitly requested */
    if (cfg->enable_on_init) {
        set_enable_pin(true);
    }

    ESP_LOGI(TAG, "Stepper init: full-step RMT with GPIO pulse verification, pulse=%u us, delay=%lu us, initially %s",
             STEPPER_PULSE_HIGH_US,
             (unsigned long)s_ctx.step_delay_us,
             cfg->enable_on_init ? "ENABLED" : "DISABLED");
    return ESP_OK;
}

void stepper_motor_deinit(void)
{
    if (!s_ctx.initialized) return;
    stepper_motor_disable();
    pcnt_unit_disable(s_ctx.pulse_counter);
    pcnt_del_channel(s_ctx.pulse_counter_channel);
    pcnt_del_unit(s_ctx.pulse_counter);
    rmt_disable(s_ctx.step_channel);
    rmt_del_encoder(s_ctx.step_encoder);
    rmt_del_channel(s_ctx.step_channel);
    s_ctx.step_encoder = NULL;
    s_ctx.step_channel = NULL;
    s_ctx.pulse_counter_channel = NULL;
    s_ctx.pulse_counter = NULL;
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

    ESP_LOGI(TAG, "Starting move: %lu step pulses, delay=%lu us",
             (unsigned long)steps, (unsigned long)s_ctx.step_delay_us);

    rmt_symbol_word_t step_pulse = {
        .level0 = 1,
        .duration0 = STEPPER_PULSE_HIGH_US,
        .level1 = 0,
        .duration1 = s_ctx.step_delay_us,
    };
    rmt_transmit_config_t tx_config = {
        .loop_count = (int)steps,
        .flags.eot_level = 0,
    };

    int observed_pulses = 0;
    int64_t move_start_us = esp_timer_get_time();
    esp_err_t ret = pcnt_unit_clear_count(s_ctx.pulse_counter);
    if (ret == ESP_OK) {
        ret = pcnt_unit_start(s_ctx.pulse_counter);
    }
    if (ret == ESP_OK) {
        ret = rmt_transmit(s_ctx.step_channel, s_ctx.step_encoder,
                           &step_pulse, sizeof(step_pulse), &tx_config);
    }
    if (ret == ESP_OK) {
        ret = rmt_tx_wait_all_done(s_ctx.step_channel, -1);
    }
    esp_err_t counter_ret = pcnt_unit_stop(s_ctx.pulse_counter);
    if (counter_ret == ESP_OK) {
        counter_ret = pcnt_unit_get_count(s_ctx.pulse_counter, &observed_pulses);
    }
    if (ret == ESP_OK && counter_ret != ESP_OK) {
        ret = counter_ret;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "STEP RMT transmission failed: %s", esp_err_to_name(ret));
    } else if (observed_pulses != (int)steps) {
        ESP_LOGE(TAG, "STEP verification failed: requested=%lu, observed=%d",
                 (unsigned long)steps, observed_pulses);
        ret = ESP_ERR_INVALID_RESPONSE;
    }

    /* Restore previous enable state (disable if it was disabled before) */
    if (!was_enabled) {
        /* Let the last full step settle before removing holding current. */
        esp_rom_delay_us(100000);
        stepper_motor_disable();
    }

    if (ret == ESP_OK) {
        int64_t elapsed_us = esp_timer_get_time() - move_start_us;
        ESP_LOGI(TAG, "Move complete: requested=%lu, observed=%d, elapsed=%lld us",
                 (unsigned long)steps, observed_pulses, (long long)elapsed_us);
    }
    return ret;
}
