#include "buzzer.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <stdbool.h>

static const char *TAG = "BUZZER";

#define BUZZER_LEDC_MODE      LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_TIMER     LEDC_TIMER_0
#define BUZZER_LEDC_CHANNEL   LEDC_CHANNEL_0
#define BUZZER_LEDC_DUTY_RES  LEDC_TIMER_10_BIT
#define BUZZER_LEDC_DUTY_ON   512

static bool s_initialized = false;
static volatile bool s_playing = false;
static QueueHandle_t s_pattern_queue = NULL;

typedef enum {
    BUZZER_PATTERN_NEAR = 0,
    BUZZER_PATTERN_DUE
} buzzer_pattern_t;

static esp_err_t buzzer_tone_blocking(uint32_t frequency_hz, uint32_t duration_ms)
{
    if (!s_initialized || frequency_hz == 0 || duration_ms == 0) return ESP_ERR_INVALID_STATE;

    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, frequency_hz));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_LEDC_DUTY_ON));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL));
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL));
    return ESP_OK;
}

static void play_near_pattern(void)
{
    buzzer_tone_blocking(1400, 90);
    vTaskDelay(pdMS_TO_TICKS(70));
    buzzer_tone_blocking(1800, 90);
}

static void play_due_pattern(void)
{
    buzzer_tone_blocking(2600, 180);
    vTaskDelay(pdMS_TO_TICKS(90));
    buzzer_tone_blocking(2600, 180);
    vTaskDelay(pdMS_TO_TICKS(90));
    buzzer_tone_blocking(2100, 260);
}

static void buzzer_task(void *arg)
{
    (void)arg;
    buzzer_pattern_t pattern;

    while (1) {
        if (xQueueReceive(s_pattern_queue, &pattern, portMAX_DELAY) == pdTRUE) {
            s_playing = true;
            if (pattern == BUZZER_PATTERN_NEAR) {
                play_near_pattern();
            } else {
                play_due_pattern();
            }
            s_playing = false;
        }
    }
}

static void queue_pattern(buzzer_pattern_t pattern)
{
    if (!s_initialized || !s_pattern_queue || s_playing) return;
    if (uxQueueMessagesWaiting(s_pattern_queue) > 0) return;
    xQueueSend(s_pattern_queue, &pattern, 0);
}

esp_err_t buzzer_init(gpio_num_t pin)
{
    if (s_initialized) return ESP_ERR_INVALID_STATE;

    ledc_timer_config_t timer_cfg = {
        .speed_mode = BUZZER_LEDC_MODE,
        .duty_resolution = BUZZER_LEDC_DUTY_RES,
        .timer_num = BUZZER_LEDC_TIMER,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) return ret;

    ledc_channel_config_t channel_cfg = {
        .gpio_num = pin,
        .speed_mode = BUZZER_LEDC_MODE,
        .channel = BUZZER_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BUZZER_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ret = ledc_channel_config(&channel_cfg);
    if (ret != ESP_OK) return ret;

    s_pattern_queue = xQueueCreate(2, sizeof(buzzer_pattern_t));
    if (!s_pattern_queue) return ESP_ERR_NO_MEM;

    if (xTaskCreate(buzzer_task, "buzzer_task", 2048, NULL, 4, NULL) != pdPASS) {
        vQueueDelete(s_pattern_queue);
        s_pattern_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Buzzer initialised on GPIO%d", pin);
    return ESP_OK;
}

void buzzer_deinit(void)
{
    if (!s_initialized) return;
    ledc_stop(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    s_initialized = false;
}

esp_err_t buzzer_tone(uint32_t frequency_hz, uint32_t duration_ms)
{
    return buzzer_tone_blocking(frequency_hz, duration_ms);
}

void buzzer_near_alert(void)
{
    queue_pattern(BUZZER_PATTERN_NEAR);
}

void buzzer_due_alert(void)
{
    queue_pattern(BUZZER_PATTERN_DUE);
}
