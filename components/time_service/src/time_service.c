#include "time_service.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "TIME_SVC";

static volatile time_t s_current_timestamp = 0;
static esp_timer_handle_t s_tick_timer = NULL;
static bool s_initialized = false;

static void tick_callback(void *arg)
{
    (void)arg;
    s_current_timestamp++;
}

esp_err_t time_service_init(void)
{
    if (s_initialized) return ESP_OK;

    const esp_timer_create_args_t timer_args = {
        .callback = tick_callback,
        .name = "time_tick"
    };
    esp_err_t ret = esp_timer_create(&timer_args, &s_tick_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create tick timer");
        return ret;
    }

    ret = esp_timer_start_periodic(s_tick_timer, 1000000); /* 1 second */
    if (ret != ESP_OK) {
        esp_timer_delete(s_tick_timer);
        return ret;
    }

    /* Default start time: 2026-05-20 00:00:00 UTC */
    struct tm default_tm = {
        .tm_year = 126, .tm_mon = 4, .tm_mday = 20,
        .tm_hour = 0, .tm_min = 0, .tm_sec = 0
    };
    s_current_timestamp = mktime(&default_tm);

    s_initialized = true;
    ESP_LOGI(TAG, "Time service initialized (software RTC)");
    return ESP_OK;
}

esp_err_t time_service_get_timestamp(time_t *timestamp)
{
    if (!timestamp) return ESP_ERR_INVALID_ARG;
    *timestamp = s_current_timestamp;
    return ESP_OK;
}

esp_err_t time_service_get_tm(struct tm *tm)
{
    if (!tm) return ESP_ERR_INVALID_ARG;
    time_t now = s_current_timestamp;
    localtime_r(&now, tm);
    return ESP_OK;
}

esp_err_t time_service_set_timestamp(time_t timestamp)
{
    s_current_timestamp = timestamp;
    ESP_LOGI(TAG, "Time manually set to %lld", (long long)timestamp);
    return ESP_OK;
}