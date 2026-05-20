#include "time_service.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include <string.h>

static const char *TAG = "TIME_SVC";

static volatile time_t s_current_timestamp_utc = 0;
static int s_timezone_offset_seconds = 0;      // offset to add for local time
static esp_timer_handle_t s_tick_timer = NULL;
static bool s_initialized = false;
static bool s_ntp_sync_in_progress = false;

/* 1‑second timer callback – increments UTC timestamp */
static void tick_callback(void *arg)
{
    (void)arg;
    s_current_timestamp_utc++;
}

/* NTP callback – sets UTC timestamp */
static void ntp_sync_callback(struct timeval *tv)
{
    if (tv) {
        s_current_timestamp_utc = tv->tv_sec;
        ESP_LOGI(TAG, "NTP sync successful, UTC time = %lld", (long long)s_current_timestamp_utc);
    } else {
        ESP_LOGW(TAG, "NTP sync failed");
    }
    s_ntp_sync_in_progress = false;
}

esp_err_t time_service_init(void)
{
    if (s_initialized) return ESP_OK;

    const esp_timer_create_args_t timer_args = {
        .callback = tick_callback,
        .name = "time_tick"
    };
    esp_err_t ret = esp_timer_create(&timer_args, &s_tick_timer);
    if (ret != ESP_OK) return ret;

    ret = esp_timer_start_periodic(s_tick_timer, 1000000);
    if (ret != ESP_OK) {
        esp_timer_delete(s_tick_timer);
        return ret;
    }

    /* Dummy UTC start time (will be overwritten by NTP) */
    struct tm default_tm = {
        .tm_year = 126, .tm_mon = 4, .tm_mday = 20,
        .tm_hour = 0, .tm_min = 0, .tm_sec = 0
    };
    s_current_timestamp_utc = mktime(&default_tm);
    s_timezone_offset_seconds = 0;  // default UTC

    s_initialized = true;
    ESP_LOGI(TAG, "Time service initialized (UTC)");
    return ESP_OK;
}

esp_err_t time_service_sync_ntp(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (s_ntp_sync_in_progress) return ESP_OK;

    esp_sntp_stop();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(ntp_sync_callback);
    esp_sntp_init();

    s_ntp_sync_in_progress = true;
    ESP_LOGI(TAG, "NTP sync started");
    return ESP_OK;
}

/* Return UTC timestamp (raw) */
esp_err_t time_service_get_timestamp(time_t *timestamp)
{
    if (!timestamp) return ESP_ERR_INVALID_ARG;
    *timestamp = s_current_timestamp_utc;
    return ESP_OK;
}

/* Return LOCAL time (UTC + offset) */
esp_err_t time_service_get_tm(struct tm *tm)
{
    if (!tm) return ESP_ERR_INVALID_ARG;
    time_t local_sec = s_current_timestamp_utc + s_timezone_offset_seconds;
    localtime_r(&local_sec, tm);
    return ESP_OK;
}

/* Set UTC timestamp directly */
esp_err_t time_service_set_timestamp(time_t timestamp)
{
    s_current_timestamp_utc = timestamp;
    ESP_LOGI(TAG, "UTC time manually set to %lld", (long long)timestamp);
    return ESP_OK;
}

/* Set timezone offset in hours (e.g., +3 for Tanzania) */
void time_service_set_timezone(int hours)
{
    s_timezone_offset_seconds = hours * 3600;
    ESP_LOGI(TAG, "Timezone offset set to %+d hours (UTC%+d)", hours, hours);
}