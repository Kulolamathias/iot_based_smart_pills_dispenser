#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "time_service.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Software RTC Test");

    /* Initialize time service */
    ESP_ERROR_CHECK(time_service_init());

    /* Set a manual time (e.g., 2026-05-20 12:00:00) */
    struct tm init_tm = {
        .tm_year = 126,
        .tm_mon = 4,
        .tm_mday = 20,
        .tm_hour = 12,
        .tm_min = 0,
        .tm_sec = 0
    };
    time_t init_ts = mktime(&init_tm);
    time_service_set_timestamp(init_ts);

    /* Print time every second */
    while (1) {
        struct tm now;
        time_service_get_tm(&now);
        char buf[64];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &now);
        ESP_LOGI(TAG, "Current time: %s", buf);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}