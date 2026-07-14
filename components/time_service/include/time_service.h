#ifndef TIME_SERVICE_H
#define TIME_SERVICE_H

#include <time.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*time_service_sync_cb_t)(void);

esp_err_t time_service_init(void);

/**
 * @brief Start NTP synchronization (non-blocking)
 * 
 * When a valid time is received, the internal timestamp will be updated.
 * Should be called after WiFi is connected.
 * 
 * @return ESP_OK on success (SNTP started)
 */
esp_err_t time_service_sync_ntp(void);

esp_err_t time_service_get_timestamp(time_t *timestamp);
esp_err_t time_service_get_tm(struct tm *tm);                  // returns LOCAL time (with offset)
esp_err_t time_service_set_timestamp(time_t timestamp);        // set UTC
void time_service_set_timezone(int hours);                     // offset from UTC (e.g., +3 for Tanzania)
bool time_service_is_synchronized(void);
void time_service_register_sync_cb(time_service_sync_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* TIME_SERVICE_H */
