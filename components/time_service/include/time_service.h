#ifndef TIME_SERVICE_H
#define TIME_SERVICE_H

#include <time.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t time_service_init(void);
esp_err_t time_service_get_timestamp(time_t *timestamp);
esp_err_t time_service_get_tm(struct tm *tm);
esp_err_t time_service_set_timestamp(time_t timestamp);

#ifdef __cplusplus
}
#endif

#endif /* TIME_SERVICE_H */