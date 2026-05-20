#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ultrasonic_init(gpio_num_t trig_pin, gpio_num_t echo_pin, uint32_t timeout_us);
float ultrasonic_measure_blocking(void);
void ultrasonic_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* ULTRASONIC_H */