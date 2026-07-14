#ifndef _BUZZER_H_
#define _BUZZER_H_

#include "esp_err.h"
#include "driver/gpio.h"

esp_err_t buzzer_init(gpio_num_t pin);
void buzzer_deinit(void);
esp_err_t buzzer_tone(uint32_t frequency_hz, uint32_t duration_ms);
void buzzer_near_alert(void);
void buzzer_due_alert(void);

#endif
