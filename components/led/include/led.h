#ifndef _LED_H_
#define _LED_H_

#include "esp_err.h"
#include "driver/gpio.h"
#include <stdbool.h>

typedef enum {
    RGB_LED_OFF = 0,
    RGB_LED_GREEN,
    RGB_LED_RED,
    RGB_LED_YELLOW,
    RGB_LED_BLUE,
    RGB_LED_CYAN,
    RGB_LED_IDLE,
    RGB_LED_WHITE
} rgb_led_color_t;

esp_err_t rgb_led_init(gpio_num_t red_pin, gpio_num_t green_pin, gpio_num_t blue_pin, bool active_high);
void rgb_led_set(rgb_led_color_t color);
void rgb_led_deinit(void);

#endif
