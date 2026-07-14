#include "led.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "RGB_LED";

#define RGB_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define RGB_LEDC_TIMER      LEDC_TIMER_1
#define RGB_LEDC_DUTY_RES   LEDC_TIMER_10_BIT
#define RGB_LEDC_MAX_DUTY   1023
#define RGB_RED_CHANNEL     LEDC_CHANNEL_1
#define RGB_GREEN_CHANNEL   LEDC_CHANNEL_2
#define RGB_BLUE_CHANNEL    LEDC_CHANNEL_3
#define RGB_YELLOW_RED_DUTY 410

static gpio_num_t s_red_pin;
static gpio_num_t s_green_pin;
static gpio_num_t s_blue_pin;
static bool s_active_high = true;
static bool s_initialized = false;

static uint32_t output_duty(uint16_t brightness)
{
    if (brightness > RGB_LEDC_MAX_DUTY) {
        brightness = RGB_LEDC_MAX_DUTY;
    }

    return s_active_high ? brightness : (RGB_LEDC_MAX_DUTY - brightness);
}

static void set_channel(ledc_channel_t channel, uint16_t brightness)
{
    uint32_t duty = output_duty(brightness);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_set_duty(RGB_LEDC_MODE, channel, duty));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_update_duty(RGB_LEDC_MODE, channel));
}

static esp_err_t configure_channel(ledc_channel_t channel, gpio_num_t pin)
{
    ledc_channel_config_t cfg = {
        .gpio_num = pin,
        .speed_mode = RGB_LEDC_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = RGB_LEDC_TIMER,
        .duty = output_duty(0),
        .hpoint = 0,
    };

    return ledc_channel_config(&cfg);
}

esp_err_t rgb_led_init(gpio_num_t red_pin, gpio_num_t green_pin, gpio_num_t blue_pin, bool active_high)
{
    s_red_pin = red_pin;
    s_green_pin = green_pin;
    s_blue_pin = blue_pin;
    s_active_high = active_high;

    ledc_timer_config_t timer_cfg = {
        .speed_mode = RGB_LEDC_MODE,
        .duty_resolution = RGB_LEDC_DUTY_RES,
        .timer_num = RGB_LEDC_TIMER,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) return ret;

    ret = configure_channel(RGB_RED_CHANNEL, red_pin);
    if (ret != ESP_OK) return ret;
    ret = configure_channel(RGB_GREEN_CHANNEL, green_pin);
    if (ret != ESP_OK) return ret;
    ret = configure_channel(RGB_BLUE_CHANNEL, blue_pin);
    if (ret != ESP_OK) return ret;

    s_initialized = true;
    rgb_led_set(RGB_LED_OFF);
    ESP_LOGI(TAG, "RGB LED PWM initialised R=%d G=%d B=%d active_%s",
             red_pin, green_pin, blue_pin, active_high ? "high" : "low");
    return ESP_OK;
}

void rgb_led_set(rgb_led_color_t color)
{
    if (!s_initialized) return;

    uint16_t red = 0;
    uint16_t green = 0;
    uint16_t blue = 0;

    switch (color) {
        case RGB_LED_GREEN:
            green = RGB_LEDC_MAX_DUTY;
            break;
        case RGB_LED_RED:
            red = RGB_LEDC_MAX_DUTY;
            break;
        case RGB_LED_YELLOW:
            red = RGB_YELLOW_RED_DUTY;
            green = RGB_LEDC_MAX_DUTY;
            break;
        case RGB_LED_BLUE:
            blue = RGB_LEDC_MAX_DUTY;
            break;
        case RGB_LED_CYAN:
            green = 420;
            blue = RGB_LEDC_MAX_DUTY;
            break;
        case RGB_LED_IDLE:
            red = 120;
            green = 170;
            blue = 900;
            break;
        case RGB_LED_WHITE:
            red = 260;
            green = 320;
            blue = RGB_LEDC_MAX_DUTY;
            break;
        case RGB_LED_OFF:
        default:
            break;
    }

    set_channel(RGB_RED_CHANNEL, red);
    set_channel(RGB_GREEN_CHANNEL, green);
    set_channel(RGB_BLUE_CHANNEL, blue);
}

void rgb_led_deinit(void)
{
    if (!s_initialized) return;
    rgb_led_set(RGB_LED_OFF);
    ledc_stop(RGB_LEDC_MODE, RGB_RED_CHANNEL, output_duty(0));
    ledc_stop(RGB_LEDC_MODE, RGB_GREEN_CHANNEL, output_duty(0));
    ledc_stop(RGB_LEDC_MODE, RGB_BLUE_CHANNEL, output_duty(0));
    gpio_reset_pin(s_red_pin);
    gpio_reset_pin(s_green_pin);
    gpio_reset_pin(s_blue_pin);
    s_initialized = false;
}
