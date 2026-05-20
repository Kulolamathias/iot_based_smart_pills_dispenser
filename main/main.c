#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "time_service.h"
#include "lcd_i2c.h"
#include "ultrasonic.h"
#include "stepper_motor.h"
#include "keypad.h"

static const char *TAG = "MAIN";

// ===== I2C for LCD =====
#define I2C_MASTER_SCL_IO      7
#define I2C_MASTER_SDA_IO      6
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ     100000
#define LCD_I2C_ADDR            0x27
#define LCD_COLS                20
#define LCD_ROWS                4

// ===== Ultrasonic =====
#define ULTRASONIC_TRIG_PIN     15
#define ULTRASONIC_ECHO_PIN     16
#define ULTRASONIC_TIMEOUT_US   30000

// ===== Stepper (A4988) =====
#define STEP_PIN                3
#define DIR_PIN                 4
#define ENABLE_PIN              5
#define MS1_PIN                 9
#define MS2_PIN                 10
#define MS3_PIN                 11
#define MICROSTEP_MODE          STEP_SIXTEENTH
#define STEP_DELAY_US           1000

// ===== Keypad 4x4 =====
#define KEYPAD_ROW_PINS         {1, 2, 42, 41}
#define KEYPAD_COL_PINS         {40, 39, 38, 37}

static lcd_handle_t *s_lcd = NULL;
static float s_last_distance = -1.0f;
static char s_last_time_str[20] = "";
static int s_last_second = -1;

// Distance measurement callback (runs in timer context, not ISR)
static void measure_distance_cb(void *arg)
{
    float dist = ultrasonic_measure_blocking();
    if (dist != s_last_distance) {
        s_last_distance = dist;
        if (s_lcd) {
            lcd_set_cursor(s_lcd, 2, 0);
            if (dist > 0.1f) {
                lcd_printf(s_lcd, "Distance: %.1f cm   ", dist);
            } else {
                lcd_printf(s_lcd, "Distance: out of range");
            }
        }
        ESP_LOGI(TAG, "Distance: %.1f cm", dist);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Integrated Test");

    // 1. I2C for LCD
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    // 2. Software RTC
    ESP_ERROR_CHECK(time_service_init());
    struct tm init_tm = { .tm_year = 126, .tm_mon = 4, .tm_mday = 20,
                          .tm_hour = 12, .tm_min = 0, .tm_sec = 0 };
    time_service_set_timestamp(mktime(&init_tm));

    // 3. LCD
    lcd_config_t lcd_cfg = {
        .i2c_port = I2C_MASTER_NUM,
        .i2c_addr = LCD_I2C_ADDR,
        .rows = LCD_ROWS,
        .cols = LCD_COLS,
        .backlight_enable = true,
        .i2c_timeout_ms = 1000,
        .cmd_delay_us = 50
    };
    s_lcd = lcd_i2c_init(&lcd_cfg);
    if (!s_lcd) {
        ESP_LOGE(TAG, "LCD init failed");
        return;
    }
    lcd_clear(s_lcd);
    lcd_printf(s_lcd, "Pill Dispenser");
    lcd_set_cursor(s_lcd, 1, 0);
    lcd_printf(s_lcd, "System Ready");

    // 4. Ultrasonic
    ESP_ERROR_CHECK(ultrasonic_init(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, ULTRASONIC_TIMEOUT_US));

    // Start a periodic timer to measure distance every 2 seconds
    const esp_timer_create_args_t timer_args = {
        .callback = measure_distance_cb,
        .name = "dist_timer"
    };
    esp_timer_handle_t dist_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &dist_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(dist_timer, 2 * 1000 * 1000)); // 2 seconds

    // 5. Stepper motor
    stepper_config_t stepper_cfg = {
        .step_pin = STEP_PIN,
        .dir_pin = DIR_PIN,
        .enable_pin = ENABLE_PIN,
        .ms1_pin = MS1_PIN,
        .ms2_pin = MS2_PIN,
        .ms3_pin = MS3_PIN,
        .microstep = MICROSTEP_MODE,
        .step_delay_us = STEP_DELAY_US,
        .enable_on_init = true
    };
    ESP_ERROR_CHECK(stepper_motor_init(&stepper_cfg));

    // 6. Keypad
    gpio_num_t row_pins[4] = KEYPAD_ROW_PINS;
    gpio_num_t col_pins[4] = KEYPAD_COL_PINS;
    ESP_ERROR_CHECK(keypad_init(row_pins, col_pins));
    ESP_ERROR_CHECK(keypad_start());

    // 7. Main loop – only update time when second changes
    char key;
    while (1) {
        struct tm now;
        time_service_get_tm(&now);
        int current_second = now.tm_sec;
        if (current_second != s_last_second) {
            s_last_second = current_second;
            char time_str[20];
            strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &now);
            if (strcmp(time_str, s_last_time_str) != 0) {
                strcpy(s_last_time_str, time_str);
                lcd_set_cursor(s_lcd, 0, 0);
                lcd_printf(s_lcd, "Time: %s", time_str);
            }
        }

        // Check keypad (non‑blocking)
        if (keypad_get_key(&key)) {
            ESP_LOGI(TAG, "Key pressed: %c", key);
            lcd_set_cursor(s_lcd, 3, 0);
            lcd_printf(s_lcd, "Key: %c          ", key);

            // Example: rotate stepper on 'A'
            if (key == 'A') {
                uint32_t steps_per_rev = 48;
                switch (MICROSTEP_MODE) {
                    case STEP_FULL:      steps_per_rev *= 1; break;
                    case STEP_HALF:      steps_per_rev *= 2; break;
                    case STEP_QUARTER:   steps_per_rev *= 4; break;
                    case STEP_EIGHTH:    steps_per_rev *= 8; break;
                    case STEP_SIXTEENTH: steps_per_rev *= 16; break;
                }
                ESP_LOGI(TAG, "Rotating stepper %lu steps", steps_per_rev);
                stepper_motor_set_direction(true);
                stepper_motor_rotate_steps(steps_per_rev);
                vTaskDelay(pdMS_TO_TICKS(500));
                stepper_motor_set_direction(false);
                stepper_motor_rotate_steps(steps_per_rev);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // short delay, no LCD flicker
    }
}