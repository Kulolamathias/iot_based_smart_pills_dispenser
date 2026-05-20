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
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "scheduler.h"

static const char *TAG = "MAIN";

// ========== I2C for LCD ==========
#define I2C_MASTER_SCL_IO      7
#define I2C_MASTER_SDA_IO      6
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ     100000
#define LCD_I2C_ADDR            0x27
#define LCD_COLS                20
#define LCD_ROWS                4

// ========== Ultrasonic ==========
#define ULTRASONIC_TRIG_PIN     15
#define ULTRASONIC_ECHO_PIN     16
#define ULTRASONIC_TIMEOUT_US   30000

// ========== Stepper (A4988) – STEP moved to safe GPIO17 ==========
#define STEP_PIN                17
#define DIR_PIN                 4
#define ENABLE_PIN              5
#define MS1_PIN                 9
#define MS2_PIN                 10
#define MS3_PIN                 11
#define MICROSTEP_MODE          STEP_SIXTEENTH
#define STEP_DELAY_US           1000

// ========== Keypad 4x4 ==========
#define KEYPAD_ROW_PINS         {1, 2, 42, 41}
#define KEYPAD_COL_PINS         {40, 39, 38, 37}

// ========== WiFi & MQTT Credentials ==========
#define WIFI_SSID               "Mathias' Sxx U..."
#define WIFI_PASSWORD           "1234567890223"
#define MQTT_BROKER_URI         "mqtt://102.223.8.140:1883"
#define MQTT_USERNAME           "mqtt_user"
#define MQTT_PASSWORD           "ega12345"

// ========== Scheduler Configuration ==========
// Assuming carousel has 4 chambers; full revolution = 48 steps * 16 microsteps = 768 steps
#define STEPS_PER_CHAMBER       192      // 768 / 4
#define HAND_WAIT_TIMEOUT_SEC   10       // seconds to wait for hand after unlocking

static lcd_handle_t *s_lcd = NULL;
static float s_last_distance = -1.0f;
static char s_last_time_str[20] = "";
static int s_last_second = -1;

// ========== Helper: update LCD time (only when second changes) ==========
static void update_lcd_time(void)
{
    struct tm now;
    time_service_get_tm(&now);   // returns LOCAL time (with offset)
    int current_second = now.tm_sec;
    if (current_second != s_last_second) {
        s_last_second = current_second;
        char time_str[20];
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &now);
        if (strcmp(time_str, s_last_time_str) != 0) {
            strcpy(s_last_time_str, time_str);
            lcd_set_cursor(s_lcd, 0, 0);
            lcd_printf(s_lcd, "%s", time_str);   // no prefix
        }
    }
}

// ========== Distance measurement callback (timer) ==========
static void distance_measure_cb(void *arg)
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

// ========== MQTT Callbacks ==========
static void mqtt_command_cb(const char *action, const char *payload, void *user_data)
{
    ESP_LOGI(TAG, "MQTT command: action=%s, payload=%s", action, payload);
    if (strcmp(action, "dispense_now") == 0) {
        scheduler_dispense_now();
        mqtt_manager_publish_log("dispensed", "manual", -1, "MQTT command");
    } else if (strcmp(action, "reboot") == 0) {
        ESP_LOGI(TAG, "Rebooting in 1 second...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    } else if (strcmp(action, "clear_schedule") == 0) {
        scheduler_clear();
        mqtt_manager_publish_log("schedule_updated", NULL, -1, "cleared by command");
    } else {
        ESP_LOGW(TAG, "Unknown action: %s", action);
    }
}

static void mqtt_schedule_cb(const char *schedule_json, void *user_data)
{
    ESP_LOGI(TAG, "New schedule received: %s", schedule_json);
    ESP_LOG_BUFFER_HEXDUMP(TAG, schedule_json, strlen(schedule_json), ESP_LOG_INFO);
    esp_err_t ret = scheduler_set_schedule(schedule_json);
    if (ret == ESP_OK) {
        mqtt_manager_publish_log("schedule_updated", NULL, -1, "schedule stored");
    } else {
        ESP_LOGE(TAG, "Failed to parse schedule");
        mqtt_manager_publish_log("error", NULL, -1, "invalid schedule JSON");
    }
}

static void mqtt_connected_cb(void)
{
    ESP_LOGI(TAG, "MQTT connected, publishing online status");
    mqtt_manager_publish_status("online");
}

static void wifi_connected_cb(void)
{
    ESP_LOGI(TAG, "WiFi connected, starting MQTT and NTP sync");
    mqtt_manager_start();
    time_service_sync_ntp();
}

// ========== Main ==========
void app_main(void)
{
    ESP_LOGI(TAG, "Starting Integrated Pill Dispenser with Scheduler");

    // ---------- 1. I2C for LCD ----------
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

    // ---------- 2. Software RTC ----------
    ESP_ERROR_CHECK(time_service_init());
    time_service_set_timezone(3);   // Tanzania (UTC+3)
    // Set initial dummy time (will be updated by NTP when WiFi connects)
    struct tm init_tm = {
        .tm_year = 126, .tm_mon = 4, .tm_mday = 20,
        .tm_hour = 12, .tm_min = 0, .tm_sec = 0
    };
    time_t init_ts = mktime(&init_tm);
    time_service_set_timestamp(init_ts);
    ESP_LOGI(TAG, "Software RTC initialised");

    // ---------- 3. LCD ----------
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

    // ---------- 4. Ultrasonic ----------
    ESP_ERROR_CHECK(ultrasonic_init(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, ULTRASONIC_TIMEOUT_US));
    const esp_timer_create_args_t timer_args = {
        .callback = distance_measure_cb,
        .name = "dist_timer"
    };
    esp_timer_handle_t dist_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &dist_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(dist_timer, 2 * 1000 * 1000));
    ESP_LOGI(TAG, "Ultrasonic started");

    // ---------- 5. Stepper motor ----------
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
    ESP_LOGI(TAG, "Stepper motor initialised");

    // ---------- 6. Keypad ----------
    gpio_num_t row_pins[4] = KEYPAD_ROW_PINS;
    gpio_num_t col_pins[4] = KEYPAD_COL_PINS;
    ESP_ERROR_CHECK(keypad_init(row_pins, col_pins));
    ESP_ERROR_CHECK(keypad_start());
    ESP_LOGI(TAG, "Keypad initialised");

    // ---------- 7. WiFi & MQTT ----------
    ESP_ERROR_CHECK(wifi_manager_init(WIFI_SSID, WIFI_PASSWORD));
    wifi_manager_register_connected_cb(wifi_connected_cb);
    wifi_manager_start();

    ESP_ERROR_CHECK(mqtt_manager_init(MQTT_BROKER_URI, MQTT_USERNAME, MQTT_PASSWORD));
    mqtt_manager_register_connected_cb(mqtt_connected_cb);
    mqtt_manager_register_command_cb(mqtt_command_cb, NULL);
    mqtt_manager_register_schedule_cb(mqtt_schedule_cb, NULL);

    // ---------- 8. Scheduler ----------
    ESP_ERROR_CHECK(scheduler_init(STEPS_PER_CHAMBER, HAND_WAIT_TIMEOUT_SEC));
    ESP_LOGI(TAG, "Scheduler initialised with %d steps/chamber, hand wait %d sec",
             STEPS_PER_CHAMBER, HAND_WAIT_TIMEOUT_SEC);

    // ---------- 9. Main loop ----------
    char key;
    while (1) {
        // Update LCD time (only when second changes)
        update_lcd_time();

        // Show number of pending doses on LCD line 3 (optional)
        int pending = scheduler_get_pending_count();
        lcd_set_cursor(s_lcd, 3, 0);
        lcd_printf(s_lcd, "Pending: %d    ", pending);

        // Check keypad
        if (keypad_get_key(&key)) {
            ESP_LOGI(TAG, "Key pressed: %c", key);
            lcd_set_cursor(s_lcd, 3, 0);
            lcd_printf(s_lcd, "Key: %c          ", key);
            vTaskDelay(pdMS_TO_TICKS(500)); // show key for 0.5 sec
            lcd_set_cursor(s_lcd, 3, 0);
            lcd_printf(s_lcd, "Pending: %d    ", pending); // restore pending count

            // If key 'A' is pressed, manually dispense
            if (key == 'A') {
                scheduler_dispense_now();
                mqtt_manager_publish_log("dispensed", "manual", -1, "key A");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}