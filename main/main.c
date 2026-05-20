#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "time_service.h"
#include "lcd_i2c.h"
#include "ultrasonic.h"
#include "stepper_motor.h"
#include "keypad.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"

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

// ===== Stepper (A4988) – moved STEP to safe pin 17 (avoid GPIO3 strapping) =====
#define STEP_PIN                17   // was 3 (strapping pin)
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

// ===== WiFi & MQTT Credentials (from smart bin project) =====
#define WIFI_SSID               "Mathias' Sxx U..."
#define WIFI_PASSWORD           "1234567890223"
#define MQTT_BROKER_URI         "mqtt://102.223.8.140:1883"
#define MQTT_USERNAME           "mqtt_user"
#define MQTT_PASSWORD           "ega12345"

static lcd_handle_t *s_lcd = NULL;
static float s_last_distance = -1.0f;
static char s_last_time_str[20] = "";
static int s_last_second = -1;

// Device ID (MAC) will be obtained from wifi_manager or directly
static char s_device_id[18] = {0};

// MQTT topics base
static char s_base_topic[64] = {0};

// Forward declarations
static void update_lcd_time(void);
static void distance_measure_cb(void *arg);
static void mqtt_message_handler(const char *topic, const char *payload, void *user_data);
static void mqtt_connected_cb(void);
static void wifi_connected_cb(void);

// ===== Utility: get device ID from MAC =====
static void get_device_id(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_device_id, sizeof(s_device_id), "%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    snprintf(s_base_topic, sizeof(s_base_topic), "smartpill/dispenser/%s", s_device_id);
    ESP_LOGI(TAG, "Device ID: %s, base topic: %s", s_device_id, s_base_topic);
}

// ===== LCD update only when second changes =====
static void update_lcd_time(void)
{
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
}

// ===== Distance measurement callback (timer) =====
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

// ===== MQTT message handler =====
static void mqtt_message_handler(const char *topic, const char *payload, void *user_data)
{
    ESP_LOGI(TAG, "MQTT message: %s -> %s", topic, payload);
    // TODO: parse JSON and handle commands like "dispense_now", "set_schedule"
    // For now, just log.
}

// ===== MQTT connected callback: subscribe to command and schedule topics =====
static void mqtt_connected_cb(void)
{
    char topic[128];
    snprintf(topic, sizeof(topic), "%s/command", s_base_topic);
    mqtt_manager_subscribe(topic);
    snprintf(topic, sizeof(topic), "%s/schedule", s_base_topic);
    mqtt_manager_subscribe(topic);
    ESP_LOGI(TAG, "Subscribed to %s/command and %s/schedule", s_base_topic, s_base_topic);
    // Publish online status
    snprintf(topic, sizeof(topic), "%s/status", s_base_topic);
    mqtt_manager_publish(topic, "online", true);
}

// ===== WiFi connected callback: start MQTT =====
static void wifi_connected_cb(void)
{
    ESP_LOGI(TAG, "WiFi connected, starting MQTT");
    mqtt_manager_start();
}

// ===== Main =====
void app_main(void)
{
    ESP_LOGI(TAG, "Starting Integrated Pill Dispenser");

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
    const esp_timer_create_args_t timer_args = {
        .callback = distance_measure_cb,
        .name = "dist_timer"
    };
    esp_timer_handle_t dist_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &dist_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(dist_timer, 2 * 1000 * 1000));

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

    // 7. WiFi & MQTT
    get_device_id();   // sets s_device_id and s_base_topic
    wifi_manager_init(WIFI_SSID, WIFI_PASSWORD);
    wifi_manager_register_connected_cb(wifi_connected_cb);

    mqtt_manager_init(MQTT_BROKER_URI, MQTT_USERNAME, MQTT_PASSWORD);
    mqtt_manager_register_connected_cb(mqtt_connected_cb);
    mqtt_manager_register_message_cb(mqtt_message_handler, NULL);

    wifi_manager_start();   // starts WiFi connection; on connect, MQTT will start

    // 8. Main loop: update time, handle keypad, and optionally stepper demo
    char key;
    while (1) {
        update_lcd_time();

        if (keypad_get_key(&key)) {
            ESP_LOGI(TAG, "Key pressed: %c", key);
            lcd_set_cursor(s_lcd, 3, 0);
            lcd_printf(s_lcd, "Key: %c          ", key);

            // Example: press 'A' to dispense one full rotation
            if (key == 'A') {
                uint32_t steps_per_rev = 48;
                switch (MICROSTEP_MODE) {
                    case STEP_FULL:      steps_per_rev *= 1; break;
                    case STEP_HALF:      steps_per_rev *= 2; break;
                    case STEP_QUARTER:   steps_per_rev *= 4; break;
                    case STEP_EIGHTH:    steps_per_rev *= 8; break;
                    case STEP_SIXTEENTH: steps_per_rev *= 16; break;
                }
                ESP_LOGI(TAG, "Dispensing: %lu steps", steps_per_rev);
                stepper_motor_set_direction(true);
                stepper_motor_rotate_steps(steps_per_rev);
                vTaskDelay(pdMS_TO_TICKS(500));
                stepper_motor_set_direction(false);
                stepper_motor_rotate_steps(steps_per_rev);
                // Publish log via MQTT
                char topic[128];
                snprintf(topic, sizeof(topic), "%s/log", s_base_topic);
                mqtt_manager_publish(topic, "Dispensed manually (key A)", false);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}