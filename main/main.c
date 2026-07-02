/**
 * @file main.c
 * @brief Smart Pill Dispenser – fully integrated with RTC, LCD, stepper,
 *        ultrasonic, keypad, WiFi, MQTT, and scheduler.
 * 
 * This is the main entry point. It initialises all components, sets up
 * callbacks for WiFi, MQTT, and keypad, and runs the main loop.
 * 
 * @author Matthithyahu
 * @date 2026-05-20
 * @version 2.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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

/* ========================================================================= */
/* Hardware pin definitions (adjust to your wiring)                         */
/* ========================================================================= */
#define I2C_MASTER_SCL_IO      6
#define I2C_MASTER_SDA_IO      7
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ     100000
#define LCD_I2C_ADDR            0x27
#define LCD_COLS                20
#define LCD_ROWS                4

#define ULTRASONIC_TRIG_PIN     8
#define ULTRASONIC_ECHO_PIN     18
#define ULTRASONIC_TIMEOUT_US   30000

#define STEP_PIN                5
#define DIR_PIN                 4
#define ENABLE_PIN              21
#define STEP_DELAY_US           8000

#define KEYPAD_ROW_PINS         {1, 2, 42, 41}
#define KEYPAD_COL_PINS         {40, 39, 38, 3}

#define WIFI_SSID               "Mathias' Sxx U..."
#define WIFI_PASSWORD           "1234567890223"
#define MQTT_BROKER_URI         "mqtt://102.223.8.140:1883"
#define MQTT_USERNAME           "mqtt_user"
#define MQTT_PASSWORD           "ega12345"

#define MOTOR_FULL_STEPS_PER_REV 200
#define CAROUSEL_CHAMBERS       20
#define STEPS_PER_CHAMBER       (MOTOR_FULL_STEPS_PER_REV / CAROUSEL_CHAMBERS)
#define HAND_WAIT_TIMEOUT_SEC   10       /* Seconds to wait for hand */

/* ========================================================================= */
/* Global variables used by scheduler                                       */
/* ========================================================================= */
lcd_handle_t *s_lcd = NULL;
SemaphoreHandle_t s_lcd_mutex = NULL;

static float s_last_distance = -1.0f;
static char s_last_time_str[20] = "";
static int s_last_second = -1;

/* ========================================================================= */
/* Helper: update LCD time (only when second changes)                       */
/* ========================================================================= */
static void update_lcd_time(void)
{
    struct tm now;
    time_service_get_tm(&now);   /* Returns local time (Tanzania, UTC+3) */
    int current_second = now.tm_sec;
    if (current_second != s_last_second) {
        s_last_second = current_second;
        char time_str[20];
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &now);
        if (strcmp(time_str, s_last_time_str) != 0) {
            strcpy(s_last_time_str, time_str);
            xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
            lcd_set_cursor(s_lcd, 0, 0);
            lcd_printf(s_lcd, "%s", time_str);
            xSemaphoreGive(s_lcd_mutex);
        }
    }
}

/* ========================================================================= */
/* Distance measurement callback (called by timer every 2 seconds)          */
/* ========================================================================= */
static void distance_measure_cb(void *arg)
{
    (void)arg;
    float dist = ultrasonic_measure_blocking();
    if (dist != s_last_distance) {
        s_last_distance = dist;
        xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
        lcd_set_cursor(s_lcd, 2, 0);
        if (dist > 0.1f) {
            lcd_printf(s_lcd, "Distance: %.1f cm   ", dist);
        } else {
            lcd_printf(s_lcd, "Distance: out of range");
        }
        xSemaphoreGive(s_lcd_mutex);
        ESP_LOGI(TAG, "Distance: %.1f cm", dist);
    }
}

/* ========================================================================= */
/* MQTT callbacks                                                           */
/* ========================================================================= */
static void mqtt_command_cb(const char *action, const char *payload, void *user_data)
{
    (void)user_data;
    ESP_LOGI(TAG, "MQTT command: action=%s", action);
    if (strcmp(action, "dispense_now") == 0) {
        scheduler_dispense_now();
    } else if (strcmp(action, "advance_chamber") == 0) {
        scheduler_calibration_move_steps(STEPS_PER_CHAMBER);
    } else if (strcmp(action, "motor_rev_test") == 0) {
        scheduler_calibration_move_steps(MOTOR_FULL_STEPS_PER_REV);
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
    (void)user_data;
    ESP_LOGI(TAG, "New schedule received");
    esp_err_t ret = scheduler_set_schedule(schedule_json);
    if (ret == ESP_OK) {
        mqtt_manager_publish_log("schedule_updated", NULL, -1, "schedule stored");
        /* Show confirmation on LCD */
        xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
        lcd_set_cursor(s_lcd, 3, 0);
        lcd_printf(s_lcd, "Schedule updated ");
        xSemaphoreGive(s_lcd_mutex);
        vTaskDelay(pdMS_TO_TICKS(1500));
    } else {
        ESP_LOGE(TAG, "Failed to set schedule");
        mqtt_manager_publish_log("error", NULL, -1, "invalid schedule JSON");
        xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
        lcd_set_cursor(s_lcd, 3, 0);
        lcd_printf(s_lcd, "Invalid schedule ");
        xSemaphoreGive(s_lcd_mutex);
        vTaskDelay(pdMS_TO_TICKS(1500));
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

/* ========================================================================= */
/* Main application entry point                                             */
/* ========================================================================= */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting Smart Pill Dispenser v2.0");

    /* Create LCD mutex */
    s_lcd_mutex = xSemaphoreCreateMutex();
    if (!s_lcd_mutex) {
        ESP_LOGE(TAG, "Failed to create LCD mutex");
        return;
    }

    /* 1. I2C for LCD */
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

    /* 2. Software RTC (UTC) with Tanzania timezone (UTC+3) */
    ESP_ERROR_CHECK(time_service_init());
    time_service_set_timezone(3);   /* Tanzania */
    struct tm init_tm = {
        .tm_year = 126, .tm_mon = 4, .tm_mday = 20,
        .tm_hour = 12, .tm_min = 0, .tm_sec = 0
    };
    time_t init_ts = mktime(&init_tm);
    time_service_set_timestamp(init_ts);
    ESP_LOGI(TAG, "Software RTC initialised (UTC+3 display)");

    /* 3. LCD */
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

    /* 4. Ultrasonic sensor */
    ESP_ERROR_CHECK(ultrasonic_init(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, ULTRASONIC_TIMEOUT_US));
    const esp_timer_create_args_t timer_args = {
        .callback = distance_measure_cb,
        .name = "dist_timer"
    };
    esp_timer_handle_t dist_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &dist_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(dist_timer, 2 * 1000 * 1000));
    ESP_LOGI(TAG, "Ultrasonic started");

    /* 5. Stepper motor */
    stepper_config_t stepper_cfg = {
        .step_pin = STEP_PIN,
        .dir_pin = DIR_PIN,
        .enable_pin = ENABLE_PIN,
        .step_delay_us = STEP_DELAY_US,
        .enable_on_init = false
    };
    ESP_ERROR_CHECK(stepper_motor_init(&stepper_cfg));
    ESP_LOGI(TAG, "Stepper motor initialised: %d full steps/rev, %d chambers, %d steps/chamber",
             MOTOR_FULL_STEPS_PER_REV, CAROUSEL_CHAMBERS, STEPS_PER_CHAMBER);

    /* 6. Keypad */
    gpio_num_t row_pins[4] = KEYPAD_ROW_PINS;
    gpio_num_t col_pins[4] = KEYPAD_COL_PINS;
    ESP_ERROR_CHECK(keypad_init(row_pins, col_pins));
    ESP_ERROR_CHECK(keypad_start());
    ESP_LOGI(TAG, "Keypad initialised");

    /* 7. WiFi & MQTT */
    ESP_ERROR_CHECK(wifi_manager_init(WIFI_SSID, WIFI_PASSWORD));
    wifi_manager_register_connected_cb(wifi_connected_cb);
    wifi_manager_start();

    ESP_ERROR_CHECK(mqtt_manager_init(MQTT_BROKER_URI, MQTT_USERNAME, MQTT_PASSWORD));
    mqtt_manager_register_connected_cb(mqtt_connected_cb);
    mqtt_manager_register_command_cb(mqtt_command_cb, NULL);
    mqtt_manager_register_schedule_cb(mqtt_schedule_cb, NULL);

    /* 8. Scheduler */
    ESP_ERROR_CHECK(scheduler_init(STEPS_PER_CHAMBER, HAND_WAIT_TIMEOUT_SEC));
    ESP_LOGI(TAG, "Scheduler initialised: %d steps/chamber, %d sec hand wait",
             STEPS_PER_CHAMBER, HAND_WAIT_TIMEOUT_SEC);

    /* 9. Main loop */
    char key;
    while (1) {
        update_lcd_time();

        /* Update pending dose count on LCD line 3 */
        int pending = scheduler_get_pending_count();
        xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
        lcd_set_cursor(s_lcd, 3, 0);
        lcd_printf(s_lcd, "Pending: %d    ", pending);
        xSemaphoreGive(s_lcd_mutex);

        /* Check for keypad input */
        if (keypad_get_key(&key)) {
            ESP_LOGI(TAG, "Key pressed: %c", key);
            /* Show key momentarily */
            xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
            lcd_set_cursor(s_lcd, 3, 0);
            lcd_printf(s_lcd, "Key: %c          ", key);
            xSemaphoreGive(s_lcd_mutex);
            vTaskDelay(pdMS_TO_TICKS(500));

            if (key == 'A') {
                scheduler_dispense_now();
            } else if (key == 'B') {
                scheduler_calibration_move_steps(STEPS_PER_CHAMBER);
            } else if (key == 'C') {
                scheduler_calibration_move_steps(MOTOR_FULL_STEPS_PER_REV);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
