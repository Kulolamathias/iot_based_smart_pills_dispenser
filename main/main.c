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
#include <stdbool.h>
#include <stdlib.h>
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
#define HAND_WAIT_REMINDER_SEC  10       /* LCD/log reminder interval while waiting for hand */

/* ========================================================================= */
/* Global variables used by scheduler                                       */
/* ========================================================================= */
lcd_handle_t *s_lcd = NULL;
SemaphoreHandle_t s_lcd_mutex = NULL;

static float s_last_distance = -1.0f;
static char s_last_time_str[20] = "";
static int s_last_second = -1;

typedef enum {
    LOCAL_UI_IDLE = 0,
    LOCAL_UI_MED_SLOT,
    LOCAL_UI_TIME,
    LOCAL_UI_DOSE_COUNT,
    LOCAL_UI_CONFIRM,
    LOCAL_UI_DONE
} local_ui_state_t;

typedef struct {
    local_ui_state_t state;
    char input[8];
    size_t input_len;
    int med_slot;
    int hour;
    int minute;
    int dose_count;
    int message_ticks;
} local_schedule_ui_t;

static local_schedule_ui_t s_local_ui = {
    .state = LOCAL_UI_IDLE
};

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

static void lcd_line(uint8_t row, const char *text)
{
    if (!s_lcd || !s_lcd_mutex) return;

    xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
    lcd_set_cursor(s_lcd, row, 0);
    lcd_printf(s_lcd, "%-20.20s", text);
    xSemaphoreGive(s_lcd_mutex);
}

static void local_ui_clear_input(void)
{
    s_local_ui.input_len = 0;
    s_local_ui.input[0] = '\0';
}

static void local_ui_show(void)
{
    char line[32];

    switch (s_local_ui.state) {
        case LOCAL_UI_MED_SLOT:
            lcd_line(0, "Register dose");
            lcd_line(1, "Medicine slot 01-20");
            snprintf(line, sizeof(line), "Slot: %s_", s_local_ui.input);
            lcd_line(2, line);
            lcd_line(3, "#=OK *=Cancel");
            break;

        case LOCAL_UI_TIME:
            lcd_line(0, "Dose time");
            lcd_line(1, "Enter HHMM 24-hour");
            snprintf(line, sizeof(line), "Time: %s_", s_local_ui.input);
            lcd_line(2, line);
            lcd_line(3, "#=OK B=Back");
            break;

        case LOCAL_UI_DOSE_COUNT:
            lcd_line(0, "How many doses?");
            lcd_line(1, "Usually chambers used");
            snprintf(line, sizeof(line), "Count: %s_", s_local_ui.input);
            lcd_line(2, line);
            lcd_line(3, "#=OK B=Back");
            break;

        case LOCAL_UI_CONFIRM:
            lcd_line(0, "Confirm schedule");
            snprintf(line, sizeof(line), "MED%02d %02d:%02d", s_local_ui.med_slot,
                     s_local_ui.hour, s_local_ui.minute);
            lcd_line(1, line);
            snprintf(line, sizeof(line), "%d daily dose(s)", s_local_ui.dose_count);
            lcd_line(2, line);
            lcd_line(3, "A=Save B=Back *=No");
            break;

        case LOCAL_UI_DONE:
            lcd_line(0, "Local schedule");
            lcd_line(1, "Saved successfully");
            lcd_line(2, "Works offline now");
            lcd_line(3, "Returning...");
            break;

        case LOCAL_UI_IDLE:
        default:
            break;
    }
}

static void local_ui_start(void)
{
    memset(&s_local_ui, 0, sizeof(s_local_ui));
    s_local_ui.state = LOCAL_UI_MED_SLOT;
    local_ui_show();
}

static void local_ui_cancel(void)
{
    memset(&s_local_ui, 0, sizeof(s_local_ui));
    lcd_line(0, "Local schedule");
    lcd_line(1, "Cancelled");
    lcd_line(2, "");
    lcd_line(3, "");
    vTaskDelay(pdMS_TO_TICKS(800));
}

static bool local_ui_add_digit(char key, size_t max_len)
{
    if (key < '0' || key > '9') return false;
    if (s_local_ui.input_len >= max_len) return false;

    s_local_ui.input[s_local_ui.input_len++] = key;
    s_local_ui.input[s_local_ui.input_len] = '\0';
    return true;
}

static int local_ui_input_int(void)
{
    if (s_local_ui.input_len == 0) return -1;
    return atoi(s_local_ui.input);
}

static void local_ui_save_schedule(void)
{
    struct tm now;
    char json[192];
    esp_err_t ret;

    time_service_get_tm(&now);
    snprintf(json, sizeof(json),
             "[{\"name\":\"MED%02d\",\"times\":[\"%02d:%02d\"],"
             "\"duration_days\":%d,\"total_pills\":%d,"
             "\"start_date\":\"%04d-%02d-%02d\"}]",
             s_local_ui.med_slot,
             s_local_ui.hour,
             s_local_ui.minute,
             s_local_ui.dose_count + 1,
             s_local_ui.dose_count,
             now.tm_year + 1900,
             now.tm_mon + 1,
             now.tm_mday);

    ret = scheduler_set_schedule(json);
    if (ret == ESP_OK) {
        mqtt_manager_publish_log("schedule_updated", NULL, -1, "saved from keypad");
        s_local_ui.state = LOCAL_UI_DONE;
        s_local_ui.message_ticks = 20;
        local_ui_show();
    } else {
        lcd_line(0, "Save failed");
        lcd_line(1, "Try again");
        lcd_line(2, "");
        lcd_line(3, "B=Back *=Cancel");
    }
}

static bool local_ui_handle_key(char key)
{
    int value;

    if (s_local_ui.state == LOCAL_UI_IDLE) return false;

    if (key == '*') {
        local_ui_cancel();
        return true;
    }

    switch (s_local_ui.state) {
        case LOCAL_UI_MED_SLOT:
            if (local_ui_add_digit(key, 2)) {
                local_ui_show();
            } else if (key == '#') {
                value = local_ui_input_int();
                if (value >= 1 && value <= CAROUSEL_CHAMBERS) {
                    s_local_ui.med_slot = value;
                    local_ui_clear_input();
                    s_local_ui.state = LOCAL_UI_TIME;
                } else {
                    lcd_line(2, "Use 01 to 20");
                    vTaskDelay(pdMS_TO_TICKS(700));
                }
                local_ui_show();
            }
            break;

        case LOCAL_UI_TIME:
            if (local_ui_add_digit(key, 4)) {
                local_ui_show();
            } else if (key == 'B') {
                local_ui_clear_input();
                s_local_ui.state = LOCAL_UI_MED_SLOT;
                local_ui_show();
            } else if (key == '#') {
                if (s_local_ui.input_len == 4) {
                    int hh = (s_local_ui.input[0] - '0') * 10 + (s_local_ui.input[1] - '0');
                    int mm = (s_local_ui.input[2] - '0') * 10 + (s_local_ui.input[3] - '0');
                    if (hh >= 0 && hh <= 23 && mm >= 0 && mm <= 59) {
                        s_local_ui.hour = hh;
                        s_local_ui.minute = mm;
                        local_ui_clear_input();
                        s_local_ui.state = LOCAL_UI_DOSE_COUNT;
                    } else {
                        lcd_line(2, "Invalid time");
                        vTaskDelay(pdMS_TO_TICKS(700));
                    }
                } else {
                    lcd_line(2, "Need 4 digits");
                    vTaskDelay(pdMS_TO_TICKS(700));
                }
                local_ui_show();
            }
            break;

        case LOCAL_UI_DOSE_COUNT:
            if (local_ui_add_digit(key, 2)) {
                local_ui_show();
            } else if (key == 'B') {
                local_ui_clear_input();
                s_local_ui.state = LOCAL_UI_TIME;
                local_ui_show();
            } else if (key == '#') {
                value = local_ui_input_int();
                if (value >= 1 && value <= CAROUSEL_CHAMBERS) {
                    s_local_ui.dose_count = value;
                    local_ui_clear_input();
                    s_local_ui.state = LOCAL_UI_CONFIRM;
                } else {
                    lcd_line(2, "Use 01 to 20");
                    vTaskDelay(pdMS_TO_TICKS(700));
                }
                local_ui_show();
            }
            break;

        case LOCAL_UI_CONFIRM:
            if (key == 'A' || key == '#') {
                local_ui_save_schedule();
            } else if (key == 'B') {
                s_local_ui.state = LOCAL_UI_DOSE_COUNT;
                local_ui_clear_input();
                local_ui_show();
            }
            break;

        case LOCAL_UI_DONE:
            break;

        case LOCAL_UI_IDLE:
        default:
            break;
    }

    return true;
}

static bool local_ui_active(void)
{
    return s_local_ui.state != LOCAL_UI_IDLE;
}

/* ========================================================================= */
/* Distance measurement callback (called by timer every 2 seconds)          */
/* ========================================================================= */
static void distance_measure_cb(void *arg)
{
    (void)arg;
    if (local_ui_active()) return;

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
    ESP_ERROR_CHECK(scheduler_init(STEPS_PER_CHAMBER, HAND_WAIT_REMINDER_SEC));
    ESP_LOGI(TAG, "Scheduler initialised: %d steps/chamber, %d sec hand reminder",
             STEPS_PER_CHAMBER, HAND_WAIT_REMINDER_SEC);

    /* 9. Main loop */
    char key;
    while (1) {
        if (s_local_ui.state == LOCAL_UI_DONE) {
            if (s_local_ui.message_ticks > 0) {
                s_local_ui.message_ticks--;
            } else {
                memset(&s_local_ui, 0, sizeof(s_local_ui));
                lcd_clear(s_lcd);
                lcd_printf(s_lcd, "Pill Dispenser");
                lcd_set_cursor(s_lcd, 1, 0);
                lcd_printf(s_lcd, "System Ready");
            }
        }

        if (!local_ui_active()) {
            update_lcd_time();

            /* Update pending dose count on LCD line 3 */
            int pending = scheduler_get_pending_count();
            xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
            lcd_set_cursor(s_lcd, 3, 0);
            lcd_printf(s_lcd, "Pending: %d    ", pending);
            xSemaphoreGive(s_lcd_mutex);
        }

        /* Check for keypad input */
        if (keypad_get_key(&key)) {
            ESP_LOGI(TAG, "Key pressed: %c", key);

            if (local_ui_handle_key(key)) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

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
            } else if (key == 'D') {
                local_ui_start();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
