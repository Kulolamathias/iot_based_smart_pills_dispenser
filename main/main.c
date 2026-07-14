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
#include "led.h"
#include "buzzer.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "scheduler.h"

static const char *TAG = "MAIN";

/* ========================================================================= */
/* Hardware pin definitions (adjust to your wiring)                       =  */
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

#define BUZZER_PIN              12
#define RGB_LED_RED_PIN         15
#define RGB_LED_GREEN_PIN       16
#define RGB_LED_BLUE_PIN        17
#define RGB_LED_ACTIVE_HIGH     false

#define KEYPAD_ROW_PINS         {1, 2, 42, 41}
#define KEYPAD_COL_PINS         {40, 39, 38, 3}

#define WIFI_SSID               "Mathias' Sxx U..."
#define WIFI_PASSWORD           "1234567890223"

// #define WIFI_SSID               "chaz"
// #define WIFI_PASSWORD           "0123456789"

// #define WIFI_SSID              "Igman"
// #define WIFI_PASSWORD           "igman_0123"

#define MQTT_BROKER_URI         "mqtt://102.223.8.140:1883"
#define MQTT_USERNAME           "mqtt_user"
#define MQTT_PASSWORD           "ega12345"

#define MOTOR_FULL_STEPS_PER_REV       200
/* A4988 is wired for full-step mode: one STEP pulse is one motor full step. */
#define MOTOR_EFFECTIVE_STEPS_PER_REV  MOTOR_FULL_STEPS_PER_REV
#define CAROUSEL_CHAMBERS              20
#define STEPS_PER_CHAMBER              (MOTOR_EFFECTIVE_STEPS_PER_REV / CAROUSEL_CHAMBERS)

_Static_assert((MOTOR_EFFECTIVE_STEPS_PER_REV % CAROUSEL_CHAMBERS) == 0,
               "Carousel pulses per revolution must divide evenly by chamber count");
#define HAND_WAIT_REMINDER_SEC  10       /* LCD/log reminder interval while waiting for hand */
#define DOSE_NEAR_WINDOW_SEC    (2 * 60)
#define NEAR_BUZZ_INTERVAL_SEC  20
#define DUE_BUZZ_INTERVAL_SEC   10
#define LOCAL_TIME_OFFSET_SEC   (3 * 3600)
#define LCD_NOTICE_DURATION_MS  2500

/* ========================================================================= */
/* Global variables used by scheduler                                       */
/* ========================================================================= */
lcd_handle_t *s_lcd = NULL;
SemaphoreHandle_t s_lcd_mutex = NULL;

static int64_t s_last_near_buzz_us = 0;
static int64_t s_last_due_buzz_us = 0;
static bool s_output_test_active = false;
static bool s_mqtt_started = false;
static rgb_led_color_t s_last_alert_color = RGB_LED_OFF;
static char s_lcd_cache[LCD_ROWS][LCD_COLS + 1] = {{0}};
static bool s_lcd_cache_valid = false;
static char s_notice_lines[LCD_ROWS][LCD_COLS + 1] = {{0}};
static int64_t s_notice_until_us = 0;

static const char *alert_color_name(rgb_led_color_t color)
{
    switch (color) {
        case RGB_LED_GREEN: return "green";
        case RGB_LED_RED: return "red";
        case RGB_LED_YELLOW: return "yellow";
        case RGB_LED_BLUE: return "blue";
        case RGB_LED_CYAN: return "cyan";
        case RGB_LED_IDLE: return "idle-white";
        case RGB_LED_WHITE: return "white";
        case RGB_LED_OFF:
        default: return "off";
    }
}

static bool set_alert_color(rgb_led_color_t color, const char *reason)
{
    bool changed = s_last_alert_color != color;
    rgb_led_set(color);
    if (changed) {
        s_last_alert_color = color;
        ESP_LOGI(TAG, "RGB alert: %s (%s)", alert_color_name(color), reason);
    }
    return changed;
}

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

static void copy_lcd_text(char destination[LCD_COLS + 1], const char *text)
{
    snprintf(destination, LCD_COLS + 1, "%-20.20s", text ? text : "");
}

static void format_local_due_time(time_t utc_timestamp, char *buffer, size_t buffer_size)
{
    struct tm local_due;
    time_t local_timestamp = utc_timestamp + LOCAL_TIME_OFFSET_SEC;
    gmtime_r(&local_timestamp, &local_due);
    strftime(buffer, buffer_size, "%H:%M", &local_due);
}

static void show_lcd_notice(const char *line0, const char *line1,
                            const char *line2, const char *line3)
{
    if (!s_lcd_mutex) return;

    xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
    copy_lcd_text(s_notice_lines[0], line0);
    copy_lcd_text(s_notice_lines[1], line1);
    copy_lcd_text(s_notice_lines[2], line2);
    copy_lcd_text(s_notice_lines[3], line3);
    s_notice_until_us = esp_timer_get_time() + (LCD_NOTICE_DURATION_MS * 1000LL);
    s_lcd_cache_valid = false;
    xSemaphoreGive(s_lcd_mutex);
}

static bool get_lcd_notice(char lines[LCD_ROWS][LCD_COLS + 1])
{
    bool active = false;
    if (!s_lcd_mutex) return false;

    xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
    if (esp_timer_get_time() < s_notice_until_us) {
        memcpy(lines, s_notice_lines, sizeof(s_notice_lines));
        active = true;
    } else {
        s_notice_until_us = 0;
    }
    xSemaphoreGive(s_lcd_mutex);
    return active;
}

static void render_lcd_screen(char lines[LCD_ROWS][LCD_COLS + 1])
{
    if (!s_lcd || !s_lcd_mutex) return;

    xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
    for (uint8_t row = 0; row < LCD_ROWS; row++) {
        if (!s_lcd_cache_valid || strcmp(s_lcd_cache[row], lines[row]) != 0) {
            lcd_set_cursor(s_lcd, row, 0);
            lcd_printf(s_lcd, "%s", lines[row]);
            strlcpy(s_lcd_cache[row], lines[row], sizeof(s_lcd_cache[row]));
        }
    }
    s_lcd_cache_valid = true;
    xSemaphoreGive(s_lcd_mutex);
}

static void lcd_line(uint8_t row, const char *text)
{
    if (!s_lcd || !s_lcd_mutex) return;

    xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
    lcd_set_cursor(s_lcd, row, 0);
    lcd_printf(s_lcd, "%-20.20s", text);
    s_lcd_cache_valid = false;
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

    if (!time_service_is_synchronized()) {
        lcd_line(0, "Clock not ready");
        lcd_line(1, "Waiting for time sync");
        lcd_line(2, "Please try again");
        lcd_line(3, "B=Back *=Cancel");
        return;
    }

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

static void update_lcd_dashboard(void)
{
    scheduler_status_t status;
    char raw[LCD_ROWS][48] = {{0}};
    char lines[LCD_ROWS][LCD_COLS + 1] = {{0}};

    if (scheduler_get_status(&status) != ESP_OK) return;

    switch (status.activity) {
        case SCHEDULER_ACTIVITY_WAITING_FOR_HAND:
            snprintf(raw[0], sizeof(raw[0]), "DOSE READY NOW");
            snprintf(raw[1], sizeof(raw[1]), "%.31s", status.active_medicine);
            snprintf(raw[2], sizeof(raw[2]), "Place hand at outlet");
            if (status.hand_distance_cm > 7.0f) {
                snprintf(raw[3], sizeof(raw[3]), "Move closer: %.1f cm", status.hand_distance_cm);
            } else if (status.hand_distance_cm > 0.1f) {
                snprintf(raw[3], sizeof(raw[3]), "Hand detected");
            } else {
                snprintf(raw[3], sizeof(raw[3]), "Hold within 7 cm");
            }
            break;

        case SCHEDULER_ACTIVITY_DISPENSING:
            snprintf(raw[0], sizeof(raw[0]), "Dispensing medicine");
            snprintf(raw[1], sizeof(raw[1]), "%.31s", status.active_medicine);
            snprintf(raw[2], sizeof(raw[2]), "Please wait...");
            break;

        case SCHEDULER_ACTIVITY_SUCCESS:
            snprintf(raw[0], sizeof(raw[0]), "Dose dispensed");
            snprintf(raw[1], sizeof(raw[1]), "%.31s", status.active_medicine);
            snprintf(raw[2], sizeof(raw[2]), "Please take it now");
            snprintf(raw[3], sizeof(raw[3]), "Thank you");
            break;

        case SCHEDULER_ACTIVITY_ERROR:
            snprintf(raw[0], sizeof(raw[0]), "Dose not released");
            snprintf(raw[1], sizeof(raw[1]), "%.31s", status.active_medicine);
            snprintf(raw[2], sizeof(raw[2]), "Please seek help");
            snprintf(raw[3], sizeof(raw[3]), "Dose remains pending");
            break;

        case SCHEDULER_ACTIVITY_IDLE:
        default:
            break;
    }

    if (status.activity == SCHEDULER_ACTIVITY_IDLE && get_lcd_notice(lines)) {
        render_lcd_screen(lines);
        return;
    }

    if (status.activity == SCHEDULER_ACTIVITY_IDLE && !time_service_is_synchronized()) {
        snprintf(raw[0], sizeof(raw[0]), "Pill Dispenser");
        snprintf(raw[1], sizeof(raw[1]), "Setting clock...");
        snprintf(raw[2], sizeof(raw[2]), "Connecting service");
        snprintf(raw[3], sizeof(raw[3]), "Please wait");
    } else if (status.activity == SCHEDULER_ACTIVITY_IDLE) {
        struct tm now;
        time_t now_utc = 0;
        char due_time[8] = "";

        time_service_get_tm(&now);
        time_service_get_timestamp(&now_utc);
        strftime(raw[0], sizeof(raw[0]), "%d/%m/%Y %H:%M:%S", &now);

        if (!status.has_next_dose) {
            snprintf(raw[1], sizeof(raw[1]), "System ready");
            snprintf(raw[2], sizeof(raw[2]), "No doses scheduled");
            snprintf(raw[3], sizeof(raw[3]), "Press D to add dose");
        } else {
            int64_t seconds_to_due = (int64_t)(status.next_due_time - now_utc);
            format_local_due_time(status.next_due_time, due_time, sizeof(due_time));

            if (seconds_to_due <= 0) {
                snprintf(raw[1], sizeof(raw[1]), "Dose is due now");
                snprintf(raw[2], sizeof(raw[2]), "%.31s", status.next_medicine);
                snprintf(raw[3], sizeof(raw[3]), "Preparing dispenser");
            } else if (seconds_to_due <= DOSE_NEAR_WINDOW_SEC) {
                long minutes = (long)(seconds_to_due / 60);
                long seconds = (long)(seconds_to_due % 60);
                snprintf(raw[1], sizeof(raw[1]), "Dose in %02ld:%02ld", minutes, seconds);
                snprintf(raw[2], sizeof(raw[2]), "%.11s at %s", status.next_medicine, due_time);
                snprintf(raw[3], sizeof(raw[3]), "Please stay nearby");
            } else {
                snprintf(raw[1], sizeof(raw[1]), "Next dose at %s", due_time);
                snprintf(raw[2], sizeof(raw[2]), "%.31s", status.next_medicine);
                snprintf(raw[3], sizeof(raw[3]), "%d dose%s remaining",
                         status.pending_count, status.pending_count == 1 ? "" : "s");
            }
        }
    }

    for (uint8_t row = 0; row < LCD_ROWS; row++) {
        copy_lcd_text(lines[row], raw[row]);
    }
    render_lcd_screen(lines);
}

static void update_alert_outputs(void)
{
    scheduler_status_t status;
    time_t now_utc = 0;
    int64_t now_us = esp_timer_get_time();
    bool changed;

    if (s_output_test_active) return;
    if (scheduler_get_status(&status) != ESP_OK) return;

    if (status.activity == SCHEDULER_ACTIVITY_SUCCESS) {
        set_alert_color(RGB_LED_BLUE, "dispense success");
        return;
    }

    if (status.activity == SCHEDULER_ACTIVITY_WAITING_FOR_HAND ||
        status.activity == SCHEDULER_ACTIVITY_DISPENSING) {
        changed = set_alert_color(RGB_LED_RED, "dose due");
        if (changed || (now_us - s_last_due_buzz_us) >= (DUE_BUZZ_INTERVAL_SEC * 1000000LL)) {
            s_last_due_buzz_us = now_us;
            buzzer_due_alert();
        }
        return;
    }

    if (status.activity == SCHEDULER_ACTIVITY_ERROR) {
        changed = set_alert_color(RGB_LED_RED, "dispense error");
        if (changed) {
            s_last_due_buzz_us = now_us;
            buzzer_due_alert();
        }
        return;
    }

    if (!time_service_is_synchronized()) {
        set_alert_color(RGB_LED_IDLE, "waiting for synchronized time");
        return;
    }

    if (!status.has_next_dose) {
        set_alert_color(RGB_LED_IDLE, "no pending schedule");
        return;
    }

    time_service_get_timestamp(&now_utc);
    int64_t seconds_to_due = (int64_t)(status.next_due_time - now_utc);

    if (seconds_to_due <= 0) {
        changed = set_alert_color(RGB_LED_RED, "dose time reached");
        if (changed || (now_us - s_last_due_buzz_us) >= (DUE_BUZZ_INTERVAL_SEC * 1000000LL)) {
            s_last_due_buzz_us = now_us;
            buzzer_due_alert();
        }
    } else if (seconds_to_due <= DOSE_NEAR_WINDOW_SEC) {
        changed = set_alert_color(RGB_LED_YELLOW, "dose time near");
        if (changed || (now_us - s_last_near_buzz_us) >= (NEAR_BUZZ_INTERVAL_SEC * 1000000LL)) {
            s_last_near_buzz_us = now_us;
            buzzer_near_alert();
        }
    } else {
        set_alert_color(RGB_LED_GREEN, "pending future schedule");
    }
}

static void alert_self_test(void)
{
    ESP_LOGI(TAG, "Running alert self-test");
    s_output_test_active = true;
    rgb_led_set(RGB_LED_RED);
    buzzer_tone(1800, 120);
    vTaskDelay(pdMS_TO_TICKS(900));
    rgb_led_set(RGB_LED_GREEN);
    buzzer_tone(2200, 120);
    vTaskDelay(pdMS_TO_TICKS(900));
    rgb_led_set(RGB_LED_BLUE);
    buzzer_tone(2600, 120);
    vTaskDelay(pdMS_TO_TICKS(900));
    rgb_led_set(RGB_LED_YELLOW);
    vTaskDelay(pdMS_TO_TICKS(900));
    rgb_led_set(RGB_LED_WHITE);
    vTaskDelay(pdMS_TO_TICKS(900));
    s_output_test_active = false;
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
        scheduler_calibration_move_steps(MOTOR_EFFECTIVE_STEPS_PER_REV);
    } else if (strcmp(action, "reboot") == 0) {
        ESP_LOGI(TAG, "Rebooting in 1 second...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    } else if (strcmp(action, "clear_schedule") == 0) {
        scheduler_clear();
        mqtt_manager_publish_log("schedule_updated", NULL, -1, "cleared by command");
        show_lcd_notice("Schedule cleared", "No doses pending", "", "");
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
        scheduler_status_t status;
        char next_line[32] = "Schedule is active";
        char count_line[32] = "";
        const char *medicine = "";

        scheduler_get_status(&status);
        if (status.has_next_dose) {
            char due_time[8];
            format_local_due_time(status.next_due_time, due_time, sizeof(due_time));
            snprintf(next_line, sizeof(next_line), "Next dose: %s", due_time);
            medicine = status.next_medicine;
        } else if (status.activity == SCHEDULER_ACTIVITY_WAITING_FOR_HAND) {
            snprintf(next_line, sizeof(next_line), "Dose is due now");
            medicine = status.active_medicine;
        }
        snprintf(count_line, sizeof(count_line), "%d dose%s planned",
                 status.pending_count, status.pending_count == 1 ? "" : "s");

        mqtt_manager_publish_log("schedule_updated", NULL, -1, "schedule stored");
        show_lcd_notice("Schedule saved", next_line, medicine, count_line);
    } else {
        ESP_LOGE(TAG, "Failed to set schedule");
        if (ret == ESP_ERR_NOT_FOUND) {
            mqtt_manager_publish_log("error", NULL, -1, "no future dose times");
            show_lcd_notice("Schedule not saved", "No future dose times", "Check date and time", "Try again");
        } else {
            mqtt_manager_publish_log("error", NULL, -1, "invalid schedule JSON");
            show_lcd_notice("Schedule not saved", "Invalid schedule", "Check app details", "Try again");
        }
    }
}

static void mqtt_connected_cb(void)
{
    ESP_LOGI(TAG, "MQTT connected, publishing online status");
    mqtt_manager_publish_status("online");
}

static void start_mqtt_once(void)
{
    if (s_mqtt_started) return;

    esp_err_t ret = mqtt_manager_start();
    if (ret == ESP_OK) {
        s_mqtt_started = true;
        ESP_LOGI(TAG, "MQTT started after clock synchronization");
    } else {
        ESP_LOGE(TAG, "MQTT start failed: %s", esp_err_to_name(ret));
    }
}

static void time_synchronized_cb(void)
{
    ESP_LOGI(TAG, "Clock synchronized; MQTT schedules can now be accepted safely");
    start_mqtt_once();
}

static void wifi_connected_cb(void)
{
    ESP_LOGI(TAG, "WiFi connected, synchronizing clock");
    if (time_service_is_synchronized()) {
        start_mqtt_once();
    } else {
        time_service_sync_ntp();
    }
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
    time_service_set_timezone(3);
    time_service_register_sync_cb(time_synchronized_cb);
    ESP_LOGI(TAG, "Software clock initialised; waiting for NTP");

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

    /* 3b. Alert outputs */
    ESP_ERROR_CHECK(rgb_led_init(RGB_LED_RED_PIN,
                                 RGB_LED_GREEN_PIN,
                                 RGB_LED_BLUE_PIN,
                                 RGB_LED_ACTIVE_HIGH));
    ESP_ERROR_CHECK(buzzer_init(BUZZER_PIN));
    rgb_led_set(RGB_LED_WHITE);
    ESP_LOGI(TAG, "Alerts initialised: buzzer GPIO%d, RGB R=%d G=%d B=%d",
             BUZZER_PIN, RGB_LED_RED_PIN, RGB_LED_GREEN_PIN, RGB_LED_BLUE_PIN);
    buzzer_tone(2200, 120);

    /* 4. Ultrasonic sensor */
    ESP_ERROR_CHECK(ultrasonic_init(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, ULTRASONIC_TIMEOUT_US));
    ESP_LOGI(TAG, "Ultrasonic ready for due-dose hand detection");

    /* 5. Stepper motor */
    stepper_config_t stepper_cfg = {
        .step_pin = STEP_PIN,
        .dir_pin = DIR_PIN,
        .enable_pin = ENABLE_PIN,
        .step_delay_us = STEP_DELAY_US,
        .enable_on_init = false
    };
    ESP_ERROR_CHECK(stepper_motor_init(&stepper_cfg));
    ESP_LOGI(TAG,
             "Stepper motor initialised: motor=%d full steps/rev, effective=%d pulses/rev, %d chambers, %d pulses/chamber",
             MOTOR_FULL_STEPS_PER_REV, MOTOR_EFFECTIVE_STEPS_PER_REV,
             CAROUSEL_CHAMBERS, STEPS_PER_CHAMBER);

    /* 6. Keypad */
    gpio_num_t row_pins[4] = KEYPAD_ROW_PINS;
    gpio_num_t col_pins[4] = KEYPAD_COL_PINS;
    ESP_ERROR_CHECK(keypad_init(row_pins, col_pins));
    ESP_ERROR_CHECK(keypad_start());
    ESP_LOGI(TAG, "Keypad initialised");

    /* 7. MQTT callbacks are ready before networking can deliver messages. */
    ESP_ERROR_CHECK(mqtt_manager_init(MQTT_BROKER_URI, MQTT_USERNAME, MQTT_PASSWORD));
    mqtt_manager_register_connected_cb(mqtt_connected_cb);
    mqtt_manager_register_command_cb(mqtt_command_cb, NULL);
    mqtt_manager_register_schedule_cb(mqtt_schedule_cb, NULL);

    /* 8. Scheduler is ready before retained MQTT schedules can arrive. */
    ESP_ERROR_CHECK(scheduler_init(STEPS_PER_CHAMBER, HAND_WAIT_REMINDER_SEC));
    ESP_LOGI(TAG, "Scheduler initialised: %d steps/chamber, %d sec hand reminder",
             STEPS_PER_CHAMBER, HAND_WAIT_REMINDER_SEC);

    /* 9. Start WiFi. MQTT starts only after NTP confirms valid time. */
    ESP_ERROR_CHECK(wifi_manager_init(WIFI_SSID, WIFI_PASSWORD));
    wifi_manager_register_connected_cb(wifi_connected_cb);
    ESP_ERROR_CHECK(wifi_manager_start());

    /* 10. Main loop */
    char key;
    while (1) {
        if (s_local_ui.state == LOCAL_UI_DONE) {
            if (s_local_ui.message_ticks > 0) {
                s_local_ui.message_ticks--;
            } else {
                memset(&s_local_ui, 0, sizeof(s_local_ui));
                s_lcd_cache_valid = false;
            }
        }

        update_alert_outputs();

        if (!local_ui_active()) {
            update_lcd_dashboard();
        }

        /* Check for keypad input */
        if (keypad_get_key(&key)) {
            ESP_LOGI(TAG, "Key pressed: %c", key);

            if (local_ui_handle_key(key)) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            if (key == 'A') {
                scheduler_dispense_now();
            } else if (key == 'B') {
                scheduler_calibration_move_steps(STEPS_PER_CHAMBER);
            } else if (key == 'C') {
                scheduler_calibration_move_steps(MOTOR_EFFECTIVE_STEPS_PER_REV);
            } else if (key == 'D') {
                local_ui_start();
            } else if (key == '#') {
                alert_self_test();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
