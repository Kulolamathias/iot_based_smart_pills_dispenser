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
#include "nvs.h"
#include "nvs_flash.h"
#include "cJSON.h"
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
#define LOCAL_UI_TIMEOUT_SEC    45
#define LOCAL_UI_MESSAGE_MS     1800
#define LOCAL_UI_DONE_MS        4500
#define LOCAL_MAX_DAILY_TIMES   4
#define LOCAL_MAX_TOTAL_PILLS   999
#define LOCAL_MAX_PILLS_PER_DOSE 99
#define LOCAL_ACCESS_PIN_LEN    4
#define LOCAL_UI_NVS_NAMESPACE  "local_ui"
#define LOCAL_UI_NVS_PIN_KEY    "access_pin"
#define LOCAL_UI_NVS_MED_SEQ_KEY "medicine_seq"

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
    LOCAL_UI_AUTH,
    LOCAL_UI_MENU,
    LOCAL_UI_REPLACE_WARNING,
    LOCAL_UI_DAILY_COUNT,
    LOCAL_UI_TIME,
    LOCAL_UI_QUANTITY_METHOD,
    LOCAL_UI_TOTAL_DOSES,
    LOCAL_UI_TOTAL_PILLS,
    LOCAL_UI_PILLS_PER_DOSE,
    LOCAL_UI_REVIEW,
    LOCAL_UI_END_REVIEW,
    LOCAL_UI_NEW_PIN,
    LOCAL_UI_CONFIRM_PIN,
    LOCAL_UI_SET_DATE,
    LOCAL_UI_SET_CLOCK_TIME,
    LOCAL_UI_CLOCK_CONFIRM,
    LOCAL_UI_MESSAGE
} local_ui_state_t;

typedef enum {
    LOCAL_QUANTITY_NONE = 0,
    LOCAL_QUANTITY_DOSES,
    LOCAL_QUANTITY_PILLS
} local_quantity_mode_t;

typedef struct {
    local_ui_state_t state;
    local_ui_state_t message_next_state;
    char input[12];
    size_t input_len;
    int daily_count;
    int time_index;
    int times[LOCAL_MAX_DAILY_TIMES][2];
    local_quantity_mode_t quantity_mode;
    int total_doses;
    int total_pills;
    int pills_per_dose;
    int duration_days;
    struct tm end_local;
    int clock_day;
    int clock_month;
    int clock_year;
    int clock_hour;
    int clock_minute;
    char medicine[SCHEDULER_MEDICINE_NAME_LEN];
    char pin_candidate[LOCAL_ACCESS_PIN_LEN + 1];
    char message_lines[LCD_ROWS][LCD_COLS + 1];
    int64_t last_activity_us;
    int64_t message_until_us;
} local_schedule_ui_t;

static local_schedule_ui_t s_local_ui = {
    .state = LOCAL_UI_IDLE
};
static char s_local_access_pin[LOCAL_ACCESS_PIN_LEN + 1] = "1234";
static uint8_t s_local_medicine_sequence = 1;

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

static void local_ui_show(void);

static bool local_ui_pin_is_valid(const char *pin)
{
    if (!pin || strlen(pin) != LOCAL_ACCESS_PIN_LEN) return false;
    for (size_t i = 0; i < LOCAL_ACCESS_PIN_LEN; i++) {
        if (pin[i] < '0' || pin[i] > '9') return false;
    }
    return true;
}

static void local_ui_storage_init(void)
{
    nvs_handle_t nvs;
    if (nvs_open(LOCAL_UI_NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGW(TAG, "Local UI storage unavailable; using default PIN");
        return;
    }

    char stored_pin[LOCAL_ACCESS_PIN_LEN + 1] = {0};
    size_t pin_size = sizeof(stored_pin);
    if (nvs_get_str(nvs, LOCAL_UI_NVS_PIN_KEY, stored_pin, &pin_size) == ESP_OK &&
        local_ui_pin_is_valid(stored_pin)) {
        strlcpy(s_local_access_pin, stored_pin, sizeof(s_local_access_pin));
    } else {
        nvs_set_str(nvs, LOCAL_UI_NVS_PIN_KEY, s_local_access_pin);
    }

    uint8_t sequence = 1;
    if (nvs_get_u8(nvs, LOCAL_UI_NVS_MED_SEQ_KEY, &sequence) == ESP_OK &&
        sequence >= 1 && sequence <= 99) {
        s_local_medicine_sequence = sequence;
    } else {
        nvs_set_u8(nvs, LOCAL_UI_NVS_MED_SEQ_KEY, s_local_medicine_sequence);
    }

    nvs_commit(nvs);
    nvs_close(nvs);
}

static esp_err_t local_ui_store_pin(const char *pin)
{
    nvs_handle_t nvs;
    if (!local_ui_pin_is_valid(pin)) return ESP_ERR_INVALID_ARG;
    esp_err_t ret = nvs_open(LOCAL_UI_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret != ESP_OK) return ret;
    ret = nvs_set_str(nvs, LOCAL_UI_NVS_PIN_KEY, pin);
    if (ret == ESP_OK) ret = nvs_commit(nvs);
    nvs_close(nvs);
    return ret;
}

static void local_ui_advance_medicine_sequence(void)
{
    s_local_medicine_sequence = (s_local_medicine_sequence >= 99)
                                    ? 1
                                    : (uint8_t)(s_local_medicine_sequence + 1);

    nvs_handle_t nvs;
    if (nvs_open(LOCAL_UI_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, LOCAL_UI_NVS_MED_SEQ_KEY, s_local_medicine_sequence);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

static void local_ui_clear_input(void)
{
    s_local_ui.input_len = 0;
    s_local_ui.input[0] = '\0';
}

static void local_ui_set_state(local_ui_state_t state)
{
    s_local_ui.state = state;
    local_ui_clear_input();
    s_local_ui.last_activity_us = esp_timer_get_time();
    local_ui_show();
}

static void local_ui_show_message(const char *line0, const char *line1,
                                  const char *line2, const char *line3,
                                  local_ui_state_t next_state, uint32_t duration_ms)
{
    copy_lcd_text(s_local_ui.message_lines[0], line0);
    copy_lcd_text(s_local_ui.message_lines[1], line1);
    copy_lcd_text(s_local_ui.message_lines[2], line2);
    copy_lcd_text(s_local_ui.message_lines[3], line3);
    s_local_ui.message_next_state = next_state;
    s_local_ui.message_until_us = esp_timer_get_time() + (duration_ms * 1000LL);
    s_local_ui.state = LOCAL_UI_MESSAGE;
    local_ui_show();
}

static bool local_ui_add_digit(char key, size_t max_len)
{
    if (key < '0' || key > '9' || s_local_ui.input_len >= max_len) return false;
    s_local_ui.input[s_local_ui.input_len++] = key;
    s_local_ui.input[s_local_ui.input_len] = '\0';
    return true;
}

static int local_ui_input_int(void)
{
    return s_local_ui.input_len == 0 ? -1 : atoi(s_local_ui.input);
}

static void local_ui_format_pin(char *buffer, size_t size)
{
    size_t count = s_local_ui.input_len;
    if (count > LOCAL_ACCESS_PIN_LEN) count = LOCAL_ACCESS_PIN_LEN;
    memset(buffer, '*', count);
    buffer[count] = '\0';
    strlcat(buffer, "_", size);
}

static void local_ui_format_time_input(char *buffer, size_t size)
{
    char digits[5] = "____";
    for (size_t i = 0; i < s_local_ui.input_len && i < 4; i++) {
        digits[i] = s_local_ui.input[i];
    }
    snprintf(buffer, size, "%c%c:%c%c", digits[0], digits[1], digits[2], digits[3]);
}

static void local_ui_format_date_input(char *buffer, size_t size)
{
    char digits[9] = "________";
    for (size_t i = 0; i < s_local_ui.input_len && i < 8; i++) {
        digits[i] = s_local_ui.input[i];
    }
    snprintf(buffer, size, "%c%c/%c%c/%c%c%c%c",
             digits[0], digits[1], digits[2], digits[3],
             digits[4], digits[5], digits[6], digits[7]);
}

static bool local_ui_is_leap_year(int year)
{
    return (year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0));
}

static int local_ui_days_in_month(int year, int month)
{
    static const uint8_t days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month < 1 || month > 12) return 0;
    if (month == 2 && local_ui_is_leap_year(year)) return 29;
    return days[month - 1];
}

static bool local_ui_valid_date(int year, int month, int day)
{
    int max_day = local_ui_days_in_month(year, month);
    return year >= 2024 && year <= 2099 && day >= 1 && day <= max_day;
}

static void local_ui_add_days(struct tm *date, int days)
{
    int year = date->tm_year + 1900;
    int month = date->tm_mon + 1;
    int day = date->tm_mday;

    while (days-- > 0) {
        day++;
        if (day > local_ui_days_in_month(year, month)) {
            day = 1;
            month++;
            if (month > 12) {
                month = 1;
                year++;
            }
        }
    }

    date->tm_year = year - 1900;
    date->tm_mon = month - 1;
    date->tm_mday = day;
}

static time_t local_ui_make_utc_timestamp(int year, int month, int day,
                                           int hour, int minute)
{
    static const uint8_t days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    time_t total_days = 0;

    for (int y = 1970; y < year; y++) {
        total_days += 365 + (local_ui_is_leap_year(y) ? 1 : 0);
    }
    for (int m = 1; m < month; m++) {
        total_days += days[m - 1];
        if (m == 2 && local_ui_is_leap_year(year)) total_days++;
    }
    total_days += day - 1;

    return total_days * 86400 + hour * 3600 + minute * 60 - LOCAL_TIME_OFFSET_SEC;
}

static void local_ui_sort_times(void)
{
    for (int i = 0; i < s_local_ui.daily_count - 1; i++) {
        for (int j = i + 1; j < s_local_ui.daily_count; j++) {
            int left = s_local_ui.times[i][0] * 60 + s_local_ui.times[i][1];
            int right = s_local_ui.times[j][0] * 60 + s_local_ui.times[j][1];
            if (right < left) {
                int hour = s_local_ui.times[i][0];
                int minute = s_local_ui.times[i][1];
                s_local_ui.times[i][0] = s_local_ui.times[j][0];
                s_local_ui.times[i][1] = s_local_ui.times[j][1];
                s_local_ui.times[j][0] = hour;
                s_local_ui.times[j][1] = minute;
            }
        }
    }
}

static bool local_ui_time_already_entered(int hour, int minute)
{
    for (int i = 0; i < s_local_ui.time_index; i++) {
        if (s_local_ui.times[i][0] == hour && s_local_ui.times[i][1] == minute) {
            return true;
        }
    }
    return false;
}

static bool local_ui_calculate_end(void)
{
    if (!time_service_is_synchronized() || s_local_ui.total_doses < 1 ||
        s_local_ui.daily_count < 1) {
        return false;
    }

    local_ui_sort_times();
    struct tm now;
    if (time_service_get_tm(&now) != ESP_OK) return false;

    int generated = 0;
    int current_minute = now.tm_hour * 60 + now.tm_min;
    for (int day_offset = 0; day_offset < 366; day_offset++) {
        for (int i = 0; i < s_local_ui.daily_count; i++) {
            int dose_minute = s_local_ui.times[i][0] * 60 + s_local_ui.times[i][1];
            if (day_offset == 0 && dose_minute < current_minute) continue;

            generated++;
            if (generated == s_local_ui.total_doses) {
                s_local_ui.end_local = now;
                local_ui_add_days(&s_local_ui.end_local, day_offset);
                s_local_ui.end_local.tm_hour = s_local_ui.times[i][0];
                s_local_ui.end_local.tm_min = s_local_ui.times[i][1];
                s_local_ui.end_local.tm_sec = 0;
                /* total_doses caps the plan; one guard day covers a save at a minute boundary. */
                s_local_ui.duration_days = day_offset + 2;
                return true;
            }
        }
    }
    return false;
}

static void local_ui_format_times(char *buffer, size_t size)
{
    buffer[0] = '\0';
    for (int i = 0; i < s_local_ui.daily_count; i++) {
        char item[8];
        if (s_local_ui.daily_count <= 2) {
            snprintf(item, sizeof(item), "%02d:%02d", s_local_ui.times[i][0],
                     s_local_ui.times[i][1]);
        } else {
            snprintf(item, sizeof(item), "%02d%02d", s_local_ui.times[i][0],
                     s_local_ui.times[i][1]);
        }
        if (i > 0) strlcat(buffer, " ", size);
        strlcat(buffer, item, size);
    }
}

static void local_ui_reset_plan(void)
{
    s_local_ui.daily_count = 0;
    s_local_ui.time_index = 0;
    memset(s_local_ui.times, 0, sizeof(s_local_ui.times));
    s_local_ui.quantity_mode = LOCAL_QUANTITY_NONE;
    s_local_ui.total_doses = 0;
    s_local_ui.total_pills = 0;
    s_local_ui.pills_per_dose = 0;
    s_local_ui.duration_days = 0;
    memset(&s_local_ui.end_local, 0, sizeof(s_local_ui.end_local));
}

static void local_ui_show(void)
{
    if (s_local_ui.state == LOCAL_UI_IDLE) return;

    char raw[LCD_ROWS][48] = {{0}};
    char lines[LCD_ROWS][LCD_COLS + 1] = {{0}};
    char formatted[24] = "";

    switch (s_local_ui.state) {
        case LOCAL_UI_AUTH:
            local_ui_format_pin(formatted, sizeof(formatted));
            snprintf(raw[0], sizeof(raw[0]), "Protected setup");
            snprintf(raw[1], sizeof(raw[1]), "Enter 4-digit PIN");
            snprintf(raw[2], sizeof(raw[2]), "PIN: %s", formatted);
            snprintf(raw[3], sizeof(raw[3]), "#=Login *=Exit");
            break;

        case LOCAL_UI_MENU:
            snprintf(raw[0], sizeof(raw[0]), "Schedule settings");
            snprintf(raw[1], sizeof(raw[1]), "1 New dose plan");
            snprintf(raw[2], sizeof(raw[2]), "2 Change access PIN");
            snprintf(raw[3], sizeof(raw[3]), "3 Set clock  *=Exit");
            break;

        case LOCAL_UI_REPLACE_WARNING:
            snprintf(raw[0], sizeof(raw[0]), "Existing plan found");
            snprintf(raw[1], sizeof(raw[1]), "New plan replaces it");
            snprintf(raw[2], sizeof(raw[2]), "Current doses remain");
            snprintf(raw[3], sizeof(raw[3]), "#=Continue B=Menu");
            break;

        case LOCAL_UI_DAILY_COUNT:
            snprintf(raw[0], sizeof(raw[0]), "Times taken per day");
            snprintf(raw[1], sizeof(raw[1]), "Enter 1 to %d", LOCAL_MAX_DAILY_TIMES);
            snprintf(raw[2], sizeof(raw[2]), "Daily times: %s_", s_local_ui.input);
            snprintf(raw[3], sizeof(raw[3]), "#=Next B=Menu");
            break;

        case LOCAL_UI_TIME:
            local_ui_format_time_input(formatted, sizeof(formatted));
            snprintf(raw[0], sizeof(raw[0]), "Dose time %d of %d",
                     s_local_ui.time_index + 1, s_local_ui.daily_count);
            snprintf(raw[1], sizeof(raw[1]), "24-hour format HHMM");
            snprintf(raw[2], sizeof(raw[2]), "Time: %s", formatted);
            snprintf(raw[3], sizeof(raw[3]), "#=Next B=Back");
            break;

        case LOCAL_UI_QUANTITY_METHOD:
            snprintf(raw[0], sizeof(raw[0]), "Treatment quantity");
            snprintf(raw[1], sizeof(raw[1]), "1 Know total doses");
            snprintf(raw[2], sizeof(raw[2]), "2 Count total pills");
            snprintf(raw[3], sizeof(raw[3]), "B=Back *=Exit");
            break;

        case LOCAL_UI_TOTAL_DOSES:
            snprintf(raw[0], sizeof(raw[0]), "Total prepared doses");
            snprintf(raw[1], sizeof(raw[1]), "Enter 1 to %d", CAROUSEL_CHAMBERS);
            snprintf(raw[2], sizeof(raw[2]), "Doses: %s_", s_local_ui.input);
            snprintf(raw[3], sizeof(raw[3]), "#=Review B=Back");
            break;

        case LOCAL_UI_TOTAL_PILLS:
            snprintf(raw[0], sizeof(raw[0]), "Available pill count");
            snprintf(raw[1], sizeof(raw[1]), "Enter 1 to %d", LOCAL_MAX_TOTAL_PILLS);
            snprintf(raw[2], sizeof(raw[2]), "Pills: %s_", s_local_ui.input);
            snprintf(raw[3], sizeof(raw[3]), "#=Next B=Back");
            break;

        case LOCAL_UI_PILLS_PER_DOSE:
            snprintf(raw[0], sizeof(raw[0]), "Pills per dose");
            snprintf(raw[1], sizeof(raw[1]), "Enter 1 to %d", LOCAL_MAX_PILLS_PER_DOSE);
            snprintf(raw[2], sizeof(raw[2]), "Each dose: %s_", s_local_ui.input);
            snprintf(raw[3], sizeof(raw[3]), "#=Review B=Back");
            break;

        case LOCAL_UI_REVIEW:
            local_ui_format_times(formatted, sizeof(formatted));
            snprintf(raw[0], sizeof(raw[0]), "%s", s_local_ui.medicine);
            if (s_local_ui.quantity_mode == LOCAL_QUANTITY_PILLS) {
                snprintf(raw[1], sizeof(raw[1]), "%d/day %d doses, %dea",
                         s_local_ui.daily_count, s_local_ui.total_doses,
                         s_local_ui.pills_per_dose);
            } else {
                snprintf(raw[1], sizeof(raw[1]), "%d/day  %d total doses",
                         s_local_ui.daily_count, s_local_ui.total_doses);
            }
            snprintf(raw[2], sizeof(raw[2]), "%s", formatted);
            snprintf(raw[3], sizeof(raw[3]), "#=End B=Edit *=No");
            break;

        case LOCAL_UI_END_REVIEW: {
            int final_pills = s_local_ui.pills_per_dose;
            if (s_local_ui.quantity_mode == LOCAL_QUANTITY_PILLS &&
                (s_local_ui.total_pills % s_local_ui.pills_per_dose) != 0) {
                final_pills = s_local_ui.total_pills % s_local_ui.pills_per_dose;
            }
            snprintf(raw[0], sizeof(raw[0]), "Treatment ends");
            snprintf(raw[1], sizeof(raw[1]), "%02d/%02d/%04d %02d:%02d",
                     s_local_ui.end_local.tm_mday,
                     s_local_ui.end_local.tm_mon + 1,
                     s_local_ui.end_local.tm_year + 1900,
                     s_local_ui.end_local.tm_hour,
                     s_local_ui.end_local.tm_min);
            if (s_local_ui.quantity_mode == LOCAL_QUANTITY_PILLS) {
                snprintf(raw[2], sizeof(raw[2]), "Last dose: %d pill%s", final_pills,
                         final_pills == 1 ? "" : "s");
            } else {
                snprintf(raw[2], sizeof(raw[2]), "%d dose times planned",
                         s_local_ui.total_doses);
            }
            snprintf(raw[3], sizeof(raw[3]), "A=Save B=Back *=No");
            break;
        }

        case LOCAL_UI_NEW_PIN:
            local_ui_format_pin(formatted, sizeof(formatted));
            snprintf(raw[0], sizeof(raw[0]), "Choose a new PIN");
            snprintf(raw[1], sizeof(raw[1]), "Use exactly 4 digits");
            snprintf(raw[2], sizeof(raw[2]), "New PIN: %s", formatted);
            snprintf(raw[3], sizeof(raw[3]), "#=Next B=Menu");
            break;

        case LOCAL_UI_CONFIRM_PIN:
            local_ui_format_pin(formatted, sizeof(formatted));
            snprintf(raw[0], sizeof(raw[0]), "Confirm new PIN");
            snprintf(raw[1], sizeof(raw[1]), "Enter it once more");
            snprintf(raw[2], sizeof(raw[2]), "PIN: %s", formatted);
            snprintf(raw[3], sizeof(raw[3]), "#=Save B=Back");
            break;

        case LOCAL_UI_SET_DATE:
            local_ui_format_date_input(formatted, sizeof(formatted));
            snprintf(raw[0], sizeof(raw[0]), "Set current date");
            snprintf(raw[1], sizeof(raw[1]), "Enter DDMMYYYY");
            snprintf(raw[2], sizeof(raw[2]), "%s", formatted);
            snprintf(raw[3], sizeof(raw[3]), "#=Next B=Menu");
            break;

        case LOCAL_UI_SET_CLOCK_TIME:
            local_ui_format_time_input(formatted, sizeof(formatted));
            snprintf(raw[0], sizeof(raw[0]), "Set current time");
            snprintf(raw[1], sizeof(raw[1]), "24-hour format HHMM");
            snprintf(raw[2], sizeof(raw[2]), "%s", formatted);
            snprintf(raw[3], sizeof(raw[3]), "#=Next B=Back");
            break;

        case LOCAL_UI_CLOCK_CONFIRM:
            snprintf(raw[0], sizeof(raw[0]), "Confirm local clock");
            snprintf(raw[1], sizeof(raw[1]), "%02d/%02d/%04d",
                     s_local_ui.clock_day, s_local_ui.clock_month, s_local_ui.clock_year);
            snprintf(raw[2], sizeof(raw[2]), "Time %02d:%02d",
                     s_local_ui.clock_hour, s_local_ui.clock_minute);
            snprintf(raw[3], sizeof(raw[3]), "A=Save B=Back *=No");
            break;

        case LOCAL_UI_MESSAGE:
            memcpy(lines, s_local_ui.message_lines, sizeof(lines));
            render_lcd_screen(lines);
            return;

        case LOCAL_UI_IDLE:
        default:
            return;
    }

    for (int row = 0; row < LCD_ROWS; row++) {
        copy_lcd_text(lines[row], raw[row]);
    }
    render_lcd_screen(lines);
}

static void local_ui_start(void)
{
    memset(&s_local_ui, 0, sizeof(s_local_ui));
    s_local_ui.state = LOCAL_UI_AUTH;
    s_local_ui.last_activity_us = esp_timer_get_time();
    snprintf(s_local_ui.medicine, sizeof(s_local_ui.medicine), "Local Dose %02u",
             s_local_medicine_sequence);
    local_ui_show();
}

static void local_ui_cancel(void)
{
    memset(&s_local_ui, 0, sizeof(s_local_ui));
    s_local_ui.state = LOCAL_UI_IDLE;
    s_lcd_cache_valid = false;
    show_lcd_notice("Setup cancelled", "Nothing was saved", "", "");
}

static void local_ui_begin_plan(void)
{
    local_ui_reset_plan();
    if (!time_service_is_synchronized()) {
        local_ui_show_message("Clock is not ready", "Choose 3 Set clock", "or connect to WiFi",
                              "Returning to menu", LOCAL_UI_MENU, LOCAL_UI_MESSAGE_MS + 700);
    } else if (scheduler_get_pending_count() > 0) {
        local_ui_set_state(LOCAL_UI_REPLACE_WARNING);
    } else {
        local_ui_set_state(LOCAL_UI_DAILY_COUNT);
    }
}

static void local_ui_restart_plan_edit(void)
{
    local_ui_reset_plan();
    local_ui_set_state(LOCAL_UI_DAILY_COUNT);
}

static void local_ui_save_schedule(void)
{
    if (!time_service_is_synchronized()) {
        local_ui_show_message("Schedule not saved", "Clock is not ready", "Set clock and retry", "",
                              LOCAL_UI_END_REVIEW, LOCAL_UI_MESSAGE_MS + 700);
        return;
    }
    if (!local_ui_calculate_end()) {
        local_ui_show_message("Schedule not saved", "Could not calculate", "the final dose time", "",
                              LOCAL_UI_END_REVIEW, LOCAL_UI_MESSAGE_MS);
        return;
    }

    struct tm now;
    if (time_service_get_tm(&now) != ESP_OK) {
        local_ui_show_message("Schedule not saved", "Clock read failed", "Please try again", "",
                              LOCAL_UI_END_REVIEW, LOCAL_UI_MESSAGE_MS);
        return;
    }

    cJSON *root = cJSON_CreateArray();
    cJSON *entry = cJSON_CreateObject();
    cJSON *times = cJSON_CreateArray();
    if (!root || !entry || !times) {
        if (times) cJSON_Delete(times);
        if (entry) cJSON_Delete(entry);
        if (root) cJSON_Delete(root);
        local_ui_show_message("Schedule not saved", "Memory unavailable", "Please try again", "",
                              LOCAL_UI_END_REVIEW, LOCAL_UI_MESSAGE_MS);
        return;
    }

    cJSON_AddStringToObject(entry, "name", s_local_ui.medicine);
    for (int i = 0; i < s_local_ui.daily_count; i++) {
        char value[6];
        snprintf(value, sizeof(value), "%02d:%02d", s_local_ui.times[i][0],
                 s_local_ui.times[i][1]);
        cJSON_AddItemToArray(times, cJSON_CreateString(value));
    }
    cJSON_AddItemToObject(entry, "times", times);
    cJSON_AddNumberToObject(entry, "duration_days", s_local_ui.duration_days);
    cJSON_AddNumberToObject(entry, "total_doses", s_local_ui.total_doses);
    if (s_local_ui.quantity_mode == LOCAL_QUANTITY_PILLS) {
        cJSON_AddNumberToObject(entry, "total_pills", s_local_ui.total_pills);
        cJSON_AddNumberToObject(entry, "pills_per_dose", s_local_ui.pills_per_dose);
    }

    char start_date[11];
    strftime(start_date, sizeof(start_date), "%Y-%m-%d", &now);
    cJSON_AddStringToObject(entry, "start_date", start_date);
    cJSON_AddItemToArray(root, entry);

    char *json = cJSON_PrintUnformatted(root);
    if (!json) {
        cJSON_Delete(root);
        local_ui_show_message("Schedule not saved", "Memory unavailable", "Please try again", "",
                              LOCAL_UI_END_REVIEW, LOCAL_UI_MESSAGE_MS);
        return;
    }

    esp_err_t ret = scheduler_set_schedule(json);
    free(json);
    cJSON_Delete(root);

    if (ret == ESP_OK) {
        char count_line[32];
        char end_line[32];
        mqtt_manager_publish_log("schedule_updated", s_local_ui.medicine, -1,
                                 "saved from keypad");
        snprintf(count_line, sizeof(count_line), "%d doses, %d per day",
                 s_local_ui.total_doses, s_local_ui.daily_count);
        snprintf(end_line, sizeof(end_line), "Ends %02d/%02d at %02d:%02d",
                 s_local_ui.end_local.tm_mday, s_local_ui.end_local.tm_mon + 1,
                 s_local_ui.end_local.tm_hour, s_local_ui.end_local.tm_min);
        local_ui_advance_medicine_sequence();
        local_ui_show_message("Schedule saved", s_local_ui.medicine, count_line, end_line,
                              LOCAL_UI_IDLE, LOCAL_UI_DONE_MS);
    } else {
        local_ui_show_message("Schedule not saved", "No future dose times", "Review date and time", "",
                              LOCAL_UI_END_REVIEW, LOCAL_UI_MESSAGE_MS + 700);
    }
}

static void local_ui_save_manual_clock(void)
{
    time_t utc = local_ui_make_utc_timestamp(s_local_ui.clock_year, s_local_ui.clock_month,
                                              s_local_ui.clock_day, s_local_ui.clock_hour,
                                              s_local_ui.clock_minute);
    esp_err_t ret = time_service_set_timestamp(utc);
    if (ret == ESP_OK) {
        char date_line[24];
        char time_line[24];
        snprintf(date_line, sizeof(date_line), "%02d/%02d/%04d",
                 s_local_ui.clock_day, s_local_ui.clock_month, s_local_ui.clock_year);
        snprintf(time_line, sizeof(time_line), "Local time %02d:%02d",
                 s_local_ui.clock_hour, s_local_ui.clock_minute);
        local_ui_show_message("Clock saved", date_line, time_line, "NTP will refine it",
                              LOCAL_UI_MENU, LOCAL_UI_MESSAGE_MS + 700);
    } else {
        local_ui_show_message("Clock not saved", "Please try again", "", "",
                              LOCAL_UI_CLOCK_CONFIRM, LOCAL_UI_MESSAGE_MS);
    }
}

static bool local_ui_handle_key(char key)
{
    if (s_local_ui.state == LOCAL_UI_IDLE) return false;

    s_local_ui.last_activity_us = esp_timer_get_time();
    if (key == '*') {
        local_ui_cancel();
        return true;
    }
    if (s_local_ui.state == LOCAL_UI_MESSAGE) return true;

    int value;
    switch (s_local_ui.state) {
        case LOCAL_UI_AUTH:
            if (local_ui_add_digit(key, LOCAL_ACCESS_PIN_LEN)) {
                local_ui_show();
            } else if (key == '#') {
                if (s_local_ui.input_len == LOCAL_ACCESS_PIN_LEN &&
                    strcmp(s_local_ui.input, s_local_access_pin) == 0) {
                    local_ui_set_state(LOCAL_UI_MENU);
                } else {
                    local_ui_clear_input();
                    local_ui_show_message("Access denied", "Incorrect PIN", "Returning home", "",
                                          LOCAL_UI_IDLE, LOCAL_UI_MESSAGE_MS);
                }
            }
            break;

        case LOCAL_UI_MENU:
            if (key == '1') {
                local_ui_begin_plan();
            } else if (key == '2') {
                local_ui_set_state(LOCAL_UI_NEW_PIN);
            } else if (key == '3') {
                local_ui_set_state(LOCAL_UI_SET_DATE);
            }
            break;

        case LOCAL_UI_REPLACE_WARNING:
            if (key == '#') {
                local_ui_set_state(LOCAL_UI_DAILY_COUNT);
            } else if (key == 'B') {
                local_ui_set_state(LOCAL_UI_MENU);
            }
            break;

        case LOCAL_UI_DAILY_COUNT:
            if (local_ui_add_digit(key, 1)) {
                local_ui_show();
            } else if (key == 'B') {
                local_ui_set_state(LOCAL_UI_MENU);
            } else if (key == '#') {
                value = local_ui_input_int();
                if (value >= 1 && value <= LOCAL_MAX_DAILY_TIMES) {
                    s_local_ui.daily_count = value;
                    s_local_ui.time_index = 0;
                    memset(s_local_ui.times, 0, sizeof(s_local_ui.times));
                    local_ui_set_state(LOCAL_UI_TIME);
                } else {
                    local_ui_clear_input();
                    local_ui_show_message("Invalid daily count", "Enter 1 to 4", "", "",
                                          LOCAL_UI_DAILY_COUNT, LOCAL_UI_MESSAGE_MS);
                }
            }
            break;

        case LOCAL_UI_TIME:
            if (local_ui_add_digit(key, 4)) {
                local_ui_show();
            } else if (key == 'B') {
                if (s_local_ui.time_index > 0) {
                    s_local_ui.time_index--;
                    memset(s_local_ui.times[s_local_ui.time_index], 0,
                           sizeof(s_local_ui.times[s_local_ui.time_index]));
                    local_ui_set_state(LOCAL_UI_TIME);
                } else {
                    local_ui_set_state(LOCAL_UI_DAILY_COUNT);
                }
            } else if (key == '#') {
                if (s_local_ui.input_len != 4) {
                    local_ui_clear_input();
                    local_ui_show_message("Time needs 4 digits", "Example: 0830", "means 08:30", "",
                                          LOCAL_UI_TIME, LOCAL_UI_MESSAGE_MS);
                    break;
                }
                int hour = (s_local_ui.input[0] - '0') * 10 + (s_local_ui.input[1] - '0');
                int minute = (s_local_ui.input[2] - '0') * 10 + (s_local_ui.input[3] - '0');
                if (hour > 23 || minute > 59) {
                    local_ui_clear_input();
                    local_ui_show_message("Invalid time", "Use 0000 to 2359", "24-hour clock", "",
                                          LOCAL_UI_TIME, LOCAL_UI_MESSAGE_MS);
                } else if (local_ui_time_already_entered(hour, minute)) {
                    local_ui_clear_input();
                    local_ui_show_message("Time already entered", "Choose another time", "", "",
                                          LOCAL_UI_TIME, LOCAL_UI_MESSAGE_MS);
                } else {
                    s_local_ui.times[s_local_ui.time_index][0] = hour;
                    s_local_ui.times[s_local_ui.time_index][1] = minute;
                    s_local_ui.time_index++;
                    if (s_local_ui.time_index < s_local_ui.daily_count) {
                        local_ui_set_state(LOCAL_UI_TIME);
                    } else {
                        local_ui_set_state(LOCAL_UI_QUANTITY_METHOD);
                    }
                }
            }
            break;

        case LOCAL_UI_QUANTITY_METHOD:
            if (key == '1') {
                s_local_ui.quantity_mode = LOCAL_QUANTITY_DOSES;
                local_ui_set_state(LOCAL_UI_TOTAL_DOSES);
            } else if (key == '2') {
                s_local_ui.quantity_mode = LOCAL_QUANTITY_PILLS;
                local_ui_set_state(LOCAL_UI_TOTAL_PILLS);
            } else if (key == 'B') {
                s_local_ui.time_index = s_local_ui.daily_count - 1;
                local_ui_set_state(LOCAL_UI_TIME);
            }
            break;

        case LOCAL_UI_TOTAL_DOSES:
            if (local_ui_add_digit(key, 2)) {
                local_ui_show();
            } else if (key == 'B') {
                local_ui_set_state(LOCAL_UI_QUANTITY_METHOD);
            } else if (key == '#') {
                value = local_ui_input_int();
                if (value >= 1 && value <= CAROUSEL_CHAMBERS) {
                    s_local_ui.total_doses = value;
                    if (local_ui_calculate_end()) {
                        local_ui_set_state(LOCAL_UI_REVIEW);
                    }
                } else {
                    local_ui_clear_input();
                    local_ui_show_message("Invalid dose total", "Enter 1 to 20", "Prepared dose limit", "",
                                          LOCAL_UI_TOTAL_DOSES, LOCAL_UI_MESSAGE_MS);
                }
            }
            break;

        case LOCAL_UI_TOTAL_PILLS:
            if (local_ui_add_digit(key, 3)) {
                local_ui_show();
            } else if (key == 'B') {
                local_ui_set_state(LOCAL_UI_QUANTITY_METHOD);
            } else if (key == '#') {
                value = local_ui_input_int();
                if (value >= 1 && value <= LOCAL_MAX_TOTAL_PILLS) {
                    s_local_ui.total_pills = value;
                    local_ui_set_state(LOCAL_UI_PILLS_PER_DOSE);
                } else {
                    local_ui_clear_input();
                    local_ui_show_message("Invalid pill total", "Enter 1 to 999", "", "",
                                          LOCAL_UI_TOTAL_PILLS, LOCAL_UI_MESSAGE_MS);
                }
            }
            break;

        case LOCAL_UI_PILLS_PER_DOSE:
            if (local_ui_add_digit(key, 2)) {
                local_ui_show();
            } else if (key == 'B') {
                local_ui_set_state(LOCAL_UI_TOTAL_PILLS);
            } else if (key == '#') {
                value = local_ui_input_int();
                if (value < 1 || value > LOCAL_MAX_PILLS_PER_DOSE ||
                    value > s_local_ui.total_pills) {
                    local_ui_clear_input();
                    local_ui_show_message("Invalid dose amount", "Check pills per dose", "", "",
                                          LOCAL_UI_PILLS_PER_DOSE, LOCAL_UI_MESSAGE_MS);
                    break;
                }
                s_local_ui.pills_per_dose = value;
                s_local_ui.total_doses =
                    (s_local_ui.total_pills + value - 1) / value;
                if (s_local_ui.total_doses > CAROUSEL_CHAMBERS) {
                    char needed[32];
                    snprintf(needed, sizeof(needed), "Needs %d dose times",
                             s_local_ui.total_doses);
                    local_ui_show_message("Capacity exceeded", needed,
                                          "Maximum is 20", "Adjust pill totals",
                                          LOCAL_UI_TOTAL_PILLS, LOCAL_UI_MESSAGE_MS + 700);
                } else if (local_ui_calculate_end()) {
                    local_ui_set_state(LOCAL_UI_REVIEW);
                }
            }
            break;

        case LOCAL_UI_REVIEW:
            if (key == '#') {
                local_ui_set_state(LOCAL_UI_END_REVIEW);
            } else if (key == 'B') {
                local_ui_restart_plan_edit();
            }
            break;

        case LOCAL_UI_END_REVIEW:
            if (key == 'A' || key == '#') {
                local_ui_save_schedule();
            } else if (key == 'B') {
                local_ui_set_state(LOCAL_UI_REVIEW);
            }
            break;

        case LOCAL_UI_NEW_PIN:
            if (local_ui_add_digit(key, LOCAL_ACCESS_PIN_LEN)) {
                local_ui_show();
            } else if (key == 'B') {
                local_ui_set_state(LOCAL_UI_MENU);
            } else if (key == '#') {
                if (s_local_ui.input_len == LOCAL_ACCESS_PIN_LEN) {
                    strlcpy(s_local_ui.pin_candidate, s_local_ui.input,
                            sizeof(s_local_ui.pin_candidate));
                    local_ui_set_state(LOCAL_UI_CONFIRM_PIN);
                } else {
                    local_ui_clear_input();
                    local_ui_show_message("PIN needs 4 digits", "Please try again", "", "",
                                          LOCAL_UI_NEW_PIN, LOCAL_UI_MESSAGE_MS);
                }
            }
            break;

        case LOCAL_UI_CONFIRM_PIN:
            if (local_ui_add_digit(key, LOCAL_ACCESS_PIN_LEN)) {
                local_ui_show();
            } else if (key == 'B') {
                local_ui_set_state(LOCAL_UI_NEW_PIN);
            } else if (key == '#') {
                if (s_local_ui.input_len != LOCAL_ACCESS_PIN_LEN ||
                    strcmp(s_local_ui.input, s_local_ui.pin_candidate) != 0) {
                    s_local_ui.pin_candidate[0] = '\0';
                    local_ui_clear_input();
                    local_ui_show_message("PINs did not match", "Choose PIN again", "", "",
                                          LOCAL_UI_NEW_PIN, LOCAL_UI_MESSAGE_MS);
                } else if (local_ui_store_pin(s_local_ui.pin_candidate) == ESP_OK) {
                    strlcpy(s_local_access_pin, s_local_ui.pin_candidate,
                            sizeof(s_local_access_pin));
                    s_local_ui.pin_candidate[0] = '\0';
                    local_ui_show_message("PIN changed", "Use the new PIN", "next time you enter", "",
                                          LOCAL_UI_MENU, LOCAL_UI_MESSAGE_MS + 400);
                } else {
                    local_ui_show_message("PIN not saved", "Storage error", "Please try again", "",
                                          LOCAL_UI_NEW_PIN, LOCAL_UI_MESSAGE_MS);
                }
            }
            break;

        case LOCAL_UI_SET_DATE:
            if (local_ui_add_digit(key, 8)) {
                local_ui_show();
            } else if (key == 'B') {
                local_ui_set_state(LOCAL_UI_MENU);
            } else if (key == '#') {
                if (s_local_ui.input_len != 8) {
                    local_ui_clear_input();
                    local_ui_show_message("Date needs 8 digits", "Example: 14072026", "means 14/07/2026", "",
                                          LOCAL_UI_SET_DATE, LOCAL_UI_MESSAGE_MS + 400);
                    break;
                }
                int day = (s_local_ui.input[0] - '0') * 10 + (s_local_ui.input[1] - '0');
                int month = (s_local_ui.input[2] - '0') * 10 + (s_local_ui.input[3] - '0');
                int year = atoi(&s_local_ui.input[4]);
                if (!local_ui_valid_date(year, month, day)) {
                    local_ui_clear_input();
                    local_ui_show_message("Invalid date", "Use DDMMYYYY", "Year 2024 to 2099", "",
                                          LOCAL_UI_SET_DATE, LOCAL_UI_MESSAGE_MS);
                } else {
                    s_local_ui.clock_day = day;
                    s_local_ui.clock_month = month;
                    s_local_ui.clock_year = year;
                    local_ui_set_state(LOCAL_UI_SET_CLOCK_TIME);
                }
            }
            break;

        case LOCAL_UI_SET_CLOCK_TIME:
            if (local_ui_add_digit(key, 4)) {
                local_ui_show();
            } else if (key == 'B') {
                local_ui_set_state(LOCAL_UI_SET_DATE);
            } else if (key == '#') {
                if (s_local_ui.input_len != 4) {
                    local_ui_clear_input();
                    local_ui_show_message("Time needs 4 digits", "Example: 0830", "means 08:30", "",
                                          LOCAL_UI_SET_CLOCK_TIME, LOCAL_UI_MESSAGE_MS);
                    break;
                }
                int hour = (s_local_ui.input[0] - '0') * 10 + (s_local_ui.input[1] - '0');
                int minute = (s_local_ui.input[2] - '0') * 10 + (s_local_ui.input[3] - '0');
                if (hour > 23 || minute > 59) {
                    local_ui_clear_input();
                    local_ui_show_message("Invalid time", "Use 0000 to 2359", "", "",
                                          LOCAL_UI_SET_CLOCK_TIME, LOCAL_UI_MESSAGE_MS);
                } else {
                    s_local_ui.clock_hour = hour;
                    s_local_ui.clock_minute = minute;
                    local_ui_set_state(LOCAL_UI_CLOCK_CONFIRM);
                }
            }
            break;

        case LOCAL_UI_CLOCK_CONFIRM:
            if (key == 'A' || key == '#') {
                local_ui_save_manual_clock();
            } else if (key == 'B') {
                local_ui_set_state(LOCAL_UI_SET_CLOCK_TIME);
            }
            break;

        case LOCAL_UI_MESSAGE:
        case LOCAL_UI_IDLE:
        default:
            break;
    }

    return true;
}

static void local_ui_tick(void)
{
    if (s_local_ui.state == LOCAL_UI_IDLE) return;

    scheduler_status_t status;
    if (scheduler_get_status(&status) == ESP_OK &&
        (status.activity == SCHEDULER_ACTIVITY_WAITING_FOR_HAND ||
         status.activity == SCHEDULER_ACTIVITY_DISPENSING)) {
        memset(&s_local_ui, 0, sizeof(s_local_ui));
        s_local_ui.state = LOCAL_UI_IDLE;
        s_lcd_cache_valid = false;
        ESP_LOGI(TAG, "Local setup closed because a scheduled dose is due");
        return;
    }

    int64_t now_us = esp_timer_get_time();
    if (s_local_ui.state == LOCAL_UI_MESSAGE) {
        if (now_us >= s_local_ui.message_until_us) {
            local_ui_state_t next = s_local_ui.message_next_state;
            s_local_ui.message_until_us = 0;
            if (next == LOCAL_UI_IDLE) {
                memset(&s_local_ui, 0, sizeof(s_local_ui));
                s_local_ui.state = LOCAL_UI_IDLE;
                s_lcd_cache_valid = false;
            } else {
                local_ui_set_state(next);
            }
        }
        return;
    }

    if ((now_us - s_local_ui.last_activity_us) >= (LOCAL_UI_TIMEOUT_SEC * 1000000LL)) {
        local_ui_show_message("Setup timed out", "Nothing was saved", "Returning home", "",
                              LOCAL_UI_IDLE, LOCAL_UI_MESSAGE_MS);
    }
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
                snprintf(raw[3], sizeof(raw[3]), "Hold hand steady...");
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
        snprintf(raw[2], sizeof(raw[2]), "WiFi or manual setup");
        snprintf(raw[3], sizeof(raw[3]), "Press D for setup");
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
        ESP_LOGI(TAG, "MQTT started after a valid clock became available");
    } else {
        ESP_LOGE(TAG, "MQTT start failed: %s", esp_err_to_name(ret));
    }
}

static void time_synchronized_cb(void)
{
    ESP_LOGI(TAG, "Clock ready; schedules can now be accepted safely");
    start_mqtt_once();
}

static void wifi_connected_cb(void)
{
    ESP_LOGI(TAG, "WiFi connected, synchronizing clock");
    time_service_sync_ntp();
    if (time_service_is_synchronized()) start_mqtt_once();
}

/* ========================================================================= */
/* Main application entry point                                             */
/* ========================================================================= */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting Smart Pill Dispenser v2.0");

    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);
    local_ui_storage_init();

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
        local_ui_tick();
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
