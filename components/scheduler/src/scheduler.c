/**
 * @file scheduler.c
 * @brief Medicine schedule manager – stores, checks, and triggers doses.
 * 
 * This component parses a JSON schedule (as defined in the MQTT contract),
 * generates future dose events (local time Tanzania, UTC+3), stores them in
 * NVS, and runs a periodic timer to check for due doses. When a dose is due,
 * it controls the stepper motor and ultrasonic sensor to dispense the pill.
 * 
 * @author Matthithyahu
 * @date 2026-05-20
 * @version 2.1
 */

#include "scheduler.h"
#include "time_service.h"
#include "stepper_motor.h"
#include "ultrasonic.h"
#include "mqtt_manager.h"
#include "lcd_i2c.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "SCHEDULER";

/* NVS storage keys */
#define NVS_NAMESPACE   "scheduler"
#define NVS_KEY_SCHEDULE "schedule_json"
#define NVS_KEY_CHAMBER  "chamber_index"

/* Maximum number of dose events that can be stored */
#define MAX_DOSES 100

/* Tanzania timezone offset (UTC+3) in seconds */
#define TZ_OFFSET_SEC   (3 * 3600)

/* Hand detection distance threshold (cm) */
#define HAND_DETECT_CM  15.0f

/* Physical carousel layout */
#define CAROUSEL_CHAMBERS 20

/* ------------------------------------------------------------------------- */
/* Static variables – configuration and state                               */
/* ------------------------------------------------------------------------- */
static uint32_t s_steps_per_chamber = 0;      /* Steps to advance one chamber */
static uint32_t s_hand_wait_sec = 0;          /* Seconds to wait for hand */
static uint8_t s_current_chamber = 0;

typedef struct {
    time_t timestamp;           /* UTC timestamp of the dose */
    char medicine[32];          /* Medicine name */
    int remaining_pills;        /* Pills left after this dose (or -1 unlimited) */
} dose_t;

static dose_t s_doses[MAX_DOSES];
static int s_dose_count = 0;
static bool s_initialized = false;
static esp_timer_handle_t s_check_timer = NULL;
static bool s_dispensing_in_progress = false;
static SemaphoreHandle_t s_scheduler_mutex = NULL;
static QueueHandle_t s_dispense_queue = NULL;

/* External LCD handle and mutex (set from main.c) */
extern lcd_handle_t *s_lcd;
extern SemaphoreHandle_t s_lcd_mutex;

static bool queue_next_dose_for_dispense(bool require_due_time);
static void dispense_task(void *arg);

/* ------------------------------------------------------------------------- */
/* Helper: convert year, month, day, hour, minute, second to UTC timestamp   */
/* This is a portable replacement for the missing _mkgmtime().               */
/* ------------------------------------------------------------------------- */
static time_t make_utc_timestamp(int year, int month, int day, int hour, int min, int sec)
{
    static const int days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    time_t days = 0;
    /* Years from 1970 to year-1 */
    for (int y = 1970; y < year; y++) {
        days += 365 + ((y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 1 : 0);
    }
    /* Months from January to month-1 */
    for (int m = 1; m < month; m++) {
        days += days_in_month[m-1];
        if (m == 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0))) days += 1;
    }
    days += day - 1;
    return days * 86400 + hour * 3600 + min * 60 + sec;
}

/* ------------------------------------------------------------------------- */
/* Convert local time (Tanzania, UTC+3) to UTC timestamp.                    */
/* The user thinks in local time; we store UTC internally.                   */
/* ------------------------------------------------------------------------- */
static time_t local_to_utc(int year, int month, int day, int hour, int min, int sec)
{
    time_t utc = make_utc_timestamp(year, month, day, hour, min, sec);
    return utc - TZ_OFFSET_SEC;
}

/* ------------------------------------------------------------------------- */
/* Parse a time string "HH:MM" and combine with a local date to produce a   */
/* UTC timestamp. The date is assumed to be local.                           */
/* ------------------------------------------------------------------------- */
static time_t make_dose_timestamp(int year, int month, int day, const char *time_str)
{
    int hour, min;
    if (sscanf(time_str, "%d:%d", &hour, &min) != 2) return 0;
    return local_to_utc(year, month, day, hour, min, 0);
}

/* ------------------------------------------------------------------------- */
/* NVS helpers – store and load the raw schedule JSON                        */
/* ------------------------------------------------------------------------- */
static void save_schedule_to_nvs(const char *json)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_str(nvs, NVS_KEY_SCHEDULE, json);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

static char* load_schedule_from_nvs(void)
{
    nvs_handle_t nvs;
    size_t len;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return NULL;
    if (nvs_get_str(nvs, NVS_KEY_SCHEDULE, NULL, &len) != ESP_OK) {
        nvs_close(nvs);
        return NULL;
    }
    char *buf = malloc(len);
    if (buf) {
        nvs_get_str(nvs, NVS_KEY_SCHEDULE, buf, &len);
    }
    nvs_close(nvs);
    return buf;
}

static void save_chamber_to_nvs(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, NVS_KEY_CHAMBER, s_current_chamber);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

static void load_chamber_from_nvs(void)
{
    nvs_handle_t nvs;
    uint8_t chamber = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        if (nvs_get_u8(nvs, NVS_KEY_CHAMBER, &chamber) == ESP_OK && chamber < CAROUSEL_CHAMBERS) {
            s_current_chamber = chamber;
        }
        nvs_close(nvs);
    }
}

/* ------------------------------------------------------------------------- */
/* Parse JSON schedule and generate dose events (future only).               */
/* Returns number of generated doses.                                        */
/* ------------------------------------------------------------------------- */
static int generate_doses_from_json(const char *json)
{
    cJSON *root = cJSON_Parse(json);
    if (!root) {
        ESP_LOGE(TAG, "Invalid JSON schedule");
        return 0;
    }
    if (!cJSON_IsArray(root)) {
        ESP_LOGE(TAG, "Schedule root must be an array");
        cJSON_Delete(root);
        return 0;
    }

    int total = 0;
    /* Get current local time (Tanzania) */
    struct tm local_now;
    time_service_get_tm(&local_now);
    int current_year = local_now.tm_year + 1900;
    int current_month = local_now.tm_mon + 1;
    int current_day = local_now.tm_mday;

    /* Current UTC timestamp for comparison */
    time_t now_utc;
    time_service_get_timestamp(&now_utc);

    for (int i = 0; i < cJSON_GetArraySize(root); i++) {
        cJSON *entry = cJSON_GetArrayItem(root, i);
        cJSON *name = cJSON_GetObjectItem(entry, "name");
        cJSON *times = cJSON_GetObjectItem(entry, "times");
        cJSON *duration = cJSON_GetObjectItem(entry, "duration_days");
        cJSON *total_pills = cJSON_GetObjectItem(entry, "total_pills");
        cJSON *start_date = cJSON_GetObjectItem(entry, "start_date");

        if (!name || !cJSON_IsString(name) ||
            !times || !cJSON_IsArray(times) ||
            !duration || !cJSON_IsNumber(duration)) {
            ESP_LOGW(TAG, "Skipping invalid schedule entry");
            continue;
        }

        const char *med_name = name->valuestring;
        int duration_days = duration->valueint;
        int pills_available = total_pills ? total_pills->valueint : -1;

        /* Determine start date (local) */
        int year = current_year, month = current_month, day = current_day;
        if (start_date && cJSON_IsString(start_date)) {
            if (sscanf(start_date->valuestring, "%d-%d-%d", &year, &month, &day) != 3) {
                ESP_LOGW(TAG, "Invalid start_date, using today");
                year = current_year; month = current_month; day = current_day;
            }
        }

        /* If duration is 0, treat as indefinite (10 years of days) */
        int days_to_run = (duration_days == 0) ? 3650 : duration_days;

        for (int d = 0; d < days_to_run; d++) {
            /* Compute local date for this day */
            time_t day_ts = make_utc_timestamp(year, month, day + d, 0, 0, 0);
            struct tm day_tm;
            gmtime_r(&day_ts, &day_tm);
            int dose_year = day_tm.tm_year + 1900;
            int dose_month = day_tm.tm_mon + 1;
            int dose_day = day_tm.tm_mday;

            /* Skip if this day is in the past (local date) */
            if (dose_year < current_year) continue;
            if (dose_year == current_year && dose_month < current_month) continue;
            if (dose_year == current_year && dose_month == current_month && dose_day < current_day) continue;

            for (int t = 0; t < cJSON_GetArraySize(times); t++) {
                cJSON *time_str = cJSON_GetArrayItem(times, t);
                if (!cJSON_IsString(time_str)) continue;

                time_t dose_utc = make_dose_timestamp(dose_year, dose_month, dose_day, time_str->valuestring);
                if (dose_utc == 0) continue;

                /* Skip doses already in the past (UTC comparison) */
                if (dose_utc < now_utc) continue;

                if (total >= MAX_DOSES) {
                    ESP_LOGW(TAG, "Maximum dose limit reached, ignoring remaining");
                    break;
                }

                s_doses[total].timestamp = dose_utc;
                strlcpy(s_doses[total].medicine, med_name, sizeof(s_doses[total].medicine));
                s_doses[total].remaining_pills = pills_available;
                total++;

                if (pills_available > 0) {
                    pills_available--;
                    if (pills_available == 0) break;
                }
            }
            if (pills_available == 0) break;
        }
    }
    cJSON_Delete(root);

    /* Sort doses by timestamp */
    for (int i = 0; i < total - 1; i++) {
        for (int j = i + 1; j < total; j++) {
            if (s_doses[i].timestamp > s_doses[j].timestamp) {
                dose_t tmp = s_doses[i];
                s_doses[i] = s_doses[j];
                s_doses[j] = tmp;
            }
        }
    }
    s_dose_count = total;
    ESP_LOGI(TAG, "Generated %d future dose events", total);
    return total;
}

/* ------------------------------------------------------------------------- */
/* Dispense routine: unlock, wait for hand, rotate, lock, log.              */
/* Shows user guidance on LCD.                                              */
/* ------------------------------------------------------------------------- */
static void dispense_medicine(const dose_t *dose)
{
    /* Show guidance on LCD */
    if (s_lcd && s_lcd_mutex) {
        xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
        lcd_set_cursor(s_lcd, 2, 0);
        lcd_printf(s_lcd, "Take %s            ", dose->medicine);
        lcd_set_cursor(s_lcd, 3, 0);
        lcd_printf(s_lcd, "Place hand near    ");
        xSemaphoreGive(s_lcd_mutex);
    }

    /* 1. Unlock (placeholder) */
    ESP_LOGI(TAG, "Unlocking bin...");

    /* 2. Wait for hand detection (ultrasonic distance < threshold) */
    ESP_LOGI(TAG, "Waiting for hand to take %s", dose->medicine);
    int64_t start_time = esp_timer_get_time();
    bool hand_detected = false;
    while ((esp_timer_get_time() - start_time) < (s_hand_wait_sec * 1000000LL)) {
        float dist = ultrasonic_measure_blocking();
        if (dist > 0.1f && dist < HAND_DETECT_CM) {
            hand_detected = true;
            ESP_LOGI(TAG, "Hand detected at %.1f cm", dist);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (hand_detected) {
        /* 3. Rotate stepper by one chamber */
        stepper_motor_set_direction(true);
        esp_err_t step_ret = stepper_motor_rotate_steps(s_steps_per_chamber);
        if (step_ret != ESP_OK) {
            ESP_LOGE(TAG, "Stepper failed while dispensing %s: %s",
                     dose->medicine, esp_err_to_name(step_ret));
            mqtt_manager_publish_log("error", dose->medicine,
                                     dose->remaining_pills, "stepper failed");
            if (s_lcd && s_lcd_mutex) {
                xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
                lcd_set_cursor(s_lcd, 2, 0);
                lcd_printf(s_lcd, "Motor error       ");
                lcd_set_cursor(s_lcd, 3, 0);
                lcd_printf(s_lcd, "Dose not moved    ");
                xSemaphoreGive(s_lcd_mutex);
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
            return;
        }

        s_current_chamber = (s_current_chamber + 1) % CAROUSEL_CHAMBERS;
        save_chamber_to_nvs();
        ESP_LOGI(TAG, "Dispensed %s, next chamber=%u", dose->medicine, s_current_chamber);

        /* 4. Relock (placeholder) */
        ESP_LOGI(TAG, "Relocking bin");

        /* 5. Publish log via MQTT */
        mqtt_manager_publish_log("dispensed", dose->medicine,
                                 dose->remaining_pills - 1, "auto");

        /* Show success on LCD */
        if (s_lcd && s_lcd_mutex) {
            xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
            lcd_set_cursor(s_lcd, 2, 0);
            lcd_printf(s_lcd, "Dispensed %s      ", dose->medicine);
            lcd_set_cursor(s_lcd, 3, 0);
            lcd_printf(s_lcd, "Thank you!        ");
            xSemaphoreGive(s_lcd_mutex);
        }
    } else {
        /* Timeout – no hand */
        ESP_LOGW(TAG, "Hand not detected, dose missed");
        mqtt_manager_publish_log("missed", dose->medicine,
                                 dose->remaining_pills, "hand timeout");

        if (s_lcd && s_lcd_mutex) {
            xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
            lcd_set_cursor(s_lcd, 2, 0);
            lcd_printf(s_lcd, "Missed dose!      ");
            lcd_set_cursor(s_lcd, 3, 0);
            lcd_printf(s_lcd, "Take later        ");
            xSemaphoreGive(s_lcd_mutex);
        }
    }

    /* Brief delay to let the user read the message, then restore normal display */
    vTaskDelay(pdMS_TO_TICKS(2000));
}

/* ------------------------------------------------------------------------- */
/* Timer callback – checks every 60 seconds for due doses.                  */
/* ------------------------------------------------------------------------- */
static void check_timer_cb(void *arg)
{
    (void)arg;
    queue_next_dose_for_dispense(true);
}

static bool pop_first_dose_locked(dose_t *dose)
{
    if (s_dose_count == 0 || dose == NULL) return false;

    *dose = s_doses[0];
    for (int i = 0; i < s_dose_count - 1; i++) {
        s_doses[i] = s_doses[i + 1];
    }
    s_dose_count--;
    return true;
}

static void restore_first_dose_locked(const dose_t *dose)
{
    if (dose == NULL || s_dose_count >= MAX_DOSES) return;

    for (int i = s_dose_count; i > 0; i--) {
        s_doses[i] = s_doses[i - 1];
    }
    s_doses[0] = *dose;
    s_dose_count++;
}

static bool queue_next_dose_for_dispense(bool require_due_time)
{
    if (!s_scheduler_mutex || !s_dispense_queue) return false;

    dose_t dose;
    bool should_queue = false;

    xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
    if (!s_dispensing_in_progress && s_dose_count > 0) {
        if (!require_due_time) {
            should_queue = true;
        } else {
            time_t now_utc;
            time_service_get_timestamp(&now_utc);
            should_queue = (now_utc >= s_doses[0].timestamp);
        }

        if (should_queue) {
            pop_first_dose_locked(&dose);
            s_dispensing_in_progress = true;
        }
    }
    xSemaphoreGive(s_scheduler_mutex);

    if (!should_queue) return false;

    if (xQueueSend(s_dispense_queue, &dose, 0) == pdTRUE) {
        return true;
    }

    xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
    restore_first_dose_locked(&dose);
    s_dispensing_in_progress = false;
    xSemaphoreGive(s_scheduler_mutex);

    ESP_LOGE(TAG, "Dispense queue full; dose restored");
    return false;
}

static void dispense_task(void *arg)
{
    (void)arg;
    dose_t dose;

    while (1) {
        if (xQueueReceive(s_dispense_queue, &dose, portMAX_DELAY) == pdTRUE) {
            dispense_medicine(&dose);

            xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
            s_dispensing_in_progress = false;
            xSemaphoreGive(s_scheduler_mutex);
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */
esp_err_t scheduler_init(uint32_t steps_per_chamber, uint32_t hand_wait_timeout_sec)
{
    if (steps_per_chamber == 0 || hand_wait_timeout_sec == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    s_steps_per_chamber = steps_per_chamber;
    s_hand_wait_sec = hand_wait_timeout_sec;

    s_scheduler_mutex = xSemaphoreCreateMutex();
    if (!s_scheduler_mutex) return ESP_ERR_NO_MEM;

    s_dispense_queue = xQueueCreate(1, sizeof(dose_t));
    if (!s_dispense_queue) return ESP_ERR_NO_MEM;

    if (xTaskCreate(dispense_task, "dispense_task", 4096, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    load_chamber_from_nvs();

    /* Load any previously saved schedule from NVS */
    char *saved_json = load_schedule_from_nvs();
    if (saved_json) {
        ESP_LOGI(TAG, "Loading saved schedule from NVS");
        generate_doses_from_json(saved_json);
        free(saved_json);
    }

    /* Create periodic timer (60 seconds) */
    const esp_timer_create_args_t timer_args = {
        .callback = check_timer_cb,
        .name = "scheduler_check"
    };
    esp_err_t ret = esp_timer_create(&timer_args, &s_check_timer);
    if (ret != ESP_OK) return ret;
    esp_timer_start_periodic(s_check_timer, 60 * 1000000LL);

    s_initialized = true;
    ESP_LOGI(TAG, "Scheduler initialized: %d pending doses, steps/chamber=%lu, chamber=%u, hand wait=%lu sec",
             s_dose_count, (unsigned long)s_steps_per_chamber, s_current_chamber,
             (unsigned long)s_hand_wait_sec);
    return ESP_OK;
}

esp_err_t scheduler_set_schedule(const char *schedule_json)
{
    if (!schedule_json) return ESP_ERR_INVALID_ARG;

    if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);

    /* Clear existing schedule */
    s_dose_count = 0;

    /* Parse the new JSON and generate future doses */
    int count = generate_doses_from_json(schedule_json);
    if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);

    if (count == 0) {
        ESP_LOGW(TAG, "No future doses generated (all times are in the past)");
        /* Still consider the JSON valid – just no future doses */
        return ESP_OK;
    }

    /* Save the raw JSON to NVS for persistence across reboots */
    save_schedule_to_nvs(schedule_json);

    ESP_LOGI(TAG, "Schedule updated: %d future doses pending", count);
    return ESP_OK;
}

void scheduler_clear(void)
{
    if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
    s_dose_count = 0;
    if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);

    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_key(nvs, NVS_KEY_SCHEDULE);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_LOGI(TAG, "All schedules cleared");
}

void scheduler_dispense_now(void)
{
    if (!queue_next_dose_for_dispense(false)) {
        ESP_LOGW(TAG, "No pending doses to dispense");
    }
}

void scheduler_calibration_move_steps(uint32_t steps)
{
    if (steps == 0) return;

    if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
    if (s_dispensing_in_progress) {
        if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);
        ESP_LOGW(TAG, "Calibration move ignored; dispensing is active");
        return;
    }
    s_dispensing_in_progress = true;
    if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);

    ESP_LOGW(TAG, "Calibration move: %lu full steps", (unsigned long)steps);
    stepper_motor_set_direction(true);
    esp_err_t ret = stepper_motor_rotate_steps(steps);

    if (ret == ESP_OK && steps == s_steps_per_chamber) {
        s_current_chamber = (s_current_chamber + 1) % CAROUSEL_CHAMBERS;
        save_chamber_to_nvs();
        ESP_LOGI(TAG, "Calibration chamber advance complete, next chamber=%u", s_current_chamber);
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Calibration move failed: %s", esp_err_to_name(ret));
    }

    if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
    s_dispensing_in_progress = false;
    if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);
}

int scheduler_get_pending_count(void)
{
    int pending;
    if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
    pending = s_dose_count;
    if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);
    return pending;
}
