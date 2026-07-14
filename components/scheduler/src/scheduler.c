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
#define NVS_KEY_PENDING  "pending_json"

/* Maximum number of dose events that can be stored */
#define MAX_DOSES 100

/* Tanzania timezone offset (UTC+3) in seconds */
#define TZ_OFFSET_SEC   (3 * 3600)

/* Hand must be very close to the outlet before the carousel advances. */
#define HAND_DETECT_CM  7.0f
#define SCHEDULER_CHECK_INTERVAL_US 1000000LL
#define CURRENT_MINUTE_GRACE_SEC 59
#define SUCCESS_FEEDBACK_US 3000000LL
#define ERROR_FEEDBACK_US 5000000LL

/* Physical carousel layout */
#define CAROUSEL_CHAMBERS 20

/* ------------------------------------------------------------------------- */
/* Static variables – configuration and state                               */
/* ------------------------------------------------------------------------- */
static uint32_t s_steps_per_chamber = 0;      /* Steps to advance one chamber */
static uint32_t s_hand_wait_sec = 0;          /* Reminder log interval while waiting for hand */
static uint8_t s_current_chamber = 0;

typedef struct {
    time_t timestamp;           /* UTC timestamp of the dose */
    char medicine[32];          /* Medicine name */
    int remaining_pills;        /* Pills left after this dose (or -1 unlimited) */
} dose_t;

typedef enum {
    DISPENSE_RESULT_SUCCESS = 0,
    DISPENSE_RESULT_CANCELLED,
    DISPENSE_RESULT_FAILED
} dispense_result_t;

static dose_t s_doses[MAX_DOSES];
static int s_dose_count = 0;
static bool s_initialized = false;
static esp_timer_handle_t s_check_timer = NULL;
static bool s_dispensing_in_progress = false;
static bool s_waiting_for_hand = false;
static bool s_cancel_active_dispense = false;
static int64_t s_success_flash_until_us = 0;
static int64_t s_activity_until_us = 0;
static scheduler_activity_t s_activity = SCHEDULER_ACTIVITY_IDLE;
static dose_t s_active_dose;
static bool s_has_active_dose = false;
static char s_feedback_medicine[SCHEDULER_MEDICINE_NAME_LEN] = "";
static float s_hand_distance_cm = -1.0f;
static SemaphoreHandle_t s_scheduler_mutex = NULL;
static QueueHandle_t s_dispense_queue = NULL;

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

static void erase_schedule_state_from_nvs(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_key(nvs, NVS_KEY_SCHEDULE);
        nvs_erase_key(nvs, NVS_KEY_PENDING);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

static void save_pending_doses_to_nvs(void)
{
    nvs_handle_t nvs;
    cJSON *root = NULL;
    char *json = NULL;

    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return;

    if (s_dose_count <= 0) {
        nvs_erase_key(nvs, NVS_KEY_PENDING);
        nvs_erase_key(nvs, NVS_KEY_SCHEDULE);
        nvs_commit(nvs);
        nvs_close(nvs);
        return;
    }

    root = cJSON_CreateArray();
    if (!root) {
        nvs_close(nvs);
        return;
    }

    for (int i = 0; i < s_dose_count; i++) {
        cJSON *item = cJSON_CreateObject();
        if (!item) continue;
        cJSON_AddNumberToObject(item, "timestamp", (double)s_doses[i].timestamp);
        cJSON_AddStringToObject(item, "medicine", s_doses[i].medicine);
        cJSON_AddNumberToObject(item, "remaining_pills", s_doses[i].remaining_pills);
        cJSON_AddItemToArray(root, item);
    }

    json = cJSON_PrintUnformatted(root);
    if (json) {
        nvs_set_str(nvs, NVS_KEY_PENDING, json);
        nvs_commit(nvs);
        free(json);
    }

    cJSON_Delete(root);
    nvs_close(nvs);
}

static bool load_pending_doses_from_nvs(void)
{
    nvs_handle_t nvs;
    size_t len = 0;
    char *json = NULL;
    cJSON *root = NULL;
    int loaded = 0;

    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return false;
    if (nvs_get_str(nvs, NVS_KEY_PENDING, NULL, &len) != ESP_OK || len == 0) {
        nvs_close(nvs);
        return false;
    }

    json = malloc(len);
    if (!json) {
        nvs_close(nvs);
        return false;
    }

    if (nvs_get_str(nvs, NVS_KEY_PENDING, json, &len) != ESP_OK) {
        free(json);
        nvs_close(nvs);
        return false;
    }
    nvs_close(nvs);

    root = cJSON_Parse(json);
    free(json);
    if (!root || !cJSON_IsArray(root)) {
        if (root) cJSON_Delete(root);
        return false;
    }

    for (int i = 0; i < cJSON_GetArraySize(root) && loaded < MAX_DOSES; i++) {
        cJSON *item = cJSON_GetArrayItem(root, i);
        cJSON *timestamp = cJSON_GetObjectItem(item, "timestamp");
        cJSON *medicine = cJSON_GetObjectItem(item, "medicine");
        cJSON *remaining = cJSON_GetObjectItem(item, "remaining_pills");

        if (!cJSON_IsNumber(timestamp) || !cJSON_IsString(medicine)) continue;

        s_doses[loaded].timestamp = (time_t)timestamp->valuedouble;
        strlcpy(s_doses[loaded].medicine, medicine->valuestring, sizeof(s_doses[loaded].medicine));
        s_doses[loaded].remaining_pills = cJSON_IsNumber(remaining) ? remaining->valueint : -1;
        loaded++;
    }

    cJSON_Delete(root);
    s_dose_count = loaded;
    ESP_LOGI(TAG, "Loaded %d persisted pending dose(s)", loaded);
    return loaded > 0;
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
        return -1;
    }
    if (!cJSON_IsArray(root)) {
        ESP_LOGE(TAG, "Schedule root must be an array");
        cJSON_Delete(root);
        return -1;
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

                /* Minute-only schedules received during their current minute are due now. */
                if ((dose_utc + CURRENT_MINUTE_GRACE_SEC) < now_utc) continue;

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
/* Dispense routine: wait for hand, rotate, persist, and publish state.     */
/* ------------------------------------------------------------------------- */
static dispense_result_t dispense_medicine(const dose_t *dose)
{
    esp_err_t step_ret;

    ESP_LOGI(TAG, "Unlocking bin...");
    ESP_LOGI(TAG, "Dose due: waiting for hand within %.1f cm to dispense %s",
             HAND_DETECT_CM, dose->medicine);

    if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
    s_waiting_for_hand = true;
    s_activity = SCHEDULER_ACTIVITY_WAITING_FOR_HAND;
    s_activity_until_us = 0;
    s_hand_distance_cm = -1.0f;
    if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);

    int64_t last_feedback_time = 0;
    while (true) {
        bool cancel_requested = false;
        if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
        cancel_requested = s_cancel_active_dispense;
        if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);

        if (cancel_requested) {
            ESP_LOGW(TAG, "Dose wait cancelled before hand detection");
            if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
            s_waiting_for_hand = false;
            s_cancel_active_dispense = false;
            s_activity = SCHEDULER_ACTIVITY_IDLE;
            s_hand_distance_cm = -1.0f;
            if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);
            return DISPENSE_RESULT_CANCELLED;
        }

        float dist = ultrasonic_measure_blocking();
        if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
        s_hand_distance_cm = dist;
        if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);
        if (dist > 0.1f && dist <= HAND_DETECT_CM) {
            ESP_LOGI(TAG, "Hand detected at %.1f cm", dist);
            break;
        }

        int64_t now_us = esp_timer_get_time();
        if ((now_us - last_feedback_time) >= (s_hand_wait_sec * 1000000LL)) {
            last_feedback_time = now_us;
            if (dist > 0.1f) {
                ESP_LOGI(TAG, "Still waiting for hand: %.1f cm, need <= %.1f cm",
                         dist, HAND_DETECT_CM);
            } else {
                ESP_LOGI(TAG, "Still waiting for hand: ultrasonic out of range");
            }

        }

        vTaskDelay(pdMS_TO_TICKS(150));
    }

    if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
    s_waiting_for_hand = false;
    s_cancel_active_dispense = false;
    s_activity = SCHEDULER_ACTIVITY_DISPENSING;
    s_hand_distance_cm = -1.0f;
    if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);

    stepper_motor_set_direction(true);
    step_ret = stepper_motor_rotate_steps(s_steps_per_chamber);
    if (step_ret != ESP_OK) {
        ESP_LOGE(TAG, "Stepper failed while dispensing %s: %s",
                 dose->medicine, esp_err_to_name(step_ret));
        mqtt_manager_publish_log("error", dose->medicine,
                                 dose->remaining_pills, "stepper failed");
        if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
        strlcpy(s_feedback_medicine, dose->medicine, sizeof(s_feedback_medicine));
        s_activity = SCHEDULER_ACTIVITY_ERROR;
        s_activity_until_us = esp_timer_get_time() + ERROR_FEEDBACK_US;
        if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);
        return DISPENSE_RESULT_FAILED;
    }

    if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
    s_current_chamber = (s_current_chamber + 1) % CAROUSEL_CHAMBERS;
    strlcpy(s_feedback_medicine, dose->medicine, sizeof(s_feedback_medicine));
    s_activity = SCHEDULER_ACTIVITY_SUCCESS;
    s_activity_until_us = esp_timer_get_time() + SUCCESS_FEEDBACK_US;
    s_success_flash_until_us = s_activity_until_us;
    if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);

    save_chamber_to_nvs();
    save_pending_doses_to_nvs();
    ESP_LOGI(TAG, "Dispensed %s, next chamber=%u", dose->medicine, s_current_chamber);

    ESP_LOGI(TAG, "Relocking bin");
    mqtt_manager_publish_log("dispensed", dose->medicine,
                             dose->remaining_pills - 1, "auto");

    vTaskDelay(pdMS_TO_TICKS(2000));
    return DISPENSE_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* Timer callback - checks once per second for due doses.                   */
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
    if (require_due_time && !time_service_is_synchronized()) return false;

    dose_t dose;
    bool should_queue = false;
    time_t evaluated_at_utc = 0;

    xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
    if (!s_dispensing_in_progress && s_dose_count > 0) {
        if (!require_due_time) {
            should_queue = true;
        } else {
            time_service_get_timestamp(&evaluated_at_utc);
            should_queue = (evaluated_at_utc >= s_doses[0].timestamp);
        }

        if (should_queue) {
            pop_first_dose_locked(&dose);
            s_dispensing_in_progress = true;
            s_active_dose = dose;
            s_has_active_dose = true;
            s_activity = SCHEDULER_ACTIVITY_WAITING_FOR_HAND;
            s_activity_until_us = 0;
            s_hand_distance_cm = -1.0f;
        }
    }
    xSemaphoreGive(s_scheduler_mutex);

    if (!should_queue) return false;

    if (xQueueSend(s_dispense_queue, &dose, 0) == pdTRUE) {
        if (require_due_time) {
            ESP_LOGI(TAG,
                     "Dose queued on time: %s, scheduled=%lld, triggered=%lld, lag=%lld sec",
                     dose.medicine, (long long)dose.timestamp,
                     (long long)evaluated_at_utc,
                     (long long)(evaluated_at_utc - dose.timestamp));
        } else {
            ESP_LOGI(TAG, "Dose queued manually: %s", dose.medicine);
        }
        return true;
    }

    xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
    restore_first_dose_locked(&dose);
    s_dispensing_in_progress = false;
    s_has_active_dose = false;
    s_activity = SCHEDULER_ACTIVITY_IDLE;
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
            dispense_result_t result = dispense_medicine(&dose);

            xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
            if (result == DISPENSE_RESULT_FAILED) {
                restore_first_dose_locked(&dose);
            }
            s_has_active_dose = false;
            s_dispensing_in_progress = false;
            xSemaphoreGive(s_scheduler_mutex);

            if (result == DISPENSE_RESULT_FAILED) {
                save_pending_doses_to_nvs();
            }
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

    load_pending_doses_from_nvs();

    /* Check every second so minute-based schedules become due promptly. */
    const esp_timer_create_args_t timer_args = {
        .callback = check_timer_cb,
        .name = "scheduler_check"
    };
    esp_err_t ret = esp_timer_create(&timer_args, &s_check_timer);
    if (ret != ESP_OK) return ret;
    ret = esp_timer_start_periodic(s_check_timer, SCHEDULER_CHECK_INTERVAL_US);
    if (ret != ESP_OK) {
        esp_timer_delete(s_check_timer);
        s_check_timer = NULL;
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Scheduler initialized: %d pending doses, 1 sec due check, steps/chamber=%lu, chamber=%u, hand reminder=%lu sec",
             s_dose_count, (unsigned long)s_steps_per_chamber, s_current_chamber,
             (unsigned long)s_hand_wait_sec);
    return ESP_OK;
}

esp_err_t scheduler_set_schedule(const char *schedule_json)
{
    if (!schedule_json) return ESP_ERR_INVALID_ARG;
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!time_service_is_synchronized()) return ESP_ERR_INVALID_STATE;

    cJSON *validation = cJSON_Parse(schedule_json);
    if (!validation || !cJSON_IsArray(validation)) {
        if (validation) cJSON_Delete(validation);
        return ESP_ERR_INVALID_ARG;
    }
    cJSON_Delete(validation);

    if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);

    /* Clear existing schedule */
    s_dose_count = 0;
    if (s_activity == SCHEDULER_ACTIVITY_WAITING_FOR_HAND) {
        s_cancel_active_dispense = true;
    } else if (s_activity != SCHEDULER_ACTIVITY_DISPENSING) {
        s_activity = SCHEDULER_ACTIVITY_IDLE;
        s_activity_until_us = 0;
        s_feedback_medicine[0] = '\0';
    }

    /* Parse the new JSON and generate future doses */
    int count = generate_doses_from_json(schedule_json);
    if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);

    if (count < 0) {
        ESP_LOGE(TAG, "Schedule could not be parsed");
        return ESP_ERR_INVALID_ARG;
    }

    if (count == 0) {
        ESP_LOGW(TAG, "No future doses generated (all times are in the past)");
        erase_schedule_state_from_nvs();
        return ESP_ERR_NOT_FOUND;
    }

    /* Save the raw JSON to NVS for persistence across reboots */
    save_schedule_to_nvs(schedule_json);
    save_pending_doses_to_nvs();

    ESP_LOGI(TAG, "Schedule updated: %d future doses pending", count);
    queue_next_dose_for_dispense(true);
    return ESP_OK;
}

void scheduler_clear(void)
{
    if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
    s_dose_count = 0;
    if (s_activity == SCHEDULER_ACTIVITY_WAITING_FOR_HAND) {
        s_cancel_active_dispense = true;
    } else if (s_activity != SCHEDULER_ACTIVITY_DISPENSING) {
        s_activity = SCHEDULER_ACTIVITY_IDLE;
        s_activity_until_us = 0;
        s_feedback_medicine[0] = '\0';
    }
    if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);

    erase_schedule_state_from_nvs();
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

    ESP_LOGW(TAG, "Calibration move: %lu STEP pulses", (unsigned long)steps);
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
    pending = s_dose_count + (s_has_active_dose ? 1 : 0);
    if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);
    return pending;
}

bool scheduler_get_next_due_time(time_t *timestamp)
{
    bool has_next = false;
    if (!timestamp) return false;

    if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
    if (s_has_active_dose &&
        (s_activity == SCHEDULER_ACTIVITY_WAITING_FOR_HAND ||
         s_activity == SCHEDULER_ACTIVITY_DISPENSING)) {
        *timestamp = s_active_dose.timestamp;
        has_next = true;
    } else if (s_dose_count > 0) {
        *timestamp = s_doses[0].timestamp;
        has_next = true;
    }
    if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);

    return has_next;
}

bool scheduler_is_waiting_for_hand(void)
{
    bool waiting;
    if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);
    waiting = s_waiting_for_hand;
    if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);
    return waiting;
}

bool scheduler_success_flash_active(void)
{
    return esp_timer_get_time() < s_success_flash_until_us;
}

esp_err_t scheduler_get_status(scheduler_status_t *status)
{
    if (!status) return ESP_ERR_INVALID_ARG;

    memset(status, 0, sizeof(*status));
    status->hand_distance_cm = -1.0f;

    if (s_scheduler_mutex) xSemaphoreTake(s_scheduler_mutex, portMAX_DELAY);

    int64_t now_us = esp_timer_get_time();
    if ((s_activity == SCHEDULER_ACTIVITY_SUCCESS ||
         s_activity == SCHEDULER_ACTIVITY_ERROR) &&
        s_activity_until_us > 0 && now_us >= s_activity_until_us) {
        s_activity = SCHEDULER_ACTIVITY_IDLE;
        s_activity_until_us = 0;
        s_feedback_medicine[0] = '\0';
    }

    status->activity = s_activity;
    status->pending_count = s_dose_count + (s_has_active_dose ? 1 : 0);
    status->current_chamber = s_current_chamber;
    status->hand_distance_cm = s_hand_distance_cm;

    if (s_activity == SCHEDULER_ACTIVITY_WAITING_FOR_HAND ||
        s_activity == SCHEDULER_ACTIVITY_DISPENSING) {
        if (s_has_active_dose) {
            strlcpy(status->active_medicine, s_active_dose.medicine,
                    sizeof(status->active_medicine));
        }
    } else if (s_activity == SCHEDULER_ACTIVITY_SUCCESS ||
               s_activity == SCHEDULER_ACTIVITY_ERROR) {
        strlcpy(status->active_medicine, s_feedback_medicine,
                sizeof(status->active_medicine));
    }

    if (s_dose_count > 0) {
        status->has_next_dose = true;
        status->next_due_time = s_doses[0].timestamp;
        strlcpy(status->next_medicine, s_doses[0].medicine,
                sizeof(status->next_medicine));
    }

    if (s_scheduler_mutex) xSemaphoreGive(s_scheduler_mutex);
    return ESP_OK;
}
