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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "SCHEDULER";

#define NVS_NAMESPACE   "scheduler"
#define NVS_KEY_SCHEDULE "schedule_json"

static uint32_t s_steps_per_chamber = 768;
static uint32_t s_hand_wait_sec = 10;

#define MAX_DOSES 100
typedef struct {
    time_t timestamp;
    char medicine[32];
    int remaining_pills;
} dose_t;

static dose_t s_doses[MAX_DOSES];
static int s_dose_count = 0;
static bool s_initialized = false;
static esp_timer_handle_t s_check_timer = NULL;
static bool s_dispensing_in_progress = false;

// -------------------------------------------------------------
// Helper: convert YMD H:M:S to Unix timestamp (UTC)
// -------------------------------------------------------------
static time_t make_utc_timestamp(int year, int month, int day, int hour, int min, int sec)
{
    static const int days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    time_t days = 0;
    for (int y = 1970; y < year; y++) {
        days += 365 + ((y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 1 : 0);
    }
    for (int m = 1; m < month; m++) {
        days += days_in_month[m-1];
        if (m == 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0))) days += 1;
    }
    days += day - 1;
    return days * 86400 + hour * 3600 + min * 60 + sec;
}

static time_t make_timestamp(int year, int month, int day, const char *time_str)
{
    int hour, min;
    if (sscanf(time_str, "%d:%d", &hour, &min) != 2) return 0;
    return make_utc_timestamp(year, month, day, hour, min, 0);
}

// -------------------------------------------------------------
// NVS helpers
// -------------------------------------------------------------
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

// -------------------------------------------------------------
// Schedule parsing and generation
// -------------------------------------------------------------
static int generate_doses_from_json(const char *json)
{
    cJSON *root = cJSON_Parse(json);
    if (!root) {
        ESP_LOGE(TAG, "Invalid JSON schedule");
        return 0;
    }
    if (!cJSON_IsArray(root)) {
        ESP_LOGE(TAG, "Schedule root must be array");
        cJSON_Delete(root);
        return 0;
    }
    int total = 0;
    time_t now;
    time_service_get_timestamp(&now);
    struct tm now_tm;
    gmtime_r(&now, &now_tm);   // UTC
    int current_year = now_tm.tm_year + 1900;
    int current_month = now_tm.tm_mon + 1;
    int current_day = now_tm.tm_mday;

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
            ESP_LOGW(TAG, "Skipping invalid entry");
            continue;
        }
        const char *med_name = name->valuestring;
        int duration_days = duration->valueint;
        int pills_available = total_pills ? total_pills->valueint : -1;

        int year = current_year, month = current_month, day = current_day;
        if (start_date && cJSON_IsString(start_date)) {
            if (sscanf(start_date->valuestring, "%d-%d-%d", &year, &month, &day) != 3) {
                ESP_LOGW(TAG, "Invalid start_date, using today");
            }
        }

        int days_to_run = (duration_days == 0) ? 365*10 : duration_days;
        for (int d = 0; d < days_to_run; d++) {
            // compute current day's timestamp
            time_t day_ts = make_utc_timestamp(year, month, day + d, 0, 0, 0);
            struct tm day_tm;
            gmtime_r(&day_ts, &day_tm);
            if (duration_days == 0 && day_ts < now) continue; // skip past days for indefinite

            for (int t = 0; t < cJSON_GetArraySize(times); t++) {
                cJSON *time_str = cJSON_GetArrayItem(times, t);
                if (!cJSON_IsString(time_str)) continue;
                time_t ts = make_timestamp(day_tm.tm_year+1900, day_tm.tm_mon+1, day_tm.tm_mday, time_str->valuestring);
                if (ts < now) continue;
                if (total >= MAX_DOSES) {
                    ESP_LOGW(TAG, "Dose limit reached");
                    break;
                }
                s_doses[total].timestamp = ts;
                strlcpy(s_doses[total].medicine, med_name, sizeof(s_doses[total].medicine));
                s_doses[total].remaining_pills = pills_available;
                total++;
                if (pills_available > 0) pills_available--;
                if (pills_available == 0) break;
            }
        }
    }
    cJSON_Delete(root);

    // sort doses by timestamp
    for (int i = 0; i < total-1; i++) {
        for (int j = i+1; j < total; j++) {
            if (s_doses[i].timestamp > s_doses[j].timestamp) {
                dose_t tmp = s_doses[i];
                s_doses[i] = s_doses[j];
                s_doses[j] = tmp;
            }
        }
    }
    s_dose_count = total;
    ESP_LOGI(TAG, "Generated %d dose events", total);
    return total;
}

// -------------------------------------------------------------
// Dispensing routine
// -------------------------------------------------------------
static void dispense_medicine(const dose_t *dose)
{
    if (s_dispensing_in_progress) {
        ESP_LOGW(TAG, "Already dispensing");
        return;
    }
    s_dispensing_in_progress = true;

    // 1. Unlock relay (placeholder)
    ESP_LOGI(TAG, "Unlocking bin...");
    // TODO: set GPIO high to unlock

    // 2. Wait for hand detection
    ESP_LOGI(TAG, "Place hand to take %s", dose->medicine);
    int64_t start = esp_timer_get_time();
    bool hand_detected = false;
    while ((esp_timer_get_time() - start) < (s_hand_wait_sec * 1000000LL)) {
        float dist = ultrasonic_measure_blocking();
        if (dist > 0.1f && dist < 15.0f) {
            hand_detected = true;
            ESP_LOGI(TAG, "Hand detected at %.1f cm", dist);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (hand_detected) {
        stepper_motor_set_direction(true);
        stepper_motor_rotate_steps(s_steps_per_chamber);
        ESP_LOGI(TAG, "Relocking bin");
        // TODO: set GPIO low to lock
        char timebuf[32];
        time_t now;
        time_service_get_timestamp(&now);
        strftime(timebuf, sizeof(timebuf), "%Y-%m-%dT%H:%M:%SZ", gmtime(&now));
        mqtt_manager_publish_log("dispensed", dose->medicine, dose->remaining_pills-1, "auto");
        ESP_LOGI(TAG, "Dispensed %s", dose->medicine);
    } else {
        ESP_LOGW(TAG, "Hand not detected, dose missed");
        mqtt_manager_publish_log("missed", dose->medicine, dose->remaining_pills, "hand timeout");
    }
    s_dispensing_in_progress = false;
}

// -------------------------------------------------------------
// Timer callback – check for due doses
// -------------------------------------------------------------
static void check_timer_cb(void *arg)
{
    if (s_dose_count == 0) return;
    if (s_dispensing_in_progress) return;

    time_t now;
    time_service_get_timestamp(&now);
    if (now >= s_doses[0].timestamp) {
        dose_t dose = s_doses[0];
        for (int i = 0; i < s_dose_count-1; i++) {
            s_doses[i] = s_doses[i+1];
        }
        s_dose_count--;
        dispense_medicine(&dose);
    }
}

// -------------------------------------------------------------
// Public API
// -------------------------------------------------------------
esp_err_t scheduler_init(uint32_t steps_per_chamber, uint32_t hand_wait_timeout_sec)
{
    s_steps_per_chamber = steps_per_chamber;
    s_hand_wait_sec = hand_wait_timeout_sec;

    char *saved_json = load_schedule_from_nvs();
    if (saved_json) {
        ESP_LOGI(TAG, "Loading saved schedule");
        generate_doses_from_json(saved_json);
        free(saved_json);
    }

    const esp_timer_create_args_t timer_args = {
        .callback = check_timer_cb,
        .name = "scheduler_check"
    };
    esp_err_t ret = esp_timer_create(&timer_args, &s_check_timer);
    if (ret != ESP_OK) return ret;
    esp_timer_start_periodic(s_check_timer, 60 * 1000000LL);

    s_initialized = true;
    ESP_LOGI(TAG, "Scheduler initialized, %d pending doses", s_dose_count);
    return ESP_OK;
}

esp_err_t scheduler_set_schedule(const char *schedule_json)
{
    if (!schedule_json) return ESP_ERR_INVALID_ARG;
    s_dose_count = 0;
    int count = generate_doses_from_json(schedule_json);
    if (count == 0) {
        ESP_LOGW(TAG, "No valid doses in schedule");
        return ESP_FAIL;
    }
    save_schedule_to_nvs(schedule_json);
    ESP_LOGI(TAG, "Schedule updated, %d doses pending", count);
    return ESP_OK;
}

void scheduler_clear(void)
{
    s_dose_count = 0;
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
    if (s_dose_count == 0) {
        ESP_LOGW(TAG, "No pending doses");
        return;
    }
    if (s_dispensing_in_progress) {
        ESP_LOGW(TAG, "Already dispensing");
        return;
    }
    dose_t dose = s_doses[0];
    for (int i = 0; i < s_dose_count-1; i++) {
        s_doses[i] = s_doses[i+1];
    }
    s_dose_count--;
    dispense_medicine(&dose);
}

int scheduler_get_pending_count(void)
{
    return s_dose_count;
}