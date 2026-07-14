#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCHEDULER_MEDICINE_NAME_LEN 32

typedef enum {
    SCHEDULER_ACTIVITY_IDLE = 0,
    SCHEDULER_ACTIVITY_WAITING_FOR_HAND,
    SCHEDULER_ACTIVITY_DISPENSING,
    SCHEDULER_ACTIVITY_SUCCESS,
    SCHEDULER_ACTIVITY_ERROR
} scheduler_activity_t;

typedef struct {
    scheduler_activity_t activity;
    int pending_count;
    bool has_next_dose;
    time_t next_due_time;
    char next_medicine[SCHEDULER_MEDICINE_NAME_LEN];
    char active_medicine[SCHEDULER_MEDICINE_NAME_LEN];
    float hand_distance_cm;
    uint8_t current_chamber;
} scheduler_status_t;

/**
 * @brief Initialize the scheduler (load schedule from NVS, start checking timer)
 * 
 * @param steps_per_chamber Number of stepper motor steps to advance one chamber
 * @param hand_wait_timeout_sec Seconds between reminder logs while waiting for hand
 * @return ESP_OK on success
 */
esp_err_t scheduler_init(uint32_t steps_per_chamber, uint32_t hand_wait_timeout_sec);

/**
 * @brief Set a new schedule from JSON (clears previous schedule)
 * 
 * @param schedule_json JSON array as defined in the MQTT contract. Optional
 *        total_doses and pills_per_dose fields limit generated dose events;
 *        existing payloads default to one pill per dose.
 * @return ESP_OK if schedule accepted, else error
 */
esp_err_t scheduler_set_schedule(const char *schedule_json);

/**
 * @brief Clear all scheduled doses (stops all future dispensing)
 */
void scheduler_clear(void);

/**
 * @brief Manually trigger dispensing (emergency or test)
 * 
 * Displays the next pending dose (or all doses?) For simplicity, it will
 * dispense the next due dose immediately.
 */
void scheduler_dispense_now(void);

/**
 * @brief Move the carousel by raw full steps for calibration/testing.
 *
 * This bypasses hand detection and schedules. It is intended for mechanical
 * calibration only.
 *
 * @param steps Number of full steps to move clockwise.
 */
void scheduler_calibration_move_steps(uint32_t steps);

/**
 * @brief Get number of pending doses (not yet taken)
 */
int scheduler_get_pending_count(void);

/**
 * @brief Get the timestamp of the next pending dose.
 *
 * @param timestamp Output UTC timestamp.
 * @return true if a next dose exists.
 */
bool scheduler_get_next_due_time(time_t *timestamp);

/**
 * @brief Return true while a due dose is waiting for hand detection.
 */
bool scheduler_is_waiting_for_hand(void);

/**
 * @brief Return true briefly after a successful dispense for success feedback.
 */
bool scheduler_success_flash_active(void);

/**
 * @brief Get one coherent snapshot for alerts and the user interface.
 */
esp_err_t scheduler_get_status(scheduler_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* SCHEDULER_H */
