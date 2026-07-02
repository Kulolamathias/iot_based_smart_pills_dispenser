#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "esp_err.h"
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the scheduler (load schedule from NVS, start checking timer)
 * 
 * @param steps_per_chamber Number of stepper motor steps to advance one chamber
 * @param hand_wait_timeout_sec Maximum seconds to wait for hand after unlocking
 * @return ESP_OK on success
 */
esp_err_t scheduler_init(uint32_t steps_per_chamber, uint32_t hand_wait_timeout_sec);

/**
 * @brief Set a new schedule from JSON (clears previous schedule)
 * 
 * @param schedule_json JSON array as defined in MQTT contract
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

#ifdef __cplusplus
}
#endif

#endif /* SCHEDULER_H */
