/**
 * @file stepper_motor.h
 * @brief A4988 Stepper Motor Driver - Hardware Abstraction
 * 
 * This module controls a bipolar stepper motor via an A4988 driver.
 * It uses full-step mode only. Leave the A4988 MS1/MS2/MS3 pins disconnected
 * or tied to GND in hardware.
 * 
 * All parameters are configurable via #defines or runtime functions.
 * No dynamic memory allocation; uses static GPIO configuration.
 * 
 * @author System Architect
 * @date 2026-05-20
 * @version 1.0
 */

#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Stepper motor configuration structure
 * 
 * All values must be provided before calling stepper_motor_init().
 */
typedef struct {
    gpio_num_t step_pin;        /**< STEP pin (output) */
    gpio_num_t dir_pin;         /**< DIR pin (output) */
    gpio_num_t enable_pin;      /**< ENABLE pin (output, active low) */
    uint32_t step_delay_us;     /**< Delay between full steps in microseconds */
    bool enable_on_init;        /**< If true, driver is enabled after init (ENABLE pin low) */
} stepper_config_t;

/**
 * @brief Initialize the stepper motor driver
 * 
 * Configures STEP/DIR/ENABLE GPIO pins and optionally enables the driver.
 * Must be called before any other stepper functions.
 * 
 * @param cfg Pointer to configuration structure (must remain valid until deinit)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t stepper_motor_init(const stepper_config_t *cfg);

/**
 * @brief Deinitialize the stepper motor driver
 * 
 * Disables the driver, resets GPIO pins to default state.
 */
void stepper_motor_deinit(void);

/**
 * @brief Enable the motor driver (ENABLE pin low)
 * 
 * @return ESP_OK if enabled, error otherwise
 */
esp_err_t stepper_motor_enable(void);

/**
 * @brief Disable the motor driver (ENABLE pin high)
 * 
 * @return ESP_OK if disabled, error otherwise
 */
esp_err_t stepper_motor_disable(void);

/**
 * @brief Set rotation direction
 * 
 * @param clockwise true = clockwise, false = counter-clockwise
 * @return ESP_OK always
 */
esp_err_t stepper_motor_set_direction(bool clockwise);

/**
 * @brief Rotate the motor by a given number of steps (blocking)
 * 
 * This function blocks for the duration of the rotation.
 * Steps are generated using simple delays. For non-blocking operation,
 * use a separate task or timer.
 * 
 * @param steps Number of steps to rotate (positive integer)
 * @return ESP_OK on success, error if not initialized or steps=0
 */
esp_err_t stepper_motor_rotate_steps(uint32_t steps);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_MOTOR_H */
