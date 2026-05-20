#ifndef KEYPAD_H
#define KEYPAD_H

#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise keypad with row and column pins.
 * 
 * @param row_pins Array of 4 GPIO numbers for rows (outputs)
 * @param col_pins Array of 4 GPIO numbers for columns (inputs with pull‑ups)
 * @return ESP_OK on success
 */
esp_err_t keypad_init(const gpio_num_t row_pins[4], const gpio_num_t col_pins[4]);

/**
 * @brief Start keypad scanning task (creates a background task).
 * @return ESP_OK
 */
esp_err_t keypad_start(void);

/**
 * @brief Get the last key pressed (non‑blocking).
 * @param key Pointer to store the key character (0-9, A, B, C, D, *, #).
 * @return true if a key was available, false otherwise.
 */
bool keypad_get_key(char *key);

/**
 * @brief Stop scanning and deinit.
 */
void keypad_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* KEYPAD_H */