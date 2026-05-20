/**
 * @file ds1302.h
 * @brief DS1302 Real Time Clock Driver - 3-wire interface
 * 
 * This module provides functions to read and set the time on a DS1302 RTC.
 * It uses bit-banged GPIO for CLK, DATA, and RST pins.
 * 
 * The RTC can store seconds, minutes, hours, day of month, month, day of week,
 * and year (two-digit). The time is read in BCD format and converted to
 * binary (struct tm).
 * 
 * @author System Architect
 * @date 2026-05-20
 * @version 1.0
 */

#ifndef DS1302_H
#define DS1302_H

#include "esp_err.h"
#include <time.h>          // for struct tm
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DS1302 configuration structure
 */
typedef struct {
    gpio_num_t clk_pin;    /**< Clock pin (output) */
    gpio_num_t data_pin;   /**< Data pin (bidirectional, input/output) */
    gpio_num_t rst_pin;    /**< Reset pin (output) */
} ds1302_config_t;

/**
 * @brief Initialize the DS1302 driver
 * 
 * Configures GPIO pins and enables write protection.
 * 
 * @param cfg Pointer to configuration structure
 * @return ESP_OK on success, error otherwise
 */
esp_err_t ds1302_init(const ds1302_config_t *cfg);

/**
 * @brief Set the RTC time from a struct tm
 * 
 * Converts binary fields to BCD and writes to the RTC.
 * 
 * @param tm Pointer to struct tm with valid fields (tm_sec, tm_min, tm_hour,
 *           tm_mday, tm_mon, tm_year, tm_wday). Note: tm_mon is 0-11, tm_year is years since 1900.
 * @return ESP_OK on success, error otherwise
 */
esp_err_t ds1302_set_time(const struct tm *tm);

/**
 * @brief Get the current time from RTC into struct tm
 * 
 * Reads RTC registers, converts BCD to binary, populates struct tm.
 * 
 * @param tm Pointer to struct tm to fill
 * @return ESP_OK on success, error otherwise
 */
esp_err_t ds1302_get_time(struct tm *tm);

/**
 * @brief Get current time as Unix timestamp (seconds since 1970-01-01 UTC)
 * 
 * Uses ds1302_get_time() and mktime() to compute timestamp.
 * Note: RTC year range is 2000-2099, mktime uses years since 1900.
 * 
 * @param timestamp Pointer to store the timestamp (seconds)
 * @return ESP_OK on success, error otherwise
 */
esp_err_t ds1302_get_timestamp(time_t *timestamp);

/**
 * @brief Enable or disable write protection
 * 
 * @param enable true = write protect, false = write enable
 * @return ESP_OK
 */
esp_err_t ds1302_write_protect(bool enable);

/**
 * @brief Deinitialize the RTC driver (reset pins)
 */
void ds1302_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* DS1302_H */