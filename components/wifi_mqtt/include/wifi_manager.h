#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback when WiFi is connected and IP is obtained
 */
typedef void (*wifi_connected_cb_t)(void);

/**
 * @brief Callback when WiFi is disconnected (will auto‑reconnect)
 */
typedef void (*wifi_disconnected_cb_t)(void);

/**
 * @brief Initialise WiFi in station mode
 * 
 * @param ssid      WiFi SSID (if NULL, uses CONFIG_WIFI_SSID from Kconfig)
 * @param password  WiFi password (if NULL, uses CONFIG_WIFI_PASSWORD)
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_init(const char *ssid, const char *password);

/**
 * @brief Start WiFi connection (non‑blocking)
 * @return ESP_OK
 */
esp_err_t wifi_manager_start(void);

/**
 * @brief Register callback for connected event
 * @param cb Callback function
 */
void wifi_manager_register_connected_cb(wifi_connected_cb_t cb);

/**
 * @brief Register callback for disconnected event
 * @param cb Callback function
 */
void wifi_manager_register_disconnected_cb(wifi_disconnected_cb_t cb);

/**
 * @brief Get MAC address of the station interface
 * @param mac_buf Buffer to store MAC string (e.g., "a1b2c3d4e5f6")
 * @param len    Size of buffer (must be at least 13)
 * @return ESP_OK if MAC obtained
 */
esp_err_t wifi_manager_get_mac(char *mac_buf, size_t len);

/**
 * @brief Get current IP address as string
 * @param ip_buf Buffer to store IP (e.g., "192.168.1.100")
 * @param len    Size of buffer (must be at least 16)
 * @return ESP_OK if IP assigned, else ESP_FAIL
 */
esp_err_t wifi_manager_get_ip(char *ip_buf, size_t len);

/**
 * @brief Stop WiFi and deinit
 */
void wifi_manager_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_MANAGER_H */