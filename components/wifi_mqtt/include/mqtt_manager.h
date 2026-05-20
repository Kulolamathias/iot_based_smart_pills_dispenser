#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback for incoming command (from smartpill/dispenser/{mac}/command)
 * 
 * @param action Action string (e.g., "dispense_now", "reboot", "clear_schedule")
 * @param payload Full JSON payload (for extensibility)
 * @param user_data User pointer
 */
typedef void (*mqtt_command_cb_t)(const char *action, const char *payload, void *user_data);

/**
 * @brief Callback for incoming schedule update (from smartpill/dispenser/{mac}/schedule)
 * 
 * @param schedule_json Full JSON array of schedule entries
 * @param user_data User pointer
 */
typedef void (*mqtt_schedule_cb_t)(const char *schedule_json, void *user_data);

/**
 * @brief Callback when MQTT connection is established
 */
typedef void (*mqtt_connected_cb_t)(void);

/**
 * @brief Initialise MQTT manager
 * 
 * @param broker_uri   MQTT broker URI (e.g., "mqtt://102.223.8.140:1883")
 * @param username     Username (can be NULL)
 * @param password     Password (can be NULL)
 * @return ESP_OK on success
 */
esp_err_t mqtt_manager_init(const char *broker_uri, const char *username, const char *password);

/**
 * @brief Start MQTT client (connects asynchronously)
 * @return ESP_OK
 */
esp_err_t mqtt_manager_start(void);

/**
 * @brief Register callback for MQTT connected event
 */
void mqtt_manager_register_connected_cb(mqtt_connected_cb_t cb);

/**
 * @brief Register callback for incoming commands
 */
void mqtt_manager_register_command_cb(mqtt_command_cb_t cb, void *user_data);

/**
 * @brief Register callback for incoming schedule updates
 */
void mqtt_manager_register_schedule_cb(mqtt_schedule_cb_t cb, void *user_data);

/**
 * @brief Publish device discovery announcement (usually called once on connect)
 * 
 * @param version Firmware version string (optional, can be NULL)
 * @return ESP_OK on success
 */
esp_err_t mqtt_manager_publish_discovery(const char *version);

/**
 * @brief Publish device status (retained)
 * 
 * @param state State string: "online", "idle", "dispensing", "error"
 * @return ESP_OK on success
 */
esp_err_t mqtt_manager_publish_status(const char *state);

/**
 * @brief Publish a log event
 * 
 * @param event     Event type (e.g., "dispensed", "missed", "schedule_updated")
 * @param medicine  Medicine name (optional, can be NULL)
 * @param remaining_pills Remaining pills (optional, use -1 if not applicable)
 * @param info      Extra info string (optional, can be NULL)
 * @return ESP_OK on success
 */
esp_err_t mqtt_manager_publish_log(const char *event, const char *medicine, int remaining_pills, const char *info);

/**
 * @brief Get the device MAC (used as ID)
 * @return Pointer to static string (e.g., "a1b2c3d4e5f6")
 */
const char* mqtt_manager_get_device_id(void);

/**
 * @brief Stop and deinit MQTT
 */
void mqtt_manager_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* MQTT_MANAGER_H */