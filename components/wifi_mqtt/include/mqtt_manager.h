#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback for incoming messages on subscribed topics
 * 
 * @param topic Topic string
 * @param payload Message payload (null-terminated)
 * @param user_data User pointer
 */
typedef void (*mqtt_message_cb_t)(const char *topic, const char *payload, void *user_data);

/**
 * @brief Callback when MQTT connected
 */
typedef void (*mqtt_connected_cb_t)(void);

/**
 * @brief Initialize MQTT client with broker URI and credentials
 * 
 * @param broker_uri MQTT broker URI (e.g., "mqtt://broker.emqx.io:1883")
 * @param username Username (can be NULL)
 * @param password Password (can be NULL)
 * @return ESP_OK on success
 */
esp_err_t mqtt_manager_init(const char *broker_uri, const char *username, const char *password);

/**
 * @brief Start MQTT client (connects asynchronously)
 * @return ESP_OK
 */
esp_err_t mqtt_manager_start(void);

/**
 * @brief Subscribe to a topic (QoS 1)
 * @param topic Topic string
 * @return ESP_OK on success
 */
esp_err_t mqtt_manager_subscribe(const char *topic);

/**
 * @brief Publish a message (QoS 1, retain optional)
 * @param topic Topic string
 * @param payload Message payload
 * @param retain Retain flag
 * @return ESP_OK on success
 */
esp_err_t mqtt_manager_publish(const char *topic, const char *payload, bool retain);

/**
 * @brief Register a callback for all incoming messages (topic filter can be added later)
 * @param cb Callback function
 * @param user_data User data
 */
void mqtt_manager_register_message_cb(mqtt_message_cb_t cb, void *user_data);

/**
 * @brief Register callback for MQTT connected event
 */
void mqtt_manager_register_connected_cb(mqtt_connected_cb_t cb);

/**
 * @brief Stop and deinit MQTT
 */
void mqtt_manager_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* MQTT_MANAGER_H */