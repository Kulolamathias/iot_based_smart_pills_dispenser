#include "mqtt_manager.h"
#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG = "MQTT_MGR";

static esp_mqtt_client_handle_t s_client = NULL;
static bool s_connected = false;
static mqtt_message_cb_t s_message_cb = NULL;
static void *s_message_user = NULL;
static mqtt_connected_cb_t s_connected_cb = NULL;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            s_connected = true;
            if (s_connected_cb) s_connected_cb();
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            s_connected = false;
            break;
        case MQTT_EVENT_DATA: {
            char topic[128];
            char payload[512];
            int topic_len = event->topic_len;
            if (topic_len > sizeof(topic)-1) topic_len = sizeof(topic)-1;
            memcpy(topic, event->topic, topic_len);
            topic[topic_len] = '\0';
            int data_len = event->data_len;
            if (data_len > sizeof(payload)-1) data_len = sizeof(payload)-1;
            memcpy(payload, event->data, data_len);
            payload[data_len] = '\0';
            ESP_LOGD(TAG, "Received: topic=%s, payload=%s", topic, payload);
            if (s_message_cb) {
                s_message_cb(topic, payload, s_message_user);
            }
            break;
        }
        default:
            break;
    }
}

esp_err_t mqtt_manager_init(const char *broker_uri, const char *username, const char *password)
{
    if (!broker_uri) {
        ESP_LOGE(TAG, "Broker URI not provided");
        return ESP_ERR_INVALID_ARG;
    }
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_uri,
        .credentials = {
            .username = username,
            .authentication.password = password,
        },
        .session.keepalive = 120,
    };
    s_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_client) return ESP_FAIL;
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    return ESP_OK;
}

esp_err_t mqtt_manager_start(void)
{
    if (!s_client) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = esp_mqtt_client_start(s_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client");
        return ret;
    }
    return ESP_OK;
}

esp_err_t mqtt_manager_subscribe(const char *topic)
{
    if (!s_client || !s_connected) {
        ESP_LOGW(TAG, "MQTT not connected, cannot subscribe");
        return ESP_FAIL;
    }
    int msg_id = esp_mqtt_client_subscribe(s_client, topic, 1);
    if (msg_id < 0) return ESP_FAIL;
    ESP_LOGI(TAG, "Subscribed to %s", topic);
    return ESP_OK;
}

esp_err_t mqtt_manager_publish(const char *topic, const char *payload, bool retain)
{
    if (!s_client || !s_connected) {
        ESP_LOGW(TAG, "MQTT not connected, cannot publish");
        return ESP_FAIL;
    }
    int msg_id = esp_mqtt_client_publish(s_client, topic, payload, 0, 1, retain ? 1 : 0);
    if (msg_id < 0) return ESP_FAIL;
    return ESP_OK;
}

void mqtt_manager_register_message_cb(mqtt_message_cb_t cb, void *user_data)
{
    s_message_cb = cb;
    s_message_user = user_data;
}

void mqtt_manager_register_connected_cb(mqtt_connected_cb_t cb)
{
    s_connected_cb = cb;
}

void mqtt_manager_deinit(void)
{
    if (s_client) {
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
    }
    s_connected = false;
    ESP_LOGI(TAG, "MQTT deinitialized");
}