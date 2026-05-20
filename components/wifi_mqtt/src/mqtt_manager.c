#include "mqtt_manager.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_mac.h"
#include <string.h>
#include <time.h>

static const char *TAG = "MQTT_MGR";

static esp_mqtt_client_handle_t s_client = NULL;
static bool s_connected = false;
static char s_device_id[18] = {0};        // MAC without colons
static char s_base_topic[64] = {0};       // smartpill/dispenser/{mac}

static mqtt_connected_cb_t s_connected_cb = NULL;
static mqtt_command_cb_t s_command_cb = NULL;
static void *s_command_user = NULL;
static mqtt_schedule_cb_t s_schedule_cb = NULL;
static void *s_schedule_user = NULL;

/* Helper: get MAC address as device ID */
static void get_device_id(void)
{
    if (s_device_id[0] != 0) return;
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_device_id, sizeof(s_device_id), "%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    snprintf(s_base_topic, sizeof(s_base_topic), "smartpill/dispenser/%s", s_device_id);
    ESP_LOGI(TAG, "Device ID: %s, base topic: %s", s_device_id, s_base_topic);
}

/* Helper: publish to a sub‑topic */
static esp_err_t publish(const char *subtopic, const char *payload, bool retain)
{
    if (!s_client || !s_connected) {
        ESP_LOGW(TAG, "MQTT not connected, cannot publish to %s", subtopic);
        return ESP_FAIL;
    }
    char topic[128];
    snprintf(topic, sizeof(topic), "%s/%s", s_base_topic, subtopic);
    int msg_id = esp_mqtt_client_publish(s_client, topic, payload, 0, 1, retain ? 1 : 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish to %s", topic);
        return ESP_FAIL;
    }
    ESP_LOGD(TAG, "Published to %s: %s", topic, payload);
    return ESP_OK;
}

/* MQTT event handler */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            s_connected = true;
            /* Publish discovery announcement immediately */
            mqtt_manager_publish_discovery(NULL);
            /* Subscribe to command and schedule topics */
            char cmd_topic[128], sched_topic[128];
            snprintf(cmd_topic, sizeof(cmd_topic), "%s/command", s_base_topic);
            snprintf(sched_topic, sizeof(sched_topic), "%s/schedule", s_base_topic);
            esp_mqtt_client_subscribe(s_client, cmd_topic, 1);
            esp_mqtt_client_subscribe(s_client, sched_topic, 1);
            ESP_LOGI(TAG, "Subscribed to %s and %s", cmd_topic, sched_topic);
            if (s_connected_cb) s_connected_cb();
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            s_connected = false;
            break;
        case MQTT_EVENT_DATA: {
            char topic[128];
            char data[512];
            int topic_len = event->topic_len;
            if (topic_len > sizeof(topic)-1) topic_len = sizeof(topic)-1;
            memcpy(topic, event->topic, topic_len);
            topic[topic_len] = '\0';
            int data_len = event->data_len;
            if (data_len > sizeof(data)-1) data_len = sizeof(data)-1;
            memcpy(data, event->data, data_len);
            data[data_len] = '\0';
            ESP_LOGD(TAG, "Received: topic=%s, data=%s", topic, data);
            /* Determine which topic */
            char expected_cmd[128], expected_sched[128];
            snprintf(expected_cmd, sizeof(expected_cmd), "%s/command", s_base_topic);
            snprintf(expected_sched, sizeof(expected_sched), "%s/schedule", s_base_topic);
            if (strcmp(topic, expected_cmd) == 0) {
                /* Parse action field from JSON */
                const char *action_key = "\"action\"";
                char *action_start = strstr(data, action_key);
                if (action_start) {
                    action_start = strchr(action_start, ':');
                    if (action_start) {
                        action_start++; // skip colon
                        while (*action_start == ' ' || *action_start == '\"') action_start++;
                        char *action_end = strchr(action_start, '\"');
                        if (action_end) {
                            int len = action_end - action_start;
                            char action[32];
                            if (len < (int)sizeof(action)-1) {
                                memcpy(action, action_start, len);
                                action[len] = '\0';
                                if (s_command_cb) s_command_cb(action, data, s_command_user);
                            }
                        }
                    }
                } else {
                    // If no "action" field, treat the whole payload as a simple command string
                    if (s_command_cb) s_command_cb(data, data, s_command_user);
                }
            } else if (strcmp(topic, expected_sched) == 0) {
                if (s_schedule_cb) s_schedule_cb(data, s_schedule_user);
            }
            break;
        }
        default:
            break;
    }
}

/* Public API */

esp_err_t mqtt_manager_init(const char *broker_uri, const char *username, const char *password)
{
    if (!broker_uri) {
        ESP_LOGE(TAG, "Broker URI not provided");
        return ESP_ERR_INVALID_ARG;
    }
    get_device_id();   // ensure s_device_id and s_base_topic are set

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

void mqtt_manager_register_connected_cb(mqtt_connected_cb_t cb)
{
    s_connected_cb = cb;
}

void mqtt_manager_register_command_cb(mqtt_command_cb_t cb, void *user_data)
{
    s_command_cb = cb;
    s_command_user = user_data;
}

void mqtt_manager_register_schedule_cb(mqtt_schedule_cb_t cb, void *user_data)
{
    s_schedule_cb = cb;
    s_schedule_user = user_data;
}

esp_err_t mqtt_manager_publish_discovery(const char *version)
{
    if (!s_connected) return ESP_FAIL;
    char payload[128];
    snprintf(payload, sizeof(payload), "{\"id\":\"%s\",\"type\":\"pill_dispenser\"", s_device_id);
    if (version && version[0]) {
        char ver[32];
        snprintf(ver, sizeof(ver), ",\"version\":\"%s\"", version);
        strlcat(payload, ver, sizeof(payload));
    }
    strlcat(payload, "}", sizeof(payload));
    /* Discovery topic is fixed, no device ID */
    int msg_id = esp_mqtt_client_publish(s_client, "smartpill/discovery/announce", payload, 0, 1, 0);
    if (msg_id < 0) return ESP_FAIL;
    ESP_LOGI(TAG, "Published discovery: %s", payload);
    return ESP_OK;
}

esp_err_t mqtt_manager_publish_status(const char *state)
{
    if (!state) return ESP_ERR_INVALID_ARG;
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"state\":\"%s\"}", state);
    return publish("status", payload, true);
}

esp_err_t mqtt_manager_publish_log(const char *event, const char *medicine, int remaining_pills, const char *info)
{
    if (!event) return ESP_ERR_INVALID_ARG;
    char time_buf[32];
    time_t now = time(NULL);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%dT%H:%M:%SZ", &tm_now); // Simple UTC, adjust if needed

    char payload[256];
    int len = snprintf(payload, sizeof(payload), "{\"event\":\"%s\",\"time\":\"%s\"", event, time_buf);
    if (medicine && medicine[0]) {
        len += snprintf(payload + len, sizeof(payload) - len, ",\"medicine\":\"%s\"", medicine);
    }
    if (remaining_pills >= 0) {
        len += snprintf(payload + len, sizeof(payload) - len, ",\"remaining_pills\":%d", remaining_pills);
    }
    if (info && info[0]) {
        len += snprintf(payload + len, sizeof(payload) - len, ",\"info\":\"%s\"", info);
    }
    snprintf(payload + len, sizeof(payload) - len, "}");
    return publish("log", payload, false);
}

const char* mqtt_manager_get_device_id(void)
{
    return s_device_id;
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