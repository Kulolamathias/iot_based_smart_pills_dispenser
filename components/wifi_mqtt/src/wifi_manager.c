#include "wifi_manager.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include <string.h>

// #define CONFIG_WIFI_SSID "Kylianmbappe"         // Default SSID (can be overridden by Kconfig)
// #define CONFIG_WIFI_PASSWORD "kylianmbappe" // Default password (can be overridden by Kconfig)
#define CONFIG_WIFI_SSID "Mathias' Sxx U..."         // Default SSID (can be overridden by Kconfig)
#define CONFIG_WIFI_PASSWORD "1234567890223" // Default password (can be overridden by Kconfig)

static const char *TAG = "WIFI_MGR";

static bool s_initialized = false;
static wifi_connected_cb_t s_connected_cb = NULL;
static wifi_disconnected_cb_t s_disconnected_cb = NULL;
static char s_ip_str[16] = {0};
static char s_mac_str[18] = {0};

/* Event handler for WiFi and IP events */
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WiFi station started, connecting...");
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "WiFi connected to AP");
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
                if (s_disconnected_cb) s_disconnected_cb();
                esp_wifi_connect();
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            esp_ip4addr_ntoa(&event->ip_info.ip, s_ip_str, sizeof(s_ip_str));
            ESP_LOGI(TAG, "Got IP: %s", s_ip_str);
            if (s_connected_cb) s_connected_cb();
        }
    }
}

/* Initialise NVS (if not already done) */
static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

esp_err_t wifi_manager_init(const char *ssid, const char *password)
{
    if (s_initialized) return ESP_OK;

    /* Use Kconfig defaults if parameters are NULL */
    const char *wifi_ssid = ssid ? ssid : CONFIG_WIFI_SSID;
    const char *wifi_pass = password ? password : CONFIG_WIFI_PASSWORD;

    if (!wifi_ssid || strlen(wifi_ssid) == 0) {
        ESP_LOGE(TAG, "WiFi SSID not provided and not set in Kconfig");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initialising WiFi with SSID: %s", wifi_ssid);

    /* Initialise NVS (required for WiFi) */
    esp_err_t ret = init_nvs();
    if (ret != ESP_OK) return ret;

    /* Create default event loop and netif if not already created */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    /* Get MAC address for device discovery */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_mac_str, sizeof(s_mac_str), "%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "Device MAC: %s", s_mac_str);

    /* Initialise WiFi */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strlcpy((char*)wifi_config.sta.ssid, wifi_ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char*)wifi_config.sta.password, wifi_pass, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_initialized = true;
    return ESP_OK;
}

esp_err_t wifi_manager_start(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    /* Already started in init, but ensure connection */
    esp_wifi_connect();
    return ESP_OK;
}

void wifi_manager_register_connected_cb(wifi_connected_cb_t cb)
{
    s_connected_cb = cb;
}

void wifi_manager_register_disconnected_cb(wifi_disconnected_cb_t cb)
{
    s_disconnected_cb = cb;
}

esp_err_t wifi_manager_get_mac(char *mac_buf, size_t len)
{
    if (!mac_buf || len < 13) return ESP_ERR_INVALID_ARG;
    if (s_mac_str[0] == 0) return ESP_FAIL;
    strlcpy(mac_buf, s_mac_str, len);
    return ESP_OK;
}

esp_err_t wifi_manager_get_ip(char *ip_buf, size_t len)
{
    if (!ip_buf || len < 16) return ESP_ERR_INVALID_ARG;
    if (s_ip_str[0] == 0) return ESP_FAIL;
    strlcpy(ip_buf, s_ip_str, len);
    return ESP_OK;
}

void wifi_manager_deinit(void)
{
    if (!s_initialized) return;
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_event_loop_delete_default();
    s_initialized = false;
    ESP_LOGI(TAG, "WiFi deinitialized");
}