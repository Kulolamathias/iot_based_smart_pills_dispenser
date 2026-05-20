#include "keypad.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "KEYPAD";

static gpio_num_t s_row_pins[4];
static gpio_num_t s_col_pins[4];
static QueueHandle_t s_key_queue = NULL;
static TaskHandle_t s_scan_task = NULL;
static bool s_running = false;

// Key mapping: rows 0-3, cols 0-3
static const char s_keymap[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

static void keypad_scan_task(void *pvParameters)
{
    uint8_t row_state[4] = {0};
    TickType_t last_scan = xTaskGetTickCount();
    const TickType_t scan_interval = pdMS_TO_TICKS(50); // 50 ms debounce

    while (s_running) {
        // For each row, drive low, read columns
        for (int r = 0; r < 4; r++) {
            // Set all rows high first (disable)
            for (int i = 0; i < 4; i++) {
                gpio_set_level(s_row_pins[i], 1);
            }
            // Drive current row low
            gpio_set_level(s_row_pins[r], 0);
            vTaskDelay(pdMS_TO_TICKS(1)); // allow settling

            // Read columns
            for (int c = 0; c < 4; c++) {
                int level = gpio_get_level(s_col_pins[c]);
                if (level == 0) { // key pressed (col pulled low)
                    row_state[r] |= (1 << c);
                } else {
                    row_state[r] &= ~(1 << c);
                }
            }
        }

        // Detect changes and send to queue
        static uint8_t last_state[4] = {0};
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                bool pressed = (row_state[r] >> c) & 1;
                bool was_pressed = (last_state[r] >> c) & 1;
                if (pressed && !was_pressed) {
                    char key = s_keymap[r][c];
                    xQueueSend(s_key_queue, &key, 0);
                    ESP_LOGD(TAG, "Key pressed: %c", key);
                }
            }
            last_state[r] = row_state[r];
        }

        vTaskDelayUntil(&last_scan, scan_interval);
    }
    vTaskDelete(NULL);
}

esp_err_t keypad_init(const gpio_num_t row_pins[4], const gpio_num_t col_pins[4])
{
    if (!row_pins || !col_pins) return ESP_ERR_INVALID_ARG;
    memcpy(s_row_pins, row_pins, sizeof(s_row_pins));
    memcpy(s_col_pins, col_pins, sizeof(s_col_pins));

    // Configure row pins as outputs, default high
    gpio_config_t out_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    for (int i = 0; i < 4; i++) {
        out_cfg.pin_bit_mask = (1ULL << s_row_pins[i]);
        gpio_config(&out_cfg);
        gpio_set_level(s_row_pins[i], 1);
    }

    // Configure column pins as inputs with pull‑up
    gpio_config_t in_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    for (int i = 0; i < 4; i++) {
        in_cfg.pin_bit_mask = (1ULL << s_col_pins[i]);
        gpio_config(&in_cfg);
    }

    s_key_queue = xQueueCreate(10, sizeof(char));
    if (!s_key_queue) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "Keypad initialised");
    return ESP_OK;
}

esp_err_t keypad_start(void)
{
    if (s_running) return ESP_ERR_INVALID_STATE;
    s_running = true;
    BaseType_t ret = xTaskCreate(keypad_scan_task, "keypad_scan", 2048, NULL, 5, &s_scan_task);
    if (ret != pdPASS) {
        s_running = false;
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool keypad_get_key(char *key)
{
    if (!s_key_queue) return false;
    return xQueueReceive(s_key_queue, key, 0) == pdTRUE;
}

void keypad_deinit(void)
{
    s_running = false;
    if (s_scan_task) {
        vTaskDelete(s_scan_task);
        s_scan_task = NULL;
    }
    if (s_key_queue) {
        vQueueDelete(s_key_queue);
        s_key_queue = NULL;
    }
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(s_row_pins[i]);
        gpio_reset_pin(s_col_pins[i]);
    }
    ESP_LOGI(TAG, "Keypad deinit");
}