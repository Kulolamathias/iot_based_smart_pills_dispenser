/**
 * @file ds1302.c
 * @brief DS1302 Real Time Clock Driver - Implementation
 * 
 * The DS1302 uses a 3-wire interface with the following protocol:
 * - RST high enables communication.
 * - Command byte is sent first (LSB first) indicating read/write and register address.
 * - Then data bytes are read/written (LSB first).
 * - RST low ends communication.
 * 
 * All timing delays are taken from the DS1302 datasheet.
 * 
 * @author System Architect
 * @date 2026-05-20
 */

#include "ds1302.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us
#include <string.h>

static const char *TAG = "DS1302";

/* Static context */
typedef struct {
    gpio_num_t clk_pin;
    gpio_num_t data_pin;
    gpio_num_t rst_pin;
    bool initialized;
} ds1302_ctx_t;

static ds1302_ctx_t s_ctx = { .initialized = false };

/* Timing constants (microseconds) */
#define DS1302_T_DELAY_US     4      /**< Data setup/hold time, pulse width */
#define DS1302_T_RST_HIGH_US   4      /**< RST high before communication */

/* Register addresses (with read/write bit) */
#define DS1302_CMD_SECOND      0x80   /**< Seconds register (write address) */
#define DS1302_CMD_MINUTE      0x82
#define DS1302_CMD_HOUR        0x84
#define DS1302_CMD_DATE        0x86
#define DS1302_CMD_MONTH       0x88
#define DS1302_CMD_DAY         0x8A
#define DS1302_CMD_YEAR        0x8C
#define DS1302_CMD_CONTROL     0x8E   /**< Write protect, etc. */

/* Burst mode registers */
#define DS1302_CMD_BURST_READ  0xBF
#define DS1302_CMD_BURST_WRITE 0xBE

/* Control register bits */
#define DS1302_BIT_WP          0x80   /**< Write protect bit (1 = protected) */
#define DS1302_BIT_CH          0x80   /**< Clock halt bit (1 = oscillator off) */

/* Forward declarations of internal functions */
static void set_clk_high(void);
static void set_clk_low(void);
static void set_data_high(void);
static void set_data_low(void);
static bool read_data(void);
static void set_data_dir_output(void);
static void set_data_dir_input(void);
static void send_byte(uint8_t byte);
static uint8_t recv_byte(void);
static void start_communication(void);
static void stop_communication(void);
static uint8_t bcd_to_bin(uint8_t bcd);
static uint8_t bin_to_bcd(uint8_t bin);

/* ------------------------------------------------------------- */
/* Low-level GPIO helpers                                       */
/* ------------------------------------------------------------- */
static void set_clk_high(void)
{
    gpio_set_level(s_ctx.clk_pin, 1);
}

static void set_clk_low(void)
{
    gpio_set_level(s_ctx.clk_pin, 0);
}

static void set_data_high(void)
{
    gpio_set_level(s_ctx.data_pin, 1);
}

static void set_data_low(void)
{
    gpio_set_level(s_ctx.data_pin, 0);
}

static bool read_data(void)
{
    return gpio_get_level(s_ctx.data_pin) != 0;
}

static void set_data_dir_output(void)
{
    gpio_set_direction(s_ctx.data_pin, GPIO_MODE_OUTPUT);
}

static void set_data_dir_input(void)
{
    gpio_set_direction(s_ctx.data_pin, GPIO_MODE_INPUT);
}

/* ------------------------------------------------------------- */
/* Bit-banging protocol helpers                                 */
/* ------------------------------------------------------------- */
static void send_byte(uint8_t byte)
{
    /* Send LSB first */
    for (int i = 0; i < 8; i++) {
        if (byte & (1 << i)) {
            set_data_high();
        } else {
            set_data_low();
        }
        esp_rom_delay_us(DS1302_T_DELAY_US);
        set_clk_high();
        esp_rom_delay_us(DS1302_T_DELAY_US);
        set_clk_low();
        esp_rom_delay_us(DS1302_T_DELAY_US);
    }
}

static uint8_t recv_byte(void)
{
    uint8_t byte = 0;
    set_data_dir_input();
    /* Read LSB first */
    for (int i = 0; i < 8; i++) {
        set_clk_high();
        esp_rom_delay_us(DS1302_T_DELAY_US);
        if (read_data()) {
            byte |= (1 << i);
        }
        set_clk_low();
        esp_rom_delay_us(DS1302_T_DELAY_US);
    }
    set_data_dir_output();   /* restore output mode */
    return byte;
}

static void start_communication(void)
{
    set_clk_low();
    esp_rom_delay_us(DS1302_T_DELAY_US);
    gpio_set_level(s_ctx.rst_pin, 1);
    esp_rom_delay_us(DS1302_T_RST_HIGH_US);
}

static void stop_communication(void)
{
    set_clk_low();
    gpio_set_level(s_ctx.rst_pin, 0);
    esp_rom_delay_us(DS1302_T_DELAY_US);
}

/* ------------------------------------------------------------- */
/* BCD helpers                                                  */
/* ------------------------------------------------------------- */
static uint8_t bcd_to_bin(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static uint8_t bin_to_bcd(uint8_t bin)
{
    return ((bin / 10) << 4) | (bin % 10);
}

/* ------------------------------------------------------------- */
/* Public API                                                   */
/* ------------------------------------------------------------- */
esp_err_t ds1302_init(const ds1302_config_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    s_ctx.clk_pin = cfg->clk_pin;
    s_ctx.data_pin = cfg->data_pin;
    s_ctx.rst_pin = cfg->rst_pin;
    s_ctx.initialized = false;

    /* Configure GPIOs */
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    uint64_t pin_mask = (1ULL << s_ctx.clk_pin) |
                        (1ULL << s_ctx.data_pin) |
                        (1ULL << s_ctx.rst_pin);
    io_conf.pin_bit_mask = pin_mask;
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set initial levels: clk low, rst low, data low (output) */
    set_clk_low();
    gpio_set_level(s_ctx.rst_pin, 0);
    set_data_low();
    set_data_dir_output();

    s_ctx.initialized = true;

    /* Disable write protection to allow writes */
    ds1302_write_protect(false);

    ESP_LOGI(TAG, "DS1302 initialized on pins: CLK=%d, DATA=%d, RST=%d",
             s_ctx.clk_pin, s_ctx.data_pin, s_ctx.rst_pin);
    return ESP_OK;
}

esp_err_t ds1302_write_protect(bool enable)
{
    if (!s_ctx.initialized) return ESP_ERR_INVALID_STATE;

    start_communication();
    send_byte(DS1302_CMD_CONTROL);
    send_byte(enable ? DS1302_BIT_WP : 0x00);
    stop_communication();

    ESP_LOGD(TAG, "Write protect %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t ds1302_set_time(const struct tm *tm)
{
    if (!s_ctx.initialized) return ESP_ERR_INVALID_STATE;
    if (!tm) return ESP_ERR_INVALID_ARG;

    /* Convert binary fields to BCD */
    uint8_t sec = bin_to_bcd(tm->tm_sec);
    uint8_t min = bin_to_bcd(tm->tm_min);
    uint8_t hour = bin_to_bcd(tm->tm_hour);
    uint8_t date = bin_to_bcd(tm->tm_mday);
    uint8_t month = bin_to_bcd(tm->tm_mon + 1);    /* tm_mon is 0-11 */
    uint8_t day = bin_to_bcd(tm->tm_wday);         /* tm_wday: 0=Sunday..6=Saturday */
    uint8_t year = bin_to_bcd(tm->tm_year % 100);  /* years since 1900, take last two digits */

    /* Ensure CH bit is cleared in seconds register (start oscillator) */
    sec &= ~DS1302_BIT_CH;

    /* Write burst mode (all 8 registers at once) */
    start_communication();
    send_byte(DS1302_CMD_BURST_WRITE);
    send_byte(sec);
    send_byte(min);
    send_byte(hour);
    send_byte(date);
    send_byte(month);
    send_byte(day);
    send_byte(year);
    /* Control register (optional) – we already set write protect */
    send_byte(0x00);   /* control register value (0 = no write protect, etc.) */
    stop_communication();

    ESP_LOGI(TAG, "Time set to %04d-%02d-%02d %02d:%02d:%02d (wday=%d)",
             tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
             tm->tm_hour, tm->tm_min, tm->tm_sec, tm->tm_wday);
    return ESP_OK;
}

esp_err_t ds1302_get_time(struct tm *tm)
{
    if (!s_ctx.initialized) return ESP_ERR_INVALID_STATE;
    if (!tm) return ESP_ERR_INVALID_ARG;

    uint8_t sec, min, hour, date, month, day, year;

    /* Read all time registers using burst mode */
    start_communication();
    send_byte(DS1302_CMD_BURST_READ);
    sec = recv_byte();
    min = recv_byte();
    hour = recv_byte();
    date = recv_byte();
    month = recv_byte();
    day = recv_byte();
    year = recv_byte();
    /* Read control register but ignore */
    recv_byte();
    stop_communication();

    /* Check clock halt bit */
    if (sec & DS1302_BIT_CH) {
        ESP_LOGW(TAG, "RTC oscillator halted; time may be invalid");
        return ESP_FAIL;
    }

    /* Convert BCD to binary */
    tm->tm_sec = bcd_to_bin(sec & 0x7F);
    tm->tm_min = bcd_to_bin(min);
    tm->tm_hour = bcd_to_bin(hour);
    tm->tm_mday = bcd_to_bin(date);
    tm->tm_mon = bcd_to_bin(month) - 1;   /* month in tm is 0-11 */
    tm->tm_wday = bcd_to_bin(day);
    tm->tm_year = bcd_to_bin(year) + 2000 - 1900;  /* convert to years since 1900 */
    tm->tm_isdst = -1;   /* unknown */

    return ESP_OK;
}

esp_err_t ds1302_get_timestamp(time_t *timestamp)
{
    if (!timestamp) return ESP_ERR_INVALID_ARG;
    struct tm tm;
    esp_err_t ret = ds1302_get_time(&tm);
    if (ret != ESP_OK) return ret;

    /* mktime expects years since 1900, months 0-11, etc. */
    *timestamp = mktime(&tm);
    if (*timestamp == (time_t)-1) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

void ds1302_deinit(void)
{
    if (!s_ctx.initialized) return;

    gpio_reset_pin(s_ctx.clk_pin);
    gpio_reset_pin(s_ctx.data_pin);
    gpio_reset_pin(s_ctx.rst_pin);
    s_ctx.initialized = false;
    ESP_LOGI(TAG, "DS1302 deinitialized");
}