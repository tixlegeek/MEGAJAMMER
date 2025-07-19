// led_strip.c

#include "led_strip.h"
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "driver/rmt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "led_strip";

// RMT channel unique
#define RMT_CHANNEL   RMT_CHANNEL_0
// APB 80 MHz / div 8 → 10 MHz → tick = 0,1 µs
#define RMT_CLK_DIV   8

// WS2812 timing en ticks RMT (tick = 0,1 µs)
#define T0H_TICKS   3    // 0.3 µs
#define T0L_TICKS   9    // 0.9 µs
#define T1H_TICKS   9    // 0.9 µs
#define T1L_TICKS   3    // 0.3 µs
#define RESET_TICKS 500  // 50 µs / 0,1 µs

static size_t            s_led_count = 0;
static uint8_t          *s_buffer    = NULL;
static SemaphoreHandle_t s_mutex     = NULL;

esp_err_t led_strip_init(gpio_num_t gpio_num, size_t led_count)
{
    ESP_LOGI(TAG, ">> led_strip_init(gpio=%d, count=%u)", gpio_num, led_count);

    if (s_buffer) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (led_count == 0) {
        ESP_LOGE(TAG, "Invalid LED count: %u", led_count);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "Allocating buffer for %u LEDs (%u bytes)", led_count, led_count * 3);
    s_buffer = malloc(led_count * 3);
    if (!s_buffer) {
        ESP_LOGE(TAG, "Buffer allocation failed");
        return ESP_ERR_NO_MEM;
    }
    memset(s_buffer, 0, led_count * 3);
    s_led_count = led_count;
    ESP_LOGI(TAG, "Buffer allocated and zeroed");

    ESP_LOGD(TAG, "Creating mutex");
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        ESP_LOGE(TAG, "Mutex creation failed");
        free(s_buffer);
        s_buffer = NULL;
        s_led_count = 0;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Mutex created");

    ESP_LOGD(TAG, "Configuring RMT on channel %d at GPIO %d", RMT_CHANNEL, gpio_num);
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(gpio_num, RMT_CHANNEL);
    config.clk_div = RMT_CLK_DIV;
    esp_err_t err = rmt_config(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_config failed: %s", esp_err_to_name(err));
        goto _fail;
    }
    ESP_LOGI(TAG, "rmt_config succeeded");

    ESP_LOGD(TAG, "Installing RMT driver");
    err = rmt_driver_install(config.channel, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_driver_install failed: %s", esp_err_to_name(err));
        goto _fail;
    }
    ESP_LOGI(TAG, "RMT driver installed");

    ESP_LOGI(TAG, "<< led_strip_init: success");
    return ESP_OK;

_fail:
    if (s_mutex) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }
    if (s_buffer) {
        free(s_buffer);
        s_buffer = NULL;
    }
    s_led_count = 0;
    return err;
}

void led_strip_deinit(void)
{
    if (s_buffer) {
        free(s_buffer);
        s_buffer = NULL;
    }
    s_led_count = 0;

    if (s_mutex) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }

    rmt_driver_uninstall(RMT_CHANNEL);
}

esp_err_t led_strip_show(void)
{
    if (!s_buffer) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // On réserve un item par bit (24 bits par LED) + RESET
    size_t total_items = s_led_count * 24 + 1;
    rmt_item32_t *items = malloc(total_items * sizeof(rmt_item32_t));
    if (!items) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NO_MEM;
    }

    size_t idx = 0;
    for (size_t i = 0; i < s_led_count; ++i) {
        // WS2812 exige l’ordre GRB
        uint8_t grb[3] = { s_buffer[3*i + 1], s_buffer[3*i + 0], s_buffer[3*i + 2] };
        for (int c = 0; c < 3; ++c) {
            for (int bit = 7; bit >= 0; --bit) {
                bool level = (grb[c] >> bit) & 1;
                items[idx].level0   = 1;
                items[idx].duration0= level ? T1H_TICKS : T0H_TICKS;
                items[idx].level1   = 0;
                items[idx].duration1= level ? T1L_TICKS : T0L_TICKS;
                idx++;
            }
        }
    }
    // RESET à la fin (LOW long)
    items[idx].level0    = 0;
    items[idx].duration0 = RESET_TICKS;
    items[idx].level1    = 0;
    items[idx].duration1 = 0;
    idx++;

    esp_err_t err = rmt_write_items(RMT_CHANNEL, items, idx, false);
    if (err == ESP_OK) {
        rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);
    } else {
        ESP_LOGE(TAG, "rmt_write_items failed: %d", err);
    }

    free(items);
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t led_strip_set_pixel(size_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_buffer) {
        return ESP_ERR_INVALID_STATE;
    }
    if (index >= s_led_count) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    size_t pos = 3 * index;
    s_buffer[pos + 0] = r;
    s_buffer[pos + 1] = g;
    s_buffer[pos + 2] = b;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t led_strip_fill(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_buffer) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    for (size_t i = 0; i < s_led_count; ++i) {
        size_t p = 3 * i;
        s_buffer[p + 0] = r;
        s_buffer[p + 1] = g;
        s_buffer[p + 2] = b;
    }
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t led_strip_clear(void)
{
    return led_strip_fill(0, 0, 0);
}

size_t led_strip_get_length(void)
{
    return s_led_count;
}
