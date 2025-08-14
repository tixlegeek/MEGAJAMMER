/**
▞▚ ▚▗▘▛▚ ▛▀ ▛▚ ▛▚ ▌▐ ▙▐ ▌▞  ▞▚ ▞▚ ▙▟ ▛▚ ▞▚ ▙▐ ▚▗▘
▌▗  ▌ ▛▚ ▛  ▛▚ ▛▘ ▌▐ ▌▜ ▛▖  ▌▗ ▌▐ ▌▐ ▛▘ ▛▜ ▌▜  ▌
▝▘  ▘ ▀▘ ▀▀ ▘▝ ▘  ▝▘ ▘▝ ▘▝ ▘▝▘ ▝▘ ▘▝ ▘  ▘▝ ▘▝  ▘
      https://cyberpunk.company/tixlegeek
    @tixlegeek - tixlegeek@cyberpunk.company
    Copyright© 2024 Cyberpunk.company GPLv3
         This program is free software
                See LICENSE.txt
*/

#include "board.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "nrf24l01_reg.h"
#include "nrf24l01_smol.h"

#include "jam.h"

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

static char* TAG = "MEGAJAMMER";


trigger_state_t trigger_state = TRIGGER_OFF;
jammer_state_t jammer_state = JAMMER_OFF;

static void trigger_task(void *arg) {
  while (true) {
    trigger_state = (trigger_state_t)gpio_get_level(PIN_TRIGGER);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void trigger_init() {
  // GPIO configuration structure
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << PIN_TRIGGER), // Select the trigger pin
      .mode = GPIO_MODE_INPUT,               // Set as input
      .pull_up_en = GPIO_PULLUP_DISABLE,     // Disable pull-up
      .pull_down_en = GPIO_PULLDOWN_ENABLE,  // Disable pull-down
      .intr_type = GPIO_INTR_DISABLE         // Disable interrupt
  };
  gpio_config(&io_conf);

  trigger_state = (trigger_state_t)gpio_get_level(PIN_TRIGGER);
}

void power_relay_init() {
  // GPIO configuration structure
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << PIN_POWER_RELAY), // Select the trigger pin
      .mode = GPIO_MODE_OUTPUT_OD,               // Set as input
      .pull_up_en = GPIO_PULLUP_DISABLE,         // Disable pull-up
      .pull_down_en = GPIO_PULLDOWN_DISABLE,     // Disable pull-down
      .intr_type = GPIO_INTR_DISABLE             // Disable interrupt
  };
  gpio_config(&io_conf);

  gpio_set_level(PIN_POWER_RELAY, 0);
}


/**
 * @brief FreeRTOS task to periodically hop RF channels
 * @param arg Pointer to module context
 */
static void jam_task(void *arg) {
  nrf24_ctx_t *ctx = (nrf24_ctx_t *)arg;
  esp_task_wdt_add(NULL);

  for (;;) {

    jamming_channels_t set = g_jam_set;
    const channel_set_t *cs = &jamming_sets[set];
    jamming_callback_t cb = (jamming_callback_t)cs->cb;

    if ((cs->cb != NULL) && (jammer_state == JAMMER_ON)) {
      if(cb(ctx, cs)!=ESP_OK){
        ESP_LOGE(TAG, "%s CB returned an error.", cs->name);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_task_wdt_reset();
  }
}

const char *get_next_jammin_line() {
  static const char *lyrics[] = {
      "Ooh, yeah! All right!",
      "We're jammin', I wanna jam it with you",
      "We're jammin', jammin', and I hope you like jammin' too",
      "Ain't no rules, ain't no vow, we can do it anyhow",
      "I and I will see you through",
      "'Cause every day we pay the price with a little sacrifice",
      "Jammin' till the jam is through",
      "We're jammin'...",
      "To think that jammin' was a thing of the past",
      "We're jammin'...",
      "And I hope this jam is gonna last",
      NULL // Sentinel (marks end)
  };

  static size_t current_line = 0;

  // Reset to start if we reached the end or NULL
  if (lyrics[current_line] == NULL) {
    current_line = 0;
  }

  // Return current line and move to next
  return lyrics[current_line++];
}

static void power_relay_set(power_relay_state_t state) {
  gpio_set_level(PIN_POWER_RELAY, (uint32_t)state);
}

static void jammer_on() {
  if (jammer_state == JAMMER_ON)
    return;
  power_relay_set(POWER_RELAY_ON);
  ESP_LOGW(TAG, "%s", get_next_jammin_line());
  led_strip_fill(64, 64, 0);
  led_strip_show();
  vTaskDelay(pdMS_TO_TICKS(1000));
  jammer_state = JAMMER_ON;
  led_strip_fill(64, 0, 0);
  led_strip_show();
}

static void jammer_off() {
  if (jammer_state == JAMMER_OFF)
    return;
  jammer_state = JAMMER_OFF;
  power_relay_set(POWER_RELAY_OFF);
  ESP_LOGW(TAG, "JAM OFF");
  led_strip_fill(0, 64, 0);
  led_strip_show();
}

/**
 * @brief Main application entry point
 */
void app_main(void) {
  esp_task_wdt_config_t wdt_config = {.timeout_ms = 10000,
                                      .trigger_panic = true,
                                      .idle_core_mask = (1 << 0) | (1 << 1)};

  if (esp_task_wdt_init(&wdt_config) != ESP_OK) {
    ESP_LOGW(TAG, "TWDT already initialized");
  }
  esp_log_level_set("rmt", ESP_LOG_VERBOSE);
  spi_mutex = xSemaphoreCreateMutex();
  ESP_ERROR_CHECK(spi_mutex ? ESP_OK : ESP_FAIL);

  spi_bus_config_t buscfg = {.miso_io_num = PIN_MISO,
                             .mosi_io_num = PIN_MOSI,
                             .sclk_io_num = PIN_CLK,
                             .quadwp_io_num = -1,
                             .quadhd_io_num = -1};

  trigger_init();
  power_relay_init();

  ESP_LOGI("INIT", "led_strip_init");
  led_strip_init(PIN_LED, 24);

  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

  static const uint8_t addr[NRF_ADDR_WIDTH] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
  static nrf24_ctx_t nrf1_ctx, nrf2_ctx;

  // JAMMER Off at start
  jammer_on();
  jammer_set_channel_set(JAM_BT_CLASSIC);

  ESP_LOGI("INIT", "nrf24_init_tx");
  ESP_ERROR_CHECK(
      nrf24_init_tx(&nrf1_ctx, "NRF1", PIN_CS_TX1, PIN_CE_TX1, addr, 16));
  ESP_LOGI("INIT", "nrf24_init_tx");
  ESP_ERROR_CHECK(
      nrf24_init_tx(&nrf2_ctx, "NRF2", PIN_CS_TX2, PIN_CE_TX2, addr, 16));
  ESP_LOGI("INIT", "nrf24_set_datarate");
  ESP_ERROR_CHECK(nrf24_set_datarate(&nrf1_ctx, NRF24_DATARATE_2MBPS));
  ESP_LOGI("INIT", "nrf24_set_datarate");
  ESP_ERROR_CHECK(nrf24_set_datarate(&nrf2_ctx, NRF24_DATARATE_2MBPS));

  ESP_LOGI("INIT", "NRF1");
  xTaskCreatePinnedToCore(jam_task, "NRF1", 2048, &nrf1_ctx, 5, NULL, 0);
  ESP_LOGI("INIT", "NRF2");
  xTaskCreatePinnedToCore(jam_task, "NRF2", 2048, &nrf2_ctx, 5, NULL, 1);
  ESP_LOGI("INIT", "trigger");
  xTaskCreatePinnedToCore(trigger_task, "trigger", 2048, NULL, 5, NULL, 0);

  ESP_LOGI(TAG, "Carrier hopping tasks started");
  while (1) {
    if (trigger_state == TRIGGER_ON) {
      jammer_on();
    } else {
      jammer_off();
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
