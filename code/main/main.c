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

#include <stdarg.h>
#include <string.h>
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

#include <stdlib.h>

#define TAG "MEGAJAMMER"

/** @brief SPI bus pin configuration */
#define PIN_MISO GPIO_NUM_19
#define PIN_MOSI GPIO_NUM_23
#define PIN_CLK GPIO_NUM_18

/** @brief TX Module 1 pin configuration */
#define PIN_CS_TX1 GPIO_NUM_25
#define PIN_CE_TX1 GPIO_NUM_33
#define PIN_IRQ_TX1 GPIO_NUM_32

/** @brief TX Module 2 pin configuration */
#define PIN_CS_TX2 GPIO_NUM_14
#define PIN_CE_TX2 GPIO_NUM_27
#define PIN_IRQ_TX2 GPIO_NUM_26

#define PIN_TRIGGER GPIO_NUM_22
#define PIN_POWER_RELAY GPIO_NUM_4
#define PIN_LED GPIO_NUM_21
/** @brief NRF24L01+ command and register definitions */
#define CMD_W_REGISTER 0x20
#define CMD_R_REGISTER 0x00
#define CMD_R_RX_PAYLOAD 0x61
#define CMD_W_TX_PAYLOAD 0xA0
#define CMD_FLUSH_RX 0xE2
#define CMD_FLUSH_TX 0xE1
#define CMD_REUSE_TX_PL 0xE3
#define CMD_ACTIVATE 0x50
#define CMD_R_RX_PL_WID 0x60
#define CMD_W_ACK_PAYLOAD 0xA8
#define CMD_W_TX_PAYLOAD_NO_ACK 0xB0
#define CMD_NOP 0xFF

#define REG_CONFIG 0x00
#define REG_EN_AA 0x01
#define REG_EN_RXADDR 0x02
#define REG_SETUP_AW 0x03
#define REG_SETUP_RETR 0x04
#define REG_RF_CH 0x05
#define REG_RF_SETUP 0x06
#define REG_STATUS 0x07
#define REG_OBSERVE_TX 0x08
#define REG_RPD 0x09
#define REG_RX_ADDR_P0 0x0A
#define REG_RX_ADDR_P1 0x0B
#define REG_RX_ADDR_P2 0x0C
#define REG_RX_ADDR_P3 0x0D
#define REG_RX_ADDR_P4 0x0E
#define REG_RX_ADDR_P5 0x0F
#define REG_TX_ADDR 0x10
#define REG_RX_PW_P0 0x11
#define REG_RX_PW_P1 0x12
#define REG_RX_PW_P2 0x13
#define REG_RX_PW_P3 0x14
#define REG_RX_PW_P4 0x15
#define REG_RX_PW_P5 0x16
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD 0x1C
#define REG_FEATURE 0x1D

#define CONFIG_PWR_UP (1 << 1)
#define CONFIG_PRIM_RX (1 << 0)

#define MASK_CONT_WAVE (1 << 7)
#define MASK_PLL_LOCK (1 << 4)
#define MASK_RF_PWR (3 << 1)

#define NRF_ADDR_WIDTH 5

/**
 * @enum nrf24_datarate_t
 * @brief Supported data rates for NRF24L01+
 */
typedef enum {
  NRF24_DATARATE_250KBPS = 0,
  NRF24_DATARATE_1MBPS = 1,
  NRF24_DATARATE_2MBPS = 2
} nrf24_datarate_t;

/**
 * @struct nrf24_ctx_t
 * @brief Holds context for one NRF24 module
 */
typedef struct {
  spi_device_handle_t spi;      /**< SPI device handle */
  gpio_num_t ce_pin;            /**< Chip Enable pin */
  gpio_num_t cs_pin;            /**< Chip Select pin */
  uint8_t channel;              /**< RF channel */
  uint8_t addr[NRF_ADDR_WIDTH]; /**< TX address */
  size_t payload_size;          /**< Payload size in bytes */
} nrf24_ctx_t;

static SemaphoreHandle_t spi_mutex;

// Bluetooth Low Energy (BLE) Advertising Channels (3 channels)
const uint8_t nrf24_ble_advertising_channels[] = {2, 26,
                                                  80};  // 2402, 2426, 2480 MHz

// Bluetooth Low Energy (BLE) Data Channels (37 channels)
const uint8_t nrf24_ble_data_channels[] = {
    4,  6,  8,  10, 12, 14, 16, 18, 20, 22,  // 2404-2422 MHz
    24, 26, 28, 30, 32, 34, 36, 38, 40, 42,  // 2424-2442 MHz
    44, 46, 48, 50, 52, 54, 56, 58, 60, 62,  // 2444-2462 MHz
    64, 66, 68, 70, 72, 74, 76               // 2464-2476 MHz
};

// Classic Bluetooth (79 channels)
const uint8_t nrf24_bluetooth_channels[] = {
    2,  4,  6,  8,  10, 12, 14, 16, 18, 20,  // 2402-2420 MHz
    22, 24, 26, 28, 30, 32, 34, 36, 38, 40,  // 2422-2440 MHz
    42, 44, 46, 48, 50, 52, 54, 56, 58, 60,  // 2442-2460 MHz
    62, 64, 66, 68, 70, 72, 74, 76, 78, 80   // 2462-2480 MHz
};

// WiFi 2.4GHz Channels (1-14)
const uint8_t nrf24_wifi_channels[] = {
    12, 17, 22, 27, 32, 37, 42, 47, 52, 57,  // Channels 1-10: 2412-2457 MHz
    62, 67, 72, 84                           // Channels 11-14: 2462-2484 MHz
};

// Zigbee Channels (11-26)
const uint8_t nrf24_zigbee_channels[] = {
    5,  10, 15, 20, 25, 30, 35, 40, 45, 50,  // Channels 11-20: 2405-2450 MHz
    55, 60, 65, 70, 75, 80                   // Channels 21-26: 2455-2480 MHz
};

typedef enum {
  TRIGGER_OFF = 0,
  TRIGGER_ON = 1,
} trigger_state_t;

typedef enum {
  POWER_RELAY_OFF = 1,
  POWER_RELAY_ON = 0,
} power_relay_state_t;

typedef enum {
  JAMMER_OFF = 0,
  JAMMER_ON = 1,

} jammer_state_t;
trigger_state_t trigger_state = TRIGGER_OFF;
jammer_state_t jammer_state = JAMMER_OFF;

static void trigger_task(void* arg) {
  while (true) {
    trigger_state = (trigger_state_t)gpio_get_level(PIN_TRIGGER);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void trigger_init() {
  // GPIO configuration structure
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << PIN_TRIGGER),  // Select the trigger pin
      .mode = GPIO_MODE_INPUT,                // Set as input
      .pull_up_en = GPIO_PULLUP_DISABLE,      // Disable pull-up
      .pull_down_en = GPIO_PULLDOWN_ENABLE,   // Disable pull-down
      .intr_type = GPIO_INTR_DISABLE          // Disable interrupt
  };
  gpio_config(&io_conf);

  trigger_state = (trigger_state_t)gpio_get_level(PIN_TRIGGER);
}

void power_relay_init() {
  // GPIO configuration structure
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << PIN_POWER_RELAY),  // Select the trigger pin
      .mode = GPIO_MODE_OUTPUT_OD,                // Set as input
      .pull_up_en = GPIO_PULLUP_DISABLE,          // Disable pull-up
      .pull_down_en = GPIO_PULLDOWN_DISABLE,      // Disable pull-down
      .intr_type = GPIO_INTR_DISABLE              // Disable interrupt
  };
  gpio_config(&io_conf);

  gpio_set_level(PIN_POWER_RELAY, 0);
}

/**
 * @brief Picks a channel from multiple arrays
 * @param array Array
 * @param size Size
 * @return A random value from the array
 */
uint8_t pick(uint8_t array[], size_t size) {
  if (array == NULL || size == 0) {
    return 0;
  }

  // Generate random index and return the element
  size_t random_index = esp_random() % size;
  return array[random_index];
}

/**
 * @brief Perform SPI transfer to a given NRF24 module
 * @param ctx Pointer to module context
 * @param tx Transmit buffer
 * @param rx Receive buffer
 * @param len Number of bytes to transfer
 * @return ESP_OK on success or error code
 */
static esp_err_t spi_transfer(nrf24_ctx_t* ctx,
                              const void* tx,
                              void* rx,
                              size_t len) {
  if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    ESP_LOGE(TAG, "SPI mutex take failed");
    return ESP_FAIL;
  }
  spi_transaction_t t = {.length = len * 8, .tx_buffer = tx, .rx_buffer = rx};
  esp_err_t ret = spi_device_polling_transmit(ctx->spi, &t);
  xSemaphoreGive(spi_mutex);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SPI transfer failed: %s", esp_err_to_name(ret));
  }
  return ret;
}

/**
 * @brief Write a single register of the NRF24 module
 * @param ctx Pointer to module context
 * @param reg Register address
 * @param val Value to write
 * @return ESP_OK on success or error code
 */
static esp_err_t nrf24_write_reg(nrf24_ctx_t* ctx, uint8_t reg, uint8_t val) {
  uint8_t buf[2] = {CMD_W_REGISTER | (reg & 0x1F), val};
  return spi_transfer(ctx, buf, NULL, sizeof(buf));
}

/**
 * @brief Read a single register of the NRF24 module
 * @param ctx Pointer to module context
 * @param reg Register address
 * @param val Pointer to store the read value
 * @return ESP_OK on success or error code
 */
static esp_err_t nrf24_read_reg(nrf24_ctx_t* ctx, uint8_t reg, uint8_t* val) {
  uint8_t buf_tx[2] = {CMD_R_REGISTER | (reg & 0x1F), 0xFF};
  uint8_t buf_rx[2] = {0};
  esp_err_t ret = spi_transfer(ctx, buf_tx, buf_rx, sizeof(buf_tx));
  if (ret == ESP_OK) {
    *val = buf_rx[1];
  }
  return ret;
}

/**
 * @brief Configure the data rate of an NRF24 module
 * @param ctx Pointer to module context
 * @param drate Desired data rate
 * @return ESP_OK on success or error code
 */
static esp_err_t nrf24_set_datarate(nrf24_ctx_t* ctx, nrf24_datarate_t drate) {
  uint8_t rf_setup;
  esp_err_t ret = nrf24_read_reg(ctx, REG_RF_SETUP, &rf_setup);
  if (ret != ESP_OK)
    return ret;

  rf_setup &= ~((1 << 3) | (1 << 2));
  switch (drate) {
    case NRF24_DATARATE_250KBPS:
      rf_setup |= (1 << 3);
      break;
    case NRF24_DATARATE_2MBPS:
      rf_setup |= (1 << 2);
      break;
    default:
      break;
  }
  return nrf24_write_reg(ctx, REG_RF_SETUP, rf_setup);
}

/**
 * @brief Initialize a NRF24 module in TX mode
 * @param ctx Pointer to module context
 * @param cs Chip Select GPIO
 * @param ce Chip Enable GPIO
 * @param addr Pointer to address array
 * @param payload_size Size of TX payload
 * @return ESP_OK on success or error code
 */
static esp_err_t nrf24_init_tx(nrf24_ctx_t* ctx,
                               gpio_num_t cs,
                               gpio_num_t ce,
                               const uint8_t addr[NRF_ADDR_WIDTH],
                               size_t payload_size) {
  ctx->cs_pin = cs;
  ctx->ce_pin = ce;
  memcpy(ctx->addr, addr, NRF_ADDR_WIDTH);
  ctx->payload_size = payload_size;

  gpio_config_t io_conf = {.pin_bit_mask = 1ULL << ce,
                           .mode = GPIO_MODE_OUTPUT};
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  gpio_set_level(ce, 0);

  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 16000000,
      .mode = 0,
      .spics_io_num = cs,
      .queue_size = 1,
  };
  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &ctx->spi));

  ESP_ERROR_CHECK(nrf24_write_reg(ctx, REG_CONFIG, CONFIG_PWR_UP));
  vTaskDelay(pdMS_TO_TICKS(150));

  return ESP_OK;
}

/**
 * @brief Start a continuous RF carrier
 * @param ctx Pointer to module context
 * @param channel RF channel to use (0-125)
 * @param pwr_level Power level (0–3)
 * @return ESP_OK on success or error code
 */
static esp_err_t nrf24_start_const_carrier(nrf24_ctx_t* ctx,
                                           uint8_t channel,
                                           uint8_t pwr_level) {
  if (channel > 125 || pwr_level > 3)
    return ESP_ERR_INVALID_ARG;

  ESP_ERROR_CHECK(nrf24_write_reg(ctx, REG_RF_CH, channel));

  uint8_t rf_setup;
  uint8_t cmd = REG_RF_SETUP;
  ESP_ERROR_CHECK(spi_transfer(ctx, &cmd, &rf_setup, 1));

  rf_setup = (rf_setup & ~MASK_RF_PWR) | ((pwr_level & 0x03) << 1);
  rf_setup |= MASK_CONT_WAVE | MASK_PLL_LOCK;
  ESP_ERROR_CHECK(nrf24_write_reg(ctx, REG_RF_SETUP, rf_setup));

  gpio_set_level(ctx->ce_pin, 1);
  return ESP_OK;
}

/**
 * @brief Stop the continuous RF carrier
 * @param ctx Pointer to module context
 * @return ESP_OK on success or error code
 */
static esp_err_t nrf24_stop_const_carrier(nrf24_ctx_t* ctx) {
  gpio_set_level(ctx->ce_pin, 0);

  uint8_t rf_setup;
  uint8_t cmd = REG_RF_SETUP;
  ESP_ERROR_CHECK(spi_transfer(ctx, &cmd, &rf_setup, 1));
  rf_setup &= ~(MASK_CONT_WAVE | MASK_PLL_LOCK);
  ESP_ERROR_CHECK(nrf24_write_reg(ctx, REG_RF_SETUP, rf_setup));
  return ESP_OK;
}

/**
 * @brief FreeRTOS task to periodically hop RF channels
 * @param arg Pointer to module context
 */
static void hop_task(void* arg) {
  nrf24_ctx_t* ctx = (nrf24_ctx_t*)arg;
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

  for (;;) {
    // Bt jamming
    // uint8_t channel =
    //    pick(nrf24_bluetooth_channels, sizeof(nrf24_bluetooth_channels) /
    //                                     sizeof(nrf24_bluetooth_channels[0]));
    // WiFi 2.4 jamming
    uint8_t channel =
        pick(nrf24_wifi_channels,
             sizeof(nrf24_wifi_channels) / sizeof(nrf24_wifi_channels[0]));
    // ZigBee jamming
    // uint8_t channel = pick(nrf24_zigbee_channels,
    // sizeof(nrf24_zigbee_channels)/sizeof(nrf24_zigbee_channels[0]));
    nrf24_stop_const_carrier(ctx);
    if (jammer_state == JAMMER_ON) {
      nrf24_start_const_carrier(ctx, channel, 3);
      vTaskDelay(pdMS_TO_TICKS(15));
    } else {
      vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_ERROR_CHECK(esp_task_wdt_reset());
  }
}

const char* get_next_jammin_line() {
  static const char* lyrics[] = {
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
      NULL  // Sentinel (marks end)
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

  ESP_LOGI("INIT", "nrf24_init_tx");
  ESP_ERROR_CHECK(nrf24_init_tx(&nrf1_ctx, PIN_CS_TX1, PIN_CE_TX1, addr, 16));
  ESP_LOGI("INIT", "nrf24_init_tx");
  ESP_ERROR_CHECK(nrf24_init_tx(&nrf2_ctx, PIN_CS_TX2, PIN_CE_TX2, addr, 16));
  ESP_LOGI("INIT", "nrf24_set_datarate");
  ESP_ERROR_CHECK(nrf24_set_datarate(&nrf1_ctx, NRF24_DATARATE_2MBPS));
  ESP_LOGI("INIT", "nrf24_set_datarate");
  ESP_ERROR_CHECK(nrf24_set_datarate(&nrf2_ctx, NRF24_DATARATE_2MBPS));

  ESP_LOGI("INIT", "NRF1");
  xTaskCreatePinnedToCore(hop_task, "NRF1", 2048, &nrf1_ctx, 5, NULL, 0);
  ESP_LOGI("INIT", "NRF2");
  xTaskCreatePinnedToCore(hop_task, "NRF2", 2048, &nrf2_ctx, 5, NULL, 1);
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
