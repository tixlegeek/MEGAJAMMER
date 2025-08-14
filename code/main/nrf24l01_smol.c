#include "nrf24l01_smol.h"
static char* TAG = "NRF24L01";

SemaphoreHandle_t spi_mutex;
/**
 * @brief Perform SPI transfer to a given NRF24 module
 * @param ctx Pointer to module context
 * @param tx Transmit buffer
 * @param rx Receive buffer
 * @param len Number of bytes to transfer
 * @return ESP_OK on success or error code
 */
static esp_err_t spi_transfer(nrf24_ctx_t *ctx, const void *tx, void *rx,
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
static esp_err_t nrf24_write_reg(nrf24_ctx_t *ctx, uint8_t reg, uint8_t val) {
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
static esp_err_t nrf24_read_reg(nrf24_ctx_t *ctx, uint8_t reg, uint8_t *val) {
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
esp_err_t nrf24_set_datarate(nrf24_ctx_t *ctx, nrf24_datarate_t drate) {
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
esp_err_t nrf24_init_tx(nrf24_ctx_t *ctx, char *name, gpio_num_t cs,
                               gpio_num_t ce,
                               const uint8_t addr[NRF_ADDR_WIDTH],
                               size_t payload_size) {
  memset(ctx->name, 0, 32);
  strncpy(ctx->name, name, 32);
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
esp_err_t nrf24_start_const_carrier(nrf24_ctx_t *ctx, uint8_t channel,
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
esp_err_t nrf24_stop_const_carrier(nrf24_ctx_t *ctx) {
  gpio_set_level(ctx->ce_pin, 0);

  uint8_t rf_setup;
  uint8_t cmd = REG_RF_SETUP;
  ESP_ERROR_CHECK(spi_transfer(ctx, &cmd, &rf_setup, 1));
  rf_setup &= ~(MASK_CONT_WAVE | MASK_PLL_LOCK);
  ESP_ERROR_CHECK(nrf24_write_reg(ctx, REG_RF_SETUP, rf_setup));
  return ESP_OK;
}
