#ifndef _NRF24L01_SMOL_H_
#define _NRF24L01_SMOL_H_
#include "nrf24l01_reg.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <string.h>

/*
static esp_err_t spi_transfer(nrf24_ctx_t *ctx, const void *tx, void *rx,
                              size_t len);
static esp_err_t nrf24_write_reg(nrf24_ctx_t *ctx, uint8_t reg, uint8_t val);
static esp_err_t nrf24_read_reg(nrf24_ctx_t *ctx, uint8_t reg, uint8_t *val);
*/
esp_err_t nrf24_set_datarate(nrf24_ctx_t *ctx, nrf24_datarate_t drate);
esp_err_t nrf24_init_tx(nrf24_ctx_t *ctx, char *name, gpio_num_t cs,
                               gpio_num_t ce,
                               const uint8_t addr[NRF_ADDR_WIDTH],
                               size_t payload_size);

esp_err_t nrf24_start_const_carrier(nrf24_ctx_t *ctx, uint8_t channel,uint8_t pwr_level);

esp_err_t nrf24_stop_const_carrier(nrf24_ctx_t *ctx) ;

extern SemaphoreHandle_t spi_mutex;

#endif /* end of include guard: _NRF24L01_SMOL_H_ */
