#ifndef _JAM_H_
#define _JAM_H_

#include "nrf24l01_reg.h"
#include "nrf24l01_smol.h"

#include "utils.h"
#include "esp_log.h"

#include <stdint.h>
#include <stddef.h>

typedef esp_err_t (*jamming_callback_t)(nrf24_ctx_t *ctx, void *arg);

typedef struct {
  const char *name;
  const uint8_t *list;
  size_t count;
  jamming_callback_t cb;
} channel_set_t;

/**
 * Available jamming channel sets.
 */
typedef enum {
  JAM_BLE_ADV,
  JAM_BLE_DATA,
  JAM_BT_CLASSIC,
  JAM_WIFI,
  JAM_ZIGBEE,
  JAM_BLE_ALL,
  JAM_WIFI_1_6_11,
  JAM_SAFE_NO_WIFI,
  JAM_COUNT
} jamming_channels_t;

extern volatile jamming_channels_t g_jam_set ;
/* Byte arrays (defined once in a .c file) */
extern const uint8_t nrf24_ble_adv[];
extern const uint8_t nrf24_ble_data[];
extern const uint8_t nrf24_bt_classic[];
extern const uint8_t nrf24_wifi_centers[];
extern const uint8_t nrf24_zigbee[];
extern const uint8_t nrf24_ble_all[];
extern const uint8_t nrf24_wifi_1_6_11[];
extern const uint8_t nrf24_safe_outside_wifi[];
extern const channel_set_t jamming_sets[];
extern jammer_state_t jammer_state;
void jammer_set_channel_set(jamming_channels_t set);

jamming_callback_t jamming_cb_zigbee(nrf24_ctx_t *ctx, void *arg);
jamming_callback_t jamming_cb_wifi(nrf24_ctx_t *ctx, void *arg);
jamming_callback_t jamming_cb_bt(nrf24_ctx_t *ctx, void *arg);
jamming_callback_t jamming_cb_ble(nrf24_ctx_t *ctx, void *arg);

#endif /* end of include guard: _JAM_H_ */
