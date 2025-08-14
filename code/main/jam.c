#include "jam.h"

/**
 * BLE advertising channels: 2402, 2426, 2480 MHz.
 * RF_CH values: 2, 26, 80.
 */
const uint8_t nrf24_ble_adv[] = {2, 26, 80};

/**
 * BLE data channels: 2404..2476 MHz, step 2 MHz.
 * RF_CH values: 4, 6, ..., 76.
 */
const uint8_t nrf24_ble_data[] = {
    4,  6,  8,  10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40,
    42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76};

/**
 * Bluetooth Classic channels: 2402..2480 MHz, all integers.
 * RF_CH values: 2..80.
 */
const uint8_t nrf24_bt_classic[] = {
    2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17,
    18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33,
    34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
    50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65,
    66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80};

/**
 * Wi-Fi 2.4 GHz channel centers (channels 1..14).
 * RF_CH values: 12, 17, 22, ..., 84.
 */
const uint8_t nrf24_wifi_centers[] = {12, 17, 22, 27, 32, 37, 42,
                                      47, 52, 57, 62, 67, 72, 84};

/**
 * Zigbee channels 11..26: 2405..2480 MHz, step 5 MHz.
 * RF_CH values: 5, 10, ..., 80.
 */
const uint8_t nrf24_zigbee[] = {5,  10, 15, 20, 25, 30, 35, 40,
                                45, 50, 55, 60, 65, 70, 75, 80};

/**
 * BLE all channels (advertising + data, no duplicates).
 */
const uint8_t nrf24_ble_all[] = {2,  4,  6,  8,  10, 12, 14, 16, 18, 20,
                                 22, 24, 26, 28, 30, 32, 34, 36, 38, 40,
                                 42, 44, 46, 48, 50, 52, 54, 56, 58, 60,
                                 62, 64, 66, 68, 70, 72, 74, 76, 80};

/**
 * Wi-Fi non-overlapping channels 1, 6, 11.
 */
const uint8_t nrf24_wifi_1_6_11[] = {12, 37, 62};

/**
 * Channels outside typical 20 MHz Wi-Fi coverage.
 * Excludes ±10 MHz around Wi-Fi 1, 6, and 11.
 */
const uint8_t nrf24_safe_outside_wifi[] = {
    0,   1,   23,  24,  25,  26,  48,  49,  50,  51,  73,  74,  75,
    76,  77,  78,  79,  80,  81,  82,  83,  84,  85,  86,  87,  88,
    89,  90,  91,  92,  93,  94,  95,  96,  97,  98,  99,  100, 101,
    102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114,
    115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125};

// Currently selected jamming set.
volatile jamming_channels_t g_jam_set = JAM_WIFI;
/**
 * Registry of all channel sets.
 */
const channel_set_t jamming_sets[JAM_COUNT] = {
    [JAM_BLE_ADV] = {"BLE_ADV", nrf24_ble_adv, COUNT_OF(nrf24_ble_adv),
                     jamming_cb_ble},
    [JAM_BLE_DATA] = {"BLE_DATA", nrf24_ble_data, COUNT_OF(nrf24_ble_data),
                      jamming_cb_ble},
    [JAM_BT_CLASSIC] = {"BT_CLASSIC", nrf24_bt_classic,

                        COUNT_OF(nrf24_bt_classic), jamming_cb_bt},
    [JAM_WIFI] = {"WIFI_CENTERS", nrf24_wifi_centers,

                  COUNT_OF(nrf24_wifi_centers), jamming_cb_wifi},
    [JAM_ZIGBEE] = {"ZIGBEE", nrf24_zigbee, COUNT_OF(nrf24_zigbee), jamming_cb_zigbee},
    [JAM_BLE_ALL] = {"BLE_ALL", nrf24_ble_all, COUNT_OF(nrf24_ble_all),
                     jamming_cb_ble},
    [JAM_WIFI_1_6_11] = {"WIFI_1_6_11", nrf24_wifi_1_6_11,
                         COUNT_OF(nrf24_wifi_1_6_11), jamming_cb_wifi},
    [JAM_SAFE_NO_WIFI] = {"SAFE_NO_WIFI", nrf24_safe_outside_wifi,
                          COUNT_OF(nrf24_safe_outside_wifi), jamming_cb_bt}};

void jamming_channels_debug(jamming_channels_t set) {
  switch (set) {
  case JAM_BLE_ADV:
    ESP_LOGI("JAM_SET", "Setting jamming on channels JAM_BLE_ADV");
    break;
  case JAM_BLE_DATA:
    ESP_LOGI("JAM_SET", "Setting jamming on channels JAM_BLE_DATA");
    break;
  case JAM_BT_CLASSIC:
    ESP_LOGI("JAM_SET", "Setting jamming on channels JAM_BT_CLASSIC");
    break;
  case JAM_WIFI:
    ESP_LOGI("JAM_SET", "Setting jamming on channels JAM_WIFI");
    break;
  case JAM_ZIGBEE:
    ESP_LOGI("JAM_SET", "Setting jamming on channels JAM_ZIGBEE");
    break;
  case JAM_BLE_ALL:
    ESP_LOGI("JAM_SET", "Setting jamming on channels JAM_BLE_ALL");
    break;
  case JAM_WIFI_1_6_11:
    ESP_LOGI("JAM_SET", "Setting jamming on channels JAM_WIFI_1_6_11");
    break;
  case JAM_SAFE_NO_WIFI:
    ESP_LOGI("JAM_SET", "Setting jamming on channels JAM_SAFE_NO_WIFI");
    break;
  default:
    ESP_LOGE("JAM_SET", "RANGE ERROR");
    break;
  }
}

/**
 * @brief Set the channel list to jamm on
 * @param set a jamming_channels_t jamming set
 */
void jammer_set_channel_set(jamming_channels_t set) {
  jamming_channels_debug(set);
  if (set < JAM_COUNT) {
    g_jam_set = set;
  }
}

/*

*/
jamming_callback_t jamming_cb_zigbee(nrf24_ctx_t *ctx, void *arg) {
  channel_set_t *channel_set = (channel_set_t *)arg;
  //ESP_LOGI(__FUNCTION__, "%s ON %s", channel_set->name, ctx->name);

  /* TODO:
    Here, channel hoping is not a good strategy.
  */

  return ESP_ERR_NOT_SUPPORTED;
}
/*

*/
jamming_callback_t jamming_cb_wifi(nrf24_ctx_t *ctx, void *arg) {
  channel_set_t *channel_set = (channel_set_t *)arg;
  //ESP_LOGI(__FUNCTION__, "%s ON %s", channel_set->name, ctx->name);

  /* TODO:
    Here, channel hoping would be useless. To do: add a user interface so we can choose the channel?
  */

  return ESP_ERR_NOT_SUPPORTED;
}

/*
  Channel hoping (BT/BLE)
*/
jamming_callback_t jamming_cb_bt(nrf24_ctx_t *ctx, void *arg) {
  channel_set_t *channel_set = (channel_set_t *)arg;
  uint8_t channel = pick(channel_set->list, channel_set->count);
  //ESP_LOGI(__FUNCTION__, "%s ON %s:%d", channel_set->name, ctx->name, channel);
  nrf24_stop_const_carrier(ctx);
  nrf24_start_const_carrier(ctx, channel, 3);
  return ESP_OK;
}

/*

*/
jamming_callback_t jamming_cb_ble(nrf24_ctx_t *ctx, void *arg) {
  channel_set_t *channel_set = (channel_set_t *)arg;
  uint8_t channel = pick(channel_set->list, channel_set->count);
  //ESP_LOGI(__FUNCTION__, "%s ON %s:%d", channel_set->name, ctx->name, channel);
  nrf24_stop_const_carrier(ctx);
  nrf24_start_const_carrier(ctx, channel, 3);
  return ESP_OK;
}
