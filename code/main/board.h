#ifndef _BOARD_H_
#define _BOARD_H_

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

#endif /* end of include guard: _BOARD_H_ */
