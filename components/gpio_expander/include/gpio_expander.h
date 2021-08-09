// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _DRIVER_GPIO_EX_H_
#define _DRIVER_GPIO_EX_H_

#include <stdbool.h>
#include <stdint.h>

#define I2C_PORT I2C_NUM_0
#define I2C_ADDRESS 0b0100001
#define I2C_GPIO_SDA 14
#define I2C_GPIO_SCL 27
#define I2C_FREQ_HZ 100000

#ifdef __cplusplus
extern "C" {
#endif

#define BIT(nr) (1UL << (nr))

#define BIT31 0x80000000
#define BIT30 0x40000000
#define BIT29 0x20000000
#define BIT28 0x10000000
#define BIT27 0x08000000
#define BIT26 0x04000000
#define BIT25 0x02000000
#define BIT24 0x01000000
#define BIT23 0x00800000
#define BIT22 0x00400000
#define BIT21 0x00200000
#define BIT20 0x00100000
#define BIT19 0x00080000
#define BIT18 0x00040000
#define BIT17 0x00020000
#define BIT16 0x00010000
#define BIT15 0x00008000
#define BIT14 0x00004000
#define BIT13 0x00002000
#define BIT12 0x00001000
#define BIT11 0x00000800
#define BIT10 0x00000400
#define BIT9 0x00000200
#define BIT8 0x00000100
#define BIT7 0x00000080
#define BIT6 0x00000040
#define BIT5 0x00000020
#define BIT4 0x00000010
#define BIT3 0x00000008
#define BIT2 0x00000004
#define BIT1 0x00000002
#define BIT0 0x00000001

#define GPIO_EX_SEL_0 (BIT(0)) /*!< Pin 0 selected */
#define GPIO_EX_SEL_1 (BIT(1)) /*!< Pin 1 selected */
#define GPIO_EX_SEL_2                          \
  (BIT(2)) /*!< Pin 2 selected                 \
             @note There are more macros       \
             like that up to pin 39,           \
             excluding pins 20, 24 and 28..31. \
             They are not shown here           \
             to reduce redundant information. */
/** @cond */
#define GPIO_EX_SEL_3 (BIT(3))   /*!< Pin 3 selected */
#define GPIO_EX_SEL_4 (BIT(4))   /*!< Pin 4 selected */
#define GPIO_EX_SEL_5 (BIT(5))   /*!< Pin 5 selected */
#define GPIO_EX_SEL_6 (BIT(6))   /*!< Pin 6 selected */
#define GPIO_EX_SEL_7 (BIT(7))   /*!< Pin 7 selected */
#define GPIO_EX_SEL_8 (BIT(8))   /*!< Pin 8 selected */
#define GPIO_EX_SEL_9 (BIT(9))   /*!< Pin 9 selected */
#define GPIO_EX_SEL_10 (BIT(10)) /*!< Pin 10 selected */
#define GPIO_EX_SEL_11 (BIT(11)) /*!< Pin 11 selected */
#define GPIO_EX_SEL_12 (BIT(12)) /*!< Pin 12 selected */
#define GPIO_EX_SEL_13 (BIT(13)) /*!< Pin 13 selected */
#define GPIO_EX_SEL_14 (BIT(14)) /*!< Pin 14 selected */
#define GPIO_EX_SEL_15 (BIT(15)) /*!< Pin 15 selected */
#define GPIO_EX_SEL_16 (BIT(16)) /*!< Pin 16 selected */

#define GPIO_EX_APP_CPU_INTR_ENA (BIT(0))
#define GPIO_EX_APP_CPU_NMI_INTR_ENA (BIT(1))
#define GPIO_EX_PRO_CPU_INTR_ENA (BIT(2))
#define GPIO_EX_PRO_CPU_NMI_INTR_ENA (BIT(3))
#define GPIO_EX_SDIO_EXT_INTR_ENA (BIT(4))

#define GPIO_EX_MODE_DEF_DISABLE (0)
#define GPIO_EX_MODE_DEF_INPUT (BIT0)
#define GPIO_EX_MODE_DEF_OUTPUT (BIT1)
#define GPIO_EX_MODE_DEF_OD (BIT2)

#define ESP_OK 0 /*!< esp_err_t value indicating success (no error) */

/** @endcond */

#define GPIO_EX_IS_VALID_GPIO_EX(gpio_ex_num) \
  ((gpio_ex_num < GPIO_EX_PIN_COUNT &&        \
    GPIO_EX_PIN_MUX_REG[gpio_ex_num] != 0)) /*!< Check whether it is a valid GPIO_EX number */
#define GPIO_EX_IS_VALID_OUTPUT_GPIO_EX(gpio_ex_num) \
  ((GPIO_EX_IS_VALID_GPIO_EX(gpio_ex_num)) &&        \
   (gpio_ex_num < 34)) /*!< Check whether it can be a valid GPIO_EX number of output mode */

typedef int32_t esp_err_t;
typedef struct intr_handle_data_t intr_handle_data_t;
typedef intr_handle_data_t* intr_handle_t;

typedef enum {
  GPIO_EX_NUM_0 = 0,   /*!< GPIO_EX0, input and output */
  GPIO_EX_NUM_1 = 1,   /*!< GPIO_EX1, input and output */
  GPIO_EX_NUM_2 = 2,   /*!< GPIO_EX2, input and output
                         @note There are more enumerations like that
                         up to GPIO_EX39, excluding GPIO_EX20, GPIO_EX24 and GPIO_EX28..31.
                         They are not shown here to reduce redundant information.
                         @note GPIO_EX34..39 are input mode only. */
                       /** @cond */
  GPIO_EX_NUM_3 = 3,   /*!< GPIO_EX3, input and output */
  GPIO_EX_NUM_4 = 4,   /*!< GPIO_EX4, input and output */
  GPIO_EX_NUM_5 = 5,   /*!< GPIO_EX5, input and output */
  GPIO_EX_NUM_6 = 6,   /*!< GPIO_EX6, input and output */
  GPIO_EX_NUM_7 = 7,   /*!< GPIO_EX7, input and output */
  GPIO_EX_NUM_8 = 8,   /*!< GPIO_EX8, input and output */
  GPIO_EX_NUM_9 = 9,   /*!< GPIO_EX9, input and output */
  GPIO_EX_NUM_10 = 10, /*!< GPIO_EX10, input and output */
  GPIO_EX_NUM_11 = 11, /*!< GPIO_EX11, input and output */
  GPIO_EX_NUM_12 = 12, /*!< GPIO_EX12, input and output */
  GPIO_EX_NUM_13 = 13, /*!< GPIO_EX13, input and output */
  GPIO_EX_NUM_14 = 14, /*!< GPIO_EX14, input and output */
  GPIO_EX_NUM_15 = 15, /*!< GPIO_EX15, input and output */
  /** @endcond */
} gpio_ex_num_t;

typedef enum {
  GPIO_EX_MODE_DISABLE = GPIO_EX_MODE_DEF_DISABLE, /*!< GPIO_EX mode : disable input and output             */
  GPIO_EX_MODE_INPUT = GPIO_EX_MODE_DEF_INPUT,     /*!< GPIO_EX mode : input only                           */
  GPIO_EX_MODE_OUTPUT = GPIO_EX_MODE_DEF_OUTPUT,   /*!< GPIO_EX mode : output only mode                     */
} gpio_ex_mode_t;

/**
 * @brief Configuration parameters of GPIO_EX pad for gpio_ex_config function
 */
typedef struct {
  uint64_t pin_bit_mask; /*!< GPIO_EX pin: set with bit mask, each bit maps to a GPIO_EX */
  gpio_ex_mode_t mode;   /*!< GPIO_EX mode: set input/output mode                     */
} gpio_ex_config_t;

/**
 * @brief Initialises I2C if not initialised and configures the given pin to input/output as
 *          specified in the configuration structure.
 *
 * @param pgpio_ex_config configuration of a given pin.
 * @return esp_err_t
 */
esp_err_t gpio_ex_config(gpio_ex_config_t* pgpio_ex_config);

esp_err_t gpio_ex_set_level(gpio_ex_num_t gpio_ex_num, uint32_t level);

int gpio_ex_get_level(gpio_ex_num_t gpio_ex_num);

esp_err_t gpio_ex_set_direction(gpio_ex_num_t gpio_ex_num, gpio_ex_mode_t mode);

esp_err_t gpio_ex_isr_register(void (*fn)(gpio_ex_num_t, void*), void* arg);

// Test only
void __gpio_ex_reset_i2c();

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_GPIO_EX_H_ */
