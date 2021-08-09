#include "gpio_expander.h"

#include <stdio.h>

#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ****************************************************************************************************************** */
/*                                                  Private Variables                                                 */
/* ****************************************************************************************************************** */
static bool gpio_ex_i2c_initialised = false;

/* ****************************************************************************************************************** */
/*                                                    Private Types                                                   */
/* ****************************************************************************************************************** */
typedef enum {
  CMD_INPUT_PORT_0 = 0,
  CMD_INPUT_PORT_1,
  CMD_OUTPUT_PORT_0,
  CMD_OUTPUT_PORT_1,
  CMD_POL_INV_PORT_0,
  CMD_POL_INV_PORT_1,
  CMD_CONFIG_PORT_0,
  CMD_CONFIG_PORT_1,
} gpio_ex_reg_t;

typedef enum {
  CMD_INPUT = 0,
  CMD_OUTPUT = 2,
  CMD_POL_INV = 4,
  CMD_CONFIG = 6,
} gpio_ex_cmd_t;

/* ****************************************************************************************************************** */
/*                                                  Private Functions                                                 */
/* ****************************************************************************************************************** */

static esp_err_t _gpio_ex_i2c_init() {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_GPIO_SDA,
      .sda_io_num = I2C_GPIO_SCL,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_FREQ_HZ,
  };
  i2c_param_config(I2C_PORT, &conf);
  i2c_driver_install(I2C_PORT, conf.mode, false, false, 0);
  return ESP_OK;
}

static uint8_t _read_from_reg(gpio_ex_reg_t reg) {
  uint8_t ret;
  uint8_t reg_casted;
  reg_casted = reg;
  i2c_master_write_read_device(I2C_PORT, I2C_ADDRESS, (uint8_t*)&reg_casted, sizeof(uint8_t), &ret, sizeof(uint8_t),
                               portMAX_DELAY);
  return ret;
}

static uint8_t _write_to_reg(gpio_ex_reg_t reg, uint8_t val) {
  const static uint8_t IND_CMD = 0, IND_DATA = 1;
  static uint8_t data[2];
  data[IND_CMD] = reg;
  data[IND_DATA] = val;
  i2c_master_write_to_device(I2C_PORT, I2C_ADDRESS, data, 2, portMAX_DELAY);
}

static bool _is_port0(gpio_ex_num_t my_pin) { return my_pin < (1ULL << 8); }

static void _update_bit(gpio_ex_cmd_t reg, uint16_t msk, bool on) {
  if (!_is_port0(msk)) {
    msk = msk >> 8;
    reg += 1;
  }
  uint8_t val = _read_from_reg(reg);
  if (on) {
    val |= msk;
  } else {
    val &= ~msk;
  }
  _write_to_reg(reg, val);
}

static bool _read_bit(gpio_ex_cmd_t reg, uint16_t msk) {
  if (!_is_port0(msk)) {
    msk = msk >> 8;
    reg += 1;
  }
  return (_read_from_reg(reg) & msk) > 0;
}

/* ****************************************************************************************************************** */
/*                                            Public Functions Definitions                                            */
/* ****************************************************************************************************************** */
esp_err_t gpio_ex_config(gpio_ex_config_t* cfg) {
  esp_err_t err;
  uint8_t data[2];
  uint8_t prev = 0;
  if (!gpio_ex_i2c_initialised) {
    if ((err = _gpio_ex_i2c_init()) != ESP_OK) {
      return err;
    }
    gpio_ex_i2c_initialised = true;
  }
  _update_bit(CMD_CONFIG, cfg->pin_bit_mask, cfg->mode == GPIO_EX_MODE_OUTPUT);
  return ESP_OK;
}

esp_err_t gpio_ex_set_level(gpio_ex_num_t gpio_ex_num, uint32_t level) { _update_bit(CMD_OUTPUT, gpio_ex_num, level); }

int gpio_ex_get_level(gpio_ex_num_t gpio_ex_num) { return _read_bit(CMD_INPUT, gpio_ex_num); }

/* ****************************************************************************************************************** */
/*                                        TODO: Not Implemented Public Methods                                        */
/* ****************************************************************************************************************** */

esp_err_t gpio_ex_intr_enable(gpio_ex_num_t gpio_ex_num) { return -1; }

esp_err_t gpio_ex_intr_disable(gpio_ex_num_t gpio_ex_num) { return -1; }

esp_err_t gpio_ex_set_direction(gpio_ex_num_t gpio_ex_num, gpio_ex_mode_t mode) { return -1; }

esp_err_t gpio_ex_is_port0r_register(void (*fn)(void*), void* arg) { return -1; }

esp_err_t gpio_ex_install_is_port0r_service(void) { return -1; }

void gpio_ex_uninstall_is_port0r_service() {}

esp_err_t gpio_ex_is_port0r_handler_remove(gpio_ex_num_t gpio_ex_num) { return -1; }

/* ****************************************************************************************************************** */
/*                                                   Testing Methods                                                  */
/* ****************************************************************************************************************** */

void __gpio_ex_reset_i2c() { gpio_ex_i2c_initialised = false; }

#ifdef __cplusplus
}
#endif