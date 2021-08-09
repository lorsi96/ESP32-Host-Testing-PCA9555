/**
 * @file test_gpio_expander.c
 * @author Lucas Orsi (lorsi@itba.edu.ar)
 * @version 0.1
 * @date 2021-08-08
 * @copyright Copyright (c) 2021
 *
 */

/* ****************************************************************************************************************** */
/*                                                     Inclusions                                                     */
/* ****************************************************************************************************************** */

#include "gpio_expander.h"
#include "mock_gpio.h"
#include "mock_i2c.h"
#include "unity.h"

/* ****************************************************************************************************************** */
/*                                             Assertion Macros/Functions                                             */
/* ****************************************************************************************************************** */
static inline void TEST_ASSERT_I2C_WRITE_PARAMS(uint8_t hist_ind, uint8_t* data, uint8_t len) {
  TEST_ASSERT_EQUAL_HEX(I2C_PORT, i2c_master_write_to_device_fake.arg0_history[hist_ind]);
  TEST_ASSERT_EQUAL_HEX(I2C_ADDRESS, i2c_master_write_to_device_fake.arg1_history[hist_ind]);
  for (uint8_t i = 0; i < len; i++) {
    TEST_ASSERT_EQUAL_HEX(data[i], i2c_master_write_to_device_fake.arg2_history[hist_ind][i]);
  }
  TEST_ASSERT_EQUAL(len, i2c_master_write_to_device_fake.arg3_history[hist_ind]);
}

static inline void TEST_ASSERT_I2C_READ_PARAMS(uint8_t hist_ind, uint8_t reg) {
  TEST_ASSERT_EQUAL_HEX(I2C_PORT, i2c_master_write_read_device_fake.arg0_history[hist_ind]);
  TEST_ASSERT_EQUAL_HEX(I2C_ADDRESS, i2c_master_write_read_device_fake.arg1_history[hist_ind]);
  TEST_ASSERT_EQUAL_HEX(reg, ((uint8_t*)(i2c_master_write_read_device_fake.arg2_history[hist_ind]))[0]);
}

/* ****************************************************************************************************************** */
/*                                                  Private Variables                                                 */
/* ****************************************************************************************************************** */
static uint8_t fake_read_data = 0x00;

/* ****************************************************************************************************************** */
/*                                                  Private Functions                                                 */
/* ****************************************************************************************************************** */
static esp_err_t i2c_master_fake_read_device(i2c_port_t i2c_num, uint8_t device_address, const uint8_t* write_buffer,
                                             size_t write_size, uint8_t* read_buffer, size_t read_size,
                                             TickType_t ticks_to_wait) {
  *read_buffer = fake_read_data;
  return ESP_OK;
}

/* ****************************************************************************************************************** */
/*                                                Tests SetUp/TearDown                                                */
/* ****************************************************************************************************************** */
void setUp() {
  FFF_RESET_HISTORY();
  RESET_FAKE(i2c_master_read);
  RESET_FAKE(i2c_master_write_to_device);
  RESET_FAKE(i2c_master_write_read_device);
  RESET_FAKE(i2c_master_write_byte);
  fake_read_data = 0;
  __gpio_ex_reset_i2c();
}

void tearDown() {}

/* ****************************************************************************************************************** */
/*                                                   Concrete Tests                                                   */
/* ****************************************************************************************************************** */
void test_configure_initialises_i2c_correctly() {
  gpio_ex_config_t test_cfg = {.mode = GPIO_EX_MODE_OUTPUT, .pin_bit_mask = BIT12};

  gpio_ex_config(&test_cfg);

  TEST_ASSERT_CALLED_TIMES(1, i2c_param_config);
  TEST_ASSERT_CALLED_TIMES(1, i2c_driver_install);
}

void test_configure_initialises_i2c_only_once() {
  gpio_ex_config_t test_cfg = {.mode = GPIO_EX_MODE_OUTPUT, .pin_bit_mask = BIT12};
  gpio_ex_config_t test_cfg2 = {.mode = GPIO_EX_MODE_OUTPUT, .pin_bit_mask = BIT3};

  gpio_ex_config(&test_cfg);
  gpio_ex_config(&test_cfg2);

  TEST_ASSERT_CALLED_TIMES(1, i2c_param_config);
  TEST_ASSERT_CALLED_TIMES(1, i2c_driver_install);
}

void test_configure_initialises_output_pin_correctly_port_1() {
  gpio_ex_config_t test_cfg = {.mode = GPIO_EX_MODE_OUTPUT, .pin_bit_mask = BIT12};
  uint8_t exp[] = {0x07, BIT12 >> 8};

  gpio_ex_config(&test_cfg);

  TEST_ASSERT_I2C_WRITE_PARAMS(0, exp, sizeof(exp) / sizeof(uint8_t));
}

void test_configure_initialises_output_pin_correctly_port_0() {
  gpio_ex_config_t test_cfg = {.mode = GPIO_EX_MODE_OUTPUT, .pin_bit_mask = BIT7};

  uint8_t data[] = {0x06,  // Port 0 Configuration.
                    BIT7};

  gpio_ex_config(&test_cfg);

  TEST_ASSERT_I2C_WRITE_PARAMS(0, data, sizeof(data) / sizeof(uint8_t));
}

void test_configure_initialises_input_pin_correctly_port_1() {
  gpio_ex_config_t test_cfg = {.mode = GPIO_EX_MODE_INPUT, .pin_bit_mask = BIT12};

  uint8_t data[] = {0x07, 0b00000000};

  gpio_ex_config(&test_cfg);

  TEST_ASSERT_I2C_WRITE_PARAMS(0, data, sizeof(data) / sizeof(uint8_t));
}

void test_configure_initialises_input_pin_correctly_port_0() {
  gpio_ex_config_t test_cfg = {.mode = GPIO_EX_MODE_INPUT, .pin_bit_mask = BIT7};

  uint8_t data[] = {0x06, 0b00000000};

  gpio_ex_config(&test_cfg);

  TEST_ASSERT_I2C_WRITE_PARAMS(0, data, sizeof(data) / sizeof(uint8_t));
}

void test_configure_initialises_output_pin_independently() {
  gpio_ex_config_t cfg = {.mode = GPIO_EX_MODE_OUTPUT, .pin_bit_mask = BIT3};

  uint8_t data2[] = {0x06, BIT3 | BIT5};

  fake_read_data = BIT5;
  i2c_master_write_read_device_fake.custom_fake = i2c_master_fake_read_device;
  gpio_ex_config(&cfg);

  TEST_ASSERT_I2C_READ_PARAMS(0, 0x06);
  TEST_ASSERT_I2C_WRITE_PARAMS(0, data2, sizeof(data2) / sizeof(uint8_t));
}

void test_configure_initialises_input_pin_independently() {
  gpio_ex_config_t cfg = {.mode = GPIO_EX_MODE_INPUT, .pin_bit_mask = BIT15};
  uint8_t expected_wr[] = {0x07, 0x7F};

  fake_read_data = 0xFF;
  i2c_master_write_read_device_fake.custom_fake = i2c_master_fake_read_device;

  gpio_ex_config(&cfg);

  TEST_ASSERT_I2C_READ_PARAMS(0, 0x07);
  TEST_ASSERT_I2C_WRITE_PARAMS(0, expected_wr, sizeof(expected_wr) / sizeof(uint8_t));
}

void test_set_level_port0_independently() {
  gpio_ex_num_t test_pin = BIT6;
  uint8_t test_port0_prev_state = 0x0A;

  uint8_t expect[] = {0x02, 0x4A};

  fake_read_data = test_port0_prev_state;
  i2c_master_write_read_device_fake.custom_fake = i2c_master_fake_read_device;

  gpio_ex_set_level(test_pin, true);

  TEST_ASSERT_I2C_WRITE_PARAMS(0, expect, sizeof(expect) / sizeof(uint8_t));
}

void test_set_level_port1_independently() {
  gpio_ex_num_t test_pin = BIT9;
  uint8_t test_port1_prev_state = 0xA0;

  uint8_t expect[] = {0x03, 0xA2};

  fake_read_data = test_port1_prev_state;
  i2c_master_write_read_device_fake.custom_fake = i2c_master_fake_read_device;

  gpio_ex_set_level(test_pin, true);

  TEST_ASSERT_I2C_WRITE_PARAMS(0, expect, sizeof(expect) / sizeof(uint8_t));
}

void test_get_level_port0_independently() {
  gpio_ex_num_t test_pin = BIT7;
  uint8_t test_port0_prev_state = 0x80;

  fake_read_data = test_port0_prev_state;
  i2c_master_write_read_device_fake.custom_fake = i2c_master_fake_read_device;

  bool res = gpio_ex_get_level(test_pin);

  TEST_ASSERT_I2C_READ_PARAMS(0, 0x00);
  TEST_ASSERT_TRUE(res);
}

void test_get_level_port1_independently() {
  gpio_ex_num_t test_port1_pin = BIT8;
  uint8_t test_port1_prev_state = 0xFE;

  fake_read_data = test_port1_prev_state;
  i2c_master_write_read_device_fake.custom_fake = i2c_master_fake_read_device;

  bool res = gpio_ex_get_level(test_port1_pin);

  TEST_ASSERT_I2C_READ_PARAMS(0, 0x01);
  TEST_ASSERT_FALSE(res);
}
