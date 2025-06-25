#include "icm20948.h"
#include "icm20948_config.h"
#include "icm20948_registers.h"
#include <assert.h>
#include <string.h>

static icm20948_err_t icm20948_bus_initialize(icm20948_t const *icm20948) {
  return icm20948->interface.bus_initialize
             ? icm20948->interface.bus_initialize(icm20948->interface.bus_user)
             : ICM20948_ERR_NULL;
}

static icm20948_err_t icm20948_bus_deinitialize(icm20948_t const *icm20948) {
  return icm20948->interface.bus_deinitialize
             ? icm20948->interface.bus_deinitialize(
                   icm20948->interface.bus_user)
             : ICM20948_ERR_NULL;
}

static icm20948_err_t icm20948_bus_write_data(icm20948_t const *icm20948,
                                              uint8_t address,
                                              uint8_t const *data,
                                              size_t data_size) {
  return icm20948->interface.bus_write_data
             ? icm20948->interface.bus_write_data(icm20948->interface.bus_user,
                                                  address, data, data_size)
             : ICM20948_ERR_NULL;
}

static icm20948_err_t icm20948_bus_read_data(icm20948_t const *icm20948,
                                             uint8_t address, uint8_t *data,
                                             size_t data_size) {
  return icm20948->interface.bus_read_data
             ? icm20948->interface.bus_read_data(icm20948->interface.bus_user,
                                                 address, data, data_size)
             : ICM20948_ERR_NULL;
}

static icm20948_err_t icm20948_bank_write(icm20948_t const *icm20948,
                                          icm20948_bank_t bank, uint8_t address,
                                          uint8_t const *data,
                                          size_t data_size) {
  icm20948_err_t err = icm20948_bank_select(icm20948, bank);
  err |= icm20948_bus_write_data(icm20948, address, data, data_size);

  return err;
}

static icm20948_err_t icm20948_bank_read(icm20948_t const *icm20948,
                                         icm20948_bank_t bank, uint8_t address,
                                         uint8_t *data, size_t data_size) {
  icm20948_err_t err = icm20948_bank_select(icm20948, bank);
  err |= icm20948_bus_read_data(icm20948, address, data, data_size);

  return err;
}

icm20948_err_t icm20948_initialize(icm20948_t *icm20948,
                                   icm20948_config_t const *config,
                                   icm20948_interface_t const *interface) {
  assert(icm20948 && config && interface);

  memset(icm20948, 0, sizeof(*icm20948));
  memcpy(&icm20948->config, config, sizeof(*config));
  memcpy(&icm20948->interface, interface, sizeof(*interface));

  return icm20948_bus_initialize(icm20948);
}

icm20948_err_t icm20948_deinitialize(icm20948_t *icm20948) {
  assert(icm20948);

  icm20948_err_t err = icm20948_bus_deinitialize(icm20948);

  memset(icm20948, 0, sizeof(*icm20948));

  return err;
}

icm20948_err_t icm20948_get_accel_data_x_scaled(icm20948_t const *icm20948,
                                                float32_t *scaled) {
  assert(icm20948 && scaled);

  int16_t raw = {};

  icm20948_err_t err = icm20948_get_accel_data_x_raw(icm20948, &raw);

  *scaled = (float32_t)raw * icm20948->config.accel_scale;

  return err;
}

icm20948_err_t icm20948_get_accel_data_y_scaled(icm20948_t const *icm20948,
                                                float32_t *scaled) {
  assert(icm20948 && scaled);

  int16_t raw = {};

  icm20948_err_t err = icm20948_get_accel_data_y_raw(icm20948, &raw);

  *scaled = (float32_t)raw * icm20948->config.accel_scale;

  return err;
}

icm20948_err_t icm20948_get_accel_data_z_scaled(icm20948_t const *icm20948,
                                                float32_t *scaled) {
  assert(icm20948 && scaled);

  int16_t raw = {};

  icm20948_err_t err = icm20948_get_accel_data_z_raw(icm20948, &raw);

  *scaled = (float32_t)raw * icm20948->config.accel_scale;

  return err;
}

icm20948_err_t icm20948_get_accel_data_scaled(icm20948_t const *icm20948,
                                              vec3_float32_t *scaled) {
  assert(icm20948 && scaled);

  vec3_int16_t raw = {};

  icm20948_err_t err = icm20948_get_accel_data_raw(icm20948, &raw);

  scaled->x = (float32_t)raw.x * icm20948->config.accel_scale;
  scaled->y = (float32_t)raw.y * icm20948->config.accel_scale;
  scaled->z = (float32_t)raw.z * icm20948->config.accel_scale;

  return err;
}

icm20948_err_t icm20948_get_accel_data_x_raw(icm20948_t const *icm20948,
                                             int16_t *raw) {
  assert(icm20948 && raw);

  icm20948_accel_xout_reg_t reg = {};

  icm20948_err_t err = icm20948_get_accel_xout_reg(icm20948, &reg);

  *raw = reg.accel_xout;

  return err;
}

icm20948_err_t icm20948_get_accel_data_y_raw(icm20948_t const *icm20948,
                                             int16_t *raw) {
  assert(icm20948 && raw);

  icm20948_accel_yout_reg_t reg = {};

  icm20948_err_t err = icm20948_get_accel_yout_reg(icm20948, &reg);

  *raw = reg.accel_yout;

  return err;
}

icm20948_err_t icm20948_get_accel_data_z_raw(icm20948_t const *icm20948,
                                             int16_t *raw) {
  assert(icm20948 && raw);

  icm20948_accel_zout_reg_t reg = {};

  icm20948_err_t err = icm20948_get_accel_zout_reg(icm20948, &reg);

  *raw = reg.accel_zout;

  return err;
}

icm20948_err_t icm20948_get_accel_data_raw(icm20948_t const *icm20948,
                                           vec3_int16_t *raw) {
  assert(icm20948 && raw);

  icm20948_accel_out_reg_t reg = {};

  icm20948_err_t err = icm20948_get_accel_out_reg(icm20948, &reg);

  raw->x = reg.accel_xout;
  raw->y = reg.accel_yout;
  raw->z = reg.accel_zout;

  return err;
}

icm20948_err_t icm20948_get_gyro_data_x_scaled(icm20948_t const *icm20948,
                                               float32_t *scaled) {
  assert(icm20948 && scaled);

  int16_t raw = {};

  icm20948_err_t err = icm20948_get_gyro_data_x_raw(icm20948, &raw);

  *scaled = (float32_t)raw * icm20948->config.gyro_scale;

  return err;
}

icm20948_err_t icm20948_get_gyro_data_y_scaled(icm20948_t const *icm20948,
                                               float32_t *scaled) {
  assert(icm20948 && scaled);

  int16_t raw = {};

  icm20948_err_t err = icm20948_get_gyro_data_y_raw(icm20948, &raw);

  *scaled = (float32_t)raw * icm20948->config.gyro_scale;

  return err;
}

icm20948_err_t icm20948_get_gyro_data_z_scaled(icm20948_t const *icm20948,
                                               float32_t *scaled) {
  assert(icm20948 && scaled);

  int16_t raw = {};

  icm20948_err_t err = icm20948_get_gyro_data_z_raw(icm20948, &raw);

  *scaled = (float32_t)raw * icm20948->config.gyro_scale;

  return err;
}

icm20948_err_t icm20948_get_gyro_data_scaled(icm20948_t const *icm20948,
                                             vec3_float32_t *scaled) {
  assert(icm20948 && scaled);

  vec3_int16_t raw = {};

  icm20948_err_t err = icm20948_get_gyro_data_raw(icm20948, &raw);

  scaled->x = (float32_t)raw.x * icm20948->config.gyro_scale;
  scaled->y = (float32_t)raw.y * icm20948->config.gyro_scale;
  scaled->z = (float32_t)raw.z * icm20948->config.gyro_scale;

  return err;
}

icm20948_err_t icm20948_get_gyro_data_x_raw(icm20948_t const *icm20948,
                                            int16_t *raw) {
  assert(icm20948 && raw);

  icm20948_gyro_xout_reg_t reg = {};

  icm20948_err_t err = icm20948_get_gyro_xout_reg(icm20948, &reg);

  *raw = reg.gyro_xout;

  return err;
}

icm20948_err_t icm20948_get_gyro_data_y_raw(icm20948_t const *icm20948,
                                            int16_t *raw) {
  assert(icm20948 && raw);

  icm20948_gyro_yout_reg_t reg = {};

  icm20948_err_t err = icm20948_get_gyro_yout_reg(icm20948, &reg);

  *raw = reg.gyro_yout;

  return err;
}

icm20948_err_t icm20948_get_gyro_data_z_raw(icm20948_t const *icm20948,
                                            int16_t *raw) {
  assert(icm20948 && raw);

  icm20948_gyro_zout_reg_t reg = {};

  icm20948_err_t err = icm20948_get_gyro_zout_reg(icm20948, &reg);

  *raw = reg.gyro_zout;

  return err;
}

icm20948_err_t icm20948_get_gyro_data_raw(icm20948_t const *icm20948,
                                          vec3_int16_t *raw) {
  assert(icm20948 && raw);

  icm20948_gyro_out_reg_t reg = {};

  icm20948_err_t err = icm20948_get_gyro_out_reg(icm20948, &reg);

  raw->x = reg.gyro_xout;
  raw->y = reg.gyro_yout;
  raw->z = reg.gyro_zout;

  return err;
}

icm20948_err_t icm20948_get_temp_data_scaled(icm20948_t const *icm20948,
                                             float32_t *scaled) {
  assert(icm20948 && scaled);

  int16_t raw = {};

  icm20948_err_t err = icm20948_get_temp_data_raw(icm20948, &raw);

  *scaled = (float32_t)raw * ICM20948_TEMP_SCALE;

  return err;
}

icm20948_err_t icm20948_get_temp_data_raw(icm20948_t const *icm20948,
                                          int16_t *raw) {
  assert(icm20948 && raw);

  icm20948_temp_out_reg_t reg = {};

  icm20948_err_t err = icm20948_get_temp_out_reg(icm20948, &reg);

  *raw = reg.temp_out;

  return err;
}

icm20948_err_t icm20948_ext_slv_read(icm20948_t const *icm20948,
                                     icm20948_slave_num_t slave_num,
                                     uint8_t address, uint8_t *data,
                                     uint8_t data_size) {
  assert(icm20948 && data);

  icm20948_i2c_slv_addr_reg_t slv_addr_reg = {};
  icm20948_err_t err =
      icm20948_get_i2c_slv_addr_reg(icm20948, slave_num, &slv_addr_reg);
  slv_addr_reg.i2c_slv_rw = true;
  err |= icm20948_set_i2c_slv_addr_reg(icm20948, slave_num, &slv_addr_reg);

  icm20948_i2c_slv_reg_t slv_reg = {};
  err |= icm20948_get_i2c_slv_reg(icm20948, slave_num, &slv_reg);
  slv_reg.i2c_slv_reg = address;
  err |= icm20948_set_i2c_slv_reg(icm20948, slave_num, &slv_reg);

  icm20948_i2c_slv_ctrl_reg_t slv_ctrl_reg = {};
  err |= icm20948_get_i2c_slv_ctrl_reg(icm20948, slave_num, &slv_ctrl_reg);
  slv_ctrl_reg.i2c_slv_leng = data_size & 0x0FU;
  err |= icm20948_set_i2c_slv_ctrl_reg(icm20948, slave_num, &slv_ctrl_reg);

  err |= icm20948_bank_read(icm20948, ICM20948_BANK_0,
                            ICM20948_REG_ADDRESS_EXT_SLV_SENS_DATA_00, data,
                            data_size);

  return err;
}

icm20948_err_t icm20948_ext_slv_write(icm20948_t const *icm20948,
                                      icm20948_slave_num_t slave_num,
                                      uint8_t address, uint8_t const *data,
                                      uint8_t data_size) {
  assert(icm20948 && data);

  icm20948_i2c_slv_addr_reg_t slv_addr_reg = {};
  icm20948_err_t err =
      icm20948_get_i2c_slv_addr_reg(icm20948, slave_num, &slv_addr_reg);
  slv_addr_reg.i2c_slv_rw = false;
  err |= icm20948_set_i2c_slv_addr_reg(icm20948, slave_num, &slv_addr_reg);

  icm20948_i2c_slv_reg_t slv_reg = {};
  err |= icm20948_get_i2c_slv_reg(icm20948, slave_num, &slv_reg);
  slv_reg.i2c_slv_reg = address;
  err |= icm20948_set_i2c_slv_reg(icm20948, slave_num, &slv_reg);

  icm20948_i2c_slv_ctrl_reg_t slv_ctrl_reg = {};
  err |= icm20948_get_i2c_slv_ctrl_reg(icm20948, slave_num, &slv_ctrl_reg);
  slv_ctrl_reg.i2c_slv_leng = data_size & 0x0FU;
  err |= icm20948_set_i2c_slv_ctrl_reg(icm20948, slave_num, &slv_ctrl_reg);

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_0, slave_num, data,
                             data_size);

  return err;
}

icm20948_err_t icm20948_fifo_read(icm20948_t const *icm20948, uint8_t *data,
                                  uint8_t data_size) {
  assert(icm20948 && data);

  return icm20948_bank_read(icm20948, ICM20948_BANK_0,
                            ICM20948_REG_ADDRESS_FIFO_R_W, data, data_size);
}

icm20948_err_t icm20948_fifo_write(icm20948_t const *icm20948,
                                   uint8_t const *data, uint8_t data_size) {
  assert(icm20948 && data);

  return icm20948_bank_write(icm20948, ICM20948_BANK_0,
                             ICM20948_REG_ADDRESS_FIFO_R_W, data, data_size);
}

icm20948_err_t icm20948_bank_select(icm20948_t const *icm20948,
                                    icm20948_bank_t bank) {
  assert(icm20948);

  icm20948_bank_sel_reg_t reg = {.user_bank = bank};

  return icm20948_set_reg_bank_sel_reg(icm20948, &reg);
}

icm20948_err_t icm20948_get_reg_bank_sel_reg(icm20948_t const *icm20948,
                                             icm20948_bank_sel_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bus_write_data(
      icm20948, ICM20948_REG_ADDRESS_BANK_SEL, &data, sizeof(data));

  reg->user_bank = (data >> 4U) & 0x03U;

  return err;
}

icm20948_err_t
icm20948_set_reg_bank_sel_reg(icm20948_t const *icm20948,
                              icm20948_bank_sel_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data = (reg->user_bank & 0x03U) << 4U;

  return icm20948_bus_write_data(icm20948, ICM20948_REG_ADDRESS_BANK_SEL, &data,
                                 sizeof(data));
}

icm20948_err_t icm20948_get_who_am_i_reg(icm20948_t const *icm20948,
                                         icm20948_who_am_i_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_WHO_AM_I, &data, sizeof(data));

  reg->who_am_i = data & 0xFFU;

  return err;
}

icm20948_err_t icm20948_get_user_ctrl_reg(icm20948_t const *icm20948,
                                          icm20948_user_ctrl_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_USER_CTRL, &data, sizeof(data));

  reg->dmp_en = (data >> 7U) & 0x01U;
  reg->fifo_en = (data >> 6U) & 0x01U;
  reg->i2c_mst_en = (data >> 5U) & 0x01U;
  reg->i2c_if_dis = (data >> 4U) & 0x01U;
  reg->dmp_rst = (data >> 3U) & 0x01U;
  reg->sram_rst = (data >> 2U) & 0x01U;
  reg->i2c_mst_rst = (data >> 1U) & 0x01U;

  return err;
}

icm20948_err_t icm20948_set_user_ctrl_reg(icm20948_t const *icm20948,
                                          icm20948_user_ctrl_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_USER_CTRL, &data, sizeof(data));

  data &= ~((0x01U << 7U) | (0x01U << 6U) | (0x01U << 5U) | (0x01U << 4U) |
            (0x01U << 3U) | (0x01U << 2U) | (0x01U << 1U));

  data |= (reg->dmp_en & 0x01U) << 7U;
  data |= (reg->fifo_en & 0x01U) << 6U;
  data |= (reg->i2c_mst_en & 0x01U) << 5U;
  data |= (reg->i2c_if_dis & 0x01U) << 4U;
  data |= (reg->dmp_rst & 0x01U) << 3U;
  data |= (reg->sram_rst & 0x01U) << 2U;
  data |= (reg->i2c_mst_rst & 0x01U) << 1U;

  err |=
      icm20948_bank_write(icm20948, ICM20948_BANK_0,
                          ICM20948_REG_ADDRESS_USER_CTRL, &data, sizeof(data));

  return err;
}

icm20948_err_t icm20948_get_lp_config_reg(icm20948_t const *icm20948,
                                          icm20948_lp_config_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_LP_CONFIG, &data, sizeof(data));

  reg->i2c_mst_cycle = (data >> 6U) & 0x01U;
  reg->accel_cycle = (data >> 5U) & 0x01U;
  reg->gyro_cycle = (data >> 4U) & 0x01U;

  return err;
}

icm20948_err_t icm20948_set_lp_config_reg(icm20948_t const *icm20948,
                                          icm20948_lp_config_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_LP_CONFIG, &data, sizeof(data));

  data &= ~((0x01U << 6U) | (0x01U << 5U) | (0x01 << 4U));

  data |= (reg->i2c_mst_cycle & 0x01U) << 6U;
  data |= (reg->accel_cycle & 0x01U) << 5U;
  data |= (reg->gyro_cycle & 0x01U) << 4U;

  err |=
      icm20948_bank_write(icm20948, ICM20948_BANK_0,
                          ICM20948_REG_ADDRESS_LP_CONFIG, &data, sizeof(data));

  return err;
}

icm20948_err_t icm20948_get_pwr_mgmt_1_reg(icm20948_t const *icm20948,
                                           icm20948_pwr_mgmt_1_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_PWR_MGMT_1, &data, sizeof(data));

  reg->device_reset = (data >> 7U) & 0x01U;
  reg->sleep = (data >> 6U) & 0x01U;
  reg->lp_en = (data >> 5U) & 0x01U;
  reg->temp_dis = (data >> 3U) & 0x01U;
  reg->clksel = data & 0x07U;

  return err;
}

icm20948_err_t
icm20948_set_pwr_mgmt_1_reg(icm20948_t const *icm20948,
                            icm20948_pwr_mgmt_1_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_PWR_MGMT_1, &data, sizeof(data));

  data &=
      ~((0x01U << 7U) | (0x01U << 6U) | (0x01U << 5U) | (0x01U << 3U) | 0x07U);

  data |= (reg->device_reset & 0x01U) << 7U;
  data |= (reg->sleep & 0x01U) << 6U;
  data |= (reg->lp_en & 0x01U) << 5U;
  data |= (reg->temp_dis & 0x01U) << 3U;
  data |= reg->clksel & 0x07U;

  err |=
      icm20948_bank_write(icm20948, ICM20948_BANK_0,
                          ICM20948_REG_ADDRESS_PWR_MGMT_1, &data, sizeof(data));

  return err;
}

icm20948_err_t icm20948_get_pwr_mgmt_2_reg(icm20948_t const *icm20948,
                                           icm20948_pwr_mgmt_2_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_PWR_MGMT_2, &data, sizeof(data));

  reg->disable_accel = (data >> 3U) & 0x07U;
  reg->disable_gyro = data & 0x07U;

  return err;
}

icm20948_err_t
icm20948_set_pwr_mgmt_2_reg(icm20948_t const *icm20948,
                            icm20948_pwr_mgmt_2_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_PWR_MGMT_2, &data, sizeof(data));

  data &= ~((0x07U << 3U) | 0x07U);

  data |= (reg->disable_accel & 0x07U) << 3U;
  data |= reg->disable_gyro & 0x07U;

  err |=
      icm20948_bank_write(icm20948, ICM20948_BANK_0,
                          ICM20948_REG_ADDRESS_PWR_MGMT_2, &data, sizeof(data));

  return err;
}

icm20948_err_t icm20948_get_int_pin_cfg_reg(icm20948_t const *icm20948,
                                            icm20948_int_pin_cfg_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_INT_PIN_CFG, &data, sizeof(data));

  reg->int1_actl = (data >> 7U) & 0x01U;
  reg->int1_open = (data >> 6U) & 0x01U;
  reg->int1_latch_en = (data >> 5U) & 0x01U;
  reg->int_anyrd_2clear = (data >> 4U) & 0x01U;
  reg->actl_fsync = (data >> 3U) & 0x01U;
  reg->fsync_int_mode_en = (data >> 2U) & 0x01U;
  reg->bypass_en = (data >> 1U) & 0x01U;

  return err;
}

icm20948_err_t
icm20948_set_int_pin_cfg_reg(icm20948_t const *icm20948,
                             icm20948_int_pin_cfg_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_INT_PIN_CFG, &data, sizeof(data));

  data &= ~((0x01U << 7U) | (0x01U << 6U) | (0x01U << 5U) | (0x01U << 4U) |
            (0x01U << 3U) | (0x01U << 2U) | (0x01U << 1U));

  data |= (reg->int1_actl & 0x01U) << 7U;
  data |= (reg->int1_open & 0x01U) << 6U;
  data |= (reg->int1_latch_en & 0x01U) << 5U;
  data |= (reg->int_anyrd_2clear & 0x01U) << 4U;
  data |= (reg->actl_fsync & 0x01U) << 3U;
  data |= (reg->fsync_int_mode_en & 0x01U) << 2U;
  data |= (reg->bypass_en & 0x01U) << 1U;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_0,
                             ICM20948_REG_ADDRESS_INT_PIN_CFG, &data,
                             sizeof(data));

  return err;
}

icm20948_err_t icm20948_get_int_enable_reg(icm20948_t const *icm20948,
                                           icm20948_int_enable_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_INT_ENABLE, &data, sizeof(data));

  reg->reg_wof_en = (data >> 7U) & 0x01U;
  reg->wom_int_en = (data >> 3U) & 0x01U;
  reg->pll_rdy_en = (data >> 2U) & 0x01U;
  reg->dmp_int1_en = (data >> 1U) & 0x01U;
  reg->i2c_mst_int_en = data & 0x01U;

  return err;
}

icm20948_err_t
icm20948_set_int_enable_reg(icm20948_t const *icm20948,
                            icm20948_int_enable_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_INT_ENABLE, &data, sizeof(data));

  data &=
      ~((0x01U << 7U) | (0x01U << 3U) | (0x01U << 2U) | (0x01U << 1U) | 0x01U);

  data |= (reg->reg_wof_en & 0x01U) << 7U;
  data |= (reg->wom_int_en & 0x01U) << 3U;
  data |= (reg->pll_rdy_en & 0x01U) << 2U;
  data |= (reg->dmp_int1_en & 0x01U) << 1U;
  data |= reg->i2c_mst_int_en & 0x01U;

  err |=
      icm20948_bank_write(icm20948, ICM20948_BANK_0,
                          ICM20948_REG_ADDRESS_INT_ENABLE, &data, sizeof(data));

  return err;
}

icm20948_err_t icm20948_get_int_enable_1_reg(icm20948_t const *icm20948,
                                             icm20948_int_enable_1_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_0,
                                          ICM20948_REG_ADDRESS_INT_ENABLE_1,
                                          &data, sizeof(data));

  reg->raw_data_0_rdy_en = data & 0x7FU;

  return err;
}

icm20948_err_t
icm20948_set_int_enable_1_reg(icm20948_t const *icm20948,
                              icm20948_int_enable_1_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_0,
                                          ICM20948_REG_ADDRESS_INT_ENABLE_1,
                                          &data, sizeof(data));

  data &= ~0x7FU;

  data |= reg->raw_data_0_rdy_en & 0x7FU;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_0,
                             ICM20948_REG_ADDRESS_INT_ENABLE_1, &data,
                             sizeof(data));

  return err;
}

icm20948_err_t icm20948_get_int_enable_2_reg(icm20948_t const *icm20948,
                                             icm20948_int_enable_2_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_0,
                                          ICM20948_REG_ADDRESS_INT_ENABLE_2,
                                          &data, sizeof(data));

  reg->fifo_overflow_en = data & 0x1FU;

  return err;
}

icm20948_err_t
icm20948_set_int_enable_2_reg(icm20948_t const *icm20948,
                              icm20948_int_enable_2_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_0,
                                          ICM20948_REG_ADDRESS_INT_ENABLE_2,
                                          &data, sizeof(data));

  data &= ~0x1FU;

  data |= reg->fifo_overflow_en & 0x1FU;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_0,
                             ICM20948_REG_ADDRESS_INT_ENABLE_2, &data,
                             sizeof(data));

  return err;
}

icm20948_err_t icm20948_get_int_enable_3_reg(icm20948_t const *icm20948,
                                             icm20948_int_enable_3_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_0,
                                          ICM20948_REG_ADDRESS_INT_ENABLE_3,
                                          &data, sizeof(data));

  reg->fifo_wm_en = data & 0x1FU;

  return err;
}

icm20948_err_t
icm20948_set_int_enable_3_reg(icm20948_t const *icm20948,
                              icm20948_int_enable_3_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_0,
                                          ICM20948_REG_ADDRESS_INT_ENABLE_3,
                                          &data, sizeof(data));

  data &= ~0x7FU;

  data |= reg->fifo_wm_en & 0x7FU;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_0,
                             ICM20948_REG_ADDRESS_INT_ENABLE_3, &data,
                             sizeof(data));

  return err;
}

icm20948_err_t
icm20948_get_i2c_mst_status_reg(icm20948_t const *icm20948,
                                icm20948_i2c_mst_status_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_0,
                                          ICM20948_REG_ADDRESS_I2C_MST_STATUS,
                                          &data, sizeof(data));

  reg->pass_through = (data >> 7U) & 0x01U;
  reg->i2c_slv4_done = (data >> 6U) & 0x01U;
  reg->i2c_lost_arb = (data >> 5U) & 0x01U;
  reg->i2c_slv4_nack = (data >> 4U) & 0x01U;
  reg->i2c_slv3_nack = (data >> 3U) & 0x01U;
  reg->i2c_slv2_nack = (data >> 1U) & 0x01U;
  reg->i2c_slv1_nack = (data >> 1U) & 0x01U;
  reg->i2c_slv0_nack = data & 0x01U;

  return err;
}

icm20948_err_t icm20948_get_int_status_reg(icm20948_t const *icm20948,
                                           icm20948_int_status_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_INT_STATUS, &data, sizeof(data));

  reg->wom_int = (data >> 3U) & 0x01U;
  reg->pll_rdy_int = (data >> 2U) & 0x01U;
  reg->dmp_int1 = (data >> 1U) & 0x01U;
  reg->i2c_mst_int = data & 0x01U;

  return err;
}

icm20948_err_t icm20948_get_int_status_1_reg(icm20948_t const *icm20948,
                                             icm20948_int_status_1_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_0,
                                          ICM20948_REG_ADDRESS_INT_STATUS_1,
                                          &data, sizeof(data));

  reg->raw_data_0_rdy_int = data & 0x01U;

  return err;
}

icm20948_err_t icm20948_get_int_status_2_reg(icm20948_t const *icm20948,
                                             icm20948_int_status_2_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_0,
                                          ICM20948_REG_ADDRESS_INT_STATUS_2,
                                          &data, sizeof(data));

  reg->fifo_overflow_int = data & 0x1FU;

  return err;
}

icm20948_err_t icm20948_get_int_status_3_reg(icm20948_t const *icm20948,
                                             icm20948_int_status_3_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_0,
                                          ICM20948_REG_ADDRESS_INT_STATUS_3,
                                          &data, sizeof(data));

  reg->fifo_wm_int = data & 0x1FU;

  return err;
}

icm20948_err_t icm20948_get_delay_time_reg(icm20948_t const *icm20948,
                                           icm20948_delay_time_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_DELAY_TIMEH, data, sizeof(data));

  reg->delay_time = (uint16_t)(((data[0] & 0xFFU) << 8U) | (data[1] & 0xFFU));

  return err;
}

icm20948_err_t icm20948_get_accel_xout_reg(icm20948_t const *icm20948,
                                           icm20948_accel_xout_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_ACCEL_XOUT_H, data, sizeof(data));

  reg->accel_xout = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

  return err;
}

icm20948_err_t icm20948_get_accel_yout_reg(icm20948_t const *icm20948,
                                           icm20948_accel_yout_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_ACCEL_YOUT_H, data, sizeof(data));

  reg->accel_yout = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

  return err;
}

icm20948_err_t icm20948_get_accel_zout_reg(icm20948_t const *icm20948,
                                           icm20948_accel_zout_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_ACCEL_ZOUT_H, data, sizeof(data));

  reg->accel_zout = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

  return err;
}

icm20948_err_t icm20948_get_accel_out_reg(icm20948_t const *icm20948,
                                          icm20948_accel_out_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[6] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_ACCEL_XOUT_H, data, sizeof(data));

  reg->accel_xout = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));
  reg->accel_yout = (int16_t)(((data[2] & 0xFF) << 8) | (data[3] & 0xFF));
  reg->accel_zout = (int16_t)(((data[4] & 0xFF) << 8) | (data[5] & 0xFF));

  return err;
}

icm20948_err_t icm20948_get_gyro_xout_reg(icm20948_t const *icm20948,
                                          icm20948_gyro_xout_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_GYRO_XOUT_H, data, sizeof(data));

  reg->gyro_xout = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

  return err;
}

icm20948_err_t icm20948_get_gyro_yout_reg(icm20948_t const *icm20948,
                                          icm20948_gyro_yout_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_GYRO_YOUT_H, data, sizeof(data));

  reg->gyro_yout = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

  return err;
}

icm20948_err_t icm20948_get_gyro_zout_reg(icm20948_t const *icm20948,
                                          icm20948_gyro_zout_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_GYRO_ZOUT_H, data, sizeof(data));

  reg->gyro_zout = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

  return err;
}

icm20948_err_t icm20948_get_gyro_out_reg(icm20948_t const *icm20948,
                                         icm20948_gyro_out_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[6] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_GYRO_XOUT_H, data, sizeof(data));

  reg->gyro_xout = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));
  reg->gyro_yout = (int16_t)(((data[2] & 0xFF) << 8) | (data[3] & 0xFF));
  reg->gyro_zout = (int16_t)(((data[4] & 0xFF) << 8) | (data[5] & 0xFF));

  return err;
}

icm20948_err_t icm20948_get_temp_out_reg(icm20948_t const *icm20948,
                                         icm20948_temp_out_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_TEMP_OUT_H, data, sizeof(data));

  reg->temp_out = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

  return err;
}

icm20948_err_t
icm20948_get_ext_slv_sens_data_reg(icm20948_t const *icm20948, uint8_t reg_num,
                                   icm20948_ext_slv_sens_data_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(
      icm20948, ICM20948_BANK_0,
      ICM20948_REG_ADDRESS_EXT_SLV_SENS_DATA_00 + (reg_num & 0x17U), &data,
      sizeof(data));

  reg->ext_slv_sens_data = data & 0xFFU;

  return err;
}

icm20948_err_t icm20948_get_fifo_en_1_reg(icm20948_t const *icm20948,
                                          icm20948_fifo_en_1_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_FIFO_EN_1, &data, sizeof(data));

  reg->slv_3_fifo_en = (data >> 3U) & 0x01U;
  reg->slv_2_fifo_en = (data >> 2U) & 0x01U;
  reg->slv_1_fifo_en = (data >> 1U) & 0x01U;
  reg->slv_0_fifo_en = data & 0x01U;

  return err;
}

icm20948_err_t icm20948_set_fifo_en_1_reg(icm20948_t const *icm20948,
                                          icm20948_fifo_en_1_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_FIFO_EN_1, &data, sizeof(data));

  data &= ~((0x01U << 3U) | (0x01U << 2U) | (0x01U << 1U) | 0x01U);

  data |= (reg->slv_3_fifo_en & 0x01U) << 3U;
  data |= (reg->slv_2_fifo_en & 0x01U) << 2U;
  data |= (reg->slv_1_fifo_en & 0x01U) << 1U;
  data |= reg->slv_0_fifo_en & 0x01U;

  err |=
      icm20948_bank_write(icm20948, ICM20948_BANK_0,
                          ICM20948_REG_ADDRESS_FIFO_EN_1, &data, sizeof(data));

  return err;
}

icm20948_err_t icm20948_get_fifo_en_2_reg(icm20948_t const *icm20948,
                                          icm20948_fifo_en_2_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_FIFO_EN_2, &data, sizeof(data));

  reg->accel_fifo_en = (data >> 4U) & 0x01U;
  reg->gyro_z_fifo_en = (data >> 3U) & 0x01U;
  reg->gyro_y_fifo_en = (data >> 2U) & 0x01U;
  reg->gyro_x_fifo_en = (data >> 1U) & 0x01U;
  reg->temp_fifo_en = data & 0x01U;

  return err;
}

icm20948_err_t icm20948_set_fifo_en_2_reg(icm20948_t const *icm20948,
                                          icm20948_fifo_en_2_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_FIFO_EN_2, &data, sizeof(data));

  data &=
      ~((0x01U << 4U) | (0x01U << 3U) | (0x01U << 2U) | (0x01U << 1U) | 0x01U);

  data |= (reg->accel_fifo_en & 0x01U) << 4U;
  data |= (reg->gyro_z_fifo_en & 0x01U) << 3U;
  data |= (reg->gyro_y_fifo_en & 0x01U) << 2U;
  data |= (reg->gyro_x_fifo_en & 0x01U) << 1U;
  data |= reg->temp_fifo_en & 0x01U;

  err |=
      icm20948_bank_write(icm20948, ICM20948_BANK_0,
                          ICM20948_REG_ADDRESS_FIFO_EN_2, &data, sizeof(data));
}

icm20948_err_t icm20948_get_fifo_rst_reg(icm20948_t const *icm20948,
                                         icm20948_fifo_rst_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_FIFO_RST, &data, sizeof(data));

  reg->fifo_reset = data & 0x1FU;

  return err;
}

icm20948_err_t icm20948_set_fifo_rst_reg(icm20948_t const *icm20948,
                                         icm20948_fifo_rst_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_FIFO_RST, &data, sizeof(data));

  data &= ~0x1FU;

  data |= reg->fifo_reset & 0x1FU;

  err |=
      icm20948_bank_write(icm20948, ICM20948_BANK_0,
                          ICM20948_REG_ADDRESS_FIFO_RST, &data, sizeof(data));
}

icm20948_err_t icm20948_get_fifo_mode_reg(icm20948_t const *icm20948,
                                          icm20948_fifo_mode_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_FIFO_MODE, &data, sizeof(data));

  reg->fifo_mode = data & 0x1FU;

  return err;
}

icm20948_err_t icm20948_set_fifo_mode_reg(icm20948_t const *icm20948,
                                          icm20948_fifo_mode_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_FIFO_MODE, &data, sizeof(data));

  data &= ~0x1FU;

  data |= reg->fifo_mode & 0x1FU;

  err |=
      icm20948_bank_write(icm20948, ICM20948_BANK_0,
                          ICM20948_REG_ADDRESS_FIFO_MODE, &data, sizeof(data));
}

icm20948_err_t icm20948_get_fifo_cnt_reg(icm20948_t const *icm20948,
                                         icm20948_fifo_cnt_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_FIFO_COUNTH, data, sizeof(data));

  reg->fifo_cnt = (uint16_t)(((data[0] & 0x1FU) << 8U) | (data[1] & 0xFFU));

  return err;
}

icm20948_err_t icm20948_get_fifo_r_w_reg(icm20948_t const *icm20948,
                                         icm20948_fifo_r_w_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_FIFO_R_W, &data, sizeof(data));

  reg->fifo_r_w = data & 0xFFU;

  return err;
}

icm20948_err_t icm20948_set_fifo_r_w_reg(icm20948_t const *icm20948,
                                         icm20948_fifo_r_w_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data = reg->fifo_r_w & 0xFFU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_0,
                             ICM20948_REG_ADDRESS_FIFO_R_W, &data,
                             sizeof(data));
}

icm20948_err_t
icm20948_get_data_rdy_status(icm20948_t const *icm20948,
                             icm20948_data_rdy_status_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_0,
                                          ICM20948_REG_ADDRESS_DATA_RDY_STATUS,
                                          &data, sizeof(data));

  reg->wof_status = (data >> 7U) & 0x01U;
  reg->raw_data_rdy = data & 0x0FU;

  return err;
}

icm20948_err_t icm20948_get_fifo_cfg_reg(icm20948_t const *icm20948,
                                         icm20948_fifo_cfg_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_FIFO_CFG, &data, sizeof(data));

  reg->fifo_cfg = data & 0x01U;

  return err;
}

icm20948_err_t icm20948_set_fifo_cfg_reg(icm20948_t const *icm20948,
                                         icm20948_fifo_cfg_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_0,
                         ICM20948_REG_ADDRESS_FIFO_CFG, &data, sizeof(data));

  data &= ~0x01U;

  data |= reg->fifo_cfg & 0x01U;

  err |= icm20948_bank_read(icm20948, ICM20948_BANK_0,
                            ICM20948_REG_ADDRESS_FIFO_CFG, &data, sizeof(data));
}

icm20948_err_t
icm20948_get_self_test_x_gyro_reg(icm20948_t const *icm20948,
                                  icm20948_selt_test_x_gyro_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_1,
                                          ICM20948_REG_ADDRESS_SELF_TEST_X_GYRO,
                                          &data, sizeof(data));

  reg->xg_st_data = data & 0xFFU;

  return err;
}

icm20948_err_t
icm20948_set_self_test_x_gyro_reg(icm20948_t const *icm20948,
                                  icm20948_selt_test_x_gyro_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data = reg->xg_st_data & 0xFFU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_1,
                             ICM20948_REG_ADDRESS_SELF_TEST_X_GYRO, &data,
                             sizeof(data));
}

icm20948_err_t
icm20948_get_self_test_y_gyro_reg(icm20948_t const *icm20948,
                                  icm20948_self_test_y_gyro_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_1,
                                          ICM20948_REG_ADDRESS_SELF_TEST_Y_GYRO,
                                          &data, sizeof(data));

  reg->yg_st_data = data & 0xFFU;

  return err;
}

icm20948_err_t
icm20948_set_self_test_y_gyro_reg(icm20948_t const *icm20948,
                                  icm20948_self_test_y_gyro_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data = reg->yg_st_data & 0xFFU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_1,
                             ICM20948_REG_ADDRESS_SELF_TEST_Y_GYRO, &data,
                             sizeof(data));
}

icm20948_err_t
icm20948_get_self_test_z_gyro_reg(icm20948_t const *icm20948,
                                  icm20948_self_test_z_gyro_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_1,
                                          ICM20948_REG_ADDRESS_SELF_TEST_Z_GYRO,
                                          &data, sizeof(data));

  reg->zg_st_data = data & 0xFFU;

  return err;
}

icm20948_err_t
icm20948_set_self_test_z_gyro_reg(icm20948_t const *icm20948,
                                  icm20948_self_test_z_gyro_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data = reg->zg_st_data & 0xFFU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_1,
                             ICM20948_REG_ADDRESS_SELF_TEST_Z_GYRO, &data,
                             sizeof(data));
}

icm20948_err_t
icm20948_get_self_test_x_accel_reg(icm20948_t const *icm20948,
                                   icm20948_self_test_x_accel_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(
      icm20948, ICM20948_BANK_1, ICM20948_REG_ADDRESS_SELF_TEST_X_ACCEL, &data,
      sizeof(data));

  reg->xa_st_data = data & 0xFFU;

  return err;
}

icm20948_err_t icm20948_set_self_test_x_accel_reg(
    icm20948_t const *icm20948, icm20948_self_test_x_accel_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data = reg->xa_st_data & 0xFFU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_1,
                             ICM20948_REG_ADDRESS_SELF_TEST_X_ACCEL, &data,
                             sizeof(data));
}

icm20948_err_t
icm20948_get_self_test_y_accel_reg(icm20948_t const *icm20948,
                                   icm20948_self_test_y_gyro_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(
      icm20948, ICM20948_BANK_1, ICM20948_REG_ADDRESS_SELF_TEST_Y_ACCEL, &data,
      sizeof(data));

  reg->yg_st_data = data & 0xFFU;

  return err;
}

icm20948_err_t
icm20948_set_self_test_y_accel_reg(icm20948_t const *icm20948,
                                   icm20948_self_test_y_gyro_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data = reg->yg_st_data & 0xFFU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_1,
                             ICM20948_REG_ADDRESS_SELF_TEST_Y_ACCEL, &data,
                             sizeof(data));
}

icm20948_err_t
icm20948_get_self_test_z_accel_reg(icm20948_t const *icm20948,
                                   icm20948_self_test_z_gyro_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(
      icm20948, ICM20948_BANK_1, ICM20948_REG_ADDRESS_SELF_TEST_Z_ACCEL, &data,
      sizeof(data));

  reg->zg_st_data = data & 0xFFU;

  return err;
}

icm20948_err_t
icm20948_set_self_test_z_accel_reg(icm20948_t const *icm20948,
                                   icm20948_self_test_z_gyro_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data = reg->zg_st_data & 0xFFU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_1,
                             ICM20948_REG_ADDRESS_SELF_TEST_Z_ACCEL, &data,
                             sizeof(data));
}

icm20948_err_t icm20948_get_xa_offs_reg(icm20948_t const *icm20948,
                                        icm20948_xa_offs_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_1,
                         ICM20948_REG_ADDRESS_XA_OFFS_H, data, sizeof(data));

  reg->xa_offs = (int16_t)(((data[0] & 0xFF) << 7) | ((data[1] >> 1) & 0x7F));

  return err;
}

icm20948_err_t icm20948_set_xa_offs_reg(icm20948_t const *icm20948,
                                        icm20948_xa_offs_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_1,
                         ICM20948_REG_ADDRESS_XA_OFFS_H, data, sizeof(data));

  data[0] &= ~0xFF;
  data[1] &= ~(0x7F << 1);

  data[0] |= (uint8_t)((reg->xa_offs >> 8) & 0xFF);
  data[0] |= (uint8_t)((reg->xa_offs & 0x7F) << 1);

  err |=
      icm20948_bank_write(icm20948, ICM20948_BANK_1,
                          ICM20948_REG_ADDRESS_XA_OFFS_H, data, sizeof(data));

  return err;
}

icm20948_err_t icm20948_get_ya_offs_reg(icm20948_t const *icm20948,
                                        icm20948_ya_offs_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_1,
                         ICM20948_REG_ADDRESS_YA_OFFS_H, data, sizeof(data));

  reg->ya_offs = (int16_t)(((data[0] & 0xFF) << 7) | ((data[1] >> 1) & 0x7F));

  return err;
}

icm20948_err_t icm20948_set_ya_offs_reg(icm20948_t const *icm20948,
                                        icm20948_ya_offs_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_1,
                         ICM20948_REG_ADDRESS_YA_OFFS_H, data, sizeof(data));

  data[0] &= ~0xFF;
  data[1] &= ~(0x7F << 1);

  data[0] |= (uint8_t)((reg->ya_offs >> 8) & 0xFF);
  data[0] |= (uint8_t)((reg->ya_offs & 0x7F) << 1);

  err |=
      icm20948_bank_write(icm20948, ICM20948_BANK_1,
                          ICM20948_REG_ADDRESS_YA_OFFS_H, data, sizeof(data));

  return err;
}

icm20948_err_t icm20948_get_za_offs_reg(icm20948_t const *icm20948,
                                        icm20948_za_offs_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_1,
                         ICM20948_REG_ADDRESS_ZA_OFFS_H, data, sizeof(data));

  reg->za_offs = (int16_t)(((data[0] & 0xFF) << 7) | ((data[1] >> 1) & 0x7F));

  return err;
}

icm20948_err_t icm20948_set_za_offs_reg(icm20948_t const *icm20948,
                                        icm20948_za_offs_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_1,
                         ICM20948_REG_ADDRESS_ZA_OFFS_H, data, sizeof(data));

  data[0] &= ~0xFF;
  data[1] &= ~(0x7F << 1);

  data[0] |= (uint8_t)((reg->za_offs >> 8) & 0xFF);
  data[0] |= (uint8_t)((reg->za_offs & 0x7F) << 1);

  err |=
      icm20948_bank_write(icm20948, ICM20948_BANK_1,
                          ICM20948_REG_ADDRESS_ZA_OFFS_H, data, sizeof(data));

  return err;
}

icm20948_err_t icm20948_get_tbc_pll_reg(icm20948_t const *icm20948,
                                        icm20948_tbc_pll_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_1,
                         ICM20948_REG_ADDRESS_TBC_PLL, &data, sizeof(data));

  reg->tbc_pll = data & 0xFFU;

  return err;
}

icm20948_err_t icm20948_set_tbc_pll_reg(icm20948_t const *icm20948,
                                        icm20948_tbc_pll_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data = reg->tbc_pll & 0xFFU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_1,
                             ICM20948_REG_ADDRESS_TBC_PLL, &data, sizeof(data));
}

icm20948_err_t
icm20948_get_gyro_smplrt_div_reg(icm20948_t const *icm20948,
                                 icm20948_gyro_smplrt_div_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_GYRO_SMPLRT_DIV,
                                          &data, sizeof(data));

  reg->gyro_smplrt_div = data & 0xFFU;

  return err;
}

icm20948_err_t
icm20948_set_gyro_smplrt_div_reg(icm20948_t const *icm20948,
                                 icm20948_gyro_smplrt_div_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data = reg->gyro_smplrt_div & 0xFFU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_2,
                             ICM20948_REG_ADDRESS_GYRO_SMPLRT_DIV, &data,
                             sizeof(data));
}

icm20948_err_t
icm20948_get_gyro_config_1_reg(icm20948_t const *icm20948,
                               icm20948_gyro_config_1_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_GYRO_CONFIG_1,
                                          &data, sizeof(data));

  reg->gyro_dplfcfg = (data >> 3U) & 0x7U;
  reg->gyro_fs_sel = (data >> 1U) & 0x03U;
  reg->gyro_fchoice = data & 0x01U;

  return err;
}

icm20948_err_t
icm20948_set_gyro_config_1_reg(icm20948_t const *icm20948,
                               icm20948_gyro_config_1_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_GYRO_CONFIG_1,
                                          &data, sizeof(data));

  data &= ~((0x07U << 3U) | (0x03U << 1U) | 0x01U);

  data |= (reg->gyro_dplfcfg & 0x07U) << 3U;
  data |= (reg->gyro_fs_sel & 0x03U) << 1U;
  data |= reg->gyro_fchoice & 0x01U;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_2,
                             ICM20948_REG_ADDRESS_GYRO_CONFIG_1, &data,
                             sizeof(data));

  return err;
}

icm20948_err_t
icm20948_get_gyro_config_2_reg(icm20948_t const *icm20948,
                               icm20948_gyro_config_2_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_GYRO_CONFIG_2,
                                          &data, sizeof(data));

  reg->xgyro_cten = (data >> 5U) & 0x01U;
  reg->ygyro_cten = (data >> 4U) & 0x01U;
  reg->zgyro_cten = (data >> 3U) & 0x01U;
  reg->gyro_avgcfg = data & 0x07U;

  return err;
}

icm20948_err_t
icm20948_set_gyro_config_2_reg(icm20948_t const *icm20948,
                               icm20948_gyro_config_2_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_GYRO_CONFIG_2,
                                          &data, sizeof(data));

  data &= ~((0x01U << 5U) | (0x01U << 4U) | (0x01U << 3U) | 0x07U);

  data |= (reg->xgyro_cten & 0x01U) << 5U;
  data |= (reg->ygyro_cten & 0x01U) << 4U;
  data |= (reg->zgyro_cten & 0x01U) << 3U;
  data |= reg->gyro_avgcfg & 0x07U;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_2,
                             ICM20948_REG_ADDRESS_GYRO_CONFIG_2, &data,
                             sizeof(data));

  return err;
}

icm20948_err_t icm20948_get_xg_offs_usr_reg(icm20948_t const *icm20948,
                                            icm20948_xg_offs_usr_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_2,
                         ICM20948_REG_ADDRESS_XG_OFFS_USRH, data, sizeof(data));

  reg->xg_offs_usr = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

  return err;
}

icm20948_err_t
icm20948_set_xg_offs_usr_reg(icm20948_t const *icm20948,
                             icm20948_xg_offs_usr_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  data[0] = (uint8_t)((reg->xg_offs_usr >> 8) & 0xFF);
  data[1] = (uint8_t)(reg->xg_offs_usr & 0xFF);

  return icm20948_bank_write(icm20948, ICM20948_BANK_2,
                             ICM20948_REG_ADDRESS_XG_OFFS_USRH, data,
                             sizeof(data));
}

icm20948_err_t icm20948_get_yg_offs_usr_reg(icm20948_t const *icm20948,
                                            icm20948_yg_offs_usr_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_2,
                         ICM20948_REG_ADDRESS_YG_OFFS_USRH, data, sizeof(data));

  reg->yg_offs_usr = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

  return err;
}

icm20948_err_t
icm20948_set_yg_offs_usr_reg(icm20948_t const *icm20948,
                             icm20948_yg_offs_usr_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  data[0] = (uint8_t)((reg->yg_offs_usr >> 8) & 0xFF);
  data[1] = (uint8_t)(reg->yg_offs_usr & 0xFF);

  return icm20948_bank_write(icm20948, ICM20948_BANK_2,
                             ICM20948_REG_ADDRESS_YG_OFFS_USRH, data,
                             sizeof(data));
}

icm20948_err_t icm20948_get_zg_offs_usr_reg(icm20948_t const *icm20948,
                                            icm20948_zg_offs_usr_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_2,
                         ICM20948_REG_ADDRESS_ZG_OFFS_USRH, data, sizeof(data));

  reg->zg_offs_usr = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

  return err;
}

icm20948_err_t
icm20948_set_zg_offs_usr_reg(icm20948_t const *icm20948,
                             icm20948_zg_offs_usr_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  data[0] = (uint8_t)((reg->zg_offs_usr >> 8) & 0xFF);
  data[1] = (uint8_t)(reg->zg_offs_usr & 0xFF);

  return icm20948_bank_write(icm20948, ICM20948_BANK_2,
                             ICM20948_REG_ADDRESS_ZG_OFFS_USRH, data,
                             sizeof(data));
}

icm20948_err_t icm20948_get_odr_align_en_reg(icm20948_t const *icm20948,
                                             icm20948_odr_align_en_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_ODR_ALIGN_EN,
                                          &data, sizeof(data));

  reg->odr_align_en = data & 0x01U;

  return err;
}

icm20948_err_t
icm20948_set_odr_align_en_reg(icm20948_t const *icm20948,
                              icm20948_odr_align_en_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_ODR_ALIGN_EN,
                                          &data, sizeof(data));

  data &= ~0x01U;

  data |= reg->odr_align_en & 0x01U;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_2,
                             ICM20948_REG_ADDRESS_ODR_ALIGN_EN, &data,
                             sizeof(data));

  return err;
}

icm20948_err_t
icm20948_get_accel_smplrt_div_reg(icm20948_t const *icm20948,
                                  icm20948_accel_smplrt_div_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err = icm20948_bank_read(
      icm20948, ICM20948_BANK_2, ICM20948_REG_ADDRESS_ACCEL_SMPLRT_DIV_1, data,
      sizeof(data));

  reg->accel_smplrt_div =
      (uint16_t)(((data[0] & 0x0FU) << 8U) | (data[1] & 0xFFU));

  return err;
}

icm20948_err_t
icm20948_set_accel_smplrt_div_reg(icm20948_t const *icm20948,
                                  icm20948_accel_smplrt_div_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data[2] = {};

  icm20948_err_t err = icm20948_bank_read(
      icm20948, ICM20948_BANK_2, ICM20948_REG_ADDRESS_ACCEL_SMPLRT_DIV_1, data,
      sizeof(data));

  data[0] &= ~0x0FU;
  data[1] &= ~0xFFU;

  data[0] |= (reg->accel_smplrt_div >> 8U) & 0x0FU;
  data[1] |= reg->accel_smplrt_div & 0xFFU;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_2,
                             ICM20948_REG_ADDRESS_ACCEL_SMPLRT_DIV_1, data,
                             sizeof(data));

  return err;
}

icm20948_err_t
icm20948_get_accel_intel_ctrl_reg(icm20948_t const *icm20948,
                                  icm20948_accel_intel_ctrl_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_ACCEL_INTEL_CTRL,
                                          &data, sizeof(data));

  reg->accel_intel_en = (data >> 1U) & 0x01U;
  reg->accel_intel_mode_int = data & 0x01U;

  return err;
}

icm20948_err_t
icm20948_set_accel_intel_ctrl_reg(icm20948_t const *icm20948,
                                  icm20948_accel_intel_ctrl_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_ACCEL_INTEL_CTRL,
                                          &data, sizeof(data));

  data &= ~((0x01U << 1U) | 0x01U);

  data |= (reg->accel_intel_en & 0x01U) << 1U;
  data |= reg->accel_intel_mode_int & 0x01U;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_2,
                             ICM20948_REG_ADDRESS_ACCEL_INTEL_CTRL, &data,
                             sizeof(data));

  return err;
}

icm20948_err_t
icm20948_get_accel_wom_thr_reg(icm20948_t const *icm20948,
                               icm20948_accel_wom_thr_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_ACCEL_WOM_THR,
                                          &data, sizeof(data));

  reg->wom_threshold = data & 0xFFU;

  return err;
}

icm20948_err_t
icm20948_set_accel_wom_thr_reg(icm20948_t const *icm20948,
                               icm20948_accel_wom_thr_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data = reg->wom_threshold & 0xFFU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_2,
                             ICM20948_REG_ADDRESS_ACCEL_WOM_THR, &data,
                             sizeof(data));
}

icm20948_err_t
icm20948_get_accel_config_1_reg(icm20948_t const *icm20948,
                                icm20948_accel_config_1_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_ACCEL_CONFIG_1,
                                          &data, sizeof(data));

  reg->accel_dlpfcfg = (data >> 3U) & 0x07U;
  reg->accel_fs_sel = (data >> 1U) & 0x03U;
  reg->accel_fchoice = data & 0x01U;

  return err;
}

icm20948_err_t
icm20948_set_accel_config_1_reg(icm20948_t const *icm20948,
                                icm20948_accel_config_1_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_ACCEL_CONFIG_1,
                                          &data, sizeof(data));

  data &= ~((0x07U << 3U) | (0x03U << 1U) | 0x01U);

  data |= (reg->accel_dlpfcfg & 0x07U) << 3U;
  data |= (reg->accel_fs_sel & 0x03U) << 1U;
  data |= reg->accel_fchoice & 0x01U;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_2,
                             ICM20948_REG_ADDRESS_ACCEL_CONFIG_1, &data,
                             sizeof(data));
  return err;
}

icm20948_err_t
icm20948_get_accel_config_2_reg(icm20948_t const *icm20948,
                                icm20948_accel_config_2_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_ACCEL_CONFIG_2,
                                          &data, sizeof(data));

  reg->ax_st_en_reg = (data >> 4U) & 0x01U;
  reg->ay_st_en_reg = (data >> 3U) & 0x01U;
  reg->az_st_en_reg = (data >> 2U) & 0x01U;
  reg->dec3_cfg = data & 0x03U;

  return err;
}

icm20948_err_t
icm20948_set_accel_config_2_reg(icm20948_t const *icm20948,
                                icm20948_accel_config_2_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_ACCEL_CONFIG_2,
                                          &data, sizeof(data));

  data &= ~((0x01U << 4U) | (0x01U << 3U) | (0x01U << 2U) | 0x03U);

  data |= (reg->ax_st_en_reg & 0x01U) << 4U;
  data |= (reg->ay_st_en_reg & 0x01U) << 3U;
  data |= (reg->az_st_en_reg & 0x01U) << 2U;
  data |= reg->dec3_cfg & 0x03U;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_2,
                             ICM20948_REG_ADDRESS_ACCEL_CONFIG_2, &data,
                             sizeof(data));
  return err;
}

icm20948_err_t icm20948_get_fsync_config_reg(icm20948_t const *icm20948,
                                             icm20948_fsync_config_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_FSYNC_CONFIG,
                                          &data, sizeof(data));

  reg->delay_time_en = (data >> 7U) & 0x01U;
  reg->wof_deglitch_en = (data >> 5U) & 0x01U;
  reg->wof_edge_int = (data >> 4U) & 0x01U;
  reg->ext_sync_set = data & 0x0FU;

  return err;
}

icm20948_err_t
icm20948_set_fsync_config_reg(icm20948_t const *icm20948,
                              icm20948_fsync_config_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_FSYNC_CONFIG,
                                          &data, sizeof(data));

  data &= ~((0x01U << 7U) | (0x01U << 5U) | (0x01U << 4U) | 0x0FU);

  data |= (reg->delay_time_en & 0x01U) << 7U;
  data |= (reg->wof_deglitch_en & 0x01U) << 5U;
  data |= (reg->wof_edge_int & 0x01U) << 4U;
  data |= reg->ext_sync_set & 0x0FU;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_2,
                             ICM20948_REG_ADDRESS_FSYNC_CONFIG, &data,
                             sizeof(data));

  return err;
}

icm20948_err_t icm20948_get_temp_config_reg(icm20948_t const *icm20948,
                                            icm20948_temp_config_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_2,
                         ICM20948_REG_ADDRESS_TEMP_CONFIG, &data, sizeof(data));

  reg->temp_dlpfcfg = data & 0x07U;

  return err;
}

icm20948_err_t
icm20948_set_temp_config_reg(icm20948_t const *icm20948,
                             icm20948_temp_config_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_2,
                         ICM20948_REG_ADDRESS_TEMP_CONFIG, &data, sizeof(data));

  data &= ~0x07U;

  data |= reg->temp_dlpfcfg & 0x07U;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_2,
                             ICM20948_REG_ADDRESS_TEMP_CONFIG, &data,
                             sizeof(data));
  return err;
}

icm20948_err_t icm20948_get_mod_ctrl_usr_reg(icm20948_t const *icm20948,
                                             icm20948_mod_ctrl_usr_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_MOD_CTRL_USR,
                                          &data, sizeof(data));

  reg->reg_lp_dmp_en = data & 0x01U;

  return err;
}

icm20948_err_t
icm20948_set_mod_ctrl_usr_reg(icm20948_t const *icm20948,
                              icm20948_mod_ctrl_usr_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_2,
                                          ICM20948_REG_ADDRESS_MOD_CTRL_USR,
                                          &data, sizeof(data));

  data &= ~0x01U;

  data |= reg->reg_lp_dmp_en & 0x01U;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_2,
                             ICM20948_REG_ADDRESS_MOD_CTRL_USR, &data,
                             sizeof(data));

  return err;
}

icm20948_err_t
icm20948_get_i2c_mst_odr_config_reg(icm20948_t const *icm20948,
                                    icm20948_i2c_mst_odr_config_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(
      icm20948, ICM20948_BANK_3, ICM20948_REG_ADDRESS_I2C_MST_ODR_CONFIG, &data,
      sizeof(data));

  reg->i2c_mst_odr_config = data & 0x0FU;

  return err;
}

icm20948_err_t icm20948_set_i2c_mst_odr_config_reg(
    icm20948_t const *icm20948, icm20948_i2c_mst_odr_config_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(
      icm20948, ICM20948_BANK_3, ICM20948_REG_ADDRESS_I2C_MST_ODR_CONFIG, &data,
      sizeof(data));

  data &= ~0x0FU;

  data |= reg->i2c_mst_odr_config & 0x0FU;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_3,
                             ICM20948_REG_ADDRESS_I2C_MST_ODR_CONFIG, &data,
                             sizeof(data));

  return err;
}

icm20948_err_t icm20948_get_i2c_mst_ctrl_reg(icm20948_t const *icm20948,
                                             icm20948_i2c_mst_ctrl_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_3,
                                          ICM20948_REG_ADDRESS_I2C_MST_CTRL,
                                          &data, sizeof(data));

  reg->mult_mst_en = (data >> 7U) & 0x01U;
  reg->i2c_mst_p_nsr = (data >> 4U) & 0x01U;
  reg->i2c_mst_clk = data & 0x0FU;

  return err;
}

icm20948_err_t
icm20948_set_i2c_mst_ctrl_reg(icm20948_t const *icm20948,
                              icm20948_i2c_mst_ctrl_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_3,
                                          ICM20948_REG_ADDRESS_I2C_MST_CTRL,
                                          &data, sizeof(data));

  data &= ~((0x01U << 7U) | (0x01U << 4U) | 0x0FU);

  data |= (reg->mult_mst_en & 0x01U) << 7U;
  data |= (reg->i2c_mst_p_nsr & 0x01U) << 4U;
  data |= reg->i2c_mst_clk & 0x0FU;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_3,
                             ICM20948_REG_ADDRESS_I2C_MST_CTRL, &data,
                             sizeof(data));

  return err;
}

icm20948_err_t
icm20948_get_i2c_mst_delay_ctrl_reg(icm20948_t const *icm20948,
                                    icm20948_i2c_mst_delay_ctrl_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(
      icm20948, ICM20948_BANK_3, ICM20948_REG_ADDRESS_I2C_MST_DELAY_CTRL, &data,
      sizeof(data));

  reg->delay_es_shadow = (data >> 7U) & 0x01U;
  reg->i2c_slv4_delay_en = (data >> 4U) & 0x01U;
  reg->i2c_slv3_delay_en = (data >> 3U) & 0x01U;
  reg->i2c_slv2_delay_en = (data >> 2U) & 0x01U;
  reg->i2c_slv1_delay_en = (data >> 1U) & 0x01U;
  reg->i2c_slv0_delay_en = data & 0x01U;

  return err;
}

icm20948_err_t icm20948_set_i2c_mst_delay_ctrl_reg(
    icm20948_t const *icm20948, icm20948_i2c_mst_delay_ctrl_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(
      icm20948, ICM20948_BANK_3, ICM20948_REG_ADDRESS_I2C_MST_DELAY_CTRL, &data,
      sizeof(data));

  data &= ~((0x01U << 7U) | (0x01U << 4U) | (0x01U << 3U) | (0x01U << 2U) |
            (0x01U << 1U) | 0x01U);

  data |= (reg->delay_es_shadow & 0x01U) << 7U;
  data |= (reg->i2c_slv4_delay_en & 0x01U) << 4U;
  data |= (reg->i2c_slv3_delay_en & 0x01U) << 3U;
  data |= (reg->i2c_slv2_delay_en & 0x01U) << 2U;
  data |= (reg->i2c_slv1_delay_en & 0x01U) << 1U;
  data |= reg->i2c_slv0_delay_en & 0x01U;

  err |= icm20948_bank_write(icm20948, ICM20948_BANK_3,
                             ICM20948_REG_ADDRESS_I2C_MST_DELAY_CTRL, &data,
                             sizeof(data));
  return err;
}

icm20948_err_t icm20948_get_i2c_slv_addr_reg(icm20948_t const *icm20948,
                                             icm20948_slave_num_t slave_num,
                                             icm20948_i2c_slv_addr_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_3,
                                          ICM20948_REG_ADDRESS_I2C_SLV0_ADDR +
                                              (slave_num & 0x03U),
                                          &data, sizeof(data));

  reg->i2c_slv_rw = (data >> 7U) & 0x01U;
  reg->i2c_id = data & 0x3FU;

  return err;
}

icm20948_err_t
icm20948_set_i2c_slv_addr_reg(icm20948_t const *icm20948,
                              icm20948_slave_num_t slave_num,
                              icm20948_i2c_slv_addr_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data |= (reg->i2c_slv_rw & 0x01U) << 7U;
  data |= reg->i2c_id & 0x3FU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_3,
                             ICM20948_REG_ADDRESS_I2C_SLV0_ADDR +
                                 (slave_num & 0x03U),
                             &data, sizeof(data));
}

icm20948_err_t icm20948_get_i2c_slv_reg(icm20948_t const *icm20948,
                                        icm20948_slave_num_t slave_num,
                                        icm20948_i2c_slv_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_3,
                                          ICM20948_REG_ADDRESS_I2C_SLV0_REG +
                                              (slave_num & 0x03U),
                                          &data, sizeof(data));

  reg->i2c_slv_reg = data & 0xFFU;

  return err;
}

icm20948_err_t icm20948_set_i2c_slv_reg(icm20948_t const *icm20948,
                                        icm20948_slave_num_t slave_num,
                                        icm20948_i2c_slv_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data = reg->i2c_slv_reg & 0xFFU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_3,
                             ICM20948_REG_ADDRESS_I2C_SLV0_REG +
                                 (slave_num & 0x03U),
                             &data, sizeof(data));
}

icm20948_err_t icm20948_get_i2c_slv_ctrl_reg(icm20948_t const *icm20948,
                                             icm20948_slave_num_t slave_num,
                                             icm20948_i2c_slv_ctrl_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_3,
                                          ICM20948_REG_ADDRESS_I2C_SLV0_CTRL +
                                              (slave_num & 0x03U),
                                          &data, sizeof(data));

  reg->i2c_slv_en = (data >> 7U) & 0x01U;
  reg->i2c_slv_byte_sw = (data >> 6U) & 0x01U;
  reg->i2c_slv_reg_dis = (data >> 5U) & 0x01U;
  reg->i2c_slv_grp = (data >> 4U) & 0x01U;
  reg->i2c_slv_leng = data & 0x0FU;

  return err;
}

icm20948_err_t
icm20948_set_i2c_slv_ctrl_reg(icm20948_t const *icm20948,
                              icm20948_slave_num_t slave_num,
                              icm20948_i2c_slv_ctrl_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data |= (reg->i2c_slv_en & 0x01U) << 7U;
  data |= (reg->i2c_slv_byte_sw & 0x01U) << 6U;
  data |= (reg->i2c_slv_reg_dis & 0x01U) << 5U;
  data |= (reg->i2c_slv_grp & 0x01U) << 4U;
  data |= reg->i2c_slv_leng & 0x0FU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_3,
                             ICM20948_REG_ADDRESS_I2C_SLV0_CTRL +
                                 (slave_num & 0x03U),
                             &data, sizeof(data));
}

icm20948_err_t icm20948_get_i2c_slv_do_reg(icm20948_t const *icm20948,
                                           icm20948_slave_num_t slave_num,
                                           icm20948_i2c_slv_do_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_3,
                         ICM20948_REG_ADDRESS_I2C_SLV0_DO + (slave_num & 0x03U),
                         &data, sizeof(data));

  reg->i2c_slv_do = data & 0xFFU;

  return err;
}

icm20948_err_t
icm20948_set_i2c_slv_do_reg(icm20948_t const *icm20948,
                            icm20948_slave_num_t slave_num,
                            icm20948_i2c_slv_do_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data = reg->i2c_slv_do & 0xFFU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_3,
                             ICM20948_REG_ADDRESS_I2C_SLV0_DO +
                                 (slave_num & 0x03U),
                             &data, sizeof(data));
}

icm20948_err_t
icm20948_get_i2c_slv4_addr_reg(icm20948_t const *icm20948,
                               icm20948_i2c_slv4_addr_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_3,
                                          ICM20948_REG_ADDRESS_I2C_SLV4_ADDR,
                                          &data, sizeof(data));

  reg->i2c_slv4_rw = (data >> 7U) & 0x01U;
  reg->i2c_id_4 = data & 0x7FU;

  return err;
}

icm20948_err_t
icm20948_set_i2c_slv4_addr_reg(icm20948_t const *icm20948,
                               icm20948_i2c_slv4_addr_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data |= (reg->i2c_slv4_rw & 0x01U) << 7U;
  data |= reg->i2c_id_4 & 0x7FU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_3,
                             ICM20948_REG_ADDRESS_I2C_SLV4_ADDR, &data,
                             sizeof(data));
}

icm20948_err_t icm20948_get_i2c_slv4_reg(icm20948_t const *icm20948,
                                         icm20948_i2c_slv4_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_3,
                                          ICM20948_REG_ADDRESS_I2C_SLV4_REG,
                                          &data, sizeof(data));

  reg->i2c_slv4_reg = data & 0xFFU;

  return err;
}

icm20948_err_t icm20948_set_i2c_slv4_reg(icm20948_t const *icm20948,
                                         icm20948_i2c_slv4_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data = reg->i2c_slv4_reg & 0xFFU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_3,
                             ICM20948_REG_ADDRESS_I2C_SLV4_REG, &data,
                             sizeof(data));
}

icm20948_err_t
icm20948_get_i2c_slv4_ctrl_reg(icm20948_t const *icm20948,
                               icm20948_i2c_slv4_ctrl_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err = icm20948_bank_read(icm20948, ICM20948_BANK_3,
                                          ICM20948_REG_ADDRESS_I2C_SLV4_CTRL,
                                          &data, sizeof(data));

  reg->i2c_slv4_en = (data >> 7U) & 0x01U;
  reg->i2c_slv4_int_en = (data >> 6U) & 0x01U;
  reg->i2c_slv4_reg_dis = (data >> 5U) & 0x01U;
  reg->i2c_slv4_dly = data & 0x1FU;

  return err;
}

icm20948_err_t
icm20948_set_i2c_slv4_ctrl_reg(icm20948_t const *icm20948,
                               icm20948_i2c_slv4_ctrl_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data |= (reg->i2c_slv4_en & 0x01U) << 7U;
  data |= (reg->i2c_slv4_int_en & 0x01U) << 6U;
  data |= (reg->i2c_slv4_reg_dis & 0x01U) << 5U;
  data |= reg->i2c_slv4_dly & 0x1FU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_3,
                             ICM20948_REG_ADDRESS_I2C_SLV4_CTRL, &data,
                             sizeof(data));
}

icm20948_err_t icm20948_get_i2c_slv4_do_reg(icm20948_t const *icm20948,
                                            icm20948_i2c_slv4_do_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_3,
                         ICM20948_REG_ADDRESS_I2C_SLV4_DO, &data, sizeof(data));

  reg->i2c_slv4_do = data & 0xFFU;

  return err;
}

icm20948_err_t
icm20948_set_i2c_slv4_do_reg(icm20948_t const *icm20948,
                             icm20948_i2c_slv4_do_reg_t const *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  data = reg->i2c_slv4_do & 0xFFU;

  return icm20948_bank_write(icm20948, ICM20948_BANK_3,
                             ICM20948_REG_ADDRESS_I2C_SLV4_DO, &data,
                             sizeof(data));
}

icm20948_err_t icm20948_get_i2c_slv4_di_reg(icm20948_t const *icm20948,
                                            icm20948_i2c_slv4_di_reg_t *reg) {
  assert(icm20948 && reg);

  uint8_t data = {};

  icm20948_err_t err =
      icm20948_bank_read(icm20948, ICM20948_BANK_3,
                         ICM20948_REG_ADDRESS_I2C_SLV4_DI, &data, sizeof(data));

  reg->i2c_slv4_di = data & 0xFFU;

  return err;
}
