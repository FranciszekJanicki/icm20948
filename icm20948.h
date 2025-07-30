#ifndef ICM20948_ICM20948_H
#define ICM20948_ICM20948_H

#include "icm20948_config.h"
#include "icm20948_registers.h"

typedef struct {
  icm20948_config_t config;
  icm20948_interface_t interface;
} icm20948_t;

icm20948_err_t icm20948_initialize(icm20948_t *icm20948,
                                   icm20948_config_t const *config,
                                   icm20948_interface_t const *interface);
icm20948_err_t icm20948_deinitialize(icm20948_t *icm20948);

icm20948_err_t icm20948_get_accel_data_x_scaled(icm20948_t const *icm20948,
                                                float32_t *scaled);
icm20948_err_t icm20948_get_accel_data_y_scaled(icm20948_t const *icm20948,
                                                float32_t *scaled);
icm20948_err_t icm20948_get_accel_data_z_scaled(icm20948_t const *icm20948,
                                                float32_t *scaled);
icm20948_err_t icm20948_get_accel_data_scaled(icm20948_t const *icm20948,
                                              vec3_float32_t *scaled);

icm20948_err_t icm20948_get_accel_data_x_raw(icm20948_t const *icm20948,
                                             int16_t *raw);
icm20948_err_t icm20948_get_accel_data_y_raw(icm20948_t const *icm20948,
                                             int16_t *raw);
icm20948_err_t icm20948_get_accel_data_z_raw(icm20948_t const *icm20948,
                                             int16_t *raw);
icm20948_err_t icm20948_get_accel_data_raw(icm20948_t const *icm20948,
                                           vec3_int16_t *raw);

icm20948_err_t icm20948_get_gyro_data_x_scaled(icm20948_t const *icm20948,
                                               float32_t *scaled);
icm20948_err_t icm20948_get_gyro_data_y_scaled(icm20948_t const *icm20948,
                                               float32_t *scaled);
icm20948_err_t icm20948_get_gyro_data_z_scaled(icm20948_t const *icm20948,
                                               float32_t *scaled);
icm20948_err_t icm20948_get_gyro_data_scaled(icm20948_t const *icm20948,
                                             vec3_float32_t *scaled);

icm20948_err_t icm20948_get_gyro_data_x_raw(icm20948_t const *icm20948,
                                            int16_t *raw);
icm20948_err_t icm20948_get_gyro_data_y_raw(icm20948_t const *icm20948,
                                            int16_t *raw);
icm20948_err_t icm20948_get_gyro_data_z_raw(icm20948_t const *icm20948,
                                            int16_t *raw);
icm20948_err_t icm20948_get_gyro_data_raw(icm20948_t const *icm20948,
                                          vec3_int16_t *raw);

icm20948_err_t icm20948_get_temp_data_scaled(icm20948_t const *icm20948,
                                             float32_t *scaled);

icm20948_err_t icm20948_get_temp_data_raw(icm20948_t const *icm20948,
                                          int16_t *raw);

icm20948_err_t icm20948_ext_slv_read(icm20948_t const *icm20948,
                                     icm20948_slave_num_t slave_num,
                                     uint8_t address, uint8_t *data,
                                     uint8_t data_size);

icm20948_err_t icm20948_ext_slv_write(icm20948_t const *icm20948,
                                      icm20948_slave_num_t slave_num,
                                      uint8_t address, uint8_t const *data,
                                      uint8_t data_size);

icm20948_err_t icm20948_fifo_read(icm20948_t const *icm20948, uint8_t *data,
                                  uint8_t data_size);

icm20948_err_t icm20948_fifo_write(icm20948_t const *icm20948,
                                   uint8_t const *data, uint8_t data_size);

icm20948_err_t icm20948_bank_select(icm20948_t const *icm20948,
                                    icm20948_bank_t bank);

icm20948_err_t icm20948_get_reg_bank_sel_reg(icm20948_t const *icm20948,
                                             icm20948_bank_sel_reg_t *reg);
icm20948_err_t
icm20948_set_reg_bank_sel_reg(icm20948_t const *icm20948,
                              icm20948_bank_sel_reg_t const *reg);

icm20948_err_t icm20948_get_who_am_i_reg(icm20948_t const *icm20948,
                                         icm20948_who_am_i_reg_t *reg);

icm20948_err_t icm20948_get_user_ctrl_reg(icm20948_t const *icm20948,
                                          icm20948_user_ctrl_reg_t *reg);
icm20948_err_t icm20948_set_user_ctrl_reg(icm20948_t const *icm20948,
                                          icm20948_user_ctrl_reg_t const *reg);

icm20948_err_t icm20948_get_lp_config_reg(icm20948_t const *icm20948,
                                          icm20948_lp_config_reg_t *reg);
icm20948_err_t icm20948_set_lp_config_reg(icm20948_t const *icm20948,
                                          icm20948_lp_config_reg_t const *reg);

icm20948_err_t icm20948_get_pwr_mgmt_1_reg(icm20948_t const *icm20948,
                                           icm20948_pwr_mgmt_1_reg_t *reg);
icm20948_err_t
icm20948_set_pwr_mgmt_1_reg(icm20948_t const *icm20948,
                            icm20948_pwr_mgmt_1_reg_t const *reg);

icm20948_err_t icm20948_get_pwr_mgmt_2_reg(icm20948_t const *icm20948,
                                           icm20948_pwr_mgmt_2_reg_t *reg);
icm20948_err_t
icm20948_set_pwr_mgmt_2_reg(icm20948_t const *icm20948,
                            icm20948_pwr_mgmt_2_reg_t const *reg);

icm20948_err_t icm20948_get_int_pin_cfg_reg(icm20948_t const *icm20948,
                                            icm20948_int_pin_cfg_reg_t *reg);
icm20948_err_t
icm20948_set_int_pin_cfg_reg(icm20948_t const *icm20948,
                             icm20948_int_pin_cfg_reg_t const *reg);

icm20948_err_t icm20948_get_int_enable_reg(icm20948_t const *icm20948,
                                           icm20948_int_enable_reg_t *reg);
icm20948_err_t
icm20948_set_int_enable_reg(icm20948_t const *icm20948,
                            icm20948_int_enable_reg_t const *reg);

icm20948_err_t icm20948_get_int_enable_1_reg(icm20948_t const *icm20948,
                                             icm20948_int_enable_1_reg_t *reg);
icm20948_err_t
icm20948_set_int_enable_1_reg(icm20948_t const *icm20948,
                              icm20948_int_enable_1_reg_t const *reg);

icm20948_err_t icm20948_get_int_enable_2_reg(icm20948_t const *icm20948,
                                             icm20948_int_enable_2_reg_t *reg);
icm20948_err_t
icm20948_set_int_enable_2_reg(icm20948_t const *icm20948,
                              icm20948_int_enable_2_reg_t const *reg);

icm20948_err_t icm20948_get_int_enable_3_reg(icm20948_t const *icm20948,
                                             icm20948_int_enable_3_reg_t *reg);
icm20948_err_t
icm20948_set_int_enable_3_reg(icm20948_t const *icm20948,
                              icm20948_int_enable_3_reg_t const *reg);

icm20948_err_t
icm20948_get_i2c_mst_status_reg(icm20948_t const *icm20948,
                                icm20948_i2c_mst_status_reg_t *reg);

icm20948_err_t icm20948_get_int_status_reg(icm20948_t const *icm20948,
                                           icm20948_int_status_reg_t *reg);

icm20948_err_t icm20948_get_int_status_1_reg(icm20948_t const *icm20948,
                                             icm20948_int_status_1_reg_t *reg);

icm20948_err_t icm20948_get_int_status_2_reg(icm20948_t const *icm20948,
                                             icm20948_int_status_2_reg_t *reg);

icm20948_err_t icm20948_get_int_status_3_reg(icm20948_t const *icm20948,
                                             icm20948_int_status_3_reg_t *reg);

icm20948_err_t icm20948_get_delay_time_reg(icm20948_t const *icm20948,
                                           icm20948_delay_time_reg_t *reg);

icm20948_err_t icm20948_get_accel_xout_reg(icm20948_t const *icm20948,
                                           icm20948_accel_xout_reg_t *reg);

icm20948_err_t icm20948_get_accel_yout_reg(icm20948_t const *icm20948,
                                           icm20948_accel_yout_reg_t *reg);

icm20948_err_t icm20948_get_accel_zout_reg(icm20948_t const *icm20948,
                                           icm20948_accel_zout_reg_t *reg);

icm20948_err_t icm20948_get_accel_out_reg(icm20948_t const *icm20948,
                                          icm20948_accel_out_reg_t *reg);

icm20948_err_t icm20948_get_gyro_xout_reg(icm20948_t const *icm20948,
                                          icm20948_gyro_xout_reg_t *reg);

icm20948_err_t icm20948_get_gyro_yout_reg(icm20948_t const *icm20948,
                                          icm20948_gyro_yout_reg_t *reg);

icm20948_err_t icm20948_get_gyro_zout_reg(icm20948_t const *icm20948,
                                          icm20948_gyro_zout_reg_t *reg);

icm20948_err_t icm20948_get_gyro_out_reg(icm20948_t const *icm20948,
                                         icm20948_gyro_out_reg_t *reg);

icm20948_err_t icm20948_get_temp_out_reg(icm20948_t const *icm20948,
                                         icm20948_temp_out_reg_t *reg);

icm20948_err_t
icm20948_get_ext_slv_sens_data_reg(icm20948_t const *icm20948, uint8_t reg_num,
                                   icm20948_ext_slv_sens_data_reg_t *reg);

icm20948_err_t icm20948_get_fifo_en_1_reg(icm20948_t const *icm20948,
                                          icm20948_fifo_en_1_reg_t *reg);
icm20948_err_t icm20948_set_fifo_en_1_reg(icm20948_t const *icm20948,
                                          icm20948_fifo_en_1_reg_t const *reg);

icm20948_err_t icm20948_get_fifo_en_2_reg(icm20948_t const *icm20948,
                                          icm20948_fifo_en_2_reg_t *reg);
icm20948_err_t icm20948_set_fifo_en_2_reg(icm20948_t const *icm20948,
                                          icm20948_fifo_en_2_reg_t const *reg);

icm20948_err_t icm20948_get_fifo_rst_reg(icm20948_t const *icm20948,
                                         icm20948_fifo_rst_reg_t *reg);
icm20948_err_t icm20948_set_fifo_rst_reg(icm20948_t const *icm20948,
                                         icm20948_fifo_rst_reg_t const *reg);

icm20948_err_t icm20948_get_fifo_mode_reg(icm20948_t const *icm20948,
                                          icm20948_fifo_mode_reg_t *reg);
icm20948_err_t icm20948_set_fifo_mode_reg(icm20948_t const *icm20948,
                                          icm20948_fifo_mode_reg_t const *reg);

icm20948_err_t icm20948_get_fifo_cnt_reg(icm20948_t const *icm20948,
                                         icm20948_fifo_cnt_reg_t *reg);

icm20948_err_t icm20948_get_fifo_r_w_reg(icm20948_t const *icm20948,
                                         icm20948_fifo_r_w_reg_t *reg);
icm20948_err_t icm20948_set_fifo_r_w_reg(icm20948_t const *icm20948,
                                         icm20948_fifo_r_w_reg_t const *reg);

icm20948_err_t
icm20948_get_data_rdy_status(icm20948_t const *icm20948,
                             icm20948_data_rdy_status_reg_t *reg);

icm20948_err_t icm20948_get_fifo_cfg_reg(icm20948_t const *icm20948,
                                         icm20948_fifo_cfg_reg_t *reg);
icm20948_err_t icm20948_set_fifo_cfg_reg(icm20948_t const *icm20948,
                                         icm20948_fifo_cfg_reg_t const *reg);

icm20948_err_t
icm20948_get_self_test_x_gyro_reg(icm20948_t const *icm20948,
                                  icm20948_selt_test_x_gyro_reg_t *reg);
icm20948_err_t
icm20948_set_self_test_x_gyro_reg(icm20948_t const *icm20948,
                                  icm20948_selt_test_x_gyro_reg_t const *reg);

icm20948_err_t
icm20948_get_self_test_y_gyro_reg(icm20948_t const *icm20948,
                                  icm20948_self_test_y_gyro_reg_t *reg);
icm20948_err_t
icm20948_set_self_test_y_gyro_reg(icm20948_t const *icm20948,
                                  icm20948_self_test_y_gyro_reg_t const *reg);

icm20948_err_t
icm20948_get_self_test_z_gyro_reg(icm20948_t const *icm20948,
                                  icm20948_self_test_z_gyro_reg_t *reg);
icm20948_err_t
icm20948_set_self_test_z_gyro_reg(icm20948_t const *icm20948,
                                  icm20948_self_test_z_gyro_reg_t const *reg);

icm20948_err_t
icm20948_get_self_test_x_accel_reg(icm20948_t const *icm20948,
                                   icm20948_self_test_x_accel_reg_t *reg);
icm20948_err_t
icm20948_set_self_test_x_accel_reg(icm20948_t const *icm20948,
                                   icm20948_self_test_x_accel_reg_t const *reg);

icm20948_err_t
icm20948_get_self_test_y_accel_reg(icm20948_t const *icm20948,
                                   icm20948_self_test_y_gyro_reg_t *reg);
icm20948_err_t
icm20948_set_self_test_y_accel_reg(icm20948_t const *icm20948,
                                   icm20948_self_test_y_gyro_reg_t const *reg);

icm20948_err_t
icm20948_get_self_test_z_accel_reg(icm20948_t const *icm20948,
                                   icm20948_self_test_z_gyro_reg_t *reg);
icm20948_err_t
icm20948_set_self_test_z_accel_reg(icm20948_t const *icm20948,
                                   icm20948_self_test_z_gyro_reg_t const *reg);

icm20948_err_t icm20948_get_xa_offs_reg(icm20948_t const *icm20948,
                                        icm20948_xa_offs_reg_t *reg);
icm20948_err_t icm20948_set_xa_offs_reg(icm20948_t const *icm20948,
                                        icm20948_xa_offs_reg_t const *reg);

icm20948_err_t icm20948_get_ya_offs_reg(icm20948_t const *icm20948,
                                        icm20948_ya_offs_reg_t *reg);
icm20948_err_t icm20948_set_ya_offs_reg(icm20948_t const *icm20948,
                                        icm20948_ya_offs_reg_t const *reg);

icm20948_err_t icm20948_get_za_offs_reg(icm20948_t const *icm20948,
                                        icm20948_za_offs_reg_t *reg);
icm20948_err_t icm20948_set_za_offs_reg(icm20948_t const *icm20948,
                                        icm20948_za_offs_reg_t const *reg);

icm20948_err_t icm20948_get_tbc_pll_reg(icm20948_t const *icm20948,
                                        icm20948_tbc_pll_reg_t *reg);
icm20948_err_t icm20948_set_tbc_pll_reg(icm20948_t const *icm20948,
                                        icm20948_tbc_pll_reg_t const *reg);

icm20948_err_t
icm20948_get_gyro_smplrt_div_reg(icm20948_t const *icm20948,
                                 icm20948_gyro_smplrt_div_reg_t *reg);
icm20948_err_t
icm20948_set_gyro_smplrt_div_reg(icm20948_t const *icm20948,
                                 icm20948_gyro_smplrt_div_reg_t const *reg);

icm20948_err_t
icm20948_get_gyro_config_1_reg(icm20948_t const *icm20948,
                               icm20948_gyro_config_1_reg_t *reg);
icm20948_err_t
icm20948_set_gyro_config_1_reg(icm20948_t const *icm20948,
                               icm20948_gyro_config_1_reg_t const *reg);

icm20948_err_t
icm20948_get_gyro_config_2_reg(icm20948_t const *icm20948,
                               icm20948_gyro_config_2_reg_t *reg);
icm20948_err_t
icm20948_set_gyro_config_2_reg(icm20948_t const *icm20948,
                               icm20948_gyro_config_2_reg_t const *reg);

icm20948_err_t icm20948_get_xg_offs_usr_reg(icm20948_t const *icm20948,
                                            icm20948_xg_offs_usr_reg_t *reg);
icm20948_err_t
icm20948_set_xg_offs_usr_reg(icm20948_t const *icm20948,
                             icm20948_xg_offs_usr_reg_t const *reg);

icm20948_err_t icm20948_get_yg_offs_usr_reg(icm20948_t const *icm20948,
                                            icm20948_yg_offs_usr_reg_t *reg);
icm20948_err_t
icm20948_set_yg_offs_usr_reg(icm20948_t const *icm20948,
                             icm20948_yg_offs_usr_reg_t const *reg);

icm20948_err_t icm20948_get_zg_offs_usr_reg(icm20948_t const *icm20948,
                                            icm20948_zg_offs_usr_reg_t *reg);
icm20948_err_t
icm20948_set_zg_offs_usr_reg(icm20948_t const *icm20948,
                             icm20948_zg_offs_usr_reg_t const *reg);

icm20948_err_t icm20948_get_odr_align_en_reg(icm20948_t const *icm20948,
                                             icm20948_odr_align_en_reg_t *reg);
icm20948_err_t
icm20948_set_odr_align_en_reg(icm20948_t const *icm20948,
                              icm20948_odr_align_en_reg_t const *reg);

icm20948_err_t
icm20948_get_accel_smplrt_div_reg(icm20948_t const *icm20948,
                                  icm20948_accel_smplrt_div_reg_t *reg);
icm20948_err_t
icm20948_set_accel_smplrt_div_reg(icm20948_t const *icm20948,
                                  icm20948_accel_smplrt_div_reg_t const *reg);

icm20948_err_t
icm20948_get_accel_intel_ctrl_reg(icm20948_t const *icm20948,
                                  icm20948_accel_intel_ctrl_reg_t *reg);
icm20948_err_t
icm20948_set_accel_intel_ctrl_reg(icm20948_t const *icm20948,
                                  icm20948_accel_intel_ctrl_reg_t const *reg);

icm20948_err_t
icm20948_get_accel_wom_thr_reg(icm20948_t const *icm20948,
                               icm20948_accel_wom_thr_reg_t *reg);
icm20948_err_t
icm20948_set_accel_wom_thr_reg(icm20948_t const *icm20948,
                               icm20948_accel_wom_thr_reg_t const *reg);

icm20948_err_t
icm20948_get_accel_config_1_reg(icm20948_t const *icm20948,
                                icm20948_accel_config_1_reg_t *reg);
icm20948_err_t
icm20948_set_accel_config_1_reg(icm20948_t const *icm20948,
                                icm20948_accel_config_1_reg_t const *reg);

icm20948_err_t
icm20948_get_accel_config_2_reg(icm20948_t const *icm20948,
                                icm20948_accel_config_2_reg_t *reg);
icm20948_err_t
icm20948_set_accel_config_2_reg(icm20948_t const *icm20948,
                                icm20948_accel_config_2_reg_t const *reg);

icm20948_err_t icm20948_get_fsync_config_reg(icm20948_t const *icm20948,
                                             icm20948_fsync_config_reg_t *reg);
icm20948_err_t
icm20948_set_fsync_config_reg(icm20948_t const *icm20948,
                              icm20948_fsync_config_reg_t const *reg);

icm20948_err_t icm20948_get_temp_config_reg(icm20948_t const *icm20948,
                                            icm20948_temp_config_reg_t *reg);
icm20948_err_t
icm20948_set_temp_config_reg(icm20948_t const *icm20948,
                             icm20948_temp_config_reg_t const *reg);

icm20948_err_t icm20948_get_mod_ctrl_usr_reg(icm20948_t const *icm20948,
                                             icm20948_mod_ctrl_usr_reg_t *reg);
icm20948_err_t
icm20948_set_mod_ctrl_usr_reg(icm20948_t const *icm20948,
                              icm20948_mod_ctrl_usr_reg_t const *reg);

icm20948_err_t
icm20948_get_i2c_mst_odr_config_reg(icm20948_t const *icm20948,
                                    icm20948_i2c_mst_odr_config_reg_t *reg);
icm20948_err_t icm20948_set_i2c_mst_odr_config_reg(
    icm20948_t const *icm20948, icm20948_i2c_mst_odr_config_reg_t const *reg);

icm20948_err_t icm20948_get_i2c_mst_ctrl_reg(icm20948_t const *icm20948,
                                             icm20948_i2c_mst_ctrl_reg_t *reg);
icm20948_err_t
icm20948_set_i2c_mst_ctrl_reg(icm20948_t const *icm20948,
                              icm20948_i2c_mst_ctrl_reg_t const *reg);

icm20948_err_t
icm20948_get_i2c_mst_delay_ctrl_reg(icm20948_t const *icm20948,
                                    icm20948_i2c_mst_delay_ctrl_reg_t *reg);
icm20948_err_t icm20948_set_i2c_mst_delay_ctrl_reg(
    icm20948_t const *icm20948, icm20948_i2c_mst_delay_ctrl_reg_t const *reg);

icm20948_err_t icm20948_get_i2c_slv_addr_reg(icm20948_t const *icm20948,
                                             icm20948_slave_num_t slave_num,
                                             icm20948_i2c_slv_addr_reg_t *reg);
icm20948_err_t
icm20948_set_i2c_slv_addr_reg(icm20948_t const *icm20948,
                              icm20948_slave_num_t slave_num,
                              icm20948_i2c_slv_addr_reg_t const *reg);

icm20948_err_t icm20948_get_i2c_slv_reg(icm20948_t const *icm20948,
                                        icm20948_slave_num_t slave_num,
                                        icm20948_i2c_slv_reg_t *reg);
icm20948_err_t icm20948_set_i2c_slv_reg(icm20948_t const *icm20948,
                                        icm20948_slave_num_t slave_num,
                                        icm20948_i2c_slv_reg_t const *reg);

icm20948_err_t icm20948_get_i2c_slv_ctrl_reg(icm20948_t const *icm20948,
                                             icm20948_slave_num_t slave_num,
                                             icm20948_i2c_slv_ctrl_reg_t *reg);
icm20948_err_t
icm20948_set_i2c_slv_ctrl_reg(icm20948_t const *icm20948,
                              icm20948_slave_num_t slave_num,
                              icm20948_i2c_slv_ctrl_reg_t const *reg);

icm20948_err_t icm20948_get_i2c_slv_do_reg(icm20948_t const *icm20948,
                                           icm20948_slave_num_t slave_num,
                                           icm20948_i2c_slv_do_reg_t *reg);
icm20948_err_t
icm20948_set_i2c_slv_do_reg(icm20948_t const *icm20948,
                            icm20948_slave_num_t slave_num,
                            icm20948_i2c_slv_do_reg_t const *reg);

icm20948_err_t
icm20948_get_i2c_slv4_addr_reg(icm20948_t const *icm20948,
                               icm20948_i2c_slv4_addr_reg_t *reg);
icm20948_err_t
icm20948_set_i2c_slv4_addr_reg(icm20948_t const *icm20948,
                               icm20948_i2c_slv4_addr_reg_t const *reg);

icm20948_err_t icm20948_get_i2c_slv4_reg(icm20948_t const *icm20948,
                                         icm20948_i2c_slv4_reg_t *reg);
icm20948_err_t icm20948_set_i2c_slv4_reg(icm20948_t const *icm20948,
                                         icm20948_i2c_slv4_reg_t const *reg);

icm20948_err_t
icm20948_get_i2c_slv4_ctrl_reg(icm20948_t const *icm20948,
                               icm20948_i2c_slv4_ctrl_reg_t *reg);
icm20948_err_t
icm20948_set_i2c_slv4_ctrl_reg(icm20948_t const *icm20948,
                               icm20948_i2c_slv4_ctrl_reg_t const *reg);

icm20948_err_t icm20948_get_i2c_slv4_do_reg(icm20948_t const *icm20948,
                                            icm20948_i2c_slv4_do_reg_t *reg);
icm20948_err_t
icm20948_set_i2c_slv4_do_reg(icm20948_t const *icm20948,
                             icm20948_i2c_slv4_do_reg_t const *reg);

icm20948_err_t icm20948_get_i2c_slv4_di_reg(icm20948_t const *icm20948,
                                            icm20948_i2c_slv4_di_reg_t *reg);

#endif // ICM20948_ICM20948_H