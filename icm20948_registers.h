#ifndef ICM20948_ICM20948_REGISTERS_H
#define ICM20948_ICM20948_REGISTERS_H

#include <stdint.h>

typedef struct {
  uint8_t user_bank : 2;
} icm20948_bank_sel_reg_t;

typedef struct {
  uint8_t who_am_i : 8;
} icm20948_who_am_i_reg_t;

typedef struct {
  uint8_t dmp_en : 1;
  uint8_t fifo_en : 1;
  uint8_t i2c_mst_en : 1;
  uint8_t i2c_if_dis : 1;
  uint8_t dmp_rst : 1;
  uint8_t sram_rst : 1;
  uint8_t i2c_mst_rst : 1;
} icm20948_user_ctrl_reg_t;

typedef struct {
  uint8_t i2c_mst_cycle : 1;
  uint8_t accel_cycle : 1;
  uint8_t gyro_cycle : 1;
} icm20948_lp_config_reg_t;

typedef struct {
  uint8_t device_reset : 1;
  uint8_t sleep : 1;
  uint8_t lp_en : 1;
  uint8_t temp_dis : 1;
  uint8_t clksel : 3;
} icm20948_pwr_mgmt_1_reg_t;

typedef struct {
  uint8_t disable_accel : 3;
  uint8_t disable_gyro : 3;
} icm20948_pwr_mgmt_2_reg_t;

typedef struct {
  uint8_t int1_actl : 1;
  uint8_t int1_open : 1;
  uint8_t int1_latch_en : 1;
  uint8_t int_anyrd_2clear : 1;
  uint8_t actl_fsync : 1;
  uint8_t fsync_int_mode_en : 1;
  uint8_t bypass_en : 1;
} icm20948_int_pin_cfg_reg_t;

typedef struct {
  uint8_t reg_wof_en : 1;
  uint8_t wom_int_en : 1;
  uint8_t pll_rdy_en : 1;
  uint8_t dmp_int1_en : 1;
  uint8_t i2c_mst_int_en : 1;
} icm20948_int_enable_reg_t;

typedef struct {
  uint8_t raw_data_0_rdy_en : 1;
} icm20948_int_enable_1_reg_t;

typedef struct {
  uint8_t fifo_overflow_en : 5;
} icm20948_int_enable_2_reg_t;

typedef struct {
  uint8_t fifo_wm_en : 5;
} icm20948_int_enable_3_reg_t;

typedef struct {
  uint8_t pass_through : 1;
  uint8_t i2c_slv4_done : 1;
  uint8_t i2c_lost_arb : 1;
  uint8_t i2c_slv4_nack : 1;
  uint8_t i2c_slv3_nack : 1;
  uint8_t i2c_slv2_nack : 1;
  uint8_t i2c_slv1_nack : 1;
  uint8_t i2c_slv0_nack : 1;
} icm20948_i2c_mst_status_reg_t;

typedef struct {
  uint8_t wom_int : 1;
  uint8_t pll_rdy_int : 1;
  uint8_t dmp_int1 : 1;
  uint8_t i2c_mst_int : 1;
} icm20948_int_status_reg_t;

typedef struct {
  uint8_t raw_data_0_rdy_int : 1;
} icm20948_int_status_1_reg_t;

typedef struct {
  uint8_t fifo_overflow_int : 5;
} icm20948_int_status_2_reg_t;

typedef struct {
  uint8_t fifo_wm_int : 5;
} icm20948_int_status_3_reg_t;

typedef struct {
  uint16_t delay_time : 16;
} icm20948_delay_time_reg_t;

typedef struct {
  int16_t accel_xout : 16;
} icm20948_accel_xout_reg_t;

typedef struct {
  int16_t accel_yout : 16;
} icm20948_accel_yout_reg_t;

typedef struct {
  int16_t accel_zout : 16;
} icm20948_accel_zout_reg_t;

typedef struct {
  int16_t accel_xout : 16;
  int16_t accel_yout : 16;
  int16_t accel_zout : 16;
} icm20948_accel_out_reg_t;

typedef struct {
  int16_t gyro_xout : 16;
} icm20948_gyro_xout_reg_t;

typedef struct {
  int16_t gyro_yout : 16;
} icm20948_gyro_yout_reg_t;

typedef struct {
  int16_t gyro_zout : 16;
} icm20948_gyro_zout_reg_t;

typedef struct {
  int16_t gyro_xout : 16;
  int16_t gyro_yout : 16;
  int16_t gyro_zout : 16;
} icm20948_gyro_out_reg_t;

typedef struct {
  int16_t temp_out : 16;
} icm20948_temp_out_reg_t;

typedef struct {
  uint8_t ext_slv_sens_data : 8;
} icm20948_ext_slv_sens_data_reg_t;

typedef struct {
  uint8_t slv_3_fifo_en : 1;
  uint8_t slv_2_fifo_en : 1;
  uint8_t slv_1_fifo_en : 1;
  uint8_t slv_0_fifo_en : 1;
} icm20948_fifo_en_1_reg_t;

typedef struct {
  uint8_t accel_fifo_en : 1;
  uint8_t gyro_z_fifo_en : 1;
  uint8_t gyro_y_fifo_en : 1;
  uint8_t gyro_x_fifo_en : 1;
  uint8_t temp_fifo_en : 1;
} icm20948_fifo_en_2_reg_t;

typedef struct {
  uint8_t fifo_reset : 5;
} icm20948_fifo_rst_reg_t;

typedef struct {
  uint8_t fifo_mode : 5;
} icm20948_fifo_mode_reg_t;

typedef struct {
  uint16_t fifo_cnt : 16;
} icm20948_fifo_cnt_reg_t;

typedef struct {
  uint8_t fifo_r_w : 8;
} icm20948_fifo_r_w_reg_t;

typedef struct {
  uint8_t wof_status : 1;
  uint8_t raw_data_rdy : 4;
} icm20948_data_rdy_status_reg_t;

typedef struct {
  uint8_t fifo_cfg : 1;
} icm20948_fifo_cfg_reg_t;

typedef struct {
  uint8_t xg_st_data : 8;
} icm20948_selt_test_x_gyro_reg_t;

typedef struct {
  uint8_t yg_st_data : 8;
} icm20948_self_test_y_gyro_reg_t;

typedef struct {
  uint8_t zg_st_data : 8;
} icm20948_self_test_z_gyro_reg_t;

typedef struct {
  uint8_t xa_st_data : 8;
} icm20948_self_test_x_accel_reg_t;

typedef struct {
  uint8_t ya_st_data : 8;
} icm20948_self_test_y_accel_reg_t;

typedef struct {
  uint8_t za_st_data : 8;
} icm20948_self_test_z_accel_reg_t;

typedef struct {
  int16_t xa_offs : 15;
} icm20948_xa_offs_reg_t;

typedef struct {
  int16_t ya_offs : 15;
} icm20948_ya_offs_reg_t;

typedef struct {
  int16_t za_offs : 15;
} icm20948_za_offs_reg_t;

typedef struct {
  uint8_t tbc_pll : 8;
} icm20948_tbc_pll_reg_t;

typedef struct {
  uint8_t gyro_smplrt_div : 8;
} icm20948_gyro_smplrt_div_reg_t;

typedef struct {
  uint8_t gyro_dplfcfg : 3;
  uint8_t gyro_fs_sel : 2;
  uint8_t gyro_fchoice : 1;
} icm20948_gyro_config_1_reg_t;

typedef struct {
  uint8_t xgyro_cten : 1;
  uint8_t ygyro_cten : 1;
  uint8_t zgyro_cten : 1;
  uint8_t gyro_avgcfg : 3;
} icm20948_gyro_config_2_reg_t;

typedef struct {
  int16_t xg_offs_usr : 16;
} icm20948_xg_offs_usr_reg_t;

typedef struct {
  int16_t yg_offs_usr : 16;
} icm20948_yg_offs_usr_reg_t;

typedef struct {
  int16_t zg_offs_usr : 16;
} icm20948_zg_offs_usr_reg_t;

typedef struct {
  uint8_t odr_align_en : 1;
} icm20948_odr_align_en_reg_t;

typedef struct {
  uint16_t accel_smplrt_div : 12;
} icm20948_accel_smplrt_div_reg_t;

typedef struct {
  uint8_t accel_intel_en : 1;
  uint8_t accel_intel_mode_int : 1;
} icm20948_accel_intel_ctrl_reg_t;

typedef struct {
  uint8_t wom_threshold : 8;
} icm20948_accel_wom_thr_reg_t;

typedef struct {
  uint8_t accel_dlpfcfg : 3;
  uint8_t accel_fs_sel : 2;
  uint8_t accel_fchoice : 1;
} icm20948_accel_config_1_reg_t;

typedef struct {
  uint8_t ax_st_en_reg : 1;
  uint8_t ay_st_en_reg : 1;
  uint8_t az_st_en_reg : 1;
  uint8_t dec3_cfg : 2;
} icm20948_accel_config_2_reg_t;

typedef struct {
  uint8_t delay_time_en : 1;
  uint8_t wof_deglitch_en : 1;
  uint8_t wof_edge_int : 1;
  uint8_t ext_sync_set : 4;
} icm20948_fsync_config_reg_t;

typedef struct {
  uint8_t temp_dlpfcfg : 3;
} icm20948_temp_config_reg_t;

typedef struct {
  uint8_t reg_lp_dmp_en : 1;
} icm20948_mod_ctrl_usr_reg_t;

typedef struct {
  uint8_t i2c_mst_odr_config : 4;
} icm20948_i2c_mst_odr_config_reg_t;

typedef struct {
  uint8_t mult_mst_en : 1;
  uint8_t i2c_mst_p_nsr : 1;
  uint8_t i2c_mst_clk : 4;
} icm20948_i2c_mst_ctrl_reg_t;

typedef struct {
  uint8_t delay_es_shadow : 1;
  uint8_t i2c_slv4_delay_en : 1;
  uint8_t i2c_slv3_delay_en : 1;
  uint8_t i2c_slv2_delay_en : 1;
  uint8_t i2c_slv1_delay_en : 1;
  uint8_t i2c_slv0_delay_en : 1;
} icm20948_i2c_mst_delay_ctrl_reg_t;

typedef struct {
  uint8_t i2c_slv_rw : 1;
  uint8_t i2c_id : 7;
} icm20948_i2c_slv_addr_reg_t;

typedef struct {
  uint8_t i2c_slv_reg : 8;
} icm20948_i2c_slv_reg_t;

typedef struct {
  uint8_t i2c_slv_en : 1;
  uint8_t i2c_slv_byte_sw : 1;
  uint8_t i2c_slv_reg_dis : 1;
  uint8_t i2c_slv_grp : 1;
  uint8_t i2c_slv_leng : 4;
} icm20948_i2c_slv_ctrl_reg_t;

typedef struct {
  uint8_t i2c_slv_do : 8;
} icm20948_i2c_slv_do_reg_t;

typedef struct {
  uint8_t i2c_slv4_rw : 1;
  uint8_t i2c_id_4 : 7;
} icm20948_i2c_slv4_addr_reg_t;

typedef struct {
  uint8_t i2c_slv4_reg : 8;
} icm20948_i2c_slv4_reg_t;

typedef struct {
  uint8_t i2c_slv4_en : 1;
  uint8_t i2c_slv4_int_en : 1;
  uint8_t i2c_slv4_reg_dis : 1;
  uint8_t i2c_slv4_dly : 5;
} icm20948_i2c_slv4_ctrl_reg_t;

typedef struct {
  uint8_t i2c_slv4_do : 8;
} icm20948_i2c_slv4_do_reg_t;

typedef struct {
  uint8_t i2c_slv4_di : 8;
} icm20948_i2c_slv4_di_reg_t;

#endif // ICM20948_ICM20948_REGISTERS_H