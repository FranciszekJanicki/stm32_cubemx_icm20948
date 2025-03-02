#ifndef ICM20948_REGISTERS_HPP
#define ICM20948_REGISTERS_HPP

#include <cstdint>

#define PACKED __attribute__((__packed__))

namespace ICM20948 {

    struct REG_BANK_SEL {
        std::uint8_t : 2;
        std::uint8_t user_bank : 2;
        std::uint8_t : 4;
    } PACKED;

    namespace Bank0 {

        struct WHO_AM_I {
            std::uint8_t who_am_i : 8;
        } PACKED;

        struct USER_CTRL {
            std::uint8_t dmp_en : 1;
            std::uint8_t fifo_en : 1;
            std::uint8_t i2c_mst_en : 1;
            std::uint8_t i2c_if_dis : 1;
            std::uint8_t dmp_rst : 1;
            std::uint8_t sram_rst : 1;
            std::uint8_t i2c_mst_rst : 1;
        } PACKED;

        struct LP_CONFIG {
            std::uint8_t : 1;
            std::uint8_t i2c_mst_cycle : 1;
            std::uint8_t accel_cycle : 1;
            std::uint8_t gyro_cycle : 1;
            std::uint8_t : 4;
        } PACKED;

        struct PWR_MGMT_1 {
            std::uint8_t device_reset : 1;
            std::uint8_t sleep : 1;
            std::uint8_t lp_en : 1;
            std::uint8_t : 1;
            std::uint8_t temp_dis : 1;
            std::uint8_t clksel : 3;
        } PACKED;

        struct PWR_MGMT_2 {
            std::uint8_t : 2;
            std::uint8_t disable_accel : 3;
            std::uint8_t disable_gyro : 3;
        } PACKED;

        struct INT_PIN_CFG {
            std::uint8_t int1_actl : 1;
            std::uint8_t int1_open : 1;
            std::uint8_t int1_latch_en : 1;
            std::uint8_t int_anyrd_2clear : 1;
            std::uint8_t actl_fsync : 1;
            std::uint8_t fsync_int_mode_en : 1;
            std::uint8_t bypass_en : 1;
            std::uint8_t : 1;

        } PACKED;

        struct INT_ENABLE {
            std::uint8_t reg_wof_en : 1;
            std::uint8_t : 3;
            std::uint8_t wom_int_en : 1;
            std::uint8_t pll_rdy_en : 1;
            std::uint8_t dmp_int1_en : 1;
            std::uint8_t i2c_mst_int_en : 1;
        } PACKED;

        struct INT_ENABLE_1 {
            std::uint8_t : 7;
            std::uint8_t raw_data_0_rdy_en : 1;
        } PACKED;

        struct INT_ENABLE_2 {
            std::uint8_t : 3;
            std::uint8_t fifo_overflow_en : 5;
        } PACKED;

        struct INT_ENABLE_3 {
            std::uint8_t : 3;
            std::uint8_t fifo_wm_en : 5;
        } PACKED;

        struct I2C_MST_STATUS {
            std::uint8_t pass_through : 1;
            std::uint8_t i2c_slv4_done : 1;
            std::uint8_t i2c_lost_arb : 1;
            std::uint8_t i2c_slv4_nack : 1;
            std::uint8_t i2c_slv3_nack : 1;
            std::uint8_t i2c_slv2_nack : 1;
            std::uint8_t i2c_slv1_nack : 1;
            std::uint8_t i2c_slv0_nack : 1;
        } PACKED;

        struct INT_STATUS {
            std::uint8_t : 4;
            std::uint8_t wom_int : 1;
            std::uint8_t pll_rdy_int : 1;
            std::uint8_t dmp_int1 : 1;
            std::uint8_t i2c_mst_int : 1;
        } PACKED;

        struct INT_STATUS_1 {
            std::uint8_t : 7;
            std::uint8_t raw_data_0_rdy_int : 1;
        } PACKED;

        struct INT_STATUS_2 {
            std::uint8_t : 3;
            std::uint8_t fifo_overflow_int : 5;
        } PACKED;

        struct INT_STATUS_3 {
            std::uint8_t : 3;
            std::uint8_t fifo_wm_int : 5;
        } PACKED;

        struct DELAY_TIME {
            std::uint8_t delay_time_h : 8;
            std::uint8_t delay_time_l : 8;
        } PACKED;

        struct ACCEL_XOUT {
            std::uint8_t accel_xout_h : 8;
            std::uint8_t accel_xout_l : 8;
        } PACKED;

        struct ACCEL_YOUT {
            std::uint8_t accel_yout_h : 8;
            std::uint8_t accel_yout_l : 8;
        } PACKED;

        struct ACCEL_ZOUT {
            std::uint8_t accel_zout_h : 8;
            std::uint8_t accel_zout_l : 8;
        } PACKED;

        struct ACCEL_OUT {
            ACCEL_XOUT accel_xout;
            ACCEL_YOUT accel_yout;
            ACCEL_ZOUT accel_zout;
        } PACKED;

        struct GYRO_XOUT {
            std::uint8_t gyro_xout_h : 8;
            std::uint8_t gyro_xout_l : 8;
        } PACKED;

        struct GYRO_YOUT {
            std::uint8_t gyro_yout_h : 8;
            std::uint8_t gyro_yout_l : 8;
        } PACKED;

        struct GYRO_ZOUT {
            std::uint8_t gyro_zout_h : 8;
            std::uint8_t gyro_zout_l : 8;
        } PACKED;

        struct GYRO_OUT {
            GYRO_XOUT gyro_xout;
            GYRO_YOUT gyro_yout;
            GYRO_ZOUT gyro_zout;
        } PACKED;

        struct TEMP_OUT {
            std::uint8_t temp_out_h : 8;
            std::uint8_t temp_out_l : 8;
        } PACKED;

        struct EXT_SLV_SENS_DATA {
            std::uint8_t ext_slv_sens_data : 8;
        } PACKED;

        struct FIFO_EN_1 {
            std::uint8_t : 4;
            std::uint8_t slv_3_fifo_en : 1;
            std::uint8_t slv_2_fifo_en : 1;
            std::uint8_t slv_1_fifo_en : 1;
            std::uint8_t slv_0_fifo_en : 1;
        } PACKED;

        struct FIFO_EN_2 {
            std::uint8_t : 3;
            std::uint8_t accel_fifo_en : 1;
            std::uint8_t gyro_z_fifo_en : 1;
            std::uint8_t gyro_y_fifo_en : 1;
            std::uint8_t gyro_x_fifo_en : 1;
            std::uint8_t temp_fifo_en : 1;
        } PACKED;

        struct FIFO_RST {
            std::uint8_t : 3;
            std::uint8_t fifo_reset : 5;
        } PACKED;

        struct FIFO_MODE {
            std::uint8_t : 3;
            std::uint8_t fifo_mode : 5;
        } PACKED;

        struct FIFO_COUNT {
            std::uint8_t fifo_cnt_h : 8;
            std::uint8_t fifo_cnt_l : 8;
        } PACKED;

        struct FIFO_R_W {
            std::uint8_t fifo_r_w : 8;
        } PACKED;

        struct DATA_RDY_STATUS {
            std::uint8_t wof_status : 1;
            std::uint8_t : 3;
            std::uint8_t raw_data_rdy : 4;
        } PACKED;

        struct FIFO_CFG {
            std::uint8_t : 7;
            std::uint8_t fifo_cfg : 1;
        } PACKED;

    }; // namespace Bank0

    namespace Bank1 {

        struct SELF_TEST_X_GYRO {
            std::uint8_t xg_st_data : 8;
        } PACKED;

        struct SELF_TEST_Y_GYRO {
            std::uint8_t yg_st_data : 8;
        } PACKED;

        struct SELF_TEST_Z_GYRO {
            std::uint8_t zg_st_data : 8;
        } PACKED;

        struct SELF_TEST_X_ACCEL {
            std::uint8_t xa_st_data : 8;
        } PACKED;

        struct SELF_TEST_Y_ACCEL {
            std::uint8_t ya_st_data : 8;
        } PACKED;

        struct SELF_TEST_Z_ACCEL {
            std::uint8_t za_st_data : 8;
        } PACKED;

        struct XA_OFFS {
            std::uint8_t xa_offs_h : 8;
            std::uint8_t xa_offs_l : 7;
            std::uint8_t : 1;
        } PACKED;

        struct YA_OFFS {
            std::uint8_t ya_offs_h : 8;
            std::uint8_t ya_offs_l : 7;
            std::uint8_t : 1;
        } PACKED;

        struct ZA_OFFS {
            std::uint8_t za_offs_h : 8;
            std::uint8_t za_offs_l : 7;
            std::uint8_t : 1;
        } PACKED;

        struct TIMEBASE_CORRECTION_PLL {
            std::uint8_t tbc_pll : 8;
        } PACKED;

    }; // namespace Bank1

    namespace Bank2 {

        struct GYRO_SMPLRT_DIV {
            std::uint8_t gyro_smplrt_div : 8;
        } PACKED;

        struct GYRO_CONFIG_1 {
            std::uint8_t : 2;
            std::uint8_t gyro_dplfcfg : 3;
            std::uint8_t gyro_fs_sel : 2;
            std::uint8_t gyro_fchoice : 1;
        } PACKED;

        struct GYRO_CONFIG_2 {
            std::uint8_t : 2;
            std::uint8_t xgyro_cten : 1;
            std::uint8_t ygyro_cten : 1;
            std::uint8_t zgyro_cten : 1;
            std::uint8_t gyro_avgcfg : 3;
        } PACKED;

        struct XG_OFFS_USR {
            std::uint8_t xg_offs_usr_h : 8;
            std::uint8_t xg_offs_usr_l : 8;
        } PACKED;

        struct YG_OFFS_USR {
            std::uint8_t yg_offs_usr_h : 8;
            std::uint8_t yg_offs_usr_l : 8;
        } PACKED;

        struct ZG_OFFS_USR {
            std::uint8_t zg_offs_usr_h : 8;
            std::uint8_t zg_offs_usr_l : 8;
        } PACKED;

        struct ODR_ALIGN_EN {
            std::uint8_t : 7;
            std::uint8_t odr_align_en : 1;
        } PACKED;

        struct ACCEL_SMPLRT_DIV {
            std::uint8_t : 4;
            std::uint8_t accel_smplrt_div : 12;
        } PACKED;

        struct ACCEL_INTEL_CTRL {
            std::uint8_t : 6;
            std::uint8_t accel_intel_en : 1;
            std::uint8_t accel_intel_mode_int : 1;
        } PACKED;

        struct ACCEL_WOM_THR {
            std::uint8_t wom_threshold : 8;
        } PACKED;

        struct ACCEL_CONFIG_1 {
            std::uint8_t : 2;
            std::uint8_t accel_dlpfcfg : 3;
            std::uint8_t accel_fs_sel : 2;
            std::uint8_t accel_fchoice : 1;
        } PACKED;

        struct ACCEL_CONFIG_2 {
            std::uint8_t : 3;
            std::uint8_t ax_st_en_reg : 1;
            std::uint8_t ay_st_en_reg : 1;
            std::uint8_t az_st_en_reg : 1;
            std::uint8_t dec3_cfg : 2;
        } PACKED;

        struct FSYNC_CONFIG {
            std::uint8_t delay_time_en : 1;
            std::uint8_t : 1;
            std::uint8_t wof_deglitch_en : 1;
            std::uint8_t wof_edge_int : 1;
            std::uint8_t ext_sync_set : 4;
        } PACKED;

        struct TEMP_CONFIG {
            std::uint8_t : 5;
            std::uint8_t temp_dlpfcfg : 3;
        } PACKED;

        struct MOD_CTRL_USR {
            std::uint8_t : 7;
            std::uint8_t reg_lp_dmp_en : 1;
        } PACKED;

    }; // namespace Bank2

    namespace Bank3 {

        struct I2C_MST_ODR_CONFIG {
            std::uint8_t : 4;
            std::uint8_t i2c_mst_odr_config : 4;
        } PACKED;

        struct I2C_MST_CTRL {
            std::uint8_t mult_mst_en : 1;
            std::uint8_t : 2;
            std::uint8_t i2c_mst_p_nsr : 1;
            std::uint8_t i2c_mst_clk : 4;
        } PACKED;

        struct I2C_MST_DELAY_CTRL {
            std::uint8_t delay_es_shadow : 1;
            std::uint8_t : 2;
            std::uint8_t i2c_slv4_delay_en : 1;
            std::uint8_t i2c_slv3_delay_en : 1;
            std::uint8_t i2c_slv2_delay_en : 1;
            std::uint8_t i2c_slv1_delay_en : 1;
            std::uint8_t i2c_slv0_delay_en : 1;
        } PACKED;

        struct I2C_SLV_ADDR {
            std::uint8_t i2c_slv_rnw : 1;
            std::uint8_t ic_id : 7;
        } PACKED;

        struct I2C_SLV_REG {
            std::uint8_t i2c_slv_reg : 8;
        } PACKED;

        struct I2C_SLV_CTRL {
            std::uint8_t i2c_slv_en : 1;
            std::uint8_t i2c_slv_byte_sw : 1;
            std::uint8_t i2c_slv_reg_dis : 1;
            std::uint8_t i2c_slv_grp : 1;
            std::uint8_t i2c_slv_leng : 4;
        } PACKED;

        struct I2C_SLV_DO {
            std::uint8_t i2c_slv_do : 8;
        } PACKED;

        struct I2C_SLV4_ADDR {
            std::uint8_t i2c_slv4_rnw : 1;
            std::uint8_t i2c_id_4 : 7;
        } PACKED;

        struct I2C_SLV4_REG {
            std::uint8_t i2c_slv4_reg : 8;
        } PACKED;

        struct I2C_SLV4_CTRL {
            std::uint8_t i2c_slv4_en : 1;
            std::uint8_t i2c_slv4_int_en : 1;
            std::uint8_t i2c_slv4_reg_dis : 1;
            std::uint8_t i2c_slv4_dly : 5;
        } PACKED;

        struct I2C_SLV4_DO {
            std::uint8_t i2c_slv4_do : 8;
        } PACKED;

        struct I2C_SLV4_DI {
            std::uint8_t i2c_slv4_di : 8;
        } PACKED;

    }; // namespace Bank3

}; // namespace ICM20948

#undef PACKED

#endif // ICM20948_REGISTERS_HPP