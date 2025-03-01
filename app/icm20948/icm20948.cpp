#include "icm20948.hpp"
#include "icm20948_config.hpp"
#include "icm20948_registers.hpp"

namespace ICM20948 {

    ICM20948::ICM20948(I2CDevice&& i2c_device) noexcept : i2c_device_{std::forward<I2CDevice>(i2c_device)}
    {
        this->initialize();
    }

    ICM20948::~ICM20948() noexcept
    {
        this->deinitialize();
    }

    void ICM20948::initialize() noexcept
    {}

    void ICM20948::deinitialize() noexcept
    {}

    std::uint8_t ICM20948::read_byte(Bank0::RA const reg_address) const noexcept
    {
        this->select_bank(Bank::USER_BANK_0);
        return this->i2c_device_.read_byte(std::to_underlying(reg_address));
    }

    std::uint8_t ICM20948::read_byte(Bank1::RA const reg_address) const noexcept
    {
        this->select_bank(Bank::USER_BANK_1);
        return this->i2c_device_.read_byte(std::to_underlying(reg_address));
    }

    std::uint8_t ICM20948::read_byte(Bank2::RA const reg_address) const noexcept
    {
        this->select_bank(Bank::USER_BANK_2);
        return this->i2c_device_.read_byte(std::to_underlying(reg_address));
    }

    std::uint8_t ICM20948::read_byte(Bank3::RA const reg_address) const noexcept
    {
        this->select_bank(Bank::USER_BANK_3);
        return this->i2c_device_.read_byte(std::to_underlying(reg_address));
    }

    void ICM20948::write_byte(Bank0::RA const reg_address, std::uint8_t const byte) const noexcept
    {
        this->select_bank(Bank::USER_BANK_0);
        this->i2c_device_.write_byte(std::to_underlying(reg_address), byte);
    }

    void ICM20948::write_byte(Bank1::RA const reg_address, std::uint8_t const byte) const noexcept
    {
        this->select_bank(Bank::USER_BANK_1);
        this->i2c_device_.write_byte(std::to_underlying(reg_address), byte);
    }

    void ICM20948::write_byte(Bank2::RA const reg_address, std::uint8_t const byte) const noexcept
    {
        this->select_bank(Bank::USER_BANK_2);
        this->i2c_device_.write_byte(std::to_underlying(reg_address), byte);
    }

    void ICM20948::write_byte(Bank3::RA const reg_address, std::uint8_t const byte) const noexcept
    {
        this->select_bank(Bank::USER_BANK_3);
        this->i2c_device_.write_byte(std::to_underlying(reg_address), byte);
    }

    void ICM20948::select_bank(Bank const bank) const noexcept
    {
        this->set_reg_bank_sel_register(REG_BANK_SEL{.user_bank = std::to_underlying(bank)});
    }

    REG_BANK_SEL ICM20948::ICM20948::get_reg_bank_sel_register() const noexcept
    {
        return std::bit_cast<REG_BANK_SEL>(this->i2c_device_.read_byte(RA_REG_BANK_SEL));
    }

    void ICM20948::set_reg_bank_sel_register(REG_BANK_SEL const reg_bank_sel) const noexcept
    {
        this->i2c_device_.write_byte(RA_REG_BANK_SEL, std::bit_cast<std::uint8_t>(reg_bank_sel));
    }

    Bank0::WHO_AM_I ICM20948::get_who_am_i_register() const noexcept
    {}

    Bank0::USER_CTRL ICM20948::get_user_ctrl_register() const noexcept
    {}

    void ICM20948::set_user_ctrl_register(Bank0::USER_CTRL const user_ctrl) const noexcept
    {}

    Bank0::LP_CONFIG ICM20948::get_lp_config_register() const noexcept
    {}

    void ICM20948::set_lp_config_register(Bank0::LP_CONFIG const lp_config) const noexcept
    {}

    Bank0::PWR_MGMT_1 ICM20948::get_pwr_mgmt_1_register() const noexcept
    {}

    void ICM20948::set_pwr_mgmt_1_register(Bank0::PWR_MGMT_1 const pwr_mgmt_1) const noexcept
    {}

    Bank0::PWR_MGMT_2 ICM20948::get_pwr_mgmt_2_register() const noexcept
    {}

    void ICM20948::set_pwr_mgmt_2_register(Bank0::PWR_MGMT_2 const pwr_mgmt_2) const noexcept
    {}

    Bank0::INT_PIN_CFG ICM20948::get_int_pin_cfg_register() const noexcept
    {}

    void ICM20948::set_int_pin_cfg_register(Bank0::INT_PIN_CFG const int_pin_cfg) const noexcept
    {}

    Bank0::INT_ENABLE ICM20948::get_int_enable_register() const noexcept
    {}

    void ICM20948::set_int_enable_register(Bank0::INT_ENABLE const int_enable) const noexcept
    {}

    Bank0::INT_ENABLE_1 ICM20948::get_int_enable_1_register() const noexcept
    {}

    void ICM20948::set_int_enable_1_register(Bank0::INT_ENABLE_1 const int_enable_1) const noexcept
    {}

    Bank0::INT_ENABLE_2 ICM20948::get_int_enable_2_register() const noexcept
    {}

    void ICM20948::set_int_enable_2_register(Bank0::INT_ENABLE_2 const int_enable_2) const noexcept
    {}

    Bank0::INT_ENABLE_3 ICM20948::get_int_enable_3_register() const noexcept
    {}

    void ICM20948::set_int_enable_3_register(Bank0::INT_ENABLE_3 const int_enable_3) const noexcept
    {}

    Bank0::I2C_MST_STATUS ICM20948::get_i2c_mst_status_register() const noexcept
    {}

    Bank0::INT_STATUS ICM20948::get_int_status_register() const noexcept
    {}

    Bank0::INT_STATUS_1 ICM20948::get_int_status_1_register() const noexcept
    {}

    Bank0::INT_STATUS_2 ICM20948::get_int_status_2_register() const noexcept
    {}

    Bank0::INT_STATUS_3 ICM20948::get_int_status_3_register() const noexcept
    {}

    Bank0::DELAY_TIME ICM20948::get_delay_time_registers() const noexcept
    {}

    Bank0::ACCEL_XOUT ICM20948::get_accel_xout_registers() const noexcept
    {}

    Bank0::ACCEL_YOUT ICM20948::get_accel_yout_registers() const noexcept
    {}

    Bank0::ACCEL_ZOUT ICM20948::get_accel_zout_registers() const noexcept
    {}

    Bank0::ACCEL_OUT ICM20948::get_accel_out_registers() const noexcept
    {}

    Bank0::GYRO_XOUT ICM20948::get_gyro_xout_registers() const noexcept
    {}

    Bank0::GYRO_YOUT ICM20948::get_gyro_yout_registers() const noexcept
    {}

    Bank0::GYRO_ZOUT ICM20948::get_gyro_zout_registers() const noexcept
    {}

    Bank0::GYRO_OUT ICM20948::get_gyro_out_registers() const noexcept
    {}

    Bank0::TEMP_OUT ICM20948::get_temp_out_registers() const noexcept
    {}

    Bank0::EXT_SLV_SENS_DATA ICM20948::get_ext_slv_sens_data_register(std::uint8_t const num) const noexcept
    {}

    Bank0::FIFO_EN_1 ICM20948::get_fifo_en_1_register() const noexcept
    {}

    void ICM20948::set_fifo_en_1_register(Bank0::FIFO_EN_1 const fifo_en_1) const noexcept
    {}

    Bank0::FIFO_EN_2 ICM20948::get_fifo_en_2_register() const noexcept
    {}

    void ICM20948::set_fifo_en_2_register(Bank0::FIFO_EN_2 const fifo_en_2) const noexcept
    {}

    Bank0::FIFO_RST ICM20948::get_fifo_rst_register() const noexcept
    {}

    void ICM20948::set_fifo_rst_register(Bank0::FIFO_RST const fifo_rst) const noexcept
    {}

    Bank0::FIFO_MODE ICM20948::get_fifo_mode_register() const noexcept
    {}

    void ICM20948::set_fifo_mode_register(Bank0::FIFO_MODE const fifo_mode) const noexcept
    {}

    Bank0::FIFO_COUNT ICM20948::get_fifo_count_registers() const noexcept
    {}

    Bank0::FIFO_R_W ICM20948::get_fifo_r_w_register() const noexcept
    {}

    void ICM20948::set_fifo_r_w_register(Bank0::FIFO_R_W const fifo_r_w) const noexcept
    {}

    Bank0::DATA_RDY_STATUS ICM20948::get_data_rdy_status() const noexcept
    {}

    Bank0::FIFO_CFG ICM20948::get_fifo_cfg_register() const noexcept
    {}

    void ICM20948::set_fifo_cfg_register(Bank0::FIFO_CFG const fifo_cfg) const noexcept
    {}

    Bank1::SELF_TEST_X_GYRO ICM20948::get_self_test_x_gyro_register() const noexcept
    {}

    void ICM20948::set_self_test_x_gyro_register(Bank1::SELF_TEST_X_GYRO const self_text_x_gyro) const noexcept
    {}

    Bank1::SELF_TEST_Y_GYRO ICM20948::get_self_test_y_gyro_register() const noexcept
    {}

    void ICM20948::set_self_test_x_gyro_register(Bank1::SELF_TEST_Y_GYRO const self_text_y_gyro) const noexcept
    {}

    Bank1::SELF_TEST_Z_GYRO ICM20948::get_self_test_z_gyro_register() const noexcept
    {}

    void ICM20948::set_self_test_x_gyro_register(Bank1::SELF_TEST_Z_GYRO const self_text_z_gyro) const noexcept
    {}

    Bank1::SELF_TEST_X_ACCEL ICM20948::get_self_test_x_accel_register() const noexcept
    {}

    void ICM20948::set_self_test_x_accel_register(Bank1::SELF_TEST_X_ACCEL const self_text_x_accel) const noexcept
    {}

    Bank1::SELF_TEST_Y_ACCEL ICM20948::get_self_test_y_accel_register() const noexcept
    {}

    void ICM20948::set_self_test_y_accel_register(Bank1::SELF_TEST_Y_ACCEL const self_text_y_accel) const noexcept
    {}

    Bank1::SELF_TEST_Z_ACCEL ICM20948::get_self_test_z_accel_register() const noexcept
    {}

    void ICM20948::set_self_test_z_accel_register(Bank1::SELF_TEST_Z_ACCEL const self_text_z_accel) const noexcept
    {}

    Bank1::XA_OFFS ICM20948::get_xa_offs_registers() const noexcept
    {}

    void ICM20948::set_xa_offs_registers(Bank1::XA_OFFS const xa_offs) const noexcept
    {}

    Bank1::YA_OFFS ICM20948::get_ya_offs_registers() const noexcept
    {}

    void ICM20948::set_ya_offs_registers(Bank1::YA_OFFS const ya_offs) const noexcept
    {}

    Bank1::ZA_OFFS ICM20948::get_za_offs_registers() const noexcept
    {}

    void ICM20948::set_za_offs_registers(Bank1::ZA_OFFS const za_offs) const noexcept
    {}

    Bank1::TIMEBASE_CORRECTION_PLL ICM20948::get_timebase_correction_pll_register() const noexcept
    {}

    void
    ICM20948::set_timebase_correction_pll(Bank1::TIMEBASE_CORRECTION_PLL const timebase_correction_pll) const noexcept
    {}

    Bank2::GYRO_SMPLRT_DIV ICM20948::get_gyro_smplrt_div_register() const noexcept
    {}

    void ICM20948::set_gyro_smplrt_div_register(Bank2::GYRO_SMPLRT_DIV const gyro_smplrt_div) const noexcept
    {}

    Bank2::GYRO_CONFIG_1 ICM20948::get_gyro_config_1_register() const noexcept
    {}

    void ICM20948::set_gyro_config_1_register(Bank2::GYRO_CONFIG_1 const gyro_config_1) const noexcept
    {}

    Bank2::GYRO_CONFIG_2 ICM20948::get_gyro_config_2_register() const noexcept
    {}

    void ICM20948::set_gyro_config_2_register(Bank2::GYRO_CONFIG_2 const gyro_config_2) const noexcept
    {}

    Bank2::XG_OFFS_USR ICM20948::get_xg_offs_usr_registers() const noexcept
    {}

    void ICM20948::set_xg_offs_usr_registers(Bank2::XG_OFFS_USR const xg_offs_usr) const noexcept
    {}

    Bank2::YG_OFFS_USR ICM20948::get_yg_offs_usr_registers() const noexcept
    {}

    void ICM20948::set_yg_offs_usr_registers(Bank2::YG_OFFS_USR const yg_offs_usr) const noexcept
    {}

    Bank2::ZG_OFFS_USR ICM20948::get_zg_offs_usr_registers() const noexcept
    {}

    void ICM20948::set_zg_offs_usr_registers(Bank2::ZG_OFFS_USR const zg_offs_usr) const noexcept
    {}

    Bank2::ODR_ALIGN_EN ICM20948::get_odr_align_en_register() const noexcept
    {}

    void ICM20948::set_odr_align_en_register(Bank2::ODR_ALIGN_EN const odr_align_en) const noexcept
    {}

    Bank2::ACCEL_SMPLRT_DIV ICM20948::get_accel_smplrt_div_registers() const noexcept
    {}

    void ICM20948::set_accel_smplrt_div_registers(Bank2::ACCEL_SMPLRT_DIV const accel_smplrt_div) const noexcept
    {}

    Bank2::ACCEL_INTEL_CTRL ICM20948::get_accel_intel_ctrl_register() const noexcept
    {}

    void ICM20948::set_accel_intel_ctrl_register(Bank2::ACCEL_INTEL_CTRL const accel_intel_ctrl) const noexcept
    {}

    Bank2::ACCEL_WOM_THR ICM20948::get_accel_wom_thr_register() const noexcept
    {}

    void ICM20948::set_accel_wom_thr_register(Bank2::ACCEL_WOM_THR const accel_wom_thr) const noexcept
    {}

    Bank2::ACCEL_CONFIG ICM20948::get_accel_config_register() const noexcept
    {}

    void ICM20948::set_accel_config_register(Bank2::ACCEL_CONFIG const accel_config) const noexcept
    {}

    Bank2::ACCEL_CONFIG_2 ICM20948::get_accel_config_2_register() const noexcept
    {}

    void ICM20948::set_accel_config_2_register(Bank2::ACCEL_CONFIG_2 const accel_config_2) const noexcept
    {}

    Bank2::FSYNC_CONFIG ICM20948::get_fsync_config_register() const noexcept
    {}

    void ICM20948::set_fsync_config_register(Bank2::FSYNC_CONFIG const fsync_config) const noexcept
    {}

    Bank2::TEMP_CONFIG ICM20948::get_temp_config_register() const noexcept
    {}

    void ICM20948::set_temp_config_register(Bank2::TEMP_CONFIG const temp_config) const noexcept
    {}

    Bank2::MOD_CTRL_USR ICM20948::get_mod_ctrl_usr_register() const noexcept
    {}

    void ICM20948::set_mod_ctrl_usr_register(Bank2::MOD_CTRL_USR const mod_ctrl_usr) const noexcept
    {}

    Bank3::I2C_MST_ODR_CONFIG ICM20948::get_i2c_mst_odr_config_register() const noexcept
    {}

    void ICM20948::set_i2c_mst_odr_config_register(Bank3::I2C_MST_ODR_CONFIG const i2c_mst_odr_config) const noexcept
    {}

    Bank3::I2C_MST_CTRL ICM20948::get_i2c_mst_ctrl_register() const noexcept
    {}

    void ICM20948::set_i2c_mst_ctrl_register(Bank3::I2C_MST_CTRL const i2c_mst_ctrl) const noexcept
    {}

    Bank3::I2C_MST_DELAY_CTRL ICM20948::get_i2c_mst_delay_ctrl_register() const noexcept
    {}

    void ICM20948::set_i2c_mst_delay_ctrl_register(Bank3::I2C_MST_DELAY_CTRL const i2c_mst_delay_ctrl) const noexcept
    {}

    Bank3::I2C_SLV_ADDR ICM20948::get_i2c_slv_addr_register(SlaveNum const slave_num) const noexcept
    {}

    void ICM20948::set_i2c_slv_addr_register(SlaveNum const slave_num,
                                             Bank3::I2C_SLV_ADDR const i2c_slv_addr) const noexcept
    {}

    Bank3::I2C_SLV_REG ICM20948::get_i2c_slv_reg_register(SlaveNum const slave_num) const noexcept
    {}

    void ICM20948::set_i2c_slv_reg_register(SlaveNum const slave_num,
                                            Bank3::I2C_SLV_REG const i2c_slv_reg) const noexcept
    {}

    Bank3::I2C_SLV_CTRL ICM20948::get_i2c_slv_ctrl_register(SlaveNum const slave_num) const noexcept
    {}

    void ICM20948::set_i2c_slv_ctrl_register(SlaveNum const slave_num,
                                             Bank3::I2C_SLV_CTRL const i2c_slv_ctrl) const noexcept
    {}

    Bank3::I2C_SLV_DO ICM20948::get_i2c_slv_do_register(SlaveNum const slave_num) const noexcept
    {}

    void ICM20948::set_i2c_slv_do_register(SlaveNum const slave_num, Bank3::I2C_SLV_DO const i2c_slv_do) const noexcept
    {}

    Bank3::I2C_SLV4_ADDR ICM20948::get_i2c_slv4_addr_register() const noexcept
    {}

    void ICM20948::set_i2c_slv4_addr_register(Bank3::I2C_SLV4_ADDR const i2c_slv4_addr) const noexcept
    {}

    Bank3::I2C_SLV4_REG ICM20948::get_i2c_slv4_reg_register() const noexcept
    {}

    void ICM20948::set_i2c_slv4_reg_register(Bank3::I2C_SLV4_REG const i2c_slv4_reg) const noexcept
    {}

    Bank3::I2C_SLV4_CTRL ICM20948::get_i2c_slv4_ctrl_register() const noexcept
    {}

    void ICM20948::set_i2c_slv4_ctrl_register(Bank3::I2C_SLV4_CTRL const i2c_slv4_ctrl) const noexcept
    {}

    Bank3::I2C_SLV4_DO ICM20948::get_i2c_slv4_do_register() const noexcept
    {}

    void ICM20948::set_i2c_slv4_do_register(Bank3::I2C_SLV4_DO const i2c_slv4_do) const noexcept
    {}

    Bank3::I2C_SLV4_DI ICM20948::get_i2c_slv4_di_register() const noexcept
    {}

}; // namespace ICM20948