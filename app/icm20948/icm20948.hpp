#ifndef ICM20948_HPP
#define ICM20948_HPP

#include "ak09916.hpp"
#include "i2c_device.hpp"
#include "icm20948_config.hpp"
#include "icm20948_dmp_mem.hpp"
#include "icm20948_registers.hpp"
#include "utility.hpp"
#include <utility>

namespace ICM20948 {

    struct ICM20948 {
    public:
        using I2CDevice = Utility::I2CDevice;

        ICM20948() noexcept = default;

        ICM20948(I2CDevice&& i2c_device) noexcept;

        ICM20948(ICM20948 const& other) = delete;
        ICM20948(ICM20948&& other) noexcept = default;

        ICM20948& operator=(ICM20948 const& other) = delete;
        ICM20948& operator=(ICM20948&& other) noexcept = default;

        ~ICM20948() noexcept;

    private:
        void initialize() noexcept;
        void deinitialize() noexcept;

        std::uint8_t read_byte(Bank const bank, std::uint8_t const reg_address) const noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> read_bytes(Bank const bank, std::uint8_t const reg_address) const noexcept;

        void write_byte(Bank const bank, std::uint8_t const reg_address, std::uint8_t const byte) const noexcept;

        template <std::size_t SIZE>
        void write_bytes(Bank const bank,
                         std::uint8_t const reg_address,
                         std::array<std::uint8_t, SIZE> const& bytes) const noexcept;

        void select_bank(Bank const bank) const noexcept;

        REG_BANK_SEL get_reg_bank_sel_register() const noexcept;
        void set_reg_bank_sel_register(REG_BANK_SEL const reg_bank_sel) const noexcept;

        Bank0::WHO_AM_I get_who_am_i_register() const noexcept;

        Bank0::USER_CTRL get_user_ctrl_register() const noexcept;
        void set_user_ctrl_register(Bank0::USER_CTRL const user_ctrl) const noexcept;

        Bank0::LP_CONFIG get_lp_config_register() const noexcept;
        void set_lp_config_register(Bank0::LP_CONFIG const lp_config) const noexcept;

        Bank0::PWR_MGMT_1 get_pwr_mgmt_1_register() const noexcept;
        void set_pwr_mgmt_1_register(Bank0::PWR_MGMT_1 const pwr_mgmt_1) const noexcept;

        Bank0::PWR_MGMT_2 get_pwr_mgmt_2_register() const noexcept;
        void set_pwr_mgmt_2_register(Bank0::PWR_MGMT_2 const pwr_mgmt_2) const noexcept;

        Bank0::INT_PIN_CFG get_int_pin_cfg_register() const noexcept;
        void set_int_pin_cfg_register(Bank0::INT_PIN_CFG const int_pin_cfg) const noexcept;

        Bank0::INT_ENABLE get_int_enable_register() const noexcept;
        void set_int_enable_register(Bank0::INT_ENABLE const int_enable) const noexcept;

        Bank0::INT_ENABLE_1 get_int_enable_1_register() const noexcept;
        void set_int_enable_1_register(Bank0::INT_ENABLE_1 const int_enable_1) const noexcept;

        Bank0::INT_ENABLE_2 get_int_enable_2_register() const noexcept;
        void set_int_enable_2_register(Bank0::INT_ENABLE_2 const int_enable_2) const noexcept;

        Bank0::INT_ENABLE_3 get_int_enable_3_register() const noexcept;
        void set_int_enable_3_register(Bank0::INT_ENABLE_3 const int_enable_3) const noexcept;

        Bank0::I2C_MST_STATUS get_i2c_mst_status_register() const noexcept;

        Bank0::INT_STATUS get_int_status_register() const noexcept;

        Bank0::INT_STATUS_1 get_int_status_1_register() const noexcept;

        Bank0::INT_STATUS_2 get_int_status_2_register() const noexcept;

        Bank0::INT_STATUS_3 get_int_status_3_register() const noexcept;

        Bank0::DELAY_TIME get_delay_time_registers() const noexcept;

        Bank0::ACCEL_XOUT get_accel_xout_registers() const noexcept;

        Bank0::ACCEL_YOUT get_accel_yout_registers() const noexcept;

        Bank0::ACCEL_ZOUT get_accel_zout_registers() const noexcept;

        Bank0::ACCEL_OUT get_accel_out_registers() const noexcept;

        Bank0::GYRO_XOUT get_gyro_xout_registers() const noexcept;

        Bank0::GYRO_YOUT get_gyro_yout_registers() const noexcept;

        Bank0::GYRO_ZOUT get_gyro_zout_registers() const noexcept;

        Bank0::GYRO_OUT get_gyro_out_registers() const noexcept;

        Bank0::TEMP_OUT get_temp_out_registers() const noexcept;

        Bank0::EXT_SLV_SENS_DATA get_ext_slv_sens_data_register(std::uint8_t const num) const noexcept;

        Bank0::FIFO_EN_1 get_fifo_en_1_register() const noexcept;
        void set_fifo_en_1_register(Bank0::FIFO_EN_1 const fifo_en_1) const noexcept;

        Bank0::FIFO_EN_2 get_fifo_en_2_register() const noexcept;
        void set_fifo_en_2_register(Bank0::FIFO_EN_2 const fifo_en_2) const noexcept;

        Bank0::FIFO_RST get_fifo_rst_register() const noexcept;
        void set_fifo_rst_register(Bank0::FIFO_RST const fifo_rst) const noexcept;

        Bank0::FIFO_MODE get_fifo_mode_register() const noexcept;
        void set_fifo_mode_register(Bank0::FIFO_MODE const fifo_mode) const noexcept;

        Bank0::FIFO_COUNT get_fifo_count_registers() const noexcept;

        Bank0::FIFO_R_W get_fifo_r_w_register() const noexcept;
        void set_fifo_r_w_register(Bank0::FIFO_R_W const fifo_r_w) const noexcept;

        Bank0::DATA_RDY_STATUS get_data_rdy_status() const noexcept;

        Bank0::FIFO_CFG get_fifo_cfg_register() const noexcept;
        void set_fifo_cfg_register(Bank0::FIFO_CFG const fifo_cfg) const noexcept;

        Bank1::SELF_TEST_X_GYRO get_self_test_x_gyro_register() const noexcept;
        void set_self_test_x_gyro_register(Bank1::SELF_TEST_X_GYRO const self_text_x_gyro) const noexcept;

        Bank1::SELF_TEST_Y_GYRO get_self_test_y_gyro_register() const noexcept;
        void set_self_test_x_gyro_register(Bank1::SELF_TEST_Y_GYRO const self_text_y_gyro) const noexcept;

        Bank1::SELF_TEST_Z_GYRO get_self_test_z_gyro_register() const noexcept;
        void set_self_test_x_gyro_register(Bank1::SELF_TEST_Z_GYRO const self_text_z_gyro) const noexcept;

        Bank1::SELF_TEST_X_ACCEL get_self_test_x_accel_register() const noexcept;
        void set_self_test_x_accel_register(Bank1::SELF_TEST_X_ACCEL const self_text_x_accel) const noexcept;

        Bank1::SELF_TEST_Y_ACCEL get_self_test_y_accel_register() const noexcept;
        void set_self_test_y_accel_register(Bank1::SELF_TEST_Y_ACCEL const self_text_y_accel) const noexcept;

        Bank1::SELF_TEST_Z_ACCEL get_self_test_z_accel_register() const noexcept;
        void set_self_test_z_accel_register(Bank1::SELF_TEST_Z_ACCEL const self_text_z_accel) const noexcept;

        Bank1::XA_OFFS get_xa_offs_registers() const noexcept;
        void set_xa_offs_registers(Bank1::XA_OFFS const xa_offs) const noexcept;

        Bank1::YA_OFFS get_ya_offs_registers() const noexcept;
        void set_ya_offs_registers(Bank1::YA_OFFS const ya_offs) const noexcept;

        Bank1::ZA_OFFS get_za_offs_registers() const noexcept;
        void set_za_offs_registers(Bank1::ZA_OFFS const za_offs) const noexcept;

        Bank1::TIMEBASE_CORRECTION_PLL get_timebase_correction_pll_register() const noexcept;
        void set_timebase_correction_pll(Bank1::TIMEBASE_CORRECTION_PLL const timebase_correction_pll) const noexcept;

        Bank2::GYRO_SMPLRT_DIV get_gyro_smplrt_div_register() const noexcept;
        void set_gyro_smplrt_div_register(Bank2::GYRO_SMPLRT_DIV const gyro_smplrt_div) const noexcept;

        Bank2::GYRO_CONFIG_1 get_gyro_config_1_register() const noexcept;
        void set_gyro_config_1_register(Bank2::GYRO_CONFIG_1 const gyro_config_1) const noexcept;

        Bank2::GYRO_CONFIG_2 get_gyro_config_2_register() const noexcept;
        void set_gyro_config_2_register(Bank2::GYRO_CONFIG_2 const gyro_config_2) const noexcept;

        Bank2::XG_OFFS_USR get_xg_offs_usr_registers() const noexcept;
        void set_xg_offs_usr_registers(Bank2::XG_OFFS_USR const xg_offs_usr) const noexcept;

        Bank2::YG_OFFS_USR get_yg_offs_usr_registers() const noexcept;
        void set_yg_offs_usr_registers(Bank2::YG_OFFS_USR const yg_offs_usr) const noexcept;

        Bank2::ZG_OFFS_USR get_zg_offs_usr_registers() const noexcept;
        void set_zg_offs_usr_registers(Bank2::ZG_OFFS_USR const zg_offs_usr) const noexcept;

        Bank2::ODR_ALIGN_EN get_odr_align_en_register() const noexcept;
        void set_odr_align_en_register(Bank2::ODR_ALIGN_EN const odr_align_en) const noexcept;

        Bank2::ACCEL_SMPLRT_DIV get_accel_smplrt_div_registers() const noexcept;
        void set_accel_smplrt_div_registers(Bank2::ACCEL_SMPLRT_DIV const accel_smplrt_div) const noexcept;

        Bank2::ACCEL_INTEL_CTRL get_accel_intel_ctrl_register() const noexcept;
        void set_accel_intel_ctrl_register(Bank2::ACCEL_INTEL_CTRL const accel_intel_ctrl) const noexcept;

        Bank2::ACCEL_WOM_THR get_accel_wom_thr_register() const noexcept;
        void set_accel_wom_thr_register(Bank2::ACCEL_WOM_THR const accel_wom_thr) const noexcept;

        Bank2::ACCEL_CONFIG_1 get_accel_config_1_register() const noexcept;
        void set_accel_config_1_register(Bank2::ACCEL_CONFIG_1 const accel_config_1) const noexcept;

        Bank2::ACCEL_CONFIG_2 get_accel_config_2_register() const noexcept;
        void set_accel_config_2_register(Bank2::ACCEL_CONFIG_2 const accel_config_2) const noexcept;

        Bank2::FSYNC_CONFIG get_fsync_config_register() const noexcept;
        void set_fsync_config_register(Bank2::FSYNC_CONFIG const fsync_config) const noexcept;

        Bank2::TEMP_CONFIG get_temp_config_register() const noexcept;
        void set_temp_config_register(Bank2::TEMP_CONFIG const temp_config) const noexcept;

        Bank2::MOD_CTRL_USR get_mod_ctrl_usr_register() const noexcept;
        void set_mod_ctrl_usr_register(Bank2::MOD_CTRL_USR const mod_ctrl_usr) const noexcept;

        Bank3::I2C_MST_ODR_CONFIG get_i2c_mst_odr_config_register() const noexcept;
        void set_i2c_mst_odr_config_register(Bank3::I2C_MST_ODR_CONFIG const i2c_mst_odr_config) const noexcept;

        Bank3::I2C_MST_CTRL get_i2c_mst_ctrl_register() const noexcept;
        void set_i2c_mst_ctrl_register(Bank3::I2C_MST_CTRL const i2c_mst_ctrl) const noexcept;

        Bank3::I2C_MST_DELAY_CTRL get_i2c_mst_delay_ctrl_register() const noexcept;
        void set_i2c_mst_delay_ctrl_register(Bank3::I2C_MST_DELAY_CTRL const i2c_mst_delay_ctrl) const noexcept;

        Bank3::I2C_SLV_ADDR get_i2c_slv_addr_register(SlaveNum const slave_num) const noexcept;
        void set_i2c_slv_addr_register(SlaveNum const slave_num, Bank3::I2C_SLV_ADDR const i2c_slv_addr) const noexcept;

        Bank3::I2C_SLV_REG get_i2c_slv_reg_register(SlaveNum const slave_num) const noexcept;
        void set_i2c_slv_reg_register(SlaveNum const slave_num, Bank3::I2C_SLV_REG const i2c_slv_reg) const noexcept;

        Bank3::I2C_SLV_CTRL get_i2c_slv_ctrl_register(SlaveNum const slave_num) const noexcept;
        void set_i2c_slv_ctrl_register(SlaveNum const slave_num, Bank3::I2C_SLV_CTRL const i2c_slv_ctrl) const noexcept;

        Bank3::I2C_SLV_DO get_i2c_slv_do_register(SlaveNum const slave_num) const noexcept;
        void set_i2c_slv_do_register(SlaveNum const slave_num, Bank3::I2C_SLV_DO const i2c_slv_do) const noexcept;

        Bank3::I2C_SLV4_ADDR get_i2c_slv4_addr_register() const noexcept;
        void set_i2c_slv4_addr_register(Bank3::I2C_SLV4_ADDR const i2c_slv4_addr) const noexcept;

        Bank3::I2C_SLV4_REG get_i2c_slv4_reg_register() const noexcept;
        void set_i2c_slv4_reg_register(Bank3::I2C_SLV4_REG const i2c_slv4_reg) const noexcept;

        Bank3::I2C_SLV4_CTRL get_i2c_slv4_ctrl_register() const noexcept;
        void set_i2c_slv4_ctrl_register(Bank3::I2C_SLV4_CTRL const i2c_slv4_ctrl) const noexcept;

        Bank3::I2C_SLV4_DO get_i2c_slv4_do_register() const noexcept;
        void set_i2c_slv4_do_register(Bank3::I2C_SLV4_DO const i2c_slv4_do) const noexcept;

        Bank3::I2C_SLV4_DI get_i2c_slv4_di_register() const noexcept;

        bool initialized_{false};

        I2CDevice i2c_device_{};
    };

    template <std::size_t SIZE>
    std::array<std::uint8_t, SIZE> ICM20948::read_bytes(Bank const bank, std::uint8_t const reg_address) const noexcept
    {
        this->select_bank(bank);
        return this->i2c_device_.read_bytes<SIZE>(reg_address);
    }

    template <std::size_t SIZE>
    void ICM20948::write_bytes(Bank const bank,
                               std::uint8_t const reg_address,
                               std::array<std::uint8_t, SIZE> const& bytes) const noexcept
    {
        this->select_bank(bank);
        this->i2c_device_.write_bytes(reg_address, bytes);
    }

}; // namespace ICM20948

#endif // ICM20948_HPP