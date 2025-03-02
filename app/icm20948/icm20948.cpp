#include "icm20948.hpp"
#include "icm20948_config.hpp"
#include "icm20948_registers.hpp"

namespace ICM20948 {

    ICM20948::ICM20948(I2CDevice&& i2c_device,
                       AK09916&& magnetometer,
                       Bank0::Config const& bank0_config,
                       Bank1::Config const& bank1_config,
                       Bank2::Config const& bank2_config,
                       Bank3::Config const& bank3_config) noexcept :
        i2c_device_{std::forward<I2CDevice>(i2c_device)}, magnetometer_{std::forward<AK09916>(magnetometer)}
    {
        //  this->initialize();
    }

    ICM20948::~ICM20948() noexcept
    {
        this->deinitialize();
    }

    std::optional<float> ICM20948::get_acceleration_x_scaled() const noexcept
    {
        return this->get_acceleration_x_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->accel_scale_; });
    }

    std::optional<float> ICM20948::get_acceleration_y_scaled() const noexcept
    {
        return this->get_acceleration_y_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->accel_scale_; });
    }

    std::optional<float> ICM20948::get_acceleration_z_scaled() const noexcept
    {
        return this->get_acceleration_z_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->accel_scale_; });
    }

    std::optional<Vec3D<float>> ICM20948::get_acceleration_scaled() const noexcept
    {
        return this->get_acceleration_raw().transform(
            [this](Vec3D<std::int16_t> const raw) { return static_cast<Vec3D<float>>(raw) / this->accel_scale_; });
    }

    std::optional<float> ICM20948::get_rotation_x_scaled() const noexcept
    {
        return this->get_rotation_x_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->gyro_scale_; });
    }

    std::optional<float> ICM20948::get_rotation_y_scaled() const noexcept
    {
        return this->get_rotation_y_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->gyro_scale_; });
    }

    std::optional<float> ICM20948::get_rotation_z_scaled() const noexcept
    {
        return this->get_rotation_z_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->gyro_scale_; });
    }

    std::optional<Vec3D<float>> ICM20948::get_rotation_scaled() const noexcept
    {
        return this->get_rotation_raw().transform(
            [this](Vec3D<std::int16_t> const raw) { return static_cast<Vec3D<float>>(raw) / this->gyro_scale_; });
    }

    std::optional<float> ICM20948::get_magnetic_field_x_scaled() const noexcept
    {
        return this->magnetometer_.get_magnetic_field_x_scaled();
    }

    std::optional<float> ICM20948::get_magnetic_field_y_scaled() const noexcept
    {
        return this->magnetometer_.get_magnetic_field_y_scaled();
    }

    std::optional<float> ICM20948::get_magnetic_field_z_scaled() const noexcept
    {
        return this->magnetometer_.get_magnetic_field_z_scaled();
    }

    std::optional<Vec3D<float>> ICM20948::get_magnetic_field_scaled() const noexcept
    {
        return this->magnetometer_.get_magnetic_field_scaled();
    }

    void ICM20948::initialize(Bank0::Config const& bank0_config,
                              Bank1::Config const& bank1_config,
                              Bank2::Config const& bank2_config,
                              Bank3::Config const& bank3_config) noexcept
    {
        if (this->is_valid_device_id()) {
            this->device_reset();

            this->initialize_bank(bank0_config);
            this->initialize_bank(bank1_config);
            this->initialize_bank(bank2_config);
            this->initialize_bank(bank3_config);

            this->initialized_ = true;
        }
    }

    void ICM20948::deinitialize() noexcept
    {
        if (this->is_valid_device_id()) {
            this->device_reset();

            this->initialized_ = false;
        }
    }

    void ICM20948::initialize_bank(Bank0::Config const& config) const noexcept
    {
        this->set_user_ctrl_register(config.user_ctrl);
        this->set_lp_config_register(config.lp_config);
        this->set_pwr_mgmt_1_register(config.pwr_mgmt_1);
        this->set_pwr_mgmt_2_register(config.pwr_mgmt_2);
        this->set_int_pin_cfg_register(config.int_pin_cfg);
        this->set_int_enable_register(config.int_enable);
    }

    void ICM20948::initialize_bank(Bank1::Config const& config) const noexcept
    {}

    void ICM20948::initialize_bank(Bank2::Config const& config) const noexcept
    {
        this->set_gyro_smplrt_div_register(config.gyro_smplrt_div);
        this->set_gyro_config_1_register(config.gyro_config_1);
        this->set_accel_smplrt_div_registers(config.accel_smplrt_div);
        this->set_accel_config_1_register(config.accel_config_1);
        this->set_accel_config_2_register(config.accel_config_2);
    }

    void ICM20948::initialize_bank(Bank3::Config const& config) const noexcept
    {}

    std::uint8_t ICM20948::get_device_id() const noexcept
    {
        return std::bit_cast<std::uint8_t>(this->get_who_am_i_register());
    }

    bool ICM20948::is_valid_device_id() const noexcept
    {
        return this->get_device_id() == this->i2c_device_.dev_address();
    }

    void ICM20948::device_wake_up() const noexcept
    {
        this->set_pwr_mgmt_1_register(PWR_MGMT_1{0U});
        HAL_Delay(200UL);
    }

    void ICM20948::device_reset() const noexcept
    {
        auto pwr_mgmt_1 = this->get_pwr_mgmt_1_register();
        pwr_mgmt_1.device_reset = true;
        this->set_pwr_mgmt_1_register(pwr_mgmt_1);
        HAL_Delay(200UL);
    }

    std::optional<std::int16_t> ICM20948::get_acceleration_x_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::bit_cast<std::int16_t>(this->get_accel_xout_registers())}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<std::int16_t> ICM20948::get_acceleration_y_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::bit_cast<std::int16_t>(this->get_accel_yout_registers())}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<std::int16_t> ICM20948::get_acceleration_z_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::bit_cast<std::int16_t>(this->get_accel_zout_registers())}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<Vec3D<std::int16_t>> ICM20948::get_acceleration_raw() const noexcept
    {
        auto const accel_out = this->get_accel_out_registers();
        return this->initialized_
                   ? std ::optional<Vec3D<std::int16_t>>{std::in_place,
                                                         std::bit_cast<std::int16_t>(accel_out.accel_xout),
                                                         std::bit_cast<std::int16_t>(accel_out.accel_yout),
                                                         std::bit_cast<std::int16_t>(accel_out.accel_zout)}
                   : std ::optional<Vec3D<std::int16_t>>{std::nullopt};
    }

    std::optional<std::int16_t> ICM20948::get_rotation_x_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::bit_cast<std::int16_t>(this->get_gyro_xout_registers())}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<std::int16_t> ICM20948::get_rotation_y_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::bit_cast<std::int16_t>(this->get_gyro_yout_registers())}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<std::int16_t> ICM20948::get_rotation_z_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::bit_cast<std::int16_t>(this->get_gyro_zout_registers())}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<Vec3D<std::int16_t>> ICM20948::get_rotation_raw() const noexcept
    {
        auto const gyro_out = this->get_gyro_out_registers();
        return this->initialized_ ? std ::optional<Vec3D<std::int16_t>>{std::in_place,
                                                                        std::bit_cast<std::int16_t>(gyro_out.gyro_xout),
                                                                        std::bit_cast<std::int16_t>(gyro_out.gyro_yout),
                                                                        std::bit_cast<std::int16_t>(gyro_out.gyro_zout)}
                                  : std ::optional<Vec3D<std::int16_t>>{std::nullopt};
    }

    std::uint8_t ICM20948::read_byte(Bank const bank, std::uint8_t const reg_address) const noexcept
    {
        this->select_bank(bank);
        return this->i2c_device_.read_byte(reg_address);
    }

    void ICM20948::write_byte(Bank const bank, std::uint8_t const reg_address, std::uint8_t const byte) const noexcept
    {
        this->select_bank(bank);
        this->i2c_device_.write_byte(reg_address, byte);
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
    {
        return std::bit_cast<Bank0::WHO_AM_I>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::WHO_AM_I)));
    }

    Bank0::USER_CTRL ICM20948::get_user_ctrl_register() const noexcept
    {
        return std::bit_cast<Bank0::USER_CTRL>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::USER_CTRL)));
    }

    void ICM20948::set_user_ctrl_register(Bank0::USER_CTRL const user_ctrl) const noexcept
    {
        this->write_byte(Bank::USER_BANK_0,
                         std::to_underlying(Bank0::RA::USER_CTRL),
                         std::bit_cast<std::uint8_t>(user_ctrl));
    }

    Bank0::LP_CONFIG ICM20948::get_lp_config_register() const noexcept
    {
        return std::bit_cast<Bank0::LP_CONFIG>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::LP_CONFIG)));
    }

    void ICM20948::set_lp_config_register(Bank0::LP_CONFIG const lp_config) const noexcept
    {
        this->write_byte(Bank::USER_BANK_0,
                         std::to_underlying(Bank0::RA::LP_CONFIG),
                         std::bit_cast<std::uint8_t>(lp_config));
    }

    Bank0::PWR_MGMT_1 ICM20948::get_pwr_mgmt_1_register() const noexcept
    {
        return std::bit_cast<Bank0::PWR_MGMT_1>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::PWR_MGMT_1)));
    }

    void ICM20948::set_pwr_mgmt_1_register(Bank0::PWR_MGMT_1 const pwr_mgmt_1) const noexcept
    {
        this->write_byte(Bank::USER_BANK_0,
                         std::to_underlying(Bank0::RA::PWR_MGMT_1),
                         std::bit_cast<std::uint8_t>(pwr_mgmt_1));
    }

    Bank0::PWR_MGMT_2 ICM20948::get_pwr_mgmt_2_register() const noexcept
    {
        return std::bit_cast<Bank0::PWR_MGMT_2>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::PWR_MGMT_2)));
    }

    void ICM20948::set_pwr_mgmt_2_register(Bank0::PWR_MGMT_2 const pwr_mgmt_2) const noexcept
    {
        this->write_byte(Bank::USER_BANK_0,
                         std::to_underlying(Bank0::RA::PWR_MGMT_2),
                         std::bit_cast<std::uint8_t>(pwr_mgmt_2));
    }

    Bank0::INT_PIN_CFG ICM20948::get_int_pin_cfg_register() const noexcept
    {
        return std::bit_cast<Bank0::INT_PIN_CFG>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::INT_PIN_CFG)));
    }

    void ICM20948::set_int_pin_cfg_register(Bank0::INT_PIN_CFG const int_pin_cfg) const noexcept
    {
        this->write_byte(Bank::USER_BANK_0,
                         std::to_underlying(Bank0::RA::INT_PIN_CFG),
                         std::bit_cast<std::uint8_t>(int_pin_cfg));
    }

    Bank0::INT_ENABLE ICM20948::get_int_enable_register() const noexcept
    {
        return std::bit_cast<Bank0::INT_ENABLE>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::INT_ENABLE)));
    }

    void ICM20948::set_int_enable_register(Bank0::INT_ENABLE const int_enable) const noexcept
    {
        this->write_byte(Bank::USER_BANK_0,
                         std::to_underlying(Bank0::RA::INT_ENABLE),
                         std::bit_cast<std::uint8_t>(int_enable));
    }

    Bank0::INT_ENABLE_1 ICM20948::get_int_enable_1_register() const noexcept
    {
        return std::bit_cast<Bank0::INT_ENABLE_1>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::INT_ENABLE_1)));
    }

    void ICM20948::set_int_enable_1_register(Bank0::INT_ENABLE_1 const int_enable_1) const noexcept
    {
        this->write_byte(Bank::USER_BANK_0,
                         std::to_underlying(Bank0::RA::INT_ENABLE_1),
                         std::bit_cast<std::uint8_t>(int_enable_1));
    }

    Bank0::INT_ENABLE_2 ICM20948::get_int_enable_2_register() const noexcept
    {
        return std::bit_cast<Bank0::INT_ENABLE_2>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::INT_ENABLE_2)));
    }

    void ICM20948::set_int_enable_2_register(Bank0::INT_ENABLE_2 const int_enable_2) const noexcept
    {
        this->write_byte(Bank::USER_BANK_0,
                         std::to_underlying(Bank0::RA::INT_ENABLE_2),
                         std::bit_cast<std::uint8_t>(int_enable_2));
    }

    Bank0::INT_ENABLE_3 ICM20948::get_int_enable_3_register() const noexcept
    {
        return std::bit_cast<Bank0::INT_ENABLE_3>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::INT_ENABLE_3)));
    }

    void ICM20948::set_int_enable_3_register(Bank0::INT_ENABLE_3 const int_enable_3) const noexcept
    {
        this->write_byte(Bank::USER_BANK_0,
                         std::to_underlying(Bank0::RA::INT_ENABLE_3),
                         std::bit_cast<std::uint8_t>(int_enable_3));
    }

    Bank0::I2C_MST_STATUS ICM20948::get_i2c_mst_status_register() const noexcept
    {
        return std::bit_cast<Bank0::I2C_MST_STATUS>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::I2C_MST_STATUS)));
    }

    Bank0::INT_STATUS ICM20948::get_int_status_register() const noexcept
    {
        return std::bit_cast<Bank0::INT_STATUS>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::INT_STATUS)));
    }

    Bank0::INT_STATUS_1 ICM20948::get_int_status_1_register() const noexcept
    {
        return std::bit_cast<Bank0::INT_STATUS_1>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::INT_STATUS_1)));
    }

    Bank0::INT_STATUS_2 ICM20948::get_int_status_2_register() const noexcept
    {
        return std::bit_cast<Bank0::INT_STATUS_2>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::INT_STATUS_2)));
    }

    Bank0::INT_STATUS_3 ICM20948::get_int_status_3_register() const noexcept
    {
        return std::bit_cast<Bank0::INT_STATUS_3>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::INT_STATUS_3)));
    }

    Bank0::DELAY_TIME ICM20948::get_delay_time_registers() const noexcept
    {
        return std::bit_cast<Bank0::DELAY_TIME>(
            this->read_bytes<sizeof(Bank0::DELAY_TIME)>(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::DELAY_TIMEH)));
    }

    Bank0::ACCEL_XOUT ICM20948::get_accel_xout_registers() const noexcept
    {
        return std::bit_cast<Bank0::ACCEL_XOUT>(
            this->read_bytes<sizeof(Bank0::ACCEL_XOUT)>(Bank::USER_BANK_0,
                                                        std::to_underlying(Bank0::RA::ACCEL_XOUT_H)));
    }

    Bank0::ACCEL_YOUT ICM20948::get_accel_yout_registers() const noexcept
    {
        return std::bit_cast<Bank0::ACCEL_YOUT>(
            this->read_bytes<sizeof(Bank0::ACCEL_YOUT)>(Bank::USER_BANK_0,
                                                        std::to_underlying(Bank0::RA::ACCEL_YOUT_H)));
    }

    Bank0::ACCEL_ZOUT ICM20948::get_accel_zout_registers() const noexcept
    {
        return std::bit_cast<Bank0::ACCEL_ZOUT>(
            this->read_bytes<sizeof(Bank0::ACCEL_ZOUT)>(Bank::USER_BANK_0,
                                                        std::to_underlying(Bank0::RA::ACCEL_ZOUT_H)));
    }

    Bank0::ACCEL_OUT ICM20948::get_accel_out_registers() const noexcept
    {
        return std::bit_cast<Bank0::ACCEL_OUT>(
            this->read_bytes<sizeof(Bank0::ACCEL_OUT)>(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::ACCEL_XOUT_H)));
    }

    Bank0::GYRO_XOUT ICM20948::get_gyro_xout_registers() const noexcept
    {
        return std::bit_cast<Bank0::GYRO_XOUT>(
            this->read_bytes<sizeof(Bank0::GYRO_XOUT)>(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::GYRO_XOUT_H)));
    }

    Bank0::GYRO_YOUT ICM20948::get_gyro_yout_registers() const noexcept
    {
        return std::bit_cast<Bank0::GYRO_YOUT>(
            this->read_bytes<sizeof(Bank0::GYRO_YOUT)>(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::GYRO_YOUT_H)));
    }

    Bank0::GYRO_ZOUT ICM20948::get_gyro_zout_registers() const noexcept
    {
        return std::bit_cast<Bank0::GYRO_ZOUT>(
            this->read_bytes<sizeof(Bank0::GYRO_ZOUT)>(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::GYRO_ZOUT_H)));
    }

    Bank0::GYRO_OUT ICM20948::get_gyro_out_registers() const noexcept
    {
        return std::bit_cast<Bank0::GYRO_OUT>(
            this->read_bytes<sizeof(Bank0::GYRO_OUT)>(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::GYRO_XOUT_H)));
    }

    Bank0::TEMP_OUT ICM20948::get_temp_out_registers() const noexcept
    {
        return std::bit_cast<Bank0::TEMP_OUT>(
            this->read_bytes<sizeof(Bank0::TEMP_OUT)>(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::TEMP_OUT_H)));
    }

    Bank0::EXT_SLV_SENS_DATA ICM20948::get_ext_slv_sens_data_register(std::uint8_t const num) const noexcept
    {
        return std::bit_cast<Bank0::EXT_SLV_SENS_DATA>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::EXT_SLV_SENS_DATA_00)));
    }

    Bank0::FIFO_EN_1 ICM20948::get_fifo_en_1_register() const noexcept
    {
        return std::bit_cast<Bank0::FIFO_EN_1>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::FIFO_EN_1)));
    }

    void ICM20948::set_fifo_en_1_register(Bank0::FIFO_EN_1 const fifo_en_1) const noexcept
    {
        this->write_byte(Bank::USER_BANK_0,
                         std::to_underlying(Bank0::RA::FIFO_EN_1),
                         std::bit_cast<std::uint8_t>(fifo_en_1));
    }

    Bank0::FIFO_EN_2 ICM20948::get_fifo_en_2_register() const noexcept
    {
        return std::bit_cast<Bank0::FIFO_EN_2>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::FIFO_EN_2)));
    }

    void ICM20948::set_fifo_en_2_register(Bank0::FIFO_EN_2 const fifo_en_2) const noexcept
    {
        this->write_byte(Bank::USER_BANK_0,
                         std::to_underlying(Bank0::RA::FIFO_EN_2),
                         std::bit_cast<std::uint8_t>(fifo_en_2));
    }

    Bank0::FIFO_RST ICM20948::get_fifo_rst_register() const noexcept
    {
        return std::bit_cast<Bank0::FIFO_RST>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::FIFO_RST)));
    }

    void ICM20948::set_fifo_rst_register(Bank0::FIFO_RST const fifo_rst) const noexcept
    {
        this->write_byte(Bank::USER_BANK_0,
                         std::to_underlying(Bank0::RA::FIFO_RST),
                         std::bit_cast<std::uint8_t>(fifo_rst));
    }

    Bank0::FIFO_MODE ICM20948::get_fifo_mode_register() const noexcept
    {
        return std::bit_cast<Bank0::FIFO_MODE>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::FIFO_MODE)));
    }

    void ICM20948::set_fifo_mode_register(Bank0::FIFO_MODE const fifo_mode) const noexcept
    {
        this->write_byte(Bank::USER_BANK_0,
                         std::to_underlying(Bank0::RA::FIFO_MODE),
                         std::bit_cast<std::uint8_t>(fifo_mode));
    }

    Bank0::FIFO_COUNT ICM20948::get_fifo_count_registers() const noexcept
    {
        return std::bit_cast<Bank0::FIFO_COUNT>(
            this->read_bytes<sizeof(Bank0::FIFO_COUNT)>(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::FIFO_COUNTH)));
    }

    Bank0::FIFO_R_W ICM20948::get_fifo_r_w_register() const noexcept
    {
        return std::bit_cast<Bank0::FIFO_R_W>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::FIFO_R_W)));
    }

    void ICM20948::set_fifo_r_w_register(Bank0::FIFO_R_W const fifo_r_w) const noexcept
    {
        this->write_byte(Bank::USER_BANK_0,
                         std::to_underlying(Bank0::RA::FIFO_R_W),
                         std::bit_cast<std::uint8_t>(fifo_r_w));
    }

    Bank0::DATA_RDY_STATUS ICM20948::get_data_rdy_status() const noexcept
    {
        return std::bit_cast<Bank0::DATA_RDY_STATUS>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::DATA_RDY_STATUS)));
    }

    Bank0::FIFO_CFG ICM20948::get_fifo_cfg_register() const noexcept
    {
        return std::bit_cast<Bank0::FIFO_CFG>(
            this->read_byte(Bank::USER_BANK_0, std::to_underlying(Bank0::RA::FIFO_CFG)));
    }

    void ICM20948::set_fifo_cfg_register(Bank0::FIFO_CFG const fifo_cfg) const noexcept
    {
        this->write_byte(Bank::USER_BANK_0,
                         std::to_underlying(Bank0::RA::FIFO_CFG),
                         std::bit_cast<std::uint8_t>(fifo_cfg));
    }

    Bank1::SELF_TEST_X_GYRO ICM20948::get_self_test_x_gyro_register() const noexcept
    {
        return std::bit_cast<Bank1::SELF_TEST_X_GYRO>(
            this->read_byte(Bank::USER_BANK_1, std::to_underlying(Bank1::RA::SELF_TEST_X_GYRO)));
    }

    void ICM20948::set_self_test_x_gyro_register(Bank1::SELF_TEST_X_GYRO const self_text_x_gyro) const noexcept
    {}

    Bank1::SELF_TEST_Y_GYRO ICM20948::get_self_test_y_gyro_register() const noexcept
    {
        return std::bit_cast<Bank1::SELF_TEST_Y_GYRO>(
            this->read_byte(Bank::USER_BANK_1, std::to_underlying(Bank1::RA::SELF_TEST_Y_GYRO)));
    }

    void ICM20948::set_self_test_x_gyro_register(Bank1::SELF_TEST_Y_GYRO const self_text_y_gyro) const noexcept
    {
        this->write_byte(Bank::USER_BANK_1,
                         std::to_underlying(Bank1::RA::SELF_TEST_Y_ACCEL),
                         std::bit_cast<std::uint8_t>(self_text_y_gyro));
    }

    Bank1::SELF_TEST_Z_GYRO ICM20948::get_self_test_z_gyro_register() const noexcept
    {
        return std::bit_cast<Bank1::SELF_TEST_Z_GYRO>(
            this->read_byte(Bank::USER_BANK_1, std::to_underlying(Bank1::RA::SELF_TEST_Z_GYRO)));
    }

    void ICM20948::set_self_test_x_gyro_register(Bank1::SELF_TEST_Z_GYRO const self_text_z_gyro) const noexcept
    {
        this->write_byte(Bank::USER_BANK_1,
                         std::to_underlying(Bank1::RA::SELF_TEST_Z_GYRO),
                         std::bit_cast<std::uint8_t>(self_text_z_gyro));
    }

    Bank1::SELF_TEST_X_ACCEL ICM20948::get_self_test_x_accel_register() const noexcept
    {
        return std::bit_cast<Bank1::SELF_TEST_X_ACCEL>(
            this->read_byte(Bank::USER_BANK_1, std::to_underlying(Bank1::RA::SELF_TEST_X_ACCEL)));
    }

    void ICM20948::set_self_test_x_accel_register(Bank1::SELF_TEST_X_ACCEL const self_text_x_accel) const noexcept
    {
        this->write_byte(Bank::USER_BANK_1,
                         std::to_underlying(Bank1::RA::SELF_TEST_X_ACCEL),
                         std::bit_cast<std::uint8_t>(self_text_x_accel));
    }

    Bank1::SELF_TEST_Y_ACCEL ICM20948::get_self_test_y_accel_register() const noexcept
    {
        return std::bit_cast<Bank1::SELF_TEST_Y_ACCEL>(
            this->read_byte(Bank::USER_BANK_1, std::to_underlying(Bank1::RA::SELF_TEST_Y_ACCEL)));
    }

    void ICM20948::set_self_test_y_accel_register(Bank1::SELF_TEST_Y_ACCEL const self_text_y_accel) const noexcept
    {
        this->write_byte(Bank::USER_BANK_1,
                         std::to_underlying(Bank1::RA::SELF_TEST_Y_ACCEL),
                         std::bit_cast<std::uint8_t>(self_text_y_accel));
    }

    Bank1::SELF_TEST_Z_ACCEL ICM20948::get_self_test_z_accel_register() const noexcept
    {
        return std::bit_cast<Bank1::SELF_TEST_Z_ACCEL>(
            this->read_byte(Bank::USER_BANK_1, std::to_underlying(Bank1::RA::SELF_TEST_Z_ACCEL)));
    }

    void ICM20948::set_self_test_z_accel_register(Bank1::SELF_TEST_Z_ACCEL const self_text_z_accel) const noexcept
    {
        this->write_byte(Bank::USER_BANK_1,
                         std::to_underlying(Bank1::RA::SELF_TEST_Z_ACCEL),
                         std::bit_cast<std::uint8_t>(self_text_z_accel));
    }

    Bank1::XA_OFFS ICM20948::get_xa_offs_registers() const noexcept
    {
        return std::bit_cast<Bank1::XA_OFFS>(
            this->read_bytes<sizeof(Bank1::XA_OFFS)>(Bank::USER_BANK_1, std::to_underlying(Bank1::RA::XA_OFFS_H)));
    }

    void ICM20948::set_xa_offs_registers(Bank1::XA_OFFS const xa_offs) const noexcept
    {
        this->write_bytes(Bank::USER_BANK_1,
                          std::to_underlying(Bank1::RA::XA_OFFS_H),
                          std::bit_cast<std::array<std::uint8_t, sizeof(Bank1::XA_OFFS)>>(xa_offs));
    }

    Bank1::YA_OFFS ICM20948::get_ya_offs_registers() const noexcept
    {
        return std::bit_cast<Bank1::YA_OFFS>(
            this->read_bytes<sizeof(Bank1::YA_OFFS)>(Bank::USER_BANK_1, std::to_underlying(Bank1::RA::YA_OFFS_H)));
    }

    void ICM20948::set_ya_offs_registers(Bank1::YA_OFFS const ya_offs) const noexcept
    {
        this->write_bytes(Bank::USER_BANK_1,
                          std::to_underlying(Bank1::RA::YA_OFFS_H),
                          std::bit_cast<std::array<std::uint8_t, sizeof(Bank1::YA_OFFS)>>(ya_offs));
    }

    Bank1::ZA_OFFS ICM20948::get_za_offs_registers() const noexcept
    {
        return std::bit_cast<Bank1::ZA_OFFS>(
            this->read_bytes<sizeof(Bank1::ZA_OFFS)>(Bank::USER_BANK_1, std::to_underlying(Bank1::RA::ZA_OFFS_H)));
    }

    void ICM20948::set_za_offs_registers(Bank1::ZA_OFFS const za_offs) const noexcept
    {
        this->write_bytes(Bank::USER_BANK_1,
                          std::to_underlying(Bank1::RA::ZA_OFFS_H),
                          std::bit_cast<std::array<std::uint8_t, sizeof(Bank1::ZA_OFFS)>>(za_offs));
    }

    Bank1::TIMEBASE_CORRECTION_PLL ICM20948::get_timebase_correction_pll_register() const noexcept
    {
        return std::bit_cast<Bank1::TIMEBASE_CORRECTION_PLL>(
            this->read_byte(Bank::USER_BANK_1, std::to_underlying(Bank1::RA::TIMEBASE_CORRECTION_PLL)));
    }

    void
    ICM20948::set_timebase_correction_pll(Bank1::TIMEBASE_CORRECTION_PLL const timebase_correction_pll) const noexcept
    {
        this->write_byte(Bank::USER_BANK_1,
                         std::to_underlying(Bank1::RA::TIMEBASE_CORRECTION_PLL),
                         std::bit_cast<std::uint8_t>(timebase_correction_pll));
    }

    Bank2::GYRO_SMPLRT_DIV ICM20948::get_gyro_smplrt_div_register() const noexcept
    {
        return std::bit_cast<Bank2::GYRO_SMPLRT_DIV>(
            this->read_byte(Bank::USER_BANK_2, std::to_underlying(Bank2::RA::GYRO_SMPLRT_DIV)));
    }

    void ICM20948::set_gyro_smplrt_div_register(Bank2::GYRO_SMPLRT_DIV const gyro_smplrt_div) const noexcept
    {
        this->write_byte(Bank::USER_BANK_2,
                         std::to_underlying(Bank2::RA::GYRO_SMPLRT_DIV),
                         std::bit_cast<std::uint8_t>(gyro_smplrt_div));
    }

    Bank2::GYRO_CONFIG_1 ICM20948::get_gyro_config_1_register() const noexcept
    {
        return std::bit_cast<Bank2::GYRO_CONFIG_1>(
            this->read_byte(Bank::USER_BANK_2, std::to_underlying(Bank2::RA::GYRO_CONFIG_1)));
    }

    void ICM20948::set_gyro_config_1_register(Bank2::GYRO_CONFIG_1 const gyro_config_1) const noexcept
    {
        this->write_byte(Bank::USER_BANK_2,
                         std::to_underlying(Bank2::RA::GYRO_CONFIG_1),
                         std::bit_cast<std::uint8_t>(gyro_config_1));
    }

    Bank2::GYRO_CONFIG_2 ICM20948::get_gyro_config_2_register() const noexcept
    {
        return std::bit_cast<Bank2::GYRO_CONFIG_2>(
            this->read_byte(Bank::USER_BANK_2, std::to_underlying(Bank2::RA::GYRO_CONFIG_2)));
    }

    void ICM20948::set_gyro_config_2_register(Bank2::GYRO_CONFIG_2 const gyro_config_2) const noexcept
    {
        this->write_byte(Bank::USER_BANK_2,
                         std::to_underlying(Bank2::RA::GYRO_CONFIG_2),
                         std::bit_cast<std::uint8_t>(gyro_config_2));
    }

    Bank2::XG_OFFS_USR ICM20948::get_xg_offs_usr_registers() const noexcept
    {
        return std::bit_cast<Bank2::XG_OFFS_USR>(
            this->read_bytes<sizeof(Bank2::XG_OFFS_USR)>(Bank::USER_BANK_2,
                                                         std::to_underlying(Bank2::RA::XG_OFFS_USRH)));
    }

    void ICM20948::set_xg_offs_usr_registers(Bank2::XG_OFFS_USR const xg_offs_usr) const noexcept
    {
        this->write_bytes(Bank::USER_BANK_2,
                          std::to_underlying(Bank2::RA::XG_OFFS_USRH),
                          std::bit_cast<std::array<std::uint8_t, sizeof(Bank2::XG_OFFS_USR)>>(xg_offs_usr));
    }

    Bank2::YG_OFFS_USR ICM20948::get_yg_offs_usr_registers() const noexcept
    {
        return std::bit_cast<Bank2::YG_OFFS_USR>(
            this->read_bytes<sizeof(Bank2::YG_OFFS_USR)>(Bank::USER_BANK_2,
                                                         std::to_underlying(Bank2::RA::YG_OFFS_USRH)));
    }

    void ICM20948::set_yg_offs_usr_registers(Bank2::YG_OFFS_USR const yg_offs_usr) const noexcept
    {
        this->write_bytes(Bank::USER_BANK_2,
                          std::to_underlying(Bank2::RA::YG_OFFS_USRH),
                          std::bit_cast<std::array<std::uint8_t, sizeof(Bank2::YG_OFFS_USR)>>(yg_offs_usr));
    }

    Bank2::ZG_OFFS_USR ICM20948::get_zg_offs_usr_registers() const noexcept
    {
        return std::bit_cast<Bank2::ZG_OFFS_USR>(
            this->read_bytes<sizeof(Bank2::ZG_OFFS_USR)>(Bank::USER_BANK_2,
                                                         std::to_underlying(Bank2::RA::ZG_OFFS_USRH)));
    }

    void ICM20948::set_zg_offs_usr_registers(Bank2::ZG_OFFS_USR const zg_offs_usr) const noexcept
    {
        this->write_bytes(Bank::USER_BANK_2,
                          std::to_underlying(Bank2::RA::ZG_OFFS_USRH),
                          std::bit_cast<std::array<std::uint8_t, sizeof(Bank2::XG_OFFS_USR)>>(zg_offs_usr));
    }

    Bank2::ODR_ALIGN_EN ICM20948::get_odr_align_en_register() const noexcept
    {
        return std::bit_cast<Bank2::ODR_ALIGN_EN>(
            this->read_byte(Bank::USER_BANK_2, std::to_underlying(Bank2::RA::ODR_ALIGN_EN)));
    }

    void ICM20948::set_odr_align_en_register(Bank2::ODR_ALIGN_EN const odr_align_en) const noexcept
    {
        this->write_byte(Bank::USER_BANK_2,
                         std::to_underlying(Bank2::RA::ODR_ALIGN_EN),
                         std::bit_cast<std::uint8_t>(odr_align_en));
    }

    Bank2::ACCEL_SMPLRT_DIV ICM20948::get_accel_smplrt_div_registers() const noexcept
    {
        return std::bit_cast<Bank2::ACCEL_SMPLRT_DIV>(
            this->read_bytes<sizeof(Bank2::ACCEL_SMPLRT_DIV)>(Bank::USER_BANK_2,
                                                              std::to_underlying(Bank2::RA::ACCEL_SMPLRT_DIV_1)));
    }

    void ICM20948::set_accel_smplrt_div_registers(Bank2::ACCEL_SMPLRT_DIV const accel_smplrt_div) const noexcept
    {
        this->write_bytes(Bank::USER_BANK_2,
                          std::to_underlying(Bank2::RA::ACCEL_SMPLRT_DIV_1),
                          std::bit_cast<std::array<std::uint8_t, sizeof(Bank2::ACCEL_SMPLRT_DIV)>>(accel_smplrt_div));
    }

    Bank2::ACCEL_INTEL_CTRL ICM20948::get_accel_intel_ctrl_register() const noexcept
    {
        return std::bit_cast<Bank2::ACCEL_INTEL_CTRL>(
            this->read_byte(Bank::USER_BANK_2, std::to_underlying(Bank2::RA::ACCEL_INTEL_CTRL)));
    }

    void ICM20948::set_accel_intel_ctrl_register(Bank2::ACCEL_INTEL_CTRL const accel_intel_ctrl) const noexcept
    {
        this->write_byte(Bank::USER_BANK_2,
                         std::to_underlying(Bank2::RA::ACCEL_INTEL_CTRL),
                         std::bit_cast<std::uint8_t>(accel_intel_ctrl));
    }

    Bank2::ACCEL_WOM_THR ICM20948::get_accel_wom_thr_register() const noexcept
    {
        return std::bit_cast<Bank2::ACCEL_WOM_THR>(
            this->read_byte(Bank::USER_BANK_2, std::to_underlying(Bank2::RA::ACCEL_WOM_THR)));
    }

    void ICM20948::set_accel_wom_thr_register(Bank2::ACCEL_WOM_THR const accel_wom_thr) const noexcept
    {
        this->write_byte(Bank::USER_BANK_2,
                         std::to_underlying(Bank2::RA::ACCEL_WOM_THR),
                         std::bit_cast<std::uint8_t>(accel_wom_thr));
    }

    Bank2::ACCEL_CONFIG_1 ICM20948::get_accel_config_1_register() const noexcept
    {
        return std::bit_cast<Bank2::ACCEL_CONFIG_1>(
            this->read_byte(Bank::USER_BANK_2, std::to_underlying(Bank2::RA::ACCEL_CONFIG_1)));
    }

    void ICM20948::set_accel_config_1_register(Bank2::ACCEL_CONFIG_1 const accel_config_1) const noexcept
    {
        this->write_byte(Bank::USER_BANK_2,
                         std::to_underlying(Bank2::RA::ACCEL_CONFIG_1),
                         std::bit_cast<std::uint8_t>(accel_config_1));
    }

    Bank2::ACCEL_CONFIG_2 ICM20948::get_accel_config_2_register() const noexcept
    {
        return std::bit_cast<Bank2::ACCEL_CONFIG_2>(
            this->read_byte(Bank::USER_BANK_2, std::to_underlying(Bank2::RA::ACCEL_CONFIG_2)));
    }

    void ICM20948::set_accel_config_2_register(Bank2::ACCEL_CONFIG_2 const accel_config_2) const noexcept
    {
        this->write_byte(Bank::USER_BANK_2,
                         std::to_underlying(Bank2::RA::ACCEL_CONFIG_2),
                         std::bit_cast<std::uint8_t>(accel_config_2));
    }

    Bank2::FSYNC_CONFIG ICM20948::get_fsync_config_register() const noexcept
    {
        return std::bit_cast<Bank2::FSYNC_CONFIG>(
            this->read_byte(Bank::USER_BANK_2, std::to_underlying(Bank2::RA::FSYNC_CONFIG)));
    }

    void ICM20948::set_fsync_config_register(Bank2::FSYNC_CONFIG const fsync_config) const noexcept
    {
        this->write_byte(Bank::USER_BANK_2,
                         std::to_underlying(Bank2::RA::FSYNC_CONFIG),
                         std::bit_cast<std::uint8_t>(fsync_config));
    }

    Bank2::TEMP_CONFIG ICM20948::get_temp_config_register() const noexcept
    {
        return std::bit_cast<Bank2::TEMP_CONFIG>(
            this->read_byte(Bank::USER_BANK_2, std::to_underlying(Bank2::RA::TEMP_CONFIG)));
    }

    void ICM20948::set_temp_config_register(Bank2::TEMP_CONFIG const temp_config) const noexcept
    {
        this->write_byte(Bank::USER_BANK_2,
                         std::to_underlying(Bank2::RA::TEMP_CONFIG),
                         std::bit_cast<std::uint8_t>(temp_config));
    }

    Bank2::MOD_CTRL_USR ICM20948::get_mod_ctrl_usr_register() const noexcept
    {
        return std::bit_cast<Bank2::MOD_CTRL_USR>(
            this->read_byte(Bank::USER_BANK_2, std::to_underlying(Bank2::RA::MOD_CTRL_USR)));
    }

    void ICM20948::set_mod_ctrl_usr_register(Bank2::MOD_CTRL_USR const mod_ctrl_usr) const noexcept
    {
        this->write_byte(Bank::USER_BANK_2,
                         std::to_underlying(Bank2::RA::MOD_CTRL_USR),
                         std::bit_cast<std::uint8_t>(mod_ctrl_usr));
    }

    Bank3::I2C_MST_ODR_CONFIG ICM20948::get_i2c_mst_odr_config_register() const noexcept
    {
        return std::bit_cast<Bank3::I2C_MST_ODR_CONFIG>(
            this->read_byte(Bank::USER_BANK_3, std::to_underlying(Bank3::RA::I2C_MST_ODR_CONFIG)));
    }

    void ICM20948::set_i2c_mst_odr_config_register(Bank3::I2C_MST_ODR_CONFIG const i2c_mst_odr_config) const noexcept
    {
        this->write_byte(Bank::USER_BANK_3,
                         std::to_underlying(Bank3::RA::I2C_MST_ODR_CONFIG),
                         std::bit_cast<std::uint8_t>(i2c_mst_odr_config));
    }

    Bank3::I2C_MST_CTRL ICM20948::get_i2c_mst_ctrl_register() const noexcept
    {
        return std::bit_cast<Bank3::I2C_MST_CTRL>(
            this->read_byte(Bank::USER_BANK_3, std::to_underlying(Bank3::RA::I2C_MST_CTRL)));
    }

    void ICM20948::set_i2c_mst_ctrl_register(Bank3::I2C_MST_CTRL const i2c_mst_ctrl) const noexcept
    {
        this->write_byte(Bank::USER_BANK_3,
                         std::to_underlying(Bank3::RA::I2C_MST_CTRL),
                         std::bit_cast<std::uint8_t>(i2c_mst_ctrl));
    }

    Bank3::I2C_MST_DELAY_CTRL ICM20948::get_i2c_mst_delay_ctrl_register() const noexcept
    {
        return std::bit_cast<Bank3::I2C_MST_DELAY_CTRL>(
            this->read_byte(Bank::USER_BANK_3, std::to_underlying(Bank3::RA::I2C_MST_DELAY_CTRL)));
    }

    void ICM20948::set_i2c_mst_delay_ctrl_register(Bank3::I2C_MST_DELAY_CTRL const i2c_mst_delay_ctrl) const noexcept
    {
        this->write_byte(Bank::USER_BANK_3,
                         std::to_underlying(Bank3::RA::I2C_MST_DELAY_CTRL),
                         std::bit_cast<std::uint8_t>(i2c_mst_delay_ctrl));
    }

    Bank3::I2C_SLV_ADDR ICM20948::get_i2c_slv_addr_register(SlaveNum const slave_num) const noexcept
    {
        return std::bit_cast<Bank3::I2C_SLV_ADDR>(
            this->read_byte(Bank::USER_BANK_3,
                            std::to_underlying(Bank3::RA::I2C_SLV0_ADDR) + 3U * std::to_underlying(slave_num)));
    }

    void ICM20948::set_i2c_slv_addr_register(SlaveNum const slave_num,
                                             Bank3::I2C_SLV_ADDR const i2c_slv_addr) const noexcept
    {
        this->write_byte(Bank::USER_BANK_3,
                         std::to_underlying(Bank3::RA::I2C_SLV0_ADDR) + 3U * std::to_underlying(slave_num),
                         std::bit_cast<std::uint8_t>(i2c_slv_addr));
    }

    Bank3::I2C_SLV_REG ICM20948::get_i2c_slv_reg_register(SlaveNum const slave_num) const noexcept
    {
        return std::bit_cast<Bank3::I2C_SLV_REG>(
            this->read_byte(Bank::USER_BANK_3,
                            std::to_underlying(Bank3::RA::I2C_SLV0_REG) + 3U * std::to_underlying(slave_num)));
    }

    void ICM20948::set_i2c_slv_reg_register(SlaveNum const slave_num,
                                            Bank3::I2C_SLV_REG const i2c_slv_reg) const noexcept
    {
        this->write_byte(Bank::USER_BANK_3,
                         std::to_underlying(Bank3::RA::I2C_SLV0_REG) + 3U * std::to_underlying(slave_num),
                         std::bit_cast<std::uint8_t>(i2c_slv_reg));
    }

    Bank3::I2C_SLV_CTRL ICM20948::get_i2c_slv_ctrl_register(SlaveNum const slave_num) const noexcept
    {
        return std::bit_cast<Bank3::I2C_SLV_CTRL>(
            this->read_byte(Bank::USER_BANK_3,
                            std::to_underlying(Bank3::RA::I2C_SLV0_CTRL) + 3U * std::to_underlying(slave_num)));
    }

    void ICM20948::set_i2c_slv_ctrl_register(SlaveNum const slave_num,
                                             Bank3::I2C_SLV_CTRL const i2c_slv_ctrl) const noexcept
    {
        this->write_byte(Bank::USER_BANK_3,
                         std::to_underlying(Bank3::RA::I2C_SLV0_CTRL) + 3U * std::to_underlying(slave_num),
                         std::bit_cast<std::uint8_t>(i2c_slv_ctrl));
    }

    Bank3::I2C_SLV_DO ICM20948::get_i2c_slv_do_register(SlaveNum const slave_num) const noexcept
    {
        return std::bit_cast<Bank3::I2C_SLV_DO>(
            this->read_byte(Bank::USER_BANK_3,
                            std::to_underlying(Bank3::RA::I2C_SLV0_DO) + 3U * std::to_underlying(slave_num)));
    }

    void ICM20948::set_i2c_slv_do_register(SlaveNum const slave_num, Bank3::I2C_SLV_DO const i2c_slv_do) const noexcept
    {
        this->write_byte(Bank::USER_BANK_3,
                         std::to_underlying(Bank3::RA::I2C_SLV0_DO) + 3U * std::to_underlying(slave_num),
                         std::bit_cast<std::uint8_t>(i2c_slv_do));
    }

    Bank3::I2C_SLV4_ADDR ICM20948::get_i2c_slv4_addr_register() const noexcept
    {
        return std::bit_cast<Bank3::I2C_SLV4_ADDR>(
            this->read_byte(Bank::USER_BANK_3, std::to_underlying(Bank3::RA::I2C_SLV4_ADDR)));
    }

    void ICM20948::set_i2c_slv4_addr_register(Bank3::I2C_SLV4_ADDR const i2c_slv4_addr) const noexcept
    {
        this->write_byte(Bank::USER_BANK_3,
                         std::to_underlying(Bank3::RA::I2C_SLV4_ADDR),
                         std::bit_cast<std::uint8_t>(i2c_slv4_addr));
    }

    Bank3::I2C_SLV4_REG ICM20948::get_i2c_slv4_reg_register() const noexcept
    {
        return std::bit_cast<Bank3::I2C_SLV4_REG>(
            this->read_byte(Bank::USER_BANK_3, std::to_underlying(Bank3::RA::I2C_SLV4_REG)));
    }

    void ICM20948::set_i2c_slv4_reg_register(Bank3::I2C_SLV4_REG const i2c_slv4_reg) const noexcept
    {
        this->write_byte(Bank::USER_BANK_3,
                         std::to_underlying(Bank3::RA::I2C_SLV4_REG),
                         std::bit_cast<std::uint8_t>(i2c_slv4_reg));
    }

    Bank3::I2C_SLV4_CTRL ICM20948::get_i2c_slv4_ctrl_register() const noexcept
    {
        return std::bit_cast<Bank3::I2C_SLV4_CTRL>(
            this->read_byte(Bank::USER_BANK_3, std::to_underlying(Bank3::RA::I2C_SLV4_CTRL)));
    }

    void ICM20948::set_i2c_slv4_ctrl_register(Bank3::I2C_SLV4_CTRL const i2c_slv4_ctrl) const noexcept
    {
        this->write_byte(Bank::USER_BANK_3,
                         std::to_underlying(Bank3::RA::I2C_SLV4_CTRL),
                         std::bit_cast<std::uint8_t>(i2c_slv4_ctrl));
    }

    Bank3::I2C_SLV4_DO ICM20948::get_i2c_slv4_do_register() const noexcept
    {
        return std::bit_cast<Bank3::I2C_SLV4_DO>(
            this->read_byte(Bank::USER_BANK_3, std::to_underlying(Bank3::RA::I2C_SLV4_DO)));
    }

    void ICM20948::set_i2c_slv4_do_register(Bank3::I2C_SLV4_DO const i2c_slv4_do) const noexcept
    {
        this->write_byte(Bank::USER_BANK_3,
                         std::to_underlying(Bank3::RA::I2C_SLV4_DO),
                         std::bit_cast<std::uint8_t>(i2c_slv4_do));
    }

    Bank3::I2C_SLV4_DI ICM20948::get_i2c_slv4_di_register() const noexcept
    {
        return std::bit_cast<Bank3::I2C_SLV4_DI>(
            this->read_byte(Bank::USER_BANK_3, std::to_underlying(Bank3::RA::I2C_SLV4_DI)));
    }

}; // namespace ICM20948