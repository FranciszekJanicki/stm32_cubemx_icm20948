#include "icm20948_mag.hpp"
#include "ak09916.hpp"
#include "ak09916_config.hpp"
#include "ak09916_registers.hpp"

using RA = AK09916::RA;

namespace ICM20948 {

    ICM20948_Mag::ICM20948_Mag(ICM20948&& icm20948, AK09916::Config const& config, SlaveNum const slave_num) noexcept :
        slave_num_{slave_num}, base_{std::forward<ICM20948>(icm20948)}
    {
        this->initialize(config);
    }

    ICM20948_Mag::~ICM20948_Mag() noexcept
    {
        this->deinitialize();
    }

    std::optional<float> ICM20948_Mag::get_acceleration_x_scaled() const noexcept
    {
        return this->base_.get_acceleration_x_scaled();
    }

    std::optional<float> ICM20948_Mag::get_acceleration_y_scaled() const noexcept
    {
        return this->base_.get_acceleration_y_scaled();
    }

    std::optional<float> ICM20948_Mag::get_acceleration_z_scaled() const noexcept
    {
        return this->base_.get_acceleration_z_scaled();
    }

    std::optional<Vec3D<float>> ICM20948_Mag::get_acceleration_scaled() const noexcept
    {
        return this->base_.get_acceleration_scaled();
    }

    std::optional<float> ICM20948_Mag::get_rotation_x_scaled() const noexcept
    {
        return this->base_.get_rotation_x_scaled();
    }

    std::optional<float> ICM20948_Mag::get_rotation_y_scaled() const noexcept
    {
        return this->base_.get_rotation_y_scaled();
    }

    std::optional<float> ICM20948_Mag::get_rotation_z_scaled() const noexcept
    {
        return this->base_.get_rotation_z_scaled();
    }

    std::optional<Vec3D<float>> ICM20948_Mag::get_rotation_scaled() const noexcept
    {
        return this->base_.get_rotation_scaled();
    }

    std::optional<float> ICM20948_Mag::get_magnetic_field_x_scaled() const noexcept
    {
        return this->get_magnetic_field_x_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->scale_; });
    }

    std::optional<float> ICM20948_Mag::get_magnetic_field_y_scaled() const noexcept
    {
        return this->get_magnetic_field_y_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->scale_; });
    }

    std::optional<float> ICM20948_Mag::get_magnetic_field_z_scaled() const noexcept
    {
        return this->get_magnetic_field_z_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->scale_; });
    }

    std::optional<Vec3D<float>> ICM20948_Mag::get_magnetic_field_scaled() const noexcept
    {
        return this->get_magnetic_field_raw().transform(
            [this](Vec3D<std::int16_t> const& raw) { return static_cast<Vec3D<float>>(raw) / this->scale_; });
    }

    bool ICM20948_Mag::is_data_ready() const noexcept
    {
        return this->get_status_1_register().drdy;
    }

    std::uint8_t ICM20948_Mag::read_byte(std::uint8_t const reg_address) const noexcept
    {
        return this->base_.ext_slv_read_byte(this->slave_num_, reg_address);
    }

    void ICM20948_Mag::write_byte(std::uint8_t const reg_address, std::uint8_t const byte) const noexcept
    {
        this->base_.ext_slv_write_byte(this->slave_num_, reg_address, byte);
    }

    void ICM20948_Mag::initialize(AK09916::Config const& config) noexcept
    {
        if (this->is_valid_device_id() && this->is_valid_company_id()) {
            this->device_reset();
            this->set_control_1_register(config.control_1);
            this->set_control_2_register(config.control_2);
            this->set_control_3_register(config.control_3);
            this->initialized_ = true;
        }
    }

    void ICM20948_Mag::deinitialize() noexcept
    {
        if (this->is_valid_device_id() && this->is_valid_company_id()) {
            this->device_reset();
            this->initialized_ = false;
        }
    }

    void ICM20948_Mag::device_reset() const noexcept
    {
        auto control_3 = this->get_control_3_register();
        control_3.srst = true;
        this->set_control_3_register(control_3);
        HAL_Delay(200UL);
    }

    bool ICM20948_Mag::is_valid_device_id() const noexcept
    {
        return this->get_device_id() == AK09916::DEVICE_ID;
    }

    bool ICM20948_Mag::is_valid_company_id() const noexcept
    {
        return this->get_company_id() == AK09916::COMPANY_ID;
    }

    std::uint8_t ICM20948_Mag::get_device_id() const noexcept
    {
        return std::bit_cast<std::uint8_t>(this->get_who_am_2_register());
    }

    std::uint8_t ICM20948_Mag::get_company_id() const noexcept
    {
        return std::bit_cast<std::uint8_t>(this->get_who_am_1_register());
    }

    std::optional<std::int16_t> ICM20948_Mag::get_magnetic_field_x_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::byteswap(std::bit_cast<std::int16_t>(this->get_xout_registers()))}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<std::int16_t> ICM20948_Mag::get_magnetic_field_y_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::byteswap(std::bit_cast<std::int16_t>(this->get_yout_registers()))}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<std::int16_t> ICM20948_Mag::get_magnetic_field_z_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::byteswap(std::bit_cast<std::int16_t>(this->get_zout_registers()))}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<Vec3D<std::int16_t>> ICM20948_Mag::get_magnetic_field_raw() const noexcept
    {
        auto const out = this->get_out_registers();
        return this->initialized_
                   ? std::optional<Vec3D<std::int16_t>>{std::in_place,
                                                        std::byteswap(std::bit_cast<std::int16_t>(out.xout)),
                                                        std::byteswap(std::bit_cast<std::int16_t>(out.yout)),
                                                        std::byteswap(std::bit_cast<std::int16_t>(out.zout))}
                   : std::optional<Vec3D<std::int16_t>>{std::nullopt};
    }

    WHO_AM_I_1 ICM20948_Mag::get_who_am_1_register() const noexcept
    {
        return std::bit_cast<AK09916::WHO_AM_I_1>(this->read_byte(std::to_underlying(AK09916::RA::WHO_AM_I_1)));
    }

    WHO_AM_I_2 ICM20948_Mag::get_who_am_2_register() const noexcept
    {
        return std::bit_cast<AK09916::WHO_AM_I_2>(this->read_byte(std::to_underlying(AK09916::RA::WHO_AM_I_2)));
    }

    STATUS_1 ICM20948_Mag::get_status_1_register() const noexcept
    {
        return std::bit_cast<STATUS_1>(this->read_byte(std::to_underlying(AK09916::RA::STATUS_1)));
    }

    XOUT ICM20948_Mag::get_xout_registers() const noexcept
    {
        return std::bit_cast<XOUT>(this->read_bytes<sizeof(XOUT)>(std::to_underlying(AK09916::RA::XOUT_L)));
    }

    YOUT ICM20948_Mag::get_yout_registers() const noexcept
    {
        return std::bit_cast<YOUT>(this->read_bytes<sizeof(YOUT)>(std::to_underlying(AK09916::RA::YOUT_L)));
    }

    ZOUT ICM20948_Mag::get_zout_registers() const noexcept
    {
        return std::bit_cast<ZOUT>(this->read_bytes<sizeof(ZOUT)>(std::to_underlying(AK09916::RA::ZOUT_L)));
    }

    OUT ICM20948_Mag::get_out_registers() const noexcept
    {
        return std::bit_cast<OUT>(this->read_bytes<sizeof(OUT)>(std::to_underlying(AK09916::RA::XOUT_L)));
    }

    STATUS_2 ICM20948_Mag::get_status_2_register() const noexcept
    {
        return std::bit_cast<STATUS_2>(this->read_byte(std::to_underlying(AK09916::RA::STATUS_2)));
    }

    CONTROL_1 ICM20948_Mag::get_control_1_register() const noexcept
    {
        return std::bit_cast<CONTROL_1>(this->read_byte(std::to_underlying(AK09916::RA::CONTROL_1)));
    }

    void ICM20948_Mag::set_control_1_register(CONTROL_1 const control_1) const noexcept
    {
        this->write_byte(std::to_underlying(AK09916::RA::CONTROL_1), std::bit_cast<std::uint8_t>(control_1));
    }

    CONTROL_2 ICM20948_Mag::get_control_2_register() const noexcept
    {
        return std::bit_cast<CONTROL_2>(this->read_byte(std::to_underlying(AK09916::RA::CONTROL_2)));
    }

    void ICM20948_Mag::set_control_2_register(CONTROL_2 const control_2) const noexcept
    {
        this->write_byte(std::to_underlying(AK09916::RA::CONTROL_2), std::bit_cast<std::uint8_t>(control_2));
    }

    CONTROL_3 ICM20948_Mag::get_control_3_register() const noexcept
    {
        return std::bit_cast<CONTROL_3>(this->read_byte(std::to_underlying(AK09916::RA::CONTROL_3)));
    }

    void ICM20948_Mag::set_control_3_register(CONTROL_3 const control_3) const noexcept
    {
        this->write_byte(std::to_underlying(AK09916::RA::CONTROL_3), std::bit_cast<std::uint8_t>(control_3));
    }
}; // namespace ICM20948