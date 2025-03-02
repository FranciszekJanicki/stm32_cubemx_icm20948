#include "ak09916.hpp"
#include "ak09916_config.hpp"
#include "ak09916_registers.hpp"
#include "i2c_device.hpp"

namespace AK09916 {

    AK09916::AK09916(I2CDevice&& i2c_device, Config const& config) noexcept :
        i2c_device_{std::forward<I2CDevice>(i2c_device)}
    {
        this->initialize(config);
    }

    AK09916::~AK09916() noexcept
    {
        this->deinitialize();
    }

    std::optional<float> AK09916::get_magnetic_field_x_scaled() const noexcept
    {
        return this->get_magnetic_field_x_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->scale_; });
    }

    std::optional<float> AK09916::get_magnetic_field_y_scaled() const noexcept
    {
        return this->get_magnetic_field_y_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->scale_; });
    }

    std::optional<float> AK09916::get_magnetic_field_z_scaled() const noexcept
    {
        return this->get_magnetic_field_z_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->scale_; });
    }

    std::optional<Vec3D<float>> AK09916::get_magnetic_field_scaled() const noexcept
    {
        return this->get_magnetic_field_raw().transform(
            [this](Vec3D<std::int16_t> const& raw) { return static_cast<Vec3D<float>>(raw) / this->scale_; });
    }

    bool AK09916::is_data_ready() const noexcept
    {
        return this->get_status_1_register().drdy;
    }

    std::uint8_t AK09916::read_byte(std::uint8_t const reg_address) const noexcept
    {
        return this->i2c_device_.read_byte(reg_address);
    }

    void AK09916::write_byte(std::uint8_t const reg_address, std::uint8_t const byte) const noexcept
    {
        this->i2c_device_.write_byte(reg_address, byte);
    }

    void AK09916::initialize(Config const& config) noexcept
    {
        if (this->is_valid_device_id() && this->is_valid_company_id()) {
            this->device_reset();
            this->set_control_1_register(config.control_1);
            this->set_control_2_register(config.control_2);
            this->set_control_3_register(config.control_3);
            this->initialized_ = true;
        }
    }

    void AK09916::deinitialize() noexcept
    {
        if (this->is_valid_device_id() && this->is_valid_company_id()) {
            this->device_reset();
            this->initialized_ = false;
        }
    }

    void AK09916::device_reset() const noexcept
    {
        auto control_3 = this->get_control_3_register();
        control_3.srst = true;
        this->set_control_3_register(control_3);
        HAL_Delay(200UL);
    }

    bool AK09916::is_valid_device_id() const noexcept
    {
        return this->get_device_id() == DEVICE_ID;
    }

    bool AK09916::is_valid_company_id() const noexcept
    {
        return this->get_company_id() == COMPANY_ID;
    }

    std::uint8_t AK09916::get_device_id() const noexcept
    {
        return std::bit_cast<std::uint8_t>(this->get_who_am_2_register());
    }

    std::uint8_t AK09916::get_company_id() const noexcept
    {
        return std::bit_cast<std::uint8_t>(this->get_who_am_1_register());
    }

    std::optional<std::int16_t> AK09916::get_magnetic_field_x_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::byteswap(std::bit_cast<std::int16_t>(this->get_xout_registers()))}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<std::int16_t> AK09916::get_magnetic_field_y_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::byteswap(std::bit_cast<std::int16_t>(this->get_yout_registers()))}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<std::int16_t> AK09916::get_magnetic_field_z_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::byteswap(std::bit_cast<std::int16_t>(this->get_zout_registers()))}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<Vec3D<std::int16_t>> AK09916::get_magnetic_field_raw() const noexcept
    {
        auto const out = this->get_out_registers();
        return this->initialized_
                   ? std::optional<Vec3D<std::int16_t>>{std::in_place,
                                                        std::byteswap(std::bit_cast<std::int16_t>(out.xout)),
                                                        std::byteswap(std::bit_cast<std::int16_t>(out.yout)),
                                                        std::byteswap(std::bit_cast<std::int16_t>(out.zout))}
                   : std::optional<Vec3D<std::int16_t>>{std::nullopt};
    }

    WHO_AM_I_1 AK09916::get_who_am_1_register() const noexcept
    {
        return std::bit_cast<WHO_AM_I_1>(this->read_byte(std::to_underlying(RA::WHO_AM_I_1)));
    }

    WHO_AM_I_2 AK09916::get_who_am_2_register() const noexcept
    {
        return std::bit_cast<WHO_AM_I_2>(this->read_byte(std::to_underlying(RA::WHO_AM_I_2)));
    }

    STATUS_1 AK09916::get_status_1_register() const noexcept
    {
        return std::bit_cast<STATUS_1>(this->read_byte(std::to_underlying(RA::STATUS_1)));
    }

    XOUT AK09916::get_xout_registers() const noexcept
    {
        return std::bit_cast<XOUT>(this->read_bytes<sizeof(XOUT)>(std::to_underlying(RA::XOUT_L)));
    }

    YOUT AK09916::get_yout_registers() const noexcept
    {
        return std::bit_cast<YOUT>(this->read_bytes<sizeof(YOUT)>(std::to_underlying(RA::YOUT_L)));
    }

    ZOUT AK09916::get_zout_registers() const noexcept
    {
        return std::bit_cast<ZOUT>(this->read_bytes<sizeof(ZOUT)>(std::to_underlying(RA::ZOUT_L)));
    }

    OUT AK09916::get_out_registers() const noexcept
    {
        return std::bit_cast<OUT>(this->read_bytes<sizeof(OUT)>(std::to_underlying(RA::XOUT_L)));
    }

    STATUS_2 AK09916::get_status_2_register() const noexcept
    {
        return std::bit_cast<STATUS_2>(this->read_byte(std::to_underlying(RA::STATUS_2)));
    }

    CONTROL_1 AK09916::get_control_1_register() const noexcept
    {
        return std::bit_cast<CONTROL_1>(this->read_byte(std::to_underlying(RA::CONTROL_1)));
    }

    void AK09916::set_control_1_register(CONTROL_1 const control_1) const noexcept
    {
        this->write_byte(std::to_underlying(RA::CONTROL_1), std::bit_cast<std::uint8_t>(control_1));
    }

    CONTROL_2 AK09916::get_control_2_register() const noexcept
    {
        return std::bit_cast<CONTROL_2>(this->read_byte(std::to_underlying(RA::CONTROL_2)));
    }

    void AK09916::set_control_2_register(CONTROL_2 const control_2) const noexcept
    {
        this->write_byte(std::to_underlying(RA::CONTROL_2), std::bit_cast<std::uint8_t>(control_2));
    }

    CONTROL_3 AK09916::get_control_3_register() const noexcept
    {
        return std::bit_cast<CONTROL_3>(this->read_byte(std::to_underlying(RA::CONTROL_3)));
    }

    void AK09916::set_control_3_register(CONTROL_3 const control_3) const noexcept
    {
        this->write_byte(std::to_underlying(RA::CONTROL_3), std::bit_cast<std::uint8_t>(control_3));
    }

}; // namespace AK09916