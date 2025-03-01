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

    REG_BANK_SEL ICM20948::get_reg_bank_sel_register() const noexcept
    {
        return std::bit_cast<REG_BANK_SEL>(this->i2c_device_.read_byte(RA_REG_BANK_SEL));
    }

    void ICM20948::set_reg_bank_sel_register(REG_BANK_SEL const reg_bank_sel) const noexcept
    {
        this->i2c_device_.write_byte(RA_REG_BANK_SEL, std::bit_cast<std::uint8_t>(reg_bank_sel));
    }

}; // namespace ICM20948