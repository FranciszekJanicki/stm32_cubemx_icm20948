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

        std::uint8_t read_byte(Bank0::RA const reg_address) const noexcept;
        std::uint8_t read_byte(Bank1::RA const reg_address) const noexcept;
        std::uint8_t read_byte(Bank2::RA const reg_address) const noexcept;
        std::uint8_t read_byte(Bank3::RA const reg_address) const noexcept;

        void write_byte(Bank0::RA const reg_address, std::uint8_t const byte) const noexcept;
        void write_byte(Bank1::RA const reg_address, std::uint8_t const byte) const noexcept;
        void write_byte(Bank2::RA const reg_address, std::uint8_t const byte) const noexcept;
        void write_byte(Bank3::RA const reg_address, std::uint8_t const byte) const noexcept;

        void select_bank(Bank const bank) const noexcept;

        REG_BANK_SEL get_reg_bank_sel_register() const noexcept;
        void set_reg_bank_sel_register(REG_BANK_SEL const reg_bank_sel) const noexcept;

        bool initialized_{false};

        I2CDevice i2c_device_{};
    };

}; // namespace ICM20948

#endif // ICM20948_HPP