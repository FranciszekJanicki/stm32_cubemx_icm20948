#ifndef AK09916_HPP
#define AK09916_HPP

#include "ak09916_config.hpp"
#include "ak09916_registers.hpp"
#include "i2c_device.hpp"
#include "utility.hpp"
#include <optional>

namespace AK09916 {

    struct AK09916 {
    public:
        using I2CDevice = Utility::I2CDevice;

        AK09916() noexcept = default;

        AK09916(I2CDevice&& i2c_device,
                CONTROL_1 const control_1,
                CONTROL_2 const control_2,
                CONTROL_3 const control_3) noexcept;

        AK09916(AK09916 const& other) = delete;
        AK09916(AK09916&& other) noexcept = default;

        AK09916& operator=(AK09916 const& other) = delete;
        AK09916& operator=(AK09916&& other) noexcept = default;

        ~AK09916() noexcept;

        std::optional<float> get_magnetic_field_x_scaled() const noexcept;
        std::optional<float> get_magnetic_field_y_scaled() const noexcept;
        std::optional<float> get_magnetic_field_z_scaled() const noexcept;
        std::optional<Vec3D<float>> get_magnetic_field_scaled() const noexcept;

        bool is_data_ready() const noexcept;

    private:
        void initialize(CONTROL_1 const control_1, CONTROL_2 const control_2, CONTROL_3 const control_3) noexcept;
        void deinitialize() noexcept;

        void device_reset() const noexcept;

        bool is_valid_device_id() const noexcept;
        bool is_valid_company_id() const noexcept;

        std::uint8_t get_device_id() const noexcept;
        std::uint8_t get_company_id() const noexcept;

        std::optional<std::int16_t> get_magnetic_field_x_raw() const noexcept;
        std::optional<std::int16_t> get_magnetic_field_y_raw() const noexcept;
        std::optional<std::int16_t> get_magnetic_field_z_raw() const noexcept;
        std::optional<Vec3D<std::int16_t>> get_magnetic_field_raw() const noexcept;

        WHO_AM_I_1 get_who_am_1_register() const noexcept;

        WHO_AM_I_2 get_who_am_2_register() const noexcept;

        STATUS_1 get_status_1_register() const noexcept;

        XOUT get_xout_registers() const noexcept;

        YOUT get_yout_registers() const noexcept;

        ZOUT get_zout_registers() const noexcept;

        OUT get_out_registers() const noexcept;

        STATUS_2 get_status_2_register() const noexcept;

        CONTROL_1 get_control_1_register() const noexcept;
        void set_control_1_register(CONTROL_1 const control_1) const noexcept;

        CONTROL_2 get_control_2_register() const noexcept;
        void set_control_2_register(CONTROL_2 const control_2) const noexcept;

        CONTROL_3 get_control_3_register() const noexcept;
        void set_control_3_register(CONTROL_3 const control_3) const noexcept;

        bool initialized_{false};

        I2CDevice i2c_device_{};
    };

}; // namespace AK09916

#endif // AK09916_HPP