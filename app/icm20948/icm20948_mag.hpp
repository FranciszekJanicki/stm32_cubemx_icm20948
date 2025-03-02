#ifndef ICM20948_Mag_HPP
#define ICM20948_Mag_HPP

#include "ak09916_config.hpp"
#include "ak09916_registers.hpp"
#include "icm20948.hpp"
#include "utility.hpp"
#include <optional>

using namespace AK09916;

namespace ICM20948 {

    struct ICM20948_Mag {
    public:
        ICM20948_Mag() noexcept = default;

        ICM20948_Mag(ICM20948&& base, AK09916::Config const& config, SlaveNum const slave_num) noexcept;

        ICM20948_Mag(ICM20948_Mag const& other) = delete;
        ICM20948_Mag(ICM20948_Mag&& other) noexcept = default;

        ICM20948_Mag& operator=(ICM20948_Mag const& other) = delete;
        ICM20948_Mag& operator=(ICM20948_Mag&& other) noexcept = default;

        ~ICM20948_Mag() noexcept;

        std::optional<float> get_acceleration_x_scaled() const noexcept;
        std::optional<float> get_acceleration_y_scaled() const noexcept;
        std::optional<float> get_acceleration_z_scaled() const noexcept;
        std::optional<Vec3D<float>> get_acceleration_scaled() const noexcept;

        std::optional<float> get_rotation_x_scaled() const noexcept;
        std::optional<float> get_rotation_y_scaled() const noexcept;
        std::optional<float> get_rotation_z_scaled() const noexcept;
        std::optional<Vec3D<float>> get_rotation_scaled() const noexcept;

        std::optional<float> get_magnetic_field_x_scaled() const noexcept;
        std::optional<float> get_magnetic_field_y_scaled() const noexcept;
        std::optional<float> get_magnetic_field_z_scaled() const noexcept;
        std::optional<Vec3D<float>> get_magnetic_field_scaled() const noexcept;

        bool is_data_ready() const noexcept;

    private:
        std::uint8_t read_byte(std::uint8_t const reg_address) const noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> read_bytes(std::uint8_t const reg_address) const noexcept;

        void write_byte(std::uint8_t const reg_address, std::uint8_t const byte) const noexcept;

        template <std::size_t SIZE>
        void write_bytes(std::uint8_t const reg_address, std::array<std::uint8_t, SIZE> const& bytes) const noexcept;

        void initialize(AK09916::Config const& config) noexcept;
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

        float scale_{SCALE};

        SlaveNum slave_num_{};

        ICM20948 base_{};
    };

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, SIZE> ICM20948_Mag::read_bytes(std::uint8_t const reg_address) const noexcept
    {
        return this->base_.ext_slv_read_bytes<SIZE>(this->slave_num_, reg_address);
    }

    template <std::size_t SIZE>
    inline void ICM20948_Mag::write_bytes(std::uint8_t const reg_address,
                                          std::array<std::uint8_t, SIZE> const& bytes) const noexcept
    {
        this->base_.ext_slv_write_bytes(this->slave_num_, reg_address, bytes);
    }

}; // namespace ICM20948

#endif // ICM20948_MAG_HPP