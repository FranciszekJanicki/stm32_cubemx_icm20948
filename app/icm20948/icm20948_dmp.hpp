#ifndef ICM20948_DMP_HPP
#define ICM20948_DMP_HPP

#include "icm20948.hpp"
#include "icm20948_dmp_config.hpp"
#include "icm20948_dmp_img.hpp"
#include "icm20948_dmp_registers.hpp"

namespace ICM20948 {

    struct ICM20948_DMP {
    public:
        ICM20948_DMP() noexcept = default;
        ICM20948_DMP(ICM20948&& base, DMPConfig const& dmp_config) noexcept;

        ICM20948_DMP(ICM20948_DMP const& other) = delete;
        ICM20948_DMP(ICM20948_DMP&& other) noexcept = default;

        ICM20948_DMP& operator=(ICM20948_DMP const& other) = delete;
        ICM20948_DMP& operator=(ICM20948_DMP&& other) noexcept = default;

        ~ICM20948_DMP() noexcept;

        float get_roll() const noexcept;
        float get_pitch() const noexcept;
        float get_yaw() const noexcept;
        Vec3D<float> get_roll_pitch_yaw() const noexcept;

    private:
        void initialize(DMPConfig const& dmp_config) noexcept;
        void deinitialize() noexcept;

        Quat3D<std::int16_t> get_quaternion_raw() const noexcept;
        Quat3D<float> get_quaternion_scaled() const noexcept;
        Vec3D<float> get_gravity() const noexcept;

        void set_memory_bank(std::uint8_t const bank,
                             bool const prefetch_enabled = false,
                             bool const user_bank = false) const noexcept;
        void set_memory_start_address(std::uint8_t const address) const noexcept;

        std::uint8_t read_memory_byte() const noexcept;
        void write_memory_byte(std::uint8_t write_data) const noexcept;
        void read_memory_block(std::uint8_t* read_data,
                               std::size_t const read_size,
                               std::uint8_t bank,
                               std::uint8_t address) const noexcept;
        void write_memory_block(std::uint8_t* write_data,
                                std::size_t const write_size,
                                std::uint8_t bank,
                                std::uint8_t address) const noexcept;
        void write_dmp_configuration_set(std::uint8_t* write_data, std::size_t const write_size) const noexcept;

        std::array<std::uint8_t, 42UL> get_dmp_packet() const noexcept;

        bool initialized_{false};

        ICM20948 base_{};
    };

}; // namespace ICM20948

#endif // ICM20948_DMP_HPP