#ifndef ICM20948_DMP_CONFIG_HPP
#define ICM20948_DMP_CONFIG_HPP

#include "icm20948_config.hpp"
#include <cmath>
#include <cstdint>
#include <numbers>

namespace ICM20948 {

    enum struct RA : std::uint8_t {
        DMP_INT_STATUS = 0x18,
    };

    struct DMPConfig {};

    constexpr auto DMP_MEMORY_BANKS{8};
    constexpr auto DMP_MEMORY_BANK_SIZE{256UL};
    constexpr auto DMP_MEMORY_CHUNK_SIZE{16UL};
    constexpr auto FIFO_DEFAULT_TIMEOUT{11000};
    constexpr auto FIFO_MAX_COUNT{1024UL};

    inline Vec3D<float> quaternion_to_gravity(Quat3D<float> const& quaternion) noexcept
    {
        return Vec3D<float>{2 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y),
                            2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z),
                            quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y +
                                quaternion.z * quaternion.z};
    }

    inline float quaternion_to_roll(Quat3D<float> const& quaternion) noexcept
    {
        auto const gravity = quaternion_to_gravity(quaternion);
        return std::atan2(gravity.y, gravity.z);
    }

    inline float quaternion_to_pitch(Quat3D<float> const& quaternion) noexcept
    {
        auto const gravity = quaternion_to_gravity(quaternion);
        auto const pitch = std::atan2(gravity.x, std::sqrt(gravity.y * gravity.y + gravity.z * gravity.z));
        return (gravity.z < 0) ? (pitch > 0 ? std::numbers::pi_v<float> - pitch : -std::numbers::pi_v<float> - pitch)
                               : pitch;
    }

    inline float quaternion_to_yaw(Quat3D<float> const& quaternion) noexcept
    {
        return std::atan2(2 * quaternion.x * quaternion.y - 2 * quaternion.w * quaternion.z,
                          2 * quaternion.w * quaternion.w + 2 * quaternion.x * quaternion.x - 1);
    }

    inline Vec3D<float> quaternion_to_roll_pitch_yaw(Quat3D<float> const& quaternion) noexcept
    {
        return Vec3D<float>{quaternion_to_roll(quaternion),
                            quaternion_to_pitch(quaternion),
                            quaternion_to_yaw(quaternion)};
    }

}; // namespace ICM20948

#endif // ICM20948_DMP_CONFIG_HPP