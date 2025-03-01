#ifndef AK09916_CONFIG_HPP
#define AK09916_CONFIG_HPP

#include "vector3d.hpp"
#include <cstdint>

namespace AK09916 {

    template <typename T>
    using Vec3D = Utility::Vector3D<T>;

    enum struct RA : std::uint8_t {
        WHO_AM_I_1 = 0x00,
        WHO_AM_I_2 = 0x01,
        STATUS_1 = 0x10,
        XOUT_L = 0x11,
        XOUT_H = 0x12,
        YOUT_L = 0x13,
        YOUT_H = 0x14,
        ZOUT_L = 0x15,
        ZOUT_H = 0x16,
        TMPS = 0x17,
        STATUS_2 = 0x18,
        CONTROL_1 = 0x30,
        CONTROL_2 = 0x31,
        CONTROL_3 = 0x32,
        TEST_1 = 0x33,
        TEST_2 = 0x34,
    };

    enum struct Mode : std::uint8_t {
        POWER_DOWN = 0b00000,
        SINGLESHOT = 0b00001,
        CONTINUOUS_1 = 0b0010,
        CONTINUOUS_2 = 0b00100,
        CONTINUOUS_3 = 0b00110,
        CONTINUOUS_4 = 0b01000,
        SELF_TEST = 0b10000,
    };

    constexpr std::uint8_t DEV_ADDRESS = 0b0001100;
    constexpr std::uint8_t COMPANY_ID = 0b01001000;
    constexpr std::uint8_t DEVICE_ID = 0b00001001;
    constexpr float SCALE = 6666666.6666F;

}; // namespace AK09916

#endif // AK09916_CONFIG_HPP