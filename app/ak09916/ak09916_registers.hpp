#ifndef AK09916_REGISTERS_HPP
#define AK09916_REGISTERS_HPP

#include <cstdint>

#define PACKED __attribute__((__packed__))

namespace AK09916 {

    struct WHO_AM_I_1 {
        std::uint8_t wia1 : 8;
    } PACKED;

    struct WHO_AM_I_2 {
        std::uint8_t wia2 : 8;
    } PACKED;

    struct STATUS_1 {
        std::uint8_t : 6;
        std::uint8_t dor : 1;
        std::uint8_t drdy : 1;
    } PACKED;

    struct XOUT {
        std::uint8_t xout_l : 8;
        std::uint8_t xout_h : 8;
    } PACKED;

    struct YOUT {
        std::uint8_t yout_l : 8;
        std::uint8_t yout_h : 8;
    } PACKED;

    struct ZOUT {
        std::uint8_t zout_l : 8;
        std::uint8_t zout_h : 8;
    } PACKED;

    struct OUT {
        XOUT xout;
        YOUT yout;
        ZOUT zout;
    } PACKED;

    struct STATUS_2 {
        std::uint8_t : 4;
        std::uint8_t hofl : 1;
        std::uint8_t : 3;
    } PACKED;

    struct CONTROL_1 {
        std::uint8_t : 8;
    } PACKED;

    struct CONTROL_2 {
        std::uint8_t : 3;
        std::uint8_t mode : 5;
    } PACKED;

    struct CONTROL_3 {
        std::uint8_t : 7;
        std::uint8_t srst : 1;
    } PACKED;

}; // namespace AK09916

#undef PACKED

#endif // AK09916_REGISTERS_HPP