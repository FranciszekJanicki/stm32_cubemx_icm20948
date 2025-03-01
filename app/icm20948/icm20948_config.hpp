#ifndef ICM20948_CONFIG_HPP
#define ICM20948_CONFIG_HPP

#include "quaternion3d.hpp"
#include "vector3d.hpp"
#include <cstdint>

namespace ICM20948 {

    template <typename T>
    using Vec3D = Utility::Vector3D<T>;

    template <typename T>
    using Quat3D = Utility::Quaternion3D<T>;

    enum struct DevAddress : std::uint8_t {
        AD0_LOW = 0b1101000,
        AD0_HIGH = 0b1101001,
    };

    static constexpr std::uint8_t RA_REG_BANK_SEL = 0x7F;

    namespace Bank0 {

        enum struct RA : std::uint8_t {
            WHO_AM_I = 0x00,
            USER_CTRL = 0x03,
            LP_CONFIG = 0x05,
            PWR_MGMT_1 = 0x06,
            PWR_MGMT_2 = 0x07,
            INT_PIN_CFG = 0x0F,
            INT_ENABLE = 0x10,
            INT_ENABLE_1 = 0x11,
            INT_ENABLE_2 = 0x12,
            INT_ENABLE_3 = 0x13,
            I2C_MST_STATUS = 0x17,
            INT_STATUS = 0x19,
            INT_STATUS_1 = 0x1A,
            INT_STATUS_2 = 0x1B,
            INT_STATUS_3 = 0x1C,
            DELAY_TIMEH = 0x28,
            DELAY_TIMEL = 0x29,
            ACCEL_XOUT_H = 0x2D,
            ACCEL_XOUT_L = 0x2E,
            ACCEL_YOUT_H = 0x2F,
            ACCEL_YOUT_L = 0x30,
            ACCEL_ZOUT_H = 0x31,
            ACCEL_ZOUT_L = 0x32,
            GYRO_XOUT_H = 0x33,
            GYRO_XOUT_L = 0x34,
            GYRO_YOUT_H = 0x35,
            GYRO_YOUT_L = 0x36,
            GYRO_ZOUT_H = 0x37,
            GYRO_ZOUT_L = 0x38,
            TEMP_OUT_H = 0x39,
            TEMP_OUT_L = 0x3A,
            EXT_SLV_SENS_DATA_00 = 0x3B,
            FIFO_EN_1 = 0x66,
            FIFO_EN_2 = 0x67,
            FIFO_RST = 0x68,
            FIFO_MODE = 0x69,
            FIFO_COUNTH = 0x70,
            FIFO_COUNTL = 0x71,
            FIFO_R_W = 0x72,
            DATA_RDY_STATUS = 0x74,
            FIFO_CFG = 0x76,
        };

    };

    namespace Bank1 {

        enum struct RA : std::uint8_t {
            SELF_TEST_X_GYRO = 0x02,
            SELF_TEST_Y_GYRO = 0x03,
            SELF_TEST_Z_GYRO = 0x04,
            SELF_TEST_X_ACCEL = 0x0E,
            SELF_TEST_Y_ACCEL = 0x0F,
            SELF_TEST_Z_ACCEL = 0x10,
            XA_OFFS_H = 0x14,
            XA_OFFS_L = 0x15,
            YA_OFFS_H = 0x17,
            YA_OFFS_L = 0x18,
            ZA_OFFS_H = 0x1A,
            ZA_OFFS_L = 0x1B,
            TIMEBASE_CORRECTION_PLL = 0x28,
        };

    };

    namespace Bank2 {

        enum struct RA : std::uint8_t {
            GYRO_SMPLRT_DIV = 0x00,
            GYRO_CONFIG_1 = 0x01,
            GYRO_CONFIG_2 = 0x02,
            XG_OFFS_USRH = 0x03,
            XG_OFFS_USRL = 0x04,
            YG_OFFS_USRH = 0x05,
            YG_OFFS_USRL = 0x06,
            ZG_OFFS_USRH = 0x07,
            ZG_OFFS_USRL = 0x08,
            ODR_ALIGN_EN = 0x09,
            ACCEL_SMPLRT_DIV_1 = 0x10,
            ACCEL_SMPLRT_DIV_2 = 0x11,
            ACCEL_INTEL_CTRL = 0x12,
            ACCEL_WOM_THR = 0x13,
            ACCEL_CONFIG_1 = 0x14,
            ACCEL_CONFIG_2 = 0x15,
            FSYNC_CONFIG = 0x52,
            TEMP_CONFIG = 0x53,
            MOD_CTRL_USR = 0x54,
        };

    };

    namespace Bank3 {

        enum struct RA : std::uint8_t {
            I2C_MST_ODR_CONFIG = 0x00,
            I2C_MST_CTRL = 0x01,
            I2C_MST_DELAY_CTRL = 0x02,
            I2C_SLV0_ADDR = 0x03,
            I2C_SLV0_REG = 0x04,
            I2C_SLV0_CTRL = 0x05,
            I2C_SLV0_DO = 0x06,
            I2C_SLV1_ADDR = 0x07,
            I2C_SLV1_REG = 0x08,
            I2C_SLV1_CTRL = 0x09,
            I2C_SLV1_DO = 0x0A,
            I2C_SLV2_ADDR = 0x0B,
            I2C_SLV2_REG = 0x0C,
            I2C_SLV2_CTRL = 0x0D,
            I2C_SLV2_DO = 0x0E,
            I2C_SLV3_ADDR = 0x0F,
            I2C_SLV3_REG = 0x10,
            I2C_SLV3_CTRL = 0x11,
            I2C_SLV3_DO = 0x12,
            I2C_SLV4_ADDR = 0x13,
            I2C_SLV4_REG = 0x14,
            I2C_SLV4_CTRL = 0x15,
            I2C_SLV4_DO = 0x16,
            I2C_SLV4_DI = 0x17,
        };

    };

    enum struct Bank : std::uint8_t {
        USER_BANK_0 = 0x00,
        USER_BANK_1 = 0x01,
        USER_BANK_2 = 0x02,
        USER_BANK_3 = 0x03,
    };

    enum struct AccelRange : std::uint8_t {
        ACCEL_FS_2 = 0b00,
        ACCEL_FS_4 = 0b01,
        ACCEL_FS_8 = 0b10,
        ACCEL_FS_16 = 0b11,
    };

    enum struct GyroRange : std::uint8_t {
        GYRO_FS_250 = 0b00,
        GYRO_FS_500 = 0b01,
        GYRO_FS_1000 = 0b10,
        GYRO_FS_2000 = 0b11,
    };

    enum struct MagRange : std::uint8_t {};

    enum struct Mode : std::uint8_t {
        SLEEP_MODE,
        LOW_POWER_ACC_MODE,
        LOW_NOISE_ACC_MODE,
        GYRO_MODE,
        MAG_MODE,
        ACCEL_GYRO_MODE,
        ACCEL_MAG_MODE,
        ACCEL_GYRO_MAG_MODE,
    };

    enum struct ClockSource : std::uint8_t {
        INTERNAL_20MHZ = 0b00,
        AUTO_SELECT = 0b01,
        CLOCK_RESET = 0b111,
    };

    enum struct FIFOMode : std::uint8_t {
        STREAM = 0x00,
        SNAPSHOT = 0x01,
    };

    enum struct GyroLPF : std::uint8_t {
        BW_230 = 0x00,
        BW_188 = 0x01,
        BW_155 = 0x02,
        BW_73 = 0x03,
        BW_36 = 0x04,
        BW_18 = 0x05,
        BW_9 = 0x06,
        BW_377 = 0x07,
    };

    enum struct GyroFIR : std::uint8_t {
        BW_774 = 0x00,
        BW_470 = 0x01,
        BW_258 = 0x02,
        BW_135 = 0x03,
        BW_69 = 0x04,
        BW_35 = 0x05,
        BW_18 = 0x06,
        BW_9 = 0x07,
    };

    enum struct AccelLPF : std::uint8_t {
        BW_265 = 0x00,
        BW_136 = 0x02,
        BW_69 = 0x03,
        BW_34 = 0x04,
        BW_17 = 0x05,
        BW_8 = 0x06,
        BW_499 = 0x07,
    };

    enum struct AccelFIR : std::uint8_t {
        BW_1238 = 0x00,
        BW_497 = 0x00,
        BW_265 = 0x01,
        BW_137 = 0x02,
        BW_69 = 0x03,
    };

    enum struct ExtSync : std::uint8_t {
        DISABLED = 0x0,
        TEMP_OUT_L = 0x1,
        GYRO_XOUT_L = 0x2,
        GYRO_YOUT_L = 0x3,
        GYRO_ZOUT_L = 0x4,
        ACCEL_XOUT_L = 0x5,
        ACCEL_YOUT_L = 0x6,
        ACCEL_ZOUT_L = 0x7,
    };

    enum struct SlaveNum : std::uint8_t {
        SLAVE1 = 1,
        SLAVE2 = 2,
        SLAVE3 = 3,
    };

    enum struct IntMode : std::uint8_t {
        ACTIVEHIGH = 0x00,
        ACTIVELOW = 0x01,
    };

    enum struct IntDrive : std::uint8_t {
        PUSHPULL = 0x00,
        OPENDRAIN = 0x01,
    };

    enum struct IntLatch : std::uint8_t {
        PULSE50US = 0x00,
        WAITCLEAR = 0x01,
    };

    enum struct IntClear : std::uint8_t {
        STATUSREAD = 0x00,
        ANYREAD = 0x01,
    };

    inline float gyro_range_to_scale(GyroRange const gyro_range) noexcept
    {
        switch (gyro_range) {
            case GyroRange::GYRO_FS_250:
                return 131.0F;
            case GyroRange::GYRO_FS_500:
                return 65.5F;
            case GyroRange::GYRO_FS_1000:
                return 32.8F;
            case GyroRange::GYRO_FS_2000:
                return 16.4F;
            default:
                return 0.0F;
        }
    }

    inline float accel_range_to_scale(AccelRange const accel_range) noexcept
    {
        switch (accel_range) {
            case AccelRange::ACCEL_FS_2:
                return 16384.0F;
            case AccelRange::ACCEL_FS_4:
                return 8192.0F;
            case AccelRange::ACCEL_FS_8:
                return 4096.0F;
            case AccelRange::ACCEL_FS_16:
                return 2048.0F;
            default:
                return 0.0F;
        }
    }

    static constexpr auto GYRO_OUTPUT_RATE_DLPF_DIS_HZ = 1100UL;
    static constexpr auto GYRO_OUTPUT_RATE_DLPF_EN_HZ = 1100UL;
    static constexpr auto ACCEL_OUTPUT_RATE_HZ = 1125UL;

    inline std::uint8_t sampling_rate_to_gyro_smplrt_div(std::uint32_t const sampling_rate, GyroLPF const dlpf) noexcept
    {
        if (dlpf == GyroLPF::BW_377) {
            return static_cast<std::uint8_t>((GYRO_OUTPUT_RATE_DLPF_DIS_HZ / sampling_rate) - 1U);
        } else {
            return static_cast<std::uint8_t>((GYRO_OUTPUT_RATE_DLPF_EN_HZ / sampling_rate) - 1U);
        }
    }

    inline std::uint8_t sampling_rate_to_accel_smplrt_div(std::uint32_t const sampling_rate) noexcept
    {
        return static_cast<std::uint8_t>((ACCEL_OUTPUT_RATE_HZ / sampling_rate) - 1U);
    }

}; // namespace ICM20948

#endif // ICM20948_CONFIG_HPP