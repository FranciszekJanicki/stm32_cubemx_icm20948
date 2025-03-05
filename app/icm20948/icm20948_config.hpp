#ifndef ICM20948_CONFIG_HPP
#define ICM20948_CONFIG_HPP

#include "icm20948_registers.hpp"
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

    namespace Bank0 {

        enum struct RA : std::uint8_t {
            WHO_AM_I = 0x00,
            LPF = 0x01,
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
            DMP_INT_STATUS = 0x18,
            INT_STATUS = 0x19,
            INT_STATUS_1 = 0x1A,
            INT_STATUS_2 = 0x1B,
            INT_STATUS_3 = 0x1C,
            SINGLE_FIFO_PRIORITY_SEL = 0x26,
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
            EXT_SLV_SENS_DATA_01 = 0x3C,
            EXT_SLV_SENS_DATA_02 = 0x3D,
            EXT_SLV_SENS_DATA_03 = 0x3E,
            EXT_SLV_SENS_DATA_04 = 0x3F,
            EXT_SLV_SENS_DATA_05 = 0x40,
            EXT_SLV_SENS_DATA_06 = 0x41,
            EXT_SLV_SENS_DATA_07 = 0x42,
            EXT_SLV_SENS_DATA_08 = 0x43,
            EXT_SLV_SENS_DATA_09 = 0x44,
            EXT_SLV_SENS_DATA_10 = 0x45,
            EXT_SLV_SENS_DATA_11 = 0x46,
            EXT_SLV_SENS_DATA_12 = 0x47,
            EXT_SLV_SENS_DATA_13 = 0x48,
            EXT_SLV_SENS_DATA_14 = 0x49,
            EXT_SLV_SENS_DATA_15 = 0x4A,
            EXT_SLV_SENS_DATA_16 = 0x4B,
            EXT_SLV_SENS_DATA_17 = 0x4C,
            EXT_SLV_SENS_DATA_18 = 0x4D,
            EXT_SLV_SENS_DATA_19 = 0x4E,
            EXT_SLV_SENS_DATA_20 = 0x4F,
            EXT_SLV_SENS_DATA_21 = 0x50,
            EXT_SLV_SENS_DATA_22 = 0x51,
            EXT_SLV_SENS_DATA_23 = 0x52,
            FIFO_EN_1 = 0x66,
            FIFO_EN_2 = 0x67,
            FIFO_RST = 0x68,
            FIFO_MODE = 0x69,
            FIFO_COUNTH = 0x70,
            FIFO_COUNTL = 0x71,
            FIFO_R_W = 0x72,
            DATA_RDY_STATUS = 0x74,
            FIFO_CFG = 0x76,
            MEM_START_ADDR = 0x7C,
            MEM_BANK_SEL = 0x7E,
        };

        struct Config {
            USER_CTRL user_ctrl{};
            LP_CONFIG lp_config{};
            PWR_MGMT_1 pwr_mgmt_1{};
            PWR_MGMT_2 pwr_mgmt_2{};
            INT_PIN_CFG int_pin_cfg{};
            INT_ENABLE int_enable{};
            INT_ENABLE_1 int_enable_1{};
            INT_ENABLE_2 int_enable_2{};
            INT_ENABLE_3 int_enable_3{};
            FIFO_EN_1 fifo_en_1{};
            FIFO_EN_2 fifo_en_2{};
            FIFO_MODE fifo_mode{};
            FIFO_CFG fifo_cfg{};
        };

    }; // namespace Bank0

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

        struct Config {
            XA_OFFS xa_offs{};
            YA_OFFS ya_offs{};
            ZA_OFFS za_offs{};
            TIMEBASE_CORRECTION_PLL timebase_correction_pll{};
        };

    }; // namespace Bank1

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

        struct Config {
            GYRO_SMPLRT_DIV gyro_smplrt_div{};
            GYRO_CONFIG_1 gyro_config_1{};
            GYRO_CONFIG_2 gyro_config_2{};
            XG_OFFS_USR xg_offs_usr{};
            YG_OFFS_USR yg_offs_usr{};
            ZG_OFFS_USR zg_offs_usr{};
            ODR_ALIGN_EN odr_align_en{};
            ACCEL_SMPLRT_DIV accel_smplrt_div{};
            ACCEL_INTEL_CTRL accel_intel_ctrl{};
            ACCEL_WOM_THR accel_wom_thr{};
            ACCEL_CONFIG_1 accel_config_1{};
            ACCEL_CONFIG_2 accel_config_2{};
            FSYNC_CONFIG fsync_config{};
            TEMP_CONFIG temp_config{};
            MOD_CTRL_USR mod_ctrl_usr{};
        };

    }; // namespace Bank2

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

        struct Config {
            I2C_MST_ODR_CONFIG i2c_mst_odr_config{};
            I2C_MST_CTRL i2c_mst_ctrl{};
            I2C_MST_DELAY_CTRL i2c_mst_delay_ctrl{};
        };

    }; // namespace Bank3

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

    enum struct GyroDLPF : std::uint8_t {
        BW_197 = 0x00,
        BW_152 = 0x01,
        BW_120 = 0x02,
        BW_51 = 0x03,
        BW_24 = 0x04,
        BW_12 = 0x05,
        BW_6 = 0x06,
        BW_361 = 0x07,
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

    enum struct AccelDLPF : std::uint8_t {
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

    enum struct TempDLPF : std::uint8_t {
        BW_7392 = 0x00,
        BW_218 = 0x01,
        BW_126 = 0x02,
        BW_66 = 0x03,
        BW_34 = 0x04,
        BW_17 = 0x05,
        BW_9 = 0x06,
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

    inline float gyro_range_to_scale(GyroRange gyro_range) noexcept
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

    inline float gyro_config_1_to_scale(Bank2::GYRO_CONFIG_1 const gyro_config_1) noexcept
    {
        return gyro_range_to_scale(static_cast<GyroRange>(gyro_config_1.gyro_fs_sel));
    }

    inline float accel_range_to_scale(AccelRange accel_range) noexcept
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

    inline float accel_config_1_to_scale(Bank2::ACCEL_CONFIG_1 const accel_config_1) noexcept
    {
        return accel_range_to_scale(static_cast<AccelRange>(accel_config_1.accel_fs_sel));
    }

    constexpr std::uint8_t RA_REG_BANK_SEL = 0x7F;
    constexpr std::uint8_t DEVICE_ID = 0xEA;

    namespace {

        constexpr auto GYRO_OUTPUT_RATE_DLPF_DIS_HZ = 9000UL;
        constexpr auto GYRO_OUTPUT_RATE_DLPF_EN_HZ = 1100UL;

        constexpr auto ACCEL_OUTPUT_RATE_DLPF_DIS_HZ = 4500UL;
        constexpr auto ACCEL_OUTPUT_RATE_DLPF_EN_HZ = 1125UL;

    }; // namespace

    inline std::uint16_t gyro_output_rate_to_smplrt_div(std::uint32_t const output_rate,
                                                        bool const dlpf_enabled = false) noexcept
    {
        return static_cast<std::uint16_t>((dlpf_enabled ? GYRO_OUTPUT_RATE_DLPF_EN_HZ : GYRO_OUTPUT_RATE_DLPF_DIS_HZ) /
                                          output_rate) -
               1U;
    }

    inline std::uint16_t accel_output_rate_to_smplrt_div(std::uint32_t const output_rate,
                                                         bool const dlpf_enabled = false) noexcept
    {
        return static_cast<std::uint16_t>(
                   (dlpf_enabled ? ACCEL_OUTPUT_RATE_DLPF_EN_HZ : ACCEL_OUTPUT_RATE_DLPF_DIS_HZ) / output_rate) -
               1U;
    }

}; // namespace ICM20948

#endif // ICM20948_CONFIG_HPP