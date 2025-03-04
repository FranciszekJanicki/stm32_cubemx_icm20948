#include "main.h"
#include "ak09916_config.hpp"
#include "ak09916_registers.hpp"
#include "gpio.h"
#include "gpio.hpp"
#include "i2c.h"
#include "i2c_device.hpp"
#include "icm20948.hpp"
#include "icm20948_config.hpp"
#include "icm20948_mag.hpp"
#include "usart.h"
#include <cstdio>

namespace {

    volatile auto interrupt = false;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ICM_INT_Pin) {
        interrupt = true;
    }
}

int main()
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    using namespace Utility;
    using namespace ICM20948;

    auto constexpr GYRO_SAMPLING_RATE_HZ = 225UL;
    auto constexpr ACCEL_SAMPLING_RATE_HZ = 225UL;

    auto i2c_device = I2CDevice{&hi2c1, std::to_underlying(DevAddress::AD0_LOW)};

    auto user_ctrl = USER_CTRL{.dmp_en = false,
                               .fifo_en = false,
                               .i2c_mst_en = false,
                               .i2c_if_dis = false,
                               .dmp_rst = false,
                               .sram_rst = false,
                               .i2c_mst_rst = false};

    auto lp_config = LP_CONFIG{.i2c_mst_cycle = false, .accel_cycle = true, .gyro_cycle = true};

    auto pwr_mgmt_1 = PWR_MGMT_1{.device_reset = false,
                                 .sleep = false,
                                 .lp_en = false,
                                 .temp_dis = false,
                                 .clksel = std::to_underlying(ClockSource::AUTO_SELECT)};

    auto pwr_mgmt_2 = PWR_MGMT_2{.disable_accel = 0b000, .disable_gyro = 0b000};

    auto int_pin_cfg = INT_PIN_CFG{.int1_actl = std::to_underlying(IntMode::ACTIVEHIGH),
                                   .int1_open = std::to_underlying(IntDrive::PUSHPULL),
                                   .int1_latch_en = std::to_underlying(IntLatch::PULSE50US),
                                   .int_anyrd_2clear = std::to_underlying(IntClear::ANYREAD),
                                   .actl_fsync = false,
                                   .fsync_int_mode_en = false,
                                   .bypass_en = false};

    auto int_enable = INT_ENABLE{.reg_wof_en = false,
                                 .wom_int_en = false,
                                 .pll_rdy_en = false,
                                 .dmp_int1_en = false,
                                 .i2c_mst_int_en = false};

    auto int_enable_1 = INT_ENABLE_1{.raw_data_0_rdy_en = true};

    auto int_enable_2 = INT_ENABLE_2{.fifo_overflow_en = false};

    auto int_enable_3 = INT_ENABLE_3{.fifo_wm_en = false};

    auto gyro_smplrt_div = GYRO_SMPLRT_DIV{.gyro_smplrt_div = sampling_rate_to_gyro_smplrt_div(GYRO_SAMPLING_RATE_HZ)};

    auto gyro_config_1 = GYRO_CONFIG_1{.gyro_dplfcfg = std::to_underlying(GyroDLPF::BW_361),
                                       .gyro_fs_sel = std::to_underlying(GyroRange::GYRO_FS_250),
                                       .gyro_fchoice = false};

    auto gyro_config_2 = GYRO_CONFIG_2{.xgyro_cten = false,
                                       .ygyro_cten = false,
                                       .zgyro_cten = false,
                                       .gyro_avgcfg = std::to_underlying(GyroFIR::BW_774)};

    auto accel_smplrt_div =
        ACCEL_SMPLRT_DIV{.accel_smplrt_div = sampling_rate_to_accel_smplrt_div(ACCEL_SAMPLING_RATE_HZ)};

    auto accel_config_1 = ACCEL_CONFIG_1{.accel_dlpfcfg = std::to_underlying(AccelDLPF::BW_499),
                                         .accel_fs_sel = std::to_underlying(AccelRange::ACCEL_FS_2),
                                         .accel_fchoice = false};

    auto accel_config_2 = ACCEL_CONFIG_2{.ax_st_en_reg = false,
                                         .ay_st_en_reg = false,
                                         .az_st_en_reg = false,
                                         .dec3_cfg = std::to_underlying(AccelFIR::BW_1238)};

    auto icm20948 = ICM20948::ICM20948{std::move(i2c_device),
                                       Bank0::Config{.user_ctrl = user_ctrl,
                                                     .lp_config = lp_config,
                                                     .pwr_mgmt_1 = pwr_mgmt_1,
                                                     .pwr_mgmt_2 = pwr_mgmt_2,
                                                     .int_enable = int_enable,
                                                     .int_enable_1 = int_enable_1,
                                                     .int_enable_2 = int_enable_2,
                                                     .int_enable_3 = int_enable_3},
                                       Bank1::Config{},
                                       Bank2::Config{.gyro_smplrt_div = gyro_smplrt_div,
                                                     .gyro_config_1 = gyro_config_1,
                                                     .gyro_config_2 = gyro_config_2,
                                                     .accel_smplrt_div = accel_smplrt_div,
                                                     .accel_config_1 = accel_config_1,
                                                     .accel_config_2 = accel_config_2},
                                       Bank3::Config{}};

    while (true) {
        if (interrupt) {
            if (auto const& accel = icm20948.get_acceleration_scaled(); accel.has_value()) {
                auto const& [ax, ay, az] = accel.value();
                std::printf("ax: %f, ay: %f, az: %f\n\r", ax, ay, az);
            }

            if (auto const& gyro = icm20948.get_rotation_scaled(); gyro.has_value()) {
                auto const& [gx, gy, gz] = gyro.value();
                std::printf("gx: %f, gy: %f, gz: %f\n\r", gx, gy, gz);
            }

            interrupt = false;
        }
    }

    return 0;
}
