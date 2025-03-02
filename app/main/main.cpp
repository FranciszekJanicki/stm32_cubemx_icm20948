#include "main.h"
#include "ak09916_config.hpp"
#include "gpio.h"
#include "i2c.h"
#include "i2c_device.hpp"
#include "icm20948.hpp"
#include "icm20948_config.hpp"
#include "usart.h"

int main()
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    using namespace Utility;
    using namespace ICM20948;
    using namespace AK09916;

    auto constexpr SAMPLING_RATE = 200UL;

    auto ak09916_i2c_device = I2CDevice{&hi2c1, std::to_underlying(DevAddress::AD0_LOW)};

    auto ak09916 = AK09916::AK09916{std::move(ak09916_i2c_device),
                                    CONTROL_1{},
                                    CONTROL_2{.mode = std::to_underlying(AK09916::Mode::CONTINUOUS_1)},
                                    CONTROL_3{.srst = true}};

    auto icm20948_i2c_device = I2CDevice{&hi2c1, std::to_underlying(DevAddress::AD0_LOW)};

    auto bank0_config =
        Bank0::Config{.user_ctrl = USER_CTRL{.dmp_en = true,
                                             .fifo_en = true,
                                             .i2c_mst_en = false,
                                             .i2c_if_dis = false,
                                             .dmp_rst = true,
                                             .sram_rst = true,
                                             .i2c_mst_rst = true},
                      .lp_config = LP_CONFIG{.i2c_mst_cycle = false, .accel_cycle = true, .gyro_cycle = true},
                      .pwr_mgmt_1 = PWR_MGMT_1{.device_reset = true,
                                               .sleep = false,
                                               .lp_en = false,
                                               .temp_dis = true,
                                               .clksel = std::to_underlying(ClockSource::INTERNAL_20MHZ)},
                      .pwr_mgmt_2 = PWR_MGMT_2{.disable_accel = 0b000, .disable_gyro = 0b000},
                      .int_pin_cfg = INT_PIN_CFG{.int1_actl = std::to_underlying(IntMode::ACTIVEHIGH),
                                                 .int1_open = std::to_underlying(IntDrive::PUSHPULL),
                                                 .int1_latch_en = std::to_underlying(IntLatch::PULSE50US),
                                                 .int_anyrd_2clear = std::to_underlying(IntClear::ANYREAD),
                                                 .actl_fsync = false,
                                                 .fsync_int_mode_en = false,
                                                 .bypass_en = false},
                      .int_enable = INT_ENABLE{.reg_wof_en = false,
                                               .wom_int_en = false,
                                               .pll_rdy_en = false,
                                               .dmp_int1_en = true,
                                               .i2c_mst_int_en = false}};

    auto bank1_config = Bank1::Config{};

    auto bank2_config = Bank2::Config{
        .gyro_smplrt_div = GYRO_SMPLRT_DIV{.gyro_smplrt_div = sampling_rate_to_gyro_smplrt_div(SAMPLING_RATE)},
        .gyro_config_1 = GYRO_CONFIG_1{.gyro_dplfcfg = std::to_underlying(GyroDLPF::DISABLED),
                                       .gyro_fs_sel = std::to_underlying(GyroRange::GYRO_FS_250),
                                       .gyro_fchoice = true},
        .gyro_config_2 = GYRO_CONFIG_2{.xgyro_cten = false,
                                       .ygyro_cten = false,
                                       .zgyro_cten = false,
                                       .gyro_avgcfg = std::to_underlying(GyroFIR::DISABLED)},
        .accel_smplrt_div = ACCEL_SMPLRT_DIV{.accel_smplrt_div = sampling_rate_to_accel_smplrt_div(SAMPLING_RATE)},
        .accel_config_1 = ACCEL_CONFIG_1{.accel_dlpfcfg = std::to_underlying(AccelDLPF::DISABLED),
                                         .accel_fs_sel = std::to_underlying(AccelRange::ACCEL_FS_2),
                                         .accel_fchoice = true},
        .accel_config_2 = ACCEL_CONFIG_2{.ax_st_en_reg = false,
                                         .ay_st_en_reg = false,
                                         .az_st_en_reg = false,
                                         .dec3_cfg = std::to_underlying(AccelFIR::DISABLED)}};

    auto bank3_config = Bank3::Config{};

    auto icm20948 = ICM20948::ICM20948{std::move(icm20948_i2c_device),
                                       std::move(ak09916),
                                       bank0_config,
                                       bank1_config,
                                       bank2_config,
                                       bank3_config};

    while (true) {
    }

    return 0;
}
