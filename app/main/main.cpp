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

    inline auto volatile gpio_pin5_exti = false;

}; // namespace

#ifdef __cplusplus
extern "C" {
#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_5) {
        std::puts("GPIO EXTI 5 CALLBACK");
        gpio_pin5_exti = true;
    }
}

#ifdef __cplusplus
}
#endif

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

    auto i2c_device = I2CDevice{&hi2c1, std::to_underlying(DevAddress::AD0_HIGH)};

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
                                 .clksel = std::to_underlying(ClockSource::INTERNAL_20MHZ)};

    auto pwr_mgmt_2 = PWR_MGMT_2{.disable_accel = 0U, .disable_gyro = 0U};

    auto int_pin_cfg = INT_PIN_CFG{.int1_actl = std::to_underlying(IntMode::ACTIVEHIGH),
                                   .int1_open = std::to_underlying(IntDrive::PUSHPULL),
                                   .int1_latch_en = std::to_underlying(IntLatch::PULSE50US),
                                   .int_anyrd_2clear = std::to_underlying(IntClear::STATUSREAD),
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

    auto fifo_en_1 =
        FIFO_EN_1{.slv_3_fifo_en = false, .slv_2_fifo_en = false, .slv_1_fifo_en = false, .slv_0_fifo_en = false};

    auto fifo_en_2 = FIFO_EN_2{.accel_fifo_en = false,
                               .gyro_z_fifo_en = false,
                               .gyro_y_fifo_en = false,
                               .gyro_x_fifo_en = false,
                               .temp_fifo_en = false};

    auto fifo_mode = FIFO_MODE{.fifo_mode = false};

    auto fifo_cfg = FIFO_CFG{.fifo_cfg = false};

    auto bank0_config = Bank0::Config{.user_ctrl = user_ctrl,
                                      .lp_config = lp_config,
                                      .pwr_mgmt_1 = pwr_mgmt_1,
                                      .pwr_mgmt_2 = pwr_mgmt_2,
                                      .int_pin_cfg = int_pin_cfg,
                                      .int_enable = int_enable,
                                      .int_enable_1 = int_enable_1,
                                      .int_enable_2 = int_enable_2,
                                      .int_enable_3 = int_enable_3,
                                      .fifo_en_1 = fifo_en_1,
                                      .fifo_en_2 = fifo_en_2,
                                      .fifo_cfg = fifo_cfg};

    auto xa_offs = XA_OFFS{.xa_offs = 0U};

    auto ya_offs = YA_OFFS{.ya_offs = 0U};

    auto za_offs = ZA_OFFS{.za_offs = 0U};

    auto timebase_correction_pll = TIMEBASE_CORRECTION_PLL{.tbc_pll = 0U};

    auto bank1_config = Bank1::Config{.xa_offs = xa_offs,
                                      .ya_offs = ya_offs,
                                      .za_offs = za_offs,
                                      .timebase_correction_pll = timebase_correction_pll};

    auto gyro_smplrt_div =
        GYRO_SMPLRT_DIV{.gyro_smplrt_div = 1U}; // gyro_output_rate_to_smplrt_div(GYRO_SAMPLING_RATE_HZ)};

    auto gyro_config_1 = GYRO_CONFIG_1{.gyro_dplfcfg = std::to_underlying(GyroDLPF::BW_197),
                                       .gyro_fs_sel = std::to_underlying(GyroRange::GYRO_FS_250),
                                       .gyro_fchoice = false};

    auto gyro_config_2 = GYRO_CONFIG_2{.xgyro_cten = false,
                                       .ygyro_cten = false,
                                       .zgyro_cten = false,
                                       .gyro_avgcfg = std::to_underlying(GyroFIR::BW_774)};

    auto xg_offs_usr = XG_OFFS_USR{.xg_offs_usr = 0U};

    auto yg_offs_usr = YG_OFFS_USR{.yg_offs_usr = 0U};

    auto zg_offs_usr = ZG_OFFS_USR{.zg_offs_usr = 0U};

    auto odr_align_en = ODR_ALIGN_EN{.odr_align_en = false};

    auto accel_smplrt_div =
        ACCEL_SMPLRT_DIV{.accel_smplrt_div = 1U}; // accel_output_rate_to_smplrt_div(ACCEL_SAMPLING_RATE_HZ)};

    auto accel_intel_ctrl = ACCEL_INTEL_CTRL{.accel_intel_en = false, .accel_intel_mode_int = 0U};

    auto accel_wom_thr = ACCEL_WOM_THR{.wom_threshold = 0U};

    auto accel_config_1 = ACCEL_CONFIG_1{.accel_dlpfcfg = std::to_underlying(AccelDLPF::BW_265),
                                         .accel_fs_sel = std::to_underlying(AccelRange::ACCEL_FS_2),
                                         .accel_fchoice = false};

    auto accel_config_2 = ACCEL_CONFIG_2{.ax_st_en_reg = false,
                                         .ay_st_en_reg = false,
                                         .az_st_en_reg = false,
                                         .dec3_cfg = std::to_underlying(AccelFIR::BW_1238)};

    auto fsync_config =
        FSYNC_CONFIG{.delay_time_en = false, .wof_deglitch_en = false, .wof_edge_int = false, .ext_sync_set = 0U};

    auto temp_config = TEMP_CONFIG{.temp_dlpfcfg = std::to_underlying(TempDLPF::BW_7392)};

    auto mod_ctrl_usr = MOD_CTRL_USR{.reg_lp_dmp_en = false};

    auto bank2_config = Bank2::Config{.gyro_smplrt_div = gyro_smplrt_div,
                                      .gyro_config_1 = gyro_config_1,
                                      .gyro_config_2 = gyro_config_2,
                                      .xg_offs_usr = xg_offs_usr,
                                      .yg_offs_usr = yg_offs_usr,
                                      .zg_offs_usr = zg_offs_usr,
                                      .odr_align_en = odr_align_en,
                                      .accel_smplrt_div = accel_smplrt_div,
                                      .accel_intel_ctrl = accel_intel_ctrl,
                                      .accel_wom_thr = accel_wom_thr,
                                      .accel_config_1 = accel_config_1,
                                      .accel_config_2 = accel_config_2,
                                      .fsync_config = fsync_config,
                                      .temp_config = temp_config,
                                      .mod_ctrl_usr = mod_ctrl_usr};

    auto i2c_mst_odr_config = I2C_MST_ODR_CONFIG{.i2c_mst_odr_config = 0U};

    auto i2c_mst_ctrl = I2C_MST_CTRL{.mult_mst_en = false, .i2c_mst_p_nsr = false, .i2c_mst_clk = 0U};

    auto i2c_mst_delay_ctrl = I2C_MST_DELAY_CTRL{.delay_es_shadow = false,
                                                 .i2c_slv4_delay_en = false,
                                                 .i2c_slv3_delay_en = false,
                                                 .i2c_slv2_delay_en = false,
                                                 .i2c_slv1_delay_en = false,
                                                 .i2c_slv0_delay_en = false};

    auto bank3_config = Bank3::Config{.i2c_mst_odr_config = i2c_mst_odr_config,
                                      .i2c_mst_ctrl = i2c_mst_ctrl,
                                      .i2c_mst_delay_ctrl = i2c_mst_delay_ctrl};

    auto icm20948 = ICM20948::ICM20948{std::move(i2c_device), bank0_config, bank1_config, bank2_config, bank3_config};

    while (true) {
        if (gpio_pin5_exti) {
            auto const& [ax, ay, az] = icm20948.get_acceleration_raw().value();
            std::printf("ax: %f, ay: %f, az: %f\n\r", ax, ay, az);

            auto const& [gx, gy, gz] = icm20948.get_rotation_raw().value();
            std::printf("gx: %f, gy: %f, gz: %f\n\r", gx, gy, gz);

            gpio_pin5_exti = false;
        }
    }

    return 0;
}
