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

    auto ak09916_i2c_device = I2CDevice{&hi2c1, std::to_underlying(DevAddress::AD0_LOW)};

    auto ak09916 = AK09916::AK09916{std::move(ak09916_i2c_device),
                                    AK09916::CONTROL_1{},
                                    AK09916::CONTROL_2{.mode = std::to_underlying(AK09916::Mode::CONTINUOUS_1)},
                                    AK09916::CONTROL_3{.srst = true}};

    auto icm20948_i2c_device = I2CDevice{&hi2c1, std::to_underlying(DevAddress::AD0_LOW)};

    auto bank0_config = Bank0::Config{};

    auto bank1_config = Bank1::Config{};

    auto bank2_config = Bank2::Config{};

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
