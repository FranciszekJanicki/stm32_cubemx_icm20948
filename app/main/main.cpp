#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "i2c_device.hpp"
#include "icm20948.hpp"
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

    auto i2c_device = I2CDevice{&hi2c1, 0U};

    auto icm20948 = ICM20948::ICM20948{std::move(i2c_device)};

    while (true) {
    }

    return 0;
}
