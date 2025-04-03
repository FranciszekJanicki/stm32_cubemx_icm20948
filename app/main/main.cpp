#include "main.h"
#include "gpio.h"
#include "gpio.hpp"
#include "i2c.h"
#include "i2c_device.hpp"
#include "icm20948.hpp"
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
    //  if (GPIO_Pin == GPIO_PIN_5) {
    std::puts("GPIO EXTI 5 CALLBACK");
    gpio_pin5_exti = true;
    // }
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

    using namespace ICM20948;

    auto i2c_device = I2CDevice{&hi2c1, ICM_20948_I2C_ADDR_AD1};
    i2c_device.bus_scan();

    auto icm20948 = ICM20948::ICM20948{std::move(i2c_device)};

    while (1) {
        if (gpio_pin5_exti) {
            auto const& [r, p, y] = icm20948.get_roll_pitch_yaw().value();
            std::printf("roll: %f, pitch: %f, yaw: %f\n\r", r, p, y);
            gpio_pin5_exti = false;
        }
    }
}
