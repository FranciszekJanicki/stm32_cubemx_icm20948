#include "gpio.h"
#include "i2c.h"
#include "icm_sensor.h"
#include "main.h"
#include "usart.h"
#include <cstdio>

int main()
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    icm_sensor_init();

    icm_sensor_data_t data = {0};

    while (true) {
        icm_sensor_read(&data);

        printf("x: %f, y: %f, z: %f\n\r",
               (double)data.q1 * Q_SCALE,
               (double)data.q2 * Q_SCALE,
               (double)data.q3 * Q_SCALE);

        HAL_Delay(100);
    }
}
