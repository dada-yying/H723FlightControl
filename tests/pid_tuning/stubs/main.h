#pragma once
#include "stm32h7xx_hal.h"
#define GPIO_PIN_0 1U
#define GPIO_PIN_3 8U
#define BMI088_ACC_CS_Pin GPIO_PIN_0
#define BMI088_GYRO_CS_Pin GPIO_PIN_3
#define GPIOC ((void *)0x40020800U)
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
static uint16_t test_spi_cs;
static uint32_t test_spi_cs_release_count;
static void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state)
{
    (void)port;
    if (state == GPIO_PIN_SET) { test_spi_cs = 0; test_spi_cs_release_count++; }
    else test_spi_cs = pin;
}
