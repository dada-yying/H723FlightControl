#pragma once
#include "stm32h7xx_hal.h"
typedef struct { uint32_t ErrorCode; } SPI_HandleTypeDef;
static SPI_HandleTypeDef hspi2;
static HAL_StatusTypeDef (*test_spi_hook)(const uint8_t *, uint8_t *, uint16_t, uint32_t);
static uint32_t test_spi_calls;
static HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, const uint8_t *tx,
                                               uint8_t *rx, uint16_t size, uint32_t timeout)
{
    (void)spi;
    test_spi_calls++;
    if (test_spi_hook) return test_spi_hook(tx, rx, size, timeout);
    return HAL_ERROR;
}
