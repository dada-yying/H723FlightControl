#include "stm32h7xx_hal.h"
#include "../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_spi1_eot_guard.h"
#include <assert.h>
#include <stdio.h>
uint32_t fake_tick;
int main(void)
{
    SPI_HandleTypeDef s={0};
    s.Instance=SPI1;s.Init.Mode=SPI_MODE_MASTER;DWT->CTRL=0;fake_nops=0;
    SPI1_SD_EotGuard(&s);
    assert(fake_nops>=SystemCoreClock/1000000U);
    fake_nops=0;s.Instance=SPI2;SPI1_SD_EotGuard(&s);assert(!fake_nops);
    s.Instance=SPI6;SPI1_SD_EotGuard(&s);assert(!fake_nops);
    s.Instance=SPI1;s.Init.Mode=0;SPI1_SD_EotGuard(&s);assert(!fake_nops);
    puts("PASS EOT guard: finite >=1us margin for SD SPI1, no DWT dependency, SPI2/SPI6 excluded");return 0;
}
