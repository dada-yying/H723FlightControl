/* Local SPI1 SD workaround for STM32H72xx/73xx ES0491 section 2.21.5.
 * HAL_SPI_IRQHandler clears EOT before SPI_CloseTransfer, so do not test EOT
 * here. Delay also on error close; it is finite and never masks interrupts.
 * The current SPI1 SCK is 7.5 MHz. >=1 us exceeds one complete SCK period.
 * SPI2 (BMI088) and SPI6 are deliberately excluded. */
#ifndef STM32H7XX_SPI1_EOT_GUARD_H
#define STM32H7XX_SPI1_EOT_GUARD_H
extern volatile uint32_t g_spi1_completed_after_deadline_count;
static void SPI1_SD_EotGuard(const SPI_HandleTypeDef *hspi)
{
    uint32_t n;
    if(hspi->Instance!=SPI1 || hspi->Init.Mode!=SPI_MODE_MASTER)return;
    /* Every loop executes at least one NOP core cycle; loop overhead increases
     * the margin. Avoid a dependency on DWT being enabled or advancing. */
    n=SystemCoreClock/1000000U+1U;
    while(n--){__NOP();}
    __DSB();
}
#endif
