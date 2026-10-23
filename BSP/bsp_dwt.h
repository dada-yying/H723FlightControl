#ifndef BSP_DWT_H
#define BSP_DWT_H
#include "stm32h7xx_hal.h"
void BspDwt_Init(void);
/* Enable-only: safe during runtime and early fault handling. */
void BspDwt_Enable(void);
static __inline uint32_t BspDwt_Cycles(void)
{
    return DWT->CYCCNT;
}
static __inline uint32_t BspDwt_ElapsedCycles(uint32_t start)
{
    return (uint32_t)(BspDwt_Cycles() - start);
}
static __inline uint32_t BspDwt_Us(uint32_t cycles)
{
    return cycles / (SystemCoreClock / 1000000U);
}
static __inline uint32_t BspDwt_IsEnabled(void)
{
    return DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk;
}
#endif
