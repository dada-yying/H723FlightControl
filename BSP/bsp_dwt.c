#include "bsp_dwt.h"
#include "FreeRTOS.h"
#include "task.h"
static uint8_t s_reset_done;
void BspDwt_Enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
void BspDwt_Init(void)
{
    BspDwt_Enable();
    /* Startup owner only. Runtime calls may enable, but must never reset. */
    if ((s_reset_done == 0U) && (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED))
    {
        DWT->CYCCNT = 0U;
        s_reset_done = 1U;
    }
}
