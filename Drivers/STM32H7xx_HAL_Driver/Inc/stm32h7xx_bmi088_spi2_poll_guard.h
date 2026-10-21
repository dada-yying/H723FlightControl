/* Local BMI088 SPI2 polling guard. Only 8-bit master TxRx with the existing
 * 1ms timeout opts in. SPI1/SPI6 and other HAL transfer modes stay unchanged. */
#ifndef STM32H7XX_BMI088_SPI2_POLL_GUARD_H
#define STM32H7XX_BMI088_SPI2_POLL_GUARD_H
typedef struct {
    uint32_t complete, tick_ms, phase, elapsed_us;
    uint32_t tx_remaining, rx_remaining, spi_sr, spi_cr1, spi_cr2, spi_cfg1;
} Bmi088Spi2PollFault_t;
typedef struct {
    uint32_t revision, transfer_count, tick_boundary_avoided_count;
    uint32_t completed_after_deadline_count, timeout_count, hardware_error_count;
    Bmi088Spi2PollFault_t first_fault;
} Bmi088Spi2PollDiag_t;
extern volatile Bmi088Spi2PollDiag_t g_bmi088_spi2_poll_diag;
typedef struct {
    uint32_t cycle_start, tick_start, cycle_budget, loops;
    uint8_t enabled, use_dwt, tick_counted;
} Bmi088Spi2PollGuard_t;

static void Bmi088Spi2Poll_Begin(Bmi088Spi2PollGuard_t *g,
                               const SPI_HandleTypeDef *s,uint32_t timeout,uint32_t tick)
{
    g->enabled=(uint8_t)(s->Instance==SPI2 && s->Init.Mode==SPI_MODE_MASTER &&
                        s->Init.DataSize==SPI_DATASIZE_8BIT && timeout==1U);
    if(!g->enabled)return;
    g->cycle_start=DWT->CYCCNT;g->tick_start=tick;
    g->cycle_budget=SystemCoreClock/1000U;g->loops=0;g->tick_counted=0;
    g->use_dwt=(uint8_t)((DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk) && g->cycle_budget);
    g_bmi088_spi2_poll_diag.revision=1;
    g_bmi088_spi2_poll_diag.transfer_count++;
}
static int Bmi088Spi2Poll_Expired(Bmi088Spi2PollGuard_t *g)
{
    uint32_t tick_elapsed=(uint32_t)(HAL_GetTick()-g->tick_start);
    int expired=(g->use_dwt &&
                 (uint32_t)(DWT->CYCCNT-g->cycle_start)>=g->cycle_budget) ||
                tick_elapsed>1U || ++g->loops>g->cycle_budget+1U;
    /* >=1 tick can mean only microseconds in real time. DWT measures the
     * actual 1ms; >1 tick and finite loops also bound a disabled/stopped DWT. */
    if(tick_elapsed>=1U && !expired && !g->tick_counted){
        g->tick_counted=1;g_bmi088_spi2_poll_diag.tick_boundary_avoided_count++;
    }
    return expired;
}
static void Bmi088Spi2Poll_Fault(const Bmi088Spi2PollGuard_t *g,
                               const SPI_HandleTypeDef *s,uint32_t phase)
{
    volatile Bmi088Spi2PollFault_t *f=&g_bmi088_spi2_poll_diag.first_fault;
    if(f->complete)return;
    f->tick_ms=HAL_GetTick();f->phase=phase; /* 1=TX/RX stall, 2=EOT, 3=HW flags. */
    f->elapsed_us=g->use_dwt && SystemCoreClock>=1000000U ?
        (uint32_t)(DWT->CYCCNT-g->cycle_start)/(SystemCoreClock/1000000U):0;
    f->tx_remaining=s->TxXferCount;f->rx_remaining=s->RxXferCount;
    f->spi_sr=s->Instance->SR;f->spi_cr1=s->Instance->CR1;
    f->spi_cr2=s->Instance->CR2;f->spi_cfg1=s->Instance->CFG1;
    __DSB();f->complete=1;
}
/* Keep the original EOT check, but use the same precise/finite wait budget.
 * A completion appearing between the flag check and clock check wins. */
static HAL_StatusTypeDef Bmi088Spi2Poll_WaitEot(Bmi088Spi2PollGuard_t *g,SPI_HandleTypeDef *s)
{
    while(!__HAL_SPI_GET_FLAG(s,SPI_FLAG_EOT)){
        if(Bmi088Spi2Poll_Expired(g)){
            if(__HAL_SPI_GET_FLAG(s,SPI_FLAG_EOT))return HAL_OK;
            g_bmi088_spi2_poll_diag.timeout_count++;
            Bmi088Spi2Poll_Fault(g,s,2);return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}
#endif
