/* Includes the function extracted from the ACTUAL production HAL source by
 * run_sd_poll.ps1. Model a 7ms preemption exactly before the final RX read. */
#include "stm32h7xx_hal.h"
#include <assert.h>
#include <stdio.h>
#define __IO volatile
#define assert_param(x) ((void)0)
#define __HAL_LOCK(s) ((void)0)
#define __HAL_UNLOCK(s) ((void)0)
#define SPI_2LINES(s) ((void)0)
#define MODIFY_REG(r,m,v) ((r)=((r)&~(m))|(v))
#define SET_BIT(r,b) ((r)|=(b))
#define __HAL_SPI_ENABLE(s) SET_BIT((s)->Instance->CR1,1)
#define SPI_CR2_TSIZE 65535U
#define SPI_CR1_CSTART 512U
#define SPI_FLAG_TXP 2U
#define SPI_FLAG_RXP 1U
#define SPI_FLAG_EOT 8U
#define SPI_FLAG_RXWNE 32768U
#define SPI_DATASIZE_16BIT 15U
#define HAL_MAX_DELAY UINT32_MAX
#define HAL_SPI_STATE_READY 1U
#define HAL_SPI_STATE_BUSY_TX_RX 5U
#define HAL_SPI_ERROR_NONE 0U
#define HAL_SPI_ERROR_TIMEOUT 256U
#define HAL_SPI_ERROR_FLAG 32U
#define RESET 0
uint32_t fake_tick;
volatile uint32_t g_spi1_completed_after_deadline_count;
static unsigned mode,preempted;
enum {NORMAL,PREEMPT_DONE,NO_RX,NO_TX,NO_EOT,PREEMPT_NO_RX};
static int flag(SPI_HandleTypeDef *s,unsigned f)
{
    if(f==SPI_FLAG_TXP)return mode!=NO_TX;
    if(f==SPI_FLAG_RXP){
        if((mode==PREEMPT_DONE || mode==PREEMPT_NO_RX) && !preempted){fake_tick+=7;preempted=1;}
        if(mode==NO_RX || mode==NO_TX || mode==PREEMPT_NO_RX){fake_tick++;return 0;}
        s->Instance->RXDR=0xa5;return 1;
    }
    return 1;
}
#define __HAL_SPI_GET_FLAG(s,f) flag((s),(f))
static void SPI_CloseTransfer(SPI_HandleTypeDef *s)
{s->Instance->CR1&=~1U;s->TxXferCount=s->RxXferCount=0;}
static HAL_StatusTypeDef SPI_WaitOnFlagUntilTimeout(SPI_HandleTypeDef *s,unsigned f,unsigned state,unsigned timeout,unsigned start)
{(void)s;(void)f;(void)state;(void)start;if(mode==NO_EOT){fake_tick+=timeout;return HAL_TIMEOUT;}return HAL_OK;}
#include "build/spi_poll_body.inc"
static HAL_StatusTypeDef run(SPI_TypeDef *spi,unsigned m)
{
    SPI_HandleTypeDef h={0};uint8_t tx=0xff,rx=0;
    h.Instance=spi;h.Init.Mode=SPI_MODE_MASTER;h.Init.DataSize=SPI_DATASIZE_8BIT;h.State=HAL_SPI_STATE_READY;
    mode=m;fake_tick=preempted=0;memset(spi,0,sizeof(*spi));
    {HAL_StatusTypeDef result=HAL_SPI_TransmitReceive(&h,&tx,&rx,1,5);
     if(m==NORMAL || m==PREEMPT_DONE)assert(rx==0xa5);
     if(result==HAL_TIMEOUT)assert(h.ErrorCode&HAL_SPI_ERROR_TIMEOUT);
     return result;}
}
int main(void)
{
    assert(run(SPI1,NORMAL)==HAL_OK);
#ifdef SD_POLL_EXPECT_OLD
    assert(run(SPI1,PREEMPT_DONE)==HAL_TIMEOUT);
    puts("REPRODUCED: production HAL returns TIMEOUT after TX/RX both complete following a 7ms preemption");
#else
    assert(run(SPI1,PREEMPT_DONE)==HAL_OK && g_spi1_completed_after_deadline_count==1);
    puts("PASS SPI1: 7ms preemption after byte completion no longer becomes a false TIMEOUT");
#endif
    assert(run(SPI1,NO_RX)==HAL_TIMEOUT);
    assert(run(SPI1,NO_TX)==HAL_TIMEOUT);
    assert(run(SPI1,PREEMPT_NO_RX)==HAL_TIMEOUT);
    assert(run(SPI1,NO_EOT)==HAL_ERROR);
    assert(run(SPI2,PREEMPT_DONE)==HAL_TIMEOUT);
    assert(run(SPI6,PREEMPT_DONE)==HAL_TIMEOUT);
    puts("PASS real pending TX/RX/EOT failures still fail; SPI2 and SPI6 retain original HAL behavior");return 0;
}
