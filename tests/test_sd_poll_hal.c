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
enum {NORMAL,PREEMPT_DONE,NO_RX,NO_TX,NO_EOT,PREEMPT_NO_RX,
      BMI_LATE_RX,BMI_FROZEN_RX,BMI_LATE_EOT,BMI_EMPTY_EOT,BMI_FIFO_TAIL};
static unsigned eot_checks;
static int flag(SPI_HandleTypeDef *s,unsigned f)
{
    if(f==SPI_FLAG_TXP){if(mode==NO_TX){fake_tick++;DWT->CYCCNT+=480000U;return 0;}return 1;}
    if(f==SPI_FLAG_EOT){
        if(mode==NO_EOT){fake_tick++;DWT->CYCCNT+=480000U;return 0;}
        if(mode==BMI_LATE_EOT && !eot_checks++){
            fake_tick+=2;DWT->CYCCNT+=960000U;return 0;
        }
        return 1;
    }
    if(f==SPI_FLAG_RXP){
        if((mode==PREEMPT_DONE || mode==PREEMPT_NO_RX) && !preempted){fake_tick+=7;preempted=1;}
        if(mode==NO_RX || mode==NO_TX || mode==PREEMPT_NO_RX || mode==BMI_EMPTY_EOT){
            fake_tick++;DWT->CYCCNT+=480000U;return 0;
        }
        if(mode==BMI_FROZEN_RX)return 0;
        if(mode==BMI_LATE_RX && !preempted)return 0;
        if(mode==BMI_FIFO_TAIL){s->Instance->SR|=SPI_SR_RXPLVL_0;s->Instance->RXDR=0xa5;return 0;}
        s->Instance->RXDR=0xa5;return 1;
    }
    return 1;
}
#define __HAL_SPI_GET_FLAG(s,f) flag((s),(f))
static void SPI_CloseTransfer(SPI_HandleTypeDef *s)
{
    if(s->Instance->SR&(SPI_SR_OVR|SPI_SR_UDR|SPI_SR_MODF|SPI_SR_TIFRE))s->ErrorCode|=HAL_SPI_ERROR_FLAG;
    s->Instance->CR1&=~1U;s->TxXferCount=s->RxXferCount=0;
}
static HAL_StatusTypeDef SPI_WaitOnFlagUntilTimeout(SPI_HandleTypeDef *s,unsigned f,unsigned state,unsigned timeout,unsigned start)
{(void)s;(void)f;(void)state;(void)start;if(mode==NO_EOT){fake_tick+=timeout;return HAL_TIMEOUT;}return HAL_OK;}
#include "../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_bmi088_spi2_poll_guard.h"
volatile Bmi088Spi2PollDiag_t g_bmi088_spi2_poll_diag;
#include "build/spi_poll_body.inc"
static SPI_HandleTypeDef *bmi_probe;
static unsigned bmi_hook_kind,bmi_disable_dwt;
static void bmi_tick_edge(void)
{
    if(bmi_probe->State!=HAL_SPI_STATE_BUSY_TX_RX || bmi_probe->TxXferCount)return;
    if(((bmi_hook_kind==3 || bmi_hook_kind==4) && bmi_probe->RxXferCount) ||
       ((bmi_hook_kind!=3 && bmi_hook_kind!=4) && !bmi_probe->RxXferCount)){
        test_tick_hook=NULL;preempted=1;
        fake_tick+=(bmi_hook_kind==1 || bmi_hook_kind==4)?1:2;
        DWT->CYCCNT+=(bmi_hook_kind==1 || bmi_hook_kind==4)?480:960000;
    }
}
static HAL_StatusTypeDef run_bmi(unsigned n,unsigned m,unsigned hook,unsigned hw_error)
{
    SPI_HandleTypeDef h={0};uint8_t tx[10],rx[10]={0};unsigned i;HAL_StatusTypeDef result;
    memset(tx,0xff,sizeof(tx));memset(SPI2,0,sizeof(*SPI2));
    h.Instance=SPI2;h.Init.Mode=SPI_MODE_MASTER;h.Init.DataSize=SPI_DATASIZE_8BIT;h.State=HAL_SPI_STATE_READY;
    mode=m;fake_tick=UINT32_MAX;preempted=eot_checks=0;
    DWT->CYCCNT=UINT32_MAX-100;DWT->CTRL=bmi_disable_dwt?0:DWT_CTRL_CYCCNTENA_Msk;
    SPI2->SR=hw_error;bmi_probe=&h;bmi_hook_kind=hook;
    test_tick_hook=hook?bmi_tick_edge:NULL;
    result=HAL_SPI_TransmitReceive(&h,tx,rx,(uint16_t)n,1);
    test_tick_hook=NULL;
    if(result==HAL_OK)for(i=0;i<n;i++)assert(rx[i]==0xa5);
    return result;
}
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
    puts("PASS SPI1 legacy guard, pending errors and unrelated SPI2(timeout=5ms)/SPI6 behavior");
    memset((void *)&g_bmi088_spi2_poll_diag,0,sizeof(g_bmi088_spi2_poll_diag));
    for(unsigned n=1;n<=10;n++){
        unsigned before=g_bmi088_spi2_poll_diag.tick_boundary_avoided_count;
        assert(run_bmi(n,NORMAL,1,0)==HAL_OK);
        assert(g_bmi088_spi2_poll_diag.tick_boundary_avoided_count==before+1);
    }
    assert(g_bmi088_spi2_poll_diag.revision==1 && !g_bmi088_spi2_poll_diag.first_fault.complete);
    assert(run_bmi(8,BMI_LATE_RX,4,0)==HAL_OK); /* Tick edge while RX still pending. */
    assert(run_bmi(1,NORMAL,2,0)==HAL_OK);
    assert(g_bmi088_spi2_poll_diag.completed_after_deadline_count==1);
    assert(run_bmi(1,BMI_LATE_RX,3,0)==HAL_OK);
    assert(g_bmi088_spi2_poll_diag.completed_after_deadline_count==2);
    assert(run_bmi(1,BMI_FIFO_TAIL,0,0)==HAL_OK);
    assert(run_bmi(1,BMI_LATE_EOT,0,0)==HAL_OK);
    assert(run_bmi(1,NO_TX,0,0)==HAL_TIMEOUT);
    assert(g_bmi088_spi2_poll_diag.first_fault.complete && g_bmi088_spi2_poll_diag.first_fault.phase==1);
    {Bmi088Spi2PollFault_t frozen=g_bmi088_spi2_poll_diag.first_fault;
     assert(run_bmi(1,NO_RX,0,0)==HAL_TIMEOUT);
     {Bmi088Spi2PollFault_t after=g_bmi088_spi2_poll_diag.first_fault;assert(!memcmp(&after,&frozen,sizeof(after)));}}
    g_bmi088_spi2_poll_diag.first_fault.complete=0;
    assert(run_bmi(1,NO_EOT,0,0)==HAL_ERROR);
    assert(g_bmi088_spi2_poll_diag.first_fault.phase==2);
    assert(run_bmi(1,BMI_EMPTY_EOT,0,0)==HAL_TIMEOUT);
    assert(run_bmi(1,BMI_FROZEN_RX,0,0)==HAL_TIMEOUT);
    bmi_disable_dwt=1;
    assert(run_bmi(1,NORMAL,1,0)==HAL_OK);
    assert(run_bmi(1,NO_RX,0,0)==HAL_TIMEOUT);
    assert(run_bmi(1,BMI_FROZEN_RX,0,0)==HAL_TIMEOUT);
    bmi_disable_dwt=0;
    {SPI_HandleTypeDef h={0};Bmi088Spi2PollGuard_t g;
     h.Init.Mode=SPI_MODE_MASTER;h.Init.DataSize=SPI_DATASIZE_8BIT;
     h.Instance=SPI1;Bmi088Spi2Poll_Begin(&g,&h,1,0);assert(!g.enabled);
     h.Instance=SPI6;Bmi088Spi2Poll_Begin(&g,&h,1,0);assert(!g.enabled);
     h.Instance=SPI2;Bmi088Spi2Poll_Begin(&g,&h,0,0);assert(!g.enabled);
     Bmi088Spi2Poll_Begin(&g,&h,HAL_MAX_DELAY,0);assert(!g.enabled);
     Bmi088Spi2Poll_Begin(&g,&h,5,0);assert(!g.enabled);
     h.Init.Mode=0;Bmi088Spi2Poll_Begin(&g,&h,1,0);assert(!g.enabled);
     h.Init.Mode=SPI_MODE_MASTER;h.Init.DataSize=SPI_DATASIZE_16BIT;
     Bmi088Spi2Poll_Begin(&g,&h,1,0);assert(!g.enabled);}
    for(unsigned i=0;i<4;i++){
        const unsigned flags[]={SPI_SR_OVR,SPI_SR_UDR,SPI_SR_MODF,SPI_SR_TIFRE};
        g_bmi088_spi2_poll_diag.first_fault.complete=0;
        assert(run_bmi(1,NORMAL,0,flags[i])==HAL_ERROR);
        assert(g_bmi088_spi2_poll_diag.first_fault.phase==3 &&
               (g_bmi088_spi2_poll_diag.first_fault.spi_sr&flags[i]));
    }
    assert(run_bmi(8,NORMAL,1,0)==HAL_OK);
    puts("PASS BMI088 actual HAL: 1..10-byte tick edges, DWT/tick wrap, late RX/EOT, FIFO tail, real TX/RX/EOT stalls, frozen clocks, HW errors, first-fault latch");
    return 0;
}
