#include <assert.h>
#include <stdio.h>
#include "sd_poll_model.h"
#include "../App/storage/sd_spi.c"
uint32_t fake_tick;
static unsigned aborts;
static uint8_t wire_byte(uint8_t out){return (uint8_t)~out;}
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *s)
{aborts++;s->State=HAL_SPI_STATE_READY;memset(s->Instance,0,sizeof(*s->Instance));return HAL_OK;}
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *s,uint8_t *t,uint8_t *r,uint16_t n)
{(void)s;(void)t;(void)r;(void)n;assert(!"short transfers must not start DMA");return HAL_ERROR;}
static void reset_poll(unsigned mode)
{
    memset((void *)&g_sd_spi_diag,0,sizeof(g_sd_spi_diag));
    memset(SPI1,0,sizeof(*SPI1));hspi1_sd.Instance=SPI1;
    hspi1_sd.Init.Mode=SPI_MODE_MASTER;hspi1_sd.State=HAL_SPI_STATE_READY;
    hspi1_sd.ErrorCode=0;io_failed=dma_done=dma_error=0;transport=TRANSPORT_IDLE;
    io_operation=SD_IO_READ;poll_mode=mode;poll_preempted=0;
    fake_tick=0;DWT->CYCCNT=0;DWT->CTRL=DWT_CTRL_CYCCNTENA_Msk;
    test_tick_hook=NULL;poll_deadline_byte=0;
}
int main(void)
{
    uint8_t tx[6]={0x51,0,0,0,0,0xff},rx[6];unsigned i,j,blocked,clean;
    reset_poll(POLL_NORMAL);blocked=test_notify_block_count;clean=cleaned;
    for(i=0;i<256;i++){uint8_t in,out=(uint8_t)i;assert(poll_transfer(&out,&in,1));assert(in==(uint8_t)~out);}
    assert(poll_transfer(tx,rx,6));for(i=0;i<6;i++)assert(rx[i]==(uint8_t)~tx[i]);
    assert(g_sd_spi_diag.short_bytes==262 && !g_sd_spi_diag.byte_dma_count);
    assert(cleaned==clean && test_notify_block_count==blocked);
    assert(hspi1_sd.State==HAL_SPI_STATE_READY && !(SPI1->CR1&SPI_CR1_SPE));
    reset_poll(POLL_PREEMPT_DONE);fake_tick=UINT32_MAX-3;DWT->CYCCNT=UINT32_MAX-10;
    assert(poll_transfer(tx,rx,6) && poll_preempted && !g_sd_spi_diag.io_fault.complete);
    reset_poll(POLL_PREEMPT_MIDDLE);
    assert(poll_transfer(tx,rx,6) && poll_preempted && !g_sd_spi_diag.io_fault.complete);
    assert(g_sd_spi_diag.short_progress_recoveries==1);
    reset_poll(POLL_PREEMPT_MIDDLE);fake_tick=UINT32_MAX-3;DWT->CYCCNT=UINT32_MAX-10;
    assert(poll_transfer(tx,rx,6) && g_sd_spi_diag.short_progress_recoveries==1);
    reset_poll(POLL_DEADLINE_RX);
    assert(poll_transfer(tx,rx,1) && poll_preempted && rx[0]==(uint8_t)~tx[0]);
    assert(!g_sd_spi_diag.io_fault.complete && g_sd_spi_diag.short_deadline_rechecks==1);
    reset_poll(POLL_DEADLINE_RX);fake_tick=UINT32_MAX-3;DWT->CYCCNT=UINT32_MAX-10;
    assert(poll_transfer(tx,rx,6) && poll_preempted && g_sd_spi_diag.short_deadline_rechecks==1);
    for(i=0;i<6;i++)assert(rx[i]==(uint8_t)~tx[i]);
    puts("PASS exact revision 7 regression: SR=0x301F completes after RXP test / before deadline, no false timeout");
    for(j=1;j<=6;j++){
        reset_poll(POLL_DEADLINE_RX);poll_deadline_byte=j;
        assert(poll_transfer(tx,rx,6) && g_sd_spi_diag.short_deadline_rechecks==1);
        for(i=0;i<6;i++)assert(rx[i]==(uint8_t)~tx[i]);
    }
    reset_poll(POLL_FIFO_TAIL);
    assert(poll_transfer(tx,rx,6) && g_sd_spi_diag.short_fifo_tail_reads==6);
    for(i=0;i<6;i++)assert(rx[i]==(uint8_t)~tx[i]);
    reset_poll(POLL_EMPTY_EOT);memset(rx,0xa5,sizeof(rx));
    assert(!poll_transfer(tx,rx,1) && rx[0]==0xa5);
    assert(g_sd_spi_diag.short_fault_sent==1 && !g_sd_spi_diag.short_fault_received);
    reset_poll(POLL_DEADLINE_ERROR);
    assert(!poll_transfer(tx,rx,1) && poll_preempted);
    assert(g_sd_spi_diag.io_fault.kind==SD_IO_FAULT_SHORT_HW &&
           (g_sd_spi_diag.short_fault_status&SPI_SR_OVR));
    reset_poll(POLL_DEADLINE_EOT);
    assert(poll_transfer(tx,rx,1) && poll_preempted && g_sd_spi_diag.short_deadline_rechecks==1);
    for(i=POLL_NO_TX;i<=POLL_NO_EOT;i++){
        reset_poll(i);j=aborts;assert(!poll_transfer(tx,rx,6));
        assert(aborts==j+1 && io_failed && g_sd_spi_diag.short_timeout_count==1);
        assert(g_sd_spi_diag.io_fault.kind==SD_IO_FAULT_SHORT_TIMEOUT && g_sd_spi_diag.io_fault.complete);
        assert(g_sd_spi_diag.io_fault.transfer_size==6 && transport==TRANSPORT_IDLE);
        {SdSpiIoFault_t frozen=g_sd_spi_diag.io_fault;
         io_failed=0;assert(!poll_transfer(tx,rx,6));
         {SdSpiIoFault_t after=g_sd_spi_diag.io_fault;assert(!memcmp(&after,&frozen,sizeof(after)));}}
    }
    for(i=0;i<4;i++){
        static const uint32_t flags[]={SPI_SR_OVR,SPI_SR_UDR,SPI_SR_MODF,SPI_SR_TIFRE};
        reset_poll(POLL_ERROR);poll_error_flag=flags[i];assert(!poll_transfer(tx,rx,6));
        assert(g_sd_spi_diag.io_fault.kind==SD_IO_FAULT_SHORT_HW);
        assert(g_sd_spi_diag.io_fault.spi_sr&flags[i]);
    }
    reset_poll(POLL_FROZEN);DWT->CTRL=0;
    assert(!poll_transfer(tx,rx,6) && fake_tick==0 && g_sd_spi_diag.short_timeout_count==1);
    reset_poll(POLL_NORMAL);transport=TRANSPORT_DMA;
    assert(!poll_transfer(tx,rx,6) && g_sd_spi_diag.io_fault.kind==SD_IO_FAULT_TRANSPORT_BUSY);
    reset_poll(POLL_NORMAL);SPI1->IER=8;
    assert(!poll_transfer(tx,rx,6) && g_sd_spi_diag.io_fault.kind==SD_IO_FAULT_TRANSPORT_BUSY);
    reset_poll(POLL_NORMAL);assert(poll_transfer(tx,rx,6));
    puts("PASS production register polling: 1/6 bytes, no DMA/cache/sleep, preemption/wrap, TX/RX/EOT timeout, OVR/UDR/MODF/TIFRE, frozen clocks, owner exclusion, fault freeze/recovery");
    return 0;
}
