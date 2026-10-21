/* Register model around the actual production polling loop; SPI wire handling
 * is supplied by the test. No replacement of poll_transfer(). */
#include "stm32h7xx_hal.h"
enum {POLL_NORMAL,POLL_NO_TX,POLL_NO_RX,POLL_NO_EOT,POLL_ERROR,
      POLL_PREEMPT_DONE,POLL_FROZEN,POLL_PREEMPT_MIDDLE,POLL_DEADLINE_RX,
      POLL_FIFO_TAIL,POLL_EMPTY_EOT,POLL_DEADLINE_ERROR,POLL_DEADLINE_EOT};
static unsigned poll_mode,poll_written,poll_preempted,poll_six_count;
static unsigned poll_rx_delay;
static unsigned poll_deadline_byte;
static unsigned poll_tick_calls;
static uint32_t poll_error_flag=SPI_SR_OVR;
static uint8_t wire_byte(uint8_t out);
static void complete_at_deadline(void)
{
    /* The low-priority task is preempted after testing RXP and before testing
     * the clock. The hardware finishes in the meantime: user's SR=0x301F. */
    test_tick_hook=NULL;fake_tick+=10;DWT->CYCCNT+=10U*480000U;
    SPI1->SR=poll_written==SPI1->CR2?0x301f:0x2007;poll_preempted=1;
    if(poll_mode==POLL_DEADLINE_ERROR)SPI1->SR|=SPI_SR_OVR;
}
static void complete_eot_at_deadline(void)
{
    /* TX progress reads once, RX progress twice. The fourth read is the
     * deadline decision after the earlier EOT test. */
    if(++poll_tick_calls<4)return;
    test_tick_hook=NULL;fake_tick+=10;DWT->CYCCNT+=10U*480000U;
    SPI1->SR|=SPI_SR_EOT;poll_preempted=1;
}
static void model_poll_step(void)
{
    if((poll_mode==POLL_DEADLINE_RX || poll_mode==POLL_DEADLINE_ERROR) &&
       poll_written==(poll_deadline_byte?poll_deadline_byte:SPI1->CR2) &&
       !(SPI1->SR&SPI_SR_RXP) && !poll_preempted)test_tick_hook=complete_at_deadline;
    if(poll_rx_delay && --poll_rx_delay==0){
        SPI1->SR|=SPI_SR_RXP;
        if(poll_written==SPI1->CR2)SPI1->SR|=SPI_SR_EOT;
    }
    if(poll_mode==POLL_PREEMPT_MIDDLE && poll_written==1 &&
       (SPI1->SR&SPI_SR_RXP) && !poll_preempted){
        fake_tick+=9;DWT->CYCCNT+=9U*480000U;poll_preempted=1;
    }
    if(poll_mode==POLL_PREEMPT_DONE && (SPI1->SR&SPI_SR_EOT) && !poll_preempted){
        fake_tick+=9;DWT->CYCCNT+=9U*480000U;poll_preempted=1;
    }
    if(poll_mode==POLL_ERROR)SPI1->SR|=poll_error_flag;
    if(poll_mode==POLL_NO_TX || poll_mode==POLL_FROZEN)SPI1->SR&=~SPI_SR_TXP;
    if(poll_mode==POLL_NO_EOT)SPI1->SR&=~SPI_SR_EOT;
    if(poll_mode!=POLL_NORMAL && poll_mode!=POLL_PREEMPT_DONE &&
       poll_mode!=POLL_FROZEN && poll_mode!=POLL_PREEMPT_MIDDLE &&
       poll_mode!=POLL_DEADLINE_RX && poll_mode!=POLL_FIFO_TAIL &&
       poll_mode!=POLL_DEADLINE_ERROR && poll_mode!=POLL_DEADLINE_EOT){
        fake_tick++;DWT->CYCCNT+=480000U;
    }
}
static void model_poll_write(uint8_t v)
{
    SPI1->TXDR=v;SPI1->RXDR=wire_byte(v);poll_written++;
    if((poll_mode==POLL_DEADLINE_RX || poll_mode==POLL_DEADLINE_ERROR) &&
       poll_written==(poll_deadline_byte?poll_deadline_byte:SPI1->CR2) && !poll_preempted)return;
    if(poll_mode==POLL_EMPTY_EOT){SPI1->SR|=SPI_SR_EOT;return;}
    if(poll_mode==POLL_PREEMPT_MIDDLE){poll_rx_delay=2;return;}
    if(poll_mode==POLL_FIFO_TAIL){
        SPI1->SR|=SPI_SR_RXPLVL_0;
        if(poll_written==SPI1->CR2)SPI1->SR|=SPI_SR_EOT;
        return;
    }
    if(poll_mode!=POLL_NO_RX)SPI1->SR|=SPI_SR_RXP;
    if(poll_mode==POLL_DEADLINE_EOT && poll_written==SPI1->CR2){
        poll_tick_calls=0;test_tick_hook=complete_eot_at_deadline;return;
    }
    if(poll_written==SPI1->CR2){
        SPI1->SR|=SPI_SR_EOT;
        if(poll_written==6)poll_six_count++;
    }
}
static uint8_t model_poll_read(void)
{SPI1->SR&=~(SPI_SR_RXP|SPI_SR_RXWNE|SPI_SR_RXPLVL);return (uint8_t)SPI1->RXDR;}
static void model_poll_clear(uint32_t flags)
{
    SPI1->IFCR=flags;SPI1->SR=SPI_SR_TXP;
    SPI1->CR1&=~SPI_CR1_CSTART;poll_written=poll_rx_delay=0;
}
#define SD_SPI_POLL_STEP model_poll_step
#define SD_SPI_WRITE8 model_poll_write
#define SD_SPI_READ8 model_poll_read
#define SD_SPI_CLEAR model_poll_clear
