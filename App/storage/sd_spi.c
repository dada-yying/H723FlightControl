/* SPI-mode SD driver. Sole caller is the independent storage task.
 * No changes to SPI123 PLL: GPIO mode-0 startup <=250 kHz, then SPI1 7.5 MHz.
 * Data uses isolated DMA bounce buffers; metadata may reside in DTCM. */
#include "sd_spi.h"
#include "blackbox_disk.h"
#include "diskio.h"
#include "blackbox_crc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "../../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_spi1_eot_guard.h"
#include <string.h>

SPI_HandleTypeDef hspi1_sd;
DMA_HandleTypeDef hdma_spi1_tx, hdma_spi1_rx;
volatile uint32_t g_bb_disk_error, g_bb_disk_max_ms, g_bb_disk_init_failed;
volatile uint32_t g_bb_disk_dma_max_ms, g_bb_disk_busy_max_ms, g_bb_disk_init_max_ms;
static DSTATUS disk_state=STA_NOINIT;
static uint8_t fast, high_capacity, io_failed;
static uint32_t sectors, card_cid[4];
static TaskHandle_t waiter;
static volatile uint8_t dma_done,dma_error;
enum {TRANSPORT_IDLE,TRANSPORT_POLL,TRANSPORT_DMA};
static volatile uint8_t transport;
static uint32_t dma_start_cycle;
static volatile uint32_t dma_irq_cycle;
volatile SdSpiDiag_t g_sd_spi_diag;
static uint8_t diag_active, diag_mosi_byte, diag_low_phase_byte, diag_have_bit;
static uint32_t diag_last_bit_cycle;
static SdSpiIoOperation_t io_operation;
static uint32_t io_sector,io_sector_count;
static int dma_transfer(unsigned n);
#define SD_SPI_SHORT_TIMEOUT_MS 5U
#define SD_SPI_CLEAR_FLAGS (SPI_IFCR_EOTC|SPI_IFCR_TXTFC|SPI_IFCR_OVRC|SPI_IFCR_UDRC|SPI_IFCR_MODFC|SPI_IFCR_TIFREC|SPI_IFCR_SUSPC)
#define SD_SPI_ERROR_FLAGS (SPI_SR_OVR|SPI_SR_UDR|SPI_SR_MODF|SPI_SR_TIFRE)
/* Register-level seams let host tests exercise the production polling loop. */
#ifndef SD_SPI_POLL_STEP
#define SD_SPI_POLL_STEP() ((void)0)
#define SD_SPI_WRITE8(v) (*(volatile uint8_t *)&SPI1->TXDR=(v))
#define SD_SPI_READ8() (*(volatile uint8_t *)&SPI1->RXDR)
#define SD_SPI_CLEAR(v) (SPI1->IFCR=(v))
#endif
#ifndef SD_SPI_STATUS
#define SD_SPI_STATUS() (SPI1->SR)
#endif
#ifndef SD_SPI_GPIO_WRITE
#define SD_SPI_GPIO_WRITE(p,v) ((p)->BSRR=(v))
#endif
#ifndef SD_SPI_GPIO_READ
#define SD_SPI_GPIO_READ(p) ((p)->IDR)
#endif
__attribute__((section(".sd_dma"), aligned(32))) static uint8_t tx_dma[544];
__attribute__((section(".sd_dma"), aligned(32))) static uint8_t rx_dma[544];

static void maximum(volatile uint32_t *p,uint32_t n){if(n>*p)*p=n;}
static void io_fault(SdSpiIoFaultKind_t kind,uint32_t error,HAL_StatusTypeDef status,
                     uint32_t elapsed,uint8_t tx,uint8_t rx)
{
    volatile SdSpiIoFault_t *f=&g_sd_spi_diag.io_fault;
    if(io_operation==SD_IO_NONE || f->complete)return;
    f->init_attempt=g_sd_spi_diag.init_attempts;f->tick_ms=HAL_GetTick();
    f->operation=io_operation;f->kind=kind;f->elapsed_ms=elapsed;
    f->sector=io_sector;f->sector_count=io_sector_count;f->error_code=error;
    f->hal_status=status;f->hal_error=hspi1_sd.ErrorCode;f->hal_state=hspi1_sd.State;
    if(hspi1_sd.Instance){
        f->spi_sr=hspi1_sd.Instance->SR;f->spi_cr1=hspi1_sd.Instance->CR1;
        f->spi_cr2=hspi1_sd.Instance->CR2;f->spi_cfg1=hspi1_sd.Instance->CFG1;
        f->spi_ier=hspi1_sd.Instance->IER;
    }
    f->tx_dma_remaining=hdma_spi1_tx.Instance?__HAL_DMA_GET_COUNTER(&hdma_spi1_tx):0;
    f->rx_dma_remaining=hdma_spi1_rx.Instance?__HAL_DMA_GET_COUNTER(&hdma_spi1_rx):0;
    f->transfer_size=g_sd_spi_diag.last_transfer_size;
    f->command=g_sd_spi_diag.last_command;f->r1=g_sd_spi_diag.last_r1;
    f->byte_tx=tx;f->byte_rx=rx;f->fast_mode=fast;
    f->dma_complete=dma_done;f->dma_failed=dma_error;
    __DSB();f->complete=1;
}
static void abort_transport(void)
{
    /* Disarm callbacks BEFORE abort. HAL abort stops both DMA streams and SPI;
     * discard pending IRQs before another transaction may reuse the buffers. */
    transport=TRANSPORT_IDLE;waiter=NULL;__DSB();
    (void)HAL_SPI_Abort(&hspi1_sd);
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream3_IRQn);
    HAL_NVIC_ClearPendingIRQ(SPI1_IRQn);
    io_failed=1;
}
static int transport_idle(void)
{
    return transport==TRANSPORT_IDLE && hspi1_sd.State==HAL_SPI_STATE_READY &&
           !(SPI1->CR1&SPI_CR1_SPE) && !SPI1->IER &&
           !(SPI1->CFG1&(SPI_CFG1_RXDMAEN|SPI_CFG1_TXDMAEN));
}
/* DSIZE=8: FIFO occupancy can expose the final byte even if packet-level RXP
 * is not asserted. EOT on its own is NOT evidence of a received byte. */
static int poll_rx_available(uint32_t sr)
{
    return (sr&(SPI_SR_RXP|SPI_SR_RXWNE|SPI_SR_RXPLVL))!=0;
}
static int poll_can_progress(uint32_t sr,unsigned sent,unsigned received,unsigned n)
{
    return (received<sent && poll_rx_available(sr)) ||
           (sent<n && sent==received && (sr&SPI_SR_TXP)) ||
           (received==n && (sr&SPI_SR_EOT));
}
/* Independently implemented short/bulk split, informed by Betaflight's
 * bus_spi_ll.c / sdcard_spi.c. Single owner, IRQs remain enabled. */
static int poll_transfer(const uint8_t *tx,uint8_t *rx,unsigned n)
{
    uint32_t start=HAL_GetTick(),progress_tick=start,cycle=DWT->CYCCNT,loops=0;
    uint32_t limit=SystemCoreClock/200U+1U,sr=0;
    unsigned sent=0,received=0;uint8_t value=0;
    SdSpiIoFaultKind_t fault=SD_IO_FAULT_SHORT_TIMEOUT;
    HAL_StatusTypeDef status=HAL_TIMEOUT;
    g_sd_spi_diag.last_transfer_size=n;
    if(io_failed)return 0;
    if(!n || n>16 || !transport_idle()){
        fault=SD_IO_FAULT_TRANSPORT_BUSY;status=HAL_BUSY;goto fail;
    }
    transport=TRANSPORT_POLL;dma_done=dma_error=0;waiter=NULL;
    hspi1_sd.State=HAL_SPI_STATE_BUSY_TX_RX;
    hspi1_sd.ErrorCode=HAL_SPI_ERROR_NONE;
    SD_SPI_CLEAR(SD_SPI_CLEAR_FLAGS);
    SPI1->CR2=(SPI1->CR2&~SPI_CR2_TSIZE)|n;
    SPI1->CR1|=SPI_CR1_SPE;
    SPI1->CR1|=SPI_CR1_CSTART;
    for(;;){
        SD_SPI_POLL_STEP();
        sr=SD_SPI_STATUS();
        if(sr&SD_SPI_ERROR_FLAGS){fault=SD_IO_FAULT_SHORT_HW;status=HAL_ERROR;break;}
        /* At most one byte outstanding: no FIFO overrun if preempted. */
        if(received<sent && poll_rx_available(sr)){
            if(!(sr&SPI_SR_RXP))g_sd_spi_diag.short_fifo_tail_reads++;
            value=SD_SPI_READ8();if(rx)rx[received]=value;received++;
            if((uint32_t)(HAL_GetTick()-progress_tick)>=SD_SPI_SHORT_TIMEOUT_MS)
                g_sd_spi_diag.short_progress_recoveries++;
            /* Completed hardware progress may have waited for a preempted
             * task. Give the NEXT byte/EOT its own bounded stall budget. */
            progress_tick=HAL_GetTick();cycle=DWT->CYCCNT;loops=0;
        }
        if(sent<n && sent==received && (sr&SPI_SR_TXP)){
            SD_SPI_WRITE8(tx?tx[sent]:0xff);sent++;
            progress_tick=HAL_GetTick();cycle=DWT->CYCCNT;loops=0;
            /* Consume an immediately available final RX before deadline test. */
            continue;
        }
        if(received==n && (sr&SPI_SR_EOT)){
            SPI1_SD_EotGuard(&hspi1_sd);
            SPI1->CR1&=~SPI_CR1_SPE;
            SD_SPI_CLEAR(SD_SPI_CLEAR_FLAGS);
            hspi1_sd.State=HAL_SPI_STATE_READY;transport=TRANSPORT_IDLE;
            g_sd_spi_diag.short_transfer_count++;g_sd_spi_diag.short_bytes+=n;
            return 1;
        }
        if((uint32_t)(HAL_GetTick()-progress_tick)>=SD_SPI_SHORT_TIMEOUT_MS ||
           ((DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk) && (uint32_t)(DWT->CYCCNT-cycle)>=limit) ||
           ++loops>=limit){
            /* Reading the clock can resume AFTER a long preemption that
             * started after the earlier SR read. The SD byte/EOT may already
             * be complete now (observed failure SR=0x301F). Revalidate the
             * hardware AFTER detecting expiry; never abort from stale SR. */
            sr=SD_SPI_STATUS();
            if(sr&SD_SPI_ERROR_FLAGS){fault=SD_IO_FAULT_SHORT_HW;status=HAL_ERROR;break;}
            if(poll_can_progress(sr,sent,received,n)){
                g_sd_spi_diag.short_deadline_rechecks++;
                continue;
            }
            break;
        }
    }
fail:
    if(fault==SD_IO_FAULT_SHORT_TIMEOUT){g_sd_spi_diag.short_timeout_count++;hspi1_sd.ErrorCode|=HAL_SPI_ERROR_TIMEOUT;}
    else g_sd_spi_diag.short_error_count++;
    g_bb_disk_error=BB_DISK_TIMEOUT;
    if(!g_sd_spi_diag.io_fault.complete && io_operation!=SD_IO_NONE){
        g_sd_spi_diag.short_fault_sent=sent;
        g_sd_spi_diag.short_fault_received=received;
        g_sd_spi_diag.short_fault_wait_ms=(uint32_t)(HAL_GetTick()-progress_tick);
        g_sd_spi_diag.short_fault_status=sr;
    }
    io_fault(fault,g_bb_disk_error,status,HAL_GetTick()-start,tx?tx[0]:0xff,value);
    abort_transport();return 0;
}
static void delay_edge(void)
{
#ifdef SD_SPI_TEST_DELAY
    SD_SPI_TEST_DELAY();
#else
    uint32_t t=DWT->CYCCNT, ticks=SystemCoreClock/500000U;
    while((uint32_t)(DWT->CYCCNT-t)<ticks){}
#endif
}
static void cs(int high){SD_SPI_GPIO_WRITE(GPIOE,high?GPIO_PIN_15:((uint32_t)GPIO_PIN_15<<16));}
static uint8_t spi_byte(uint8_t out)
{
#ifdef SD_SPI_TEST_BYTE
    if(!fast)return SD_SPI_TEST_BYTE(out);
#endif
    uint8_t in=0;unsigned i;
    if(io_failed)return 0;
    if(fast){
        if(!poll_transfer(&out,&in,1))return 0;
        return in;
    }
    diag_mosi_byte=diag_low_phase_byte=0;
    for(i=0;i<8;i++){
        SD_SPI_GPIO_WRITE(GPIOD,(out&0x80)?GPIO_PIN_7:((uint32_t)GPIO_PIN_7<<16));
        delay_edge();
        if(diag_active){
            uint32_t b=SD_SPI_GPIO_READ(GPIOB),d=SD_SPI_GPIO_READ(GPIOD);
            uint32_t now=DWT->CYCCNT,elapsed=(uint32_t)(now-diag_last_bit_cycle);
            if(diag_have_bit){
                if(elapsed<g_sd_spi_diag.cmd0.bit_period_min_cycles)g_sd_spi_diag.cmd0.bit_period_min_cycles=elapsed;
                if(elapsed>g_sd_spi_diag.cmd0.bit_period_max_cycles)g_sd_spi_diag.cmd0.bit_period_max_cycles=elapsed;
            }
            diag_last_bit_cycle=now;diag_have_bit=1;g_sd_spi_diag.cmd0.gpio_checked=1;
            if(SD_SPI_GPIO_READ(GPIOE)&GPIO_PIN_15)g_sd_spi_diag.cmd0.cs_not_low_count++;
            if(b&GPIO_PIN_3)g_sd_spi_diag.cmd0.sck_not_low_count++;
            if(((d&GPIO_PIN_7)!=0)!=((out&0x80)!=0))g_sd_spi_diag.cmd0.mosi_mismatch_count++;
            diag_mosi_byte=(uint8_t)((diag_mosi_byte<<1)|((d&GPIO_PIN_7)?1:0));
            diag_low_phase_byte=(uint8_t)((diag_low_phase_byte<<1)|((b&GPIO_PIN_4)?1:0));
        }
        SD_SPI_GPIO_WRITE(GPIOB,GPIO_PIN_3);delay_edge();
        {uint32_t b=SD_SPI_GPIO_READ(GPIOB);
         in=(uint8_t)((in<<1)|((b&GPIO_PIN_4)?1:0));
         if(diag_active && !(b&GPIO_PIN_3))g_sd_spi_diag.cmd0.sck_not_high_count++;}
        SD_SPI_GPIO_WRITE(GPIOB,(uint32_t)GPIO_PIN_3<<16);out<<=1;
    }
    return in;
}
static int ready(uint32_t timeout)
{
    uint32_t start=HAL_GetTick(); unsigned burst=0;uint8_t value=0xff;
    do {
        value=spi_byte(0xff);
        if(value==0xff){maximum(&g_bb_disk_busy_max_ms,HAL_GetTick()-start);return !io_failed;}
        if(io_failed)break;
        if(++burst==32){burst=0;vTaskDelay(1);}
    }while((uint32_t)(HAL_GetTick()-start)<timeout);
    maximum(&g_bb_disk_busy_max_ms,HAL_GetTick()-start);
    g_sd_spi_diag.ready_timeouts++;
    io_fault(SD_IO_FAULT_CARD_BUSY,BB_DISK_TIMEOUT,HAL_OK,HAL_GetTick()-start,0xff,value);
    io_failed=1;g_bb_disk_error=BB_DISK_TIMEOUT;return 0;
}
static void deselect(void){cs(1);(void)spi_byte(0xff);}
static uint8_t crc7(const uint8_t *p,unsigned n)
{
    unsigned i;uint8_t c=0;
    while(n--){uint8_t d=*p++;for(i=0;i<8;i++){c<<=1;if((d^c)&0x80)c^=9;d<<=1;}}
    return (uint8_t)((c<<1)|1);
}
static uint16_t crc16(const uint8_t *p,unsigned n)
{
    return BlackboxCrc16(p,n);
}
static uint8_t command(uint8_t cmd,uint32_t arg)
{
    uint8_t p[6],r=0xff;unsigned i;
    uint32_t start_cycle=0;
    g_sd_spi_diag.last_command=cmd;g_sd_spi_diag.last_argument=arg;
    g_sd_spi_diag.last_r1=0xff;g_sd_spi_diag.command_sent=0;
    g_sd_spi_diag.last_response_count=0;g_sd_spi_diag.command_count++;
    if(cmd!=12){
        deselect();cs(0);
        /* Preserve the user's CMD0 reset-path correction. */
        if(cmd!=0 && !ready(500))return 0xff;
    }
    p[0]=0x40|cmd;p[1]=(uint8_t)(arg>>24);p[2]=(uint8_t)(arg>>16);p[3]=(uint8_t)(arg>>8);p[4]=(uint8_t)arg;p[5]=crc7(p,5);
    if(cmd==0 && !g_sd_spi_diag.cmd0.complete && !fast){
        memset((void *)&g_sd_spi_diag.cmd0,0,sizeof(g_sd_spi_diag.cmd0));
        g_sd_spi_diag.cmd0.attempt=g_sd_spi_diag.init_attempts;
        g_sd_spi_diag.cmd0.tick_ms=HAL_GetTick();
        g_sd_spi_diag.cmd0.bit_period_min_cycles=UINT32_MAX;
        g_sd_spi_diag.cmd0.cs_mode=(uint8_t)((GPIOE->MODER>>30)&3);
        g_sd_spi_diag.cmd0.sck_mode=(uint8_t)((GPIOB->MODER>>6)&3);
        g_sd_spi_diag.cmd0.mosi_mode=(uint8_t)((GPIOD->MODER>>14)&3);
        g_sd_spi_diag.cmd0.miso_mode=(uint8_t)((GPIOB->MODER>>8)&3);
        g_sd_spi_diag.cmd0.miso_pull=(uint8_t)((GPIOB->PUPDR>>8)&3);
        for(i=0;i<6;i++)g_sd_spi_diag.cmd0.tx[i]=p[i];
        start_cycle=DWT->CYCCNT;diag_have_bit=0;diag_active=1;
    }
    if(fast){if(!poll_transfer(p,NULL,6))return 0xff;}
    else for(i=0;i<6;i++){
        spi_byte(p[i]);
        if(diag_active)g_sd_spi_diag.cmd0.mosi_readback[i]=diag_mosi_byte;
    }
    g_sd_spi_diag.command_sent=!io_failed;
    if(cmd==12)spi_byte(0xff);
    for(i=0;i<16;i++){
        r=spi_byte(0xff);g_sd_spi_diag.last_response_count=(uint8_t)(i+1);
        if(diag_active){
            g_sd_spi_diag.cmd0.rx[i]=r;
            g_sd_spi_diag.cmd0.rx_low_phase[i]=diag_low_phase_byte;
            g_sd_spi_diag.cmd0.rx_count=(uint8_t)(i+1);
        }
        if(!(r&0x80))break;
    }
    g_sd_spi_diag.last_r1=r;
    if(r&0x80)g_sd_spi_diag.response_timeouts++;
    if(diag_active){
        uint32_t cycles=(uint32_t)(DWT->CYCCNT-start_cycle);
        g_sd_spi_diag.cmd0.duration_us=SystemCoreClock>=1000000U?cycles/(SystemCoreClock/1000000U):0;
        g_sd_spi_diag.cmd0.r1=r;diag_active=0;
        __DSB();g_sd_spi_diag.cmd0.complete=1;
    }
    return r;
}
static int dma_transfer(unsigned n)
{
    uint32_t start=HAL_GetTick();HAL_StatusTypeDef s;
    uint32_t cache_bytes=(n+31U)&~31U;
    g_sd_spi_diag.last_transfer_size=n;
    if(io_failed)return 0;
    if((n!=514 && n!=516) || !transport_idle()){
        io_fault(SD_IO_FAULT_TRANSPORT_BUSY,BB_DISK_TIMEOUT,HAL_BUSY,0,tx_dma[0],0);
        g_bb_disk_error=BB_DISK_TIMEOUT;abort_transport();return 0;
    }
    waiter=xTaskGetCurrentTaskHandle();(void)ulTaskNotifyTake(pdTRUE,0);
    dma_done=dma_error=0;
    if(SCB->CCR&SCB_CCR_DC_Msk){
        SCB_CleanDCache_by_Addr((uint32_t *)tx_dma,cache_bytes);
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)rx_dma,cache_bytes);
    }
    __DSB();
    dma_start_cycle=DWT->CYCCNT;transport=TRANSPORT_DMA;
    s=HAL_SPI_TransmitReceive_DMA(&hspi1_sd,tx_dma,rx_dma,(uint16_t)n);
    while(s==HAL_OK && !dma_done && !dma_error && (uint32_t)(HAL_GetTick()-start)<100)
        (void)ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(10));
    maximum(&g_bb_disk_dma_max_ms,HAL_GetTick()-start);
    if(s!=HAL_OK || dma_error || !dma_done){
        io_fault(s!=HAL_OK?SD_IO_FAULT_DMA_START:(dma_error?SD_IO_FAULT_DMA_IRQ:SD_IO_FAULT_DMA_WAIT),
                 BB_DISK_TIMEOUT,s,HAL_GetTick()-start,tx_dma[0],rx_dma[0]);
        /* Bounded HAL abort before either bounce buffer can be reused. */
        g_bb_disk_error=BB_DISK_TIMEOUT;abort_transport();return 0;
    }
    {
        uint32_t resume=(uint32_t)(DWT->CYCCNT-dma_irq_cycle);
        uint32_t cycles_per_us=SystemCoreClock/1000000U;
        maximum(&g_sd_spi_diag.dma_resume_max_cycles,resume);
        if(cycles_per_us){
            g_sd_spi_diag.dma_irq_total_us+=(uint32_t)(dma_irq_cycle-dma_start_cycle)/cycles_per_us;
            g_sd_spi_diag.dma_resume_total_us+=resume/cycles_per_us;
            g_sd_spi_diag.dma_latency_samples++;
        }
    }
    transport=TRANSPORT_IDLE;waiter=NULL;
    __DSB();if(SCB->CCR&SCB_CCR_DC_Msk)SCB_InvalidateDCache_by_Addr((uint32_t *)rx_dma,cache_bytes);
    __DSB();
    g_sd_spi_diag.sector_dma_count++;g_sd_spi_diag.block_dma_count++;
    g_sd_spi_diag.dma_bytes+=n;
    return 1;
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *s)
{
    BaseType_t wake=pdFALSE;if(s!=&hspi1_sd)return;
    if(transport!=TRANSPORT_DMA || dma_done || dma_error){g_sd_spi_diag.ignored_callback_count++;return;}
    dma_irq_cycle=DWT->CYCCNT;
    maximum(&g_sd_spi_diag.dma_to_irq_max_cycles,(uint32_t)(dma_irq_cycle-dma_start_cycle));
    __DSB();dma_done=1;
    if(waiter)vTaskNotifyGiveFromISR(waiter,&wake);portYIELD_FROM_ISR(wake);
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *s)
{
    BaseType_t wake=pdFALSE;if(s!=&hspi1_sd)return;
    if(transport!=TRANSPORT_DMA || dma_done || dma_error){g_sd_spi_diag.ignored_callback_count++;return;}
    dma_error=1;
    if(waiter)vTaskNotifyGiveFromISR(waiter,&wake);portYIELD_FROM_ISR(wake);
}
static int receive(uint8_t *p,unsigned n)
{
    uint32_t start=HAL_GetTick();uint8_t token;uint16_t crc;unsigned burst=0,i;
    do{token=spi_byte(0xff);if(token!=0xff)break;if(++burst==32){burst=0;vTaskDelay(1);}}
    while(!io_failed && (uint32_t)(HAL_GetTick()-start)<200);
    if(token!=0xfe){io_fault(SD_IO_FAULT_READ_TOKEN,BB_DISK_TIMEOUT,HAL_OK,HAL_GetTick()-start,0xff,token);g_bb_disk_error=BB_DISK_TIMEOUT;return 0;}
    if(fast && n==512){
        memset(tx_dma,0xff,514);if(!dma_transfer(514))return 0;
        crc=(uint16_t)((uint16_t)rx_dma[512]<<8)|rx_dma[513];
        if(crc!=crc16(rx_dma,512)){
            io_fault(SD_IO_FAULT_DATA_CRC,0x80000003U,HAL_OK,HAL_GetTick()-start,0xff,0);
            g_bb_disk_error=0x80000003U;return 0;
        }
        memcpy(p,rx_dma,512);return 1;
    }
    for(i=0;i<n;i++)p[i]=spi_byte(0xff);
    crc=(uint16_t)spi_byte(0xff)<<8;crc|=spi_byte(0xff);
    if(io_failed || crc!=crc16(p,n)){io_fault(SD_IO_FAULT_DATA_CRC,0x80000003U,HAL_OK,HAL_GetTick()-start,0xff,0);g_bb_disk_error=0x80000003U;return 0;}return 1;
}
static int transmit(const uint8_t *p,uint8_t token)
{
    uint16_t crc;uint8_t response=0xff;unsigned i;
    if(!ready(500))return 0;
    /* N_WR before a start/stop token, with CS continuously asserted. */
    if(token==0xfd){
        spi_byte(0xff);spi_byte(token);
        /* The card may delay assertion of busy by one byte (N_BR). */
        spi_byte(0xff);return ready(500);
    }
    tx_dma[0]=0xff;tx_dma[1]=token;memcpy(tx_dma+2,p,512);crc=crc16(p,512);
    tx_dma[514]=(uint8_t)(crc>>8);tx_dma[515]=(uint8_t)crc;
    if(!dma_transfer(516))return 0;
    for(i=0;i<16 && !io_failed;i++){response=spi_byte(0xff);if(response!=0xff)break;}
    if(io_failed)return 0;
    if((response&31)!=5){io_fault(SD_IO_FAULT_DATA_RESPONSE,0x80000100U|response,HAL_OK,0,0xff,response);g_bb_disk_error=0x80000100U|response;return 0;}
    return ready(500);
}
static void slow_gpio(void)
{
    GPIO_InitTypeDef g={0};
    __HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_GPIOD_CLK_ENABLE();__HAL_RCC_GPIOE_CLK_ENABLE();
    cs(1);GPIOB->BSRR=(uint32_t)GPIO_PIN_3<<16;GPIOD->BSRR=GPIO_PIN_7;
    g.Mode=GPIO_MODE_OUTPUT_PP;g.Speed=GPIO_SPEED_FREQ_HIGH;g.Pin=GPIO_PIN_15;HAL_GPIO_Init(GPIOE,&g);
    g.Pin=GPIO_PIN_3;HAL_GPIO_Init(GPIOB,&g);g.Pin=GPIO_PIN_7;HAL_GPIO_Init(GPIOD,&g);
    g.Mode=GPIO_MODE_INPUT;g.Pull=GPIO_PULLUP;g.Pin=GPIO_PIN_4;HAL_GPIO_Init(GPIOB,&g);
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
}
static int fast_spi(void)
{
    GPIO_InitTypeDef g={0};DMA_HandleTypeDef *d;unsigned i;
    __HAL_RCC_SPI1_CLK_ENABLE();__HAL_RCC_DMA1_CLK_ENABLE();
    /* Existing PLL1Q is 240 MHz. Do not reconfigure shared SPI123 source. */
    hspi1_sd.Instance=SPI1;
    hspi1_sd.Init.Mode=SPI_MODE_MASTER;hspi1_sd.Init.Direction=SPI_DIRECTION_2LINES;
    hspi1_sd.Init.DataSize=SPI_DATASIZE_8BIT;hspi1_sd.Init.CLKPolarity=SPI_POLARITY_LOW;
    hspi1_sd.Init.CLKPhase=SPI_PHASE_1EDGE;hspi1_sd.Init.NSS=SPI_NSS_SOFT;
    hspi1_sd.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_32;hspi1_sd.Init.FirstBit=SPI_FIRSTBIT_MSB;
    hspi1_sd.Init.TIMode=SPI_TIMODE_DISABLE;hspi1_sd.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;
    hspi1_sd.Init.NSSPMode=SPI_NSS_PULSE_DISABLE;hspi1_sd.Init.NSSPolarity=SPI_NSS_POLARITY_LOW;
    hspi1_sd.Init.FifoThreshold=SPI_FIFO_THRESHOLD_01DATA;hspi1_sd.Init.MasterKeepIOState=SPI_MASTER_KEEP_IO_STATE_ENABLE;
    if(HAL_SPI_Init(&hspi1_sd)!=HAL_OK)return 0;
    for(i=0;i<2;i++){
        d=i?&hdma_spi1_rx:&hdma_spi1_tx;
        d->Instance=i?DMA1_Stream3:DMA1_Stream1;
        d->Init.Request=i?DMA_REQUEST_SPI1_RX:DMA_REQUEST_SPI1_TX;
        d->Init.Direction=i?DMA_PERIPH_TO_MEMORY:DMA_MEMORY_TO_PERIPH;
        d->Init.PeriphInc=DMA_PINC_DISABLE;d->Init.MemInc=DMA_MINC_ENABLE;
        d->Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;d->Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;
        d->Init.Mode=DMA_NORMAL;d->Init.Priority=DMA_PRIORITY_LOW;d->Init.FIFOMode=DMA_FIFOMODE_DISABLE;
        if(HAL_DMA_Init(d)!=HAL_OK)return 0;
    }
    __HAL_LINKDMA(&hspi1_sd,hdmatx,hdma_spi1_tx);__HAL_LINKDMA(&hspi1_sd,hdmarx,hdma_spi1_rx);
    g.Mode=GPIO_MODE_AF_PP;g.Speed=GPIO_SPEED_FREQ_VERY_HIGH;g.Alternate=GPIO_AF5_SPI1;
    g.Pin=GPIO_PIN_3|GPIO_PIN_4;HAL_GPIO_Init(GPIOB,&g);g.Pin=GPIO_PIN_7;HAL_GPIO_Init(GPIOD,&g);
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn,6,0);HAL_NVIC_SetPriority(DMA1_Stream3_IRQn,6,0);HAL_NVIC_SetPriority(SPI1_IRQn,6,0);
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream1_IRQn);HAL_NVIC_ClearPendingIRQ(DMA1_Stream3_IRQn);HAL_NVIC_ClearPendingIRQ(SPI1_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);HAL_NVIC_EnableIRQ(SPI1_IRQn);
    fast=1;return 1;
}
void BB_DiskReset(void)
{
    transport=TRANSPORT_IDLE;waiter=NULL;dma_done=dma_error=0;
    HAL_NVIC_DisableIRQ(SPI1_IRQn);HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);HAL_NVIC_DisableIRQ(DMA1_Stream3_IRQn);
    if(hspi1_sd.Instance){(void)HAL_SPI_Abort(&hspi1_sd);(void)HAL_SPI_DeInit(&hspi1_sd);}
    if(hdma_spi1_tx.Instance)(void)HAL_DMA_DeInit(&hdma_spi1_tx);
    if(hdma_spi1_rx.Instance)(void)HAL_DMA_DeInit(&hdma_spi1_rx);
    __HAL_RCC_SPI1_FORCE_RESET();__HAL_RCC_SPI1_RELEASE_RESET();
    fast=io_failed=0;disk_state=STA_NOINIT;waiter=NULL;
    io_operation=SD_IO_NONE;
    g_bb_disk_error=g_bb_disk_init_failed=0;slow_gpio();
}
int BB_CardPresent(void){return 1;}
void BB_CardIdentity(uint32_t cid[4]){memcpy(cid,card_cid,16);}
DSTATUS disk_initialize(BYTE drive)
{
    uint32_t start=HAL_GetTick(),arg;uint8_t r,ocr[4],csd[16];unsigned i;
    if(drive)return STA_NOINIT;
    g_sd_spi_diag.version=0x20261020U;g_sd_spi_diag.init_attempts++;
    g_sd_spi_diag.revision=9;
    g_sd_spi_diag.stage=SD_INIT_CLOCKS;
    BB_DiskReset();vTaskDelay(pdMS_TO_TICKS(10));
    for(i=0;i<10;i++)spi_byte(0xff);
    g_sd_spi_diag.stage=SD_INIT_CMD0;
    if(command(0,0)!=1)goto fail;
    g_sd_spi_diag.stage=SD_INIT_CMD8;
    r=command(8,0x1aa);arg=0;
    if(r==1){for(i=0;i<4;i++)ocr[i]=spi_byte(0xff);if(ocr[2]!=1 || ocr[3]!=0xaa)goto fail;arg=0x40000000;}
    else if(r!=5)goto fail;
    g_sd_spi_diag.stage=SD_INIT_ACMD41;
    do {
        if(command(55,0)>1)goto fail;
        r=command(41,arg);if(r==0)break;if(r!=1)goto fail;
        deselect();vTaskDelay(pdMS_TO_TICKS(10));
    }while((uint32_t)(HAL_GetTick()-start)<2000);
    g_sd_spi_diag.stage=SD_INIT_OCR;
    if(r || command(58,0)!=0)goto fail;
    for(i=0;i<4;i++)ocr[i]=spi_byte(0xff);
    high_capacity=(ocr[0]&0x40)!=0;
    g_sd_spi_diag.stage=SD_INIT_BLOCK_LENGTH;
    if(!high_capacity && command(16,512)!=0)goto fail;
    g_sd_spi_diag.stage=SD_INIT_CRC;
    if(command(59,1)!=0)goto fail; /* Require transport CRC checking. */
    g_sd_spi_diag.stage=SD_INIT_CSD;
    if(command(9,0)!=0 || !receive(csd,16))goto fail;
    if((csd[0]>>6)==1){uint32_t size=((uint32_t)(csd[7]&63)<<16)|((uint32_t)csd[8]<<8)|csd[9];sectors=(size+1)*1024U;}
    else if((csd[0]>>6)==0){
        uint32_t n=(csd[5]&15)+((csd[10]&128)>>7)+((csd[9]&3)<<1)+2;
        uint32_t size=(csd[8]>>6)|((uint32_t)csd[7]<<2)|((uint32_t)(csd[6]&3)<<10);
        if(n<9 || n>31)goto fail;sectors=(size+1)<<(n-9);
    }else goto fail;
    g_sd_spi_diag.stage=SD_INIT_CID;
    if(!sectors || command(10,0)!=0 || !receive((uint8_t *)card_cid,16))goto fail;
    g_sd_spi_diag.stage=SD_INIT_FAST_SPI;
    deselect();if(!fast_spi())goto fail;
    g_sd_spi_diag.stage=SD_INIT_READY;g_sd_spi_diag.init_successes++;
    maximum(&g_bb_disk_init_max_ms,HAL_GetTick()-start);g_bb_disk_init_failed=0;return disk_state=0;
fail:
    g_sd_spi_diag.init_failures++;g_sd_spi_diag.failed_stage=g_sd_spi_diag.stage;
    g_sd_spi_diag.failure_command=g_sd_spi_diag.last_command;
    g_sd_spi_diag.failure_argument=g_sd_spi_diag.last_argument;
    g_sd_spi_diag.failure_r1=g_sd_spi_diag.last_r1;
    g_sd_spi_diag.failure_command_sent=g_sd_spi_diag.command_sent;
    deselect();g_bb_disk_init_failed=1;if(!g_bb_disk_error)g_bb_disk_error=0x80000004U;
    maximum(&g_bb_disk_init_max_ms,HAL_GetTick()-start);return disk_state=STA_NOINIT;
}
DSTATUS disk_status(BYTE drive){return drive?STA_NOINIT:disk_state;}
static int preerase(UINT n)
{
    uint8_t r;
    g_sd_spi_diag.preerase_count++;
    r=command(55,0);
    if(r==0 && !io_failed)r=command(23,n);
    g_sd_spi_diag.preerase_last_r1=r;
    if(!io_failed && r==0){g_sd_spi_diag.preerase_accepted++;return 1;}
    /* Only the illegal-command bit alone means unsupported, not a timeout
     * or an idle/CRC/address error combined with that bit. */
    if(!io_failed && r==4){g_sd_spi_diag.preerase_unsupported++;return 1;}
    g_sd_spi_diag.preerase_errors++;
    if(!g_bb_disk_error)g_bb_disk_error=0x80000005U;
    io_fault(SD_IO_FAULT_COMMAND,g_bb_disk_error,HAL_OK,0,0xff,r);return 0;
}
static DRESULT transfer(BYTE drive,BYTE *data,DWORD sector,UINT n,int write)
{
    uint32_t start=HAL_GetTick(),arg;UINT i;int ok=1;
    if(drive || !n || !data || sector>=sectors || n>sectors-sector)return RES_PARERR;
    if(disk_state)return RES_NOTRDY;
    io_operation=write?SD_IO_WRITE:SD_IO_READ;io_sector=sector;io_sector_count=n;
    arg=high_capacity?sector:sector*512U;
    if(write && n>1)ok=preerase(n);
    if(ok && command(write?(n>1?25:24):(n>1?18:17),arg)!=0){
        io_fault(SD_IO_FAULT_COMMAND,0x80000005U,HAL_OK,HAL_GetTick()-start,0xff,g_sd_spi_diag.last_r1);ok=0;
    }
    for(i=0;ok && i<n;i++)ok=write?transmit(data+i*512,n>1?0xfc:0xfe):receive(data+i*512,512);
    /* Do not emit a stop for a stream whose command was never accepted. */
    if(n>1 && !io_failed && g_sd_spi_diag.last_r1==0 &&
       g_sd_spi_diag.last_command==(write?25:18)){
        if(write){if(!transmit(NULL,0xfd))ok=0;}
        else {if(command(12,0)!=0 || !ready(500))ok=0;}
    }
    deselect();maximum(&g_bb_disk_max_ms,HAL_GetTick()-start);
    if(!ok || io_failed){io_operation=SD_IO_NONE;disk_state=STA_NOINIT;if(!g_bb_disk_error)g_bb_disk_error=0x80000005U;return RES_ERROR;}
    if(write)g_sd_spi_diag.written_sectors+=n;else g_sd_spi_diag.read_sectors+=n;
    io_operation=SD_IO_NONE;
    return RES_OK;
}
DRESULT disk_read(BYTE d,BYTE *p,DWORD s,UINT n){return transfer(d,p,s,n,0);}
DRESULT disk_write(BYTE d,const BYTE *p,DWORD s,UINT n){return transfer(d,(BYTE *)p,s,n,1);}
DRESULT disk_ioctl(BYTE d,BYTE cmd,void *p)
{
    int ok;if(d || disk_state)return RES_NOTRDY;
    switch(cmd){
    case CTRL_SYNC:
        io_operation=SD_IO_SYNC;io_sector=io_sector_count=0;g_sd_spi_diag.sync_calls++;
        cs(0);ok=ready(500);deselect();io_operation=SD_IO_NONE;
        if(!ok){disk_state=STA_NOINIT;return RES_ERROR;}
        g_sd_spi_diag.sync_successes++;return RES_OK;
    case GET_SECTOR_COUNT:if(!p)return RES_PARERR;*(DWORD *)p=sectors;return RES_OK;
    case GET_SECTOR_SIZE:if(!p)return RES_PARERR;*(WORD *)p=512;return RES_OK;
    case GET_BLOCK_SIZE:if(!p)return RES_PARERR;*(DWORD *)p=1;return RES_OK;
    default:return RES_PARERR;}
}
HAL_StatusTypeDef BB_RandomSession(uint8_t session[16])
{
    RCC_PeriphCLKInitTypeDef clock={0};uint32_t start=HAL_GetTick(),v;unsigned i;
    __HAL_RCC_HSI48_ENABLE();
    while(!__HAL_RCC_GET_FLAG(RCC_FLAG_HSI48RDY)){if((uint32_t)(HAL_GetTick()-start)>=100)return HAL_TIMEOUT;vTaskDelay(1);}
    clock.PeriphClockSelection=RCC_PERIPHCLK_RNG;clock.RngClockSelection=RCC_RNGCLKSOURCE_HSI48;
    if(HAL_RCCEx_PeriphCLKConfig(&clock)!=HAL_OK)return HAL_ERROR;
    __HAL_RCC_RNG_CLK_ENABLE();__HAL_RCC_RNG_FORCE_RESET();__HAL_RCC_RNG_RELEASE_RESET();
    RNG->CR|=RNG_CR_RNGEN;
    for(i=0;i<4;i++){
        start=HAL_GetTick();
        while(!(RNG->SR&RNG_SR_DRDY)){
            if((RNG->SR&(RNG_SR_CECS|RNG_SR_SECS)) || (uint32_t)(HAL_GetTick()-start)>=100){RNG->CR&=~RNG_CR_RNGEN;return HAL_ERROR;}
            vTaskDelay(1);
        }
        if(RNG->SR&(RNG_SR_CECS|RNG_SR_SECS)){RNG->CR&=~RNG_CR_RNGEN;return HAL_ERROR;}
        v=RNG->DR;memcpy(session+4*i,&v,4);
    }
    RNG->CR&=~RNG_CR_RNGEN;return HAL_OK;
}
