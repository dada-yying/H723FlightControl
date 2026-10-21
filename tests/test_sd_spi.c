/* Actual protocol + register polling loop + byte-by-byte DMA wire model. */
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include "sd_poll_model.h"
static void wire_cs(GPIO_TypeDef *,uint32_t);
#define SD_SPI_GPIO_WRITE wire_cs
#define SD_SPI_TEST_BYTE wire_byte
#include "../App/storage/sd_spi.c"
uint32_t fake_tick;
static uint8_t reply[4096],command_bytes[6],written[512];
static unsigned rp,rn,cp,write_pos,write_mode,read_pattern,dma_mode,aborts,stuck;
static unsigned init_reject,multi_read,corrupt_crc,selected,card_idle=1;
static unsigned preerase_reply,app_reply,write_reply=5,stop_count,cmd_counts[64];
static unsigned token_missing,preerase_arg,spi_bytes,dma_calls;
static uint16_t write_crc;
static void push(uint8_t v){assert(rn<sizeof(reply));reply[rn++]=v;}
static void push_crc(const uint8_t *p,unsigned n){uint16_t c=crc16(p,n);push(c>>8);push(c);}
static void read_block(void)
{
    uint8_t data[512];unsigned i;
    push(0xfe);for(i=0;i<512;i++){data[i]=(uint8_t)(i+read_pattern);push(data[i]);}
    push_crc(data,512);if(corrupt_crc)reply[rn-1]^=1;
}
static void respond(unsigned cmd)
{
    uint8_t reg[16]={0};unsigned i;cmd_counts[cmd]++;
    switch(cmd){
    case 0:card_idle=1;push(init_reject?0xff:1);break;
    case 8:push(1);push(0);push(0);push(1);push(0xaa);break;
    case 55:push(card_idle?1:(uint8_t)app_reply);break;
    case 41:card_idle=0;push(0);break;
    case 58:push(0);push(0xc0);push(0xff);push(0x80);push(0);break;
    case 59:push(0);break;
    case 9:
        reg[0]=0x40;reg[8]=0x1f;reg[9]=0xff;
        push(0);push(0xfe);for(i=0;i<16;i++)push(reg[i]);push_crc(reg,16);break;
    case 10:
        for(i=0;i<16;i++)reg[i]=(uint8_t)(i+1);
        push(0);push(0xfe);for(i=0;i<16;i++)push(reg[i]);push_crc(reg,16);break;
    case 17:push(0);if(!token_missing)read_block();break;
    case 18:multi_read=1;push(0);if(!token_missing)read_block();break;
    case 12:multi_read=0;push(0xff);push(0);break;
    case 23:
        preerase_arg=((unsigned)command_bytes[1]<<24)|((unsigned)command_bytes[2]<<16)|
                     ((unsigned)command_bytes[3]<<8)|command_bytes[4];
        push((uint8_t)preerase_reply);break;
    case 24:case 25:write_mode=cmd;push(0);break;
    default:assert(!"unexpected command");
    }
}
static void wire_cs(GPIO_TypeDef *p,uint32_t value)
{
    p->BSRR=value;
    if(p!=GPIOE)return;
    selected=(value&GPIO_PIN_15)?0:1;
    if(!selected){rp=rn=cp=write_pos=write_mode=multi_read=0;}
}
static uint8_t wire_byte(uint8_t out)
{
    spi_bytes++;
    if(!selected)return 0xff;
    if(stuck)return 0;
    if(write_pos){
        if(write_pos<=512)written[write_pos-1]=out;
        else if(write_pos==513)write_crc=(uint16_t)out<<8;
        else {
            write_crc|=out;assert(write_crc==crc16(written,512));
            write_pos=0;push(0xff);push((uint8_t)write_reply);push(0);push(0);push(0xff);
            return 0xff;
        }
        write_pos++;return 0xff;
    }
    if(out==0x4c && multi_read && !cp){rp=rn=0;}
    if(rp<rn){uint8_t v=reply[rp++];if(rp==rn)rp=rn=0;return v;}
    if(cp || (out&0xc0)==0x40){
        command_bytes[cp++]=out;
        if(cp==6){assert(crc7(command_bytes,5)==command_bytes[5]);cp=0;respond(command_bytes[0]&63);}
        return 0xff;
    }
    if(out==0xfd){assert(write_mode==25);write_mode=0;stop_count++;push(0xff);push(0);push(0xff);return 0xff;}
    if(out==0xfe || out==0xfc){assert(write_mode==(out==0xfe?24:25));write_pos=1;return 0xff;}
    if(multi_read && !token_missing){read_block();return 0xff;}
    return 0xff;
}
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *p)
{aborts++;p->State=HAL_SPI_STATE_READY;memset(p->Instance,0,sizeof(*p->Instance));return HAL_OK;}
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *s,uint8_t *tx,uint8_t *rx,uint16_t n)
{
    unsigned i;assert(n==514 || n==516);assert(cleaned>=2 && last_clean_size==544);
    assert(selected && transport==TRANSPORT_DMA);dma_calls++;
    if(dma_mode==1)return HAL_BUSY;
    if(dma_mode==2)return HAL_OK;
    if(dma_mode==3){HAL_SPI_ErrorCallback(s);return HAL_OK;}
    for(i=0;i<n;i++)rx[i]=wire_byte(tx[i]);
    DWT->CYCCNT+=4800;HAL_SPI_TxRxCpltCallback(s);
    if(dma_mode==4){fake_tick+=120;DWT->CYCCNT+=480000;}
    return HAL_OK;
}
static void reinit(void)
{
    stuck=init_reject=token_missing=corrupt_crc=0;dma_mode=poll_mode=POLL_NORMAL;
    app_reply=preerase_reply=0;write_reply=5;
    g_sd_spi_diag.io_fault.complete=0;
    assert(disk_initialize(0)==0);
}
int main(void)
{
    uint8_t b[1024];uint32_t value;unsigned i,j,before,calls;
    SCB->CCR=SCB_CCR_DC_Msk;
    assert(crc7((uint8_t[]){0x40,0,0,0,0},5)==0x95);
    reinit();assert(high_capacity && fast && sectors==8388608);
    assert(g_sd_spi_diag.revision==10 && g_sd_spi_diag.stage==SD_INIT_READY);
    assert(g_sd_spi_diag.cmd0.complete && g_sd_spi_diag.cmd0.r1==1);
    assert(disk_ioctl(0,GET_SECTOR_COUNT,&value)==RES_OK && value==8388608);
    read_pattern=37;assert(disk_read(0,b,100,1)==RES_OK);
    for(i=0;i<512;i++)assert(b[i]==(uint8_t)(i+37));
    assert(disk_read(0,b,200,2)==RES_OK && !multi_read && !selected);
    for(i=0;i<1024;i++)assert(b[i]==(uint8_t)(i+37));
    for(i=0;i<1024;i++)b[i]=(uint8_t)(i*7);
    calls=dma_calls;
    assert(disk_write(0,b,101,2)==RES_OK && !memcmp(written,b+512,512));
    assert(dma_calls-calls==2 && stop_count==1 && !selected && !rn);
    assert(preerase_arg==2 && g_sd_spi_diag.preerase_accepted==1);
    assert(!g_sd_spi_diag.byte_dma_count && g_sd_spi_diag.block_dma_count==5);
    assert(g_sd_spi_diag.dma_bytes==3*514+2*516 && last_invalidate_size==544);
    assert(g_sd_spi_diag.dma_latency_samples==5 && g_sd_spi_diag.dma_irq_total_us==50);
    assert(poll_six_count>=5);
    assert(disk_read(0,b,sectors,1)==RES_PARERR);
    assert(disk_write(0,b,102,1)==RES_OK && !selected);
    for(j=1;j<=4;j++){
        reinit();dma_mode=j;i=aborts;
        if(j==4){
            assert(disk_read(0,b,10,1)==RES_OK);
            assert(g_sd_spi_diag.dma_to_irq_max_cycles>=4800 && g_sd_spi_diag.dma_resume_max_cycles>=480000);
            assert(g_sd_spi_diag.dma_resume_total_us>=1000);
            continue;
        }
        assert(disk_read(0,b,10,1)==RES_ERROR && aborts==i+1 && disk_state==STA_NOINIT);
        assert(g_sd_spi_diag.io_fault.kind==(j==1?SD_IO_FAULT_DMA_START:(j==2?SD_IO_FAULT_DMA_WAIT:SD_IO_FAULT_DMA_IRQ)));
        before=g_sd_spi_diag.ignored_callback_count;
        HAL_SPI_TxRxCpltCallback(&hspi1_sd);HAL_SPI_ErrorCallback(&hspi1_sd);
        assert(g_sd_spi_diag.ignored_callback_count==before+2 && transport==TRANSPORT_IDLE);
    }
    reinit();stuck=1;i=fake_tick;
    assert(disk_ioctl(0,CTRL_SYNC,0)==RES_ERROR && fake_tick-i>=500);
    reinit();token_missing=1;
    assert(disk_read(0,b,10,1)==RES_ERROR && g_sd_spi_diag.io_fault.kind==SD_IO_FAULT_READ_TOKEN);
    reinit();init_reject=1;assert(disk_initialize(0)==STA_NOINIT);
    assert(g_sd_spi_diag.failed_stage==SD_INIT_CMD0 && g_sd_spi_diag.failure_r1==0xff);
    reinit();corrupt_crc=1;memset(b,0xa5,sizeof(b));
    assert(disk_read(0,b,10,1)==RES_ERROR && g_sd_spi_diag.io_fault.kind==SD_IO_FAULT_DATA_CRC);
    for(i=0;i<sizeof(b);i++)assert(b[i]==0xa5);
    reinit();write_reply=0x0b;
    assert(disk_write(0,b,10,1)==RES_ERROR && g_sd_spi_diag.io_fault.kind==SD_IO_FAULT_DATA_RESPONSE);
    for(j=0;j<2;j++){
        reinit();if(j)app_reply=4;else preerase_reply=4;
        before=g_sd_spi_diag.preerase_unsupported;
        assert(disk_write(0,b,10,2)==RES_OK && g_sd_spi_diag.preerase_unsupported==before+1);
    }
    for(j=0;j<3;j++){
        reinit();preerase_reply=j==0?0xff:(j==1?0x0c:1);
        before=cmd_counts[25];calls=dma_calls;i=stop_count;
        assert(disk_write(0,b,10,2)==RES_ERROR && cmd_counts[25]==before && dma_calls==calls && stop_count==i);
        assert(g_sd_spi_diag.io_fault.kind==SD_IO_FAULT_COMMAND);
    }
    reinit();poll_mode=POLL_PREEMPT_DONE;poll_preempted=0;fake_tick=UINT32_MAX-3;
    assert(disk_ioctl(0,CTRL_SYNC,0)==RES_OK && poll_preempted);
    /* Same delayed-byte event through full commands/read/write, not just a
     * standalone poll call. Exercise each byte position in the six-byte CMD. */
    for(i=1;i<=6;i++){
        unsigned checks;
        reinit();poll_mode=POLL_DEADLINE_RX;poll_preempted=0;poll_deadline_byte=i;
        checks=g_sd_spi_diag.short_deadline_rechecks;
        assert(disk_read(0,b,10,1)==RES_OK && poll_preempted);
        assert(g_sd_spi_diag.short_deadline_rechecks==checks+1 && !g_sd_spi_diag.io_fault.complete);
        poll_preempted=0;
        assert(disk_write(0,b,11,1)==RES_OK && poll_preempted);
    }
    reinit();poll_mode=POLL_FIFO_TAIL;
    assert(disk_read(0,b,10,2)==RES_OK && disk_write(0,b,10,2)==RES_OK);
    assert(g_sd_spi_diag.short_fifo_tail_reads>0 && !g_sd_spi_diag.io_fault.complete);
    puts("PASS real SD wire: command/CRC/CS ordering, 514/516 DMA, read/multiwrite/stop, preerase support/errors, timeout/late IRQ/recovery, unchanged buffers after bad CRC");
    return 0;
}
