/* Execute production spi_byte(), including its actual bit loop and capture.
 * Model BSRR/IDR registers and a mode-0 card; no byte-level replacement. */
#include "stm32h7xx_hal.h"
#include <assert.h>
#include <stdio.h>
static void model_write(GPIO_TypeDef *,uint32_t);
static uint32_t model_read(GPIO_TypeDef *);
static void model_delay(void);
#define SD_SPI_GPIO_WRITE model_write
#define SD_SPI_GPIO_READ model_read
#define SD_SPI_TEST_DELAY model_delay
#include "../App/storage/sd_spi.c"
uint32_t fake_tick;
static unsigned selected,command_bits,response_bits,in_response;
static unsigned fixed_mode,fixed_bits,fixed_rx,mosi_bits,wire_bit_count;
static unsigned readback_mosi_bad,readback_sck_bad,high_phase_ones;
static uint8_t wire_command[6],responses[32];
static void prepare_miso(void)
{
    unsigned bit;
    if(fixed_mode)bit=(fixed_rx>>(7-(fixed_bits&7)))&1;
    else if(selected && in_response && response_bits<sizeof(responses)*8)
        bit=(responses[response_bits/8]>>(7-(response_bits&7)))&1;
    else bit=1;
    GPIOB->IDR=(GPIOB->IDR&~GPIO_PIN_4)|(bit?GPIO_PIN_4:0);
}
static void model_write(GPIO_TypeDef *p,uint32_t value)
{
    uint32_t old=p->IDR;
    p->BSRR=value;p->IDR=(old|(value&0xffffU))&~(value>>16);
    if(p==GPIOE && ((old^p->IDR)&GPIO_PIN_15)){
        selected=!(p->IDR&GPIO_PIN_15);
        command_bits=response_bits=in_response=0;
        if(selected)memset(wire_command,0,sizeof(wire_command));
        prepare_miso();
    }
    if(p==GPIOB && ((old^p->IDR)&GPIO_PIN_3)){
        if(p->IDR&GPIO_PIN_3){
            mosi_bits=(mosi_bits<<1)|((GPIOD->IDR&GPIO_PIN_7)?1:0);wire_bit_count++;
            if(selected && !fixed_mode){
                if(in_response)response_bits++;
                else {
                    wire_command[command_bits/8]=(uint8_t)((wire_command[command_bits/8]<<1)|((GPIOD->IDR&GPIO_PIN_7)?1:0));
                    if(++command_bits==48)in_response=1;
                }
            }
        }else {if(fixed_mode)fixed_bits++;prepare_miso();}
    }
}
static uint32_t model_read(GPIO_TypeDef *p)
{
    uint32_t v=p->IDR;
    if(p==GPIOD && readback_mosi_bad)v^=GPIO_PIN_7;
    if(p==GPIOB && readback_sck_bad)v|=GPIO_PIN_3;
    if(p==GPIOB && high_phase_ones && (p->IDR&GPIO_PIN_3))v|=GPIO_PIN_4;
    return v;
}
static void model_delay(void){DWT->CYCCNT+=960;}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *s,const uint8_t *t,uint8_t *r,uint16_t n,uint32_t timeout)
{(void)s;(void)t;(void)r;(void)n;(void)timeout;assert(!"SD fast bytes must not use polling HAL");return HAL_ERROR;}
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *s,uint8_t *t,uint8_t *r,uint16_t n)
{(void)s;(void)t;(void)r;(void)n;assert(!"GPIO initialization must not start DMA");return HAL_ERROR;}
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *s){(void)s;return HAL_OK;}
static void reset_model(void)
{
    memset(gpio,0,sizeof(gpio));memset((void *)&g_sd_spi_diag,0,sizeof(g_sd_spi_diag));
    selected=command_bits=response_bits=in_response=fixed_mode=fixed_bits=0;
    mosi_bits=wire_bit_count=readback_mosi_bad=readback_sck_bad=high_phase_ones=0;
    fast=io_failed=diag_active=0;
    GPIOE->IDR=GPIO_PIN_15;
    GPIOE->MODER=1U<<30;GPIOB->MODER=1U<<6;GPIOB->PUPDR=1U<<8;GPIOD->MODER=1U<<14;
    memset(responses,0xff,sizeof(responses));responses[1]=1;prepare_miso();
    DWT->CYCCNT=0;
}
int main(void)
{
    unsigned v;SdSpiCmd0Trace_t frozen;const uint8_t expected[]={0x40,0,0,0,0,0x95};
    reset_model();fixed_mode=1;
    /* All input patterns and complemented output patterns through real MSB loop. */
    for(v=0;v<256;v++){
        fixed_rx=v;fixed_bits=0;mosi_bits=wire_bit_count=0;prepare_miso();
        assert(spi_byte((uint8_t)~v)==v && wire_bit_count==8 && (uint8_t)mosi_bits==(uint8_t)~v);
    }
    reset_model();assert(command(0,0)==1);
    assert(!memcmp(wire_command,expected,6));frozen=g_sd_spi_diag.cmd0;
    assert(frozen.complete && frozen.gpio_checked && frozen.rx_count==2 && frozen.r1==1);
    assert(frozen.cs_mode==1 && frozen.sck_mode==1 && frozen.mosi_mode==1 && !frozen.miso_mode && frozen.miso_pull==1);
    assert(!memcmp(frozen.tx,expected,6) && !memcmp(frozen.mosi_readback,expected,6));
    assert(frozen.rx[0]==0xff && frozen.rx[1]==1 && !memcmp(frozen.rx,frozen.rx_low_phase,2));
    assert(!frozen.cs_not_low_count && !frozen.sck_not_low_count && !frozen.sck_not_high_count && !frozen.mosi_mismatch_count);
    assert(frozen.duration_us==256 && frozen.bit_period_min_cycles==1920 && frozen.bit_period_max_cycles==1920);
    memset(responses,0xff,sizeof(responses));assert(command(0,0)==0xff);
    {SdSpiCmd0Trace_t after=g_sd_spi_diag.cmd0;assert(!memcmp(&frozen,&after,sizeof(after)));}
    g_sd_spi_diag.cmd0.complete=0;assert(command(0,0)==0xff);
    assert(g_sd_spi_diag.cmd0.complete && g_sd_spi_diag.cmd0.rx_count==16 && g_sd_spi_diag.cmd0.r1==0xff);
    assert(g_sd_spi_diag.response_timeouts==2);
    reset_model();readback_mosi_bad=1;readback_sck_bad=1;assert(command(0,0)==1);
    assert(g_sd_spi_diag.cmd0.mosi_mismatch_count && g_sd_spi_diag.cmd0.sck_not_low_count);
    reset_model();high_phase_ones=1;assert(command(0,0)==0xff);
    assert(g_sd_spi_diag.cmd0.rx[1]==0xff && g_sd_spi_diag.cmd0.rx_low_phase[1]==1);
    reset_model();DWT->CYCCNT=UINT32_MAX-100;assert(command(0,0)==1);
    assert(g_sd_spi_diag.cmd0.duration_us==256 && g_sd_spi_diag.cmd0.bit_period_min_cycles==1920);
    puts("PASS SD GPIO: actual MSB/mode0/readback/phase/capture, CMD0 and timestamp wrap");
    return 0;
}
