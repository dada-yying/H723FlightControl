#pragma once
#include <stdint.h>
#include <string.h>
#define __attribute__(x)
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT} HAL_StatusTypeDef;
typedef struct {uint32_t Instance;struct {uint32_t Request,Direction,PeriphInc,MemInc,PeriphDataAlignment,MemDataAlignment,Mode,Priority,FIFOMode;} Init;} DMA_HandleTypeDef;
typedef struct {uint32_t SR,CR1,CR2,CFG1,IER,TXDR,RXDR,IFCR;} SPI_TypeDef;
#define HAL_SPI_STATE_READY 1U
#define HAL_SPI_STATE_BUSY_TX_RX 5U
#define HAL_SPI_ERROR_NONE 0U
#define HAL_SPI_ERROR_TIMEOUT 256U
#define SPI_CR1_SPE 1U
#define SPI_CR1_CSTART 512U
#define SPI_CR2_TSIZE 65535U
#define SPI_CFG1_RXDMAEN (1U<<14)
#define SPI_CFG1_TXDMAEN (1U<<15)
#define SPI_SR_RXP 1U
#define SPI_SR_RXWNE 0x8000U
#define SPI_SR_RXPLVL 0x6000U
#define SPI_SR_RXPLVL_0 0x2000U
#define SPI_SR_TXP 2U
#define SPI_SR_EOT 8U
#define SPI_SR_UDR 32U
#define SPI_SR_OVR 64U
#define SPI_SR_TIFRE 256U
#define SPI_SR_MODF 512U
#define SPI_IFCR_EOTC 8U
#define SPI_IFCR_TXTFC 16U
#define SPI_IFCR_UDRC 32U
#define SPI_IFCR_OVRC 64U
#define SPI_IFCR_TIFREC 256U
#define SPI_IFCR_MODFC 512U
#define SPI_IFCR_SUSPC 2048U
typedef struct {SPI_TypeDef *Instance;void *hdmatx,*hdmarx;uint32_t ErrorCode,State;
 const uint8_t *pTxBuffPtr;uint8_t *pRxBuffPtr;uint16_t TxXferCount,RxXferCount,TxXferSize,RxXferSize;
 void *TxISR,*RxISR;
 struct {uint32_t Mode,Direction,DataSize,CLKPolarity,CLKPhase,NSS,BaudRatePrescaler,FirstBit,TIMode,CRCCalculation,NSSPMode,NSSPolarity,FifoThreshold,MasterKeepIOState;} Init;} SPI_HandleTypeDef;
typedef struct {uint32_t Pin,Mode,Speed,Pull,Alternate;} GPIO_InitTypeDef;
typedef struct {uint32_t PeriphClockSelection,RngClockSelection;} RCC_PeriphCLKInitTypeDef;
typedef struct {uint32_t BSRR,IDR,MODER,PUPDR;} GPIO_TypeDef;
static GPIO_TypeDef gpio[3];
#define GPIOB (&gpio[0])
#define GPIOD (&gpio[1])
#define GPIOE (&gpio[2])
static struct {uint32_t CYCCNT,CTRL;} dwt;
static struct {uint32_t DEMCR;} debug;
static struct {uint32_t CCR;} scb;
static struct {uint32_t CR,SR,DR;} rng;
#define DWT (&dwt)
#define CoreDebug (&debug)
#define SCB (&scb)
#define RNG (&rng)
static const uint32_t SystemCoreClock=480000000;
extern uint32_t fake_tick;
static void (*test_tick_hook)(void);
static uint32_t HAL_GetTick(void){if(test_tick_hook)test_tick_hook();return fake_tick;}
static void HAL_GPIO_Init(void *p,GPIO_InitTypeDef *g){(void)p;(void)g;}
static void HAL_NVIC_DisableIRQ(int x){(void)x;}
static void HAL_NVIC_EnableIRQ(int x){(void)x;}
static void HAL_NVIC_ClearPendingIRQ(int x){(void)x;}
static void HAL_NVIC_SetPriority(int x,int y,int z){(void)x;(void)y;(void)z;}
static HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *p){p->State=HAL_SPI_STATE_READY;memset(p->Instance,0,sizeof(*p->Instance));return HAL_OK;}
static HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *p){(void)p;return HAL_OK;}
static HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *p){(void)p;return HAL_OK;}
static HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *p){(void)p;return HAL_OK;}
static HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *p){(void)p;return HAL_OK;}
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *,uint8_t *,uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *,const uint8_t *,uint8_t *,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *);
static unsigned cleaned,invalidated,last_clean_size,last_invalidate_size;
static void SCB_CleanDCache_by_Addr(uint32_t *p,int n){(void)p;cleaned++;last_clean_size=n;}
static void SCB_CleanInvalidateDCache_by_Addr(uint32_t *p,int n){(void)p;cleaned++;last_clean_size=n;}
static void SCB_InvalidateDCache_by_Addr(uint32_t *p,int n){(void)p;invalidated++;last_invalidate_size=n;}
#define __DSB() ((void)0)
#define __HAL_LINKDMA(p,f,d) ((p)->f=&(d))
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)
#define __HAL_RCC_SPI1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_DMA1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_SPI1_FORCE_RESET() ((void)0)
#define __HAL_RCC_SPI1_RELEASE_RESET() ((void)0)
#define __HAL_RCC_HSI48_ENABLE() ((void)0)
#define __HAL_RCC_RNG_CLK_ENABLE() ((void)0)
#define __HAL_RCC_RNG_FORCE_RESET() ((void)0)
#define __HAL_RCC_RNG_RELEASE_RESET() ((void)0)
#define __HAL_RCC_GET_FLAG(x) 1
#define GPIO_PIN_3 8
#define GPIO_PIN_4 16
#define GPIO_PIN_7 128
#define GPIO_PIN_15 32768
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_MODE_INPUT 2
#define GPIO_MODE_AF_PP 3
#define GPIO_SPEED_FREQ_HIGH 1
#define GPIO_SPEED_FREQ_VERY_HIGH 2
#define GPIO_PULLUP 1
#define GPIO_AF5_SPI1 5
static SPI_TypeDef fake_spi1, fake_spi2, fake_spi6;
#define SPI1 (&fake_spi1)
#define SPI2 (&fake_spi2)
#define SPI6 (&fake_spi6)
#define __HAL_DMA_GET_COUNTER(d) 0U
static unsigned fake_nops;
static void (*test_nop_hook)(void);
#define __NOP() do {fake_nops++;if(test_nop_hook)test_nop_hook();} while(0)
#define SPI1_IRQn 1
#define DMA1_Stream1 1
#define DMA1_Stream3 3
#define DMA1_Stream1_IRQn 2
#define DMA1_Stream3_IRQn 3
#define DMA_REQUEST_SPI1_RX 1
#define DMA_REQUEST_SPI1_TX 2
#define DMA_PERIPH_TO_MEMORY 1
#define DMA_MEMORY_TO_PERIPH 2
#define DMA_PINC_DISABLE 0
#define DMA_MINC_ENABLE 1
#define DMA_PDATAALIGN_BYTE 0
#define DMA_MDATAALIGN_BYTE 0
#define DMA_NORMAL 0
#define DMA_PRIORITY_LOW 0
#define DMA_FIFOMODE_DISABLE 0
#define SPI_MODE_MASTER 1
#define SPI_DIRECTION_2LINES 0
#define SPI_DATASIZE_8BIT 8
#define SPI_POLARITY_LOW 0
#define SPI_PHASE_1EDGE 0
#define SPI_NSS_SOFT 1
#define SPI_BAUDRATEPRESCALER_32 32
#define SPI_FIRSTBIT_MSB 0
#define SPI_TIMODE_DISABLE 0
#define SPI_CRCCALCULATION_DISABLE 0
#define SPI_NSS_PULSE_DISABLE 0
#define SPI_NSS_POLARITY_LOW 0
#define SPI_FIFO_THRESHOLD_01DATA 0
#define SPI_MASTER_KEEP_IO_STATE_ENABLE 1
#define SCB_CCR_DC_Msk 1
#define CoreDebug_DEMCR_TRCENA_Msk 1
#define DWT_CTRL_CYCCNTENA_Msk 1
#define RCC_FLAG_HSI48RDY 1
#define RCC_PERIPHCLK_RNG 1
#define RCC_RNGCLKSOURCE_HSI48 1
#define RNG_CR_RNGEN 1
#define RNG_SR_DRDY 1
#define RNG_SR_CECS 2
#define RNG_SR_SECS 4
