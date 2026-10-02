#pragma once
#include <stdint.h>
#include <string.h>
#define __attribute__(x)
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { void *hdmarx; uint32_t ErrorCode; } UART_HandleTypeDef;
static struct { uint32_t CCR; } test_scb;
static struct { uint32_t DEMCR; } test_debug;
static struct { uint32_t CYCCNT, CTRL; } test_dwt;
#define SCB (&test_scb)
#define CoreDebug (&test_debug)
#define DWT (&test_dwt)
#define SCB_CCR_DC_Msk (1U << 16)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
static uint32_t SystemCoreClock = 480000000U;
static uint32_t test_tick;
static uint8_t test_uart_bytes[128];
static uint16_t test_uart_size;
static uint32_t test_uart_timeout;
static uint32_t test_uart_duration_ms, test_uart_calls;
static HAL_StatusTypeDef test_uart_status = HAL_OK;
static HAL_StatusTypeDef test_dma_status = HAL_OK, test_abort_status = HAL_OK;
static uint32_t test_dma_calls, test_abort_calls, test_clean_calls, test_clean_size;
static uint8_t *test_dma_source;
static uint16_t test_dma_size;
static uint8_t test_abort_immediate;
static uint32_t test_rx_remaining;
static HAL_StatusTypeDef test_rx_status = HAL_OK, test_rx_abort_status = HAL_OK;
static HAL_StatusTypeDef test_rx_it_status = HAL_OK;
static uint8_t *test_rx_it_buffer;
static uint32_t test_rx_it_calls;
#define __HAL_DMA_GET_COUNTER(a) test_rx_remaining
static void (*test_clean_hook)(void);
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *uart);
#define __DSB() ((void)0)
#define __DMB() ((void)0)
#define __HAL_DMA_DISABLE_IT(a,b) ((void)0)
#define __HAL_UART_CLEAR_FLAG(a,b) ((void)0)
#define __HAL_UART_SEND_REQ(a,b) ((void)0)
static uint32_t HAL_GetTick(void) { return test_tick; }
static void SCB_CleanInvalidateDCache_by_Addr(uint32_t *p, uint32_t n) { (void)p; (void)n; }
static void SCB_InvalidateDCache_by_Addr(uint32_t *p, uint32_t n) { (void)p; (void)n; }
static void SCB_CleanDCache_by_Addr(uint32_t *p, uint32_t n)
{ (void)p; test_clean_calls++; test_clean_size=n; if(test_clean_hook) test_clean_hook(); }
static HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u, uint8_t *p, uint32_t n)
{ (void)u; (void)p; test_rx_remaining=n; return test_rx_status; }
static HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) { (void)u; return test_rx_abort_status; }
static HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{ (void)u; (void)n; test_rx_it_buffer=p; test_rx_it_calls++; return test_rx_it_status; }
static HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, const uint8_t *p, uint16_t n, uint32_t timeout)
{ (void)u; memcpy(test_uart_bytes, p, n); test_uart_size=n; test_uart_timeout=timeout;
  test_uart_calls++; test_tick+=test_uart_duration_ms; return test_uart_status; }
static HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{ (void)u; test_dma_calls++; test_dma_source=p; test_dma_size=n; return test_dma_status; }
static HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *u)
{ test_abort_calls++; if(test_abort_immediate && test_abort_status==HAL_OK) HAL_UART_AbortTransmitCpltCallback(u);
  return test_abort_status; }
