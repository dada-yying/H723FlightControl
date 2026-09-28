#pragma once
#include <stdint.h>
#include <string.h>
#define __attribute__(x)
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef struct { void *hdmarx; } UART_HandleTypeDef;
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
#define __DSB() ((void)0)
#define __DMB() ((void)0)
#define __HAL_DMA_DISABLE_IT(a,b) ((void)0)
#define __HAL_UART_CLEAR_FLAG(a,b) ((void)0)
#define __HAL_UART_SEND_REQ(a,b) ((void)0)
static uint32_t HAL_GetTick(void) { return test_tick; }
static void SCB_CleanInvalidateDCache_by_Addr(uint32_t *p, uint32_t n) { (void)p; (void)n; }
static void SCB_InvalidateDCache_by_Addr(uint32_t *p, uint32_t n) { (void)p; (void)n; }
static HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u, uint8_t *p, uint32_t n)
{ (void)u; (void)p; (void)n; return HAL_OK; }
static HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) { (void)u; return HAL_OK; }
static HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, uint8_t *p, uint16_t n, uint32_t timeout)
{ (void)u; memcpy(test_uart_bytes, p, n); test_uart_size=n; test_uart_timeout=timeout; return HAL_OK; }
