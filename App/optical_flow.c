#include "optical_flow.h"
#include "usart.h"
#include <string.h>

#define OF_DMA_SIZE 256U
#define OF_QUEUE_SIZE 1024U
__attribute__((section(".optical_flow_dma"), aligned(32)))
static uint8_t of_dma[OF_DMA_SIZE];
static uint8_t of_queue[OF_QUEUE_SIZE];
static uint32_t of_ticks[OF_QUEUE_SIZE];
static volatile uint32_t of_head, of_tail;
static uint32_t of_dma_pos, of_retry_tick;
static volatile uint8_t of_restart;
static uint8_t of_retry_wait;
static TaskHandle_t of_task;
static uint8_t of_parse[71];
static uint32_t of_parse_ticks[71];
static uint32_t of_parse_size;
/* Task/getter-maintained extended clock; serviced at least every 10 ms.
 * Queue timestamps are mapped to this clock before publishing a sample. */
static uint32_t of_clock_tick;
static uint64_t of_clock_ms, of_sample_ms;
volatile OpticalFlowData_t g_optical_flow_data;
volatile OpticalFlowStats_t g_optical_flow_stats;

static uint64_t OF_Now(void)
{
    uint32_t now = HAL_GetTick();
    of_clock_ms += (uint32_t)(now - of_clock_tick);
    of_clock_tick = now;
    return of_clock_ms;
}
static uint32_t OF_U32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint16_t OF_U16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static void OF_Notify(void)
{
    BaseType_t woken = pdFALSE;
    if (of_task != NULL) vTaskNotifyGiveFromISR(of_task, &woken);
    portYIELD_FROM_ISR(woken);
}
void OpticalFlow_Init(HAL_StatusTypeDef status)
{
    memset((void *)&g_optical_flow_data, 0, sizeof(g_optical_flow_data));
    memset((void *)&g_optical_flow_stats, 0, sizeof(g_optical_flow_stats));
    of_head = of_tail = of_dma_pos = of_parse_size = 0;
    of_restart = of_retry_wait = 0;
    of_task = NULL;
    of_clock_tick = HAL_GetTick();
    of_clock_ms = of_clock_tick;
    of_sample_ms = 0;
    g_optical_flow_data.age_ms = UINT32_MAX;
    g_optical_flow_stats.last_status = status;
    g_optical_flow_stats.initialized = (status == HAL_OK);
}
void OpticalFlow_SetTaskHandle(TaskHandle_t handle) { of_task = handle; }

HAL_StatusTypeDef OpticalFlow_Start(void)
{
    HAL_StatusTypeDef status;
    if (!g_optical_flow_stats.initialized) return HAL_ERROR;
    /* Called by OpticalFlowTask only. Stop callbacks before resetting the ring.
     * HAL abort is bounded; no peripheral init failure reaches Error_Handler. */
    g_optical_flow_stats.receiving = 0;
    status = HAL_UART_AbortReceive(&huart10);
    taskENTER_CRITICAL();
    of_head = of_tail = of_dma_pos = of_parse_size = 0;
    of_restart = 0;
    taskEXIT_CRITICAL();
    if (status == HAL_OK)
    {
        __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_PEF | UART_CLEAR_FEF |
                             UART_CLEAR_NEF | UART_CLEAR_OREF | UART_CLEAR_IDLEF);
        __HAL_UART_SEND_REQ(&huart10, UART_RXDATA_FLUSH_REQUEST);
        if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
            SCB_CleanInvalidateDCache_by_Addr((uint32_t *)of_dma, sizeof(of_dma));
        __DSB();
        /* Set before HAL enables DMA interrupts; an immediate error must persist. */
        g_optical_flow_stats.receiving = 1;
        status = HAL_UARTEx_ReceiveToIdle_DMA(&huart10, of_dma, sizeof(of_dma));
    }
    g_optical_flow_stats.last_status = status;
    g_optical_flow_stats.restart_count++;
    if (status != HAL_OK)
    {
        g_optical_flow_stats.receiving = 0;
        g_optical_flow_stats.restart_failures++;
        of_restart = 1;
    }
    of_retry_tick = HAL_GetTick();
    of_retry_wait = (status != HAL_OK);
    return status;
}
void OpticalFlow_OnRxEvent(UART_HandleTypeDef *uart)
{
    uint32_t pos, tick;
    if (uart != &huart10 || !g_optical_flow_stats.receiving || of_restart) return;
    pos = (OF_DMA_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx)) % OF_DMA_SIZE;
    tick = HAL_GetTick();
    if (pos == of_dma_pos) return;
    /* RX buffer is read-only for CPU. Invalidate whole owned cache lines, never
     * clean live RX data back over DMA writes. HT interrupts remain enabled. */
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
        SCB_InvalidateDCache_by_Addr((uint32_t *)of_dma, sizeof(of_dma));
    __DSB();
    while (of_dma_pos != pos)
    {
        uint32_t head = of_head;
        if ((uint32_t)(head - of_tail) >= OF_QUEUE_SIZE)
        {
            g_optical_flow_stats.queue_overflows++;
            of_restart = 1;
            break;
        }
        of_queue[head % OF_QUEUE_SIZE] = of_dma[of_dma_pos];
        of_ticks[head % OF_QUEUE_SIZE] = tick;
        __DMB();
        of_head = head + 1U;
        of_dma_pos = (of_dma_pos + 1U) % OF_DMA_SIZE;
        g_optical_flow_stats.byte_count++;
    }
    OF_Notify();
}
void OpticalFlow_OnError(UART_HandleTypeDef *uart)
{
    if (uart != &huart10) return;
    g_optical_flow_stats.uart_errors++;
    g_optical_flow_stats.last_uart_error = uart->ErrorCode;
    of_restart = 1;
    OF_Notify();
}
static void OF_Publish(uint32_t tick)
{
    OpticalFlowData_t d = {0};
    const uint8_t *p = &of_parse[6];
    d.head = of_parse[0]; d.dev_id = of_parse[1]; d.sys_id = of_parse[2];
    d.msg_id = of_parse[3]; d.seq = of_parse[4]; d.len = of_parse[5];
    d.time_ms = OF_U32(p); d.distance = OF_U32(p + 4);
    d.strength = p[8]; d.precision = p[9]; d.tof_status = p[10];
    d.reserved1 = p[11];
    d.flow_vel_x = (int16_t)OF_U16(p + 12); d.flow_vel_y = (int16_t)OF_U16(p + 14);
    d.flow_quality = p[16]; d.flow_status = p[17]; d.reserved2 = OF_U16(p + 18);
    d.checksum = of_parse[26]; d.has_sample = 1;
    d.received_tick = tick;
    memcpy(d.raw, of_parse, sizeof(d.raw));
    taskENTER_CRITICAL();
    /* Never publish queued bytes across a discontinuity signalled by an ISR. */
    if (!of_restart)
    {
        uint64_t now = OF_Now();
        uint32_t age = (uint32_t)(of_clock_tick - tick);
        of_sample_ms = now - age;
        d.age_ms = age;
        d.rx_count = g_optical_flow_data.rx_count + 1U;
        g_optical_flow_data = d;
        g_optical_flow_stats.frame_count++;
    }
    taskEXIT_CRITICAL();
}
static void OF_Byte(uint8_t byte, uint32_t tick)
{
    uint32_t i, size;
    uint8_t sum;
    of_parse_ticks[of_parse_size] = tick;
    of_parse[of_parse_size++] = byte;
    while (of_parse_size != 0U)
    {
        if (of_parse[0] != 0xEFU) goto discard;
        if (of_parse_size < 6U) return;
        if (of_parse[5] > 64U || (of_parse[3] == 0x51U && of_parse[5] != 20U))
        {
            g_optical_flow_stats.length_errors++;
            goto discard;
        }
        size = (uint32_t)of_parse[5] + 7U;
        if (of_parse_size < size) return;
        sum = 0;
        for (i = 0; i < size - 1U; ++i) sum = (uint8_t)(sum + of_parse[i]);
        if (sum != of_parse[size - 1U])
        {
            g_optical_flow_stats.checksum_errors++;
            goto discard;
        }
        if (of_parse[3] == 0x51U) OF_Publish(of_parse_ticks[size - 1U]);
        of_parse_size -= size;
        memmove(of_parse, of_parse + size, of_parse_size);
        memmove(of_parse_ticks, of_parse_ticks + size, of_parse_size * sizeof(uint32_t));
        continue;
discard:
        --of_parse_size;
        memmove(of_parse, of_parse + 1, of_parse_size);
        memmove(of_parse_ticks, of_parse_ticks + 1, of_parse_size * sizeof(uint32_t));
    }
}
void OpticalFlow_GetData(OpticalFlowData_t *data)
{
    uint64_t age, now;
    taskENTER_CRITICAL();
    now = OF_Now();
    age = g_optical_flow_data.has_sample ? now - of_sample_ms : UINT32_MAX;
    g_optical_flow_data.age_ms = age >= UINT32_MAX ? UINT32_MAX : (uint32_t)age;
    if (data != NULL) *data = g_optical_flow_data;
    taskEXIT_CRITICAL();
}
void OpticalFlow_ProcessPending(void)
{
    g_optical_flow_stats.task_count++;
    OpticalFlow_GetData(NULL);
    if (!g_optical_flow_stats.initialized) return;
    if (of_restart || !g_optical_flow_stats.receiving)
    {
        if (!of_retry_wait || (uint32_t)(HAL_GetTick() - of_retry_tick) >= 100U)
            (void)OpticalFlow_Start();
        return;
    }
    /* Bounded work per activation, producer cannot starve this task. */
    {
        uint32_t remaining = OF_QUEUE_SIZE;
        while (remaining-- && !of_restart && of_tail != of_head)
        {
            uint32_t tail = of_tail;
            uint8_t byte = of_queue[tail % OF_QUEUE_SIZE];
            uint32_t tick = of_ticks[tail % OF_QUEUE_SIZE];
            __DMB();
            of_tail = tail + 1U;
            OF_Byte(byte, tick);
        }
    }
}
