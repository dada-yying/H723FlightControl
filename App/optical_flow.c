#include "optical_flow.h"
#include "usart.h"
#include <string.h>

#define OF_DMA_SIZE 256U
#define OF_QUEUE_SIZE 1024U
__attribute__((section(".optical_flow_dma"), aligned(32))) static uint8_t of_dma[OF_DMA_SIZE];
static uint8_t of_queue[OF_QUEUE_SIZE];
static uint32_t of_ticks[OF_QUEUE_SIZE];
static volatile uint32_t of_head, of_tail;
static uint32_t of_dma_pos, of_retry_tick;
static volatile uint8_t of_restart;
static uint8_t of_retry_wait;
static uint8_t of_it_byte;
static TaskHandle_t of_task;
static uint8_t of_parse[71];
static uint32_t of_parse_ticks[71];
static uint32_t of_parse_size;
/* Task/getter-maintained extended clock; serviced at least every 10 ms.
 * Queue timestamps are mapped to this clock before publishing a sample. */
static uint32_t of_clock_tick;
static uint64_t of_clock_ms, of_sample_ms;
static OpticalFlowData_t g_optical_flow_data;
static OpticalFlowStats_t s_optical_task_stats;
volatile OpticalFlowCompatibilityStats_t g_optical_flow_stats;
static OpticalFlowDiag_t g_optical_flow_diag;
static OpticalFlowDiag_t s_optical_task_published;
static OpticalFlowStats_t s_optical_stats_published;
static volatile uint32_t s_requested_rx_mode;
static volatile OpticalFlowDiag_t s_optical_isr_diag;
static volatile OpticalFlowStats_t s_optical_isr_stats;
static uint8_t of_diag_rx_seen, of_diag_task_seen, of_diag_seq_seen;

/* Task-only observation, before publication. Raw latches survive later zero
 * frames and UART recovery; only OpticalFlow_Init clears them. */
static void OF_DiagFrame(const OpticalFlowData_t *d)
{
    uint32_t i;
    uint32_t mode = (g_optical_flow_diag.rx_mode == OPTICAL_FLOW_RX_IT) ? OPTICAL_FLOW_RX_IT
                                                                        : OPTICAL_FLOW_RX_DMA;
    uint16_t raw_x = (uint16_t)((uint16_t)d->raw[18] | ((uint16_t)d->raw[19] << 8));
    uint16_t raw_y = (uint16_t)((uint16_t)d->raw[20] | ((uint16_t)d->raw[21] << 8));
    /* Frame capture is task-owned; consistency comes from the snapshot lock. */
    g_optical_flow_diag.valid_frame_count++;
    g_optical_flow_diag.rx_mode_valid_frame_count[mode]++;
    if (raw_x != 0U || raw_y != 0U)
        g_optical_flow_diag.rx_mode_raw_nonzero_frame_count[mode]++;
    if (raw_x != (uint16_t)d->flow_vel_x || raw_y != (uint16_t)d->flow_vel_y)
        g_optical_flow_diag.raw_decode_mismatch_count++;
    if (of_diag_seq_seen && d->dev_id == g_optical_flow_diag.last_dev_id &&
        d->sys_id == g_optical_flow_diag.last_sys_id)
    {
        uint8_t delta = (uint8_t)(d->seq - g_optical_flow_diag.last_seq);
        if (delta != 1U)
            g_optical_flow_diag.seq_discontinuity_count++;
        if (delta == 0U)
            g_optical_flow_diag.duplicate_seq_count++;
    }
    of_diag_seq_seen = 1;
    g_optical_flow_diag.last_seq = d->seq;
    g_optical_flow_diag.last_dev_id = d->dev_id;
    g_optical_flow_diag.last_sys_id = d->sys_id;
    for (i = 0; i < OPTICAL_FLOW_FRAME_SIZE; ++i)
        g_optical_flow_diag.last_valid_raw[i] = d->raw[i];
    if (d->flow_vel_x != 0 || d->flow_vel_y != 0)
    {
        g_optical_flow_diag.nonzero_frame_count++;
        g_optical_flow_diag.rx_mode_nonzero_frame_count[mode]++;
        g_optical_flow_diag.last_nonzero_x = d->flow_vel_x;
        g_optical_flow_diag.last_nonzero_y = d->flow_vel_y;
        g_optical_flow_diag.last_nonzero_seq = d->seq;
        g_optical_flow_diag.last_nonzero_sensor_ms = d->time_ms;
        g_optical_flow_diag.last_nonzero_rx_tick = d->received_tick;
        for (i = 0; i < OPTICAL_FLOW_FRAME_SIZE; ++i)
            g_optical_flow_diag.last_nonzero_raw[i] = d->raw[i];
    }
    __DMB();
    g_optical_flow_diag.frame_capture_seq += 2U;
}

static uint64_t OF_Now(void)
{
    uint32_t now = HAL_GetTick();
    of_clock_ms += (uint32_t)(now - of_clock_tick);
    of_clock_tick = now;
    return of_clock_ms;
}
static uint32_t OF_U32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint16_t OF_U16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static void OF_Notify(void)
{
    BaseType_t woken = pdFALSE;
    if (of_task != NULL)
        vTaskNotifyGiveFromISR(of_task, &woken);
    portYIELD_FROM_ISR(woken);
}
void OpticalFlow_Init(HAL_StatusTypeDef status)
{
    memset((void *)&g_optical_flow_data, 0, sizeof(g_optical_flow_data));
    memset((void *)&s_optical_task_stats, 0, sizeof(s_optical_task_stats));
    memset((void *)&g_optical_flow_stats, 0, sizeof(g_optical_flow_stats));
    memset((void *)&g_optical_flow_diag, 0, sizeof(g_optical_flow_diag));
    memset((void *)&s_optical_isr_diag, 0, sizeof(s_optical_isr_diag));
    memset((void *)&s_optical_isr_stats, 0, sizeof(s_optical_isr_stats));
    g_optical_flow_diag.version = 0x00010002U;
    s_requested_rx_mode = 0;
    of_diag_rx_seen = of_diag_task_seen = of_diag_seq_seen = 0;
    of_head = of_tail = of_dma_pos = of_parse_size = 0;
    of_restart = of_retry_wait = 0;
    of_task = NULL;
    of_clock_tick = HAL_GetTick();
    of_clock_ms = of_clock_tick;
    of_sample_ms = 0;
    g_optical_flow_data.age_ms = UINT32_MAX;
    s_optical_task_stats.last_status = status;
    s_optical_task_stats.initialized = (status == HAL_OK);
    s_optical_stats_published = s_optical_task_stats;
    s_optical_task_published = g_optical_flow_diag;
}
void OpticalFlow_SetTaskHandle(TaskHandle_t handle)
{
    of_task = handle;
}

HAL_StatusTypeDef OpticalFlow_Start(void)
{
    HAL_StatusTypeDef status;
    uint32_t mode = s_requested_rx_mode;
    if (mode > OPTICAL_FLOW_RX_IT)
        mode = (g_optical_flow_diag.rx_mode == OPTICAL_FLOW_RX_IT) ? OPTICAL_FLOW_RX_IT
                                                                   : OPTICAL_FLOW_RX_DMA;
    if (!s_optical_task_stats.initialized)
        return HAL_ERROR;
    /* Called by OpticalFlowTask only. Stop callbacks before resetting the ring.
     * HAL abort is bounded; no peripheral init failure reaches Error_Handler. */
    s_optical_task_stats.receiving = 0;
    status = HAL_UART_AbortReceive(&huart10);
    taskENTER_CRITICAL();
    of_head = of_tail = of_dma_pos = of_parse_size = 0;
    of_restart = 0;
    /* Mark discontinuity in diagnostic history, without clearing evidence. */
    g_optical_flow_diag.restart_trace_count = s_optical_isr_diag.dma_trace_count;
    of_diag_seq_seen = 0;
    if (mode != g_optical_flow_diag.rx_mode)
        g_optical_flow_diag.rx_mode_switch_count++;
    g_optical_flow_diag.rx_mode = mode;
    taskEXIT_CRITICAL();
    if (status == HAL_OK)
    {
        __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF |
                                            UART_CLEAR_OREF | UART_CLEAR_IDLEF);
        __HAL_UART_SEND_REQ(&huart10, UART_RXDATA_FLUSH_REQUEST);
        if (mode == OPTICAL_FLOW_RX_DMA && (SCB->CCR & SCB_CCR_DC_Msk) != 0U)
            SCB_CleanInvalidateDCache_by_Addr((uint32_t *)of_dma, sizeof(of_dma));
        __DSB();
        /* Set before HAL enables DMA interrupts; an immediate error must persist. */
        s_optical_task_stats.receiving = 1;
        if (mode == OPTICAL_FLOW_RX_DMA)
            status = HAL_UARTEx_ReceiveToIdle_DMA(&huart10, of_dma, sizeof(of_dma));
        else
            status = HAL_UART_Receive_IT(&huart10, &of_it_byte, 1U);
    }
    s_optical_task_stats.last_status = status;
    s_optical_task_stats.restart_count++;
    if (status != HAL_OK)
    {
        s_optical_task_stats.receiving = 0;
        s_optical_task_stats.restart_failures++;
        of_restart = 1;
    }
    of_retry_tick = HAL_GetTick();
    of_retry_wait = (status != HAL_OK);
    return status;
}
void OpticalFlow_OnRxEvent(UART_HandleTypeDef *uart)
{
    uint32_t pos, tick, remaining, gap;
    if (uart != &huart10)
        return;
    tick = HAL_GetTick();
    s_optical_isr_diag.rx_event_count++;
    if (of_diag_rx_seen)
    {
        gap = (uint32_t)(tick - s_optical_isr_diag.rx_last_tick);
        if (gap > s_optical_isr_diag.rx_max_interval_ms)
            s_optical_isr_diag.rx_max_interval_ms = gap;
    }
    of_diag_rx_seen = 1;
    s_optical_isr_diag.rx_last_tick = tick;
    if (!s_optical_task_stats.receiving || of_restart ||
        g_optical_flow_diag.rx_mode != OPTICAL_FLOW_RX_DMA)
    {
        s_optical_isr_diag.rx_ignored_count++;
        return;
    }
    remaining = __HAL_DMA_GET_COUNTER(uart->hdmarx);
    pos = (OF_DMA_SIZE - remaining) % OF_DMA_SIZE;
    s_optical_isr_diag.dma_last_remaining = remaining;
    s_optical_isr_diag.dma_last_pos = pos;
    if (pos == of_dma_pos)
    {
        s_optical_isr_diag.rx_empty_count++;
        return;
    }
    /* RX buffer is read-only for CPU. Invalidate whole owned cache lines, never
     * clean live RX data back over DMA writes. HT interrupts remain enabled. */
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
        SCB_InvalidateDCache_by_Addr((uint32_t *)of_dma, sizeof(of_dma));
    __DSB();
    /* ISR capture progress counter is diagnostic only, never a seqlock. */
    s_optical_isr_diag.dma_last_chunk_begin = s_optical_isr_diag.dma_trace_count;
    while (of_dma_pos != pos)
    {
        uint32_t head = of_head;
        uint32_t trace_count, queued;
        uint8_t byte;
        if ((uint32_t)(head - of_tail) >= OF_QUEUE_SIZE)
        {
            s_optical_isr_stats.queue_overflows++;
            of_restart = 1;
            break;
        }
        /* One read supplies both the real queue and diagnostic history. This
         * is a CPU view of DMA memory, NOT an independent capture of UART RDR. */
        byte = of_dma[of_dma_pos];
        trace_count = s_optical_isr_diag.dma_trace_count;
        s_optical_isr_diag.dma_trace[trace_count % OPTICAL_FLOW_DIAG_TRACE_SIZE] = byte;
        s_optical_isr_diag.dma_trace_count = trace_count + 1U;
        of_queue[head % OF_QUEUE_SIZE] = byte;
        of_ticks[head % OF_QUEUE_SIZE] = tick;
        __DMB();
        of_head = head + 1U;
        queued = (uint32_t)(of_head - of_tail);
        if (queued > s_optical_isr_diag.queue_high_water_bytes)
            s_optical_isr_diag.queue_high_water_bytes = queued;
        of_dma_pos = (of_dma_pos + 1U) % OF_DMA_SIZE;
        s_optical_isr_stats.byte_count++;
    }
    s_optical_isr_diag.dma_last_chunk_size =
        (uint32_t)(s_optical_isr_diag.dma_trace_count - s_optical_isr_diag.dma_last_chunk_begin);
    __DMB();
    s_optical_isr_diag.dma_capture_seq += 2U;
    OF_Notify();
}
/* Diagnostic alternate transport. HAL reads RDR once into of_it_byte; no DMA
 * or D-Cache maintenance is involved. Parsing stays in OpticalFlowTask. */
void OpticalFlow_OnRxByte(UART_HandleTypeDef *uart)
{
    uint32_t tick, head, trace_count, queued;
    HAL_StatusTypeDef status;
    if (uart != &huart10 || !s_optical_task_stats.receiving || of_restart ||
        g_optical_flow_diag.rx_mode != OPTICAL_FLOW_RX_IT)
        return;
    tick = HAL_GetTick();
    s_optical_isr_diag.it_rx_byte_count++;
    head = of_head;
    if ((uint32_t)(head - of_tail) >= OF_QUEUE_SIZE)
    {
        s_optical_isr_stats.queue_overflows++;
        of_restart = 1;
        OF_Notify();
        return;
    }
    /* ISR capture progress counter is diagnostic only, never a seqlock. */
    trace_count = s_optical_isr_diag.dma_trace_count;
    s_optical_isr_diag.dma_last_chunk_begin = trace_count;
    s_optical_isr_diag.dma_trace[trace_count % OPTICAL_FLOW_DIAG_TRACE_SIZE] = of_it_byte;
    s_optical_isr_diag.dma_trace_count = trace_count + 1U;
    s_optical_isr_diag.dma_last_chunk_size = 1U;
    of_queue[head % OF_QUEUE_SIZE] = of_it_byte;
    of_ticks[head % OF_QUEUE_SIZE] = tick;
    __DMB();
    of_head = head + 1U;
    queued = (uint32_t)(of_head - of_tail);
    if (queued > s_optical_isr_diag.queue_high_water_bytes)
        s_optical_isr_diag.queue_high_water_bytes = queued;
    s_optical_isr_stats.byte_count++;
    __DMB();
    s_optical_isr_diag.dma_capture_seq += 2U;
    /* Rearm before waking the task. Failure uses the existing task recovery. */
    status = HAL_UART_Receive_IT(&huart10, &of_it_byte, 1U);
    if (status != HAL_OK)
    {
        s_optical_isr_diag.it_rearm_failures++;
        s_optical_isr_stats.last_status = status;
        of_restart = 1;
    }
    OF_Notify();
}
void OpticalFlow_OnError(UART_HandleTypeDef *uart)
{
    if (uart != &huart10)
        return;
    s_optical_isr_stats.uart_errors++;
    s_optical_isr_stats.last_uart_error = uart->ErrorCode;
    of_restart = 1;
    OF_Notify();
}
static void OF_Publish(uint32_t tick)
{
    OpticalFlowData_t d = {0};
    const uint8_t *p = &of_parse[6];
    d.head = of_parse[0];
    d.dev_id = of_parse[1];
    d.sys_id = of_parse[2];
    d.msg_id = of_parse[3];
    d.seq = of_parse[4];
    d.len = of_parse[5];
    d.time_ms = OF_U32(p);
    d.distance = OF_U32(p + 4);
    d.strength = p[8];
    d.precision = p[9];
    d.tof_status = p[10];
    d.reserved1 = p[11];
    d.flow_vel_x = (int16_t)OF_U16(p + 12);
    d.flow_vel_y = (int16_t)OF_U16(p + 14);
    d.flow_quality = p[16];
    d.flow_status = p[17];
    d.reserved2 = OF_U16(p + 18);
    d.checksum = of_parse[26];
    d.has_sample = 1;
    d.received_tick = tick;
    memcpy(d.raw, of_parse, sizeof(d.raw));
    OF_DiagFrame(&d);
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
        s_optical_task_stats.frame_count++;
        if (d.flow_vel_x != 0 || d.flow_vel_y != 0)
            g_optical_flow_diag.published_nonzero_count++;
        if (age > g_optical_flow_diag.max_publish_age_ms)
            g_optical_flow_diag.max_publish_age_ms = age;
    }
    else
        g_optical_flow_diag.publish_discard_count++;
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
        if (of_parse[0] != 0xEFU)
            goto discard;
        if (of_parse_size < 6U)
            return;
        if (of_parse[5] > 64U || (of_parse[3] == 0x51U && of_parse[5] != 20U))
        {
            s_optical_task_stats.length_errors++;
            goto discard;
        }
        size = (uint32_t)of_parse[5] + 7U;
        if (of_parse_size < size)
            return;
        sum = 0;
        for (i = 0; i < size - 1U; ++i)
            sum = (uint8_t)(sum + of_parse[i]);
        if (sum != of_parse[size - 1U])
        {
            s_optical_task_stats.checksum_errors++;
            goto discard;
        }
        if (of_parse[3] == 0x51U)
            OF_Publish(of_parse_ticks[size - 1U]);
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
    if (data != NULL)
    {
        *data = g_optical_flow_data;
        data->age_ms = age >= UINT32_MAX ? UINT32_MAX : (uint32_t)age;
    }
    taskEXIT_CRITICAL();
}
static void OpticalFlow_ProcessPendingInternal(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t frames_before = s_optical_task_stats.frame_count;
    if (of_diag_task_seen)
    {
        uint32_t gap = (uint32_t)(now - g_optical_flow_diag.task_last_tick);
        if (gap > g_optical_flow_diag.task_max_interval_ms)
            g_optical_flow_diag.task_max_interval_ms = gap;
    }
    of_diag_task_seen = 1;
    g_optical_flow_diag.task_last_tick = now;
    s_optical_task_stats.task_count++;
    OpticalFlow_GetData(NULL);
    if (!s_optical_task_stats.initialized)
        return;
    if (s_requested_rx_mode <= OPTICAL_FLOW_RX_IT &&
        s_requested_rx_mode != g_optical_flow_diag.rx_mode)
    {
        /* Watch changes are applied only in this task. Partial old bytes are
         * discarded by Start, exactly as in ordinary reception recovery. */
        of_retry_wait = 0;
        of_restart = 1;
    }
    if (of_restart || !s_optical_task_stats.receiving)
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
    {
        uint32_t frames = (uint32_t)(s_optical_task_stats.frame_count - frames_before);
        if (frames > g_optical_flow_diag.task_max_batch_frames)
            g_optical_flow_diag.task_max_batch_frames = frames;
    }
}

void OpticalFlow_GetStats(OpticalFlowStats_t *stats)
{
    if (!stats) return;
    /* OF-C1: task/ISR counters plus registered compatibility diagnostics.
     * Budget 3 us, NOT measured. No writable pointer escapes this module. */
    taskENTER_CRITICAL();
    *stats = s_optical_stats_published;
    stats->heap_free_bytes = g_optical_flow_stats.heap_free_bytes;
    stats->heap_min_free_bytes = g_optical_flow_stats.heap_min_free_bytes;
    stats->optical_stack_free_words = g_optical_flow_stats.optical_stack_free_words;
    stats->default_stack_free_words = g_optical_flow_stats.default_stack_free_words;
    stats->flight_stack_free_words = g_optical_flow_stats.flight_stack_free_words;
    stats->byte_count = s_optical_isr_stats.byte_count;
    stats->queue_overflows = s_optical_isr_stats.queue_overflows;
    stats->uart_errors = s_optical_isr_stats.uart_errors;
    stats->last_uart_error = s_optical_isr_stats.last_uart_error;
    taskEXIT_CRITICAL();
}

void OpticalFlow_GetDiagnostics(OpticalFlowDiag_t *diag)
{
    unsigned i;
    if (!diag) return;
    /* OF-C2: bounded full trace and task diagnostic snapshot; budget 10 us,
     * NOT measured. Only task callers, never a retrying diagnostic seqlock. */
    taskENTER_CRITICAL();
    *diag = s_optical_task_published;
    diag->requested_rx_mode = s_requested_rx_mode;
    diag->rx_event_count = s_optical_isr_diag.rx_event_count;
    diag->rx_ignored_count = s_optical_isr_diag.rx_ignored_count;
    diag->rx_empty_count = s_optical_isr_diag.rx_empty_count;
    diag->rx_last_tick = s_optical_isr_diag.rx_last_tick;
    diag->rx_max_interval_ms = s_optical_isr_diag.rx_max_interval_ms;
    diag->dma_last_remaining = s_optical_isr_diag.dma_last_remaining;
    diag->dma_last_pos = s_optical_isr_diag.dma_last_pos;
    diag->dma_capture_seq = s_optical_isr_diag.dma_capture_seq;
    diag->dma_trace_count = s_optical_isr_diag.dma_trace_count;
    diag->dma_last_chunk_begin = s_optical_isr_diag.dma_last_chunk_begin;
    diag->dma_last_chunk_size = s_optical_isr_diag.dma_last_chunk_size;
    diag->queue_high_water_bytes = s_optical_isr_diag.queue_high_water_bytes;
    diag->it_rx_byte_count = s_optical_isr_diag.it_rx_byte_count;
    diag->it_rearm_failures = s_optical_isr_diag.it_rearm_failures;
    for (i = 0; i < OPTICAL_FLOW_DIAG_TRACE_SIZE; i++) diag->dma_trace[i] = s_optical_isr_diag.dma_trace[i];
    taskEXIT_CRITICAL();
}

void OpticalFlow_ProcessPending(void)
{
    OpticalFlow_ProcessPendingInternal();
    /* OF-C3: task-side diagnostics publication only; budget 10 us, NOT measured.
     * UART recovery/parsing remain outside. ISR trace has separate storage. */
    taskENTER_CRITICAL();
    s_optical_stats_published = s_optical_task_stats;
    s_optical_task_published = g_optical_flow_diag;
    taskEXIT_CRITICAL();
}
void OpticalFlow_RequestRxMode(uint32_t mode)
{
    s_requested_rx_mode = mode;
}
