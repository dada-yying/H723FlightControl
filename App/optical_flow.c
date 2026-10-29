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
static uint8_t of_it_byte;
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
volatile OpticalFlowDiag_t g_optical_flow_diag;
static uint8_t of_diag_rx_seen, of_diag_task_seen, of_diag_seq_seen;

typedef struct { float x1, x2, y1, y2; } OF_FilterHistory;
static OF_FilterHistory of_history[4];
/* Work state belongs only to OpticalFlowTask. The published copy and clocks
 * below are committed together with the raw frame in a short critical section. */
static OpticalFlowEstimate_t of_filter_work, of_estimate;
static uint64_t of_filter_ms, of_range_ms, of_estimate_ms, of_accepted_range_ms;
static uint32_t of_previous_sensor_ms;
static uint8_t of_previous_seq, of_previous_dev, of_previous_sys;
static uint8_t of_sensor_seen, of_range_seen, of_history_ready;

static void OF_FilterReset(void)
{
    memset(&of_filter_work, 0, sizeof(of_filter_work));
    memset(of_history, 0, sizeof(of_history));
    of_filter_work.age_ms = of_filter_work.range_age_ms = UINT32_MAX;
    of_filter_work.flags = OF_FILTER_NO_SAMPLE;
    of_filter_ms = of_range_ms = 0;
    of_sensor_seen = of_range_seen = of_history_ready = 0;
}

/* Same bilinear Butterworth recurrence as the existing BMI low-pass. These
 * fixed coefficients are for 100 Hz / 5 Hz only; gaps restart the histories. */
static float OF_LowPass(OF_FilterHistory *h, float input, uint8_t initialize)
{
    float output;
    if (initialize) {
        h->x1 = h->x2 = h->y1 = h->y2 = input;
        return input;
    }
    output = 0.0200833656f * input + 0.0401667311f * h->x1 +
             0.0200833656f * h->x2 + 1.5610180758f * h->y1 -
             0.6413515381f * h->y2;
    h->x2 = h->x1; h->x1 = input;
    h->y2 = h->y1; h->y1 = output;
    return output;
}

static void OF_FilterFrame(const OpticalFlowData_t *d, uint64_t sample_ms)
{
    OpticalFlowEstimate_t *e = &of_filter_work;
    uint32_t dt = 0;
    float range = (float)d->distance * 0.001f;
    uint8_t discontinuity = 0;
    e->rx_count = d->rx_count;
    e->dt_ms = e->flags = 0;
    e->valid = 0;
    if (of_sensor_seen) {
        dt = (uint32_t)(d->time_ms - of_previous_sensor_ms);
        e->dt_ms = (uint16_t)(dt > UINT16_MAX ? UINT16_MAX : dt);
        if (d->dev_id == of_previous_dev && d->sys_id == of_previous_sys &&
            d->time_ms == of_previous_sensor_ms && d->seq == of_previous_seq) {
            e->flags = OF_FILTER_DUPLICATE;
            of_history_ready = 0;
            e->accepted_streak = 0;
            return;
        }
        discontinuity = (uint8_t)(dt != OPTICAL_FLOW_FILTER_SAMPLE_MS ||
            (uint8_t)(d->seq - of_previous_seq) != 1U ||
            d->dev_id != of_previous_dev || d->sys_id != of_previous_sys);
    }
    of_previous_sensor_ms = d->time_ms; of_previous_seq = d->seq;
    of_previous_dev = d->dev_id; of_previous_sys = d->sys_id;
    of_sensor_seen = 1;
    if (discontinuity || (of_history_ready &&
        sample_ms - of_filter_ms >= OPTICAL_FLOW_FILTER_RESET_MS)) {
        e->flags |= OF_FILTER_TIMING;
        of_history_ready = 0;
        e->accepted_streak = 0;
    }
    /* A queued old frame remains raw evidence, never a new usable estimate. */
    if (d->age_ms > OPTICAL_FLOW_FILTER_FRESH_MS) e->flags |= OF_FILTER_STALE;
    e->range_innovation_m = of_range_seen ? range - e->range_accepted_m : 0.0f;
    if (d->tof_status != 1U) e->flags |= OF_FILTER_TOF_STATUS;
    if (d->distance < OPTICAL_FLOW_RANGE_MIN_MM || d->distance > OPTICAL_FLOW_RANGE_MAX_MM)
        e->flags |= OF_FILTER_RANGE_BOUNDS;
    if (of_range_seen) {
        float innovation = e->range_innovation_m;
        float elapsed_s = (float)(sample_ms - of_range_ms) * 0.001f;
        float limit = OPTICAL_FLOW_RANGE_TOLERANCE_M +
                      OPTICAL_FLOW_RANGE_MAX_SLEW_M_S * elapsed_s;
        if (innovation < -limit || innovation > limit) e->flags |= OF_FILTER_RANGE_JUMP;
    }
    if (!(e->flags & (OF_FILTER_STALE | OF_FILTER_TOF_STATUS |
                     OF_FILTER_RANGE_BOUNDS | OF_FILTER_RANGE_JUMP))) {
        e->range_accepted_m = range;
        of_range_ms = sample_ms;
        of_range_seen = 1;
    }
    if (d->flow_status != 1U) e->flags |= OF_FILTER_FLOW_STATUS;
    if (e->flags & (OF_FILTER_STALE | OF_FILTER_TOF_STATUS | OF_FILTER_RANGE_BOUNDS |
                    OF_FILTER_RANGE_JUMP | OF_FILTER_FLOW_STATUS)) {
        /* Retain finite outputs for diagnosis, without feeding rejected data
         * (or artificial zeros) through the histories. Recovery seeds afresh. */
        of_history_ready = 0;
        e->accepted_streak = 0;
        return;
    }
    e->velocity_x_m_s = 0.01f * (float)d->flow_vel_x * e->range_accepted_m;
    e->velocity_y_m_s = 0.01f * (float)d->flow_vel_y * e->range_accepted_m;
    e->flow_x_filtered = OF_LowPass(&of_history[0], (float)d->flow_vel_x, !of_history_ready);
    e->flow_y_filtered = OF_LowPass(&of_history[1], (float)d->flow_vel_y, !of_history_ready);
    e->velocity_x_filtered_m_s = OF_LowPass(&of_history[2], e->velocity_x_m_s, !of_history_ready);
    e->velocity_y_filtered_m_s = OF_LowPass(&of_history[3], e->velocity_y_m_s, !of_history_ready);
    of_history_ready = 1;
    of_filter_ms = sample_ms;
    e->update_count++;
    if (e->accepted_streak < OPTICAL_FLOW_FILTER_READY_FRAMES) e->accepted_streak++;
    e->valid = (uint8_t)(e->accepted_streak >= OPTICAL_FLOW_FILTER_READY_FRAMES);
    if (!e->valid) e->flags |= OF_FILTER_WARMUP;
}

/* Task-only observation, before publication. Raw latches survive later zero
 * frames and UART recovery; only OpticalFlow_Init clears them. */
static void OF_DiagFrame(const OpticalFlowData_t *d)
{
    uint32_t i;
    uint32_t mode = (g_optical_flow_diag.rx_mode == OPTICAL_FLOW_RX_IT) ?
                    OPTICAL_FLOW_RX_IT : OPTICAL_FLOW_RX_DMA;
    uint16_t raw_x = (uint16_t)((uint16_t)d->raw[18] | ((uint16_t)d->raw[19] << 8));
    uint16_t raw_y = (uint16_t)((uint16_t)d->raw[20] | ((uint16_t)d->raw[21] << 8));
    g_optical_flow_diag.frame_capture_seq++;
    __DMB();
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
        if (delta != 1U) g_optical_flow_diag.seq_discontinuity_count++;
        if (delta == 0U) g_optical_flow_diag.duplicate_seq_count++;
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
    g_optical_flow_diag.frame_capture_seq++;
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
    memset((void *)&g_optical_flow_diag, 0, sizeof(g_optical_flow_diag));
    g_optical_flow_diag.version = 0x00010002U;
    of_diag_rx_seen = of_diag_task_seen = of_diag_seq_seen = 0;
    of_head = of_tail = of_dma_pos = of_parse_size = 0;
    of_restart = of_retry_wait = 0;
    of_task = NULL;
    of_clock_tick = HAL_GetTick();
    of_clock_ms = of_clock_tick;
    of_sample_ms = 0;
    OF_FilterReset();
    of_estimate = of_filter_work;
    of_estimate_ms = of_accepted_range_ms = 0;
    g_optical_flow_data.age_ms = UINT32_MAX;
    g_optical_flow_stats.last_status = status;
    g_optical_flow_stats.initialized = (status == HAL_OK);
}
void OpticalFlow_SetTaskHandle(TaskHandle_t handle) { of_task = handle; }

HAL_StatusTypeDef OpticalFlow_Start(void)
{
    HAL_StatusTypeDef status;
    uint32_t mode = g_optical_flow_diag.requested_rx_mode;
    if (mode > OPTICAL_FLOW_RX_IT)
        mode = (g_optical_flow_diag.rx_mode == OPTICAL_FLOW_RX_IT) ?
               OPTICAL_FLOW_RX_IT : OPTICAL_FLOW_RX_DMA;
    if (!g_optical_flow_stats.initialized) return HAL_ERROR;
    /* Called by OpticalFlowTask only. Stop callbacks before resetting the ring.
     * HAL abort is bounded; no peripheral init failure reaches Error_Handler. */
    g_optical_flow_stats.receiving = 0;
    status = HAL_UART_AbortReceive(&huart10);
    OF_FilterReset();
    taskENTER_CRITICAL();
    of_head = of_tail = of_dma_pos = of_parse_size = 0;
    of_restart = 0;
    /* Invalidate before HAL can re-enable reception; a preempting reader must
     * not see the old estimate become valid during the restart window. */
    of_estimate.valid = of_estimate.range_valid = of_estimate.accepted_streak = 0;
    of_estimate.flags = g_optical_flow_data.has_sample ? OF_FILTER_TRANSPORT : OF_FILTER_NO_SAMPLE;
    /* Mark discontinuity in diagnostic history, without clearing evidence. */
    g_optical_flow_diag.restart_trace_count = g_optical_flow_diag.dma_trace_count;
    of_diag_seq_seen = 0;
    if (mode != g_optical_flow_diag.rx_mode)
        g_optical_flow_diag.rx_mode_switch_count++;
    g_optical_flow_diag.rx_mode = mode;
    taskEXIT_CRITICAL();
    if (status == HAL_OK)
    {
        __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_PEF | UART_CLEAR_FEF |
                             UART_CLEAR_NEF | UART_CLEAR_OREF | UART_CLEAR_IDLEF);
        __HAL_UART_SEND_REQ(&huart10, UART_RXDATA_FLUSH_REQUEST);
        if (mode == OPTICAL_FLOW_RX_DMA && (SCB->CCR & SCB_CCR_DC_Msk) != 0U)
            SCB_CleanInvalidateDCache_by_Addr((uint32_t *)of_dma, sizeof(of_dma));
        __DSB();
        /* Set before HAL enables DMA interrupts; an immediate error must persist. */
        g_optical_flow_stats.receiving = 1;
        if (mode == OPTICAL_FLOW_RX_DMA)
            status = HAL_UARTEx_ReceiveToIdle_DMA(&huart10, of_dma, sizeof(of_dma));
        else
            status = HAL_UART_Receive_IT(&huart10, &of_it_byte, 1U);
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
    uint32_t pos, tick, remaining, gap;
    if (uart != &huart10) return;
    tick = HAL_GetTick();
    g_optical_flow_diag.rx_event_count++;
    if (of_diag_rx_seen)
    {
        gap = (uint32_t)(tick - g_optical_flow_diag.rx_last_tick);
        if (gap > g_optical_flow_diag.rx_max_interval_ms)
            g_optical_flow_diag.rx_max_interval_ms = gap;
    }
    of_diag_rx_seen = 1;
    g_optical_flow_diag.rx_last_tick = tick;
    if (!g_optical_flow_stats.receiving || of_restart ||
        g_optical_flow_diag.rx_mode != OPTICAL_FLOW_RX_DMA)
    {
        g_optical_flow_diag.rx_ignored_count++;
        return;
    }
    remaining = __HAL_DMA_GET_COUNTER(uart->hdmarx);
    pos = (OF_DMA_SIZE - remaining) % OF_DMA_SIZE;
    g_optical_flow_diag.dma_last_remaining = remaining;
    g_optical_flow_diag.dma_last_pos = pos;
    if (pos == of_dma_pos)
    {
        g_optical_flow_diag.rx_empty_count++;
        return;
    }
    /* RX buffer is read-only for CPU. Invalidate whole owned cache lines, never
     * clean live RX data back over DMA writes. HT interrupts remain enabled. */
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
        SCB_InvalidateDCache_by_Addr((uint32_t *)of_dma, sizeof(of_dma));
    __DSB();
    g_optical_flow_diag.dma_capture_seq++;
    __DMB();
    g_optical_flow_diag.dma_last_chunk_begin = g_optical_flow_diag.dma_trace_count;
    while (of_dma_pos != pos)
    {
        uint32_t head = of_head;
        uint32_t trace_count, queued;
        uint8_t byte;
        if ((uint32_t)(head - of_tail) >= OF_QUEUE_SIZE)
        {
            g_optical_flow_stats.queue_overflows++;
            of_restart = 1;
            break;
        }
        /* One read supplies both the real queue and diagnostic history. This
         * is a CPU view of DMA memory, NOT an independent capture of UART RDR. */
        byte = of_dma[of_dma_pos];
        trace_count = g_optical_flow_diag.dma_trace_count;
        g_optical_flow_diag.dma_trace[trace_count % OPTICAL_FLOW_DIAG_TRACE_SIZE] = byte;
        g_optical_flow_diag.dma_trace_count = trace_count + 1U;
        of_queue[head % OF_QUEUE_SIZE] = byte;
        of_ticks[head % OF_QUEUE_SIZE] = tick;
        __DMB();
        of_head = head + 1U;
        queued = (uint32_t)(of_head - of_tail);
        if (queued > g_optical_flow_diag.queue_high_water_bytes)
            g_optical_flow_diag.queue_high_water_bytes = queued;
        of_dma_pos = (of_dma_pos + 1U) % OF_DMA_SIZE;
        g_optical_flow_stats.byte_count++;
    }
    g_optical_flow_diag.dma_last_chunk_size =
        (uint32_t)(g_optical_flow_diag.dma_trace_count - g_optical_flow_diag.dma_last_chunk_begin);
    __DMB();
    g_optical_flow_diag.dma_capture_seq++;
    OF_Notify();
}
/* Diagnostic alternate transport. HAL reads RDR once into of_it_byte; no DMA
 * or D-Cache maintenance is involved. Parsing stays in OpticalFlowTask. */
void OpticalFlow_OnRxByte(UART_HandleTypeDef *uart)
{
    uint32_t tick, head, trace_count, queued;
    HAL_StatusTypeDef status;
    if (uart != &huart10 || !g_optical_flow_stats.receiving || of_restart ||
        g_optical_flow_diag.rx_mode != OPTICAL_FLOW_RX_IT) return;
    tick = HAL_GetTick();
    g_optical_flow_diag.it_rx_byte_count++;
    head = of_head;
    if ((uint32_t)(head - of_tail) >= OF_QUEUE_SIZE)
    {
        g_optical_flow_stats.queue_overflows++;
        of_restart = 1;
        OF_Notify();
        return;
    }
    g_optical_flow_diag.dma_capture_seq++;
    __DMB();
    trace_count = g_optical_flow_diag.dma_trace_count;
    g_optical_flow_diag.dma_last_chunk_begin = trace_count;
    g_optical_flow_diag.dma_trace[trace_count % OPTICAL_FLOW_DIAG_TRACE_SIZE] = of_it_byte;
    g_optical_flow_diag.dma_trace_count = trace_count + 1U;
    g_optical_flow_diag.dma_last_chunk_size = 1U;
    of_queue[head % OF_QUEUE_SIZE] = of_it_byte;
    of_ticks[head % OF_QUEUE_SIZE] = tick;
    __DMB();
    of_head = head + 1U;
    queued = (uint32_t)(of_head - of_tail);
    if (queued > g_optical_flow_diag.queue_high_water_bytes)
        g_optical_flow_diag.queue_high_water_bytes = queued;
    g_optical_flow_stats.byte_count++;
    __DMB();
    g_optical_flow_diag.dma_capture_seq++;
    /* Rearm before waking the task. Failure uses the existing task recovery. */
    status = HAL_UART_Receive_IT(&huart10, &of_it_byte, 1U);
    if (status != HAL_OK)
    {
        g_optical_flow_diag.it_rearm_failures++;
        g_optical_flow_stats.last_status = status;
        of_restart = 1;
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
    uint64_t sample_ms;
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
    OF_DiagFrame(&d);
    taskENTER_CRITICAL();
    sample_ms = OF_Now();
    d.age_ms = (uint32_t)(of_clock_tick - tick);
    sample_ms -= d.age_ms;
    d.rx_count = g_optical_flow_data.rx_count + 1U;
    taskEXIT_CRITICAL();
    OF_FilterFrame(&d, sample_ms);
    taskENTER_CRITICAL();
    /* Never publish queued bytes across a discontinuity signalled by an ISR. */
    if (!of_restart)
    {
        of_sample_ms = sample_ms;
        g_optical_flow_data = d;
        of_estimate = of_filter_work;
        of_estimate_ms = of_filter_ms;
        of_accepted_range_ms = of_range_ms;
        g_optical_flow_stats.frame_count++;
        if (d.flow_vel_x != 0 || d.flow_vel_y != 0)
            g_optical_flow_diag.published_nonzero_count++;
        if (d.age_ms > g_optical_flow_diag.max_publish_age_ms)
            g_optical_flow_diag.max_publish_age_ms = d.age_ms;
    }
    else g_optical_flow_diag.publish_discard_count++;
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
    OpticalFlow_GetSnapshot(data, NULL);
}
void OpticalFlow_GetSnapshot(OpticalFlowData_t *data, OpticalFlowEstimate_t *estimate)
{
    uint64_t age, now;
    taskENTER_CRITICAL();
    now = OF_Now();
    age = g_optical_flow_data.has_sample ? now - of_sample_ms : UINT32_MAX;
    g_optical_flow_data.age_ms = age >= UINT32_MAX ? UINT32_MAX : (uint32_t)age;
    if (data != NULL) *data = g_optical_flow_data;
    if (estimate != NULL) {
        *estimate = of_estimate;
        age = estimate->update_count ? now - of_estimate_ms : UINT32_MAX;
        estimate->age_ms = age >= UINT32_MAX ? UINT32_MAX : (uint32_t)age;
        /* range_valid tracks the accepted range, not the latest raw distance. */
        age = estimate->range_accepted_m > 0.0f ?
              now - of_accepted_range_ms : UINT32_MAX;
        estimate->range_age_ms = age >= UINT32_MAX ? UINT32_MAX : (uint32_t)age;
        estimate->range_valid = (uint8_t)(estimate->range_accepted_m > 0.0f &&
            estimate->range_age_ms < OPTICAL_FLOW_FILTER_RESET_MS);
        if (estimate->age_ms > OPTICAL_FLOW_FILTER_FRESH_MS) {
            estimate->valid = 0;
            estimate->flags |= OF_FILTER_STALE;
        }
        if (of_restart || !g_optical_flow_stats.receiving ||
            (estimate->flags & OF_FILTER_TRANSPORT)) {
            estimate->valid = estimate->range_valid = 0;
            estimate->flags |= OF_FILTER_TRANSPORT;
        }
        if (!estimate->range_valid) estimate->valid = 0;
    }
    taskEXIT_CRITICAL();
}
void OpticalFlow_ProcessPending(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t frames_before = g_optical_flow_stats.frame_count;
    if (of_diag_task_seen)
    {
        uint32_t gap = (uint32_t)(now - g_optical_flow_diag.task_last_tick);
        if (gap > g_optical_flow_diag.task_max_interval_ms)
            g_optical_flow_diag.task_max_interval_ms = gap;
    }
    of_diag_task_seen = 1;
    g_optical_flow_diag.task_last_tick = now;
    g_optical_flow_stats.task_count++;
    OpticalFlow_GetData(NULL);
    if (!g_optical_flow_stats.initialized) return;
    if (g_optical_flow_diag.requested_rx_mode <= OPTICAL_FLOW_RX_IT &&
        g_optical_flow_diag.requested_rx_mode != g_optical_flow_diag.rx_mode)
    {
        /* Watch changes are applied only in this task. Partial old bytes are
         * discarded by Start, exactly as in ordinary reception recovery. */
        of_retry_wait = 0;
        of_restart = 1;
    }
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
    {
        uint32_t frames = (uint32_t)(g_optical_flow_stats.frame_count - frames_before);
        if (frames > g_optical_flow_diag.task_max_batch_frames)
            g_optical_flow_diag.task_max_batch_frames = frames;
    }
}
