#include "rc_sbus.h"

#include <string.h>

#include "usart.h"

#define RC_SBUS_FRAME_SIZE 25U
#define RC_SBUS_FRAME_HEADER 0x0FU
#define RC_SBUS_FRAME_END 0x00U
#define RC_SBUS_FLAG_CH17 0x01U
#define RC_SBUS_FLAG_CH18 0x02U
#define RC_SBUS_FLAG_FRAME_LOST 0x04U
#define RC_SBUS_FLAG_FAILSAFE 0x08U
#define RC_SBUS_RAW_MIN 172U
#define RC_SBUS_RAW_CENTER 992U
#define RC_SBUS_RAW_MAX 1811U
#define RC_SBUS_DEADBAND 0.03f
#define RC_SBUS_MAX_TILT_RAD (30.0f * 3.14159265358979323846f / 180.0f)
#define RC_SBUS_MAX_YAW_RATE_RAD_S (180.0f * 3.14159265358979323846f / 180.0f)
#define RC_SBUS_TIMEOUT_MS 100U
#define RC_SBUS_DMA_BUFFER_SIZE 64U
#define RC_SBUS_RX_RING_SIZE 128U

volatile RcSbusData_t g_rc_sbus_data;
volatile RcSbusDebug_t g_rc_sbus_debug;
volatile RcSbusDiagnostic_t g_rc_sbus_diag;
#if RC_SBUS_DIAGNOSTIC
static void RcSbus_DiagnosticPoll(void);
#endif
/* Explicit linker placement in DMA1-accessible SRAM; never DTCM. */
__attribute__((section(".sbus_dma"), aligned(32)))
static uint8_t s_uart5_dma_buffer[RC_SBUS_DMA_BUFFER_SIZE];
static uint8_t s_rx_ring[RC_SBUS_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint32_t s_rx_overrun_count;
static TaskHandle_t s_rc_task_handle;
static uint8_t s_frame[RC_SBUS_FRAME_SIZE];
static uint8_t s_frame_index;
static volatile uint8_t s_restart_pending;

static void RcSbus_NotifyFromISR(void)
{
    BaseType_t woken = pdFALSE;
    if (s_rc_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(s_rc_task_handle, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

static uint16_t RcSbus_ClampRaw(uint16_t value)
{
    if (value < RC_SBUS_RAW_MIN) return RC_SBUS_RAW_MIN;
    if (value > RC_SBUS_RAW_MAX) return RC_SBUS_RAW_MAX;
    return value;
}

static float RcSbus_MapThrottle(uint16_t value)
{
    const uint16_t clamped = RcSbus_ClampRaw(value);
    return (float)(clamped - RC_SBUS_RAW_MIN) / (float)(RC_SBUS_RAW_MAX - RC_SBUS_RAW_MIN);
}

static float RcSbus_MapCentered(uint16_t value)
{
    float normalized;
    float magnitude;
    value = RcSbus_ClampRaw(value);
    if (value >= RC_SBUS_RAW_CENTER)
        normalized = (float)(value - RC_SBUS_RAW_CENTER) / (float)(RC_SBUS_RAW_MAX - RC_SBUS_RAW_CENTER);
    else
        normalized = (float)((int32_t)value - (int32_t)RC_SBUS_RAW_CENTER) / (float)(RC_SBUS_RAW_CENTER - RC_SBUS_RAW_MIN);
    magnitude = (normalized < 0.0f) ? -normalized : normalized;
    if (magnitude <= RC_SBUS_DEADBAND) return 0.0f;
    magnitude = (magnitude - RC_SBUS_DEADBAND) / (1.0f - RC_SBUS_DEADBAND);
    return (normalized < 0.0f) ? -magnitude : magnitude;
}

static uint16_t RcSbus_UnpackChannel(const uint8_t frame[RC_SBUS_FRAME_SIZE], uint8_t channel_index)
{
    const uint32_t bit_offset = (uint32_t)channel_index * 11U;
    const uint32_t byte_offset = 1U + (bit_offset >> 3U);
    const uint32_t packed = (uint32_t)frame[byte_offset] |
                            ((uint32_t)frame[byte_offset + 1U] << 8U) |
                            ((uint32_t)frame[byte_offset + 2U] << 16U);
    return (uint16_t)((packed >> (bit_offset & 0x07U)) & 0x07FFU);
}

static void RcSbus_PublishSafe(void)
{
    g_rc_sbus_data.throttle = 0.0f;
    g_rc_sbus_data.roll_target_rad = 0.0f;
    g_rc_sbus_data.pitch_target_rad = 0.0f;
    g_rc_sbus_data.yaw_rate_target_rad_s = 0.0f;
    g_rc_sbus_data.valid = 0U;
}

static void RcSbus_ProcessFrame(const uint8_t frame[RC_SBUS_FRAME_SIZE])
{
    uint8_t index;
    const uint8_t flags = frame[23];
#if RC_SBUS_DIAGNOSTIC
    if (g_rc_sbus_diag.phase < 2U)
        for (index = 0U; index < RC_SBUS_FRAME_SIZE; index++)
            g_rc_sbus_diag.capture[g_rc_sbus_diag.phase].last_frame[index] = frame[index];
#endif
    g_rc_sbus_data.frame_count++;
    for (index = 0U; index < RC_SBUS_CHANNEL_COUNT; index++)
        g_rc_sbus_data.channel[index] = RcSbus_UnpackChannel(frame, index);
    g_rc_sbus_data.ch17 = ((flags & RC_SBUS_FLAG_CH17) != 0U) ? 1U : 0U;
    g_rc_sbus_data.ch18 = ((flags & RC_SBUS_FLAG_CH18) != 0U) ? 1U : 0U;
    g_rc_sbus_data.frame_lost = ((flags & RC_SBUS_FLAG_FRAME_LOST) != 0U) ? 1U : 0U;
    g_rc_sbus_data.failsafe = ((flags & RC_SBUS_FLAG_FAILSAFE) != 0U) ? 1U : 0U;
    if (g_rc_sbus_data.frame_lost != 0U) g_rc_sbus_data.frame_lost_count++;
    if (g_rc_sbus_data.failsafe != 0U) g_rc_sbus_data.failsafe_count++;
    if ((g_rc_sbus_data.frame_lost != 0U) || (g_rc_sbus_data.failsafe != 0U))
    {
        RcSbus_PublishSafe();
        return;
    }
#if RC_SBUS_DIAGNOSTIC
    /* Capture/parse only: never publish usable control targets in this build. */
    g_rc_sbus_data.valid_frame_count++;
    RcSbus_PublishSafe();
    return;
#endif
    /* AETR: CH1=roll, CH2=pitch, CH3=throttle, CH4=yaw. */
    g_rc_sbus_data.throttle = RcSbus_MapThrottle(g_rc_sbus_data.channel[2]);
    g_rc_sbus_data.roll_target_rad = RcSbus_MapCentered(g_rc_sbus_data.channel[0]) * RC_SBUS_MAX_TILT_RAD;
    g_rc_sbus_data.pitch_target_rad = RcSbus_MapCentered(g_rc_sbus_data.channel[1]) * RC_SBUS_MAX_TILT_RAD;
    g_rc_sbus_data.yaw_rate_target_rad_s = RcSbus_MapCentered(g_rc_sbus_data.channel[3]) * RC_SBUS_MAX_YAW_RATE_RAD_S;
    g_rc_sbus_data.last_valid_tick = HAL_GetTick();
    g_rc_sbus_data.valid_frame_count++;
    g_rc_sbus_data.valid = 1U;
}

static void RcSbus_ConsumeByte(uint8_t byte)
{
    if (s_frame_index == 0U)
    {
        if (byte == RC_SBUS_FRAME_HEADER)
        {
            s_frame[0] = byte;
            s_frame_index = 1U;
        }
        return;
    }
    s_frame[s_frame_index++] = byte;
    if (s_frame_index < RC_SBUS_FRAME_SIZE) return;
    /* R7008SB: original SBUS, telemetry slot footer, or FASSTest 12CH footer. */
    if ((byte == RC_SBUS_FRAME_END) || (byte == 0x08U) ||
        ((byte & 0xCFU) == 0x04U))
    {
        RcSbus_ProcessFrame(s_frame);
        s_frame_index = 0U;
    }
    else
    {
        g_rc_sbus_data.frame_error_count++;
        s_frame_index = (byte == RC_SBUS_FRAME_HEADER) ? 1U : 0U;
        if (s_frame_index != 0U) s_frame[0] = byte;
    }
}

static HAL_StatusTypeDef RcSbus_ArmUart5Reception(void)
{
    HAL_StatusTypeDef status;
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)s_uart5_dma_buffer, RC_SBUS_DMA_BUFFER_SIZE);
    g_rc_sbus_debug.start_count++;
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_uart5_dma_buffer, RC_SBUS_DMA_BUFFER_SIZE);
    g_rc_sbus_debug.last_start_status = (uint32_t)status;
    if (status == HAL_OK)
        __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    else
    {
        g_rc_sbus_debug.start_fail_count++;
        s_restart_pending = 1U;
    }
    return status;
}

void RcSbus_Init(void)
{
    (void)memset((void *)&g_rc_sbus_data, 0, sizeof(g_rc_sbus_data));
    (void)memset((void *)&g_rc_sbus_debug, 0, sizeof(g_rc_sbus_debug));
    g_rc_sbus_debug.version = RC_SBUS_DIAGNOSTIC ? 0x00030001U : 0x00020002U;
    (void)memset((void *)&g_rc_sbus_diag, 0, sizeof(g_rc_sbus_diag));
    g_rc_sbus_diag.phase = 3U;
    g_rc_sbus_debug.dma_address = (uint32_t)s_uart5_dma_buffer;
    (void)memset(s_rx_ring, 0, sizeof(s_rx_ring));
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_overrun_count = 0U;
    s_rc_task_handle = NULL;
    s_frame_index = 0U;
    s_restart_pending = 0U;
}

HAL_StatusTypeDef RcSbus_StartReception(void)
{
    /* main calls before the scheduler: defer actual RX to RcTask.
     * A live receiver can cause a transient error during ReceiveToIdle start;
     * that must not put the whole firmware into Error_Handler. */
    s_restart_pending = 1U;
    return HAL_OK;
}
void RcSbus_SetTaskHandle(TaskHandle_t task_handle) { s_rc_task_handle = task_handle; }

void RcSbus_ProcessPending(void)
{
    uint16_t tail;
    g_rc_sbus_debug.task_count++;
#if RC_SBUS_DIAGNOSTIC
    RcSbus_DiagnosticPoll();
    return;
#endif
    if (s_restart_pending != 0U)
    {
        /* Blocking HAL abort is TASK ONLY. Never restart/abort recursively in ISR. */
        RcSbus_PublishSafe();
        if (HAL_UART_AbortReceive(&huart5) != HAL_OK) return;
        __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_PEF | UART_CLEAR_FEF |
                            UART_CLEAR_NEF | UART_CLEAR_OREF | UART_CLEAR_IDLEF);
        __HAL_UART_SEND_REQ(&huart5, UART_RXDATA_FLUSH_REQUEST);
        s_rx_tail = s_rx_head;
        s_frame_index = 0U;
        s_restart_pending = 0U;
        g_rc_sbus_debug.restart_count++;
        (void)RcSbus_ArmUart5Reception();
        return; /* retry on next task timeout if start failed */
    }
    tail = s_rx_tail;
    g_rc_sbus_data.rx_overrun_count = s_rx_overrun_count;
    while (tail != s_rx_head)
    {
        if (s_restart_pending != 0U)
        {
            RcSbus_PublishSafe();
            break;
        }
        RcSbus_ConsumeByte(s_rx_ring[tail]);
        tail = (tail + 1U) % RC_SBUS_RX_RING_SIZE;
        s_rx_tail = tail;
    }
}

void RcSbus_CheckTimeout(void)
{
    if ((g_rc_sbus_data.valid != 0U) &&
        ((HAL_GetTick() - g_rc_sbus_data.last_valid_tick) >= RC_SBUS_TIMEOUT_MS))
        RcSbus_PublishSafe();
}

void RcSbus_GetData(RcSbusData_t *data)
{
    if (data == NULL) return;
    taskENTER_CRITICAL();
    (void)memcpy(data, (const void *)&g_rc_sbus_data, sizeof(*data));
    taskEXIT_CRITICAL();
}

void RcSbus_RxBytesFromISR(const uint8_t *data, uint16_t length)
{
    uint16_t index;
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (data == NULL) return;
    for (index = 0U; index < length; index++)
    {
        uint16_t next_head = s_rx_head + 1U;
        if (next_head >= RC_SBUS_RX_RING_SIZE) next_head = 0U;
        if (next_head == s_rx_tail)
        {
            s_rx_overrun_count++;
            continue;
        }
        s_rx_ring[s_rx_head] = data[index];
        __DMB();
        s_rx_head = next_head;
    }
    if (s_rc_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(s_rc_task_handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t i;
    if (huart != &huart5) return;
    if (s_restart_pending != 0U) return;
    g_rc_sbus_debug.rx_event_count++;
    if (size > RC_SBUS_DMA_BUFFER_SIZE) size = RC_SBUS_DMA_BUFFER_SIZE;
    g_rc_sbus_debug.last_rx_size = size;
    g_rc_sbus_debug.rx_byte_count += size;
    if (size != 0U)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)s_uart5_dma_buffer, RC_SBUS_DMA_BUFFER_SIZE);
        for (i = 0U; (i < size) && (i < sizeof(g_rc_sbus_debug.last_rx)); i++)
            g_rc_sbus_debug.last_rx[i] = s_uart5_dma_buffer[i];
        RcSbus_RxBytesFromISR(s_uart5_dma_buffer, size);
    }
    (void)RcSbus_ArmUart5Reception();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart5) return;
    /* HAL has aborted DMA by this point. Preserve the error before recovery. */
    g_rc_sbus_debug.uart_error_count++;
    g_rc_sbus_debug.last_uart_error = huart->ErrorCode;
    g_rc_sbus_debug.last_dma_error = huart->hdmarx->ErrorCode;
    s_restart_pending = 1U;
    RcSbus_NotifyFromISR();
}

/* Temporary raw interrupt receiver. No HAL RX/DMA restart, parsing or UART TX in ISR. */
void RcSbus_DiagnosticIRQ(void)
{
    uint32_t status = UART5->ISR;
    uint32_t errors = status & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE);
    uint32_t phase = g_rc_sbus_diag.phase;
    if ((status & USART_ISR_RXNE_RXFNE) != 0U)
    {
        uint8_t byte = (uint8_t)UART5->RDR;
        if (phase < 2U)
        {
            volatile RcSbusCapture_t *capture = &g_rc_sbus_diag.capture[phase];
            uint32_t index = capture->byte_count % RC_SBUS_CAPTURE_SIZE;
            capture->bytes[index] = byte;
            capture->errors[index] = (uint8_t)errors;
            if ((errors == 0U) && (byte == RC_SBUS_FRAME_HEADER)) capture->clean_header_count++;
            __DMB();
            capture->byte_count++;
            g_rc_sbus_debug.rx_byte_count++;
        }
    }
    if (errors != 0U)
    {
        UART5->ICR = UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF;
        if (phase < 2U) g_rc_sbus_diag.capture[phase].error_count++;
    }
}

#if RC_SBUS_DIAGNOSTIC
static void RcSbus_DiagnosticPoll(void)
{
    static uint32_t started;
    static uint32_t consumed;
    uint32_t phase = g_rc_sbus_diag.phase;
    uint32_t now = HAL_GetTick();
    RcSbus_PublishSafe();
    if ((phase == 3U) || ((phase < 2U) && ((now - started) >= 5000U)))
    {
        /* Stop the receiver before changing polarity. Drop startup residual data/errors. */
        HAL_NVIC_DisableIRQ(UART5_IRQn);
        (void)HAL_UART_AbortReceive(&huart5);
        CLEAR_BIT(UART5->CR1, USART_CR1_UE);
        phase = (phase == 3U) ? 0U : phase + 1U;
        g_rc_sbus_diag.phase = phase;
        s_frame_index = 0U;
        consumed = 0U;
        if (phase == 2U) return; /* Freeze both captures until reset. */
        MODIFY_REG(UART5->CR2, USART_CR2_RXINV, (phase == 0U) ? USART_CR2_RXINV : 0U);
        CLEAR_BIT(UART5->CR3, USART_CR3_DMAR);
        UART5->ICR = UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF | UART_CLEAR_IDLEF;
        UART5->RQR = UART_RXDATA_FLUSH_REQUEST;
        SET_BIT(UART5->CR1, USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE);
        SET_BIT(UART5->CR3, USART_CR3_EIE);
        g_rc_sbus_diag.kernel_clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_UART5);
        g_rc_sbus_diag.capture[phase].cr1 = UART5->CR1;
        g_rc_sbus_diag.capture[phase].cr2 = UART5->CR2;
        g_rc_sbus_diag.capture[phase].brr = UART5->BRR;
        started = now;
        HAL_NVIC_ClearPendingIRQ(UART5_IRQn);
        HAL_NVIC_EnableIRQ(UART5_IRQn);
    }
    if (phase < 2U)
    {
        volatile RcSbusCapture_t *capture = &g_rc_sbus_diag.capture[phase];
        uint32_t budget = RC_SBUS_CAPTURE_SIZE;
        while (budget-- != 0U)
        {
            uint8_t byte, errors;
            uint32_t frames;
            taskENTER_CRITICAL();
            if ((capture->byte_count - consumed) > RC_SBUS_CAPTURE_SIZE)
            {
                capture->overrun_count++;
                consumed = capture->byte_count - RC_SBUS_CAPTURE_SIZE;
                s_frame_index = 0U;
            }
            if (consumed == capture->byte_count)
            {
                taskEXIT_CRITICAL();
                break;
            }
            byte = capture->bytes[consumed % RC_SBUS_CAPTURE_SIZE];
            errors = capture->errors[consumed % RC_SBUS_CAPTURE_SIZE];
            consumed++;
            taskEXIT_CRITICAL();
            if (errors != 0U) { s_frame_index = 0U; continue; }
            frames = g_rc_sbus_data.frame_count;
            RcSbus_ConsumeByte(byte);
            capture->frame_count += g_rc_sbus_data.frame_count - frames;
        }
    }
}
#endif
