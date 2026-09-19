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
/* Measured transmitter endpoints and released-stick centers (raw SBUS units). */
#define RC_SBUS_ROLL_MIN 0x014EU
#define RC_SBUS_ROLL_CENTER 0x03F0U
#define RC_SBUS_ROLL_MAX 0x068BU
#define RC_SBUS_PITCH_MIN 0x0138U
#define RC_SBUS_PITCH_CENTER 0x03E8U
#define RC_SBUS_PITCH_MAX 0x0677U
#define RC_SBUS_YAW_MIN 0x0140U
#define RC_SBUS_YAW_CENTER 0x03E0U
#define RC_SBUS_YAW_MAX 0x067FU
#define RC_SBUS_THROTTLE_MIN 348U
#define RC_SBUS_THROTTLE_MAX 1696U
#define RC_SBUS_ROLL_TARGET_SIGN (1.0f)
#define RC_SBUS_PITCH_TARGET_SIGN (-1.0f)
#define RC_SBUS_YAW_RATE_TARGET_SIGN (1.0f)
#define RC_SBUS_DEADBAND 0.03f
#define RC_SBUS_MAX_TILT_RAD (30.0f * 3.14159265358979323846f / 180.0f)
#define RC_SBUS_MAX_YAW_RATE_RAD_S (180.0f * 3.14159265358979323846f / 180.0f)
#define RC_SBUS_TIMEOUT_MS 100U
#define RC_SBUS_DMA_BUFFER_SIZE 64U
#define RC_SBUS_RX_RING_SIZE 128U

volatile RcSbusData_t g_rc_sbus_data;
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

static float RcSbus_MapThrottle(uint16_t value)
{
    if (value <= RC_SBUS_THROTTLE_MIN) return 0.0f;
    if (value >= RC_SBUS_THROTTLE_MAX) return 1.0f;
    return (float)(value - RC_SBUS_THROTTLE_MIN) /
           (float)(RC_SBUS_THROTTLE_MAX - RC_SBUS_THROTTLE_MIN);
}

static float RcSbus_MapCentered(uint16_t value, uint16_t minimum,
                               uint16_t center, uint16_t maximum)
{
    float normalized;
    /* Convert before subtraction so the centered difference is signed.
     * Normalize each side independently, preserving asymmetric travel. */
    if (value < minimum) value = minimum;
    if (value > maximum) value = maximum;
    if (value >= center)
        normalized = ((float)value - (float)center) / ((float)maximum - (float)center);
    else
        normalized = ((float)value - (float)center) / ((float)center - (float)minimum);
    if (normalized > RC_SBUS_DEADBAND)
        return (normalized - RC_SBUS_DEADBAND) / (1.0f - RC_SBUS_DEADBAND);
    if (normalized < -RC_SBUS_DEADBAND)
        return (normalized + RC_SBUS_DEADBAND) / (1.0f - RC_SBUS_DEADBAND);
    return 0.0f;
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
    g_rc_sbus_data.frame_count++;
    for (index = 0U; index < RC_SBUS_CHANNEL_COUNT; index++)
        g_rc_sbus_data.channel[index] = RcSbus_UnpackChannel(frame, index);
    g_rc_sbus_data.ch17 = ((flags & RC_SBUS_FLAG_CH17) != 0U) ? 1U : 0U;
    g_rc_sbus_data.ch18 = ((flags & RC_SBUS_FLAG_CH18) != 0U) ? 1U : 0U;
    g_rc_sbus_data.frame_lost = ((flags & RC_SBUS_FLAG_FRAME_LOST) != 0U) ? 1U : 0U;
    g_rc_sbus_data.failsafe = ((flags & RC_SBUS_FLAG_FAILSAFE) != 0U) ? 1U : 0U;
    if (g_rc_sbus_data.frame_lost != 0U) g_rc_sbus_data.frame_lost_count++;
    if (g_rc_sbus_data.failsafe != 0U) g_rc_sbus_data.failsafe_count++;
    /* Receiver failsafe takes precedence, including when both flags are set. */
    if (g_rc_sbus_data.failsafe != 0U)
    {
        RcSbus_PublishSafe();
        return;
    }
    if (g_rc_sbus_data.frame_lost != 0U)
    {
        /* Keep the last GOOD targets only within the existing 100 ms window.
         * Never refresh last_valid_tick or valid_frame_count on a lost frame.
         * Raw channels above remain available for diagnostics, not control. */
        RcSbus_CheckTimeout();
        return;
    }
    /* AETR: CH1=roll, CH2=pitch, CH3=throttle, CH4=yaw.
     * Measured right-roll/right-yaw increase raw values; forward-pitch decreases.
     * Targets are radians (roll/pitch) and radians/second (yaw rate). */
    g_rc_sbus_data.throttle = RcSbus_MapThrottle(g_rc_sbus_data.channel[2]);
    g_rc_sbus_data.roll_target_rad = RC_SBUS_ROLL_TARGET_SIGN *
        RcSbus_MapCentered(g_rc_sbus_data.channel[0], RC_SBUS_ROLL_MIN,
                          RC_SBUS_ROLL_CENTER, RC_SBUS_ROLL_MAX) * RC_SBUS_MAX_TILT_RAD;
    g_rc_sbus_data.pitch_target_rad = RC_SBUS_PITCH_TARGET_SIGN *
        RcSbus_MapCentered(g_rc_sbus_data.channel[1], RC_SBUS_PITCH_MIN,
                          RC_SBUS_PITCH_CENTER, RC_SBUS_PITCH_MAX) * RC_SBUS_MAX_TILT_RAD;
    g_rc_sbus_data.yaw_rate_target_rad_s = RC_SBUS_YAW_RATE_TARGET_SIGN *
        RcSbus_MapCentered(g_rc_sbus_data.channel[3], RC_SBUS_YAW_MIN,
                          RC_SBUS_YAW_CENTER, RC_SBUS_YAW_MAX) * RC_SBUS_MAX_YAW_RATE_RAD_S;
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
    /* After power-on, disabled D-Cache has not necessarily had its ECC
     * initialized. Never clean it in that state: maintenance can BusFault.
     * If enabled later, it must be initialized with SCB_EnableDCache first.
     * The buffer owns two complete, aligned cache lines. */
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)s_uart5_dma_buffer,
                                        RC_SBUS_DMA_BUFFER_SIZE);
    }
    __DSB(); /* Complete CPU memory accesses before handing the buffer to DMA. */
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_uart5_dma_buffer, RC_SBUS_DMA_BUFFER_SIZE);
    if (status == HAL_OK)
        __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    else
    {
        s_restart_pending = 1U;
    }
    return status;
}

void RcSbus_Init(void)
{
    (void)memset((void *)&g_rc_sbus_data, 0, sizeof(g_rc_sbus_data));
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
    if (huart != &huart5) return;
    if (s_restart_pending != 0U) return;
    if (size > RC_SBUS_DMA_BUFFER_SIZE) size = RC_SBUS_DMA_BUFFER_SIZE;
    if (size != 0U)
    {
        if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
        {
            SCB_InvalidateDCache_by_Addr((uint32_t *)s_uart5_dma_buffer,
                                       RC_SBUS_DMA_BUFFER_SIZE);
        }
        __DSB(); /* Complete maintenance before copying received bytes. */
        RcSbus_RxBytesFromISR(s_uart5_dma_buffer, size);
    }
    (void)RcSbus_ArmUart5Reception();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart5) return;
    /* HAL has aborted DMA. Request recovery in task context. */
    s_restart_pending = 1U;
    RcSbus_NotifyFromISR();
}
