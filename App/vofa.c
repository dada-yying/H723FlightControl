#include "vofa.h"

#include <string.h>

#include "usart.h"

#define VOFA_JUSTFLOAT_TAIL_SIZE       4U
#define VOFA_UART_TIMEOUT_MS           10U
#define VOFA_MAX_CHANNELS              23U

volatile VofaTxStats_t g_vofa_tx_stats;

/* One in-flight frame, owned until UART TC (not merely DMA TC). Never DMA
 * from the caller's stack/DTCM. Three exclusive 32-byte cache lines. */
__attribute__((section(".vofa_dma"), aligned(32)))
static uint8_t s_vofa_dma_frame[VOFA_FLIGHT_FRAME_SIZE];
enum { VOFA_TX_IDLE, VOFA_TX_BUSY, VOFA_TX_RECOVER, VOFA_TX_ABORTING };
static volatile uint8_t s_vofa_tx_state;
static uint32_t s_vofa_start_tick;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    uint32_t duration;
    if (uart != &huart1 || s_vofa_tx_state != VOFA_TX_BUSY) return;
    duration = HAL_GetTick() - s_vofa_start_tick;
    if (duration > g_vofa_tx_stats.max_duration_ms) g_vofa_tx_stats.max_duration_ms = duration;
    g_vofa_tx_stats.success_count++;
    g_vofa_tx_stats.last_status = HAL_OK;
    s_vofa_tx_state = VOFA_TX_IDLE;
    g_vofa_tx_stats.busy = 0U;
}

void VOFA_OnUartError(UART_HandleTypeDef *uart)
{
    if (uart != &huart1 || s_vofa_tx_state != VOFA_TX_BUSY) return;
    g_vofa_tx_stats.failure_count++;
    g_vofa_tx_stats.dma_error_count++;
    g_vofa_tx_stats.last_uart_error = uart->ErrorCode;
    g_vofa_tx_stats.last_status = HAL_ERROR;
    s_vofa_tx_state = VOFA_TX_RECOVER; /* Only task context aborts/reuses buffer. */
}

static HAL_StatusTypeDef VOFA_Service(void)
{
    HAL_StatusTypeDef status;
    taskENTER_CRITICAL();
    /* Completion callbacks have priority over the task-side deadline. */
    if (s_vofa_tx_state == VOFA_TX_BUSY &&
        (uint32_t)(HAL_GetTick() - s_vofa_start_tick) >= VOFA_UART_TIMEOUT_MS) {
        g_vofa_tx_stats.timeout_count++;
        g_vofa_tx_stats.failure_count++;
        g_vofa_tx_stats.last_status = HAL_TIMEOUT;
        s_vofa_tx_state = VOFA_TX_RECOVER;
    }
    if (s_vofa_tx_state != VOFA_TX_RECOVER) {
        status = s_vofa_tx_state == VOFA_TX_IDLE ? HAL_OK : HAL_BUSY;
        taskEXIT_CRITICAL();
        return status;
    }
    s_vofa_tx_state = VOFA_TX_ABORTING;
    taskEXIT_CRITICAL();
    /* HAL disables TX DMA/TC interrupts, stops the stream and clears its flags.
     * A failed abort retains ownership: never overwrite potentially live DMA. */
    status = HAL_UART_AbortTransmit(&huart1);
    taskENTER_CRITICAL();
    if (status == HAL_OK) {
        HAL_NVIC_ClearPendingIRQ(DMA1_Stream4_IRQn);
        HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
        s_vofa_tx_state = VOFA_TX_IDLE;
        g_vofa_tx_stats.busy = 0U;
        g_vofa_tx_stats.recovery_count++;
    } else {
        s_vofa_tx_state = VOFA_TX_RECOVER;
        g_vofa_tx_stats.abort_failure_count++;
        g_vofa_tx_stats.last_status = status;
    }
    taskEXIT_CRITICAL();
    return status;
}

static void VOFA_PackChannels(uint8_t *frame, const float *channels, uint32_t channel_count)
{
    static const uint8_t justfloat_tail[VOFA_JUSTFLOAT_TAIL_SIZE] =
        {0x00U, 0x00U, 0x80U, 0x7FU};
    const uint32_t channel_bytes = channel_count * sizeof(float);
    (void)memcpy(frame, channels, channel_bytes);
    (void)memcpy(&frame[channel_bytes], justfloat_tail, sizeof(justfloat_tail));
}

static HAL_StatusTypeDef VOFA_Transmit(const uint8_t *frame, uint16_t size)
{
    HAL_StatusTypeDef status;
    if (huart1.hdmatx == NULL) {
        g_vofa_tx_stats.failure_count++;
        g_vofa_tx_stats.last_status = HAL_ERROR;
        return HAL_ERROR;
    }
    status = VOFA_Service();
    if (status != HAL_OK) {
        g_vofa_tx_stats.busy_drop_count++;
        return status;
    }
    taskENTER_CRITICAL();
    if (s_vofa_tx_state != VOFA_TX_IDLE) {
        g_vofa_tx_stats.busy_drop_count++;
        taskEXIT_CRITICAL();
        return HAL_BUSY;
    }
    s_vofa_tx_state = VOFA_TX_BUSY;
    g_vofa_tx_stats.busy = 1U;
    memcpy(s_vofa_dma_frame, frame, size);
    taskEXIT_CRITICAL();
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
        SCB_CleanDCache_by_Addr((uint32_t *)s_vofa_dma_frame, sizeof(s_vofa_dma_frame));
    __DSB();
    s_vofa_start_tick = HAL_GetTick();
    g_vofa_tx_stats.last_status = HAL_OK;
    status = HAL_UART_Transmit_DMA(&huart1, s_vofa_dma_frame, size);
    if (status == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart1.hdmatx, DMA_IT_HT);
        g_vofa_tx_stats.submitted_count++;
    }
    else {
        taskENTER_CRITICAL();
        g_vofa_tx_stats.last_status = status;
        g_vofa_tx_stats.failure_count++;
        s_vofa_tx_state = VOFA_TX_RECOVER;
        taskEXIT_CRITICAL();
    }
    return status;
}

static HAL_StatusTypeDef VOFA_SendChannels(const float *channels, uint32_t count)
{
    uint8_t frame[VOFA_FLIGHT_FRAME_SIZE];
    if ((channels == NULL) || (count == 0U) || (count > VOFA_MAX_CHANNELS))
        return HAL_ERROR;
    VOFA_PackChannels(frame, channels, count);
    return VOFA_Transmit(frame, (uint16_t)(count * sizeof(float) + VOFA_JUSTFLOAT_TAIL_SIZE));
}

HAL_StatusTypeDef VOFA_SendFrame(const uint8_t frame[VOFA_FLIGHT_FRAME_SIZE])
{
    if (frame == NULL) return HAL_ERROR;
    return VOFA_Transmit(frame, VOFA_FLIGHT_FRAME_SIZE);
}

static void VOFA_BuildBase(uint8_t *frame,
                          const FlightControlState_t *flight,
                          const RcSbusData_t *rc, const EscPwmState_t *esc)
{
    const float degrees = 180.0f / 3.14159265358979323846f;
    const float channels[20] = {
        flight->roll_rad * degrees, flight->pitch_rad * degrees, flight->yaw_rad * degrees,
        rc->throttle, rc->roll_target_rad * degrees, rc->pitch_target_rad * degrees,
        rc->yaw_rate_target_rad_s * degrees,
        flight->roll_rate_target_rad_s * degrees, flight->roll_rate_rad_s * degrees,
        flight->pitch_rate_target_rad_s * degrees, flight->pitch_rate_rad_s * degrees,
        flight->yaw_rate_target_rad_s * degrees, flight->yaw_rate_rad_s * degrees,
        flight->roll_rate_raw_rad_s * degrees, flight->pitch_rate_raw_rad_s * degrees,
        flight->yaw_rate_raw_rad_s * degrees,
        esc->command[0], esc->command[1], esc->command[2], esc->command[3]
    };
    memcpy(frame, channels, sizeof(channels));
}

void VOFA_BuildFlightFrame(uint8_t frame[VOFA_FLIGHT_FRAME_SIZE],
                          const FlightControlState_t *flight,
                          const RcSbusData_t *rc, const EscPwmState_t *esc,
                          const OpticalFlowData_t *optical)
{
    const float extra[3] = {(float)optical->distance,
        (float)optical->flow_vel_x, (float)optical->flow_vel_y};
    VOFA_BuildBase(frame, flight, rc, esc);
    VOFA_PackChannels(frame + 80, extra, 3U);
}

void VOFA_BuildBlackboxFrame(uint8_t frame[BLACKBOX_FLIGHT_FRAME_SIZE],
                          const FlightControlState_t *flight,
                          const RcSbusData_t *rc, const EscPwmState_t *esc,
                          const OpticalFlowData_t *optical)
{
    uint32_t i, age = optical->has_sample ? optical->age_ms : UINT32_MAX;
    static const uint8_t tail[4] = {0, 0, 0x80, 0x7F};
    VOFA_BuildBase(frame, flight, rc, esc);
    memset(frame + 80, 0, 27);
    if (optical->has_sample) memcpy(frame + 80, optical->raw, 27);
    for (i = 0; i < 4; ++i) frame[107 + i] = (uint8_t)(age >> (8U * i));
    frame[111] = optical->has_sample ? 1U : 0U;
    memcpy(frame + 112, tail, sizeof(tail));
}

HAL_StatusTypeDef VOFA_SendAttitude(float roll_deg, float pitch_deg, float yaw_deg)
{
    const float channels[3] = {roll_deg, pitch_deg, yaw_deg};
    return VOFA_SendChannels(channels, 3U);
}

HAL_StatusTypeDef VOFA_SendFlightTelemetry(float roll_deg, float pitch_deg, float yaw_deg,
                                           float throttle, float roll_target_deg,
                                           float pitch_target_deg, float yaw_rate_target_deg_s,
                                           const float rates_deg_s[6], float roll_rate_raw_deg_s,
                                           float pitch_rate_raw_deg_s, float yaw_rate_raw_deg_s,
                                           const float motor_command[4])
{
    float channels[20] = {
        roll_deg, pitch_deg, yaw_deg, throttle,
        roll_target_deg, pitch_target_deg, yaw_rate_target_deg_s
    };

    if ((rates_deg_s == NULL) || (motor_command == NULL)) return HAL_ERROR;
    (void)memcpy(&channels[7], rates_deg_s, 6U * sizeof(float));
    channels[13] = roll_rate_raw_deg_s;
    channels[14] = pitch_rate_raw_deg_s;
    channels[15] = yaw_rate_raw_deg_s;
    (void)memcpy(&channels[16], motor_command, 4U * sizeof(float));
    return VOFA_SendChannels(channels, 20U);
}
