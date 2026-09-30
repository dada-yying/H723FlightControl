#include "vofa.h"

#include <string.h>

#include "usart.h"

#define VOFA_JUSTFLOAT_TAIL_SIZE       4U
#define VOFA_UART_TIMEOUT_MS           10U
#define VOFA_MAX_CHANNELS              20U

volatile VofaTxStats_t g_vofa_tx_stats;

static HAL_StatusTypeDef VOFA_SendChannels(const float *channels, uint32_t channel_count)
{
    uint8_t frame[(VOFA_MAX_CHANNELS * sizeof(float)) + VOFA_JUSTFLOAT_TAIL_SIZE];
    static const uint8_t justfloat_tail[VOFA_JUSTFLOAT_TAIL_SIZE] =
        {0x00U, 0x00U, 0x80U, 0x7FU};
    const uint32_t channel_bytes = channel_count * sizeof(float);
    uint32_t start_tick, duration_ms;
    HAL_StatusTypeDef status;

    if ((channels == NULL) || (channel_count == 0U) || (channel_count > VOFA_MAX_CHANNELS))
        return HAL_ERROR;
    (void)memcpy(frame, channels, channel_bytes);
    (void)memcpy(&frame[channel_bytes], justfloat_tail, sizeof(justfloat_tail));
    start_tick = HAL_GetTick();
    status = HAL_UART_Transmit(&huart1, frame,
                             (uint16_t)(channel_bytes + VOFA_JUSTFLOAT_TAIL_SIZE),
                             VOFA_UART_TIMEOUT_MS);
    duration_ms = HAL_GetTick() - start_tick;
    g_vofa_tx_stats.last_status = status;
    if (status == HAL_OK) g_vofa_tx_stats.success_count++;
    else g_vofa_tx_stats.failure_count++;
    if (duration_ms > g_vofa_tx_stats.max_duration_ms)
        g_vofa_tx_stats.max_duration_ms = duration_ms;
    return status;
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
