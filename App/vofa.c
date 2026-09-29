#include "vofa.h"

#include <string.h>

#include "usart.h"

#define VOFA_JUSTFLOAT_TAIL_SIZE       4U
#define VOFA_UART_TIMEOUT_MS           10U
#define VOFA_MAX_CHANNELS              14U

static HAL_StatusTypeDef VOFA_SendChannels(const float *channels, uint32_t channel_count)
{
    uint8_t frame[(VOFA_MAX_CHANNELS * sizeof(float)) + VOFA_JUSTFLOAT_TAIL_SIZE];
    static const uint8_t justfloat_tail[VOFA_JUSTFLOAT_TAIL_SIZE] =
        {0x00U, 0x00U, 0x80U, 0x7FU};
    const uint32_t channel_bytes = channel_count * sizeof(float);

    if ((channels == NULL) || (channel_count == 0U) || (channel_count > VOFA_MAX_CHANNELS))
        return HAL_ERROR;
    (void)memcpy(frame, channels, channel_bytes);
    (void)memcpy(&frame[channel_bytes], justfloat_tail, sizeof(justfloat_tail));
    return HAL_UART_Transmit(&huart1, frame,
                             (uint16_t)(channel_bytes + VOFA_JUSTFLOAT_TAIL_SIZE),
                             VOFA_UART_TIMEOUT_MS);
}

HAL_StatusTypeDef VOFA_SendAttitude(float roll_deg, float pitch_deg, float yaw_deg)
{
    const float channels[3] = {roll_deg, pitch_deg, yaw_deg};
    return VOFA_SendChannels(channels, 3U);
}

HAL_StatusTypeDef VOFA_SendFlightTelemetry(float roll_deg, float pitch_deg, float yaw_deg,
                                           float throttle, float roll_target_deg,
                                           float pitch_target_deg, float yaw_rate_target_deg_s,
                                           const float rates_deg_s[6], float roll_rate_raw_deg_s)
{
    float channels[14] = {
        roll_deg, pitch_deg, yaw_deg, throttle,
        roll_target_deg, pitch_target_deg, yaw_rate_target_deg_s
    };

    if (rates_deg_s == NULL) return HAL_ERROR;
    (void)memcpy(&channels[7], rates_deg_s, 6U * sizeof(float));
    channels[13] = roll_rate_raw_deg_s;
    return VOFA_SendChannels(channels, 14U);
}
