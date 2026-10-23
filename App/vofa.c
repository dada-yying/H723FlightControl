#include "vofa.h"

#include <string.h>

#include "usart.h"

#define VOFA_JUSTFLOAT_TAIL_SIZE 4U
#define VOFA_UART_TIMEOUT_MS 10U
#define VOFA_MAX_CHANNELS 23U

static VofaTxStats_t g_vofa_tx_stats;

static void VOFA_PackChannels(uint8_t *frame, const float *channels, uint32_t channel_count)
{
    static const uint8_t justfloat_tail[VOFA_JUSTFLOAT_TAIL_SIZE] = {0x00U, 0x00U, 0x80U, 0x7FU};
    const uint32_t channel_bytes = channel_count * sizeof(float);
    (void)memcpy(frame, channels, channel_bytes);
    (void)memcpy(&frame[channel_bytes], justfloat_tail, sizeof(justfloat_tail));
}

static HAL_StatusTypeDef VOFA_Transmit(const uint8_t *frame, uint16_t size)
{
    uint32_t start_tick, duration_ms;
    HAL_StatusTypeDef status;
    start_tick = HAL_GetTick();
    status = HAL_UART_Transmit(&huart1, frame, size, VOFA_UART_TIMEOUT_MS);
    duration_ms = HAL_GetTick() - start_tick;
    /* VOFA-C1: transmit result counters only; 2 us, NOT measured. */
    taskENTER_CRITICAL();
    g_vofa_tx_stats.last_status = status;
    if (status == HAL_OK)
        g_vofa_tx_stats.success_count++;
    else
        g_vofa_tx_stats.failure_count++;
    if (duration_ms > g_vofa_tx_stats.max_duration_ms)
        g_vofa_tx_stats.max_duration_ms = duration_ms;
    taskEXIT_CRITICAL();
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
    if (frame == NULL)
        return HAL_ERROR;
    return VOFA_Transmit(frame, VOFA_FLIGHT_FRAME_SIZE);
}

static void VOFA_BuildBase(uint8_t *frame, const FlightControlState_t *flight,
                           const RcSbusData_t *rc, const EscPwmState_t *esc)
{
    const float degrees = 180.0f / 3.14159265358979323846f;
    const float channels[20] = {flight->roll_rad * degrees,
                                flight->pitch_rad * degrees,
                                flight->yaw_rad * degrees,
                                rc->throttle,
                                rc->roll_target_rad * degrees,
                                rc->pitch_target_rad * degrees,
                                rc->yaw_rate_target_rad_s * degrees,
                                flight->roll_rate_target_rad_s * degrees,
                                flight->roll_rate_rad_s * degrees,
                                flight->pitch_rate_target_rad_s * degrees,
                                flight->pitch_rate_rad_s * degrees,
                                flight->yaw_rate_target_rad_s * degrees,
                                flight->yaw_rate_rad_s * degrees,
                                flight->roll_rate_raw_rad_s * degrees,
                                flight->pitch_rate_raw_rad_s * degrees,
                                flight->yaw_rate_raw_rad_s * degrees,
                                esc->command[0],
                                esc->command[1],
                                esc->command[2],
                                esc->command[3]};
    memcpy(frame, channels, sizeof(channels));
}

void VOFA_BuildFlightFrame(uint8_t frame[VOFA_FLIGHT_FRAME_SIZE],
                           const FlightControlState_t *flight, const RcSbusData_t *rc,
                           const EscPwmState_t *esc, const OpticalFlowData_t *optical)
{
    const float extra[3] = {(float)optical->distance, (float)optical->flow_vel_x,
                            (float)optical->flow_vel_y};
    VOFA_BuildBase(frame, flight, rc, esc);
    VOFA_PackChannels(frame + 80, extra, 3U);
}

void VOFA_BuildBlackboxFrame(uint8_t frame[BLACKBOX_FLIGHT_FRAME_SIZE],
                             const FlightControlState_t *flight, const RcSbusData_t *rc,
                             const EscPwmState_t *esc, const OpticalFlowData_t *optical)
{
    uint32_t i, age = optical->has_sample ? optical->age_ms : UINT32_MAX;
    static const uint8_t tail[4] = {0, 0, 0x80, 0x7F};
    VOFA_BuildBase(frame, flight, rc, esc);
    memset(frame + 80, 0, 27);
    if (optical->has_sample)
        memcpy(frame + 80, optical->raw, 27);
    for (i = 0; i < 4; ++i)
        frame[107 + i] = (uint8_t)(age >> (8U * i));
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
    float channels[20] = {roll_deg,
                          pitch_deg,
                          yaw_deg,
                          throttle,
                          roll_target_deg,
                          pitch_target_deg,
                          yaw_rate_target_deg_s};

    if ((rates_deg_s == NULL) || (motor_command == NULL))
        return HAL_ERROR;
    (void)memcpy(&channels[7], rates_deg_s, 6U * sizeof(float));
    channels[13] = roll_rate_raw_deg_s;
    channels[14] = pitch_rate_raw_deg_s;
    channels[15] = yaw_rate_raw_deg_s;
    (void)memcpy(&channels[16], motor_command, 4U * sizeof(float));
    return VOFA_SendChannels(channels, 20U);
}

void VOFA_GetStats(VofaTxStats_t *stats)
{
    if (!stats) return;
    /* VOFA-C2: completed transmit statistics; 2 us, NOT measured. */
    taskENTER_CRITICAL();
    *stats = g_vofa_tx_stats;
    taskEXIT_CRITICAL();
}
