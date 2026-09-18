#include "vofa.h"

#include <string.h>

#include "usart.h"

#define VOFA_CHANNEL_COUNT             3U
#define VOFA_JUSTFLOAT_TAIL_SIZE       4U
#define VOFA_FRAME_SIZE                ((VOFA_CHANNEL_COUNT * sizeof(float)) + VOFA_JUSTFLOAT_TAIL_SIZE)
#define VOFA_UART_TIMEOUT_MS           3U

HAL_StatusTypeDef VOFA_SendAttitude(float roll_deg, float pitch_deg, float yaw_deg)
{
    const float channels[VOFA_CHANNEL_COUNT] = {roll_deg, pitch_deg, yaw_deg};
    uint8_t frame[VOFA_FRAME_SIZE];
    static const uint8_t justfloat_tail[VOFA_JUSTFLOAT_TAIL_SIZE] =
        {0x00U, 0x00U, 0x80U, 0x7FU};

    /*
     * JustFloat is three native IEEE754 floats followed by its 4-byte tail.
     * STM32H723 is little-endian, matching VOFA+ JustFloat's byte order.
     */
    (void)memcpy(frame, channels, sizeof(channels));
    (void)memcpy(&frame[sizeof(channels)], justfloat_tail, sizeof(justfloat_tail));

    return HAL_UART_Transmit(&huart1, frame, (uint16_t)sizeof(frame),
                             VOFA_UART_TIMEOUT_MS);
}
