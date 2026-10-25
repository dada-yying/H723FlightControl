#ifndef VOFA_H
#define VOFA_H

#include "stm32h7xx_hal.h"
#include "flight_control.h"
#include "esc_pwm.h"
#include "optical_flow.h"

#define VOFA_FLIGHT_FRAME_SIZE 96U
#define BLACKBOX_FLIGHT_FRAME_SIZE 116U

#ifdef __cplusplus
extern "C" {
#endif

/* Common CH0..19; different extensions for VOFA and binary blackbox. */
void VOFA_BuildFlightFrame(uint8_t frame[VOFA_FLIGHT_FRAME_SIZE],
                          const FlightControlState_t *flight,
                          const RcSbusData_t *rc, const EscPwmState_t *esc,
                          const OpticalFlowData_t *optical);
void VOFA_BuildBlackboxFrame(uint8_t frame[BLACKBOX_FLIGHT_FRAME_SIZE],
                          const FlightControlState_t *flight,
                          const RcSbusData_t *rc, const EscPwmState_t *esc,
                          const OpticalFlowData_t *optical);
/* Task-only, nonblocking DMA submission. HAL_OK means accepted, not wire done.
 * Caller storage may be reused immediately; busy drops the new frame. */
HAL_StatusTypeDef VOFA_SendFrame(const uint8_t frame[VOFA_FLIGHT_FRAME_SIZE]);
void VOFA_OnUartError(UART_HandleTypeDef *uart); /* Shared UART error dispatcher. */

typedef struct
{
    uint32_t success_count;
    uint32_t failure_count;
    HAL_StatusTypeDef last_status;
    uint32_t max_duration_ms; /* Submission -> UART TC callback, includes IRQ delay. */
    uint32_t submitted_count, busy_drop_count, timeout_count, dma_error_count;
    uint32_t recovery_count, abort_failure_count, last_uart_error;
    uint8_t busy; /* Includes error recovery; buffer must remain owned. */
} VofaTxStats_t;
extern volatile VofaTxStats_t g_vofa_tx_stats;

HAL_StatusTypeDef VOFA_SendAttitude(float roll_deg, float pitch_deg, float yaw_deg);
/* CH0..2: body attitude in degrees, supplied from FlightControl_GetState(). */
HAL_StatusTypeDef VOFA_SendFlightTelemetry(float roll_deg, float pitch_deg, float yaw_deg,
                                           float throttle, float roll_target_deg,
                                           float pitch_target_deg, float yaw_rate_target_deg_s,
                                           const float rates_deg_s[6], /* R target/actual, P target/actual, Y target/actual */
                                           float roll_rate_raw_deg_s,
                                           float pitch_rate_raw_deg_s,
                                           float yaw_rate_raw_deg_s, /* CH13..15: pre-filter body rates. */
                                           const float motor_command[4]); /* CH16..19: safe ESC commands, 0..1. */

#ifdef __cplusplus
}
#endif

#endif /* VOFA_H */
