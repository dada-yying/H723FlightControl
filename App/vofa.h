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
HAL_StatusTypeDef VOFA_SendFrame(const uint8_t frame[VOFA_FLIGHT_FRAME_SIZE]);

typedef struct
{
    uint32_t success_count;
    uint32_t failure_count;
    HAL_StatusTypeDef last_status;
    uint32_t max_duration_ms; /* HAL tick resolution; includes task preemption. */
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
