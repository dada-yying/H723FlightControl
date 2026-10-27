#ifndef RC_SBUS_H
#define RC_SBUS_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC_SBUS_CHANNEL_COUNT 16U

typedef enum { RC_TUNING_ROLL = 0, RC_TUNING_YAW = 1, RC_TUNING_PITCH = 2 } RcTuningAxis_t;

typedef struct
{
    uint16_t channel[RC_SBUS_CHANNEL_COUNT];
    float throttle;
    float roll_target_rad; /* RFU X, nose-up positive; CH2 forward is negative. */
    float pitch_target_rad; /* RFU Y, right-side-down positive; CH1 right positive. */
    float yaw_rate_target_rad_s; /* RFU Z, CCW positive; CH4 right is negative. */
    float tuning_rate_rad_s;
    RcTuningAxis_t tuning_axis;
    uint8_t tuning_axis_valid;
    uint32_t frame_count;
    uint32_t valid_frame_count;
    uint32_t frame_lost_count;
    uint32_t failsafe_count;
    uint32_t frame_error_count;
    uint32_t rx_overrun_count;
    uint32_t last_valid_tick;
    uint8_t valid;
    uint8_t frame_lost;
    uint8_t failsafe;
    uint8_t ch17;
    uint8_t ch18;
    uint16_t accepted_throttle_raw; /* CH3 from the last GOOD frame, not lost-frame diagnostics. */
} RcSbusData_t;

extern volatile RcSbusData_t g_rc_sbus_data;


void RcSbus_Init(void);
HAL_StatusTypeDef RcSbus_StartReception(void);
void RcSbus_SetTaskHandle(TaskHandle_t task_handle);
void RcSbus_ProcessPending(void);
void RcSbus_CheckTimeout(void);
void RcSbus_GetData(RcSbusData_t *data);

#ifdef __cplusplus
}
#endif

#endif
