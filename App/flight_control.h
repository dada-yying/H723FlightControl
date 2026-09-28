#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;
} FlightControlPidAxis_t;

typedef struct
{
    FlightControlPidAxis_t angle_roll;
    FlightControlPidAxis_t angle_pitch;
    FlightControlPidAxis_t rate_roll;
    FlightControlPidAxis_t rate_pitch;
    FlightControlPidAxis_t rate_yaw;
} FlightControlPidConfig_t;

typedef struct
{
    uint8_t running;
    uint8_t imu_valid;
    uint8_t rc_valid;
    uint8_t armed;
    uint32_t control_hz;
    float dt_s;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float roll_rate_rad_s;
    float pitch_rate_rad_s;
    float yaw_rate_rad_s;
    float roll_rate_target_rad_s;
    float pitch_rate_target_rad_s;
    float yaw_rate_target_rad_s;
    float motor_command[4];
    uint32_t saturation_count;
    uint32_t imu_timeout_count;
} FlightControlState_t;

extern volatile FlightControlPidConfig_t g_flight_control_pid;
extern volatile FlightControlState_t g_flight_control_state;

typedef struct
{
    uint32_t pid_debug; /* Watch request: 0=normal, 1=single-axis tuning. */
    uint8_t active;
    uint8_t selected_axis; /* 0=Roll, 1=Yaw, 2=Pitch. */
    uint8_t waiting_center;
    float input_rate_rad_s;
} FlightControlTuning_t;
extern volatile FlightControlTuning_t g_flight_control_tuning;

typedef struct
{
    float q;         /* Watch: process covariance per 1 ms, 1e-6..1e3. */
    float r;         /* Watch: measurement covariance, 1e-6..1e3. */
    float applied_q; /* Observation only; invalid pairs retain previous values. */
    float applied_r;
    float gain[3];   /* Observation: last update K, Roll/Pitch/Yaw; seed uses 1. */
} FlightControlFilter_t;
extern volatile FlightControlFilter_t g_flight_control_filter;

void FlightControl_Init(void);
void FlightControl_SetTaskHandle(TaskHandle_t task_handle);
void FlightControl_NotifyFromISR(void);
void FlightControl_Run(void);
void FlightControl_OnTaskTimeout(void);
void FlightControl_GetState(FlightControlState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_CONTROL_H */
