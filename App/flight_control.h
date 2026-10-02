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
    float roll_rad;  /* Body attitude: -imu.pitch, right roll positive. */
    float pitch_rad; /* Body attitude: +imu.roll, nose down positive. */
    float yaw_rad;   /* Body attitude: -imu.yaw, right yaw positive. */
    float roll_rate_rad_s;
    float pitch_rate_rad_s;
    float yaw_rate_rad_s;
    float roll_rate_target_rad_s;
    float pitch_rate_target_rad_s;
    float yaw_rate_target_rad_s;
    float motor_command[4];
    uint32_t saturation_count;
    uint32_t imu_timeout_count;
    float roll_rate_raw_rad_s; /* Pre-filter body Roll (-gyro_y), telemetry only. */
    float pitch_rate_raw_rad_s; /* Pre-filter body Pitch (+gyro_x). */
    float yaw_rate_raw_rad_s; /* Pre-filter body Yaw (-gyro_z). */
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

/* Read-only Watch diagnostics. Roll/Pitch fixed notch precedes Kalman;
 * Yaw uses Kalman alone. Coefficients follow the averaged fresh-sample rate. */
typedef struct
{
    float center_hz;
    float bandwidth_hz; /* Exact -3 dB bandwidth. */
    float sample_hz;
    float applied_sample_hz;
    uint32_t sample_count;
    uint32_t coefficient_update_count;
    uint32_t rate_fault_count;
    uint8_t active; /* 0 only if sustained sample rate cannot support notch. */
} FlightControlNotchState_t;
extern volatile FlightControlNotchState_t g_flight_control_notch;

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
