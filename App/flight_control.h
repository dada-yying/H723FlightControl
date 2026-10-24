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
    float roll_rad;  /* RFU X: nose-up positive. */
    float pitch_rad; /* RFU Y: right-side-down positive. */
    float yaw_rad;   /* RFU Z: CCW from above positive. */
    float roll_rate_rad_s;
    float pitch_rate_rad_s;
    float yaw_rate_rad_s;
    float roll_rate_target_rad_s;
    float pitch_rate_target_rad_s;
    float yaw_rate_target_rad_s;
    float motor_command[4];
    uint32_t saturation_count;
    uint32_t imu_timeout_count;
    float roll_rate_raw_rad_s; /* Calibrated unfiltered BMI native X, rad/s. */
    float pitch_rate_raw_rad_s; /* Calibrated unfiltered BMI native Y, rad/s. */
    float yaw_rate_raw_rad_s; /* Calibrated unfiltered BMI native Z, rad/s. */
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
    float q;         /* Roll/Pitch Watch: process covariance per 1 ms, 1e-6..1e3. */
    float r;         /* Roll/Pitch Watch: measurement covariance, 1e-6..1e3. */
    float applied_q; /* Observation only; invalid pairs retain previous values. */
    float applied_r;
    float gain[3];   /* Observation: last update K, Roll/Pitch/Yaw; seed uses 1. */
    float yaw_q;     /* Yaw Watch: process covariance per 1 ms, 1e-6..1e3. */
    float yaw_r;     /* Yaw Watch: measurement covariance, 1e-6..1e3. */
    float applied_yaw_q; /* Independent valid-pair acceptance; observation only. */
    float applied_yaw_r;
} FlightControlFilter_t;
extern volatile FlightControlFilter_t g_flight_control_filter;
/* Legacy ATOM filter Watch layout only; all values stay zero, no filtering.
 * Active gyro filter parameters are in g_bmi088_gyro_filter. */

/* Legacy Watch layout retained, zero/inactive with BMI088-only feedback. */
typedef struct
{
    float center_hz;
    float bandwidth_hz; /* Exact -3 dB bandwidth. */
    float sample_hz;
    float applied_sample_hz;
    uint32_t sample_count;
    uint32_t coefficient_update_count;
    uint32_t rate_fault_count;
    uint8_t active; /* Always zero: no additional control-path notch. */
} FlightControlNotchState_t;
extern volatile FlightControlNotchState_t g_flight_control_notch;

void FlightControl_Init(void);
void FlightControl_SetTaskHandle(TaskHandle_t task_handle);
void FlightControl_OnBmi088Update(uint8_t valid); /* BMI task, after publication. */
void FlightControl_Run(void);
void FlightControl_OnTaskTimeout(void);
void FlightControl_GetState(FlightControlState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_CONTROL_H */
