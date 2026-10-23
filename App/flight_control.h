#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C"
{
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
        float roll_rate_raw_rad_s;  /* Pre-filter body Roll (-gyro_y), telemetry only. */
        float pitch_rate_raw_rad_s; /* Pre-filter body Pitch (+gyro_x). */
        float yaw_rate_raw_rad_s;   /* Pre-filter body Yaw (-gyro_z). */
    } FlightControlState_t;

    typedef struct
    {
        FlightControlPidConfig_t pid;
        float q, r, yaw_q, yaw_r;
        uint32_t pid_debug;
    } FlightControlConfig_t;

    typedef struct
    {
        FlightControlConfig_t config;
        uint32_t commit_sequence;
    } FlightControlConfigStaging_t;

#define FLIGHT_CONFIG_REJECT_PID 1U
#define FLIGHT_CONFIG_REJECT_ROLL_PITCH 2U
#define FLIGHT_CONFIG_REJECT_YAW 4U

    typedef struct
    {
        uint32_t ack_sequence;
        uint32_t cycle_sequence;
        uint32_t commit_count;
        uint32_t unstable_reject_count;
        uint32_t nonfinite_reject_count;
        uint32_t rejected_groups;
    } FlightControlConfigStatus_t;

    /* Keil Watch protocol (one writer, no overlapping transaction):
     * 1. Wait until status.ack_sequence equals staging.commit_sequence.
     * 2. Edit staging.config (PID, q/r, yaw_q/yaw_r, pid_debug).
     * 3. LAST, write a different commit_sequence (normally old + 1, wrap allowed).
     * 4. Do not edit staging again until ack_sequence matches the new sequence.
     * The task reads sequence before/after two complete copies and accepts only
     * identical copies with stable sequence. Watch bypasses CPU interrupt masks;
     * this handshake is mandatory, not protection against arbitrary debugger writes.
     * Non-finite PID values reject the complete PID group; each Q/R pair rejects
     * independently. Rejected groups retain their prior accepted values. ACK means
     * processed, not all groups accepted: inspect rejected_groups and counters.
     * Finite Q/R range checks still occur only on fresh gyro samples. Other finite
     * values gain no new limits. pid_debug still follows the existing tuning gate.
     * cycle_sequence identifies the processed transaction used by the whole cycle,
     * including retained values for rejected groups. Gain changes do not reset PID.
     * g_flight_control_config_status and old filter/tuning fields are observations;
     * only staging is writable configuration. Never modify status as an ACK shortcut.
     */
    extern volatile FlightControlConfigStaging_t g_flight_control_config_staging;
    extern volatile FlightControlConfigStatus_t g_flight_control_config_status;
    void FlightControl_GetConfig(FlightControlConfig_t *config,
                                 FlightControlConfigStatus_t *status);
    /* g_flight_control_state is private to the control task. */

    typedef struct
    {
        uint32_t pid_debug; /* Observed request: configure through staging only. */
        uint8_t active;
        uint8_t selected_axis; /* 0=Roll, 1=Yaw, 2=Pitch. */
        uint8_t waiting_center;
        float input_rate_rad_s;
    } FlightControlTuning_t;
    /* g_flight_control_tuning is private to the control task. */

    typedef struct
    {
        float q;         /* Accepted Roll/Pitch request; staging is the write interface. */
        float r;         /* Accepted Roll/Pitch measurement request. */
        float applied_q; /* Observation only; invalid pairs retain previous values. */
        float applied_r;
        float gain[3];       /* Observation: last update K, Roll/Pitch/Yaw; seed uses 1. */
        float yaw_q;         /* Accepted Yaw request; staging is the write interface. */
        float yaw_r;         /* Accepted Yaw measurement request. */
        float applied_yaw_q; /* Independent valid-pair acceptance; observation only. */
        float applied_yaw_r;
    } FlightControlFilter_t;
    /* g_flight_control_filter is private to the control task. */

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
    /* g_flight_control_notch is private to the control task. */

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
