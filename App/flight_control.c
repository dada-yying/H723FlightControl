#include "flight_control.h"

#include <math.h>
#include <string.h>

#include "bmi088_attitude.h"
#include <float.h>
#include "esc_pwm.h"
#include "rc_sbus.h"
#include "stm32h7xx_hal.h"

#define FLIGHT_MAX_RATE_TARGET_RAD_S 4.0f
#define FLIGHT_RATE_OUTPUT_LIMIT 0.50f
#define FLIGHT_RATE_INTEGRAL_LIMIT 0.30f
#define FLIGHT_IMU_TIMEOUT_MS 10U
#define FLIGHT_DEFAULT_DT_S 0.001f
#define FLIGHT_MOTOR_STOP_THROTTLE_RAW 0x0195U
#define FLIGHT_PI 3.14159265358979323846f
#define FLIGHT_YAW_HOLD_MAX_RATE_RAD_S (0.5f * FLIGHT_PI)
#define FLIGHT_YAW_START_THROTTLE 0.05f
/* Legacy Watch blocks remain zero/inactive; BMI088 owns the only gyro LPF. */
volatile FlightControlNotchState_t g_flight_control_notch;
volatile FlightControlPidConfig_t g_flight_control_pid;
volatile FlightControlState_t g_flight_control_state;
volatile FlightControlTuning_t g_flight_control_tuning;
volatile FlightControlFilter_t g_flight_control_filter;
static uint32_t s_sample_count, s_attitude_epoch;
static uint8_t s_publish_divider;
static uint8_t s_tuning_active;
static uint8_t s_previous_armed;
static RcTuningAxis_t s_selected_axis;
static uint8_t s_waiting_center;
static uint32_t s_center_after_frame;
static TaskHandle_t s_flight_task_handle;
static uint32_t s_last_cycle;
static uint8_t s_have_cycle;
static float s_roll_integral;
static float s_pitch_integral;
static float s_yaw_integral;
static float s_last_roll_error;
static float s_last_pitch_error;
static float s_last_yaw_error;
static uint8_t s_have_error;
static uint32_t s_last_control_tick;
static uint8_t s_output_saturated;
/* Flight task owns the heading; published Watch fields are observations only. */
static float s_yaw_target, s_yaw_angle_kp;
static uint8_t s_yaw_hold_active, s_yaw_control_started;

/* 限幅函数 */
static float FlightControl_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

/* 角速度PID控制器 */
static float FlightControl_RatePid(float target, float measured, float dt,
                                   const FlightControlPidAxis_t *pid,
                                   float *integral, float *last_error)
{
    float error = target - measured;
    float derivative = 0.0f;
    float output;

    if (s_have_error != 0U && dt > 0.00001f)
        derivative = (error - *last_error) / dt;
    if (s_output_saturated == 0U)
        *integral += error * dt;
    *integral = FlightControl_Clamp(*integral, -FLIGHT_RATE_INTEGRAL_LIMIT,
                                    FLIGHT_RATE_INTEGRAL_LIMIT);
    output = pid->kp * error + pid->ki * (*integral) + pid->kd * derivative;
    *last_error = error;
    return FlightControl_Clamp(output, -FLIGHT_RATE_OUTPUT_LIMIT,
                               FLIGHT_RATE_OUTPUT_LIMIT);
}

static void FlightControl_ResetController(void)
{
    s_roll_integral = 0.0f;
    s_pitch_integral = 0.0f;
    s_yaw_integral = 0.0f;
    s_last_roll_error = 0.0f;
    s_last_pitch_error = 0.0f;
    s_last_yaw_error = 0.0f;
    s_have_error = 0U;
    s_output_saturated = 0U;
}

static void FlightControl_ResetYawHeading(void)
{
    s_yaw_target = 0.0f;
    s_yaw_hold_active = 0U;
    g_flight_control_state.yaw_target_rad = 0.0f;
    g_flight_control_state.yaw_error_rad = 0.0f;
    g_flight_control_state.yaw_hold_active = 0U;
}

/* Both operands are AHRS angles in [-pi, pi]; one wrap suffices. */
static float FlightControl_YawError(float target, float measured)
{
    float error = target - measured;
    if (error >= FLIGHT_PI) error -= 2.0f * FLIGHT_PI;
    else if (error < -FLIGHT_PI) error += 2.0f * FLIGHT_PI;
    return error;
}

/* Called by the BMI task after coherent publication and acquisition bookkeeping.
 * Notify every two successful reads; faults wake control immediately. */
void FlightControl_OnBmi088Update(uint8_t valid)
{
    if (valid && ++s_publish_divider < 2U) return;
    s_publish_divider = 0U;
    if (s_flight_task_handle != NULL) xTaskNotifyGive(s_flight_task_handle);
}

static uint8_t FlightControl_Finite(float value)
{
    return (uint8_t)(value <= FLT_MAX && value >= -FLT_MAX);
}

void FlightControl_Init(void)
{
    (void)memset((void *)&g_flight_control_notch, 0, sizeof(g_flight_control_notch));
    (void)memset((void *)&g_flight_control_filter, 0, sizeof(g_flight_control_filter));
    s_publish_divider = 0U;
    s_sample_count = s_attitude_epoch = 0U;
    (void)memset((void *)&g_flight_control_tuning, 0, sizeof(g_flight_control_tuning));
    s_tuning_active = 0U;
    s_previous_armed = 0U;
    s_yaw_control_started = 0U;
    s_yaw_angle_kp = 1.0f;
    s_selected_axis = RC_TUNING_ROLL;
    s_waiting_center = 1U;
    s_center_after_frame = 0U;
    (void)memset((void *)&g_flight_control_pid, 0, sizeof(g_flight_control_pid));
    (void)memset((void *)&g_flight_control_state, 0, sizeof(g_flight_control_state));
    s_flight_task_handle = NULL;
    s_last_cycle = 0U;
    s_have_cycle = 0U;
    s_last_control_tick = HAL_GetTick();
    FlightControl_ResetController();
    FlightControl_ResetYawHeading();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    g_flight_control_pid.angle_roll.kp = 3.5f;
    g_flight_control_pid.angle_pitch.kp = 3.5f;
    g_flight_control_pid.angle_yaw.kp = 1.0f;

    g_flight_control_pid.rate_roll.kp = 0.158f;
    g_flight_control_pid.rate_roll.ki = 0.00000002f;
    g_flight_control_pid.rate_roll.kd = 0.002f;

    g_flight_control_pid.rate_pitch.kp = 0.158f;
    g_flight_control_pid.rate_pitch.ki = 0.00000002f;
    g_flight_control_pid.rate_pitch.kd = 0.002f;

    g_flight_control_pid.rate_yaw.kp = 0.3f;
    // g_flight_control_pid.rate_yaw.ki = 0.0f;
    // g_flight_control_pid.rate_yaw.kd = 0.0f;

}

void FlightControl_SetTaskHandle(TaskHandle_t task_handle)
{
    s_flight_task_handle = task_handle;
}

/* BMI sample completion cycles, not task wake time, define control dt. */
void FlightControl_Run(void)
{
    Bmi088Data_t imu;
    Bmi088GyroFilterState_t filter;
    Bmi088AttitudeState_t attitude;
    uint8_t valid;
    unsigned axis;
    RcSbusData_t rc;
    float dt = FLIGHT_DEFAULT_DT_S;
    float roll_target_rate;
    float pitch_target_rate;
    float roll_output = 0.0f;
    float pitch_output = 0.0f;
    float yaw_output = 0.0f;
    float motor[4];
    uint32_t motor_index;
    uint32_t now = HAL_GetTick();

    Bmi088_GetSnapshotWithFilter(&imu, &filter, &attitude);
    RcSbus_GetData(&rc);
//    rc.roll_target_rad += 0.02164208f;      // 校准imu的roll轴偏差
//    rc.pitch_target_rad += 0.05740388f;     // 校准imu的pitch轴偏差
    valid = (uint8_t)(imu.valid && imu.initialized && imu.has_sample &&
        filter.valid && filter.initialized && attitude.valid && attitude.initialized &&
        imu.age_ms < FLIGHT_IMU_TIMEOUT_MS &&
        filter.sample_count == imu.sample_count && attitude.sample_count == imu.sample_count &&
        filter.sample_tick_ms == imu.sample_tick_ms && attitude.sample_tick_ms == imu.sample_tick_ms);
    for (axis = 0U; axis < 3U; axis++)
        if (!FlightControl_Finite(imu.gyro_rad_s[axis]) ||
            !FlightControl_Finite(filter.gyro_rad_s[axis])) valid = 0U;
    if (!FlightControl_Finite(attitude.roll_rad) || !FlightControl_Finite(attitude.pitch_rad) ||
        !FlightControl_Finite(attitude.yaw_rad)) valid = 0U;
    if (valid && s_have_cycle && attitude.epoch == s_attitude_epoch &&
        imu.sample_count == s_sample_count) {
        FlightControl_OnTaskTimeout();
        return; /* Cached sample cannot advance PID or keep its watchdog alive. */
    }
    if (valid && s_have_cycle && attitude.epoch == s_attitude_epoch) {
        dt = SystemCoreClock ? (float)(uint32_t)(imu.completion_cycle - s_last_cycle) /
            (float)SystemCoreClock : 0.0f;
        if (!(dt >= 0.0002f && dt < 0.010f)) valid = 0U;
    } else {
        FlightControl_ResetController();
        FlightControl_ResetYawHeading(); /* Initial sample or new relative-yaw epoch. */
    }
    if (valid) {
        s_last_cycle = imu.completion_cycle;
        s_sample_count = imu.sample_count;
        s_attitude_epoch = attitude.epoch;
        g_flight_control_state.attitude_epoch = attitude.epoch;
        s_have_cycle = 1U;
        g_flight_control_state.dt_s = dt;
        g_flight_control_state.control_hz = (uint32_t)(1.0f / dt + 0.5f);
        /* RFU: native BMI XYZ; no second notch/Kalman/axis remapping. */
        g_flight_control_state.roll_rad = attitude.roll_rad;
        g_flight_control_state.pitch_rad = attitude.pitch_rad;
        g_flight_control_state.yaw_rad = attitude.yaw_rad;
        g_flight_control_state.roll_rate_raw_rad_s = imu.gyro_rad_s[0];
        g_flight_control_state.pitch_rate_raw_rad_s = imu.gyro_rad_s[1];
        g_flight_control_state.yaw_rate_raw_rad_s = imu.gyro_rad_s[2];
        g_flight_control_state.roll_rate_rad_s = filter.gyro_rad_s[0];
        g_flight_control_state.pitch_rate_rad_s = filter.gyro_rad_s[1];
        g_flight_control_state.yaw_rate_rad_s = filter.gyro_rad_s[2];
    } else s_have_cycle = 0U;
    g_flight_control_state.imu_valid = valid;
    g_flight_control_state.rc_valid = (rc.valid != 0U && rc.failsafe == 0U) ? 1U : 0U;
    g_flight_control_state.armed = EscPwm_IsArmed();

    /* Only the request is writable configuration; internal state is authoritative. */
    if ((g_flight_control_state.armed == 0U) &&
        ((g_flight_control_tuning.pid_debug == 0U) || (g_flight_control_tuning.pid_debug == 1U)) &&
        (s_tuning_active != (uint8_t)g_flight_control_tuning.pid_debug))
    {
        s_tuning_active = (uint8_t)g_flight_control_tuning.pid_debug;
        FlightControl_ResetController();
        s_waiting_center = 1U;
        s_center_after_frame = rc.valid_frame_count;
    }
    if (s_tuning_active != 0U)
    {
        if ((rc.tuning_axis_valid == 0U) || (s_selected_axis != rc.tuning_axis) ||
            ((g_flight_control_state.armed != 0U) && (s_previous_armed == 0U)))
        {
            FlightControl_ResetController();
            s_selected_axis = rc.tuning_axis;
            s_waiting_center = 1U;
            s_center_after_frame = rc.valid_frame_count;
        }
        if ((rc.valid != 0U) && (rc.failsafe == 0U) && (rc.frame_lost == 0U) &&
            (rc.tuning_axis_valid != 0U) && (rc.valid_frame_count != s_center_after_frame) &&
            (rc.tuning_rate_rad_s == 0.0f))
            s_waiting_center = 0U;
    }
    if (!g_flight_control_state.armed || !s_previous_armed || s_tuning_active) {
        FlightControl_ResetYawHeading();
        s_yaw_control_started = 0U;
    }
    s_previous_armed = g_flight_control_state.armed;
    g_flight_control_tuning.active = s_tuning_active;
    g_flight_control_tuning.selected_axis = (uint8_t)s_selected_axis;
    g_flight_control_tuning.waiting_center = (s_tuning_active != 0U) ? s_waiting_center : 0U;
    g_flight_control_tuning.input_rate_rad_s = rc.tuning_rate_rad_s;

    /* 检查飞行控制状态，IMU无效、RC无效、failsafe、超时则清零目标 */
    if ((valid == 0U) || (rc.valid == 0U) || (rc.failsafe != 0U) ||
        ((now - rc.last_valid_tick) >= 100U))
    {
        EscPwmInhibitReason_t reason = ESC_PWM_INHIBIT_RC_INVALID;
        if (!valid)
            reason = ESC_PWM_INHIBIT_IMU_TIMEOUT;
        else if (rc.failsafe != 0U)
            reason = ESC_PWM_INHIBIT_FAILSAFE;
        else if ((now - rc.last_valid_tick) >= 100U)
            reason = ESC_PWM_INHIBIT_RC_TIMEOUT;
        FlightControl_ResetController();
        FlightControl_ResetYawHeading();
        s_yaw_control_started = 0U;
        EscPwm_ForceMinimum(reason);
        s_previous_armed = 0U;
        s_waiting_center = 1U;
        g_flight_control_tuning.waiting_center = s_tuning_active;
        g_flight_control_state.running = 0U;
        g_flight_control_state.armed = 0U;
        g_flight_control_state.roll_rate_target_rad_s = 0.0f;
        g_flight_control_state.pitch_rate_target_rad_s = 0.0f;
        g_flight_control_state.yaw_rate_target_rad_s = 0.0f;
        for (motor_index = 0U; motor_index < 4U; motor_index++)
            g_flight_control_state.motor_command[motor_index] = 0.0f;
        return;
    }

    s_last_control_tick = now;

    /* Low GOOD-frame CH3 inhibits all motor commands, including tuning/PID
     * corrections. Preserve arming; a raised throttle resumes normal control. */
    if (rc.accepted_throttle_raw < FLIGHT_MOTOR_STOP_THROTTLE_RAW) {
        FlightControl_ResetController();
        FlightControl_ResetYawHeading();
        s_yaw_control_started = 0U;
        g_flight_control_state.roll_rate_target_rad_s = 0.0f;
        g_flight_control_state.pitch_rate_target_rad_s = 0.0f;
        g_flight_control_state.yaw_rate_target_rad_s = 0.0f;
        for (motor_index = 0U; motor_index < ESC_PWM_MOTOR_COUNT; motor_index++)
            g_flight_control_state.motor_command[motor_index] = 0.0f;
        EscPwm_SetAllNormalized(0.0f);
        g_flight_control_state.running = 1U;
        return;
    }

    roll_target_rate = FlightControl_Clamp(
        g_flight_control_pid.angle_roll.kp * (rc.roll_target_rad - g_flight_control_state.roll_rad),
        -FLIGHT_MAX_RATE_TARGET_RAD_S, FLIGHT_MAX_RATE_TARGET_RAD_S);
    pitch_target_rate = FlightControl_Clamp(
        g_flight_control_pid.angle_pitch.kp * (rc.pitch_target_rad - g_flight_control_state.pitch_rad),
        -FLIGHT_MAX_RATE_TARGET_RAD_S, FLIGHT_MAX_RATE_TARGET_RAD_S);
    g_flight_control_state.roll_rate_target_rad_s = roll_target_rate;
    g_flight_control_state.pitch_rate_target_rad_s = pitch_target_rate;
    g_flight_control_state.yaw_rate_target_rad_s = rc.yaw_rate_target_rad_s;

    if (!s_tuning_active && g_flight_control_state.armed) {
        const float requested_kp = g_flight_control_pid.angle_yaw.kp;
        if (FlightControl_Finite(requested_kp) && requested_kp >= 0.0f)
            s_yaw_angle_kp = requested_kp;
        /* Ground pre-start: do not capture a heading until throttle first rises.
         * Once enabled, low throttle in flight must not recapture the heading. */
        if (rc.throttle > FLIGHT_YAW_START_THROTTLE) s_yaw_control_started = 1U;
        if (rc.yaw_rate_target_rad_s != 0.0f) {
            FlightControl_ResetYawHeading(); /* Stick commands rate directly. */
        } else if (s_yaw_control_started) {
            /* Only a good centered frame can start a new hold. Lost frames
             * retain an existing hold, just like the existing RC target policy. */
            if (!s_yaw_hold_active && !rc.frame_lost) {
                s_yaw_target = attitude.yaw_rad;
                s_yaw_hold_active = 1U;
            }
            if (s_yaw_hold_active) {
                const float error = FlightControl_YawError(s_yaw_target, attitude.yaw_rad);
                g_flight_control_state.yaw_target_rad = s_yaw_target;
                g_flight_control_state.yaw_error_rad = error;
                g_flight_control_state.yaw_hold_active = 1U;
                g_flight_control_state.yaw_rate_target_rad_s = FlightControl_Clamp(
                    s_yaw_angle_kp * error,
                    -FLIGHT_YAW_HOLD_MAX_RATE_RAD_S, FLIGHT_YAW_HOLD_MAX_RATE_RAD_S);
            }
        }
    }

    if (s_tuning_active != 0U)
    {
        float target = (s_waiting_center == 0U) ? rc.tuning_rate_rad_s : 0.0f;
        roll_target_rate = (s_selected_axis == RC_TUNING_ROLL) ? target : 0.0f;
        pitch_target_rate = (s_selected_axis == RC_TUNING_PITCH) ? target : 0.0f;
        g_flight_control_state.roll_rate_target_rad_s = roll_target_rate;
        g_flight_control_state.pitch_rate_target_rad_s = pitch_target_rate;
        g_flight_control_state.yaw_rate_target_rad_s = (s_selected_axis == RC_TUNING_YAW) ? target : 0.0f;
    }

    /* 未解锁则清零PID状态变量 */
    if ((g_flight_control_state.armed == 0U) ||
        ((s_tuning_active != 0U) && (s_waiting_center != 0U)))
    {
        FlightControl_ResetController();
    }
    else
    {
        if ((s_tuning_active == 0U) || (s_selected_axis == RC_TUNING_ROLL))
            roll_output = FlightControl_RatePid(roll_target_rate, g_flight_control_state.roll_rate_rad_s,
                                                dt, (const FlightControlPidAxis_t *)&g_flight_control_pid.rate_roll,
                                                &s_roll_integral, &s_last_roll_error);
        if ((s_tuning_active == 0U) || (s_selected_axis == RC_TUNING_PITCH))
            pitch_output = FlightControl_RatePid(pitch_target_rate, g_flight_control_state.pitch_rate_rad_s,
                                                 dt, (const FlightControlPidAxis_t *)&g_flight_control_pid.rate_pitch,
                                                 &s_pitch_integral, &s_last_pitch_error);
        if ((s_tuning_active == 0U) || (s_selected_axis == RC_TUNING_YAW))
            yaw_output = FlightControl_RatePid(g_flight_control_state.yaw_rate_target_rad_s,
                                               g_flight_control_state.yaw_rate_rad_s, dt,
                                               (const FlightControlPidAxis_t *)&g_flight_control_pid.rate_yaw,
                                               &s_yaw_integral, &s_last_yaw_error);
        s_have_error = 1U;
    }

    /* M1=left rear CCW, M2=right rear CW, M3=right front CCW, M4=left front CW. */
    /* 四轴混控 */
    motor[0] = rc.throttle - roll_output + pitch_output - yaw_output;
    motor[1] = rc.throttle - roll_output - pitch_output + yaw_output;
    motor[2] = rc.throttle + roll_output - pitch_output - yaw_output;
    motor[3] = rc.throttle + roll_output + pitch_output + yaw_output;
    s_output_saturated = 0U;
    for (motor_index = 0U; motor_index < 4U; motor_index++)
    {
        if ((motor[motor_index] < 0.0f) || (motor[motor_index] > 1.0f))
        {
            g_flight_control_state.saturation_count++;
            s_output_saturated = 1U;
        }
        motor[motor_index] = FlightControl_Clamp(motor[motor_index], 0.0f, 1.0f);
        if (g_flight_control_state.armed == 0U) motor[motor_index] = 0.0f;
        g_flight_control_state.motor_command[motor_index] = motor[motor_index];
    }
    if (EscPwm_IsArmed() != 0U)
        EscPwm_SetMixedNormalized(motor);
    g_flight_control_state.running = 1U;
}

/* IMU超时断流保护 */
void FlightControl_OnTaskTimeout(void)
{
    if ((HAL_GetTick() - s_last_control_tick) >= FLIGHT_IMU_TIMEOUT_MS)
    {
        s_have_cycle = 0U;
        FlightControl_ResetController();
        FlightControl_ResetYawHeading();
        s_yaw_control_started = 0U;
        EscPwm_ForceMinimum(ESC_PWM_INHIBIT_IMU_TIMEOUT);
        s_previous_armed = 0U;
        s_waiting_center = 1U;
        g_flight_control_tuning.waiting_center = s_tuning_active;
        g_flight_control_state.running = 0U;
        g_flight_control_state.imu_valid = 0U;
        g_flight_control_state.armed = 0U;
        g_flight_control_state.roll_rate_target_rad_s = 0.0f;
        g_flight_control_state.pitch_rate_target_rad_s = 0.0f;
        g_flight_control_state.yaw_rate_target_rad_s = 0.0f;
        g_flight_control_state.motor_command[0] = 0.0f;
        g_flight_control_state.motor_command[1] = 0.0f;
        g_flight_control_state.motor_command[2] = 0.0f;
        g_flight_control_state.motor_command[3] = 0.0f;
        g_flight_control_state.imu_timeout_count++;
    }
}

void FlightControl_GetState(FlightControlState_t *state)
{
    if (state == NULL) return;
    taskENTER_CRITICAL();
    (void)memcpy(state, (const void *)&g_flight_control_state, sizeof(*state));
    taskEXIT_CRITICAL();
}
