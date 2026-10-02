#include "flight_control.h"

#include <math.h>
#include <string.h>

#include "atom_imu.h"
#include "esc_pwm.h"
#include "rc_sbus.h"
#include "stm32h7xx_hal.h"

#define FLIGHT_MAX_RATE_TARGET_RAD_S 4.0f
#define FLIGHT_RATE_OUTPUT_LIMIT 0.50f
#define FLIGHT_RATE_INTEGRAL_LIMIT 0.30f
#define FLIGHT_IMU_TIMEOUT_MS 10U
#define FLIGHT_DEFAULT_DT_S 0.001f
/* Preserve the production Kalman baseline used by both flight-log analyses. */
#define FLIGHT_FILTER_DEFAULT_Q 0.015f //原为 0.07510204f
#define FLIGHT_FILTER_DEFAULT_R 1.0f
#define FLIGHT_FILTER_MIN_NOISE 0.000001f
#define FLIGHT_FILTER_MAX_NOISE 1000.0f
/* flight-log-2 recommendation: exact -3 dB bandwidth, not RBJ alpha/Q. */
#define FLIGHT_NOTCH_CENTER_HZ 100.0f
#define FLIGHT_NOTCH_BANDWIDTH_HZ 60.0f
#define FLIGHT_NOTCH_RATE_WINDOW 128U
#define FLIGHT_NOTCH_PI 3.14159265358979323846f

typedef struct
{
    float x1, x2, y1, y2;
} FlightNotchHistory_t;

static FlightNotchHistory_t s_notch_history[2];
static float s_notch_b0, s_notch_b1, s_notch_a2;
static float s_notch_elapsed_s, s_notch_window_s, s_notch_design_hz;
static uint32_t s_notch_intervals;
static uint8_t s_notch_active;
volatile FlightControlNotchState_t g_flight_control_notch;

volatile FlightControlPidConfig_t g_flight_control_pid;
volatile FlightControlState_t g_flight_control_state;
volatile FlightControlTuning_t g_flight_control_tuning;
volatile FlightControlFilter_t g_flight_control_filter;
static float s_filter_q;
static float s_filter_r;
static float s_filtered_rate[3];
static float s_filter_covariance[3];
static uint32_t s_filter_rx_count;
static uint8_t s_filter_seen_sample;
static uint8_t s_filter_initialized;
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

static void FlightControl_NotchCoefficients(float sample_hz)
{
    /* Keep the complete rejection band comfortably below Nyquist. If the
     * sustained update rate is too low, use Kalman alone until it recovers. */
    s_notch_active = (sample_hz > 2.0f * (FLIGHT_NOTCH_CENTER_HZ +
                       FLIGHT_NOTCH_BANDWIDTH_HZ)) ? 1U : 0U;
    s_notch_design_hz = sample_hz;
    if (s_notch_active != 0U)
    {
        float beta = tanf(FLIGHT_NOTCH_PI * FLIGHT_NOTCH_BANDWIDTH_HZ / sample_hz);
        s_notch_b0 = 1.0f / (1.0f + beta);
        s_notch_b1 = -2.0f * s_notch_b0 *
            cosf(2.0f * FLIGHT_NOTCH_PI * FLIGHT_NOTCH_CENTER_HZ / sample_hz);
        s_notch_a2 = 2.0f * s_notch_b0 - 1.0f;
    }
    g_flight_control_notch.applied_sample_hz = sample_hz;
    g_flight_control_notch.active = s_notch_active;
    g_flight_control_notch.coefficient_update_count++;
}

static void FlightControl_NotchReset(void)
{
    s_notch_elapsed_s = 0.0f;
    s_notch_window_s = 0.0f;
    s_notch_intervals = 0U;
    g_flight_control_notch.sample_hz = 1.0f / FLIGHT_DEFAULT_DT_S;
    FlightControl_NotchCoefficients(1.0f / FLIGHT_DEFAULT_DT_S);
}

static void FlightControl_NotchMeasurePeriod(void)
{
    s_notch_window_s += s_notch_elapsed_s;
    s_notch_intervals++;
    if (s_notch_intervals >= FLIGHT_NOTCH_RATE_WINDOW)
    {
        float hz = (float)s_notch_intervals / s_notch_window_s;
        uint8_t active = (hz > 2.0f * (FLIGHT_NOTCH_CENTER_HZ +
                            FLIGHT_NOTCH_BANDWIDTH_HZ)) ? 1U : 0U;
        g_flight_control_notch.sample_hz = hz;
        /* Average fresh-sample intervals, not logger throughput. A 1% deadband
         * avoids coefficient modulation by ordinary task scheduling jitter. */
        if ((fabsf(hz - s_notch_design_hz) > 0.01f * s_notch_design_hz) ||
            (active != s_notch_active))
        {
            if ((active == 0U) && (s_notch_active != 0U))
                g_flight_control_notch.rate_fault_count++;
            FlightControl_NotchCoefficients(hz);
        }
        s_notch_window_s = 0.0f;
        s_notch_intervals = 0U;
    }
}

static float FlightControl_NotchApply(uint32_t axis, float input, uint8_t seed)
{
    FlightNotchHistory_t *h = &s_notch_history[axis];
    float output;
    if ((seed != 0U) || (s_notch_active == 0U))
    {
        h->x1 = h->x2 = h->y1 = h->y2 = input;
        return input;
    }
    output = s_notch_b0 * input + s_notch_b1 * h->x1 + s_notch_b0 * h->x2
             - s_notch_b1 * h->y1 - s_notch_a2 * h->y2;
    h->x2 = h->x1;
    h->x1 = input;
    h->y2 = h->y1;
    h->y1 = output;
    return output;
}

/* Task-only scalar random-walk Kalman filters, rad/s. No attitude/bias fusion.
 * Each axis owns x/P; Q/R are shared. Independent of PID resets. */
/* 卡尔曼滤波 */
static void FlightControl_FilterGyro(const AtomImuData_t *imu, float dt)
{
    float requested_q;
    float requested_r;
    float raw[3];
    float q_step;
    float initial_covariance = 0.0f;
    uint32_t axis;

    if (imu->gyro_valid == 0U)
    {
        s_filter_initialized = 0U;
        return;
    }
    /* Include duplicate-call intervals in the next fresh-sample period, but
     * never iterate the notch or its period estimator on a cached sample. */
    s_notch_elapsed_s += dt;
    if ((s_filter_seen_sample != 0U) && (imu->gyro_rx_count == s_filter_rx_count))
        return;
    if (s_filter_initialized == 0U)
        FlightControl_NotchReset();
    else
        FlightControl_NotchMeasurePeriod();
    s_notch_elapsed_s = 0.0f;
    /* Read once per fresh sample. Reject the whole pair if either is invalid.
     * Ordered bounds also reject NaN and either infinity. */
    requested_q = g_flight_control_filter.q;
    requested_r = g_flight_control_filter.r;
    if ((requested_q >= FLIGHT_FILTER_MIN_NOISE) &&
        (requested_q <= FLIGHT_FILTER_MAX_NOISE) &&
        (requested_r >= FLIGHT_FILTER_MIN_NOISE) &&
        (requested_r <= FLIGHT_FILTER_MAX_NOISE))
    {
        s_filter_q = requested_q;
        s_filter_r = requested_r;
    }
    g_flight_control_filter.applied_q = s_filter_q;
    g_flight_control_filter.applied_r = s_filter_r;
    s_filter_seen_sample = 1U;
    s_filter_rx_count = imu->gyro_rx_count;
    /* Preserve the confirmed PID/telemetry signs and axis mapping.
     * CAN decoding and Euler-angle mapping remain unchanged. */
    raw[0] = -imu->gyro_y;
    raw[1] = imu->gyro_x; /* Pitch: nose-down rate positive. */
    raw[2] = -imu->gyro_z;
    g_flight_control_state.roll_rate_raw_rad_s = raw[0];
    g_flight_control_state.pitch_rate_raw_rad_s = raw[1];
    g_flight_control_state.yaw_rate_raw_rad_s = raw[2];
    raw[0] = FlightControl_NotchApply(0U, raw[0], (uint8_t)(s_filter_initialized == 0U));
    raw[1] = FlightControl_NotchApply(1U, raw[1], (uint8_t)(s_filter_initialized == 0U));
    g_flight_control_notch.sample_count++;
    q_step = s_filter_q * (dt / FLIGHT_DEFAULT_DT_S);
    if (s_filter_initialized == 0U)
    {
        /* Positive solution of P^2 + Q*P - Q*R = 0, rationalized to
         * avoid cancellation when Q is much larger than R. */
        initial_covariance = (2.0f * q_step * s_filter_r) /
            (sqrtf(q_step * q_step + 4.0f * q_step * s_filter_r) + q_step);
    }
    for (axis = 0U; axis < 3U; axis++)
    {
        if (s_filter_initialized == 0U)
        {
            s_filtered_rate[axis] = raw[axis];
            s_filter_covariance[axis] = initial_covariance;
            g_flight_control_filter.gain[axis] = 1.0f;
        }
        else
        {
            const float predicted = s_filter_covariance[axis] + q_step;
            const float gain = predicted / (predicted + s_filter_r);
            const float remaining = 1.0f - gain;
            s_filtered_rate[axis] += gain * (raw[axis] - s_filtered_rate[axis]);
            /* Joseph form preserves nonnegative covariance at extreme Q/R. */
            s_filter_covariance[axis] = remaining * remaining * predicted +
                                       gain * gain * s_filter_r;
            g_flight_control_filter.gain[axis] = gain;
        }
    }
    s_filter_initialized = 1U;
    g_flight_control_state.roll_rate_rad_s = s_filtered_rate[0];
    g_flight_control_state.pitch_rate_rad_s = s_filtered_rate[1];
    g_flight_control_state.yaw_rate_rad_s = s_filtered_rate[2];
}

void FlightControl_Init(void)
{
    (void)memset((void *)&g_flight_control_notch, 0, sizeof(g_flight_control_notch));
    (void)memset(s_notch_history, 0, sizeof(s_notch_history));
    g_flight_control_notch.center_hz = FLIGHT_NOTCH_CENTER_HZ;
    g_flight_control_notch.bandwidth_hz = FLIGHT_NOTCH_BANDWIDTH_HZ;
    FlightControl_NotchReset();
    s_filter_q = FLIGHT_FILTER_DEFAULT_Q;
    s_filter_r = FLIGHT_FILTER_DEFAULT_R;
    (void)memset((void *)&g_flight_control_filter, 0, sizeof(g_flight_control_filter));
    g_flight_control_filter.q = s_filter_q;
    g_flight_control_filter.r = s_filter_r;
    g_flight_control_filter.applied_q = s_filter_q;
    g_flight_control_filter.applied_r = s_filter_r;
    s_filter_initialized = 0U;
    s_filter_seen_sample = 0U;
    s_filter_rx_count = 0U;
    (void)memset(s_filtered_rate, 0, sizeof(s_filtered_rate));
    (void)memset(s_filter_covariance, 0, sizeof(s_filter_covariance));
    (void)memset((void *)&g_flight_control_tuning, 0, sizeof(g_flight_control_tuning));
    s_tuning_active = 0U;
    s_previous_armed = 0U;
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
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_flight_control_pid.angle_roll.kp = 3.5;
    g_flight_control_pid.angle_pitch.kp = 3.5;
    g_flight_control_pid.rate_roll.kp = 0.15;
    g_flight_control_pid.rate_pitch.kp = 0.15;
    g_flight_control_pid.rate_yaw.kp = 0.3;
}

void FlightControl_SetTaskHandle(TaskHandle_t task_handle)
{
    s_flight_task_handle = task_handle;
}

/* 从ISR通知飞行控制任务 */
void FlightControl_NotifyFromISR(void)
{
    BaseType_t woken = pdFALSE;
    if (s_flight_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(s_flight_task_handle, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/* 计算时间间隔 */
static float FlightControl_DeltaSeconds(void)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t delta;
    if (s_have_cycle == 0U)
    {
        s_last_cycle = now;
        s_have_cycle = 1U;
        return FLIGHT_DEFAULT_DT_S;
    }
    delta = now - s_last_cycle;
    s_last_cycle = now;
    if ((SystemCoreClock == 0U) || (delta == 0U)) return FLIGHT_DEFAULT_DT_S;
    return FlightControl_Clamp((float)delta / (float)SystemCoreClock,
                               0.0002f, 0.005f);
}

void FlightControl_Run(void)
{
    AtomImuData_t imu;
    RcSbusData_t rc;
    float dt = FlightControl_DeltaSeconds();
    float roll_target_rate;
    float pitch_target_rate;
    float roll_output = 0.0f;
    float pitch_output = 0.0f;
    float yaw_output = 0.0f;
    float motor[4];
    uint32_t motor_index;
    uint32_t now = HAL_GetTick();

    AtomImu_GetData(&imu);
    RcSbus_GetData(&rc);
    g_flight_control_state.dt_s = dt;
    g_flight_control_state.control_hz = (uint32_t)(1.0f / dt + 0.5f);
    /* Confirmed airframe attitude: right roll, nose down, right yaw positive.
     * Keep native CAN fields intact; PID and telemetry share this mapping. */
    g_flight_control_state.roll_rad = -imu.pitch;
    g_flight_control_state.pitch_rad = imu.roll;
    g_flight_control_state.yaw_rad = -imu.yaw;
    FlightControl_FilterGyro(&imu, dt);
    g_flight_control_state.imu_valid = (imu.gyro_valid != 0U) ? 1U : 0U;
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
    s_previous_armed = g_flight_control_state.armed;
    g_flight_control_tuning.active = s_tuning_active;
    g_flight_control_tuning.selected_axis = (uint8_t)s_selected_axis;
    g_flight_control_tuning.waiting_center = (s_tuning_active != 0U) ? s_waiting_center : 0U;
    g_flight_control_tuning.input_rate_rad_s = rc.tuning_rate_rad_s;

    /* 检查飞行控制状态，IMU无效、RC无效、failsafe、超时则清零目标 */
    if ((imu.gyro_valid == 0U) || (rc.valid == 0U) || (rc.failsafe != 0U) ||
        ((now - rc.last_valid_tick) >= 100U))
    {
        EscPwmInhibitReason_t reason = ESC_PWM_INHIBIT_RC_INVALID;
        if (rc.failsafe != 0U)
            reason = ESC_PWM_INHIBIT_FAILSAFE;
        else if ((now - rc.last_valid_tick) >= 100U)
            reason = ESC_PWM_INHIBIT_RC_TIMEOUT;
        FlightControl_ResetController();
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

    roll_target_rate = FlightControl_Clamp(
        g_flight_control_pid.angle_roll.kp * (rc.roll_target_rad - g_flight_control_state.roll_rad),
        -FLIGHT_MAX_RATE_TARGET_RAD_S, FLIGHT_MAX_RATE_TARGET_RAD_S);
    pitch_target_rate = FlightControl_Clamp(
        g_flight_control_pid.angle_pitch.kp * (rc.pitch_target_rad - g_flight_control_state.pitch_rad),
        -FLIGHT_MAX_RATE_TARGET_RAD_S, FLIGHT_MAX_RATE_TARGET_RAD_S);
    g_flight_control_state.roll_rate_target_rad_s = roll_target_rate;
    g_flight_control_state.pitch_rate_target_rad_s = pitch_target_rate;
    g_flight_control_state.yaw_rate_target_rad_s = rc.yaw_rate_target_rad_s;

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
    motor[0] = rc.throttle + pitch_output + roll_output + yaw_output;
    motor[1] = rc.throttle + pitch_output - roll_output - yaw_output;
    motor[2] = rc.throttle - pitch_output - roll_output + yaw_output;
    motor[3] = rc.throttle - pitch_output + roll_output - yaw_output;
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
        s_filter_initialized = 0U;
        FlightControl_ResetController();
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
