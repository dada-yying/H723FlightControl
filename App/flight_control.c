#include "flight_control.h"

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
#define FLIGHT_FILTER_DEFAULT_HZ 50.0f
#define FLIGHT_FILTER_MIN_HZ 5.0f
#define FLIGHT_FILTER_MAX_HZ 200.0f
#define FLIGHT_TWO_PI 6.28318530717958647692f

volatile FlightControlPidConfig_t g_flight_control_pid;
volatile FlightControlState_t g_flight_control_state;
volatile FlightControlTuning_t g_flight_control_tuning;
volatile FlightControlFilter_t g_flight_control_filter;
static float s_filter_cutoff_hz;
static float s_filtered_rate[3];
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

static float FlightControl_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

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

/* Task-only LPF in body coordinates, rad/s. Independent of PID resets. */
static void FlightControl_FilterGyro(const AtomImuData_t *imu, float dt)
{
    const float requested = g_flight_control_filter.cutoff_hz;
    float raw[3];
    float alpha;
    uint32_t axis;

    /* Ordered bounds also reject NaN and either infinity. */
    if ((requested >= FLIGHT_FILTER_MIN_HZ) && (requested <= FLIGHT_FILTER_MAX_HZ))
        s_filter_cutoff_hz = requested;
    g_flight_control_filter.applied_cutoff_hz = s_filter_cutoff_hz;
    if (imu->gyro_valid == 0U)
    {
        s_filter_initialized = 0U;
        return;
    }
    if ((s_filter_seen_sample != 0U) && (imu->gyro_rx_count == s_filter_rx_count))
        return;
    s_filter_seen_sample = 1U;
    s_filter_rx_count = imu->gyro_rx_count;
    /* Shared PID/telemetry mapping: swap the former Roll/Pitch rate values.
     * Keep CAN decoding and Euler-angle mapping unchanged. */
    raw[0] = -imu->gyro_y;
    raw[1] = -imu->gyro_x;
    raw[2] = -imu->gyro_z;
    alpha = dt / (1.0f / (FLIGHT_TWO_PI * s_filter_cutoff_hz) + dt);
    for (axis = 0U; axis < 3U; axis++)
    {
        if (s_filter_initialized == 0U)
            s_filtered_rate[axis] = raw[axis];
        else
            s_filtered_rate[axis] += alpha * (raw[axis] - s_filtered_rate[axis]);
    }
    s_filter_initialized = 1U;
    g_flight_control_state.roll_rate_rad_s = s_filtered_rate[0];
    g_flight_control_state.pitch_rate_rad_s = s_filtered_rate[1];
    g_flight_control_state.yaw_rate_rad_s = s_filtered_rate[2];
}

void FlightControl_Init(void)
{
    s_filter_cutoff_hz = FLIGHT_FILTER_DEFAULT_HZ;
    g_flight_control_filter.cutoff_hz = FLIGHT_FILTER_DEFAULT_HZ;
    g_flight_control_filter.applied_cutoff_hz = FLIGHT_FILTER_DEFAULT_HZ;
    s_filter_initialized = 0U;
    s_filter_seen_sample = 0U;
    s_filter_rx_count = 0U;
    (void)memset(s_filtered_rate, 0, sizeof(s_filtered_rate));
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
}

void FlightControl_SetTaskHandle(TaskHandle_t task_handle)
{
    s_flight_task_handle = task_handle;
}

void FlightControl_NotifyFromISR(void)
{
    BaseType_t woken = pdFALSE;
    if (s_flight_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(s_flight_task_handle, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

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
    g_flight_control_state.roll_rad = -imu.roll;
    g_flight_control_state.pitch_rad = imu.pitch;
    g_flight_control_state.yaw_rad = imu.yaw;
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
