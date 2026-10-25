#include "esc_pwm.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"

#define ESC_PWM_STARTUP_DELAY_MS 2000U
#define ESC_PWM_ARM_CHANNEL_INDEX 6U /* SBUS CH7 uses zero-based index 6. */
#define ESC_PWM_ARM_LOW_THRESHOLD 800U
#define ESC_PWM_ARM_HIGH_THRESHOLD 1200U
#define ESC_PWM_ARM_MAX_THROTTLE 0.05f
#define ESC_PWM_RC_TIMEOUT_MS 100U

volatile EscPwmState_t g_esc_pwm_state;

static float EscPwm_ClampCommand(float command)
{
    if (command < 0.0f) return 0.0f;
    if (command > 1.0f) return 1.0f;
    return command;
}

static uint16_t EscPwm_CommandToPulse(float command)
{
    const float clamped = EscPwm_ClampCommand(command);
    return (uint16_t)((float)ESC_PWM_MIN_PULSE_US +
                      clamped * (float)(ESC_PWM_MAX_PULSE_US - ESC_PWM_MIN_PULSE_US) +
                      0.5f);
}

static void EscPwm_WritePulse(EscPwmMotor_t motor, uint16_t pulse_us)
{
    switch (motor)
    {
        case ESC_PWM_MOTOR_1:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_us);
            break;
        case ESC_PWM_MOTOR_2:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
            break;
        case ESC_PWM_MOTOR_3:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse_us);
            break;
        case ESC_PWM_MOTOR_4:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us);
            break;
        default:
            break;
    }
}

static void EscPwm_ApplyMotor(EscPwmMotor_t motor, float command)
{
    const float clamped = EscPwm_ClampCommand(command);
    const uint16_t pulse_us = EscPwm_CommandToPulse(clamped);
    EscPwm_WritePulse(motor, pulse_us);
    g_esc_pwm_state.command[(uint32_t)motor] = clamped;
    g_esc_pwm_state.pulse_us[(uint32_t)motor] = pulse_us;
}

static void EscPwm_ForceMinimumInternal(EscPwmInhibitReason_t reason, uint8_t clear_middle_seen)
{
    uint32_t motor;
    for (motor = 0U; motor < ESC_PWM_MOTOR_COUNT; motor++)
    {
        EscPwm_WritePulse((EscPwmMotor_t)motor, ESC_PWM_MIN_PULSE_US);
        g_esc_pwm_state.command[motor] = 0.0f;
        g_esc_pwm_state.pulse_us[motor] = ESC_PWM_MIN_PULSE_US;
    }
    g_esc_pwm_state.armed = 0U;
    if (clear_middle_seen != 0U) g_esc_pwm_state.arm_middle_seen = 0U;
    g_esc_pwm_state.inhibit_reason = (uint8_t)reason;
    g_esc_pwm_state.update_count++;
}

HAL_StatusTypeDef EscPwm_Init(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    (void)memset((void *)&g_esc_pwm_state, 0, sizeof(g_esc_pwm_state));
    g_esc_pwm_state.startup_tick = HAL_GetTick();
    g_esc_pwm_state.inhibit_reason = (uint8_t)ESC_PWM_INHIBIT_STARTUP;
    EscPwm_ForceMinimumInternal(ESC_PWM_INHIBIT_STARTUP, 1U);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) status = HAL_ERROR;
    if ((status == HAL_OK) && (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)) status = HAL_ERROR;
    if ((status == HAL_OK) && (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)) status = HAL_ERROR;
    if ((status == HAL_OK) && (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)) status = HAL_ERROR;

    if (status != HAL_OK)
    {
        (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
        EscPwm_ForceMinimumInternal(ESC_PWM_INHIBIT_INIT_ERROR, 1U);
        return HAL_ERROR;
    }

    g_esc_pwm_state.initialized = 1U;
    return HAL_OK;
}

void EscPwm_SetMotorNormalized(EscPwmMotor_t motor, float command)
{
    if ((uint32_t)motor >= ESC_PWM_MOTOR_COUNT) return;
    if ((g_esc_pwm_state.initialized == 0U) || (g_esc_pwm_state.armed == 0U))
    {
        EscPwm_WritePulse(motor, ESC_PWM_MIN_PULSE_US);
        g_esc_pwm_state.command[(uint32_t)motor] = 0.0f;
        g_esc_pwm_state.pulse_us[(uint32_t)motor] = ESC_PWM_MIN_PULSE_US;
        return;
    }
    EscPwm_ApplyMotor(motor, command);
    g_esc_pwm_state.update_count++;
}

void EscPwm_SetAllNormalized(float command)
{
    uint32_t motor;
    if ((g_esc_pwm_state.initialized == 0U) || (g_esc_pwm_state.armed == 0U))
    {
        EscPwm_ForceMinimumInternal((g_esc_pwm_state.initialized != 0U) ?
                           (EscPwmInhibitReason_t)g_esc_pwm_state.inhibit_reason :
                           ESC_PWM_INHIBIT_NOT_INITIALIZED,
                           0U);
        return;
    }
    for (motor = 0U; motor < ESC_PWM_MOTOR_COUNT; motor++)
        EscPwm_ApplyMotor((EscPwmMotor_t)motor, command);
    g_esc_pwm_state.update_count++;
}

void EscPwm_SetMixedNormalized(const float motor_command[ESC_PWM_MOTOR_COUNT])
{
    uint32_t motor;
    if ((motor_command == NULL) || (g_esc_pwm_state.initialized == 0U) ||
        (g_esc_pwm_state.armed == 0U))
    {
        return;
    }
    for (motor = 0U; motor < ESC_PWM_MOTOR_COUNT; motor++)
        EscPwm_ApplyMotor((EscPwmMotor_t)motor, motor_command[motor]);
    g_esc_pwm_state.update_count++;
}

void EscPwm_ForceMinimum(EscPwmInhibitReason_t reason)
{
    /* Runtime task writers use the same transaction as RC arming. Startup
     * calls the internal helper directly, before the scheduler exists. */
    taskENTER_CRITICAL();
    EscPwm_ForceMinimumInternal(reason, 1U);
    taskEXIT_CRITICAL();
}

uint8_t EscPwm_IsArmed(void)
{
    return g_esc_pwm_state.armed;
}

void EscPwm_Disarm(void)
{
    EscPwm_ForceMinimum(ESC_PWM_INHIBIT_SWITCH_LOCKED);
}

static void EscPwm_UpdateFromRcLocked(const RcSbusData_t *rc_data)
{
    uint32_t now;
    uint32_t valid_age;
    uint16_t arm_channel;

    if (g_esc_pwm_state.initialized == 0U)
    {
        EscPwm_ForceMinimumInternal(ESC_PWM_INHIBIT_NOT_INITIALIZED, 1U);
        return;
    }
    if (rc_data == NULL)
    {
        EscPwm_ForceMinimumInternal(ESC_PWM_INHIBIT_RC_INVALID, 1U);
        return;
    }

    now = HAL_GetTick();
    valid_age = now - rc_data->last_valid_tick;
    arm_channel = rc_data->channel[ESC_PWM_ARM_CHANNEL_INDEX];
    g_esc_pwm_state.arm_channel = arm_channel;

    if (rc_data->failsafe != 0U)
    {
        EscPwm_ForceMinimumInternal(ESC_PWM_INHIBIT_FAILSAFE, 1U);
        return;
    }
    if ((rc_data->valid == 0U) || (valid_age >= ESC_PWM_RC_TIMEOUT_MS))
    {
        EscPwm_ForceMinimumInternal((rc_data->last_valid_tick != 0U) ?
                           ESC_PWM_INHIBIT_RC_TIMEOUT : ESC_PWM_INHIBIT_RC_INVALID,
                           1U);
        return;
    }
    if (rc_data->frame_lost != 0U)
    {
        /* Hold the last good output. Lost-frame channel bytes are diagnostics only. */
        return;
    }

    if (arm_channel <= ESC_PWM_ARM_LOW_THRESHOLD)
    {
        /* CH7 low remains locked, but it does not satisfy the arm precondition. */
        g_esc_pwm_state.arm_middle_seen = 0U;
        EscPwm_ForceMinimumInternal(ESC_PWM_INHIBIT_SWITCH_LOCKED, 0U);
        return;
    }

    if (arm_channel < ESC_PWM_ARM_HIGH_THRESHOLD)
    {
        /* CH7 middle is the required locked/pre-arm position. */
        if (rc_data->throttle <= ESC_PWM_ARM_MAX_THROTTLE)
            g_esc_pwm_state.arm_middle_seen = 1U;
        else
            g_esc_pwm_state.arm_middle_seen = 0U;
        EscPwm_ForceMinimumInternal(ESC_PWM_INHIBIT_SWITCH_LOCKED, 0U);
        return;
    }

    if ((now - g_esc_pwm_state.startup_tick) < ESC_PWM_STARTUP_DELAY_MS)
    {
        EscPwm_ForceMinimumInternal(ESC_PWM_INHIBIT_STARTUP, 0U);
        return;
    }

    if (g_esc_pwm_state.armed == 0U)
    {
        if ((g_esc_pwm_state.arm_middle_seen == 0U) ||
            (rc_data->throttle > ESC_PWM_ARM_MAX_THROTTLE))
        {
            EscPwm_ForceMinimumInternal(ESC_PWM_INHIBIT_THROTTLE_HIGH, 0U);
            return;
        }
        g_esc_pwm_state.armed = 1U;
        /* Consume the middle-to-high sequence; the next arm requires middle again. */
        g_esc_pwm_state.arm_middle_seen = 0U;
        g_esc_pwm_state.inhibit_reason = (uint8_t)ESC_PWM_INHIBIT_NONE;
    }

    /* FlightControlTask owns motor output after arming. */
}

void EscPwm_UpdateFromRc(const RcSbusData_t *rc_data)
{
    /* Keep qualification checks AND the commit indivisible with respect to
     * FlightControlTask's newer safety lock. A pending timeout runs after
     * unmasking and therefore wins. Only scalar checks and minimum CCR writes
     * are inside; PWM startup, control/mixing and peripheral waits stay outside. */
    taskENTER_CRITICAL();
    EscPwm_UpdateFromRcLocked(rc_data);
    taskEXIT_CRITICAL();
}

void EscPwm_GetState(EscPwmState_t *state)
{
    if (state == NULL) return;
    taskENTER_CRITICAL();
    (void)memcpy(state, (const void *)&g_esc_pwm_state, sizeof(*state));
    taskEXIT_CRITICAL();
}
