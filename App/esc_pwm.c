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

/* Private formal state. Runtime transitions belong only to the bound RcTask.
 * Output fields are serialized by this module; never expose writable storage. */
static EscPwmState_t g_esc_pwm_state;
static TaskHandle_t s_rc_task;
static struct
{
    uint32_t generation;
    uint32_t request_sequence;
    uint32_t consumed_sequence;
    uint32_t request_count;
    uint32_t coalesced_count;
    uint32_t different_reason_count;
    uint32_t stale_command_count;
    EscPwmInhibitReason_t first_reason;
} s_gate;

static uint8_t EscPwm_RequestPending(void)
{
    return (uint8_t)(s_gate.request_sequence != s_gate.consumed_sequence);
}

static float EscPwm_ClampCommand(float command)
{
    if (command < 0.0f)
        return 0.0f;
    if (command > 1.0f)
        return 1.0f;
    return command;
}

static uint16_t EscPwm_CommandToPulse(float command)
{
    const float clamped = EscPwm_ClampCommand(command);
    return (uint16_t)((float)ESC_PWM_MIN_PULSE_US +
                      clamped * (float)(ESC_PWM_MAX_PULSE_US - ESC_PWM_MIN_PULSE_US) + 0.5f);
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

/* Caller holds the ESC critical section, or is the pre-scheduler initializer. */
static void EscPwm_WriteMinimumLocked(void)
{
    uint32_t motor;
    for (motor = 0U; motor < ESC_PWM_MOTOR_COUNT; motor++)
    {
        EscPwm_WritePulse((EscPwmMotor_t)motor, ESC_PWM_MIN_PULSE_US);
        g_esc_pwm_state.command[motor] = 0.0f;
        g_esc_pwm_state.pulse_us[motor] = ESC_PWM_MIN_PULSE_US;
    }
    g_esc_pwm_state.update_count++;
}

/* Only startup and the RcTask locked state-machine helper call this function. */
static void EscPwm_ForceMinimumInternal(EscPwmInhibitReason_t reason, uint8_t clear_middle_seen)
{
    EscPwm_WriteMinimumLocked();
    g_esc_pwm_state.armed = 0U;
    if (clear_middle_seen != 0U)
    {
        g_esc_pwm_state.arm_middle_seen = 0U;
    }
    g_esc_pwm_state.inhibit_reason = (uint8_t)reason;
    s_gate.generation++;
}

HAL_StatusTypeDef EscPwm_Init(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    configASSERT(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED);
    memset(&g_esc_pwm_state, 0, sizeof(g_esc_pwm_state));
    memset(&s_gate, 0, sizeof(s_gate));
    s_rc_task = NULL;
    g_esc_pwm_state.startup_tick = HAL_GetTick();
    g_esc_pwm_state.inhibit_reason = (uint8_t)ESC_PWM_INHIBIT_STARTUP;
    EscPwm_ForceMinimumInternal(ESC_PWM_INHIBIT_STARTUP, 1U);
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
        status = HAL_ERROR;
    if ((status == HAL_OK) && (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK))
        status = HAL_ERROR;
    if ((status == HAL_OK) && (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK))
        status = HAL_ERROR;
    if ((status == HAL_OK) && (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK))
        status = HAL_ERROR;
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

void EscPwm_SetRcTaskHandle(TaskHandle_t task)
{
    configASSERT(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED);
    configASSERT(task != NULL);
    s_rc_task = task;
}

void EscPwm_GetControlSnapshot(EscPwmControlSnapshot_t *snapshot)
{
    if (snapshot == NULL)
        return;
    /* ESC-A1: state, generation and pending reason; budget 2 us, NOT measured. */
    taskENTER_CRITICAL();
    snapshot->state = g_esc_pwm_state;
    snapshot->generation = s_gate.generation;
    if (EscPwm_RequestPending())
    {
        snapshot->state.armed = 0U;
        snapshot->state.arm_middle_seen = 0U;
        snapshot->state.inhibit_reason = (uint8_t)s_gate.first_reason;
    }
    taskEXIT_CRITICAL();
}

uint8_t EscPwm_CommitMixedNormalized(const float motor_command[ESC_PWM_MOTOR_COUNT],
                                     uint32_t generation)
{
    float normalized[ESC_PWM_MOTOR_COUNT];
    uint16_t pulses[ESC_PWM_MOTOR_COUNT];
    uint32_t motor;
    uint8_t accepted = 0U;
    if (motor_command == NULL)
        return 0U;
    for (motor = 0U; motor < ESC_PWM_MOTOR_COUNT; motor++)
    {
        normalized[motor] = EscPwm_ClampCommand(motor_command[motor]);
        pulses[motor] = EscPwm_CommandToPulse(normalized[motor]);
    }
    /* ESC-A2: gate recheck plus all four CCR/software writes; budget 5 us,
     * NOT measured. Float conversion is outside. No peripheral wait inside. */
    taskENTER_CRITICAL();
    if (generation != s_gate.generation)
    {
        s_gate.stale_command_count++;
    }
    else if (g_esc_pwm_state.initialized && g_esc_pwm_state.armed && !EscPwm_RequestPending())
    {
        for (motor = 0U; motor < ESC_PWM_MOTOR_COUNT; motor++)
        {
            EscPwm_WritePulse((EscPwmMotor_t)motor, pulses[motor]);
            g_esc_pwm_state.command[motor] = normalized[motor];
            g_esc_pwm_state.pulse_us[motor] = pulses[motor];
        }
        g_esc_pwm_state.update_count++;
        accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

/* Convenience commands acquire a fresh generation. Computed flight commands
 * must use GetControlSnapshot/CommitMixedNormalized around their calculation. */
void EscPwm_SetMixedNormalized(const float motor_command[ESC_PWM_MOTOR_COUNT])
{
    EscPwmControlSnapshot_t snapshot;
    EscPwm_GetControlSnapshot(&snapshot);
    (void)EscPwm_CommitMixedNormalized(motor_command, snapshot.generation);
}

void EscPwm_SetMotorNormalized(EscPwmMotor_t motor, float command)
{
    float normalized;
    uint16_t pulse;
    if ((uint32_t)motor >= ESC_PWM_MOTOR_COUNT)
        return;
    normalized = EscPwm_ClampCommand(command);
    pulse = EscPwm_CommandToPulse(normalized);
    /* ESC-A3: direct command gate and one CCR/state pair; budget 2 us,
     * NOT measured. This API is not used for delayed flight calculations. */
    taskENTER_CRITICAL();
    if (!g_esc_pwm_state.initialized || !g_esc_pwm_state.armed || EscPwm_RequestPending())
    {
        EscPwm_WritePulse(motor, ESC_PWM_MIN_PULSE_US);
        g_esc_pwm_state.command[(uint32_t)motor] = 0.0f;
        g_esc_pwm_state.pulse_us[(uint32_t)motor] = ESC_PWM_MIN_PULSE_US;
    }
    else
    {
        EscPwm_WritePulse(motor, pulse);
        g_esc_pwm_state.command[(uint32_t)motor] = normalized;
        g_esc_pwm_state.pulse_us[(uint32_t)motor] = pulse;
        g_esc_pwm_state.update_count++;
    }
    taskEXIT_CRITICAL();
}

void EscPwm_SetAllNormalized(float command)
{
    const float normalized = EscPwm_ClampCommand(command);
    const uint16_t pulse = EscPwm_CommandToPulse(normalized);
    uint32_t motor;
    /* ESC-A7: direct all-motor command and gate; budget 5 us, NOT measured.
     * Retain minimum writes/counting when disarmed without changing RC state. */
    taskENTER_CRITICAL();
    if (!g_esc_pwm_state.initialized || !g_esc_pwm_state.armed || EscPwm_RequestPending())
    {
        EscPwm_WriteMinimumLocked();
    }
    else
    {
        for (motor = 0U; motor < ESC_PWM_MOTOR_COUNT; motor++)
        {
            EscPwm_WritePulse((EscPwmMotor_t)motor, pulse);
            g_esc_pwm_state.command[motor] = normalized;
            g_esc_pwm_state.pulse_us[motor] = pulse;
        }
        g_esc_pwm_state.update_count++;
    }
    taskEXIT_CRITICAL();
}

void EscPwm_ForceMinimum(EscPwmInhibitReason_t reason)
{
    /* ESC-A4: first pending cause, request accounting, invalidation and immediate
     * four-motor minimum; budget 5 us, NOT measured. Formal arming is RC-owned. */
    taskENTER_CRITICAL();
    s_gate.request_count++;
    if (!EscPwm_RequestPending())
    {
        s_gate.first_reason = reason;
        s_gate.request_sequence++;
        s_gate.generation++;
    }
    else
    {
        s_gate.coalesced_count++;
        if (reason != s_gate.first_reason)
            s_gate.different_reason_count++;
    }
    EscPwm_WriteMinimumLocked();
    taskEXIT_CRITICAL();
}

void EscPwm_Disarm(void)
{
    EscPwm_ForceMinimum(ESC_PWM_INHIBIT_SWITCH_LOCKED);
}

uint8_t EscPwm_IsArmed(void)
{
    EscPwmControlSnapshot_t snapshot;
    EscPwm_GetControlSnapshot(&snapshot);
    return snapshot.state.armed;
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
        EscPwm_ForceMinimumInternal((rc_data->last_valid_tick != 0U) ? ESC_PWM_INHIBIT_RC_TIMEOUT
                                                                     : ESC_PWM_INHIBIT_RC_INVALID,
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
        s_gate.generation++;
        /* Consume the middle-to-high sequence; the next arm requires middle again. */
        g_esc_pwm_state.arm_middle_seen = 0U;
        g_esc_pwm_state.inhibit_reason = (uint8_t)ESC_PWM_INHIBIT_NONE;
    }

    /* FlightControlTask owns motor output after arming. */
}

void EscPwm_UpdateFromRc(const RcSbusData_t *rc_data)
{
    configASSERT((xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) ||
                 ((s_rc_task != NULL) && (xTaskGetCurrentTaskHandle() == s_rc_task)));
    /* ESC-A5: sole formal state transition, consume pending request before RC
     * decisions, four minimum CCR writes if required; budget 5 us, NOT measured.
     * Fixed branches only; no parsing, RTOS wait or HAL transfer. */
    taskENTER_CRITICAL();
    if (EscPwm_RequestPending())
    {
        g_esc_pwm_state.armed = 0U;
        g_esc_pwm_state.arm_middle_seen = 0U;
        g_esc_pwm_state.inhibit_reason = (uint8_t)s_gate.first_reason;
        s_gate.consumed_sequence = s_gate.request_sequence;
    }
    EscPwm_UpdateFromRcLocked(rc_data);
    taskEXIT_CRITICAL();
}

void EscPwm_GetState(EscPwmState_t *state)
{
    EscPwmControlSnapshot_t snapshot;
    if (state == NULL)
        return;
    EscPwm_GetControlSnapshot(&snapshot);
    *state = snapshot.state;
}

void EscPwm_GetDiagnostics(EscPwmDiagnostics_t *diagnostics)
{
    if (diagnostics == NULL)
        return;
    /* ESC-A6: diagnostic single snapshot; budget 2 us, NOT measured. */
    taskENTER_CRITICAL();
    diagnostics->generation = s_gate.generation;
    diagnostics->request_sequence = s_gate.request_sequence;
    diagnostics->consumed_sequence = s_gate.consumed_sequence;
    diagnostics->request_count = s_gate.request_count;
    diagnostics->coalesced_count = s_gate.coalesced_count;
    diagnostics->different_reason_count = s_gate.different_reason_count;
    diagnostics->stale_command_count = s_gate.stale_command_count;
    diagnostics->pending = EscPwm_RequestPending();
    diagnostics->first_reason = s_gate.first_reason;
    taskEXIT_CRITICAL();
}
