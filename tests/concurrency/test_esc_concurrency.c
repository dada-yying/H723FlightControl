/* Deterministic single-core scheduler: Flight (40) can preempt RC (32).
 * A masked switch remains pending until the outermost critical exit. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define TEST_SCHEDULER_HOOKS
#ifndef ESC_SOURCE_FILE
#define ESC_SOURCE_FILE "../../App/esc_pwm.c"
#endif
#include ESC_SOURCE_FILE

static unsigned current_priority = 32U;
static unsigned trigger_write, writes, pending, hook_enabled, dispatched;
static unsigned action, inject_outer_exit, max_depth;
static const float thrust[4] = {0.25f, 0.25f, 0.25f, 0.25f};
static void dispatch(void)
{
    unsigned previous;
    if (!pending || test_critical_depth || current_priority >= 40U)
        return;
    pending = 0;
    previous = current_priority;
    current_priority = 40U;
    test_current_task = (TaskHandle_t)(uintptr_t)40U;
    dispatched++;
    if (action == 0U)
        EscPwm_SetMixedNormalized(thrust);
    else if (action == 1U)
        EscPwm_ForceMinimum(ESC_PWM_INHIBIT_IMU_TIMEOUT);
    else
    {
        EscPwmState_t observed;
        unsigned i;
        EscPwm_GetState(&observed);
        for (i = 1; i < 4; i++)
            assert(observed.pulse_us[i] == observed.pulse_us[0]);
        assert(observed.pulse_us[0] == htim1.compare[TIM_CHANNEL_3]);
        assert(observed.pulse_us[1] == htim1.compare[TIM_CHANNEL_1]);
        assert(observed.pulse_us[2] == htim2.compare[TIM_CHANNEL_3]);
        assert(observed.pulse_us[3] == htim2.compare[TIM_CHANNEL_1]);
    }
    current_priority = previous;
    test_current_task = (TaskHandle_t)(uintptr_t)previous;
}
static void TestCriticalEnter(void)
{
    test_critical_depth++;
    if (test_critical_depth > max_depth)
        max_depth = test_critical_depth;
}
static void TestCriticalExit(void)
{
    assert(test_critical_depth);
    if (inject_outer_exit && test_critical_depth == 1U)
    {
        inject_outer_exit = 0;
        pending = 1;
    }
    test_critical_depth--;
    dispatch();
}
static void TestCompareHook(void)
{
    if (hook_enabled && ++writes == trigger_write)
    {
        hook_enabled = 0;
        pending = 1;
        dispatch();
    }
}
static RcSbusData_t rc;
static void setup(void)
{
    hook_enabled = pending = dispatched = writes = 0;
    current_priority = 32U;
    test_current_task = (TaskHandle_t)(uintptr_t)32U;
    test_scheduler_state = taskSCHEDULER_NOT_STARTED;
    inject_outer_exit = max_depth = 0;
    test_tick = 0;
    assert(EscPwm_Init() == HAL_OK);
#ifndef ESC_TEST_OLD
    EscPwm_SetRcTaskHandle(test_current_task);
    test_scheduler_state = 1;
#endif
    test_tick = 2000;
    memset(&rc, 0, sizeof(rc));
    rc.valid = 1;
    rc.last_valid_tick = test_tick;
    rc.channel[6] = 1000;
    EscPwm_UpdateFromRc(&rc);
    rc.channel[6] = 1696;
    EscPwm_UpdateFromRc(&rc);
    assert(EscPwm_IsArmed());
    EscPwm_SetMixedNormalized(thrust);
}
static void minimum(void)
{
    EscPwmState_t s;
    unsigned i;
    EscPwm_GetState(&s);
    assert(!s.armed);
    for (i = 0; i < 4; i++)
    {
        assert(s.pulse_us[i] == 1000);
        assert(s.command[i] == 0);
    }
    assert(htim1.compare[TIM_CHANNEL_3] == 1000);
    assert(htim1.compare[TIM_CHANNEL_1] == 1000);
    assert(htim2.compare[TIM_CHANNEL_3] == 1000);
    assert(htim2.compare[TIM_CHANNEL_1] == 1000);
}
int main(int argc, char **argv)
{
    EscPwmState_t s;
    unsigned point;
    assert(argc == 2);
    if (!strcmp(argv[1], "torn_stop"))
    {
        for (point = 1; point <= 4; point++)
        {
            setup();
            trigger_write = point;
            hook_enabled = 1;
            action = 0;
            rc.channel[6] = 800;
            EscPwm_UpdateFromRc(&rc);
            assert(dispatched == 1);
            minimum();
        }
    }
    else if (!strcmp(argv[1], "first_reason"))
    {
        setup();
        EscPwm_ForceMinimum(ESC_PWM_INHIBIT_IMU_TIMEOUT);
        EscPwm_ForceMinimum(ESC_PWM_INHIBIT_FAILSAFE);
        EscPwm_GetState(&s);
        assert(s.inhibit_reason == ESC_PWM_INHIBIT_IMU_TIMEOUT);
        minimum();
    }
    else if (!strcmp(argv[1], "observer_stop"))
    {
        for (point = 1; point <= 4; point++)
        {
            setup();
            trigger_write = point;
            hook_enabled = 1;
            action = 2;
            rc.channel[6] = 800;
            EscPwm_UpdateFromRc(&rc);
            assert(dispatched == 1);
            minimum();
        }
    }
#ifndef ESC_TEST_OLD
    else if (!strcmp(argv[1], "arming_request"))
    {
        EscPwmDiagnostics_t d;
        setup();
        rc.channel[6] = 1000;
        EscPwm_UpdateFromRc(&rc);
        rc.channel[6] = 1696;
        action = 1;
        inject_outer_exit = 1;
        EscPwm_UpdateFromRc(&rc);
        assert(dispatched == 1);
        assert(g_esc_pwm_state.armed == 1); /* Flight did not write formal state. */
        minimum();
        EscPwm_GetDiagnostics(&d);
        assert(d.pending);
        EscPwm_UpdateFromRc(&rc);
        assert(!g_esc_pwm_state.armed && !g_esc_pwm_state.arm_middle_seen);
        EscPwm_GetDiagnostics(&d);
        assert(!d.pending && d.consumed_sequence == d.request_sequence);
        rc.channel[6] = 1000;
        EscPwm_UpdateFromRc(&rc);
        rc.channel[6] = 1696;
        EscPwm_UpdateFromRc(&rc);
        assert(EscPwm_IsArmed());
    }
    else if (!strcmp(argv[1], "stale_generation"))
    {
        EscPwmControlSnapshot_t prior, latest;
        EscPwmDiagnostics_t d;
        setup();
        EscPwm_GetControlSnapshot(&prior);
        /* Deliberate task-boundary yield, NOT lower-priority RC preemption. */
        rc.channel[6] = 800;
        EscPwm_UpdateFromRc(&rc);
        rc.channel[6] = 1000;
        EscPwm_UpdateFromRc(&rc);
        rc.channel[6] = 1696;
        EscPwm_UpdateFromRc(&rc);
        EscPwm_GetControlSnapshot(&latest);
        assert(prior.generation != latest.generation && latest.state.armed);
        assert(!EscPwm_CommitMixedNormalized(thrust, prior.generation));
        assert(htim1.compare[TIM_CHANNEL_3] == 1000);
        assert(htim1.compare[TIM_CHANNEL_1] == 1000);
        assert(htim2.compare[TIM_CHANNEL_3] == 1000);
        assert(htim2.compare[TIM_CHANNEL_1] == 1000);
        assert(EscPwm_CommitMixedNormalized(thrust, latest.generation));
        EscPwm_GetDiagnostics(&d);
        assert(d.stale_command_count == 1);
    }
    else if (!strcmp(argv[1], "cause_accounting"))
    {
        EscPwmDiagnostics_t d;
        setup();
        EscPwm_ForceMinimum(ESC_PWM_INHIBIT_IMU_TIMEOUT);
        EscPwm_ForceMinimum(ESC_PWM_INHIBIT_IMU_TIMEOUT);
        EscPwm_ForceMinimum(ESC_PWM_INHIBIT_FAILSAFE);
        EscPwm_ForceMinimum(ESC_PWM_INHIBIT_RC_TIMEOUT);
        EscPwm_GetDiagnostics(&d);
        assert(d.request_count == 4 && d.coalesced_count == 3 && d.different_reason_count == 2);
        assert(d.first_reason == ESC_PWM_INHIBIT_IMU_TIMEOUT && d.pending);
        assert(d.request_sequence == 1 && d.consumed_sequence == 0);
        minimum();
        EscPwm_UpdateFromRc(&rc);
        EscPwm_ForceMinimum(ESC_PWM_INHIBIT_FAILSAFE);
        EscPwm_GetDiagnostics(&d);
        assert(d.first_reason == ESC_PWM_INHIBIT_FAILSAFE && d.pending);
        assert(d.request_sequence == 2 && d.consumed_sequence == 1);
    }
    else if (!strcmp(argv[1], "consume_then_new_request"))
    {
        EscPwmDiagnostics_t d;
        setup();
        EscPwm_ForceMinimum(ESC_PWM_INHIBIT_FAILSAFE);
        action = 1;
        inject_outer_exit = 1;
        EscPwm_UpdateFromRc(&rc);
        EscPwm_GetDiagnostics(&d);
        assert(d.pending && d.first_reason == ESC_PWM_INHIBIT_IMU_TIMEOUT);
        assert(d.request_sequence == 2 && d.consumed_sequence == 1);
        assert(d.request_count == 2 && d.coalesced_count == 0);
        minimum();
    }
    else if (!strcmp(argv[1], "all_interfaces_blocked"))
    {
        EscPwmControlSnapshot_t snap;
        setup();
        EscPwm_GetControlSnapshot(&snap);
        EscPwm_ForceMinimum(ESC_PWM_INHIBIT_IMU_TIMEOUT);
        EscPwm_SetMotorNormalized(ESC_PWM_MOTOR_1, 1.0f);
        minimum();
        EscPwm_SetAllNormalized(1.0f);
        minimum();
        EscPwm_SetMixedNormalized(thrust);
        minimum();
        assert(!EscPwm_CommitMixedNormalized(thrust, snap.generation));
        minimum();
        EscPwm_GetControlSnapshot(&snap);
        assert(!EscPwm_CommitMixedNormalized(thrust, snap.generation));
        minimum();
    }
    else if (!strcmp(argv[1], "nested_mask"))
    {
        setup();
        trigger_write = 2;
        action = 0;
        hook_enabled = 1;
        taskENTER_CRITICAL();
        rc.channel[6] = 800;
        EscPwm_UpdateFromRc(&rc);
        assert(pending && !dispatched && test_critical_depth == 1);
        taskEXIT_CRITICAL();
        assert(dispatched == 1 && max_depth >= 2);
        minimum();
    }
    else if (!strcmp(argv[1], "sequence_wrap"))
    {
        EscPwmControlSnapshot_t snap;
        EscPwmDiagnostics_t d;
        setup();
        s_gate.generation = s_gate.request_sequence = s_gate.consumed_sequence = UINT32_MAX;
        EscPwm_GetControlSnapshot(&snap);
        EscPwm_ForceMinimum(ESC_PWM_INHIBIT_IMU_TIMEOUT);
        EscPwm_GetDiagnostics(&d);
        assert(d.pending && d.request_sequence == 0 && d.generation == 0);
        assert(!EscPwm_CommitMixedNormalized(thrust, snap.generation));
        minimum();
        EscPwm_UpdateFromRc(&rc);
        EscPwm_GetDiagnostics(&d);
        assert(!d.pending && d.consumed_sequence == 0);
    }
    else if (!strcmp(argv[1], "wrong_owner"))
    {
        setup();
        test_current_task = (TaskHandle_t)(uintptr_t)40U;
        EscPwm_UpdateFromRc(&rc); /* Must assert before state mutation. */
        return 3;
    }
#endif
    else
        return 2;
    assert(test_critical_depth == 0);
    puts("PASS deterministic ESC interleaving");
    return 0;
}
