/* Watch writes bypass the CPU mask. Hooks model those writes, not illegal
 * lower-priority task preemption. Production hooks compile to no operations. */
#include <assert.h>
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <string.h>
static void ConfigHook(unsigned point);
#define FLIGHT_CONFIG_TEST_HOOK(point) ConfigHook(point)
#include "../../App/rc_sbus.c"
#include "../../App/optical_flow.c"
#ifndef FLIGHT_SOURCE_FILE
#define FLIGHT_SOURCE_FILE "../../App/flight_control.c"
#endif
#include FLIGHT_SOURCE_FILE

static AtomImuData_t imu;
static float motors[4];
static unsigned hook_point, hook_action, hook_fired;
static FlightControlConfig_t next_config;
void AtomImu_GetData(AtomImuData_t *data)
{
    *data = imu;
}
void EscPwm_GetControlSnapshot(EscPwmControlSnapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->state.armed = 1;
}
uint8_t EscPwm_CommitMixedNormalized(const float p[4], uint32_t generation)
{
    (void)generation;
    memcpy(motors, p, sizeof(motors));
    return 1;
}
void EscPwm_ForceMinimum(EscPwmInhibitReason_t reason)
{
    (void)reason;
}

static FlightControlConfig_t configuration(float scale)
{
    FlightControlConfig_t c;
    memset(&c, 0, sizeof(c));
    c.pid.angle_roll.kp = 0.5f * scale;
    c.pid.angle_pitch.kp = 0.75f * scale;
    c.pid.rate_roll.kp = 0.1f * scale;
    c.pid.rate_pitch.kp = 0.2f * scale;
    c.pid.rate_yaw.kp = 0.3f * scale;
    c.q = 0.02f;
    c.r = 1.0f;
    c.yaw_q = 0.015f;
    c.yaw_r = 1.0f;
    return c;
}
static void commit(FlightControlConfig_t c)
{
#ifdef FLIGHT_CONFIG_OLD
    g_flight_control_pid = c.pid;
    g_flight_control_filter.q = c.q;
    g_flight_control_filter.r = c.r;
    g_flight_control_filter.yaw_q = c.yaw_q;
    g_flight_control_filter.yaw_r = c.yaw_r;
    g_flight_control_tuning.pid_debug = c.pid_debug;
#else
    g_flight_control_config_staging.config = c;
    g_flight_control_config_staging.commit_sequence++;
#endif
}
static void ConfigHook(unsigned point)
{
    if (point != hook_point || hook_fired)
        return;
    hook_fired = 1;
    if (hook_action == 0)
        commit(next_config);
#ifndef FLIGHT_CONFIG_OLD
    else if (hook_action == 1)
    {
        /* Intentionally violate writer freeze to verify copy mismatch rejection. */
        g_flight_control_config_staging.config.pid.rate_pitch.kp += 1.0f;
    }
#endif
}
static void cycle(unsigned fresh)
{
    test_tick++;
    test_dwt.CYCCNT += 480000U;
    g_rc_sbus_data.last_valid_tick = test_tick;
    if (fresh)
        imu.gyro_rx_count++;
    FlightControl_Run();
}
static void setup(void)
{
    hook_point = hook_action = hook_fired = 0;
    test_tick = 100;
    RcSbus_Init();
    FlightControl_Init();
    memset(&imu, 0, sizeof(imu));
    imu.gyro_valid = imu.euler_valid = 1;
    imu.pitch = -0.2f;
    imu.roll = 0.1f;
    g_rc_sbus_data.valid = 1;
    g_rc_sbus_data.throttle = 0.5f;
    g_rc_sbus_data.yaw_rate_target_rad_s = 0.4f;
    commit(configuration(1));
    cycle(1);
}
static void near_value(float a, float b)
{
    assert(fabsf(a - b) < 0.00001f);
}
static void old_outputs(void)
{
    near_value(g_flight_control_state.roll_rate_target_rad_s, -0.1f);
    near_value(g_flight_control_state.pitch_rate_target_rad_s, -0.075f);
    near_value(motors[0], 0.595f);
    near_value(motors[1], 0.375f);
    near_value(motors[2], 0.645f);
    near_value(motors[3], 0.385f);
}
static void new_outputs(void)
{
    near_value(g_flight_control_state.roll_rate_target_rad_s, -0.2f);
    near_value(g_flight_control_state.pitch_rate_target_rad_s, -0.15f);
    near_value(motors[0], 0.64f);
    near_value(motors[1], 0.24f);
    near_value(motors[2], 0.84f);
    near_value(motors[3], 0.28f);
}
#ifndef FLIGHT_CONFIG_OLD
static FlightControlConfig_t snapshot(FlightControlConfigStatus_t *status)
{
    FlightControlConfig_t c;
    FlightControl_GetConfig(&c, status);
    return c;
}
static void invalid_pid(unsigned field, float value)
{
    FlightControlConfig_t c = configuration(2);
    FlightControlPidAxis_t *axis;
    switch (field / 3U)
    {
    case 0:
        axis = &c.pid.angle_roll;
        break;
    case 1:
        axis = &c.pid.angle_pitch;
        break;
    case 2:
        axis = &c.pid.rate_roll;
        break;
    case 3:
        axis = &c.pid.rate_pitch;
        break;
    default:
        axis = &c.pid.rate_yaw;
        break;
    }
    if (field % 3U == 0)
        axis->kp = value;
    else if (field % 3U == 1)
        axis->ki = value;
    else
        axis->kd = value;
    commit(c);
}
#endif
int main(int argc, char **argv)
{
    unsigned point;
    assert(argc == 2);
    if (!strcmp(argv[1], "cross_axis"))
    {
        for (point = 4; point <= 5; point++)
        {
            setup();
            old_outputs();
            next_config = configuration(2);
            hook_point = point;
            cycle(1);
            assert(hook_fired);
            old_outputs();
#ifndef FLIGHT_CONFIG_OLD
            assert(g_flight_control_config_status.cycle_sequence == 1);
            assert(g_flight_control_config_staging.commit_sequence == 2);
#endif
            hook_point = 0;
            cycle(1);
            new_outputs();
#ifndef FLIGHT_CONFIG_OLD
            assert(g_flight_control_config_status.cycle_sequence == 2);
#endif
        }
    }
#ifndef FLIGHT_CONFIG_OLD
    else if (!strcmp(argv[1], "unstable_sequence"))
    {
        for (point = 1; point <= 3; point++)
        {
            setup();
            commit(configuration(2));
            next_config = configuration(2);
            hook_point = point;
            cycle(1);
            old_outputs();
            assert(g_flight_control_config_status.ack_sequence == 1);
            assert(g_flight_control_config_status.unstable_reject_count == 1);
            hook_point = 0;
            cycle(1);
            new_outputs();
            assert(g_flight_control_config_status.ack_sequence == 3);
        }
    }
    else if (!strcmp(argv[1], "inconsistent_copy"))
    {
        setup();
        commit(configuration(2));
        hook_point = 2;
        hook_action = 1;
        cycle(1);
        old_outputs();
        assert(g_flight_control_config_status.ack_sequence == 1);
        assert(g_flight_control_config_status.unstable_reject_count == 1);
        hook_point = 0;
        cycle(1);
        assert(g_flight_control_config_status.ack_sequence == 2);
        near_value(s_cycle_config.pid.rate_pitch.kp, 1.4f);
    }
    else if (!strcmp(argv[1], "uncommitted_and_next_cycle"))
    {
        setup();
        g_flight_control_config_staging.config = configuration(2);
        cycle(1);
        old_outputs();
        assert(g_flight_control_config_status.ack_sequence == 1);
        g_flight_control_config_staging.commit_sequence++;
        old_outputs();
        cycle(1);
        new_outputs();
    }
    else if (!strcmp(argv[1], "nonfinite_pid"))
    {
        const float bad[] = {NAN, INFINITY, -INFINITY};
        unsigned field, i;
        FlightControlConfigStatus_t status;
        FlightControlConfig_t held, actual;
        setup();
        held = snapshot(&status);
        for (field = 0; field < 15; field++)
        {
            for (i = 0; i < 3; i++)
            {
                invalid_pid(field, bad[i]);
                cycle(1);
                old_outputs();
                actual = snapshot(&status);
                assert(memcmp(&actual.pid, &held.pid, sizeof(held.pid)) == 0);
                assert(status.rejected_groups == FLIGHT_CONFIG_REJECT_PID);
                assert(status.nonfinite_reject_count == field * 3 + i + 1);
                assert(status.ack_sequence == g_flight_control_config_staging.commit_sequence);
            }
        }
    }
    else if (!strcmp(argv[1], "nonfinite_qr_groups"))
    {
        const float bad[] = {NAN, INFINITY, -INFINITY};
        unsigned field, i;
        for (field = 0; field < 4; field++)
        {
            for (i = 0; i < 3; i++)
            {
                FlightControlConfig_t c = configuration(1), actual;
                FlightControlConfigStatus_t status;
                setup();
                c.q = 0.4f;
                c.r = 2;
                c.yaw_q = 0.3f;
                c.yaw_r = 3;
                if (field == 0)
                    c.q = bad[i];
                if (field == 1)
                    c.r = bad[i];
                if (field == 2)
                    c.yaw_q = bad[i];
                if (field == 3)
                    c.yaw_r = bad[i];
                commit(c);
                cycle(1);
                actual = snapshot(&status);
                assert(status.nonfinite_reject_count == 1);
                if (field < 2)
                {
                    assert(status.rejected_groups == FLIGHT_CONFIG_REJECT_ROLL_PITCH);
                    near_value(actual.q, 0.02f);
                    near_value(actual.r, 1);
                    near_value(g_flight_control_filter.applied_yaw_q, 0.3f);
                    near_value(g_flight_control_filter.applied_yaw_r, 3);
                }
                else
                {
                    assert(status.rejected_groups == FLIGHT_CONFIG_REJECT_YAW);
                    near_value(actual.yaw_q, 0.015f);
                    near_value(actual.yaw_r, 1);
                    near_value(g_flight_control_filter.applied_q, 0.4f);
                    near_value(g_flight_control_filter.applied_r, 2);
                }
            }
        }
    }
    else if (!strcmp(argv[1], "finite_rules_and_history"))
    {
        FlightControlConfig_t c, actual;
        FlightControlConfigStatus_t status;
        float history, previous_error;
        setup();
        c = configuration(1);
        history = s_roll_integral;
        previous_error = s_last_roll_error;
        c.pid.rate_roll.kp = -FLT_MAX;
        c.pid.angle_pitch.ki = FLT_MAX;
        c.q = -1;
        c.r = 2000;
        c.pid_debug = 7;
        commit(c);
        (void)FlightControl_BeginConfigCycle();
        actual = snapshot(&status);
        assert(status.rejected_groups == 0);
        assert(actual.pid.rate_roll.kp == -FLT_MAX && actual.pid.angle_pitch.ki == FLT_MAX);
        assert(actual.q == -1 && actual.r == 2000 && actual.pid_debug == 7);
        assert(s_roll_integral == history && s_last_roll_error == previous_error && s_have_error);
        /* Finite out-of-range Q/R still reject only in the old fresh-sample path. */
        c = configuration(1);
        c.q = -1;
        c.r = 2000;
        commit(c);
        cycle(1);
        near_value(g_flight_control_filter.applied_q, 0.02f);
        near_value(g_flight_control_filter.applied_r, 1);
    }
    else if (!strcmp(argv[1], "duplicate_gyro"))
    {
        FlightControlConfig_t c;
        uint32_t count;
        float covariance;
        setup();
        count = g_flight_control_notch.sample_count;
        covariance = s_filter_covariance[0];
        c = configuration(1);
        c.q = 0.3f;
        c.r = 2;
        commit(c);
        cycle(0);
        assert(g_flight_control_config_status.ack_sequence == 2);
        assert(g_flight_control_notch.sample_count == count &&
               s_filter_covariance[0] == covariance);
        near_value(g_flight_control_filter.applied_q, 0.02f);
        cycle(1);
        near_value(g_flight_control_filter.applied_q, 0.3f);
        near_value(g_flight_control_filter.applied_r, 2);
    }
    else if (!strcmp(argv[1], "sequence_wrap_and_status_isolation"))
    {
        setup();
        g_flight_control_config_staging.config = configuration(2);
        g_flight_control_config_staging.commit_sequence = UINT32_MAX;
        cycle(1);
        new_outputs();
        assert(g_flight_control_config_status.ack_sequence == UINT32_MAX);
        g_flight_control_config_status.ack_sequence = 123;
        g_flight_control_filter.q = 999;
        g_flight_control_tuning.pid_debug = 1;
        commit(configuration(1));
        cycle(1);
        old_outputs();
        assert(g_flight_control_config_status.ack_sequence == 0);
        assert(!g_flight_control_tuning.active);
        near_value(g_flight_control_filter.applied_q, 0.02f);
    }
#endif
    else
        return 2;
    assert(test_critical_depth == 0);
    puts("PASS flight configuration transaction");
    return 0;
}
