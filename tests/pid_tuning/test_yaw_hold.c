/* Exercise production RC -> control -> ESC through the existing integration fixture. */
#define CONTROL_FIXTURE_ONLY
#include "test_tuning.c"

static void test_release_restores_captured_heading(void)
{
    reset_control();
    FlightControl_Init(); /* Production yaw outer P; inner gains are set in Watch. */
    g_flight_control_pid.rate_yaw.kp = .2f;
    source(1000); /* Init resets DWT: publish a new completion timestamp. */
    arm();
    g_rc_sbus_data.yaw_rate_target_rad_s = .8f;
    g_bmi088_attitude_state.yaw_rad = .2f;
    FlightControl_Run();
    near_value(g_flight_control_state.yaw_rate_target_rad_s, .8f);

    source(1000);
    g_rc_sbus_data.yaw_rate_target_rad_s = 0;
    g_bmi088_attitude_state.yaw_rad = .6f;
    FlightControl_Run();
    near_value(g_flight_control_state.yaw_rate_target_rad_s, 0);

    source(1000);
    g_bmi088_attitude_state.yaw_rad = .7f;
    FlightControl_Run();
    /* Old rate-only implementation produces zero here: no heading restoration. */
    assert(g_flight_control_state.running && EscPwm_IsArmed());
    assert(g_flight_control_state.yaw_rate_target_rad_s < -.09f);
    assert(g_esc_pwm_state.command[0] > g_esc_pwm_state.command[1]);
}

static void yaw_setup(void)
{
    reset_control();
    g_flight_control_pid.angle_yaw.kp = 1.0f;
    arm();
}

static void yaw_step(float yaw, float stick)
{
    source(1000);
    g_bmi088_attitude_state.yaw_rad = yaw;
    g_rc_sbus_data.yaw_rate_target_rad_s = stick;
    FlightControl_Run();
    assert(g_flight_control_state.running && EscPwm_IsArmed());
}

static void test_capture_once_and_rate_override(void)
{
    yaw_setup();
    yaw_step(.2f, 0);
    assert(g_flight_control_state.yaw_hold_active);
    yaw_step(.8f, 1.2f);
    assert(!g_flight_control_state.yaw_hold_active);
    near_value(g_flight_control_state.yaw_rate_target_rad_s, 1.2f);
    yaw_step(1.1f, 0);
    near_value(g_flight_control_state.yaw_target_rad, 1.1f);
    near_value(g_flight_control_state.yaw_error_rad, 0);
    yaw_step(1.3f, 0);
    near_value(g_flight_control_state.yaw_target_rad, 1.1f);
    near_value(g_flight_control_state.yaw_error_rad, -.2f);
    near_value(g_flight_control_state.yaw_rate_target_rad_s, -.2f);
    yaw_step(-.4f, -2.0f);
    near_value(g_flight_control_state.yaw_rate_target_rad_s, -2.0f);
    yaw_step(-.5f, 0);
    near_value(g_flight_control_state.yaw_target_rad, -.5f);
    yaw_step(-.7f, 0);
    near_value(g_flight_control_state.yaw_rate_target_rad_s, .2f);

    /* Hold transition retains inner-loop integral, and brakes residual rotation. */
    yaw_step(0, .5f);
    s_yaw_integral = .12f;
    g_flight_control_pid.rate_yaw.ki = .1f;
    g_bmi088_gyro_filter.gyro_rad_s[2] = .4f;
    yaw_step(.3f, 0);
    near_value(g_flight_control_state.yaw_rate_target_rad_s, 0);
    near_value(s_yaw_integral, .1196f);
    assert(g_esc_pwm_state.command[0] > g_esc_pwm_state.command[1]);
}

static void test_wrap_limits_and_gain(void)
{
    const float degree = FLIGHT_PI / 180.0f;
    yaw_setup();
    yaw_step(179 * degree, 0);
    yaw_step(-179 * degree, 0);
    near_value(g_flight_control_state.yaw_error_rad, -2 * degree);
    yaw_step(-179 * degree, .1f);
    yaw_step(-179 * degree, 0);
    yaw_step(179 * degree, 0);
    near_value(g_flight_control_state.yaw_error_rad, 2 * degree);
    yaw_step(0, .1f);
    yaw_step(0, 0);
    yaw_step(FLIGHT_PI, 0);
    near_value(g_flight_control_state.yaw_error_rad, -FLIGHT_PI);
    near_value(g_flight_control_state.yaw_rate_target_rad_s, -FLIGHT_YAW_HOLD_MAX_RATE_RAD_S);
    g_flight_control_pid.angle_yaw.kp = 100;
    yaw_step(-.1f, 0);
    near_value(g_flight_control_state.yaw_rate_target_rad_s, FLIGHT_YAW_HOLD_MAX_RATE_RAD_S);
    g_flight_control_pid.angle_yaw.kp = NAN;
    yaw_step(.01f, 0);
    near_value(g_flight_control_state.yaw_rate_target_rad_s, -1);
    g_flight_control_pid.angle_yaw.kp = INFINITY;
    yaw_step(.01f, 0);
    near_value(g_flight_control_state.yaw_rate_target_rad_s, -1);
    g_flight_control_pid.angle_yaw.kp = -1;
    yaw_step(.01f, 0);
    near_value(g_flight_control_state.yaw_rate_target_rad_s, -1);
    g_flight_control_pid.angle_yaw.kp = 0;
    yaw_step(.01f, 0);
    near_value(g_flight_control_state.yaw_rate_target_rad_s, 0);
}

static void test_sample_epoch_and_prestart(void)
{
    yaw_setup();
    g_rc_sbus_data.throttle = 0;
    g_rc_sbus_data.accepted_throttle_raw = 348;
    yaw_step(.1f, 0);
    assert(!g_flight_control_state.yaw_hold_active);
    g_rc_sbus_data.throttle = .05f;
    g_rc_sbus_data.accepted_throttle_raw = 415;
    yaw_step(.3f, 0);
    assert(!g_flight_control_state.yaw_hold_active);
    g_rc_sbus_data.throttle = .06f;
    g_rc_sbus_data.accepted_throttle_raw = 429;
    yaw_step(.5f, 0);
    near_value(g_flight_control_state.yaw_target_rad, .5f);
    /* Below yaw's 5% start gate, but above the new motor-stop threshold. */
    g_rc_sbus_data.throttle = .04f;
    g_rc_sbus_data.accepted_throttle_raw = 402;
    yaw_step(.7f, 0);
    near_value(g_flight_control_state.yaw_target_rad, .5f);
    g_rc_sbus_data.throttle = .5f;
    g_rc_sbus_data.accepted_throttle_raw = 1022;
    yaw_step(.8f, 1);
    g_rc_sbus_data.yaw_rate_target_rad_s = 0;
    FlightControl_Run(); /* Same sample: release is processed on the next new sample. */
    assert(!g_flight_control_state.yaw_hold_active);
    source(1250);
    g_bmi088_attitude_state.yaw_rad = .9f;
    FlightControl_Run();
    near_value(g_flight_control_state.yaw_target_rad, .9f);
    near_value(g_flight_control_state.dt_s, .00125f);
    source(1000);
    g_bmi088_attitude_state.epoch = 2;
    g_bmi088_attitude_state.yaw_rad = -.2f;
    FlightControl_Run();
    assert(g_flight_control_state.attitude_epoch == 2);
    near_value(g_flight_control_state.yaw_target_rad, -.2f);
    near_value(g_flight_control_state.yaw_rate_target_rad_s, 0);
    EscPwm_Disarm();
    source(1000); FlightControl_Run();
    assert(!g_flight_control_state.yaw_hold_active);
    arm(); yaw_step(1.4f, 0);
    near_value(g_flight_control_state.yaw_target_rad, 1.4f);
}

static void test_real_rc_deadband_and_lost_frames(void)
{
    yaw_setup(); yaw_step(.1f, 0);
    frame(1024, 1040, 1696, 1024, 1024, 0);
    source(1000); FlightControl_Run();
    near_value(g_flight_control_state.yaw_rate_target_rad_s, -RC_SBUS_MAX_YAW_RATE_RAD_S);
    assert(!g_flight_control_state.yaw_hold_active);
    /* A lost frame whose raw channel is centered must not release the stick. */
    frame(1024, 1040, 1024, 1024, 1024, RC_SBUS_FLAG_FRAME_LOST);
    source(1000); FlightControl_Run();
    near_value(g_flight_control_state.yaw_rate_target_rad_s, -RC_SBUS_MAX_YAW_RATE_RAD_S);
    assert(!g_flight_control_state.yaw_hold_active);
    frame(1024, 1040, 1030, 1024, 1024, 0); /* Inside existing 3% deadband. */
    g_bmi088_attitude_state.yaw_rad = .6f;
    source(1000); FlightControl_Run();
    assert(g_flight_control_state.yaw_hold_active);
    near_value(g_flight_control_state.yaw_target_rad, .6f);
    frame(1024, 1040, 352, 1024, 1024, RC_SBUS_FLAG_FRAME_LOST);
    source(1000); g_bmi088_attitude_state.yaw_rad = .7f; FlightControl_Run();
    near_value(g_flight_control_state.yaw_target_rad, .6f);
    near_value(g_flight_control_state.yaw_rate_target_rad_s, -.1f);
}

static void test_hold_faults(void)
{
    unsigned kind, i;
    for (kind = 0; kind < 6; kind++) {
        yaw_setup(); yaw_step(.5f, 0);
        source(1000);
        switch (kind) {
        case 0: g_rc_sbus_data.valid = 0; break;
        case 1: g_rc_sbus_data.failsafe = 1; break;
        case 2: g_rc_sbus_data.last_valid_tick = test_tick - 100U; break;
        case 3: g_bmi088_attitude_state.valid = 0; break;
        case 4: g_bmi088_attitude_state.yaw_rad = NAN; break;
        default: test_tick += 10; break;
        }
        if (kind == 5) FlightControl_OnTaskTimeout();
        else FlightControl_Run();
        assert(!EscPwm_IsArmed() && !g_flight_control_state.yaw_hold_active);
        near_value(g_flight_control_state.yaw_rate_target_rad_s, 0);
        for (i = 0; i < 4; i++) near_value(g_esc_pwm_state.command[i], 0);
        g_rc_sbus_data.valid = 1; g_rc_sbus_data.failsafe = 0;
        g_bmi088_attitude_state.yaw_rad = -.8f;
        source(1000); FlightControl_Run();
        assert(!EscPwm_IsArmed() && !g_flight_control_state.yaw_hold_active);
        arm(); yaw_step(-.8f, 0);
        near_value(g_flight_control_state.yaw_target_rad, -.8f);
    }
}

static void test_yaw_tuning_bypasses_hold(void)
{
    unsigned i;
    yaw_setup(); yaw_step(.5f, 0);
    EscPwm_Disarm();
    g_flight_control_tuning.pid_debug = 1;
    for (i = 0; i < 3; i++) frame(1024, 1040, 1024, 1024, 1024, 0);
    source(1000); FlightControl_Run();
    assert(g_flight_control_tuning.active && !g_flight_control_state.yaw_hold_active);
    arm(); source(1000); FlightControl_Run();
    assert(g_flight_control_tuning.waiting_center);
    frame(1024, 1040, 1024, 1024, 1024, 0);
    source(1000); FlightControl_Run();
    assert(!g_flight_control_tuning.waiting_center);
    frame(1024, 1040, 1024, 1024, 352, 0);
    g_rc_sbus_data.throttle = .5f;
    source(1000); g_bmi088_attitude_state.yaw_rad = 2; FlightControl_Run();
    near_value(g_flight_control_state.yaw_rate_target_rad_s, g_rc_sbus_data.tuning_rate_rad_s);
    assert(!g_flight_control_state.yaw_hold_active);
    frame(1024, 1040, 1024, 1024, 1024, 0);
    source(1000); FlightControl_Run();
    near_value(g_flight_control_state.yaw_rate_target_rad_s, 0);
    assert(!g_flight_control_state.yaw_hold_active);
    g_flight_control_tuning.pid_debug = 0;
    source(1000); FlightControl_Run();
    assert(g_flight_control_tuning.active); /* Armed mode switch still deferred. */
    EscPwm_Disarm(); source(1000); FlightControl_Run();
    assert(!g_flight_control_tuning.active);
    arm(); yaw_step(-1.2f, 0);
    near_value(g_flight_control_state.yaw_target_rad, -1.2f);
}

int main(void)
{
    test_release_restores_captured_heading();
    test_capture_once_and_rate_override();
    test_wrap_limits_and_gain();
    test_sample_epoch_and_prestart();
    test_real_rc_deadband_and_lost_frames();
    test_hold_faults();
    test_yaw_tuning_bypasses_hold();
    puts("PASS yaw hold: release capture, rate override, braking/integral, wraps/limits, fresh samples/epoch, prestart/rearm, RC loss/faults and tuning bypass");
    return 0;
}
