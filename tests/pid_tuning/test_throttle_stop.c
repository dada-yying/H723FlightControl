/* Real SBUS decoder, control, ESC state and four timer compare registers. */
#define CONTROL_FIXTURE_ONLY
#include "test_tuning.c"

static void throttle_frame_full(unsigned throttle, unsigned flags, unsigned axis, unsigned slider)
{
    uint8_t bytes[25] = {0};
    uint16_t channels[16] = {0};
    unsigned c, bit;
    channels[0] = 1024; channels[1] = 1040;
    channels[2] = (uint16_t)throttle; channels[3] = 1024;
    channels[4] = (uint16_t)axis; channels[6] = 1696; channels[9] = (uint16_t)slider;
    bytes[0] = 0x0f; bytes[23] = (uint8_t)flags;
    test_tick++;
    for (c = 0; c < 16; c++) for (bit = 0; bit < 11; bit++)
        if (channels[c] & (1U << bit))
            bytes[1 + (c * 11 + bit) / 8] |= (uint8_t)(1U << ((c * 11 + bit) % 8));
    for (c = 0; c < 25; c++) RcSbus_ConsumeByte(bytes[c]);
}

static void throttle_frame(unsigned throttle, unsigned flags)
{
    throttle_frame_full(throttle, flags, 1024, 1024);
}

static void minimum_while_armed(void)
{
    unsigned i;
    assert(EscPwm_IsArmed() && g_flight_control_state.armed);
    for (i = 0; i < 4; i++) {
        assert(g_esc_pwm_state.pulse_us[i] == ESC_PWM_MIN_PULSE_US);
        near_value(g_esc_pwm_state.command[i], 0);
        near_value(g_flight_control_state.motor_command[i], 0);
    }
    assert(htim1.compare[TIM_CHANNEL_1] == 1000 && htim1.compare[TIM_CHANNEL_3] == 1000);
    assert(htim2.compare[TIM_CHANNEL_1] == 1000 && htim2.compare[TIM_CHANNEL_3] == 1000);
}

static void test_low_throttle_overrides_pid(void)
{
    reset_control(); arm();
    throttle_frame(0x017d, 0);
    source(1000);
    g_bmi088_gyro_filter.gyro_rad_s[0] = .3f;
    g_bmi088_gyro_filter.gyro_rad_s[1] = -.2f;
    g_bmi088_gyro_filter.gyro_rad_s[2] = .4f;
    FlightControl_Run();
    assert(g_flight_control_state.running && g_flight_control_state.imu_valid);
    minimum_while_armed();
}

static void test_boundaries_and_resume(void)
{
    const unsigned values[] = {0, 348, 0x017d, 0x017e, 0x017f, 1022, 0x017d, 0x017e};
    unsigned n, i;
    reset_control(); arm();
    for (n = 0; n < sizeof(values) / sizeof(values[0]); n++) {
        RcSbusData_t rc;
        throttle_frame(values[n], 0);
        RcSbus_GetData(&rc);
        assert(rc.accepted_throttle_raw == values[n]);
        EscPwm_UpdateFromRc(&rc); /* Production RC owner must keep the armed state. */
        source(1000); FlightControl_Run();
        assert(EscPwm_IsArmed() && g_flight_control_state.running);
        assert(g_esc_pwm_state.inhibit_reason == ESC_PWM_INHIBIT_NONE);
        if (values[n] < 0x017e) minimum_while_armed();
        else for (i = 0; i < 4; i++) {
            assert(g_esc_pwm_state.pulse_us[i] > ESC_PWM_MIN_PULSE_US);
            near_value(g_esc_pwm_state.command[i], RcSbus_MapThrottle((uint16_t)values[n]));
        }
    }
}

static void test_pid_reset_heading_and_watchdog(void)
{
    unsigned i;
    reset_control(); arm();
    g_flight_control_pid.angle_yaw.kp = 1;
    throttle_frame(1022, 0);
    source(1000); g_bmi088_attitude_state.yaw_rad = .4f; FlightControl_Run();
    assert(g_flight_control_state.yaw_hold_active);
    s_roll_integral = .1f; s_pitch_integral = -.2f; s_yaw_integral = .3f;
    s_last_roll_error = 1; s_last_pitch_error = -2; s_last_yaw_error = 3;
    s_have_error = s_output_saturated = 1;
    throttle_frame(0x017d, 0);
    source(1000); FlightControl_Run();
    minimum_while_armed();
    near_value(s_roll_integral, 0); near_value(s_pitch_integral, 0); near_value(s_yaw_integral, 0);
    near_value(s_last_roll_error, 0); near_value(s_last_pitch_error, 0); near_value(s_last_yaw_error, 0);
    assert(!s_have_error && !s_output_saturated && !s_yaw_control_started);
    assert(!g_flight_control_state.yaw_hold_active);
    near_value(g_flight_control_state.yaw_target_rad, 0);
    near_value(g_flight_control_state.yaw_error_rad, 0);
    near_value(g_flight_control_state.roll_rate_target_rad_s, 0);
    near_value(g_flight_control_state.pitch_rate_target_rad_s, 0);
    near_value(g_flight_control_state.yaw_rate_target_rad_s, 0);
    for (i = 0; i < 30; i++) {
        source(1000); FlightControl_Run(); FlightControl_OnTaskTimeout();
        minimum_while_armed();
        assert(s_last_control_tick == test_tick);
    }
    throttle_frame(1022, 0);
    source(1000); g_bmi088_attitude_state.yaw_rad = -.8f; FlightControl_Run();
    assert(g_flight_control_state.yaw_hold_active);
    near_value(g_flight_control_state.yaw_target_rad, -.8f);
    throttle_frame(0x017d, 0); source(1000); FlightControl_Run();
    test_tick += 10;
    FlightControl_OnTaskTimeout();
    assert(!EscPwm_IsArmed() && g_esc_pwm_state.inhibit_reason == ESC_PWM_INHIBIT_IMU_TIMEOUT);
}

static void test_good_frame_latch_and_loss(void)
{
    uint32_t last_good, count;
    reset_control(); arm();
    throttle_frame(0x017d, 0); source(1000); FlightControl_Run();
    last_good = g_rc_sbus_data.last_valid_tick;
    count = g_rc_sbus_data.valid_frame_count;
    throttle_frame(1696, RC_SBUS_FLAG_FRAME_LOST);
    assert(g_rc_sbus_data.channel[2] == 1696);
    assert(g_rc_sbus_data.accepted_throttle_raw == 0x017d);
    assert(g_rc_sbus_data.last_valid_tick == last_good && g_rc_sbus_data.valid_frame_count == count);
    source(1000); FlightControl_Run(); minimum_while_armed();
    throttle_frame(1022, 0); source(1000); FlightControl_Run();
    throttle_frame(348, RC_SBUS_FLAG_FRAME_LOST);
    assert(g_rc_sbus_data.accepted_throttle_raw == 1022);
    source(1000); FlightControl_Run();
    assert(EscPwm_IsArmed() && g_esc_pwm_state.pulse_us[0] > 1000);
    throttle_frame(0x017d, 0); source(1000); FlightControl_Run();
    minimum_while_armed();
    /* Even while low throttle is active, stale RC must execute the disarm path. */
    source(1000);
    g_rc_sbus_data.last_valid_tick = test_tick - 100U;
    FlightControl_Run();
    assert(!EscPwm_IsArmed() && g_esc_pwm_state.inhibit_reason == ESC_PWM_INHIBIT_RC_TIMEOUT);
    throttle_frame(1022, 0); source(1000); FlightControl_Run();
    assert(!EscPwm_IsArmed()); /* Good data alone never rearms. */
    RcSbus_PublishSafe();
    assert(g_rc_sbus_data.accepted_throttle_raw == 0 && !g_rc_sbus_data.valid);
    RcSbus_Init();
    assert(g_rc_sbus_data.accepted_throttle_raw == 0);
}

static void test_failsafe_and_imu_priority(void)
{
    unsigned kind, i;
    for (kind = 0; kind < 3; kind++) {
        reset_control(); arm();
        throttle_frame(0x017d, kind == 0 ? RC_SBUS_FLAG_FAILSAFE : 0);
        source(1000);
        if (kind == 1) g_rc_sbus_data.valid = 0;
        if (kind == 2) g_bmi088_attitude_state.valid = 0;
        FlightControl_Run();
        assert(!EscPwm_IsArmed() && !g_flight_control_state.running);
        assert(g_esc_pwm_state.inhibit_reason == (kind == 0 ? ESC_PWM_INHIBIT_FAILSAFE :
            kind == 1 ? ESC_PWM_INHIBIT_RC_INVALID : ESC_PWM_INHIBIT_IMU_TIMEOUT));
        for (i = 0; i < 4; i++) assert(g_esc_pwm_state.pulse_us[i] == 1000);
    }
}

static void test_all_tuning_axes(void)
{
    const unsigned positions[] = {352, 1024, 1696};
    unsigned axis, i;
    for (axis = 0; axis < 3; axis++) {
        reset_control();
        g_flight_control_tuning.pid_debug = 1;
        for (i = 0; i < 3; i++) throttle_frame_full(348, 0, positions[axis], 1024);
        source(1000); FlightControl_Run();
        arm(); source(1000); FlightControl_Run();
        assert(g_flight_control_tuning.active && g_flight_control_tuning.waiting_center);
        throttle_frame_full(348, 0, positions[axis], 1024);
        source(1000); FlightControl_Run();
        assert(!g_flight_control_tuning.waiting_center);
        throttle_frame_full(0x017d, 0, positions[axis], 352);
        assert(g_rc_sbus_data.tuning_rate_rad_s != 0);
        source(1000); FlightControl_Run();
        minimum_while_armed();
        assert(g_flight_control_tuning.active && !g_flight_control_tuning.waiting_center);
        throttle_frame_full(1022, 0, positions[axis], 352);
        source(1000); FlightControl_Run();
        assert(g_flight_control_tuning.active && EscPwm_IsArmed());
        if (axis == RC_TUNING_ROLL)
            near_value(g_flight_control_state.roll_rate_target_rad_s, g_rc_sbus_data.tuning_rate_rad_s);
        else if (axis == RC_TUNING_YAW)
            near_value(g_flight_control_state.yaw_rate_target_rad_s, g_rc_sbus_data.tuning_rate_rad_s);
        else near_value(g_flight_control_state.pitch_rate_target_rad_s, g_rc_sbus_data.tuning_rate_rad_s);
        assert(g_esc_pwm_state.pulse_us[0] != g_esc_pwm_state.pulse_us[1] ||
            g_esc_pwm_state.pulse_us[0] != g_esc_pwm_state.pulse_us[2]);
    }
}

int main(void)
{
    test_low_throttle_overrides_pid();
    test_boundaries_and_resume();
    test_pid_reset_heading_and_watchdog();
    test_good_frame_latch_and_loss();
    test_failsafe_and_imu_priority();
    test_all_tuning_axes();
    puts("PASS throttle stop: raw 0x17d/17e/17f, four CCR minimum, PID/yaw reset, armed resume, good-frame latch, watchdog/failsafe and three tuning axes");
    return 0;
}
