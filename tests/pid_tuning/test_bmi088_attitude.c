/* Production Fusion + wrapper + SPI publisher; no substitute AHRS equations. */
#define main bmi_acquisition_regression_main
#include "test_bmi088.c"
#undef main

static Bmi088Data_t input;
static uint64_t simulated_us;
static uint32_t initial_tick;

static void attitude_reset(uint32_t tick, uint32_t cycle)
{
    Bmi088_Init(HAL_ERROR);
    memset(&input, 0, sizeof(input));
    input.has_sample = input.valid = input.initialized = 1;
    input.temperature_c = 40;
    initial_tick = test_tick = tick; simulated_us = 0; test_dwt.CYCCNT = cycle;
}

/* Independent RFU analytic specific force and body angular velocity. */
static void pose(float roll, float pitch, float roll_rate, float pitch_rate, float yaw_rate)
{
    const float theta = pitch;
    input.accel_m_s2[0] = -9.80665f * sinf(theta);
    input.accel_m_s2[1] = 9.80665f * sinf(roll) * cosf(theta);
    input.accel_m_s2[2] = 9.80665f * cosf(roll) * cosf(theta);
    input.gyro_rad_s[0] = roll_rate;
    input.gyro_rad_s[1] = pitch_rate;
    input.gyro_rad_s[2] = yaw_rate;
}

static void feed(unsigned us)
{
    const Bmi088AttitudeState_t *estimate;
    const Bmi088GyroFilterState_t *filter;
    input.sample_count++;
    simulated_us += us; test_tick = initial_tick + (uint32_t)(simulated_us / 1000U);
    input.sample_tick_ms = test_tick; test_dwt.CYCCNT += us * 480U;
    filter = Bmi088GyroFilter_Compute(&input, test_dwt.CYCCNT);
    estimate = Bmi088Attitude_Compute(&input, filter, test_dwt.CYCCNT);
    taskENTER_CRITICAL();
    g_bmi088_data = input; g_bmi088_gyro_filter = *filter; g_bmi088_attitude_state = *estimate;
    taskEXIT_CRITICAL();
}

static void settle(float roll, float pitch)
{
    unsigned i;
    for (i = 0; i < 120; i++) { pose(roll, pitch, 0, 0, 0); feed(500); }
    assert(g_bmi088_attitude_state.valid && g_bmi088_attitude_state.initialized);
}

static float wrapped_error(float angle, float expected)
{
    return atan2f(sinf(angle - expected), cosf(angle - expected));
}

int main(void)
{
    unsigned axis, sign, i;
    Bmi088AttitudeState_t state, before;
    Bmi088Data_t raw;
    Bmi088GyroFilterState_t filtered;
    /* Zero tick/count/first identity, non-level startup, and inverted startup. */
    attitude_reset(0, 0);
    Bmi088Attitude_GetState(&state); assert(!state.valid && state.age_ms == UINT32_MAX);
    pose(0, 0, 0, 0, 0); feed(500); assert(!g_bmi088_attitude_state.valid);
    settle(.3f, -.2f); /* Mixing initial poses slightly changes the mean: restart. */
    attitude_reset(0, 0); settle(.3f, -.2f);
    assert(fabsf(g_bmi088_attitude_state.roll_rad - .3f) < 2e-5f);
    assert(fabsf(g_bmi088_attitude_state.pitch_rad + .2f) < 2e-5f);
    assert(fabsf(g_bmi088_attitude_state.yaw_rad) < 2e-5f);
    Bmi088_GetSnapshotWithFilter(&raw, &filtered, &state);
    assert(filtered.valid && raw.sample_count == filtered.sample_count && raw.sample_tick_ms == filtered.sample_tick_ms);
    assert(raw.sample_count == state.sample_count && raw.sample_tick_ms == state.sample_tick_ms);
    assert(!memcmp(raw.gyro_rad_s, input.gyro_rad_s, sizeof(input.gyro_rad_s)));
    assert(!memcmp(raw.accel_m_s2, input.accel_m_s2, sizeof(input.accel_m_s2)));
    for (axis = 0; axis < 3; axis++) for (sign = 0; sign < 2; sign++) {
        FusionVector expected, gravity;
        attitude_reset(0, 0); memset(input.accel_m_s2, 0, sizeof(input.accel_m_s2));
        input.accel_m_s2[axis] = sign ? -9.80665f : 9.80665f;
        for (i = 0; i < 120; i++) feed(500);
        assert(g_bmi088_attitude_state.valid);
        expected.axis.x = input.accel_m_s2[0] / 9.80665f;
        expected.axis.y = input.accel_m_s2[1] / 9.80665f;
        expected.axis.z = input.accel_m_s2[2] / 9.80665f;
        gravity = FusionAhrsGetGravity(&s_ahrs);
        assert(FusionVectorNorm(FusionVectorSubtract(gravity, expected)) < 2e-5f);
        assert(fabsf(FusionQuaternionNormSquared(FusionAhrsGetQuaternion(&s_ahrs)) - 1) < 1e-5f);
    }
    /* Independent +/- X roll, Y pitch, Z yaw; variable real dt.
     * Accel is an independent analytic gravity fixture, not the filter's q. */
    for (axis = 0; axis < 3; axis++) for (sign = 0; sign < 2; sign++) {
        float angle = 0, rate = sign ? -.7f : .7f;
        attitude_reset(0, 0); settle(0, 0);
        for (i = 0; i < 2400; i++) {
            unsigned us = i % 3 == 0 ? 250 : i % 3 == 1 ? 500 : 750;
            angle += rate * us * 1e-6f;
            pose(axis == 0 ? angle : 0, axis == 1 ? angle : 0,
                 axis == 0 ? rate : 0, axis == 1 ? rate : 0, axis == 2 ? rate : 0);
            feed(us);
            assert(g_bmi088_attitude_state.valid);
            assert(fabsf(g_bmi088_attitude_state.dt_s - us * 1e-6f) < 1e-8f);
        }
        Bmi088Attitude_GetState(&state);
        /* The 50 Hz second-order LPF adds about 4.5 ms low-frequency delay;
         * the previous unfiltered 0.002 rad tolerance would reject that delay. */
        assert(fabsf(wrapped_error(axis == 0 ? state.roll_rad : axis == 1 ? state.pitch_rad : state.yaw_rad, angle)) < .005f);
        assert(axis == 0 || fabsf(state.roll_rad) < .001f);
        assert(axis == 1 || fabsf(state.pitch_rad) < .001f);
        assert(axis == 2 || fabsf(state.yaw_rad) < .001f);
    }
    /* Coupled Euler motion requires true body rates, NOT Euler derivatives.
     * Independent analytic gravity/rate fixture catches remap handedness. */
    attitude_reset(0, 0); settle(0, 0);
    { float t = 0;
      for (i = 0; i < 8000; i++) {
        const unsigned us = i & 1U ? 750 : 250;
        float phi, theta, psi, pdot, tdot, ydot = .4f;
        t += us * 1e-6f;
        phi = .2f * sinf(2 * t); theta = .15f * sinf(1.3f * t); psi = ydot * t;
        pdot = .4f * cosf(2 * t); tdot = .195f * cosf(1.3f * t);
        pose(phi, theta, 0, 0, 0);
        input.gyro_rad_s[0] = pdot - ydot * sinf(theta);
        input.gyro_rad_s[1] = tdot * cosf(phi) + ydot * sinf(phi) * cosf(theta);
        input.gyro_rad_s[2] = -tdot * sinf(phi) + ydot * cosf(phi) * cosf(theta);
        feed(us);
        assert(g_bmi088_attitude_state.valid);
        assert(fabsf(wrapped_error(g_bmi088_attitude_state.roll_rad, phi)) < .003f);
        assert(fabsf(g_bmi088_attitude_state.pitch_rad - theta) < .003f);
        assert(fabsf(wrapped_error(g_bmi088_attitude_state.yaw_rad, psi)) < .003f);
      }
    }
    /* Yaw wraps at +/-pi; relative rotation remains correct, not clamped. */
    attitude_reset(0, 0); settle(0, 0);
    { double filtered_integral=0;
      for (i = 0; i < 8000; i++) {
        pose(0, 0, 0, 0, 1); feed(500);
        filtered_integral+=g_bmi088_gyro_filter.gyro_rad_s[2]*.0005;
      }
      assert(fabsf(wrapped_error(g_bmi088_attitude_state.yaw_rad,(float)filtered_integral))<.001f); }
    Bmi088Attitude_GetState(&state);
    assert(state.yaw_rad < 0 && fabsf(wrapped_error(state.yaw_rad, 4)) < .006f);
    /* Duplicate count is no update, count rollover is still a fresh sample. */
    before = state; input.gyro_rad_s[1] = 1;
    Bmi088Attitude_Compute(&input, &s_gyro_filter, test_dwt.CYCCNT + 240000U);
    assert(s_attitude.update_count == before.update_count);
    s_count = input.sample_count = UINT32_MAX; pose(0, 0, 0, 0, .5f); feed(500);
    assert(input.sample_count == 0 && g_bmi088_attitude_state.update_count == before.update_count + 1);
    /* Pure gyro prediction during high acceleration/free-fall, angular reject. */
    attitude_reset(0, 0); settle(0, 0);
    pose(0, 0, 0, 0, .5f); input.accel_m_s2[2] = 19.6133f; feed(500);
    assert(g_bmi088_attitude_state.valid && !g_bmi088_attitude_state.accel_used);
    assert(g_bmi088_attitude_state.yaw_rad > 0 && g_bmi088_attitude_state.accel_rejected_count);
    memset(input.accel_m_s2, 0, sizeof(input.accel_m_s2)); feed(500);
    assert(g_bmi088_attitude_state.valid && !g_bmi088_attitude_state.accel_used);
    pose(.5f, 0, 0, 0, 0); feed(500);
    assert(g_bmi088_attitude_state.valid && !g_bmi088_attitude_state.accel_used);
    /* Small gravity correction converges without directly overwriting angles. */
    attitude_reset(0, 0); settle(0, 0);
    for (i = 0; i < 16000; i++) { pose(.08f, 0, 0, 0, 0); feed(500); }
    assert(g_bmi088_attitude_state.roll_rad > .075f && g_bmi088_attitude_state.roll_rad < .081f);
    /* A moving startup must not become a false gravity alignment. */
    attitude_reset(0, 0);
    for (i = 0; i < 200; i++) { pose(0, 0, .5f, 0, 0); feed(500); }
    assert(!g_bmi088_attitude_state.valid && !g_bmi088_attitude_state.epoch);
    settle(.2f, .1f); assert(g_bmi088_attitude_state.epoch == 1);
    /* 19 ms usable, >=20 ms invalid; recover into a new relative-Yaw epoch. */
    test_tick += 19; Bmi088Attitude_GetState(&state); assert(state.valid && state.age_ms == 19);
    test_tick++; Bmi088Attitude_GetState(&state); assert(!state.valid && state.age_ms == 20);
    pose(-.2f, -.1f, 0, 0, 0); feed(20000); assert(!g_bmi088_attitude_state.valid);
    settle(-.2f, -.1f); assert(g_bmi088_attitude_state.valid && g_bmi088_attitude_state.epoch == 2);
    assert(fabsf(g_bmi088_attitude_state.yaw_rad) < 1e-6f);
    Bmi088Attitude_Invalidate(); input.valid = 0; feed(500); assert(!g_bmi088_attitude_state.valid);
    input.valid = 1; settle(0, 0); assert(g_bmi088_attitude_state.epoch == 3);
    /* NaN/Inf/saturation never contaminate last good quaternion or raw sample. */
    { const float bad[] = {NAN, INFINITY, -INFINITY, 40};
      for (i = 0; i < 4; i++) {
        attitude_reset(0, 0); settle(0, 0); input.gyro_rad_s[0] = bad[i]; feed(500);
        assert(!g_bmi088_attitude_state.valid && g_bmi088_attitude_state.rejected_count == 1);
        assert(g_bmi088_attitude_state.error == (i == 3 ? BMI088_ATTITUDE_GYRO_OVERRANGE : BMI088_ATTITUDE_BAD_INPUT));
        assert(isfinite(g_bmi088_attitude_state.quaternion[0]));
        settle(0, 0); assert(g_bmi088_attitude_state.valid);
      }
    }
    attitude_reset(0, 0); settle(0, 0); input.accel_m_s2[2] = NAN; feed(500);
    assert(!g_bmi088_attitude_state.valid && g_bmi088_attitude_state.error == BMI088_ATTITUDE_BAD_INPUT);
    /* Full-scale handling is independent of the unchanged gravity trust gate.
     * 5g was rejected by the old fixed 4g bound; now it is gyro-only motion. */
    attitude_reset(0,0);settle(0,0);
    input.accel_m_s2[2]=5.0f*9.80665f;feed(500);
    assert(g_bmi088_attitude_state.valid && !g_bmi088_attitude_state.accel_used);
    input.accel_m_s2[2]=-5.0f*9.80665f;feed(500);
    assert(g_bmi088_attitude_state.valid && !g_bmi088_attitude_state.accel_used);
    input.accel_clipped=1;feed(500);
    assert(!g_bmi088_attitude_state.valid && g_bmi088_attitude_state.error==BMI088_ATTITUDE_ACCEL_OVERRANGE);
    input.accel_clipped=0;settle(0,0);
    input.accel_m_s2[1]=7.0f*9.80665f;feed(500);
    assert(!g_bmi088_attitude_state.valid && g_bmi088_attitude_state.error==BMI088_ATTITUDE_ACCEL_OVERRANGE);
    /* Numerical fault / zero dt / long DWT wrap interval are not extrapolated. */
    attitude_reset(0, 0); settle(0, 0); s_ahrs.quaternion.array[0] = NAN; feed(500);
    assert(!g_bmi088_attitude_state.valid && g_bmi088_attitude_state.error == BMI088_ATTITUDE_NUMERICAL_ERROR);
    settle(0, 0); feed(0); assert(!g_bmi088_attitude_state.valid);
    settle(0, 0); feed(10000000); assert(!g_bmi088_attitude_state.valid);
    /* Both DWT and HAL time wrap while integration proceeds at 500 us. */
    attitude_reset(UINT32_MAX - 30U, UINT32_MAX - 100U); settle(0, 0);
    pose(0, 0, 0, 0, .5f); feed(500); Bmi088Attitude_GetState(&state);
    assert(state.valid && state.age_ms == 0 && fabsf(state.dt_s - .0005f) < 1e-8f);
    /* Real SPI acquisition publishes corrected motion and matching estimate.
     * It must not bypass calibration; the control frame is native RFU. */
    model_reset(100); ready(); measurement(136);
    for (axis = 0; axis < 3; axis++) {
        put_i16(0, 2 + 2 * axis, (int)roundf(g_bmi088_calibration.gyro_bias_rad_s[axis] / BMI088_GYRO_SCALE));
        put_i16(1, 0x12 + 2 * axis, (int)roundf((g_bmi088_calibration.accel_bias_m_s2[axis] +
            (axis == 2 ? 9.80665f / g_bmi088_calibration.accel_scale[axis] : 0)) / BMI088_ACCEL_SCALE));
    }
    for (i = 0; i < 120; i++) { test_dwt.CYCCNT += 240000U; test_tick += i & 1U; event_process(); }
    Bmi088_GetSnapshot(&raw, &state);
    assert(raw.valid && state.valid && raw.sample_count == state.sample_count && raw.sample_tick_ms == state.sample_tick_ms);
    Bmi088_GetSnapshotWithFilter(&raw, &filtered, &state);
    assert(filtered.valid && filtered.sample_count == raw.sample_count && filtered.sample_tick_ms == raw.sample_tick_ms);
    assert(raw.calibration_version == BMI088_CALIBRATION_VERSION && fabsf(state.roll_rad) < .002f && fabsf(state.pitch_rad) < .002f);
    fail_call = test_spi_calls + 1; fail_status = HAL_ERROR; event_process();
    Bmi088_GetSnapshot(&raw, &state); assert(!raw.valid && !state.valid);
    assert(test_critical_depth == 0);
    puts("PASS BMI088 AHRS: 6-face/startup, calibrated atomic pairs, +/- axes, real dt, gyro-only/gating, wraps, stale/error recovery and unchanged raw data");
    return 0;
}
