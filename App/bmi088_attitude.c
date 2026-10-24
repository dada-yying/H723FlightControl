/* BMI088 flight-control six-axis AHRS. The pinned MIT Fusion implementation
 * and its license are in ThirdParty/Fusion. No ATOM/PID/motor dependency. */
#include "bmi088_attitude.h"
#include "../ThirdParty/Fusion/FusionAhrs.h"
#include <float.h>
#include <string.h>

#define BMI_ATTITUDE_GRAVITY 9.80665f
#define BMI_ATTITUDE_ALIGN_MS 50U
#define BMI_ATTITUDE_ALIGN_SAMPLES 64U
#define BMI_ATTITUDE_ALIGN_RATE_RAD_S 0.1745329252f /* 10 deg/s */

volatile Bmi088AttitudeState_t g_bmi088_attitude_state;
static Bmi088AttitudeState_t s_attitude;
static FusionAhrs s_ahrs;
static uint32_t s_cycle, s_tick, s_count, s_align_tick, s_align_samples;
static uint8_t s_seen;
static FusionVector s_align_sum;
static const FusionAhrsSettings s_settings = {
    .sampleRate = 2000.0f, .convention = FusionConventionEnu,
    .gain = 0.5f,
    /* Saturation is rejected by this wrapper, not Fusion's startup/Yaw reset. */
    .gyroscopeRange = 0.0f, .accelerationRejection = 10.0f,
    .magneticRejection = 0.0f, .rejectionTimeout = 5.0f,
};

static int BmiAttitude_Finite(float value)
{
    return value <= FLT_MAX && value >= -FLT_MAX;
}

static void BmiAttitude_Reset(uint8_t reason)
{
    if (s_attitude.initialized) s_attitude.reset_count++;
    s_attitude.initialized = s_attitude.valid = s_attitude.accel_used = 0;
    s_attitude.error = reason;
    s_attitude.dt_s = 0;
    s_align_samples = 0;
    s_align_sum = FUSION_VECTOR_ZERO;
    /* Retain last angles/epoch for diagnostics, explicitly invalid. */
}

void Bmi088Attitude_Init(void)
{
    memset(&s_attitude, 0, sizeof(s_attitude));
    s_attitude.quaternion[0] = 1;
    s_attitude.age_ms = UINT32_MAX;
    s_attitude.error = BMI088_ATTITUDE_WAIT_SOURCE;
    s_seen = 0; s_align_samples = 0; s_align_sum = FUSION_VECTOR_ZERO;
    g_bmi088_attitude_state = s_attitude;
}

void Bmi088Attitude_Invalidate(void)
{
    BmiAttitude_Reset(BMI088_ATTITUDE_SOURCE_INVALID);
    s_seen = 0;
    taskENTER_CRITICAL();
    g_bmi088_attitude_state = s_attitude;
    taskEXIT_CRITICAL();
}

static int BmiAttitude_Extract(void)
{
    FusionQuaternion q = FusionAhrsGetQuaternion(&s_ahrs);
    FusionEuler e;
    unsigned i;
    for (i = 0; i < 4; i++) if (!BmiAttitude_Finite(q.array[i])) return 0;
    if (fabsf(FusionQuaternionNormSquared(q) - 1.0f) > 0.01f) return 0;
    e = FusionQuaternionToEuler(q);
    for (i = 0; i < 3; i++) if (!BmiAttitude_Finite(e.array[i])) return 0;
    s_attitude.roll_rad = FusionDegreesToRadians(e.angle.roll);
    s_attitude.pitch_rad = FusionDegreesToRadians(e.angle.pitch);
    s_attitude.yaw_rad = FusionDegreesToRadians(e.angle.yaw);
    memcpy(s_attitude.quaternion, q.array, sizeof(q.array));
    return 1;
}

static void BmiAttitude_Align(FusionVector acceleration)
{
    /* RFU body / relative ENU world: level specific force = (0,0,+g).
     * Heading is unobservable and starts at zero. */
    const float roll = atan2f(acceleration.axis.y, acceleration.axis.z);
    const float pitch = atan2f(-acceleration.axis.x,
        sqrtf(acceleration.axis.y * acceleration.axis.y + acceleration.axis.z * acceleration.axis.z));
    const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    FusionQuaternion q = {.array = {cr * cp, sr * cp, cr * sp, -sr * sp}};
    FusionAhrsInitialise(&s_ahrs);
    FusionAhrsSetSettings(&s_ahrs, &s_settings);
    FusionAhrsSetQuaternion(&s_ahrs, FusionQuaternionNormalise(q));
    /* Gravity-aligned already: never zero Yaw on every startup update. */
    FusionAhrsSkipStartup(&s_ahrs);
    s_attitude.initialized = s_attitude.valid = s_attitude.accel_used = 1;
    s_attitude.epoch++;
    s_attitude.error = BMI088_ATTITUDE_OK;
}

const Bmi088AttitudeState_t *Bmi088Attitude_Compute(const Bmi088Data_t *sample,
                                                  const Bmi088GyroFilterState_t *filter,
                                                  uint32_t completion_cycle)
{
    FusionVector accel, gyro;
    uint32_t elapsed_ms, cycles;
    float dt;
    unsigned i;
    if (!sample || !sample->valid || !sample->initialized || !sample->has_sample) {
        BmiAttitude_Reset(BMI088_ATTITUDE_SOURCE_INVALID); s_seen = 0;
        return &s_attitude;
    }
    /* Never integrate the same published sample twice (count zero is valid). */
    if (s_seen && sample->sample_count == s_count) {
        if (!filter || !filter->valid || !filter->initialized ||
            filter->sample_count != sample->sample_count || filter->sample_tick_ms != sample->sample_tick_ms) {
            BmiAttitude_Reset(BMI088_ATTITUDE_SOURCE_INVALID); s_seen = 0;
        }
        return &s_attitude;
    }
    elapsed_ms = (uint32_t)(sample->sample_tick_ms - s_tick);
    cycles = (uint32_t)(completion_cycle - s_cycle);
    dt = s_seen && SystemCoreClock ? (float)cycles / (float)SystemCoreClock : 0;
    if (s_seen && (elapsed_ms >= BMI088_SAMPLE_TIMEOUT_MS || dt >= 0.020f))
        BmiAttitude_Reset(BMI088_ATTITUDE_TIME_GAP);
    s_seen = 1; s_cycle = completion_cycle; s_tick = sample->sample_tick_ms; s_count = sample->sample_count;
    for (i = 0; i < 3; i++) {
        if (!BmiAttitude_Finite(sample->gyro_rad_s[i]) || !BmiAttitude_Finite(sample->accel_m_s2[i]) ||
            fabsf(sample->accel_m_s2[i]) > 4.0f * BMI_ATTITUDE_GRAVITY) {
            s_attitude.rejected_count++; BmiAttitude_Reset(BMI088_ATTITUDE_BAD_INPUT);
            return &s_attitude;
        }
        if (fabsf(sample->gyro_rad_s[i]) >= BMI088_GYRO_VALID_LIMIT_RAD_S) {
            s_attitude.rejected_count++; BmiAttitude_Reset(BMI088_ATTITUDE_GYRO_OVERRANGE);
            return &s_attitude;
        }
    }
    if (!filter || !filter->valid || !filter->initialized ||
        filter->sample_count != sample->sample_count || filter->sample_tick_ms != sample->sample_tick_ms) {
        BmiAttitude_Reset(BMI088_ATTITUDE_SOURCE_INVALID); s_seen = 0;
        return &s_attitude;
    }
    for (i = 0; i < 3; i++) if (!BmiAttitude_Finite(filter->gyro_rad_s[i])) {
        s_attitude.rejected_count++; BmiAttitude_Reset(BMI088_ATTITUDE_BAD_INPUT);
        return &s_attitude;
    }
    /* Native X=right, Y=forward, Z=up is already the right-handed control frame. */
    for (i = 0; i < 3; i++) {
        accel.array[i] = sample->accel_m_s2[i];
        gyro.array[i] = filter->gyro_rad_s[i];
    }
    s_attitude.accel_norm_m_s2 = FusionVectorNorm(accel);
    if (!s_attitude.initialized) {
        if (s_attitude.accel_norm_m_s2 < 0.8f * BMI_ATTITUDE_GRAVITY ||
            s_attitude.accel_norm_m_s2 > 1.2f * BMI_ATTITUDE_GRAVITY ||
            fabsf(sample->gyro_rad_s[0]) > BMI_ATTITUDE_ALIGN_RATE_RAD_S ||
            fabsf(sample->gyro_rad_s[1]) > BMI_ATTITUDE_ALIGN_RATE_RAD_S ||
            fabsf(sample->gyro_rad_s[2]) > BMI_ATTITUDE_ALIGN_RATE_RAD_S) {
            s_align_samples = 0; s_align_sum = FUSION_VECTOR_ZERO;
            s_attitude.error = BMI088_ATTITUDE_ALIGNING;
            return &s_attitude;
        }
        if (!s_align_samples) s_align_tick = sample->sample_tick_ms;
        s_align_samples++; s_align_sum = FusionVectorAdd(s_align_sum, accel);
        s_attitude.error = BMI088_ATTITUDE_ALIGNING;
        if (s_align_samples < BMI_ATTITUDE_ALIGN_SAMPLES ||
            (uint32_t)(sample->sample_tick_ms - s_align_tick) < BMI_ATTITUDE_ALIGN_MS) return &s_attitude;
        accel = FusionVectorScale(s_align_sum, 1.0f / (float)s_align_samples);
        /* Opposite gravity directions must not average into a zero vector. */
        if (FusionVectorNorm(accel) < 0.8f * BMI_ATTITUDE_GRAVITY) {
            s_align_samples = 0; s_align_sum = FUSION_VECTOR_ZERO; return &s_attitude;
        }
        BmiAttitude_Align(accel);
        s_attitude.dt_s = 0;
    } else {
        if (!BmiAttitude_Finite(dt) || dt <= 0) {
            s_attitude.rejected_count++; BmiAttitude_Reset(BMI088_ATTITUDE_TIME_GAP);
            return &s_attitude;
        }
        /* Additional magnitude gate: gross linear acceleration/free-fall must
         * not become gravity correction. This cannot detect all translations. */
        if (s_attitude.accel_norm_m_s2 < 0.8f * BMI_ATTITUDE_GRAVITY ||
            s_attitude.accel_norm_m_s2 > 1.2f * BMI_ATTITUDE_GRAVITY) accel = FUSION_VECTOR_ZERO;
        gyro = FusionVectorScale(gyro, 57.295779513f); /* Fusion expects deg/s. */
        FusionAhrsSetSamplePeriod(&s_ahrs, dt);
        FusionAhrsUpdateNoMagnetometer(&s_ahrs, gyro, FusionVectorScale(accel, 1.0f / BMI_ATTITUDE_GRAVITY));
        s_attitude.dt_s = dt;
        s_attitude.accel_used = (uint8_t)!s_ahrs.accelerometerIgnored;
        if (!s_attitude.accel_used) s_attitude.accel_rejected_count++;
        s_attitude.valid = 1; s_attitude.error = BMI088_ATTITUDE_OK;
    }
    if (!BmiAttitude_Extract()) {
        s_attitude.rejected_count++; BmiAttitude_Reset(BMI088_ATTITUDE_NUMERICAL_ERROR);
        return &s_attitude;
    }
    s_attitude.sample_count = sample->sample_count; s_attitude.sample_tick_ms = sample->sample_tick_ms;
    s_attitude.age_ms = 0; s_attitude.update_count++;
    return &s_attitude;
}

void Bmi088Attitude_GetState(Bmi088AttitudeState_t *state)
{
    Bmi088_GetSnapshot(NULL, state);
}
