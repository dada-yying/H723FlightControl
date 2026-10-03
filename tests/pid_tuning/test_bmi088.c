#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../App/bmi088.c"

static uint8_t registers[2][128], accel_spi_mode;
static unsigned fail_call;
static HAL_StatusTypeDef fail_status;
static int bad_config;
static uint32_t reset_tick[2], write_tick[2];
static uint8_t last_written[2];

/* Emulates both physical SPI protocols with distinct discarded bytes. This
 * catches accel's extra dummy-byte handling, burst offsets and CS mistakes. */
static HAL_StatusTypeDef sensor_spi(const uint8_t *tx, uint8_t *rx,
                                   uint16_t size, uint32_t timeout)
{
    unsigned accel = test_spi_cs == BMI088_ACC_CS_Pin;
    unsigned prefix = accel ? 2U : 1U, reg = tx[0] & 0x7FU, i;
    assert(test_critical_depth == 0 && timeout == 1);
    assert(test_spi_cs == BMI088_ACC_CS_Pin || test_spi_cs == BMI088_GYRO_CS_Pin);
    if (test_spi_calls == fail_call) { hspi2.ErrorCode = 0x20; return fail_status; }
    memset(rx, 0xDA, size);
    if (tx[0] & 0x80U) {
        assert(size > prefix && size <= 10);
        if (last_written[accel] == reg) assert((uint32_t)(test_tick - write_tick[accel]) >= 1U);
        for (i = prefix; i < size; i++) rx[i] = registers[accel][reg + i - prefix];
        if (accel && !accel_spi_mode) { memset(rx + prefix, 0, size - prefix); accel_spi_mode = 1; }
        if (bad_config && reg == (accel ? 0x40U : 0x0FU)) rx[prefix] ^= 1;
    } else {
        assert(size == 2);
        if (reg == (accel ? 0x7EU : 0x14U)) {
            assert(tx[1] == 0xB6); reset_tick[accel] = test_tick;
            if (accel) accel_spi_mode = 0;
        } else {
            assert((uint32_t)(test_tick - reset_tick[accel]) >= 80U);
            registers[accel][reg] = tx[1];
        }
        last_written[accel] = (uint8_t)reg; write_tick[accel] = test_tick;
    }
    hspi2.ErrorCode = 0;
    return HAL_OK;
}

static void model_reset(uint32_t tick)
{
    memset(registers, 0, sizeof(registers));
    registers[1][0] = 0x1E; registers[0][0] = 0x0F;
    registers[0][0x10] = 0x80;
    memset(last_written, 0xFF, sizeof(last_written));
    test_tick = tick; test_spi_cs = 0; test_spi_calls = test_spi_cs_release_count = 0;
    fail_call = 0; bad_config = 0; accel_spi_mode = 0;
    test_spi_hook = sensor_spi;
    Bmi088_Init(HAL_OK);
}

static void ready(void)
{
    unsigned n;
    for (n = 0; n < 400 && g_bmi088_stats.stage != BMI088_RUNNING; n++) {
        Bmi088_Process(); test_tick++;
    }
    assert(g_bmi088_stats.stage == BMI088_RUNNING);
    assert(g_bmi088_stats.init_success_count && g_bmi088_data.initialized);
    assert(!test_spi_cs && test_spi_calls == test_spi_cs_release_count);
    assert(registers[1][0x40] == 0xAB && registers[0][0x10] == 0x82);
}

static void put_i16(unsigned accel, unsigned reg, int value)
{
    registers[accel][reg] = (uint8_t)value;
    registers[accel][reg + 1] = (uint8_t)((uint16_t)value >> 8);
}

static void measurement(int temp)
{
    static const int gyro[] = {-32768, 1234, 32767};
    static const int acc[] = {1000, -2000, 8192};
    unsigned i;
    for (i = 0; i < 3; i++) {
        put_i16(0, 2 + 2*i, gyro[i]); put_i16(1, 0x12 + 2*i, acc[i]);
    }
    registers[1][0x22] = (uint8_t)((unsigned)(temp & 2047) >> 3);
    registers[1][0x23] = (uint8_t)((temp & 7) << 5);
}

static float expected_gyro(int counts, unsigned axis, float model_temperature)
{
    float nominal = counts * BMI088_GYRO_SCALE;
    float bias = g_bmi088_calibration.gyro_bias_rad_s[axis] +
        g_bmi088_calibration.gyro_temperature_rad_s_c[axis] * (model_temperature - 40.0f);
    return nominal - bias;
}

static float expected_accel(int counts, unsigned axis)
{
    return (counts * BMI088_ACCEL_SCALE - g_bmi088_calibration.accel_bias_m_s2[axis]) *
        g_bmi088_calibration.accel_scale[axis];
}

int main(void)
{
    Bmi088Data_t data, old;
    unsigned status, calls, n;
    model_reset(0); assert(!g_bmi088_data.has_sample && g_bmi088_data.age_ms == UINT32_MAX);
    assert(!g_bmi088_data.calibration_version && g_bmi088_data.gyro_rad_s[0] == 0);
    Bmi088_Process(); calls = test_spi_calls;
    for (test_tick = 1; test_tick < 80; test_tick++) Bmi088_Process();
    assert(test_spi_calls == calls); /* Reset wait neither spins nor uses SPI. */
    ready(); assert(!g_bmi088_data.has_sample && !g_bmi088_data.valid);
    measurement(-80); Bmi088_Process(); Bmi088_GetData(&data);
    assert(data.has_sample && data.valid && data.age_ms == 0 && data.sample_count == 1);
    assert(data.temperature_raw == -80 && data.temperature_c == 13);
    assert(data.calibration_version == BMI088_CALIBRATION_VERSION);
    assert(!data.compensation_in_range && data.compensation_temperature_c == 22.625f);
    for (n = 0; n < 3; n++) {
        assert(data.gyro_rad_s[n] == expected_gyro(n==0?-32768:n==1?1234:32767,n,22.625f));
        assert(data.accel_m_s2[n] == expected_accel(n==0?1000:n==1?-2000:8192,n));
    }
    old = data; Bmi088_GetData(&data);
    assert(!memcmp(old.gyro_rad_s, data.gyro_rad_s, 12));
    assert(!memcmp(old.accel_m_s2, data.accel_m_s2, 12));
    calls = test_spi_calls; Bmi088_Process(); assert(test_spi_calls == calls);
    /* Canonical fields are already corrected; no body rotation/filtering. */
    test_tick++; measurement(80); put_i16(0, 2, 100); Bmi088_Process(); Bmi088_GetData(&data);
    assert(data.gyro_rad_s[0] == expected_gyro(100,0,33));
    assert(data.temperature_c == 33 && data.sample_count == 2);
    assert(data.compensation_in_range && data.compensation_temperature_c == 33);
    test_tick += 19; Bmi088_GetData(&data); assert(data.valid && data.age_ms == 19);
    test_tick++; Bmi088_GetData(&data); assert(!data.valid && data.age_ms == 20);
    Bmi088_Process(); Bmi088_GetData(&data); assert(data.valid && data.sample_count == 3);

    /* Every HAL failure releases CS and publishes none of a partial read. */
    for (status = HAL_ERROR; status <= HAL_TIMEOUT; status++) {
        model_reset(100); ready(); measurement(0); Bmi088_Process(); Bmi088_GetData(&old);
        put_i16(1, 0x12, 9999); test_tick++;
        fail_call = test_spi_calls + 3; fail_status = (HAL_StatusTypeDef)status;
        Bmi088_Process(); Bmi088_GetData(&data);
        assert(!test_spi_cs && test_spi_calls == test_spi_cs_release_count);
        assert(!data.valid && !data.initialized && data.has_sample);
        assert(data.error == BMI088_ERROR_SPI && data.sample_count == old.sample_count);
        assert(!memcmp(data.accel_m_s2, old.accel_m_s2, 12));
        assert(!memcmp(data.gyro_rad_s, old.gyro_rad_s, 12));
        assert(data.calibration_version == old.calibration_version);
        assert(g_bmi088_stats.read_error_count == 1 && g_bmi088_stats.spi_error_count == 1);
        calls = test_spi_calls; test_tick += 999; Bmi088_Process(); assert(test_spi_calls == calls);
        fail_call = 0; test_tick++; ready(); Bmi088_Process(); Bmi088_GetData(&data);
        assert(data.valid && data.accel_m_s2[0] == expected_accel(9999,0) && data.sample_count == old.sample_count + 1);
    }
    model_reset(100); registers[1][0] = 0; Bmi088_Process();
    assert(g_bmi088_data.error == BMI088_ERROR_ACCEL_ID && g_bmi088_stats.stage == BMI088_RETRY_WAIT);
    model_reset(100); bad_config = 1;
    for (n = 0; n < 200 && g_bmi088_stats.stage != BMI088_RETRY_WAIT; n++) { Bmi088_Process(); test_tick++; }
    assert(g_bmi088_data.error == BMI088_ERROR_ACCEL_CONFIG && !g_bmi088_data.initialized);
    model_reset(100); ready(); measurement(0); registers[0][0] = 0; Bmi088_Process();
    assert(g_bmi088_data.error == BMI088_ERROR_GYRO_ID && !g_bmi088_data.has_sample);
    Bmi088_Init(HAL_ERROR); calls = test_spi_calls; Bmi088_Process(); assert(test_spi_calls == calls);
    assert(g_bmi088_stats.stage == BMI088_DISABLED && !g_bmi088_data.valid);
    Bmi088_OnTaskCreationFailure(); assert(g_bmi088_data.error == BMI088_ERROR_TASK);
    model_reset(UINT32_MAX - 20U); ready(); measurement(-1024); Bmi088_Process(); Bmi088_GetData(&data);
    assert(data.temperature_raw == -1024 && data.temperature_c == -105);
    test_tick = UINT32_MAX - 2U; g_bmi088_data.sample_tick_ms = test_tick;
    test_tick = 1U; Bmi088_GetData(&data); assert(data.age_ms == 4 && data.valid);
    g_bmi088_data.sample_count = UINT32_MAX; s_due_tick = test_tick; Bmi088_Process();
    assert(g_bmi088_data.has_sample && g_bmi088_data.sample_count == 0);
    assert(test_critical_depth == 0);
    /* Independent +/- XYZ gravity fixtures at the 40 C reference. */
    for (n = 0; n < 6; n++) {
        unsigned axis;
        const unsigned gravity_axis = n / 2;
        const float gravity = (n & 1U) ? -9.80665f : 9.80665f;
        memset(&data, 0, sizeof(data));
        data.temperature_c = 40;
        for (axis = 0; axis < 3; axis++) {
            data.gyro_rad_s[axis] = g_bmi088_calibration.gyro_bias_rad_s[axis];
            data.accel_m_s2[axis] = g_bmi088_calibration.accel_bias_m_s2[axis] +
                (axis == gravity_axis ? gravity / g_bmi088_calibration.accel_scale[axis] : 0);
        }
        old = data; Bmi088_ApplyCalibration(&data);
        for (axis = 0; axis < 3; axis++) {
            assert(data.gyro_rad_s[axis] == 0);
            assert(fabs(data.accel_m_s2[axis] -
                (axis == gravity_axis ? gravity : 0)) < 2e-6);
        }
    }
    data.gyro_rad_s[0] = g_bmi088_calibration.gyro_bias_rad_s[0] + .3f;
    data.gyro_rad_s[1] = g_bmi088_calibration.gyro_bias_rad_s[1] - .4f;
    Bmi088_ApplyCalibration(&data);
    assert(fabs(data.gyro_rad_s[0] - .3f) < 1e-7);
    assert(fabs(data.gyro_rad_s[1] + .4f) < 1e-7);
    /* Exact zero-rate model at cold/intermediate/hot temperatures, independent
       axes, no start-up zeroing, no requirement to reach 40 C first. */
    for (n=0;n<5;n++) {
        unsigned axis;
        static const float temperatures[] = {22.625f,25,30,35,40.25f};
        memset(&data,0,sizeof(data));data.temperature_c=temperatures[n];
        for(axis=0;axis<3;axis++)
            data.gyro_rad_s[axis]=g_bmi088_calibration.gyro_bias_rad_s[axis]+
                g_bmi088_calibration.gyro_temperature_rad_s_c[axis]*(data.temperature_c-40);
        Bmi088_ApplyCalibration(&data);
        assert(data.compensation_in_range && data.compensation_temperature_c==temperatures[n]);
        for(axis=0;axis<3;axis++)assert(fabs(data.gyro_rad_s[axis])<1e-8);
    }
    { unsigned axis;static const float bad[] = {-105,85,NAN,INFINITY,-INFINITY};
      for(n=0;n<5;n++){
        memset(&data,0,sizeof(data));data.temperature_c=bad[n];Bmi088_ApplyCalibration(&data);
        assert(!data.compensation_in_range && isfinite(data.compensation_temperature_c));
        for(axis=0;axis<3;axis++)assert(isfinite(data.gyro_rad_s[axis]));
      }
    }
    puts("PASS BMI088 SPI/init/recovery, canonical calibrated gyro/accel, warm-up compensation/bounds, single application, 6-face gravity, tick wrap");
    return 0;
}
