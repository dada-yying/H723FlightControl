/* Based on Damiao CtrBoard-H7_IMU BMI088driver/Middleware and the Bosch
 * BMI088 SPI protocol. Acquisition/logging only; no flight-controller input. */
#include "bmi088.h"
#include "bmi088_calibration.h"
#include "spi.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define BMI088_SPI_TIMEOUT_MS 1U
#define BMI088_RESET_WAIT_MS 80U
#define BMI088_RETRY_MS 1000U
#define BMI088_ACCEL_SCALE 0.0008974358974f
#define BMI088_GYRO_SCALE 0.0010652644360316953f

/* Same ranges/ODR/bandwidth and interrupt registers as the board example:
 * accel +/-3g, 800 Hz; gyro +/-2000 deg/s, 1000 Hz/116 Hz bandwidth.
 * Sensor interrupts are configured, but MCU EXTI/DMA are not used. */
static const uint8_t s_accel_config[][2] = {
    {0x7D, 0x04}, {0x7C, 0x00}, {0x40, 0xAB},
    {0x41, 0x00}, {0x53, 0x08}, {0x58, 0x04}
};
static const uint8_t s_gyro_config[][2] = {
    {0x0F, 0x00}, {0x10, 0x82}, {0x11, 0x00},
    {0x15, 0x80}, {0x16, 0x00}, {0x18, 0x01}
};

volatile Bmi088Data_t g_bmi088_data;
volatile Bmi088Stats_t g_bmi088_stats;
const Bmi088Calibration_t g_bmi088_calibration = {
    BMI088_CALIBRATION_GYRO_BIAS_RAD_S,
    BMI088_CALIBRATION_ACCEL_BIAS_M_S2,
    BMI088_CALIBRATION_ACCEL_SCALE,
    BMI088_CALIBRATION_GYRO_TEMP_RAD_S_C,
    BMI088_CALIBRATION_REFERENCE_C,
    BMI088_CALIBRATION_MIN_C,
    BMI088_CALIBRATION_MAX_C,
    BMI088_CALIBRATION_VERSION
};
static uint32_t s_due_tick, s_last_task_tick;
static uint8_t s_config_index, s_config_verify, s_task_seen;

static HAL_StatusTypeDef Bmi088_Transfer(uint8_t accel, uint8_t *tx,
                                       uint8_t *rx, uint16_t size)
{
    HAL_StatusTypeDef status;
    uint16_t pin = accel ? BMI088_ACC_CS_Pin : BMI088_GYRO_CS_Pin;
    HAL_GPIO_WritePin(GPIOC, pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, size, BMI088_SPI_TIMEOUT_MS);
    /* Always release CS, including HAL_BUSY/timeout/error. No retry in a
     * transfer and no critical section around polling or sensor delays. */
    HAL_GPIO_WritePin(GPIOC, pin, GPIO_PIN_SET);
    g_bmi088_stats.last_status = status;
    g_bmi088_stats.last_spi_error = hspi2.ErrorCode;
    if (status != HAL_OK) g_bmi088_stats.spi_error_count++;
    return status;
}

static HAL_StatusTypeDef Bmi088_Read(uint8_t accel, uint8_t reg,
                                   uint8_t *data, uint8_t size)
{
    uint8_t tx[10] = {0}, rx[10] = {0};
    const uint8_t prefix = accel ? 2U : 1U;
    HAL_StatusTypeDef status;
    if (!size || size + prefix > sizeof(tx)) return HAL_ERROR;
    tx[0] = reg | 0x80U;
    /* Accel: discard command response AND one dummy byte. Gyro: discard
     * only command response. One CS-low burst covers the entire transfer. */
    status = Bmi088_Transfer(accel, tx, rx, (uint16_t)(size + prefix));
    if (status == HAL_OK) memcpy(data, rx + prefix, size);
    return status;
}

static HAL_StatusTypeDef Bmi088_Write(uint8_t accel, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value}, rx[2] = {0};
    return Bmi088_Transfer(accel, tx, rx, sizeof(tx));
}

static void Bmi088_Fail(Bmi088Error_t error)
{
    if (g_bmi088_stats.stage == BMI088_RUNNING) g_bmi088_stats.read_error_count++;
    else g_bmi088_stats.init_error_count++;
    taskENTER_CRITICAL();
    /* Keep the last successful measurements and timestamp for diagnostics. */
    g_bmi088_data.valid = 0;
    g_bmi088_data.initialized = 0;
    g_bmi088_data.error = (uint8_t)error;
    taskEXIT_CRITICAL();
    g_bmi088_stats.stage = BMI088_RETRY_WAIT;
    s_due_tick = HAL_GetTick() + BMI088_RETRY_MS;
}

static int Bmi088_CheckId(uint8_t accel)
{
    uint8_t id;
    /* The first accel CS rising edge selects SPI; the first result after
     * power-on/reset may be discarded, as in the board example. */
    if (Bmi088_Read(accel, 0x00, &id, 1) != HAL_OK ||
        Bmi088_Read(accel, 0x00, &id, 1) != HAL_OK) {
        Bmi088_Fail(BMI088_ERROR_SPI); return 0;
    }
    if (accel) g_bmi088_stats.accel_chip_id = id;
    else g_bmi088_stats.gyro_chip_id = id;
    if (id != (accel ? 0x1EU : 0x0FU)) {
        Bmi088_Fail(accel ? BMI088_ERROR_ACCEL_ID : BMI088_ERROR_GYRO_ID);
        return 0;
    }
    return 1;
}

static void Bmi088_Configure(uint8_t accel)
{
    const uint8_t (*config)[2] = accel ? s_accel_config : s_gyro_config;
    uint8_t reg = config[s_config_index][0], value;
    if (!s_config_verify) {
        if (Bmi088_Write(accel, reg, config[s_config_index][1]) != HAL_OK) {
            Bmi088_Fail(BMI088_ERROR_SPI); return;
        }
        s_config_verify = 1;
    } else {
        if (Bmi088_Read(accel, reg, &value, 1) != HAL_OK) {
            Bmi088_Fail(BMI088_ERROR_SPI); return;
        }
        g_bmi088_stats.last_config_register = reg;
        g_bmi088_stats.last_config_value = value;
        if (value != config[s_config_index][1]) {
            Bmi088_Fail(accel ? BMI088_ERROR_ACCEL_CONFIG : BMI088_ERROR_GYRO_CONFIG);
            return;
        }
        s_config_verify = 0;
        if (++s_config_index == 6U) {
            s_config_index = 0;
            if (accel) g_bmi088_stats.stage = BMI088_RESET_GYRO;
            else {
                g_bmi088_stats.init_success_count++;
                g_bmi088_stats.stage = BMI088_RUNNING;
                taskENTER_CRITICAL();
                g_bmi088_data.initialized = 1;
                g_bmi088_data.error = BMI088_ERROR_NONE;
                taskEXIT_CRITICAL();
            }
        }
    }
    /* At least 1 ms between write/read/next write (example needs 150 us).
     * Waiting is cooperative: Flight/Rc tasks remain independently runnable. */
    s_due_tick = HAL_GetTick() + 1U;
}

static int16_t Bmi088_I16(const uint8_t *p)
{
    const uint16_t value = (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
    return (int16_t)((value & 0x8000U) ? (int32_t)value - 65536 : value);
}

static void Bmi088_ApplyCalibration(Bmi088Data_t *sample)
{
    unsigned axis;
    float temperature = sample->temperature_c, delta;
    sample->compensation_in_range = (uint8_t)(temperature >= g_bmi088_calibration.minimum_temperature_c &&
        temperature <= g_bmi088_calibration.maximum_temperature_c);
    /* Do not extrapolate beyond the observed temperature envelope. Measured
     * temperature stays unchanged for the independent heater protections.
     * Invalid NaN model input falls back to the reference, marked out of range. */
    if (temperature != temperature) temperature = g_bmi088_calibration.reference_temperature_c;
    if (temperature < g_bmi088_calibration.minimum_temperature_c) temperature = g_bmi088_calibration.minimum_temperature_c;
    if (temperature > g_bmi088_calibration.maximum_temperature_c) temperature = g_bmi088_calibration.maximum_temperature_c;
    sample->compensation_temperature_c = temperature;
    delta = temperature - g_bmi088_calibration.reference_temperature_c;
    for (axis = 0; axis < 3; axis++) {
        const float bias = g_bmi088_calibration.gyro_bias_rad_s[axis] +
            g_bmi088_calibration.gyro_temperature_rad_s_c[axis] * delta;
        sample->gyro_rad_s[axis] -= bias;
        sample->accel_m_s2[axis] = (sample->accel_m_s2[axis] -
            g_bmi088_calibration.accel_bias_m_s2[axis]) *
            g_bmi088_calibration.accel_scale[axis];
    }
    sample->calibration_version = g_bmi088_calibration.version;
}

static void Bmi088_ReadSample(void)
{
    uint8_t accel[6], gyro[8], temp[2], id;
    unsigned axis;
    Bmi088Data_t sample;
    if (Bmi088_Read(1, 0x00, &id, 1) != HAL_OK) {
        Bmi088_Fail(BMI088_ERROR_SPI); return;
    }
    g_bmi088_stats.accel_chip_id = id;
    if (id != 0x1EU) { Bmi088_Fail(BMI088_ERROR_ACCEL_ID); return; }
    if (Bmi088_Read(1, 0x12, accel, sizeof(accel)) != HAL_OK ||
        Bmi088_Read(0, 0x00, gyro, sizeof(gyro)) != HAL_OK ||
        Bmi088_Read(1, 0x22, temp, sizeof(temp)) != HAL_OK) {
        Bmi088_Fail(BMI088_ERROR_SPI); return;
    }
    g_bmi088_stats.gyro_chip_id = gyro[0];
    if (gyro[0] != 0x0FU) { Bmi088_Fail(BMI088_ERROR_GYRO_ID); return; }
    memset(&sample, 0, sizeof(sample));
    for (axis = 0; axis < 3; axis++) {
        /* Register/nominal motion data exists only in this local sample until
         * calibration, never in g_bmi088_data or the blackbox registry. */
        sample.accel_m_s2[axis] = Bmi088_I16(accel + axis * 2) * BMI088_ACCEL_SCALE;
        sample.gyro_rad_s[axis] = Bmi088_I16(gyro + 2 + axis * 2) * BMI088_GYRO_SCALE;
    }
    sample.temperature_raw = (int16_t)(((uint16_t)temp[0] << 3) | (temp[1] >> 5));
    if (sample.temperature_raw > 1023) sample.temperature_raw -= 2048;
    sample.temperature_c = sample.temperature_raw * 0.125f + 23.0f;
    sample.sample_count = g_bmi088_data.sample_count + 1U;
    sample.sample_tick_ms = HAL_GetTick(); /* Completion time, not sensor time. */
    sample.has_sample = sample.valid = sample.initialized = 1;
    Bmi088_ApplyCalibration(&sample);
    taskENTER_CRITICAL();
    g_bmi088_data = sample; /* Publish only after ALL transfers/IDs succeed. */
    taskEXIT_CRITICAL();
    s_due_tick = HAL_GetTick() + 1U;
}

void Bmi088_Init(HAL_StatusTypeDef spi_init_status)
{
    memset((void *)&g_bmi088_data, 0, sizeof(g_bmi088_data));
    memset((void *)&g_bmi088_stats, 0, sizeof(g_bmi088_stats));
    g_bmi088_data.age_ms = UINT32_MAX;
    g_bmi088_stats.last_status = spi_init_status;
    s_config_index = s_config_verify = s_task_seen = 0;
    s_due_tick = HAL_GetTick();
    if (spi_init_status != HAL_OK) {
        g_bmi088_stats.stage = BMI088_DISABLED;
        g_bmi088_stats.init_error_count = 1;
        g_bmi088_data.error = BMI088_ERROR_SPI;
    } else g_bmi088_stats.stage = BMI088_RESET_ACCEL;
}

void Bmi088_OnTaskCreationFailure(void)
{
    g_bmi088_stats.stage = BMI088_DISABLED;
    g_bmi088_data.initialized = g_bmi088_data.valid = 0;
    g_bmi088_data.error = BMI088_ERROR_TASK;
}

void Bmi088_GetData(Bmi088Data_t *data)
{
    uint32_t now;
    taskENTER_CRITICAL();
    /* Timestamp and sample are observed under the same lock. */
    now = HAL_GetTick();
    g_bmi088_data.age_ms = g_bmi088_data.has_sample ?
        (uint32_t)(now - g_bmi088_data.sample_tick_ms) : UINT32_MAX;
    g_bmi088_data.valid = (uint8_t)(g_bmi088_data.initialized &&
        g_bmi088_data.has_sample && g_bmi088_data.age_ms < BMI088_SAMPLE_TIMEOUT_MS);
    if (data) *data = g_bmi088_data;
    taskEXIT_CRITICAL();
}

void Bmi088_Process(void)
{
    const uint32_t now = HAL_GetTick();
    if (s_task_seen) {
        const uint32_t gap = (uint32_t)(now - s_last_task_tick);
        if (gap > g_bmi088_stats.task_max_interval_ms) g_bmi088_stats.task_max_interval_ms = gap;
    }
    s_last_task_tick = now; s_task_seen = 1;
    g_bmi088_stats.task_count++;
    Bmi088_GetData(NULL);
    if (g_bmi088_stats.stage == BMI088_DISABLED || (int32_t)(now - s_due_tick) < 0) return;
    switch (g_bmi088_stats.stage) {
    case BMI088_RETRY_WAIT:
        s_config_index = s_config_verify = 0;
        g_bmi088_stats.stage = BMI088_RESET_ACCEL;
        /* fall through */
    case BMI088_RESET_ACCEL:
        g_bmi088_stats.init_attempt_count++;
        if (!Bmi088_CheckId(1)) return;
        if (Bmi088_Write(1, 0x7E, 0xB6) != HAL_OK) { Bmi088_Fail(BMI088_ERROR_SPI); return; }
        g_bmi088_stats.stage = BMI088_WAIT_ACCEL;
        s_due_tick = HAL_GetTick() + BMI088_RESET_WAIT_MS;
        break;
    case BMI088_WAIT_ACCEL:
        if (Bmi088_CheckId(1)) g_bmi088_stats.stage = BMI088_CONFIG_ACCEL;
        break;
    case BMI088_CONFIG_ACCEL: Bmi088_Configure(1); break;
    case BMI088_RESET_GYRO:
        if (!Bmi088_CheckId(0)) return;
        if (Bmi088_Write(0, 0x14, 0xB6) != HAL_OK) { Bmi088_Fail(BMI088_ERROR_SPI); return; }
        g_bmi088_stats.stage = BMI088_WAIT_GYRO;
        s_due_tick = HAL_GetTick() + BMI088_RESET_WAIT_MS;
        break;
    case BMI088_WAIT_GYRO:
        if (Bmi088_CheckId(0)) g_bmi088_stats.stage = BMI088_CONFIG_GYRO;
        break;
    case BMI088_CONFIG_GYRO: Bmi088_Configure(0); break;
    case BMI088_RUNNING: Bmi088_ReadSample(); break;
    default: break;
    }
}
