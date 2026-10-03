#ifndef BMI088_H
#define BMI088_H

#include "stm32h7xx_hal.h"

#define BMI088_SAMPLE_TIMEOUT_MS 20U

typedef enum {
    BMI088_ERROR_NONE = 0,
    BMI088_ERROR_SPI,
    BMI088_ERROR_ACCEL_ID,
    BMI088_ERROR_GYRO_ID,
    BMI088_ERROR_ACCEL_CONFIG,
    BMI088_ERROR_GYRO_CONFIG,
    BMI088_ERROR_TASK
} Bmi088Error_t;

typedef enum {
    BMI088_DISABLED = 0,
    BMI088_RESET_ACCEL,
    BMI088_WAIT_ACCEL,
    BMI088_CONFIG_ACCEL,
    BMI088_RESET_GYRO,
    BMI088_WAIT_GYRO,
    BMI088_CONFIG_GYRO,
    BMI088_RUNNING,
    BMI088_RETRY_WAIT
} Bmi088Stage_t;

/* Canonical BMI088 "raw" means CALIBRATED but UNFILTERED physical XYZ values.
 * Register counts and pre-calibration motion values are private temporaries;
 * neither is published or logged. No body remapping/flight-control consumer. */
typedef struct {
    float gyro_rad_s[3];
    float accel_m_s2[3];
    float temperature_c;
    int16_t temperature_raw;
    uint32_t sample_count;
    uint32_t sample_tick_ms;
    uint32_t age_ms;
    uint8_t has_sample;
    uint8_t valid;
    uint8_t initialized;
    uint8_t error;
    uint32_t calibration_version; /* Zero before first complete read. */
    float compensation_temperature_c; /* Clamped model input, not measured T. */
    uint8_t compensation_in_range; /* Numerical model bounds, NOT accuracy. */
} Bmi088Data_t;

typedef struct {
    float gyro_bias_rad_s[3];
    float accel_bias_m_s2[3];
    float accel_scale[3];
    float gyro_temperature_rad_s_c[3];
    float reference_temperature_c;
    float minimum_temperature_c;
    float maximum_temperature_c;
    uint32_t version;
} Bmi088Calibration_t;

typedef struct {
    Bmi088Stage_t stage;
    uint32_t task_count;
    uint32_t init_attempt_count;
    uint32_t init_success_count;
    uint32_t init_error_count;
    uint32_t read_error_count;
    uint32_t spi_error_count;
    uint32_t task_max_interval_ms;
    uint32_t stack_free_words;
    HAL_StatusTypeDef last_status;
    uint32_t last_spi_error;
    uint8_t accel_chip_id;
    uint8_t gyro_chip_id;
    uint8_t last_config_register;
    uint8_t last_config_value;
} Bmi088Stats_t;

extern volatile Bmi088Data_t g_bmi088_data;
extern volatile Bmi088Stats_t g_bmi088_stats;
extern const Bmi088Calibration_t g_bmi088_calibration;

void Bmi088_Init(HAL_StatusTypeDef spi_init_status); /* Startup, before tasks. */
void Bmi088_Process(void); /* Sole SPI2 owner: Bmi088Task, about every 1 ms. */
void Bmi088_GetData(Bmi088Data_t *data); /* Short, consistent snapshot. */
void Bmi088_OnTaskCreationFailure(void);

#endif
