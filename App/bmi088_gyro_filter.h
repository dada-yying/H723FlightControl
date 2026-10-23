#ifndef BMI088_GYRO_FILTER_H
#define BMI088_GYRO_FILTER_H

#include "bmi088.h"

/* Compile-time configuration: compare 40/50/60 Hz here, not in the logger.
 * Always on. Native sensor XYZ; no body rotation or temperature correction. */
#define BMI088_GYRO_FILTER_CUTOFF_HZ 50.0f
#define BMI088_GYRO_FILTER_RATE_WINDOW 128U
#define BMI088_GYRO_FILTER_RATE_WINDOWS 3U
#define BMI088_GYRO_FILTER_RATE_TOLERANCE 0.02f

typedef enum
{
    BMI088_GYRO_FILTER_WAIT_SAMPLE = 0,
    BMI088_GYRO_FILTER_OK,
    BMI088_GYRO_FILTER_SOURCE_INVALID,
    BMI088_GYRO_FILTER_BAD_INPUT,
    BMI088_GYRO_FILTER_OVERRANGE,
    BMI088_GYRO_FILTER_TIME_ERROR,
    BMI088_GYRO_FILTER_BAD_PARAMETERS,
    BMI088_GYRO_FILTER_NUMERICAL_ERROR
} Bmi088GyroFilterError_t;

typedef struct Bmi088GyroFilterState
{
    float gyro_rad_s[3]; /* Calibrated + low-pass, native XYZ, rad/s. */
    float cutoff_hz, sample_hz, applied_sample_hz;
    float b0, b1, b2, a1, a2; /* Observer copies; Watch writes do not tune. */
    uint32_t sample_count, sample_tick_ms, age_ms;
    uint32_t update_count, reset_count, rejected_count, coefficient_update_count;
    uint32_t max_compute_duration_us;
    uint8_t has_sample, initialized, valid, error;
} Bmi088GyroFilterState_t;

void Bmi088GyroFilter_CopyWorking(Bmi088GyroFilterState_t *state); /* Acquisition owner only. */
void Bmi088GyroFilter_Init(void);
void Bmi088GyroFilter_Invalidate(void); /* Sole owner: Bmi088Task. */
/* Calculate outside critical sections. Acquisition atomically publishes this
 * private result alongside the matching raw sample and attitude. */
const Bmi088GyroFilterState_t *Bmi088GyroFilter_Compute(const Bmi088Data_t *sample,
                                                        uint32_t completion_cycle);
void Bmi088GyroFilter_GetState(Bmi088GyroFilterState_t *state);

#endif
