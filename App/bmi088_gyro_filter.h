#ifndef BMI088_GYRO_FILTER_H
#define BMI088_GYRO_FILTER_H

#include "bmi088.h"

/* Compile-time cascade configuration. Always on; calibrated native XYZ.
 * Bandwidth is the DIGITAL half-power (-3.0103 dB) crossing separation in Hz. */
#define BMI088_GYRO_NOTCH_CENTER_HZ 120.0f
#define BMI088_GYRO_NOTCH_BANDWIDTH_HZ 40.0f
#define BMI088_GYRO_FILTER_CUTOFF_HZ 50.0f
#define BMI088_GYRO_FILTER_RATE_WINDOW 128U
#define BMI088_GYRO_FILTER_RATE_WINDOWS 3U
#define BMI088_GYRO_FILTER_RATE_TOLERANCE 0.02f

typedef enum {
    BMI088_GYRO_FILTER_WAIT_SAMPLE = 0,
    BMI088_GYRO_FILTER_OK,
    BMI088_GYRO_FILTER_SOURCE_INVALID,
    BMI088_GYRO_FILTER_BAD_INPUT,
    BMI088_GYRO_FILTER_OVERRANGE,
    BMI088_GYRO_FILTER_TIME_ERROR,
    BMI088_GYRO_FILTER_BAD_PARAMETERS,
    BMI088_GYRO_FILTER_NUMERICAL_ERROR
} Bmi088GyroFilterError_t;

typedef struct Bmi088GyroFilterState {
    float gyro_rad_s[3]; /* Calibrated -> fixed notch -> LPF, native XYZ, rad/s. */
    float cutoff_hz, sample_hz, applied_sample_hz;
    float b0, b1, b2, a1, a2; /* Observer copies; Watch writes do not tune. */
    uint32_t sample_count, sample_tick_ms, age_ms;
    uint32_t update_count, reset_count, rejected_count, coefficient_update_count;
    uint32_t max_compute_duration_us;
    uint8_t has_sample, initialized, valid, error;
    float notch_center_hz, notch_bandwidth_hz;
    float notch_b0, notch_b1, notch_b2, notch_a1, notch_a2;
} Bmi088GyroFilterState_t;

extern volatile Bmi088GyroFilterState_t g_bmi088_gyro_filter;
void Bmi088GyroFilter_Init(void);
void Bmi088GyroFilter_Invalidate(void); /* Sole owner: Bmi088Task. */
/* Calculate outside critical sections. Acquisition atomically publishes this
 * private result alongside the matching raw sample and attitude. */
const Bmi088GyroFilterState_t *Bmi088GyroFilter_Compute(const Bmi088Data_t *sample,
                                                     uint32_t completion_cycle);
void Bmi088GyroFilter_GetState(Bmi088GyroFilterState_t *state);

#endif
