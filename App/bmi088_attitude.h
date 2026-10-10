#ifndef BMI088_ATTITUDE_H
#define BMI088_ATTITUDE_H

#include "bmi088.h"

typedef enum {
    BMI088_ATTITUDE_WAIT_SOURCE = 0,
    BMI088_ATTITUDE_ALIGNING,
    BMI088_ATTITUDE_OK,
    BMI088_ATTITUDE_SOURCE_INVALID,
    BMI088_ATTITUDE_TIME_GAP,
    BMI088_ATTITUDE_BAD_INPUT,
    BMI088_ATTITUDE_GYRO_OVERRANGE,
    BMI088_ATTITUDE_NUMERICAL_ERROR
} Bmi088AttitudeError_t;

typedef struct Bmi088AttitudeState {
    /* Aircraft display convention: right side down / nose down / right turn
     * are positive. Yaw is relative, NOT a magnetically referenced heading. */
    float roll_rad, pitch_rad, yaw_rad;
    float quaternion[4]; /* wxyz: FRD body -> relative NED; standard pitch UP. */
    float dt_s, accel_norm_m_s2;
    uint32_t sample_count, sample_tick_ms, age_ms;
    uint32_t update_count, epoch, reset_count, rejected_count, accel_rejected_count;
    uint8_t initialized, valid, accel_used, error;
} Bmi088AttitudeState_t;

extern volatile Bmi088AttitudeState_t g_bmi088_attitude_state;

void Bmi088Attitude_Init(void); /* Called by BMI acquisition init. */
void Bmi088Attitude_Invalidate(void); /* Sole owner: Bmi088Task. */
/* Compute outside critical sections, once per complete calibrated sample.
 * Caller atomically publishes returned state AND its matching BMI sample.
 * Returned pointer is private workspace, only the BMI acquisition owner uses it. */
const Bmi088AttitudeState_t *Bmi088Attitude_Compute(const Bmi088Data_t *sample,
                                                  uint32_t completion_cycle);
void Bmi088Attitude_GetState(Bmi088AttitudeState_t *state);

#endif
