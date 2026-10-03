#ifndef BMI088_CALIBRATION_H
#define BMI088_CALIBRATION_H

/* This individual board, six stationary faces and warm-up, 2026-10-03.
 * Provenance/selection/held-out checks: BMI088_CALIBRATION.md and
 * ../tools/analyze_bmi088_calibration.py. Native sensor XYZ, no body rotation.
 * Applied to the current nominal +/-3g and +/-2000 deg/s unit conversion.
 * Linear gyro temperature bias validated across log recordings. Accel thermal
 * candidates were rejected; preserve the six-face offset/scale. No gyro scale,
 * body rotation, adaptive zeroing or software filtering. Cold endpoint has
 * limited static coverage; these bounds are not an accuracy guarantee. */
#define BMI088_CALIBRATION_VERSION 2026100302U
#define BMI088_CALIBRATION_REFERENCE_C 40.0f
#define BMI088_CALIBRATION_MIN_C 22.625f
#define BMI088_CALIBRATION_MAX_C 40.25f
#define BMI088_CALIBRATION_GYRO_TEMP_RAD_S_C \
    {-0.0002060266970f, 0.0001034203285f, -0.00004799084795f}
#define BMI088_CALIBRATION_GYRO_BIAS_RAD_S \
    {-0.002295192478f, 0.0002904073052f, -0.0006771495606f}
#define BMI088_CALIBRATION_ACCEL_BIAS_M_S2 \
    {-0.04141649145f, -0.002665104650f, 0.02095470497f}
#define BMI088_CALIBRATION_ACCEL_SCALE \
    {1.005745032f, 1.003663594f, 1.002671821f}

#endif
