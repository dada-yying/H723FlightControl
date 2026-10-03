#ifndef BMI088_HEATER_H
#define BMI088_HEATER_H

#include "stm32h7xx_hal.h"

#define BMI088_HEATER_TARGET_C 40.0f
#define BMI088_HEATER_OVERTEMP_C 45.0f
#define BMI088_HEATER_MAX_CCR 500U /* ARR=9999, maximum duty 5%, as board example. */

typedef enum {
    BMI088_HEATER_OK = 0,
    BMI088_HEATER_WAIT_SAMPLE,
    BMI088_HEATER_SENSOR_INVALID,
    BMI088_HEATER_SENSOR_TIMEOUT,
    BMI088_HEATER_BAD_TEMPERATURE,
    BMI088_HEATER_OVERTEMPERATURE,
    BMI088_HEATER_MANUAL_STOP,
    BMI088_HEATER_PWM_ERROR
} Bmi088HeaterStopReason_t;

typedef struct {
    float target_c;
    float temperature_c;
    float error_c;
    float integral_ccr;
    float duty;
    uint32_t pwm_ccr;
    uint32_t update_count;
    uint32_t sample_age_ms;
    uint32_t overtemperature_count;
    Bmi088HeaterStopReason_t stop_reason;
    HAL_StatusTypeDef last_status;
    uint8_t initialized;
    uint8_t heating;
    uint8_t overtemperature_latched;
} Bmi088HeaterState_t;

extern volatile Bmi088HeaterState_t g_bmi088_heater_state;
/* Keil Watch emergency/manual stop. 1=off, 0=normal; default 0.
 * This does not clear an overtemperature latch. Target remains fixed at 40 C. */
extern volatile uint32_t g_bmi088_heater_force_stop;

void Bmi088Heater_Init(HAL_StatusTypeDef pwm_init_status);
void Bmi088Heater_Process(void); /* 50 Hz control, fresh complete SPI reads only. */
void Bmi088Heater_CheckTimeout(void); /* Default task: independent 10 ms guard. */

#endif
