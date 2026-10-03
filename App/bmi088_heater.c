#include "bmi088_heater.h"
#include "bmi088.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define HEATER_CONTROL_PERIOD_MS 20U
#define HEATER_KP_CCR_PER_C 100.0f
#define HEATER_KI_CCR_PER_C_S 50.0f

volatile Bmi088HeaterState_t g_bmi088_heater_state;
volatile uint32_t g_bmi088_heater_force_stop;
static float s_integral_ccr;
static uint32_t s_control_tick, s_sample_count;
static uint8_t s_control_seen, s_overtemperature;

static float Heater_Clamp(float value)
{
    if (value < 0) return 0;
    if (value > BMI088_HEATER_MAX_CCR) return BMI088_HEATER_MAX_CCR;
    return value;
}

/* Both task callers hold a short critical section. No SPI, UART or waits. */
static void Heater_Off(Bmi088HeaterStopReason_t reason)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
    s_integral_ccr = 0; s_control_seen = 0;
    g_bmi088_heater_state.integral_ccr = 0;
    g_bmi088_heater_state.pwm_ccr = 0;
    g_bmi088_heater_state.duty = 0;
    g_bmi088_heater_state.heating = 0;
    g_bmi088_heater_state.stop_reason = reason;
}

static int Heater_CheckSample(const Bmi088Data_t *sample)
{
    g_bmi088_heater_state.temperature_c = sample->temperature_c;
    g_bmi088_heater_state.sample_age_ms = sample->age_ms;
    if (!g_bmi088_heater_state.initialized) return 0; /* CCR is already zero. */
    if (s_overtemperature) {
        Heater_Off(BMI088_HEATER_OVERTEMPERATURE); return 0;
    }
    if (!sample->initialized) {
        Heater_Off(BMI088_HEATER_SENSOR_INVALID); return 0;
    }
    if (!sample->has_sample) {
        Heater_Off(BMI088_HEATER_WAIT_SAMPLE); return 0;
    }
    if (sample->age_ms >= BMI088_SAMPLE_TIMEOUT_MS) {
        Heater_Off(BMI088_HEATER_SENSOR_TIMEOUT); return 0;
    }
    if (!sample->valid) {
        Heater_Off(BMI088_HEATER_SENSOR_INVALID); return 0;
    }
    /* Ordered comparisons also reject NaN/Inf. Out-of-range temperature is
     * diagnostic failure, not a request for maximum heating. */
    if (!(sample->temperature_c >= -40.0f && sample->temperature_c <= 85.0f)) {
        Heater_Off(BMI088_HEATER_BAD_TEMPERATURE); return 0;
    }
    if (sample->temperature_c >= BMI088_HEATER_OVERTEMP_C) {
        s_overtemperature = 1;
        g_bmi088_heater_state.overtemperature_latched = 1;
        g_bmi088_heater_state.overtemperature_count++;
        Heater_Off(BMI088_HEATER_OVERTEMPERATURE); return 0;
    }
    if (g_bmi088_heater_force_stop) {
        Heater_Off(BMI088_HEATER_MANUAL_STOP); return 0;
    }
    return 1;
}

void Bmi088Heater_Init(HAL_StatusTypeDef pwm_init_status)
{
    HAL_StatusTypeDef status = pwm_init_status;
    /* Startup only. PB1 is held low by GPIO before timer initialization. */
    memset((void *)&g_bmi088_heater_state, 0, sizeof(g_bmi088_heater_state));
    g_bmi088_heater_state.target_c = BMI088_HEATER_TARGET_C;
    g_bmi088_heater_state.sample_age_ms = UINT32_MAX;
    g_bmi088_heater_force_stop = 0;
    s_integral_ccr = 0; s_control_seen = s_overtemperature = 0;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
    if (status == HAL_OK) status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    g_bmi088_heater_state.last_status = status;
    g_bmi088_heater_state.initialized = (uint8_t)(status == HAL_OK);
    g_bmi088_heater_state.stop_reason = status == HAL_OK ?
        BMI088_HEATER_WAIT_SAMPLE : BMI088_HEATER_PWM_ERROR;
    /* Initialization failure never enters global Error_Handler. */
}

void Bmi088Heater_CheckTimeout(void)
{
    Bmi088Data_t sample;
    taskENTER_CRITICAL();
    /* Nested snapshot lock is intentional: the lower-priority guard cannot
     * stop a freshly updated heater using a preempted, old sample snapshot. */
    Bmi088_GetData(&sample);
    (void)Heater_CheckSample(&sample);
    taskEXIT_CRITICAL();
}

void Bmi088Heater_Process(void)
{
    Bmi088Data_t sample;
    uint32_t now, elapsed;
    float error, dt, proposed_i, proposed_output, output;
    taskENTER_CRITICAL();
    Bmi088_GetData(&sample);
    now = HAL_GetTick();
    if (!Heater_CheckSample(&sample)) { taskEXIT_CRITICAL(); return; }
    elapsed = (uint32_t)(now - s_control_tick);
    if (s_control_seen && (elapsed < HEATER_CONTROL_PERIOD_MS ||
        sample.sample_count == s_sample_count)) { taskEXIT_CRITICAL(); return; }
    /* No integration across lost scheduling time; no catch-up control burst. */
    if (s_control_seen && elapsed > 100U) {
        s_integral_ccr = 0; s_control_seen = 0;
    }
    dt = s_control_seen ? elapsed * 0.001f : 0.0f;
    error = BMI088_HEATER_TARGET_C - sample.temperature_c;
    proposed_i = Heater_Clamp(s_integral_ccr + HEATER_KI_CCR_PER_C_S * error * dt);
    proposed_output = HEATER_KP_CCR_PER_C * error + proposed_i;
    /* Conditional anti-windup: only stop integration toward saturation. */
    if (!((proposed_output > BMI088_HEATER_MAX_CCR && error > 0) ||
          (proposed_output < 0 && error < 0))) s_integral_ccr = proposed_i;
    output = Heater_Clamp(HEATER_KP_CCR_PER_C * error + s_integral_ccr);
    g_bmi088_heater_state.pwm_ccr = (uint32_t)(output + 0.5f);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, g_bmi088_heater_state.pwm_ccr);
    g_bmi088_heater_state.target_c = BMI088_HEATER_TARGET_C;
    g_bmi088_heater_state.error_c = error;
    g_bmi088_heater_state.integral_ccr = s_integral_ccr;
    g_bmi088_heater_state.duty = g_bmi088_heater_state.pwm_ccr / 10000.0f;
    g_bmi088_heater_state.heating = (uint8_t)(g_bmi088_heater_state.pwm_ccr != 0U);
    g_bmi088_heater_state.stop_reason = BMI088_HEATER_OK;
    g_bmi088_heater_state.update_count++;
    s_sample_count = sample.sample_count; s_control_tick = now; s_control_seen = 1;
    taskEXIT_CRITICAL();
}
