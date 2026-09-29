#pragma once
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
typedef struct { uint32_t compare[4]; } TIM_HandleTypeDef;
static TIM_HandleTypeDef htim1, htim2;
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_3 2U
static HAL_StatusTypeDef test_pwm_start_status = HAL_OK;
static unsigned test_check_pwm_critical;
static void test_set_compare(TIM_HandleTypeDef *timer, unsigned channel, uint32_t pulse)
{
    if (test_check_pwm_critical) assert(test_critical_depth > 0);
    timer->compare[channel] = pulse;
}
#define __HAL_TIM_SET_COMPARE(t,c,p) test_set_compare(t,c,p)
static HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, unsigned channel)
{ (void)timer; (void)channel; return test_pwm_start_status; }
static HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, unsigned channel)
{ (void)timer; (void)channel; return HAL_OK; }
