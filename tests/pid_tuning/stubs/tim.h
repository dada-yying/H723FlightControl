#pragma once
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
typedef struct { uint32_t compare[4], autoreload, active_autoreload, counter; } TIM_HandleTypeDef;
static TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim7;
static HAL_StatusTypeDef test_timer_init_status = HAL_OK, test_timer_start_status = HAL_OK;
static unsigned test_timer_init_calls, test_timer_start_calls, test_timer_stop_calls, test_timer_started;
#define TIM_FLAG_UPDATE 1U
#define __HAL_TIM_CLEAR_FLAG(t,f) ((void)(t),(void)(f))
#define __HAL_TIM_SET_AUTORELOAD(t,p) ((t)->autoreload=(p))
#define __HAL_TIM_SET_COUNTER(t,p) ((t)->counter=(p))
#define TIM_EVENTSOURCE_UPDATE 1U
static HAL_StatusTypeDef test_timer_event_status=HAL_OK;
static HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *timer, uint32_t event)
{ (void)event; if(test_timer_event_status==HAL_OK)timer->active_autoreload=timer->autoreload;return test_timer_event_status; }
static HAL_StatusTypeDef MX_TIM4_Init(void)
{ test_timer_init_calls++; return test_timer_init_status; }
static HAL_StatusTypeDef MX_TIM7_Init(void)
{ test_timer_init_calls++; return test_timer_init_status; }
static HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer)
{ (void)timer; test_timer_start_calls++; if(test_timer_start_status==HAL_OK)test_timer_started=1; return test_timer_start_status; }
static HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *timer)
{ (void)timer; test_timer_stop_calls++; test_timer_started=0; return HAL_OK; }
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_3 2U
#define TIM_CHANNEL_4 3U
static HAL_StatusTypeDef test_pwm_start_status = HAL_OK;
static unsigned test_check_pwm_critical;
#ifdef TEST_SCHEDULER_HOOKS
static void TestCompareHook(void);
#endif
static void test_set_compare(TIM_HandleTypeDef *timer, unsigned channel, uint32_t pulse)
{
    if (test_check_pwm_critical) assert(test_critical_depth > 0);
    timer->compare[channel] = pulse;
#ifdef TEST_SCHEDULER_HOOKS
    TestCompareHook();
#endif
}
#define __HAL_TIM_SET_COMPARE(t,c,p) test_set_compare(t,c,p)
static HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, unsigned channel)
{ (void)timer; (void)channel; return test_pwm_start_status; }
static HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, unsigned channel)
{ (void)timer; (void)channel; return HAL_OK; }
