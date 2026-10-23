#pragma once
#include <stdint.h>
typedef int BaseType_t;
typedef void *TaskHandle_t;
#define pdFALSE 0
#define pdTRUE 1
#define pdMS_TO_TICKS(x) (x)
#define portYIELD_FROM_ISR(x) ((void)(x))
extern uint32_t fake_tick;
static void vTaskDelay(unsigned t){fake_tick+=t;}
static TaskHandle_t xTaskGetCurrentTaskHandle(void){return (void *)1;}
static unsigned test_notify_block_count;
static unsigned ulTaskNotifyTake(int clear,unsigned t){(void)clear;fake_tick+=t;if(t)test_notify_block_count++;return 0;}
static void vTaskNotifyGiveFromISR(TaskHandle_t t,BaseType_t *w){(void)t;(void)w;}

static unsigned test_sd_critical_depth;
#define taskENTER_CRITICAL() ((void)++test_sd_critical_depth)
#define taskEXIT_CRITICAL() ((void)--test_sd_critical_depth)
#define taskSCHEDULER_NOT_STARTED 0
static int xTaskGetSchedulerState(void) { return taskSCHEDULER_NOT_STARTED; }
