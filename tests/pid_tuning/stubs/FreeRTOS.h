#pragma once
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#define configASSERT(x) assert(x)
typedef void *TaskHandle_t;
typedef int BaseType_t;
#define pdFALSE 0
static unsigned test_critical_depth;
#ifdef TEST_SCHEDULER_HOOKS
static void TestCriticalEnter(void);
static void TestCriticalExit(void);
#define taskENTER_CRITICAL() TestCriticalEnter()
#define taskEXIT_CRITICAL() TestCriticalExit()
#else
#define taskENTER_CRITICAL() ((void)++test_critical_depth)
#define taskEXIT_CRITICAL() ((void)--test_critical_depth)
#endif
#define portYIELD_FROM_ISR(x) ((void)(x))
static unsigned test_notify_calls;
static TaskHandle_t test_notify_task;
static void vTaskNotifyGiveFromISR(TaskHandle_t t, BaseType_t *w)
{ test_notify_calls++; test_notify_task=t; (void)w; }
