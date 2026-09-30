#pragma once
#include <stdint.h>
#include <stddef.h>
typedef void *TaskHandle_t;
typedef int BaseType_t;
#define pdFALSE 0
static unsigned test_critical_depth;
#define taskENTER_CRITICAL() ((void)++test_critical_depth)
#define taskEXIT_CRITICAL() ((void)--test_critical_depth)
#define portYIELD_FROM_ISR(x) ((void)(x))
static void vTaskNotifyGiveFromISR(TaskHandle_t t, BaseType_t *w) { (void)t; (void)w; }
