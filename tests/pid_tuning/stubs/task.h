#pragma once
#include "FreeRTOS.h"

#define taskSCHEDULER_NOT_STARTED 0
static int test_scheduler_state;
static int xTaskGetSchedulerState(void) { return test_scheduler_state; }
#include "../../../BSP/bsp_dwt.c"

static TaskHandle_t test_current_task = (TaskHandle_t)1;
static TaskHandle_t xTaskGetCurrentTaskHandle(void) { return test_current_task; }
