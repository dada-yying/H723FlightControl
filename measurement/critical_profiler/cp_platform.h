#ifndef CP_PLATFORM_H
#define CP_PLATFORM_H
#ifdef CP_HOST_TEST
#include "cp_host_platform.h"
#else
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
static __inline uint32_t CpCycles(void) { return DWT->CYCCNT; }
static __inline uint32_t CpTick(void) { return HAL_GetTick(); }
static __inline uint32_t CpClock(void) { return SystemCoreClock; }
static __inline uint32_t CpClockEnabled(void) { return DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk; }
static __inline uint32_t CpIsr(void) { return __get_IPSR(); }
static __inline uint32_t CpMask(void) { return __get_BASEPRI() | __get_PRIMASK(); }
static __inline uint32_t CpRunning(void) { return xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED; }
static __inline uintptr_t CpOwner(void) { return (uintptr_t)xTaskGetCurrentTaskHandle(); }
static __inline void CpRawEnter(void) { vPortEnterCritical(); }
static __inline void CpRawExit(void) { vPortExitCritical(); }
static __inline void CpSuspend(void) { vTaskSuspendAll(); }
static __inline void CpResume(void) { (void)xTaskResumeAll(); }
static __inline void CpBarrier(void) { __DMB(); }
#endif
#endif
