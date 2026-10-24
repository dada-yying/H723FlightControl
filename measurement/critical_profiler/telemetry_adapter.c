/* Measurement-only snapshot tap: NO profiler data is placed in UART frames. */
#include "critical_profiler.h"
#include "vofa.h"
#include "cmsis_os2.h"
#include <string.h>
/* Measurement-only resource headroom. Production attributes and its build are
 * unchanged. Keep the same entry, argument, priority and name; only RC stack
 * allocation grows by 512 bytes because instrumentation adds call-stack cost. */
osThreadId_t Cp_ThreadNew(osThreadFunc_t function, void *argument, const osThreadAttr_t *attr)
{
    osThreadAttr_t measured;
    if (attr && attr->name && strcmp(attr->name, "RcTask") == 0 && attr->stack_size < 1024U)
    {
        measured = *attr;
        measured.stack_size = 1024U;
        return osThreadNew(function, argument, &measured);
    }
    return osThreadNew(function, argument, attr);
}
HAL_StatusTypeDef Cp_TelemetryTap(const uint8_t frame[VOFA_FLIGHT_FRAME_SIZE])
{
    Cp_PublishSnapshot();
    return VOFA_SendFrame(frame);
}
