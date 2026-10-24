#include <assert.h>
#include <string.h>
#include <stdio.h>
#define CP_SITE_COUNT 3U
#include "../critical_profiler.h"
#include "../../../App/vofa.c"
static unsigned snapshot_calls;
void Cp_PublishSnapshot(void) { snapshot_calls++; }
#include "../telemetry_adapter.c"
static uint32_t allocated_stack;
static osPriority_t allocated_priority;
osThreadId_t osThreadNew(osThreadFunc_t function, void *argument, const osThreadAttr_t *attr)
{
    (void)function; (void)argument;
    allocated_stack = attr->stack_size;
    allocated_priority = attr->priority;
    return (osThreadId_t)1;
}
int main(void)
{
    osThreadAttr_t attr = {0};
    uint8_t frame[VOFA_FLIGHT_FRAME_SIZE];
    unsigned i;
    for (i=0;i<sizeof(frame);i++) frame[i]=(uint8_t)i;
    assert(Cp_TelemetryTap(frame)==HAL_OK);
    assert(snapshot_calls==1 && test_uart_calls==1 && test_uart_size==sizeof(frame));
    assert(!memcmp(frame,test_uart_bytes,sizeof(frame)));
    test_uart_status=HAL_TIMEOUT;
    assert(Cp_TelemetryTap(frame)==HAL_TIMEOUT);
    assert(snapshot_calls==2 && test_uart_calls==2 && !test_critical_depth);
    attr.name="RcTask"; attr.stack_size=512; attr.priority=osPriorityAboveNormal;
    assert(Cp_ThreadNew(0,0,&attr)==(osThreadId_t)1);
    assert(allocated_stack==1024 && allocated_priority==osPriorityAboveNormal && attr.stack_size==512);
    attr.name="OtherTask";
    (void)Cp_ThreadNew(0,0,&attr); assert(allocated_stack==512);
    puts("PASS snapshot tap preserves exact frozen frame bytes and UART status");
    return 0;
}
