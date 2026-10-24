#ifndef CP_HOST_PLATFORM_H
#define CP_HOST_PLATFORM_H
#include <assert.h>
#include <stdint.h>
static uint32_t h_cycles, h_tick, h_clock = 480000000U, h_enabled = 1U;
static uint32_t h_isr, h_running = 1U, h_mask, h_nesting, h_suspended;
static uint32_t h_entries, h_exits, h_pre_delay, h_post_delay, h_frozen;
static uintptr_t h_owner = 1U;
static void (*h_post_exit_hook)(void);
static uint32_t CpCycles(void) { uint32_t v = h_cycles; if (!h_frozen) h_cycles++; return v; }
static uint32_t CpTick(void) { return h_tick; }
static uint32_t CpClock(void) { return h_clock; }
static uint32_t CpClockEnabled(void) { return h_enabled; }
static uint32_t CpIsr(void) { return h_isr; }
static uint32_t CpMask(void) { return h_mask; }
static uint32_t CpRunning(void) { return h_running; }
static uintptr_t CpOwner(void) { return h_owner; }
static void CpRawEnter(void)
{
    if (!h_frozen) { h_cycles += h_pre_delay; h_pre_delay = 0; h_cycles += 5; }
    h_mask = 0x50; h_nesting++; h_entries++;
#ifdef CP_ACTUAL_APPLICATION_STUBS
    test_critical_depth++;
#endif
}
static void CpRawExit(void)
{
    assert(h_nesting); h_nesting--; h_exits++;
#ifdef CP_ACTUAL_APPLICATION_STUBS
    assert(test_critical_depth); test_critical_depth--;
#endif
    if (!h_frozen) h_cycles += 4;
    if (!h_nesting)
    {
        h_mask = 0;
        if (!h_frozen) { h_cycles += h_post_delay; h_post_delay = 0; }
        if (h_post_exit_hook)
        {
            void (*hook)(void) = h_post_exit_hook;
            h_post_exit_hook = 0;
            hook();
        }
    }
}
static void CpSuspend(void) { assert(!h_mask); h_suspended++; }
static void CpResume(void) { assert(h_suspended); h_suspended--; }
static void CpBarrier(void) { }
#endif
