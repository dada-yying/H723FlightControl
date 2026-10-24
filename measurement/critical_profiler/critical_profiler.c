/* Measurement only. Compile WITHOUT the application forced-include header. */
#include "critical_profiler.h"
#include "cp_platform.h"
#include <limits.h>

typedef struct
{
    uint32_t site, start_envelope, start_body, tick;
    uint32_t enter_bookkeeping, inclusive_depth;
    uintptr_t owner;
    uint8_t measured;
} CpFrame;
static CpFrame s_stack[CP_MAX_DEPTH];
static uint32_t s_depth, s_clock, s_max_depth, s_startup_pairs;
/* Invalid ISR calls may latch a fault; keep the reject flag observable. */
static volatile uint32_t s_faults;
static uint32_t s_outer_hits, s_outer_body, s_outer_envelope, s_accounting_body;
static uint32_t s_snapshot_envelope, s_snapshot_sequence;
static CpSiteStats s_stats[CP_SITE_COUNT];
volatile CpSnapshot g_cp_snapshot;

static void reject(uint32_t flag)
{
    s_faults |= flag;
    g_cp_snapshot.valid = 0U;
    g_cp_snapshot.faults = s_faults;
}

static void maximum(uint32_t *value, uint32_t sample)
{
    if (sample > *value) *value = sample;
}
static void increment(uint32_t *value)
{
    if (*value == UINT32_MAX) reject(CP_FAULT_COUNTER);
    else (*value)++;
}
static uint32_t microseconds(uint32_t cycles)
{
    uint32_t divisor = s_clock / 1000000U;
    if (!divisor || s_clock % 1000000U) return 0U;
    return cycles / divisor + (uint32_t)(cycles % divisor != 0U);
}

void Cp_Enter(uint32_t site)
{
    uint32_t envelope = CpCycles();
    uint32_t pre_mask = CpMask();
    uint32_t body, measured, i;
    CpFrame *frame;
    if (CpIsr())
    {
        reject(CP_FAULT_CONTEXT);
        return; /* Never invent ISR semantics for a task-only API. */
    }
    CpRawEnter();
    body = CpCycles();
    measured = CpRunning();
    if (s_depth >= CP_MAX_DEPTH)
    {
        reject(CP_FAULT_DEPTH);
        if (s_depth != UINT32_MAX) s_depth++;
        return;
    }
    if (site >= CP_SITE_COUNT) reject(CP_FAULT_SITE);
    if (measured)
    {
        if (!CpClockEnabled() || CpClock() < 1000000U || CpClock() % 1000000U)
            reject(CP_FAULT_CLOCK);
        if (!s_clock) s_clock = CpClock();
        if (s_clock != CpClock()) reject(CP_FAULT_CLOCK);
        if (!s_depth && pre_mask) reject(CP_FAULT_EXTERNAL_MASK);
        if (s_depth && s_stack[s_depth - 1U].owner != CpOwner())
            reject(CP_FAULT_CONTEXT);
        if (!CpMask()) reject(CP_FAULT_EXTERNAL_MASK);
    }
    frame = &s_stack[s_depth];
    frame->site = site;
    frame->start_envelope = envelope;
    frame->start_body = body;
    frame->tick = CpTick();
    frame->owner = CpOwner();
    frame->measured = (uint8_t)measured;
    frame->inclusive_depth = 1U;
    s_depth++;
    if (measured) maximum(&s_max_depth, s_depth);
    /* Fixed CP_MAX_DEPTH bound; no site lookup or unbounded traversal. */
    for (i = 0U; i + 1U < s_depth; i++)
        maximum(&s_stack[i].inclusive_depth, s_depth - i);
    frame->enter_bookkeeping = (uint32_t)(CpCycles() - body);
}

void Cp_Exit(uint32_t site)
{
    uint32_t exit_start = CpCycles();
    uint32_t body_end, envelope_end, accounting_start, measured;
    uint32_t body, envelope, exit_overhead, was_outer, start_envelope, start_body, tick;
    CpFrame *frame;
    CpSiteStats *stats;
    if (CpIsr()) { reject(CP_FAULT_CONTEXT); return; }
    if (!s_depth)
    {
        reject(CP_FAULT_PAIR);
        return;
    }
    if (s_depth > CP_MAX_DEPTH)
    {
        s_depth--;
        CpRawExit();
        return;
    }
    frame = &s_stack[s_depth - 1U];
    if (frame->site != site || site >= CP_SITE_COUNT) reject(CP_FAULT_PAIR);
    measured = frame->measured;
    if (measured && (frame->owner != CpOwner() || !CpMask())) reject(CP_FAULT_CONTEXT);
    /* Consume metadata while the mask still protects the static frame. Only
     * timestamps needed across unmasking remain in local scalars, avoiding a
     * whole-frame stack copy on the small existing RC task stack. */
    start_envelope = frame->start_envelope;
    start_body = frame->start_body;
    tick = frame->tick;
    if (!measured) increment(&s_startup_pairs);
    else if (!s_faults && site < CP_SITE_COUNT)
    {
        stats = &s_stats[site];
        increment(&stats->hits);
        maximum(&stats->max_enter_envelope_cycles, (uint32_t)(start_body - start_envelope));
        maximum(&stats->max_enter_bookkeeping_cycles, frame->enter_bookkeeping);
        maximum(&stats->max_inclusive_depth, frame->inclusive_depth);
    }
    s_depth--;
    was_outer = (uint32_t)(s_depth == 0U);
    body_end = CpCycles();
    body = (uint32_t)(body_end - start_body);
    exit_overhead = (uint32_t)(body_end - exit_start);
    CpRawExit();
    envelope_end = CpCycles();
    envelope = (uint32_t)(envelope_end - start_envelope);
    /* An unrelated higher-priority task can run before this accounting lock.
     * No pointer into the reused frame stack is read after unmasking. */
    CpRawEnter();
    accounting_start = CpCycles();
    if (measured)
    {
        if (!CpClockEnabled() || CpClock() != s_clock) reject(CP_FAULT_CLOCK);
        /* One modulo wrap is supported. Debugger halt/multiple wraps with a
         * stalled independent tick are unobservable and invalidate the run. */
        if (!body || body > s_clock || envelope > s_clock ||
            (uint32_t)(CpTick() - tick) > 1000U || body > envelope)
            reject(CP_FAULT_DURATION);
        if (!s_faults && site < CP_SITE_COUNT)
        {
            stats = &s_stats[site];
            maximum(&stats->max_body_cycles, body);
            maximum(&stats->max_envelope_cycles, envelope);
            maximum(&stats->max_exit_envelope_cycles, (uint32_t)(envelope_end - body_end));
            maximum(&stats->max_exit_bookkeeping_cycles, exit_overhead);
            if (was_outer)
            {
                increment(&s_outer_hits);
                maximum(&s_outer_body, body);
                maximum(&s_outer_envelope, envelope);
            }
        }
    }
    maximum(&s_accounting_body, (uint32_t)(CpCycles() - accounting_start));
    CpRawExit();
}

void Cp_PublishSnapshot(void)
{
    uint32_t i, start = CpCycles();
    if (CpIsr()) { reject(CP_FAULT_CONTEXT); return; }
    if (!CpRunning()) return;
    if (s_depth || CpMask())
    {
        reject(CP_FAULT_ACTIVE_SNAPSHOT);
        return;
    }
    /* Valid writers are tasks. Scheduling suspension copies static stats without
     * a long IRQ mask. Its scheduling delay is recorded separately. */
    CpSuspend();
    g_cp_snapshot.complete = 0U;
    CpBarrier();
    g_cp_snapshot.version = 1U;
    g_cp_snapshot.catalog = g_cp_sites;
    g_cp_snapshot.core_clock_hz = s_clock;
    g_cp_snapshot.site_count = CP_SITE_COUNT;
    g_cp_snapshot.measurement_rc_stack_bytes = 1024U;
    g_cp_snapshot.current_depth = s_depth;
    g_cp_snapshot.max_depth = s_max_depth;
    g_cp_snapshot.startup_pairs = s_startup_pairs;
    g_cp_snapshot.outer_hits = s_outer_hits;
    g_cp_snapshot.max_outer_body_cycles = s_outer_body;
    g_cp_snapshot.max_outer_envelope_cycles = s_outer_envelope;
    g_cp_snapshot.max_outer_body_us_ceil = microseconds(s_outer_body);
    g_cp_snapshot.max_outer_envelope_us_ceil = microseconds(s_outer_envelope);
    g_cp_snapshot.max_accounting_body_cycles = s_accounting_body;
    for (i = 0U; i < CP_SITE_COUNT; i++)
    {
        g_cp_snapshot.sites[i] = s_stats[i];
        g_cp_snapshot.sites[i].max_body_us_ceil = microseconds(s_stats[i].max_body_cycles);
        g_cp_snapshot.sites[i].max_envelope_us_ceil = microseconds(s_stats[i].max_envelope_cycles);
    }
    increment(&s_snapshot_sequence);
    g_cp_snapshot.sequence = s_snapshot_sequence;
    maximum(&s_snapshot_envelope, (uint32_t)(CpCycles() - start));
    g_cp_snapshot.max_snapshot_envelope_cycles = s_snapshot_envelope;
    g_cp_snapshot.faults = s_faults;
    g_cp_snapshot.valid = (uint32_t)(s_faults == 0U && s_clock != 0U && s_outer_hits != 0U);
    CpBarrier();
    g_cp_snapshot.complete = 1U;
    CpResume();
}
