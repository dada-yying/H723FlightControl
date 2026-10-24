#ifndef CRITICAL_PROFILER_H
#define CRITICAL_PROFILER_H
#include <stdint.h>
#ifndef CP_SITE_COUNT
#include "cp_sites.h"
#endif
#define CP_MAX_DEPTH 16U
#define CP_FAULT_DEPTH 1U
#define CP_FAULT_PAIR 2U
#define CP_FAULT_CLOCK 4U
#define CP_FAULT_CONTEXT 8U
#define CP_FAULT_EXTERNAL_MASK 16U
#define CP_FAULT_DURATION 32U
#define CP_FAULT_COUNTER 64U
#define CP_FAULT_SITE 128U
#define CP_FAULT_ACTIVE_SNAPSHOT 256U
typedef struct
{
    const char *file;
    const char *function;
    uint32_t entry_line;
} CpSiteDescriptor;
extern const CpSiteDescriptor g_cp_sites[CP_SITE_COUNT];
typedef struct
{
    uint32_t hits;
    uint32_t max_body_cycles, max_envelope_cycles;
    uint32_t max_enter_envelope_cycles, max_exit_envelope_cycles;
    uint32_t max_enter_bookkeeping_cycles, max_exit_bookkeeping_cycles;
    uint32_t max_inclusive_depth;
    uint32_t max_body_us_ceil, max_envelope_us_ceil;
} CpSiteStats;
typedef struct
{
    uint32_t version, sequence, complete, valid, faults;
    uint32_t core_clock_hz, site_count, current_depth, max_depth, startup_pairs;
    uint32_t outer_hits, max_outer_body_cycles, max_outer_envelope_cycles;
    uint32_t max_outer_body_us_ceil, max_outer_envelope_us_ceil;
    uint32_t max_accounting_body_cycles, max_snapshot_envelope_cycles;
    uint32_t measurement_rc_stack_bytes;
    const CpSiteDescriptor *catalog;
    CpSiteStats sites[CP_SITE_COUNT];
} CpSnapshot;
/* Read-only Watch snapshot. Read sequence/complete before and after copying.
 * hits==0 means NOT MEASURED; valid==0 rejects the whole run. Body is the
 * known-masked sampled interval, excluding port boundary instructions.
 * Envelope includes possible preemption. Never subtract guessed overhead. */
extern volatile CpSnapshot g_cp_snapshot;
void Cp_Enter(uint32_t site);
void Cp_Exit(uint32_t site);
void Cp_PublishSnapshot(void);
#endif
