#include <stdio.h>
#include <string.h>
#include <assert.h>
#define CP_HOST_TEST
#define CP_SITE_COUNT 3U
#include "../critical_profiler.c"
const CpSiteDescriptor g_cp_sites[CP_SITE_COUNT] = {
    {"host", "outer", 1}, {"host", "inner", 2}, {"host", "other", 3}
};
static void reset(void)
{
    memset(s_stack, 0, sizeof(s_stack)); memset(s_stats, 0, sizeof(s_stats));
    memset((void *)&g_cp_snapshot, 0, sizeof(g_cp_snapshot));
    s_depth = s_faults = s_clock = s_max_depth = s_startup_pairs = 0;
    s_outer_hits = s_outer_body = s_outer_envelope = s_accounting_body = 0;
    s_snapshot_envelope = s_snapshot_sequence = 0;
    h_cycles = h_tick = h_isr = h_mask = h_nesting = h_suspended = 0;
    h_entries = h_exits = h_pre_delay = h_post_delay = h_frozen = 0;
    h_clock = 480000000U; h_running = h_enabled = 1; h_owner = 1;
    h_post_exit_hook = 0;
}
static void pair(uint32_t site) { Cp_Enter(site); h_cycles += 480; Cp_Exit(site); }
static void valid(void)
{
    Cp_PublishSnapshot(); assert(g_cp_snapshot.valid && g_cp_snapshot.complete);
    assert(!s_faults && !s_depth && !h_nesting && !h_suspended);
}
static void invalid(uint32_t fault)
{
    Cp_PublishSnapshot(); assert(s_faults & fault); assert(!g_cp_snapshot.valid);
}
static void preempting_task(void)
{
    h_owner = 2; pair(2); h_owner = 1;
}
int main(int argc, char **argv)
{
    uint32_t baseline, baseline_env, i;
    assert(argc == 2); reset();
    if (!strcmp(argv[1], "nested"))
    {
        Cp_Enter(0); h_cycles += 100; Cp_Enter(1); h_cycles += 200;
        Cp_Exit(1); h_cycles += 100; Cp_Exit(0); valid();
        assert(s_stats[0].hits == 1 && s_stats[1].hits == 1 && s_outer_hits == 1);
        assert(s_outer_body == s_stats[0].max_body_cycles);
        assert(s_stats[0].max_body_cycles > s_stats[1].max_body_cycles);
        assert(s_stats[0].max_inclusive_depth == 2 && s_max_depth == 2);
        assert(h_entries == h_exits && h_entries == 4); /* 2 application + 2 accounting */
        assert(g_cp_snapshot.sites[2].hits == 0); /* NOT MEASURED */
    }
    else if (!strcmp(argv[1], "wrap"))
    {
        pair(0); baseline = s_outer_body; baseline_env = s_outer_envelope;
        reset(); h_cycles = UINT32_MAX - 20U; pair(0); valid();
        assert(s_outer_body == baseline && s_outer_envelope == baseline_env);
        assert(g_cp_snapshot.sites[0].max_body_us_ceil == (baseline + 479U)/480U);
    }
    else if (!strcmp(argv[1], "preemption_envelope"))
    {
        pair(0); baseline = s_outer_body; baseline_env = s_outer_envelope;
        reset(); h_pre_delay = 1000; h_post_delay = 2000; pair(0); valid();
        assert(s_outer_body == baseline && s_outer_envelope == baseline_env + 3000U);
        assert(s_stats[0].max_enter_envelope_cycles >= 1000);
        assert(s_stats[0].max_exit_envelope_cycles >= 2000);
        assert(s_accounting_body && s_stats[0].max_enter_bookkeeping_cycles);
    }
    else if (!strcmp(argv[1], "post_exit_task"))
    {
        Cp_Enter(0); h_cycles += 100; h_post_exit_hook = preempting_task; Cp_Exit(0); valid();
        assert(s_stats[0].hits == 1 && s_stats[2].hits == 1 && s_outer_hits == 2);
    }
    else if (!strcmp(argv[1], "startup"))
    {
        h_running = 0; pair(0); assert(s_startup_pairs == 1 && !s_stats[0].hits);
        h_running = 1; pair(0); valid(); assert(s_stats[0].hits == 1);
    }
    else if (!strcmp(argv[1], "unbalanced"))
    {
        pair(0); valid(); Cp_Exit(0); invalid(CP_FAULT_PAIR);
        assert(!h_nesting); pair(1); assert(!s_stats[1].hits);
    }
    else if (!strcmp(argv[1], "wrong_pair"))
    {
        Cp_Enter(0); Cp_Exit(1); invalid(CP_FAULT_PAIR); assert(!h_nesting);
    }
    else if (!strcmp(argv[1], "depth_overflow"))
    {
        for (i=0;i<CP_MAX_DEPTH+2;i++) Cp_Enter(0);
        for (i=0;i<CP_MAX_DEPTH+2;i++) Cp_Exit(0);
        invalid(CP_FAULT_DEPTH); assert(!h_nesting && h_entries==h_exits);
    }
    else if (!strcmp(argv[1], "clock_faults"))
    {
        h_enabled=0; pair(0); invalid(CP_FAULT_CLOCK);
        reset(); Cp_Enter(0); h_clock=240000000; Cp_Exit(0); invalid(CP_FAULT_CLOCK);
        reset(); h_frozen=1; Cp_Enter(0); Cp_Exit(0); invalid(CP_FAULT_DURATION);
    }
    else if (!strcmp(argv[1], "context_mask"))
    {
        h_isr=1; Cp_Enter(0); Cp_Exit(0); invalid(CP_FAULT_CONTEXT); assert(!h_entries);
        reset(); h_mask=0x20; pair(0); invalid(CP_FAULT_EXTERNAL_MASK);
        reset(); Cp_Enter(0); h_owner=2; Cp_Exit(0); invalid(CP_FAULT_CONTEXT);
    }
    else if (!strcmp(argv[1], "counter_duration_site"))
    {
        s_stats[0].hits=UINT32_MAX; pair(0); invalid(CP_FAULT_COUNTER);
        reset(); Cp_Enter(0); h_cycles+=480000001U; Cp_Exit(0); invalid(CP_FAULT_DURATION);
        reset(); pair(CP_SITE_COUNT); invalid(CP_FAULT_SITE);
    }
    else if (!strcmp(argv[1], "active_snapshot"))
    {
        pair(0); valid(); Cp_Enter(0); Cp_PublishSnapshot();
        assert(!g_cp_snapshot.valid); Cp_Exit(0); invalid(CP_FAULT_ACTIVE_SNAPSHOT);
    }
    else return 2;
    puts("PASS profiler host invariant (not target timing)");
    return 0;
}
