/* Real unchanged heater and BMI source, using the existing host fixture.
 * Synthetic cycles prove nesting/accounting only, NOT board execution time. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#define CP_HOST_TEST
#define CP_ACTUAL_APPLICATION_STUBS
#include "../critical_profiler.c"
#include "cp_sites.c"
typedef struct { const char *file; uint32_t line, site; } HostSiteLine;
#include "host_site_lines.h"
static uint32_t site_for(const char *file, uint32_t line)
{
    unsigned i;
    for (i=0; i<sizeof(host_site_lines)/sizeof(host_site_lines[0]); i++)
        if (host_site_lines[i].line==line && strstr(file,host_site_lines[i].file))
            return host_site_lines[i].site;
    assert(!"Unmapped application critical call");
    return 0;
}
#undef taskENTER_CRITICAL
#undef taskEXIT_CRITICAL
#define taskENTER_CRITICAL() Cp_Enter(site_for(__FILE__,__LINE__))
#define taskEXIT_CRITICAL() Cp_Exit(site_for(__FILE__,__LINE__))
#include "heater_body.inc"
int main(void)
{
    unsigned i, heater_nested=0, bmi_hits=0;
    assert(heater_characterization_main()==0);
    Cp_PublishSnapshot();
    assert(g_cp_snapshot.valid && !s_depth && !h_nesting && !test_critical_depth);
    for (i=0;i<CP_SITE_COUNT;i++)
    {
        if (strstr(g_cp_sites[i].file,"bmi088_heater.c") &&
            g_cp_snapshot.sites[i].hits && g_cp_snapshot.sites[i].max_inclusive_depth>=2)
            heater_nested++;
        if (!strcmp(g_cp_sites[i].function,"Bmi088_GetSnapshotWithFilter"))
            bmi_hits=g_cp_snapshot.sites[i].hits;
    }
    assert(heater_nested==2 && bmi_hits>0 && s_max_depth>=2);
    printf("PASS unchanged heater/BMI: nested heater sites=%u, BMI hits=%u, max_depth=%u\n",
           heater_nested,bmi_hits,s_max_depth);
    return 0;
}
