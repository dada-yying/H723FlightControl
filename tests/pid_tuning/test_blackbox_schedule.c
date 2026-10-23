/* Production scheduler with sensor and queue fixtures. */
#define main blackbox_fixture_main
#include "test_blackbox.c"
#undef main
int main(void)
{
    unsigned i,before;
    reset();
    assert(g_blackbox_tx_stats.target_hz==1000 && g_blackbox_tx_stats.effective_hz==1000);
    for(i=0;i<100000;i++){assert(htim4.active_autoreload==999);timer_step();}
    assert(s_test_elapsed_us==100000000U && test_dma_calls==0);
    Blackbox_ProcessPending();assert(g_blackbox_tx_stats.data_enqueued_count==1);
    assert(g_blackbox_tx_stats.missed_trigger_count==99999);
    before=test_dma_calls;Blackbox_ProcessPending();assert(test_dma_calls==before);
    reset();for(i=0;i<20000;i++)recording_step();
    assert(s_test_elapsed_us==20000000 && g_blackbox_tx_stats.data_enqueued_count==20000);
    assert(!g_blackbox_tx_stats.bandwidth_skip_count && !g_blackbox_tx_stats.missed_trigger_count);
    assert(g_blackbox_tx_stats.legacy_schema_enqueue_mirror_count==0 && s_schema_size>0);
    assert(!test_dma_source[24+153]); /* No IMU still records snapshots. */
    reset();queue_reject=1;
    for(i=0;i<100;i++)recording_step();
    assert(g_blackbox_tx_stats.busy_drop_count>=100 && !g_blackbox_tx_stats.disabled);
    queue_reject=0;recording_step();assert(g_blackbox_tx_stats.data_enqueued_count==1);
    reset();s_payload_size=484;recording_step();
    assert(test_dma_size==512 && g_blackbox_tx_stats.effective_hz==1000);
    for(i=0;i<3;i++){
        reset();Blackbox_OnTaskCreationFailure();Blackbox_Init(HAL_OK);
        if(i==0)test_timer_init_status=HAL_ERROR;
        if(i==1)test_timer_event_status=HAL_ERROR;
        if(i==2)test_timer_start_status=HAL_ERROR;
        assert(Blackbox_StartRecording((TaskHandle_t)0x700)==HAL_ERROR);
        assert(g_blackbox_tx_stats.disabled && !g_blackbox_tx_stats.timer_running);
    }
    reset();Blackbox_NowUs();test_dwt.CYCCNT=UINT32_MAX-480U;Blackbox_NowUs();
    {uint64_t t=Blackbox_NowUs();test_dwt.CYCCNT+=960U;assert(Blackbox_NowUs()==t+2);}
    puts("PASS 1kHz 100s cadence, ISR only notifies, no replay, no IMU/locked recording, queue overflow, future fields, failures, clock wrap");
    return 0;
}
