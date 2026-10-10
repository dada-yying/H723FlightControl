/* Exercise the real sender/scheduler; fixture provides peripherals and snapshots. */
#define main blackbox_fixture_main
#include "test_blackbox.c"
#undef main

static void data_ready(void)
{
    s_schema_ready=1;s_schema_offset=s_schema_size;s_schema_tick=HAL_GetTick();
}

int main(void)
{
    unsigned i,before,period,stop_before;
    uint8_t held[512];
    reset();
    assert(g_blackbox_tx_stats.target_hz==700 && g_blackbox_tx_stats.effective_hz==700);
    assert(g_blackbox_tx_stats.timer_running && htim4.active_autoreload==1427);
    before=test_dma_calls;
    for(i=0;i<70000;i++) {
        period=htim4.active_autoreload+1U;
        assert(period==1428 || period==1429);
        timer_step();
    }
    assert(s_test_elapsed_us==100000000U && g_blackbox_tx_stats.timer_irq_count==70000);
    assert(test_dma_calls==before); /* ISR performs no snapshots or DMA work. */
    assert(test_notify_task==(TaskHandle_t)0x700);

    reset();data_ready();
    /* No FlightControl_Run or sensor trigger required, including locked/no IMU. */
    for(i=0;i<700;i++)recording_step();
    assert(s_test_elapsed_us==1000000U && g_blackbox_tx_stats.data_completed_count==700);
    assert(!g_blackbox_tx_stats.bandwidth_skip_count && !g_blackbox_tx_stats.missed_trigger_count);
    assert(g_blackbox_tx_stats.record_hz>=699 && g_blackbox_tx_stats.record_hz<=700);
    assert(!sbb_u32(test_dma_source+24+140) && !test_dma_source[24+152]);
    for(i=0;i<4;i++){float motor;memcpy(&motor,test_dma_source+24+64+4*i,4);assert(motor==0);}

    reset();data_ready();
    for(i=0;i<10;i++)timer_step();
    Blackbox_ProcessPending();assert(test_dma_calls==1 && g_blackbox_tx_stats.missed_trigger_count==9);
    memcpy(held,test_dma_source,512);
    Blackbox_ProcessPending();assert(test_dma_calls==1); /* No catch-up or duplicate wake. */
    timer_step();Blackbox_ProcessPending();
    assert(test_dma_calls==1 && g_blackbox_tx_stats.busy_drop_count==1);
    assert(!memcmp(held,test_dma_source,512)); /* In-flight ownership. */
    HAL_UART_TxCpltCallback(&huart7);
    timer_step();Blackbox_ProcessPending();assert(test_dma_calls==2);
    HAL_UART_TxCpltCallback(&huart7);
    test_clean_hook=timer_step;timer_step();Blackbox_ProcessPending();test_clean_hook=NULL;
    assert(g_blackbox_tx_stats.missed_trigger_count==10);
    HAL_UART_TxCpltCallback(&huart7);
    before=test_dma_calls;Blackbox_ProcessPending();assert(test_dma_calls==before);

    reset();data_ready();seed_bmi();
    g_bmi088_data.sample_tick_ms=test_tick;
    for(i=0;i<700;i++)recording_step();
    assert(g_blackbox_tx_stats.data_completed_count==700);
    assert(test_dma_source[24+152] && !test_dma_source[24+153] && !test_dma_source[24+177]);
    assert(sbb_u32(test_dma_source+24+148)==1000U);
    assert(sbb_u32(test_dma_source+16)==1000000U-1428U); /* Time origin: first service. */

    reset();
    /* Production cap calculation under a future maximum-size registry. */
    s_payload_size=484;g_blackbox_tx_stats.effective_hz=BLACKBOX_DATA_BUDGET/(s_payload_size+28U);
    s_period_base=1000000U/g_blackbox_tx_stats.effective_hz;
    s_period_remainder=1000000U%g_blackbox_tx_stats.effective_hz;
    assert(Blackbox_StartRecording((TaskHandle_t)0x700)==HAL_OK);data_ready();
    assert(g_blackbox_tx_stats.effective_hz==292);
    for(i=0;i<292;i++)recording_step();
    assert(s_test_elapsed_us==1000000U && g_blackbox_tx_stats.data_completed_count==292);
    assert(g_blackbox_tx_stats.bandwidth_skip_count==408);
    assert(test_dma_size==512 && (uint64_t)test_dma_size*292<=150000U);

    reset();
    for(i=0;i<14000;i++)recording_step();
    assert(s_test_elapsed_us==20000000U);
    assert(s_schema_ready && g_blackbox_tx_stats.schema_completed_count>100);
    assert(g_blackbox_tx_stats.data_completed_count>13000 && g_blackbox_tx_stats.data_completed_count<14000);
    assert(!g_blackbox_tx_stats.error_count && !g_blackbox_tx_stats.timeout_count);

    reset();Blackbox_OnTaskCreationFailure();
    assert(g_blackbox_tx_stats.disabled && !g_blackbox_tx_stats.timer_running && g_blackbox_tx_stats.task_error_count==1);
    before=test_notify_calls;Blackbox_NotifyFromISR();assert(test_notify_calls==before);
    for(i=0;i<3;i++) {
        reset();Blackbox_OnTaskCreationFailure();Blackbox_Init(HAL_OK);
        if(i==0)test_timer_init_status=HAL_ERROR;
        if(i==1)test_timer_event_status=HAL_ERROR;
        if(i==2)test_timer_start_status=HAL_ERROR;
        assert(Blackbox_StartRecording((TaskHandle_t)0x700)==HAL_ERROR);
        assert(g_blackbox_tx_stats.disabled && !g_blackbox_tx_stats.timer_running && g_blackbox_tx_stats.timer_error_count==1);
    }
    reset();Blackbox_SetState(BLACKBOX_DISABLED);stop_before=test_timer_stop_calls;
    Blackbox_ProcessPending();assert(!g_blackbox_tx_stats.timer_running && test_timer_stop_calls==stop_before+1);
    assert(!test_dma_calls);

    reset();data_ready();
    test_tick=UINT32_MAX-1U;Blackbox_NowUs();test_dwt.CYCCNT=UINT32_MAX-480U;Blackbox_NowUs();
    before=(uint32_t)Blackbox_NowUs();test_dwt.CYCCNT+=960U;
    assert(Blackbox_NowUs()==(uint64_t)before+2U);
    puts("PASS independent 700Hz rational phase, timer-only ISR, coalescing/no replay, stale/locked logging, schema recovery, size cap, local failures and DWT wrap");
    return 0;
}
