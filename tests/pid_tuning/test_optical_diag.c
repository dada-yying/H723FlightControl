/* Uses the existing host HAL and the actual receiver/telemetry sources. */
#define main optical_existing_main
#include "test_optical_flow.c"
#undef main

int main(void)
{
    uint8_t frame[27], bad[27];
    unsigned i;
    uint32_t events, count;
    boot(); sample(frame);
    assert(g_optical_flow_diag.version == 0x00010002U);
    assert(g_optical_flow_diag.nonzero_frame_count == 0);
    /* Nonzero, then zero: the ordinary data is current, the diagnostic latches. */
    deliver(frame, 27, 1);
    assert(g_optical_flow_diag.valid_frame_count == 1);
    assert(g_optical_flow_diag.nonzero_frame_count == 1);
    assert(g_optical_flow_diag.published_nonzero_count == 1);
    assert(g_optical_flow_diag.rx_mode_valid_frame_count[0] == 1);
    assert(g_optical_flow_diag.rx_mode_raw_nonzero_frame_count[0] == 1);
    assert(g_optical_flow_diag.raw_decode_mismatch_count == 0);
    assert(g_optical_flow_diag.last_nonzero_x == -32768);
    assert(g_optical_flow_diag.last_nonzero_y == 32767);
    assert(!memcmp((const void *)g_optical_flow_diag.last_nonzero_raw, frame, 27));
    memset(frame + 18, 0, 4); frame[4]++; checksum(frame, 27);
    test_tick = 10; deliver(frame, 27, 1);
    assert(g_optical_flow_data.flow_vel_x == 0 && g_optical_flow_data.flow_vel_y == 0);
    assert(g_optical_flow_diag.nonzero_frame_count == 1);
    assert(g_optical_flow_diag.last_nonzero_x == -32768);
    assert(!memcmp((const void *)g_optical_flow_diag.last_valid_raw, frame, 27));
    assert(s_optical_isr_diag.rx_max_interval_ms == 10);
    assert(g_optical_flow_diag.task_max_interval_ms == 10);
    assert(s_optical_isr_diag.queue_high_water_bytes == 27);
    frame[4] = 0; checksum(frame, 27); deliver(frame, 27, 1);
    assert(!g_optical_flow_diag.seq_discontinuity_count); /* FE -> FF -> 00 */
    frame[4] = 3; checksum(frame, 27); deliver(frame, 27, 1);
    deliver(frame, 27, 1);
    assert(g_optical_flow_diag.seq_discontinuity_count == 2);
    assert(g_optical_flow_diag.duplicate_seq_count == 1);
    frame[1]++; frame[4] = 100; checksum(frame, 27); deliver(frame, 27, 1);
    assert(g_optical_flow_diag.seq_discontinuity_count == 2); /* different device */
    count = g_optical_flow_diag.valid_frame_count;
    memcpy(bad, frame, 27); bad[26] ^= 1; deliver(bad, 27, 1);
    memcpy(bad, frame, 27); bad[3] = 0x52; checksum(bad, 27); deliver(bad, 27, 1);
    assert(g_optical_flow_diag.valid_frame_count == count);
    events = s_optical_isr_diag.rx_event_count;
    OpticalFlow_OnRxEvent(&huart5); assert(s_optical_isr_diag.rx_event_count == events);
    huart10.ErrorCode = 8; OpticalFlow_OnError(&huart10);
    OpticalFlow_OnRxEvent(&huart10); assert(s_optical_isr_diag.rx_ignored_count == 1);
    OpticalFlow_ProcessPending(); dma_write = 0;
    assert(g_optical_flow_diag.last_nonzero_x == -32768);
    assert(g_optical_flow_diag.restart_trace_count == s_optical_isr_diag.dma_trace_count);
    frame[4] = 222; checksum(frame, 27); deliver(frame, 27, 1);
    assert(g_optical_flow_diag.seq_discontinuity_count == 2); /* restart resets baseline */

    boot(); sample(frame);
    for (i = 0; i < 3; i++) {
        test_tick = i * 10; frame[4] = (uint8_t)i;
        frame[18] = (i == 1 ? 20 : 0); frame[19] = frame[20] = frame[21] = 0;
        checksum(frame, 27); deliver(frame, 27, 0);
    }
    OpticalFlow_ProcessPending();
    assert(g_optical_flow_data.flow_vel_x == 0);
    assert(g_optical_flow_diag.nonzero_frame_count == 1);
    assert(g_optical_flow_diag.published_nonzero_count == 1);
    assert(g_optical_flow_diag.last_nonzero_x == 20);
    assert(g_optical_flow_diag.last_nonzero_seq == 1);
    assert(g_optical_flow_diag.last_nonzero_rx_tick == 10);
    assert(s_optical_isr_diag.queue_high_water_bytes == 81);
    assert(g_optical_flow_diag.task_max_batch_frames == 3);
    assert(g_optical_flow_diag.max_publish_age_ms == 20);
    assert(g_optical_flow_diag.task_max_interval_ms == 20);

    /* A Y-only nonzero frame must be latched, with X updated to zero. */
    frame[4] = 3; frame[18] = frame[19] = 0;
    frame[20] = 0xCE; frame[21] = 0xFF; checksum(frame, 27);
    deliver(frame, 27, 1);
    assert(g_optical_flow_diag.nonzero_frame_count == 2);
    assert(g_optical_flow_diag.last_nonzero_x == 0 && g_optical_flow_diag.last_nonzero_y == -50);
    /* Diagnostic count wrapping must preserve circular indexing. */
    s_optical_isr_diag.dma_trace_count = UINT32_MAX - 9U;
    deliver(frame, 27, 1);
    assert(s_optical_isr_diag.dma_trace_count == 17U);
    for (i = 0; i < 27; ++i)
        assert(s_optical_isr_diag.dma_trace[(uint32_t)(UINT32_MAX - 9U + i) % 512U] == frame[i]);

    boot(); sample(frame);
    for (i = 0; i < 45; ++i) deliver(frame, 27, 1);
    count = s_optical_isr_diag.dma_trace_count;
    assert(count == 45U * 27 && count == s_optical_isr_stats.byte_count);
    for (i = count - OPTICAL_FLOW_DIAG_TRACE_SIZE; i < count; ++i)
        assert(s_optical_isr_diag.dma_trace[i % OPTICAL_FLOW_DIAG_TRACE_SIZE] == frame[i % 27]);
    assert(s_optical_isr_diag.dma_last_chunk_begin + s_optical_isr_diag.dma_last_chunk_size == count);
    assert(!(s_optical_isr_diag.dma_capture_seq & 1U));
    assert(!(g_optical_flow_diag.frame_capture_seq & 1U));
    assert(s_optical_isr_diag.rx_empty_count > 0); /* duplicate IDLEs */
    for (i = 0; i < 40; ++i) deliver(frame, 27, 0);
    assert(s_optical_isr_diag.queue_high_water_bytes == 1024);
    assert(s_optical_isr_stats.queue_overflows == 1);
    assert(!(s_optical_isr_diag.dma_capture_seq & 1U));

    /* Tick zero, wrap, and a validated frame discarded by a pending restart. */
    boot(); sample(frame); test_tick = 0xFFFFFFF0U;
    OpticalFlow_ProcessPending(); deliver(frame, 27, 1);
    g_optical_flow_diag.task_max_interval_ms = 0; /* test reset of maximum only */
    test_tick = 0x10; deliver(frame, 27, 1);
    assert(s_optical_isr_diag.rx_max_interval_ms == 32);
    assert(g_optical_flow_diag.task_max_interval_ms == 32);
    memcpy(of_parse, frame, 27); of_restart = 1;
    count = s_optical_task_stats.frame_count;
    OF_Publish(test_tick);
    assert(g_optical_flow_diag.valid_frame_count == 3);
    assert(g_optical_flow_diag.nonzero_frame_count == 3);
    assert(g_optical_flow_diag.published_nonzero_count == 2);
    assert(g_optical_flow_diag.publish_discard_count == 1);
    assert(s_optical_task_stats.frame_count == count);
    OpticalFlow_Init(HAL_OK);
    assert(s_optical_isr_diag.dma_trace_count == 0 && g_optical_flow_diag.nonzero_frame_count == 0);
    assert(test_critical_depth == 0);
    /* Same parser through UART byte interrupts, with live mode transitions. */
    boot(); sample(frame); test_rx_it_status = HAL_OK; test_rx_it_calls = 0;
    OpticalFlow_RequestRxMode(OPTICAL_FLOW_RX_IT);
    OpticalFlow_ProcessPending();
    assert(g_optical_flow_diag.rx_mode == OPTICAL_FLOW_RX_IT);
    assert(test_rx_it_calls == 1 && test_rx_it_buffer == &of_it_byte);
    OpticalFlow_OnRxEvent(&huart10); /* delayed DMA event cannot enter IT queue */
    assert(s_optical_isr_diag.rx_ignored_count == 1);
    OpticalFlow_OnRxByte(&huart5);
    for (i = 0; i < 27; ++i) {
        *test_rx_it_buffer = frame[i]; OpticalFlow_OnRxByte(&huart10);
    }
    OpticalFlow_ProcessPending();
    assert(g_optical_flow_data.flow_vel_x == -32768 && g_optical_flow_data.flow_vel_y == 32767);
    assert(s_optical_isr_diag.it_rx_byte_count == 27 && test_rx_it_calls == 28);
    assert(g_optical_flow_diag.rx_mode_valid_frame_count[1] == 1);
    assert(g_optical_flow_diag.rx_mode_raw_nonzero_frame_count[1] == 1);
    assert(g_optical_flow_diag.rx_mode_nonzero_frame_count[1] == 1);
    assert(g_optical_flow_diag.raw_decode_mismatch_count == 0);
    assert(!memcmp((const void *)g_optical_flow_diag.last_valid_raw, frame, 27));
    /* A rejected rearm recovers in task context without publishing partial data. */
    test_rx_it_status = HAL_BUSY; *test_rx_it_buffer = frame[0]; OpticalFlow_OnRxByte(&huart10);
    assert(s_optical_isr_diag.it_rearm_failures == 1 && of_restart);
    test_rx_it_status = HAL_OK; OpticalFlow_ProcessPending();
    assert(!of_restart && s_optical_task_stats.receiving);
    OpticalFlow_RequestRxMode(100);
    OpticalFlow_ProcessPending(); assert(g_optical_flow_diag.rx_mode == 1);
    OpticalFlow_RequestRxMode(0);
    OpticalFlow_ProcessPending(); dma_write = 0;
    assert(g_optical_flow_diag.rx_mode == 0 && g_optical_flow_diag.rx_mode_switch_count == 2);
    OpticalFlow_OnRxByte(&huart10); /* delayed IT event cannot enter DMA queue */
    deliver(frame, 27, 1);
    assert(g_optical_flow_diag.rx_mode_valid_frame_count[0] == 1);
    assert(g_optical_flow_diag.rx_mode_valid_frame_count[1] == 1);
    assert(g_optical_flow_diag.published_nonzero_count == 2);
    puts("PASS: optical diagnostics, nonzero latches, trace wrap, batching, restart, sequence and tick wrap");
    puts("PASS: direct UART IT transport, rearm recovery, mode counters and DMA/IT callback isolation");
    return 0;
}
