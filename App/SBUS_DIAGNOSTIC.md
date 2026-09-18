# Temporary SBUS bench diagnostic

Current default: `RC_SBUS_DIAGNOSTIC=0`, continuous HAL DMA reception with UART5 RX inversion explicitly disabled. The bench comparison found 332 frames and zero UART errors without inversion, versus zero frames with inversion. The procedure below is only for explicitly re-enabling temporary diagnostics; normal firmware does not stop after ten seconds and publishes RC targets for valid frames.

`RC_SBUS_DIAGNOSTIC=1` in rc_sbus.h selects raw UART5 interrupt capture instead of HAL RX DMA. Debug version: `0x00030001`.

On reset, RcTask captures inverted RX for 5 seconds, non-inverted RX for 5 seconds, then disables UART5 and freezes the captures. IMU and USART1 VOFA remain running. All RC targets and `valid` remain zero even when frames decode. This is not an operational control firmware.

Run continuously for at least 11 seconds WITHOUT breakpoints or single stepping, then halt and inspect:

- `g_rc_sbus_diag.phase`: 2 means capture complete; 0 inverted; 1 non-inverted; 3 not started.
- `g_rc_sbus_diag.kernel_clock_hz`: clock calculated from RCC configuration.
- `g_rc_sbus_diag.capture[0]`: inverted results.
- `g_rc_sbus_diag.capture[1]`: non-inverted results.
- For each capture: byte_count, error_count, clean_header_count, frame_count, overrun_count, cr1, cr2, brr, last_frame.

`bytes` and `errors` are paired circular buffers holding the last 512 bytes. If byte_count exceeds 512, oldest retained index is byte_count modulo 512. Per-byte error bits use ISR layout: PE=1, FE=2, NE=4, ORE=8, not HAL ErrorCode layout. error_count counts error IRQ observations, including ones without RXNE; byte_count counts actual RDR reads. overrun_count counts software capture-consumer overflow events. A clean 0x0F is only a candidate header; frame_count additionally requires 25 clean bytes and a supported footer (00, 04, 14, 24, 34, 08). No SBUS checksum exists; frame_count alone is not proof of correct channels. Inspect last_frame and channel response too.

Each phase resets receiver enable, flushes RDR and clears errors. A small startup transient is possible when enabling mid-byte. Diagnose sustained rates, not one startup error. Errors are not allowed into frame parsing. No automatic selection of polarity or activation of RC output occurs.

To repeat, reset. To restore the original DMA path, set RC_SBUS_DIAGNOSTIC to 0 and rebuild; confirm selected polarity separately. The footer compatibility change remains in both modes. CubeMX .ioc is deliberately unchanged: this is temporary software instrumentation, not a new permanent peripheral configuration.

Build using MDK-ARM/build_ws2812.ps1 -OutputDirectory sbus-raw-diagnostic. Hardware reception has not been verified by the build. Existing ErrorCallback breakpoints do not apply to this raw diagnostic path.
