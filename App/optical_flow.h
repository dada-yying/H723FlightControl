#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#define OPTICAL_FLOW_FRAME_SIZE 27U
typedef struct
{
    uint32_t time_ms, distance;
    uint32_t received_tick, rx_count, age_ms;
    int16_t flow_vel_x, flow_vel_y;
    uint16_t reserved2;
    uint8_t head, dev_id, sys_id, msg_id, seq, len;
    uint8_t strength, precision, tof_status, reserved1;
    uint8_t flow_quality, flow_status, checksum, has_sample;
    uint8_t raw[OPTICAL_FLOW_FRAME_SIZE];
} OpticalFlowData_t;
typedef struct
{
    uint32_t byte_count, frame_count, checksum_errors, length_errors;
    uint32_t queue_overflows, uart_errors, restart_count, restart_failures;
    uint32_t last_uart_error, task_count;
    uint32_t heap_free_bytes, heap_min_free_bytes;
    uint32_t optical_stack_free_words, default_stack_free_words, flight_stack_free_words;
    HAL_StatusTypeDef last_status;
    uint8_t initialized, receiving;
} OpticalFlowStats_t;
#define OPTICAL_FLOW_DIAG_TRACE_SIZE 512U
#define OPTICAL_FLOW_RX_DMA 0U
#define OPTICAL_FLOW_RX_IT 1U
/* Watch-only diagnostics. Never used to decide whether a sample is accepted. */
typedef struct
{
    uint32_t version;
    uint32_t rx_event_count, rx_ignored_count, rx_empty_count;
    uint32_t rx_last_tick, rx_max_interval_ms;
    uint32_t dma_last_remaining, dma_last_pos;
    uint32_t dma_capture_seq, dma_trace_count;
    uint32_t dma_last_chunk_begin, dma_last_chunk_size;
    uint32_t restart_trace_count;
    uint32_t queue_high_water_bytes;
    uint32_t task_last_tick, task_max_interval_ms, task_max_batch_frames;
    uint32_t max_publish_age_ms;
    uint32_t valid_frame_count, nonzero_frame_count, published_nonzero_count;
    uint32_t publish_discard_count;
    uint32_t seq_discontinuity_count, duplicate_seq_count;
    uint32_t frame_capture_seq;
    uint32_t last_nonzero_sensor_ms, last_nonzero_rx_tick;
    int16_t last_nonzero_x, last_nonzero_y;
    uint8_t last_seq, last_dev_id, last_sys_id, last_nonzero_seq;
    uint8_t last_valid_raw[OPTICAL_FLOW_FRAME_SIZE];
    uint8_t last_nonzero_raw[OPTICAL_FLOW_FRAME_SIZE];
    uint8_t dma_trace[OPTICAL_FLOW_DIAG_TRACE_SIZE];
    /* requested_rx_mode is the only writable diagnostic setting. Task applies
     * it by restarting USART10 reception; mode 0 remains the boot default. */
    uint32_t requested_rx_mode, rx_mode, rx_mode_switch_count;
    uint32_t it_rx_byte_count, it_rearm_failures;
    uint32_t rx_mode_valid_frame_count[2];
    uint32_t rx_mode_nonzero_frame_count[2];
    uint32_t rx_mode_raw_nonzero_frame_count[2];
    uint32_t raw_decode_mismatch_count;
} OpticalFlowDiag_t;
/* Latest data is private; getters return a consistent value copy. */
/* Registered compatibility area: generated task writes heap/stack fields. */
typedef struct
{
    uint32_t heap_free_bytes, heap_min_free_bytes;
    uint32_t optical_stack_free_words, default_stack_free_words, flight_stack_free_words;
} OpticalFlowCompatibilityStats_t;
extern volatile OpticalFlowCompatibilityStats_t g_optical_flow_stats;
void OpticalFlow_RequestRxMode(uint32_t mode);
void OpticalFlow_GetStats(OpticalFlowStats_t *stats);
void OpticalFlow_GetDiagnostics(OpticalFlowDiag_t *diag);
void OpticalFlow_Init(HAL_StatusTypeDef status);
void OpticalFlow_SetTaskHandle(TaskHandle_t handle);
HAL_StatusTypeDef OpticalFlow_Start(void);
void OpticalFlow_ProcessPending(void);
void OpticalFlow_GetData(OpticalFlowData_t *data);
void OpticalFlow_OnRxEvent(UART_HandleTypeDef *uart);
void OpticalFlow_OnRxByte(UART_HandleTypeDef *uart);
void OpticalFlow_OnError(UART_HandleTypeDef *uart);
#endif
