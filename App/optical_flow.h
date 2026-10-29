#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#define OPTICAL_FLOW_FRAME_SIZE 27U
typedef struct {
    uint32_t time_ms, distance;
    uint32_t received_tick, rx_count, age_ms;
    int16_t flow_vel_x, flow_vel_y;
    uint16_t reserved2;
    uint8_t head, dev_id, sys_id, msg_id, seq, len;
    uint8_t strength, precision, tof_status, reserved1;
    uint8_t flow_quality, flow_status, checksum, has_sample;
    uint8_t raw[OPTICAL_FLOW_FRAME_SIZE];
} OpticalFlowData_t;

/* Observation only, in the module's unverified X/Y axes. No gyro compensation
 * or world-frame transform; velocity is 0.01 * flow * accepted slant range. */
#define OPTICAL_FLOW_FILTER_SAMPLE_MS 10U
#define OPTICAL_FLOW_FILTER_FRESH_MS 30U
#define OPTICAL_FLOW_FILTER_RESET_MS 50U
#define OPTICAL_FLOW_FILTER_READY_FRAMES 3U
#define OPTICAL_FLOW_RANGE_MIN_MM 80U
#define OPTICAL_FLOW_RANGE_MAX_MM 8000U
#define OPTICAL_FLOW_RANGE_TOLERANCE_M 0.05f
#define OPTICAL_FLOW_RANGE_MAX_SLEW_M_S 2.0f
typedef enum {
    OF_FILTER_NO_SAMPLE = 1U << 0,
    OF_FILTER_STALE = 1U << 1,
    OF_FILTER_TOF_STATUS = 1U << 2,
    OF_FILTER_RANGE_BOUNDS = 1U << 3,
    OF_FILTER_RANGE_JUMP = 1U << 4,
    OF_FILTER_FLOW_STATUS = 1U << 5,
    OF_FILTER_TIMING = 1U << 6,
    OF_FILTER_DUPLICATE = 1U << 7,
    OF_FILTER_WARMUP = 1U << 8,
    OF_FILTER_TRANSPORT = 1U << 9
} OpticalFlowFilterFlag_t;
typedef struct {
    float flow_x_filtered, flow_y_filtered; /* cm/s at 1 m, not compensated. */
    float range_accepted_m, range_innovation_m;
    float velocity_x_m_s, velocity_y_m_s; /* Before the velocity low-pass. */
    float velocity_x_filtered_m_s, velocity_y_filtered_m_s;
    uint32_t age_ms, range_age_ms, rx_count, update_count;
    uint16_t dt_ms, flags;
    uint8_t valid, range_valid, accepted_streak;
} OpticalFlowEstimate_t;
typedef struct {
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
#define OPTICAL_FLOW_RX_IT  1U
/* Watch-only diagnostics. Never used to decide whether a sample is accepted. */
typedef struct {
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
extern volatile OpticalFlowData_t g_optical_flow_data;
extern volatile OpticalFlowStats_t g_optical_flow_stats;
extern volatile OpticalFlowDiag_t g_optical_flow_diag;
void OpticalFlow_Init(HAL_StatusTypeDef status);
void OpticalFlow_SetTaskHandle(TaskHandle_t handle);
HAL_StatusTypeDef OpticalFlow_Start(void);
void OpticalFlow_ProcessPending(void);
void OpticalFlow_GetData(OpticalFlowData_t *data);
/* Raw and derived data belong to the same published frame. Getters never step
 * the filters, including when BlackboxTask samples at 1 kHz. */
void OpticalFlow_GetSnapshot(OpticalFlowData_t *data, OpticalFlowEstimate_t *estimate);
void OpticalFlow_OnRxEvent(UART_HandleTypeDef *uart);
void OpticalFlow_OnRxByte(UART_HandleTypeDef *uart);
void OpticalFlow_OnError(UART_HandleTypeDef *uart);
#endif
