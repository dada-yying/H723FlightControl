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
typedef struct {
    uint32_t byte_count, frame_count, checksum_errors, length_errors;
    uint32_t queue_overflows, uart_errors, restart_count, restart_failures;
    uint32_t last_uart_error, task_count;
    uint32_t heap_free_bytes, heap_min_free_bytes;
    uint32_t optical_stack_free_words, default_stack_free_words, flight_stack_free_words;
    HAL_StatusTypeDef last_status;
    uint8_t initialized, receiving;
} OpticalFlowStats_t;
extern volatile OpticalFlowData_t g_optical_flow_data;
extern volatile OpticalFlowStats_t g_optical_flow_stats;
void OpticalFlow_Init(HAL_StatusTypeDef status);
void OpticalFlow_SetTaskHandle(TaskHandle_t handle);
HAL_StatusTypeDef OpticalFlow_Start(void);
void OpticalFlow_ProcessPending(void);
void OpticalFlow_GetData(OpticalFlowData_t *data);
void OpticalFlow_OnRxEvent(UART_HandleTypeDef *uart);
void OpticalFlow_OnError(UART_HandleTypeDef *uart);
#endif
