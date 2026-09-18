#ifndef RC_SBUS_H
#define RC_SBUS_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC_SBUS_CHANNEL_COUNT 16U
/* TEMPORARY bench diagnostic: all RC targets remain zero. Set 0 to restore DMA. */
#define RC_SBUS_DIAGNOSTIC 0
#define RC_SBUS_CAPTURE_SIZE 512U
typedef struct
{
    uint32_t byte_count;
    uint32_t error_count;
    uint32_t clean_header_count;
    uint32_t frame_count;
    uint32_t overrun_count;
    uint32_t cr1;
    uint32_t cr2;
    uint32_t brr;
    uint8_t last_frame[25]; /* Last complete, UART-error-free candidate frame. */
    uint8_t bytes[RC_SBUS_CAPTURE_SIZE];
    /* Per-byte ISR bits: bit0 PE, bit1 FE, bit2 NE, bit3 ORE. NOT HAL error codes. */
    uint8_t errors[RC_SBUS_CAPTURE_SIZE];
} RcSbusCapture_t;
typedef struct
{
    uint32_t phase; /* 0=inverted, 1=non-inverted, 2=finished; 3=not started */
    uint32_t kernel_clock_hz;
    RcSbusCapture_t capture[2];
} RcSbusDiagnostic_t;
extern volatile RcSbusDiagnostic_t g_rc_sbus_diag;
void RcSbus_DiagnosticIRQ(void);

typedef struct
{
    uint16_t channel[RC_SBUS_CHANNEL_COUNT];
    float throttle;
    float roll_target_rad;
    float pitch_target_rad;
    float yaw_rate_target_rad_s;
    uint32_t frame_count;
    uint32_t valid_frame_count;
    uint32_t frame_lost_count;
    uint32_t failsafe_count;
    uint32_t frame_error_count;
    uint32_t rx_overrun_count;
    uint32_t last_valid_tick;
    uint8_t valid;
    uint8_t frame_lost;
    uint8_t failsafe;
    uint8_t ch17;
    uint8_t ch18;
} RcSbusData_t;

extern volatile RcSbusData_t g_rc_sbus_data;

/* Diagnostic counters only: read after running, then halting the target. */
typedef struct
{
    uint32_t version;
    uint32_t task_count;
    uint32_t rx_event_count;
    uint32_t rx_byte_count;
    uint32_t uart_error_count;
    uint32_t last_uart_error;
    uint32_t last_dma_error;
    uint32_t start_count;
    uint32_t restart_count;
    uint32_t start_fail_count;
    uint32_t last_start_status;
    uint32_t dma_address;
    uint32_t last_rx_size;
    uint8_t last_rx[32];
} RcSbusDebug_t;

extern volatile RcSbusDebug_t g_rc_sbus_debug;

void RcSbus_Init(void);
HAL_StatusTypeDef RcSbus_StartReception(void);
void RcSbus_SetTaskHandle(TaskHandle_t task_handle);
void RcSbus_ProcessPending(void);
void RcSbus_CheckTimeout(void);
void RcSbus_GetData(RcSbusData_t *data);

#ifdef __cplusplus
}
#endif

#endif
