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
