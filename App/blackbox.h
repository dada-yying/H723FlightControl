#ifndef BLACKBOX_H
#define BLACKBOX_H
#include "stm32h7xx_hal.h"

typedef enum {
    BLACKBOX_DISABLED = 0, BLACKBOX_IDLE, BLACKBOX_SENDING,
    BLACKBOX_RECOVER, BLACKBOX_ABORTING
} BlackboxTxState_t;
typedef struct {
    uint32_t initialized;
    uint32_t busy;
    uint32_t disabled;
    BlackboxTxState_t state;
    uint32_t submitted_count;
    uint32_t completed_count;
    uint32_t busy_drop_count;
    uint32_t error_count;
    uint32_t timeout_count;
    uint32_t abort_count;
    HAL_StatusTypeDef last_status;
    uint32_t last_uart_error;
    uint32_t schema_submitted_count, schema_completed_count, data_completed_count;
    uint32_t bandwidth_skip_count, record_hz, schema_id, payload_size, config_error;
} BlackboxTxStats_t;
extern volatile BlackboxTxStats_t g_blackbox_tx_stats;
void Blackbox_Init(HAL_StatusTypeDef uart_init_status);
void Blackbox_SendSnapshot(void); /* Flight task only; never waits for wire time. */
void Blackbox_Service(void);      /* Also called when IMU notifications stop. */
void Blackbox_OnUartError(UART_HandleTypeDef *uart);
#endif
