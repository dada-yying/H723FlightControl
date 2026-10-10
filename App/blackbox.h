#ifndef BLACKBOX_H
#define BLACKBOX_H
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#define BLACKBOX_TARGET_HZ 700U
#define BLACKBOX_DATA_BUDGET 150000U

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
    uint32_t target_hz, effective_hz, timer_running, timer_irq_count;
    uint32_t missed_trigger_count, timer_error_count, task_error_count, stack_free_words;
    HAL_StatusTypeDef last_timer_status;
} BlackboxTxStats_t;
extern volatile BlackboxTxStats_t g_blackbox_tx_stats;
void Blackbox_Init(HAL_StatusTypeDef uart_init_status);
HAL_StatusTypeDef Blackbox_StartRecording(TaskHandle_t task);
void Blackbox_OnTaskCreationFailure(void);
void Blackbox_NotifyFromISR(void); /* TIM4 only: next ARR + task notification. */
void Blackbox_ProcessPending(void); /* Sole caller: independent BlackboxTask. */
void Blackbox_Service(void);        /* Same task; recovery/schema on idle wakes. */
void Blackbox_OnUartError(UART_HandleTypeDef *uart);
#endif
