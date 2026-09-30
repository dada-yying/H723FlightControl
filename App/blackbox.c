#include "blackbox.h"
#include "vofa.h"
#include "usart.h"
#include <string.h>

#define BLACKBOX_TIMEOUT_MS 5U
/* DMA1 cannot access DTCM. Own three complete cache lines in AXI SRAM. */
__attribute__((section(".blackbox_dma"), aligned(32)))
static uint8_t s_dma_frame[96];
static uint32_t s_state_tick;
volatile BlackboxTxStats_t g_blackbox_tx_stats;

static void Blackbox_SetState(BlackboxTxState_t state)
{
    g_blackbox_tx_stats.state = state;
    g_blackbox_tx_stats.busy = (state == BLACKBOX_SENDING ||
        state == BLACKBOX_RECOVER || state == BLACKBOX_ABORTING);
    g_blackbox_tx_stats.disabled = (state == BLACKBOX_DISABLED);
}

void Blackbox_Init(HAL_StatusTypeDef uart_init_status)
{
    /* Startup only, before the scheduler; never reset an active DMA buffer. */
    (void)memset((void *)&g_blackbox_tx_stats, 0, sizeof(g_blackbox_tx_stats));
    g_blackbox_tx_stats.last_status = uart_init_status;
    g_blackbox_tx_stats.initialized = (uart_init_status == HAL_OK);
    if (uart_init_status != HAL_OK) g_blackbox_tx_stats.error_count++;
    Blackbox_SetState(uart_init_status == HAL_OK ? BLACKBOX_IDLE : BLACKBOX_DISABLED);
}

void Blackbox_Service(void)
{
    HAL_StatusTypeDef status;
    const uint32_t now = HAL_GetTick();
    taskENTER_CRITICAL();
    if ((g_blackbox_tx_stats.state == BLACKBOX_SENDING) &&
        ((uint32_t)(now - s_state_tick) > BLACKBOX_TIMEOUT_MS))
    {
        g_blackbox_tx_stats.timeout_count++;
        Blackbox_SetState(BLACKBOX_RECOVER);
    }
    if ((g_blackbox_tx_stats.state == BLACKBOX_ABORTING) &&
        ((uint32_t)(now - s_state_tick) > BLACKBOX_TIMEOUT_MS))
    {
        g_blackbox_tx_stats.timeout_count++;
        Blackbox_SetState(BLACKBOX_DISABLED);
    }
    if (g_blackbox_tx_stats.state == BLACKBOX_RECOVER)
    {
        s_state_tick = now;
        Blackbox_SetState(BLACKBOX_ABORTING);
        g_blackbox_tx_stats.abort_count++;
        /* HAL may invoke the completion callback synchronously. State must
         * already be ABORTING and must not be overwritten after success. */
        status = HAL_UART_AbortTransmit_IT(&huart7);
        g_blackbox_tx_stats.last_status = status;
        if (status != HAL_OK)
        {
            g_blackbox_tx_stats.error_count++;
            Blackbox_SetState(BLACKBOX_DISABLED);
        }
    }
    taskEXIT_CRITICAL();
}

void Blackbox_SendSnapshot(void)
{
    FlightControlState_t flight;
    RcSbusData_t rc;
    EscPwmState_t esc;
    HAL_StatusTypeDef status;
    Blackbox_Service();
    taskENTER_CRITICAL();
    if (g_blackbox_tx_stats.state != BLACKBOX_IDLE)
    {
        if (g_blackbox_tx_stats.disabled == 0U) g_blackbox_tx_stats.busy_drop_count++;
        taskEXIT_CRITICAL();
        return;
    }
    /* Single task owns submission; no active DMA exists while IDLE. */
    taskEXIT_CRITICAL();
    FlightControl_GetState(&flight);
    RcSbus_GetData(&rc);
    EscPwm_GetState(&esc);
    VOFA_BuildFlightFrame(s_dma_frame, &flight, &rc, &esc);
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
        SCB_CleanDCache_by_Addr((uint32_t *)s_dma_frame, sizeof(s_dma_frame));
    __DSB();
    taskENTER_CRITICAL();
    if (g_blackbox_tx_stats.state != BLACKBOX_IDLE)
    {
        g_blackbox_tx_stats.busy_drop_count++;
        taskEXIT_CRITICAL();
        return; /* An error IRQ during snapshot construction requires recovery. */
    }
    s_state_tick = HAL_GetTick();
    Blackbox_SetState(BLACKBOX_SENDING);
    status = HAL_UART_Transmit_DMA(&huart7, s_dma_frame, VOFA_FLIGHT_FRAME_SIZE);
    g_blackbox_tx_stats.last_status = status;
    if (status == HAL_OK) g_blackbox_tx_stats.submitted_count++;
    else
    {
        g_blackbox_tx_stats.error_count++;
        g_blackbox_tx_stats.last_uart_error = huart7.ErrorCode;
        Blackbox_SetState(BLACKBOX_RECOVER);
    }
    taskEXIT_CRITICAL();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    /* In normal DMA mode HAL calls this from UART TC, not DMA TC. */
    if ((uart == &huart7) && (g_blackbox_tx_stats.state == BLACKBOX_SENDING))
    {
        g_blackbox_tx_stats.completed_count++;
        Blackbox_SetState(BLACKBOX_IDLE);
    }
}

void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *uart)
{
    if ((uart == &huart7) && (g_blackbox_tx_stats.state == BLACKBOX_ABORTING))
        Blackbox_SetState(BLACKBOX_IDLE);
}

void Blackbox_OnUartError(UART_HandleTypeDef *uart)
{
    if ((uart != &huart7) || g_blackbox_tx_stats.disabled) return;
    g_blackbox_tx_stats.error_count++;
    g_blackbox_tx_stats.last_uart_error = uart->ErrorCode;
    g_blackbox_tx_stats.last_status = HAL_ERROR;
    if (g_blackbox_tx_stats.state != BLACKBOX_ABORTING)
        Blackbox_SetState(BLACKBOX_RECOVER);
}
