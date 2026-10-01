#include "blackbox.h"
#include "vofa.h"
#include "usart.h"
#include <string.h>
#include "blackbox_fields.h"

#define BLACKBOX_TIMEOUT_MS 5U
/* DMA1 cannot access DTCM. Own sixteen complete cache lines in AXI SRAM. */
__attribute__((section(".blackbox_dma"), aligned(32)))
static uint8_t s_dma_frame[512];
static uint32_t s_state_tick;
static uint32_t s_cycle_last, s_sequence, s_schema_tick, s_rate_tick, s_rate_count;
static uint64_t s_cycles, s_data_due, s_schema_due;
static uint16_t s_schema_offset, s_pending_fragment;
static uint8_t s_schema_ready, s_pending_type, s_clock_started;
volatile BlackboxTxStats_t g_blackbox_tx_stats;
static uint64_t Blackbox_NowUs(void)
{
    uint32_t now=DWT->CYCCNT;
    /* FlightControl_Init may reset DWT after Blackbox_Init, before scheduling. */
    if(!s_clock_started){s_cycle_last=now;s_clock_started=1;}
    s_cycles+=(uint32_t)(now-s_cycle_last);s_cycle_last=now;
    return s_cycles/(SystemCoreClock/1000000U);
}
static void Blackbox_Submit(uint16_t size,uint8_t type);
static void Blackbox_SendSchema(uint64_t now);

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
    s_cycles=0;s_cycle_last=DWT->CYCCNT;s_clock_started=0;s_sequence=0;s_schema_offset=0;s_schema_ready=0;
    s_data_due=s_schema_due=0;s_schema_tick=s_rate_tick=HAL_GetTick();s_rate_count=0;
    s_pending_type=0;s_pending_fragment=0;
    if(!Blackbox_CreateSchema()) {
        g_blackbox_tx_stats.config_error=1;Blackbox_SetState(BLACKBOX_DISABLED);return;
    }
    g_blackbox_tx_stats.schema_id=s_schema_id;g_blackbox_tx_stats.payload_size=s_payload_size;
    g_blackbox_tx_stats.initialized = (uart_init_status == HAL_OK);
    if (uart_init_status != HAL_OK) g_blackbox_tx_stats.error_count++;
    Blackbox_SetState(uart_init_status == HAL_OK ? BLACKBOX_IDLE : BLACKBOX_DISABLED);
}

void Blackbox_Service(void)
{
    HAL_StatusTypeDef status;
    const uint32_t now = HAL_GetTick();
    uint64_t us=Blackbox_NowUs();
    if((uint32_t)(now-s_rate_tick)>=1000U) {
        taskENTER_CRITICAL();
        g_blackbox_tx_stats.record_hz=(uint32_t)((uint64_t)s_rate_count*1000U/(uint32_t)(now-s_rate_tick));
        s_rate_count=0;s_rate_tick=now;
        taskEXIT_CRITICAL();
    }
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
    if(g_blackbox_tx_stats.state==BLACKBOX_IDLE)Blackbox_SendSchema(us);
}

void Blackbox_SendSnapshot(void)
{
    uint64_t now;
    uint16_t size;
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
    if(!s_schema_ready)return;
    now=Blackbox_NowUs();
    if(now<s_data_due){g_blackbox_tx_stats.bandwidth_skip_count++;return;}
    Blackbox_Sample(s_dma_frame+SBB_HEADER,now);
    size=sbb_packet(s_dma_frame,SBB_DATA,s_payload_size,s_schema_id,s_sequence++,now);
    /* Charge attempts too: no catch-up burst after failure or blocked DMA. */
    s_data_due=now+((uint64_t)size*1000000U+149999U)/150000U;
    Blackbox_Submit(size,SBB_DATA);
}

static void Blackbox_SendSchema(uint64_t now)
{
    uint16_t n,size;
    if(s_schema_offset==s_schema_size) {
        if((uint32_t)(HAL_GetTick()-s_schema_tick)<2000U)return;
        s_schema_offset=0;
        s_schema_tick=HAL_GetTick();
    }
    if(now<s_schema_due)return;
    n=(uint16_t)(s_schema_size-s_schema_offset);if(n>96)n=96;
    sbb_p16(s_dma_frame+24,s_schema_size);sbb_p16(s_dma_frame+26,s_schema_offset);
    memcpy(s_dma_frame+28,s_schema+s_schema_offset,n);
    size=sbb_packet(s_dma_frame,SBB_SCHEMA,(uint16_t)(n+4),s_schema_id,s_sequence++,now);
    s_pending_fragment=n;
    s_schema_due=now+(s_schema_ready?20000U:((uint64_t)size*1000000U+9999U)/10000U);
    Blackbox_Submit(size,SBB_SCHEMA);
}

static void Blackbox_Submit(uint16_t size,uint8_t type)
{
    HAL_StatusTypeDef status;
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
    s_pending_type=type;
    Blackbox_SetState(BLACKBOX_SENDING);
    status = HAL_UART_Transmit_DMA(&huart7, s_dma_frame, size);
    g_blackbox_tx_stats.last_status = status;
    if (status == HAL_OK) {
        g_blackbox_tx_stats.submitted_count++;
        if(type==SBB_SCHEMA)g_blackbox_tx_stats.schema_submitted_count++;
    }
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
        if(s_pending_type==SBB_SCHEMA) {
            s_schema_offset=(uint16_t)(s_schema_offset+s_pending_fragment);
            g_blackbox_tx_stats.schema_completed_count++;
            if(s_schema_offset==s_schema_size)s_schema_ready=1;
        } else {g_blackbox_tx_stats.data_completed_count++;s_rate_count++;}
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
