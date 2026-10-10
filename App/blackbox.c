#include "blackbox.h"
#include "vofa.h"
#include "usart.h"
#include "tim.h"
#include <string.h>
#include "blackbox_fields.h"

#define BLACKBOX_TIMEOUT_MS 5U
/* DMA1 cannot access DTCM. Own sixteen complete cache lines in AXI SRAM. */
__attribute__((section(".blackbox_dma"), aligned(32)))
static uint8_t s_dma_frame[512];
static uint32_t s_state_tick;
static uint32_t s_cycle_last, s_sequence, s_schema_tick, s_rate_tick, s_rate_count;
static uint64_t s_cycles, s_schema_due;
static uint16_t s_schema_offset, s_pending_fragment;
static uint8_t s_schema_ready, s_pending_type, s_clock_started;
static TaskHandle_t s_recording_task;
static uint32_t s_period_base, s_period_remainder, s_period_fraction;
static uint32_t s_consumed_trigger, s_bandwidth_fraction;
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
    s_schema_due=0;s_schema_tick=s_rate_tick=HAL_GetTick();s_rate_count=0;
    s_pending_type=0;s_pending_fragment=0;
    s_recording_task=NULL;s_consumed_trigger=s_period_fraction=s_bandwidth_fraction=0;
    g_blackbox_tx_stats.target_hz=BLACKBOX_TARGET_HZ;
    if(!Blackbox_CreateSchema()) {
        g_blackbox_tx_stats.config_error=1;Blackbox_SetState(BLACKBOX_DISABLED);return;
    }
    g_blackbox_tx_stats.schema_id=s_schema_id;g_blackbox_tx_stats.payload_size=s_payload_size;
    g_blackbox_tx_stats.effective_hz=BLACKBOX_DATA_BUDGET/(s_payload_size+SBB_HEADER+4U);
    if(g_blackbox_tx_stats.effective_hz>BLACKBOX_TARGET_HZ)
        g_blackbox_tx_stats.effective_hz=BLACKBOX_TARGET_HZ;
    s_period_base=1000000U/g_blackbox_tx_stats.effective_hz;
    s_period_remainder=1000000U%g_blackbox_tx_stats.effective_hz;
    g_blackbox_tx_stats.initialized = (uart_init_status == HAL_OK);
    if (uart_init_status != HAL_OK) g_blackbox_tx_stats.error_count++;
    Blackbox_SetState(uart_init_status == HAL_OK ? BLACKBOX_IDLE : BLACKBOX_DISABLED);
}

static uint32_t Blackbox_NextPeriod(void)
{
    uint32_t period=s_period_base;
    s_period_fraction+=s_period_remainder;
    if(s_period_fraction>=g_blackbox_tx_stats.effective_hz) {
        s_period_fraction-=g_blackbox_tx_stats.effective_hz;period++;
    }
    return period;
}

HAL_StatusTypeDef Blackbox_StartRecording(TaskHandle_t task)
{
    HAL_StatusTypeDef status;
    if(!task || g_blackbox_tx_stats.disabled)return HAL_ERROR;
    s_recording_task=task;
    /* Shared DWT clock: enabling is safe; resetting it would break other users. */
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    status=MX_TIM4_Init();
    if(status==HAL_OK) {
        taskENTER_CRITICAL();
        s_period_fraction=0;
        __HAL_TIM_SET_AUTORELOAD(&htim4,Blackbox_NextPeriod()-1U);
        status=HAL_TIM_GenerateEvent(&htim4,TIM_EVENTSOURCE_UPDATE);
        if(status==HAL_OK) {
            __HAL_TIM_SET_COUNTER(&htim4,0U);
            __HAL_TIM_CLEAR_FLAG(&htim4,TIM_FLAG_UPDATE);
            /* ARR is preloaded: the ISR programs one period ahead, so normal
             * ISR latency does not shift a period already counting. */
            __HAL_TIM_SET_AUTORELOAD(&htim4,Blackbox_NextPeriod()-1U);
            g_blackbox_tx_stats.timer_running=1;
            status=HAL_TIM_Base_Start_IT(&htim4);
        }
        taskEXIT_CRITICAL();
    }
    g_blackbox_tx_stats.last_timer_status=status;
    if(status!=HAL_OK) {
        g_blackbox_tx_stats.timer_running=0;s_recording_task=NULL;
        (void)HAL_TIM_Base_Stop_IT(&htim4);
        g_blackbox_tx_stats.timer_error_count++;
        Blackbox_SetState(BLACKBOX_DISABLED);
    }
    return status;
}

void Blackbox_OnTaskCreationFailure(void)
{
    if(g_blackbox_tx_stats.timer_running)(void)HAL_TIM_Base_Stop_IT(&htim4);
    g_blackbox_tx_stats.timer_running=0;s_recording_task=NULL;
    g_blackbox_tx_stats.task_error_count++;
    Blackbox_SetState(BLACKBOX_DISABLED);
}

void Blackbox_NotifyFromISR(void)
{
    BaseType_t wake=pdFALSE;
    if(g_blackbox_tx_stats.timer_running && s_recording_task!=NULL) {
        __HAL_TIM_SET_AUTORELOAD(&htim4,Blackbox_NextPeriod()-1U);
        g_blackbox_tx_stats.timer_irq_count++;
        vTaskNotifyGiveFromISR(s_recording_task,&wake);
        portYIELD_FROM_ISR(wake);
    }
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

static void Blackbox_SendSnapshot(void)
{
    uint64_t now;
    uint16_t size;
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
    Blackbox_Sample(s_dma_frame+SBB_HEADER);
    size=sbb_packet(s_dma_frame,SBB_DATA,s_payload_size,s_schema_id,s_sequence++,now);
    /* TIM4 already enforces the absolute rational cadence and bandwidth cap.
     * Do not restart a per-send delay: task jitter must not halve the rate. */
    Blackbox_Submit(size,SBB_DATA);
}

void Blackbox_ProcessPending(void)
{
    uint32_t trigger,delta,after;
    uint64_t limited;
    Blackbox_Service();
    if(g_blackbox_tx_stats.disabled && g_blackbox_tx_stats.timer_running) {
        (void)HAL_TIM_Base_Stop_IT(&htim4);
        g_blackbox_tx_stats.timer_running=0;s_recording_task=NULL;
    }
    if(!g_blackbox_tx_stats.timer_running)return;
    trigger=g_blackbox_tx_stats.timer_irq_count;
    delta=(uint32_t)(trigger-s_consumed_trigger);
    if(!delta)return; /* Duplicate/timeout wake never invents a DATA record. */
    g_blackbox_tx_stats.missed_trigger_count+=delta-1U;
    limited=(uint64_t)delta*(BLACKBOX_TARGET_HZ-g_blackbox_tx_stats.effective_hz)+s_bandwidth_fraction;
    g_blackbox_tx_stats.bandwidth_skip_count+=(uint32_t)(limited/g_blackbox_tx_stats.effective_hz);
    s_bandwidth_fraction=(uint32_t)(limited%g_blackbox_tx_stats.effective_hz);
    Blackbox_SendSnapshot();
    /* Coalesce notifications that arrived during capture/submission too. */
    after=g_blackbox_tx_stats.timer_irq_count;
    g_blackbox_tx_stats.missed_trigger_count+=(uint32_t)(after-trigger);
    s_consumed_trigger=after;
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
