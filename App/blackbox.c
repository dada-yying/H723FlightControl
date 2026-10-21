#include "blackbox.h"
#include "vofa.h"
#include "storage/blackbox_storage.h"
#include "tim.h"
#include <string.h>
#include "blackbox_fields.h"

/* Single sampling-task owner. Never passed directly to DMA. */
static uint8_t s_dma_frame[512];
static uint32_t s_cycle_last, s_sequence, s_rate_tick, s_rate_count;
static uint64_t s_cycles;
static uint8_t s_clock_started;
static TaskHandle_t s_recording_task;
static uint32_t s_period_base, s_period_remainder, s_period_fraction;
static uint32_t s_consumed_trigger;
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

static void Blackbox_SetState(BlackboxTxState_t state)
{
    g_blackbox_tx_stats.state = state;
    g_blackbox_tx_stats.busy = (state == BLACKBOX_SENDING ||
        state == BLACKBOX_RECOVER || state == BLACKBOX_ABORTING);
    g_blackbox_tx_stats.disabled = (state == BLACKBOX_DISABLED);
}

void Blackbox_Init(HAL_StatusTypeDef startup_status)
{
    /* Startup only, before either producer or storage task exists. */
    (void)memset((void *)&g_blackbox_tx_stats, 0, sizeof(g_blackbox_tx_stats));
    BlackboxStorage_Init();
    g_blackbox_tx_stats.last_status = startup_status;
    s_cycles=0;s_cycle_last=DWT->CYCCNT;s_clock_started=0;s_sequence=0;
    s_rate_tick=HAL_GetTick();s_rate_count=0;
    s_recording_task=NULL;s_consumed_trigger=s_period_fraction=0;
    g_blackbox_tx_stats.target_hz=BLACKBOX_TARGET_HZ;
    if(!Blackbox_CreateSchema()) {
        g_blackbox_tx_stats.config_error=1;Blackbox_SetState(BLACKBOX_DISABLED);
        BlackboxStorage_TaskFailed();return;
    }
    g_blackbox_tx_stats.schema_id=s_schema_id;g_blackbox_tx_stats.payload_size=s_payload_size;
    g_blackbox_tx_stats.effective_hz=BLACKBOX_TARGET_HZ;
    s_period_base=1000000U/g_blackbox_tx_stats.effective_hz;
    s_period_remainder=1000000U%g_blackbox_tx_stats.effective_hz;
    g_blackbox_tx_stats.initialized = (startup_status == HAL_OK);
    if (startup_status != HAL_OK) {g_blackbox_tx_stats.error_count++;BlackboxStorage_TaskFailed();}
    Blackbox_SetState(startup_status == HAL_OK ? BLACKBOX_IDLE : BLACKBOX_DISABLED);
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
    const uint32_t now = HAL_GetTick();
    (void)Blackbox_NowUs();
    if((uint32_t)(now-s_rate_tick)>=1000U) {
        g_blackbox_tx_stats.record_hz=(uint32_t)((uint64_t)s_rate_count*1000U/(uint32_t)(now-s_rate_tick));
        s_rate_count=0;s_rate_tick=now;
    }
    /* The storage task writes a complete schema prefix in every new file.
     * Do not interleave repeat definitions into the 1kHz DATA sequence. */
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
    /* Single producer owns the temporary packet until it is copied to queue. */
    taskEXIT_CRITICAL();

    now=Blackbox_NowUs();
    Blackbox_Sample(s_dma_frame+SBB_HEADER);
    size=sbb_packet(s_dma_frame,SBB_DATA,s_payload_size,s_schema_id,s_sequence++,now);
    /* TIM4 enforces the absolute 1 kHz cadence independently of storage. */
    Blackbox_Submit(size,SBB_DATA);
}

void Blackbox_ProcessPending(void)
{
    uint32_t trigger,delta,after;
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
    Blackbox_SendSnapshot();
    /* Coalesce notifications that arrived during capture/submission too. */
    after=g_blackbox_tx_stats.timer_irq_count;
    g_blackbox_tx_stats.missed_trigger_count+=(uint32_t)(after-trigger);
    s_consumed_trigger=after;
}

static void Blackbox_Submit(uint16_t size,uint8_t type)
{
    if(!BlackboxStorage_Enqueue(s_dma_frame,size,((uint64_t)sbb_u32(s_dma_frame+16)|((uint64_t)sbb_u32(s_dma_frame+20)<<32)))) {
        g_blackbox_tx_stats.busy_drop_count++;
        return;
    }
    /* Compatibility counters now mean accepted into the SD queue, not durable writes. */
    g_blackbox_tx_stats.submitted_count++;
    g_blackbox_tx_stats.completed_count++;
    if(type==SBB_DATA){g_blackbox_tx_stats.data_completed_count++;s_rate_count++;}
}
uint16_t Blackbox_SchemaSize(void){return s_schema_size;}
const uint8_t *Blackbox_SchemaData(void){return s_schema;}
