/* Compile real ESC safety/output and VOFA packing against simulated timers. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../App/esc_pwm.c"
#include "../../App/vofa.c"

static void near_value(float actual,float expected)
{ assert(fabsf(actual-expected)<0.0001f); }
static void late_completion_during_abort(UART_HandleTypeDef *uart)
{
    uint32_t completed=g_vofa_tx_stats.success_count;
    HAL_UART_TxCpltCallback(uart);
    assert(g_vofa_tx_stats.success_count==completed && g_vofa_tx_stats.busy);
}

static void check_packet(const float expected[4])
{
    EscPwmState_t esc;
    float rates[6]={7,8,9,10,11,12}, decoded[20];
    unsigned i;
    EscPwm_GetState(&esc);
    assert(test_critical_depth==0);
    assert(VOFA_SendFlightTelemetry(0,1,2,3,4,5,6,rates,13,-14,15,esc.command)==HAL_OK);
    assert(test_uart_size==84 && !test_uart_poll_calls);
    memcpy(decoded,test_uart_bytes,sizeof(decoded));
    for(i=0;i<14;i++) near_value(decoded[i],(float)i);
    near_value(decoded[14],-14); near_value(decoded[15],15);
    for(i=0;i<4;i++) near_value(decoded[16+i],expected[i]);
    assert(memcmp(test_uart_bytes+80,"\x00\x00\x80\x7f",4)==0);
}

int main(void)
{
    RcSbusData_t rc={0};
    const float request[4]={-0.2f,0.25f,0.75f,1.2f};
    const float clamped[4]={0,0.25f,0.75f,1};
    const float minimum[4]={0};
    unsigned i;
    uint32_t calls;
    assert(EscPwm_Init()==HAL_OK);
    check_packet(minimum);
    test_tick=2000; rc.valid=1; rc.last_valid_tick=test_tick;
    rc.channel[6]=1000; EscPwm_UpdateFromRc(&rc);
    rc.channel[6]=1696; EscPwm_UpdateFromRc(&rc);
    assert(EscPwm_IsArmed());
    EscPwm_SetMixedNormalized(request); check_packet(clamped);
    assert(htim1.compare[TIM_CHANNEL_3]==1000 && htim1.compare[TIM_CHANNEL_1]==1250);
    assert(htim2.compare[TIM_CHANNEL_3]==1750 && htim2.compare[TIM_CHANNEL_1]==2000);
    EscPwm_Disarm(); EscPwm_SetMixedNormalized(request); check_packet(minimum);
    puts("PASS: real ESC snapshot, four motor order/clamping/PWM, lock zero and unchanged CH0..13 in 84-byte frame");

    memset((void *)&g_vofa_tx_stats,0,sizeof(g_vofa_tx_stats));
    test_uart_duration_ms=0;test_uart_dma_auto_complete=0;
    assert(VOFA_SendAttitude(0,0,0)==HAL_OK);
    assert(g_vofa_tx_stats.success_count==0 && g_vofa_tx_stats.submitted_count==1 && g_vofa_tx_stats.busy);
    assert(test_uart_poll_calls==0); /* No blocking fallback. */
    test_tick+=4;HAL_UART_TxCpltCallback(&huart1);
    assert(g_vofa_tx_stats.success_count==1 && g_vofa_tx_stats.failure_count==0);
    assert(g_vofa_tx_stats.last_status==HAL_OK && g_vofa_tx_stats.max_duration_ms==4);
    calls=test_uart_calls;
    for(i=0;i<3;i++) {
        const HAL_StatusTypeDef results[3]={HAL_TIMEOUT,HAL_BUSY,HAL_ERROR};
        test_dma_status=results[i];
        assert(VOFA_SendAttitude(0,0,0)==results[i]);
        assert(g_vofa_tx_stats.last_status==results[i]);
        assert(test_uart_calls==calls+i+1); /* One call, no retry. */
    }
    assert(g_vofa_tx_stats.success_count==1 && g_vofa_tx_stats.failure_count==3);
    assert(g_vofa_tx_stats.max_duration_ms==4);
    test_dma_status=HAL_OK;test_tick=0xfffffffeU;
    assert(VOFA_SendAttitude(0,0,0)==HAL_OK);
    test_tick+=5;HAL_UART_TxCpltCallback(&huart1);
    assert(g_vofa_tx_stats.success_count==2 && g_vofa_tx_stats.max_duration_ms==5);
    calls=test_uart_calls;
    assert(VOFA_SendFlightTelemetry(0,0,0,0,0,0,0,minimum,0,0,0,NULL)==HAL_ERROR);
    assert(test_uart_calls==calls); /* Invalid arguments never reach UART. */
    /* In-flight buffer belongs to DMA even after the caller changes its stack. */
    { uint8_t frame[96],saved[96];uint32_t completed=g_vofa_tx_stats.success_count;
      memset(frame,0x35,sizeof(frame));memcpy(saved,frame,sizeof(frame));
      test_scb.CCR=SCB_CCR_DC_Msk;calls=test_clean_calls;
      assert(VOFA_SendFrame(frame)==HAL_OK);
      assert(test_clean_calls==calls+1 && test_clean_size==96);
      assert(test_dma_source==s_vofa_dma_frame && test_dma_source!=frame);
      memset(frame,0xa7,sizeof(frame));
      assert(!memcmp(test_dma_source,saved,sizeof(saved)));
      calls=test_dma_calls;
      assert(VOFA_SendFrame(frame)==HAL_BUSY);
      assert(test_dma_calls==calls && g_vofa_tx_stats.busy_drop_count==1);
      assert(!memcmp(test_dma_source,saved,sizeof(saved)));
      HAL_UART_TxCpltCallback(&huart5);VOFA_OnUartError(&huart5);
      assert(g_vofa_tx_stats.success_count==completed && g_vofa_tx_stats.busy);
      HAL_UART_TxCpltCallback(&huart1);HAL_UART_TxCpltCallback(&huart1);
      assert(g_vofa_tx_stats.success_count==completed+1 && !g_vofa_tx_stats.busy);
      /* DMA failure latches in ISR; abort/reuse only on next task send. */
      assert(VOFA_SendFrame(frame)==HAL_OK);calls=test_abort_calls;
      huart1.ErrorCode=0x10;VOFA_OnUartError(&huart1);
      assert(test_abort_calls==calls && g_vofa_tx_stats.dma_error_count==1);
      assert(g_vofa_tx_stats.last_uart_error==0x10 && g_vofa_tx_stats.busy);
      assert(VOFA_SendFrame(frame)==HAL_OK);
      assert(test_abort_calls==calls+1);HAL_UART_TxCpltCallback(&huart1);
      /* No completion => 10 ms deadline. Abort failure must retain buffer. */
      assert(VOFA_SendFrame(frame)==HAL_OK);memcpy(saved,frame,sizeof(saved));
      test_tick+=10;test_abort_status=HAL_ERROR;memset(frame,0x11,sizeof(frame));
      calls=test_dma_calls;
      assert(VOFA_SendFrame(frame)==HAL_ERROR);
      assert(g_vofa_tx_stats.timeout_count==1 && g_vofa_tx_stats.abort_failure_count==1);
      assert(g_vofa_tx_stats.busy && test_dma_calls==calls && !memcmp(test_dma_source,saved,96));
      /* A completion after a failed abort cannot count as a successful frame. */
      completed=g_vofa_tx_stats.success_count;HAL_UART_TxCpltCallback(&huart1);
      assert(g_vofa_tx_stats.success_count==completed);
      test_abort_status=HAL_OK;
      test_uart_abort_hook=late_completion_during_abort;
      assert(VOFA_SendFrame(frame)==HAL_OK);HAL_UART_TxCpltCallback(&huart1);
      test_uart_abort_hook=NULL;
      assert(test_clear_pending_count>=2 && !g_vofa_tx_stats.busy);
      /* Completion serviced after the nominal deadline wins over task timeout. */
      assert(VOFA_SendFrame(frame)==HAL_OK);test_tick+=20;
      HAL_UART_TxCpltCallback(&huart1);
      calls=g_vofa_tx_stats.timeout_count;
      assert(VOFA_SendFrame(frame)==HAL_OK && g_vofa_tx_stats.timeout_count==calls);
      HAL_UART_TxCpltCallback(&huart1);
      test_scb.CCR=0;calls=test_clean_calls;
      assert(VOFA_SendFrame(frame)==HAL_OK && test_clean_calls==calls);HAL_UART_TxCpltCallback(&huart1);
      huart1.hdmatx=NULL;calls=test_dma_calls;
      assert(VOFA_SendFrame(frame)==HAL_ERROR && test_dma_calls==calls);huart1.hdmatx=(void *)1;
      assert(VOFA_SendFrame(NULL)==HAL_ERROR && test_dma_calls==calls);
    }
    puts("PASS DMA: immutable persistent buffer, cache, async completion, busy/drop, errors, timeout/abort recovery, callback isolation and tick wrap");
    return 0;
}
