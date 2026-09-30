/* Compile real ESC safety/output and VOFA packing against simulated timers. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../App/esc_pwm.c"
#include "../../App/vofa.c"

static void near_value(float actual,float expected)
{ assert(fabsf(actual-expected)<0.0001f); }

static void check_packet(const float expected[4])
{
    EscPwmState_t esc;
    float rates[6]={7,8,9,10,11,12}, decoded[20];
    unsigned i;
    EscPwm_GetState(&esc);
    assert(test_critical_depth==0);
    assert(VOFA_SendFlightTelemetry(0,1,2,3,4,5,6,rates,13,-14,15,esc.command)==HAL_OK);
    assert(test_uart_size==84 && test_uart_timeout==10);
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
    test_uart_duration_ms=4;
    assert(VOFA_SendAttitude(0,0,0)==HAL_OK);
    assert(g_vofa_tx_stats.success_count==1 && g_vofa_tx_stats.failure_count==0);
    assert(g_vofa_tx_stats.last_status==HAL_OK && g_vofa_tx_stats.max_duration_ms==4);
    calls=test_uart_calls;
    for(i=0;i<3;i++) {
        const HAL_StatusTypeDef results[3]={HAL_TIMEOUT,HAL_BUSY,HAL_ERROR};
        test_uart_status=results[i]; test_uart_duration_ms=i==0 ? 11 : 0;
        assert(VOFA_SendAttitude(0,0,0)==results[i]);
        assert(g_vofa_tx_stats.last_status==results[i]);
        assert(test_uart_calls==calls+i+1); /* One call, no retry. */
    }
    assert(g_vofa_tx_stats.success_count==1 && g_vofa_tx_stats.failure_count==3);
    assert(g_vofa_tx_stats.max_duration_ms==11);
    test_uart_status=HAL_OK; test_uart_duration_ms=5; test_tick=0xfffffffeU;
    assert(VOFA_SendAttitude(0,0,0)==HAL_OK);
    assert(g_vofa_tx_stats.success_count==2 && g_vofa_tx_stats.max_duration_ms==11);
    calls=test_uart_calls;
    assert(VOFA_SendFlightTelemetry(0,0,0,0,0,0,0,minimum,0,0,0,NULL)==HAL_ERROR);
    assert(test_uart_calls==calls); /* Invalid arguments never reach UART. */
    puts("PASS: UART success/timeout/busy/error accounting, no retry, maximum duration and tick rollover");
    return 0;
}
