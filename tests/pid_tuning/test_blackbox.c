#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "../../App/rc_sbus.c"
#include "../../App/esc_pwm.c"
#include "../../App/flight_control.c"
#include "../../App/vofa.c"
#include "../../App/blackbox.c"

void AtomImu_GetData(AtomImuData_t *data) { memset(data,0,sizeof(*data)); }
static void reset(void)
{
    assert(test_critical_depth==0);
    test_dma_calls=test_abort_calls=test_clean_calls=0;
    test_dma_status=test_abort_status=HAL_OK;
    test_abort_immediate=0; test_clean_hook=NULL;
    test_tick=100;
    test_scb.CCR=SCB_CCR_DC_Msk;
    RcSbus_Init(); FlightControl_Init();
    assert(EscPwm_Init()==HAL_OK);
    Blackbox_Init(HAL_OK);
}
static void inject_error(void) { huart7.ErrorCode=0x10; HAL_UART_ErrorCallback(&huart7); }
static void near_value(float a,float b) { assert(fabsf(a-b)<0.0001f); }

int main(void)
{
    uint8_t expected[84], held[96];
    float decoded[20], rates[6];
    FlightControlState_t flight;
    RcSbusData_t rc;
    EscPwmState_t esc;
    const float degrees=180.0f/3.14159265358979323846f;
    unsigned i;
    reset();
    g_flight_control_state.roll_rad=0.1f;
    g_flight_control_state.pitch_rad=-0.2f;
    g_flight_control_state.yaw_rad=0.3f;
    g_flight_control_state.roll_rate_target_rad_s=0.4f;
    g_flight_control_state.roll_rate_rad_s=0.5f;
    g_flight_control_state.pitch_rate_target_rad_s=-0.6f;
    g_flight_control_state.pitch_rate_rad_s=-0.7f;
    g_flight_control_state.yaw_rate_target_rad_s=0.8f;
    g_flight_control_state.yaw_rate_rad_s=0.9f;
    g_flight_control_state.roll_rate_raw_rad_s=1.0f;
    g_flight_control_state.pitch_rate_raw_rad_s=-1.1f;
    g_flight_control_state.yaw_rate_raw_rad_s=1.2f;
    g_rc_sbus_data.throttle=0.25f;
    g_rc_sbus_data.roll_target_rad=0.13f;
    g_rc_sbus_data.pitch_target_rad=-0.14f;
    g_rc_sbus_data.yaw_rate_target_rad_s=0.15f;
    /* Exercise real ESC low-throttle middle->high arming and output path. */
    RcSbus_GetData(&rc); rc.valid=1; rc.throttle=0;
    test_tick+=2000; rc.last_valid_tick=test_tick; rc.channel[6]=1000;
    EscPwm_UpdateFromRc(&rc); rc.channel[6]=1696; EscPwm_UpdateFromRc(&rc);
    assert(EscPwm_IsArmed());
    { const float motors[4]={0.1f,0.2f,0.3f,0.4f}; EscPwm_SetMixedNormalized(motors); }
    FlightControl_GetState(&flight); RcSbus_GetData(&rc); EscPwm_GetState(&esc);
    VOFA_BuildFlightFrame(expected,&flight,&rc,&esc);
    rates[0]=0.4f*degrees; rates[1]=0.5f*degrees;
    rates[2]=-0.6f*degrees; rates[3]=-0.7f*degrees;
    rates[4]=0.8f*degrees; rates[5]=0.9f*degrees;
    assert(VOFA_SendFlightTelemetry(0.1f*degrees,-0.2f*degrees,0.3f*degrees,
        0.25f,0.13f*degrees,-0.14f*degrees,0.15f*degrees,rates,
        1.0f*degrees,-1.1f*degrees,1.2f*degrees,esc.command)==HAL_OK);
    assert(memcmp(test_uart_bytes,expected,84)==0); /* Previous API has identical layout. */
    assert(VOFA_SendFrame(expected)==HAL_OK);
    Blackbox_SendSnapshot();
    assert(test_dma_calls==1 && test_dma_size==84 && test_clean_calls==1 && test_clean_size==96);
    assert(memcmp(test_dma_source,expected,84)==0 && memcmp(test_dma_source,test_uart_bytes,84)==0);
    assert(g_blackbox_tx_stats.submitted_count==1 && g_blackbox_tx_stats.busy);
    memcpy(held,test_dma_source,96);
    g_flight_control_state.roll_rad=99; Blackbox_SendSnapshot();
    assert(test_dma_calls==1 && g_blackbox_tx_stats.busy_drop_count==1);
    assert(memcmp(held,test_dma_source,96)==0);
    HAL_UART_TxCpltCallback(&huart1); assert(g_blackbox_tx_stats.busy);
    HAL_UART_TxCpltCallback(&huart7); assert(!g_blackbox_tx_stats.busy);
    assert(g_blackbox_tx_stats.completed_count==1);
    EscPwm_Disarm(); Blackbox_SendSnapshot();
    memcpy(decoded,test_dma_source,84-4);
    for(i=16;i<20;i++) near_value(decoded[i],0);
    assert(test_dma_calls==2); HAL_UART_TxCpltCallback(&huart7);
    test_scb.CCR=0; Blackbox_SendSnapshot(); assert(test_clean_calls==2);
    puts("PASS: identical 20-channel UART1/UART7 frames, locked zero, DMA ownership, cache cleaning, TC release");

    reset(); Blackbox_SendSnapshot(); test_tick+=5; Blackbox_Service();
    assert(test_abort_calls==0); test_tick++; Blackbox_Service();
    assert(test_abort_calls==1 && g_blackbox_tx_stats.state==BLACKBOX_ABORTING);
    assert(g_blackbox_tx_stats.timeout_count==1);
    HAL_UART_TxCpltCallback(&huart7); assert(g_blackbox_tx_stats.state==BLACKBOX_ABORTING);
    Blackbox_SendSnapshot(); assert(test_dma_calls==1 && test_abort_calls==1);
    HAL_UART_AbortTransmitCpltCallback(&huart7); Blackbox_SendSnapshot();
    assert(test_dma_calls==2);
    inject_error(); assert(!s_restart_pending);
    assert(g_blackbox_tx_stats.state==BLACKBOX_RECOVER && g_blackbox_tx_stats.last_uart_error==0x10);
    test_abort_immediate=1; Blackbox_Service();
    assert(g_blackbox_tx_stats.state==BLACKBOX_IDLE);
    HAL_UART_ErrorCallback(&huart5);
    assert(s_restart_pending && g_blackbox_tx_stats.state==BLACKBOX_IDLE);
    puts("PASS: >5ms timeout, async/synchronous abort recovery, late TC ignored, UART5 error routing preserved");

    reset(); Blackbox_SendSnapshot(); inject_error(); Blackbox_Service();
    test_tick+=6; Blackbox_Service(); assert(g_blackbox_tx_stats.disabled);
    HAL_UART_AbortTransmitCpltCallback(&huart7); HAL_UART_TxCpltCallback(&huart7);
    Blackbox_SendSnapshot(); assert(test_dma_calls==1 && g_blackbox_tx_stats.disabled);
    reset(); Blackbox_SendSnapshot(); inject_error(); test_abort_status=HAL_ERROR; Blackbox_Service();
    assert(g_blackbox_tx_stats.disabled);
    reset(); Blackbox_Init(HAL_ERROR); Blackbox_SendSnapshot();
    assert(test_dma_calls==0 && g_blackbox_tx_stats.disabled && !g_blackbox_tx_stats.initialized);
    for(i=0;i<3;i++) {
        const HAL_StatusTypeDef errors[3]={HAL_BUSY,HAL_ERROR,HAL_TIMEOUT};
        reset(); test_dma_status=errors[i]; Blackbox_SendSnapshot();
        assert(test_dma_calls==1 && g_blackbox_tx_stats.submitted_count==0);
        assert(g_blackbox_tx_stats.last_status==errors[i]);
        assert(g_blackbox_tx_stats.state==BLACKBOX_RECOVER);
        test_abort_immediate=1; Blackbox_Service(); test_dma_status=HAL_OK; Blackbox_SendSnapshot();
        assert(test_dma_calls==2 && g_blackbox_tx_stats.submitted_count==1);
    }
    reset(); test_tick=0xfffffffeU; Blackbox_SendSnapshot(); test_tick=4; Blackbox_Service();
    assert(test_abort_calls==1);
    reset(); test_clean_hook=inject_error; Blackbox_SendSnapshot();
    assert(test_dma_calls==0 && g_blackbox_tx_stats.state==BLACKBOX_RECOVER);
    assert(test_critical_depth==0);
    puts("PASS: initialization/abort failure isolation, stuck abort latch, submission errors, tick wrap and snapshot error race");
    return 0;
}
