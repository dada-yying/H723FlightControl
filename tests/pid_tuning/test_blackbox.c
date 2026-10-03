#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "../../App/rc_sbus.c"
#include "../../App/esc_pwm.c"
#include "../../App/flight_control.c"
#include "../../App/vofa.c"
#include "../../App/blackbox.c"
#include "../../App/optical_flow.c"
#include "../../App/bmi088.c"
void AtomImu_GetData(AtomImuData_t *p){memset(p,0,sizeof(*p));}
static void advance(unsigned us){test_dwt.CYCCNT+=us*480U;test_tick+=us/1000U;}
static void reset(void){
    test_tick=100;test_dwt.CYCCNT=0;test_dma_calls=test_abort_calls=test_clean_calls=0;
    test_dma_status=test_abort_status=HAL_OK;test_abort_immediate=0;test_clean_hook=NULL;test_scb.CCR=SCB_CCR_DC_Msk;
    RcSbus_Init();FlightControl_Init();assert(EscPwm_Init()==HAL_OK);OpticalFlow_Init(HAL_OK);Bmi088_Init(HAL_ERROR);Blackbox_Init(HAL_OK);
}
static void seed_bmi(void){
    unsigned axis;
    Bmi088Data_t sample;
    for(axis=0;axis<3;axis++) {
        g_bmi088_data.gyro_rad_s[axis]=(axis==0?-32768:axis==1?1234:32767)*BMI088_GYRO_SCALE;
        g_bmi088_data.accel_m_s2[axis]=(axis==0?1000:axis==1?-2000:8192)*BMI088_ACCEL_SCALE;
    }
    g_bmi088_data.temperature_raw=80;g_bmi088_data.temperature_c=33;
    g_bmi088_data.sample_count=0xfffffffeU;g_bmi088_data.sample_tick_ms=test_tick;
    g_bmi088_data.has_sample=g_bmi088_data.valid=g_bmi088_data.initialized=1;
    g_bmi088_data.error=BMI088_ERROR_NONE;
    sample=g_bmi088_data;Bmi088_ApplyCalibration(&sample);g_bmi088_data=sample;
}
static void definition(void){unsigned guard=0;while(!s_schema_ready){Blackbox_Service();if(g_blackbox_tx_stats.busy)HAL_UART_TxCpltCallback(&huart7);advance(20000);assert(++guard<100);} }
static void inject_error(void){huart7.ErrorCode=0x10;HAL_UART_ErrorCallback(&huart7);}
int main(int argc,char **argv){
    unsigned i,calls;uint8_t held[512],expected[96];FILE *out=NULL;
    FlightControlState_t flight;RcSbusData_t rc;EscPwmState_t esc;OpticalFlowData_t optical;
    reset();assert(s_schema_size<=8192 && s_payload_size<=484);
    definition();
    g_flight_control_state.roll_rad=.1f;g_flight_control_state.pitch_rate_raw_rad_s=-1.1f;
    FlightControl_GetState(&flight);RcSbus_GetData(&rc);EscPwm_GetState(&esc);OpticalFlow_GetData(&optical);
    VOFA_BuildFlightFrame(expected,&flight,&rc,&esc,&optical);VOFA_SendFrame(expected);
    Blackbox_SendSnapshot();assert(sbb_valid(test_dma_source,test_dma_size) && test_dma_source[5]==SBB_DATA);
    assert(test_dma_size==s_payload_size+28 && test_clean_size==512);
    assert(!memcmp(test_dma_source+16,test_dma_source+24+112,8));
    Blackbox_Sample(held,0x123456789abcdef0ULL);
    assert(sbb_u32(held+112)==0x9abcdef0U && sbb_u32(held+116)==0x12345678U);
    assert(sbb_u32(held+158)==UINT32_MAX && !held[162] && !held[163]);
    assert(!memcmp(expected,test_dma_source+24,80) && test_uart_size==96);
    for(i=0;i<4;i++){float motor;memcpy(&motor,test_dma_source+24+64+4*i,4);assert(motor==0);}
    memcpy(held,test_dma_source,512);calls=test_dma_calls;
    Blackbox_SendSnapshot();assert(test_dma_calls==calls && !memcmp(held,test_dma_source,512));
    HAL_UART_TxCpltCallback(&huart1);assert(g_blackbox_tx_stats.busy);
    HAL_UART_TxCpltCallback(&huart7);Blackbox_SendSnapshot();assert(test_dma_calls==calls && g_blackbox_tx_stats.bandwidth_skip_count);
    advance(2000);Blackbox_SendSnapshot();assert(test_dma_calls==calls+1);
    advance(6000);Blackbox_Service();assert(test_abort_calls==1 && g_blackbox_tx_stats.state==BLACKBOX_ABORTING);
    HAL_UART_TxCpltCallback(&huart7);assert(g_blackbox_tx_stats.state==BLACKBOX_ABORTING);
    HAL_UART_AbortTransmitCpltCallback(&huart7);inject_error();assert(!s_restart_pending);
    test_abort_immediate=1;Blackbox_Service();assert(g_blackbox_tx_stats.state==BLACKBOX_IDLE);
    HAL_UART_ErrorCallback(&huart5);assert(s_restart_pending);s_restart_pending=0;
    HAL_UART_ErrorCallback(&huart10);assert(g_blackbox_tx_stats.state==BLACKBOX_IDLE);
    reset();Blackbox_Service();advance(6000);Blackbox_Service();advance(6000);Blackbox_Service();assert(g_blackbox_tx_stats.disabled);
    reset();Blackbox_Init(HAL_ERROR);Blackbox_SendSnapshot();assert(!test_dma_calls && g_blackbox_tx_stats.disabled);
    for(i=HAL_ERROR;i<=HAL_TIMEOUT;i++){
        reset();test_dma_status=(HAL_StatusTypeDef)i;Blackbox_Service();assert(g_blackbox_tx_stats.state==BLACKBOX_RECOVER);
        test_abort_immediate=1;Blackbox_Service();assert(!s_schema_ready && s_schema_offset==0);
    }
    reset();test_clean_hook=inject_error;Blackbox_Service();assert(!test_dma_calls && g_blackbox_tx_stats.state==BLACKBOX_RECOVER);
    reset();seed_bmi();Blackbox_Sample(held,123);
    assert(sbb_u16(held+148)==80);
    assert(sbb_u32(held+150)==0xfffffffeU && held[163] && held[164]);
    assert(sbb_u32(held+166)==BMI088_CALIBRATION_VERSION && held[174]);
    for(i=0;i<3;i++){
        float calibrated;memcpy(&calibrated,held+120+4*i,4);
        assert(calibrated==g_bmi088_data.gyro_rad_s[i]);
        memcpy(&calibrated,held+132+4*i,4);
        assert(calibrated==g_bmi088_data.accel_m_s2[i]);
    }
    for(i=0;i<sizeof(s_fields)/sizeof(s_fields[0]);i++)
        assert(!strstr(s_fields[i].name,"bmi088_gyro_raw") && !strstr(s_fields[i].name,"bmi088_accel_raw") && !strstr(s_fields[i].name,"calibrated"));
    test_tick+=20;Blackbox_Sample(held,124);
    assert(!held[163] && held[162] && sbb_u32(held+158)==20);
    {float raw_rate;memcpy(&raw_rate,held+120,4);assert(raw_rate==g_bmi088_data.gyro_rad_s[0] && raw_rate!=-32768*BMI088_GYRO_SCALE);}
    /* Maximum packet bandwidth: at 1 kHz no more than 250 Hz with integer
       control slots; minimum interval is ceil(512/150000 seconds)=3414 us. */
    reset();definition();s_payload_size=484;calls=0;
    for(i=0;i<1000;i++){unsigned before=test_dma_calls;Blackbox_SendSnapshot();
        if(test_dma_calls!=before && test_dma_source[5]==SBB_DATA)calls++;
        if(g_blackbox_tx_stats.busy)HAL_UART_TxCpltCallback(&huart7);advance(1000);}
    assert(calls<=250 && calls>=240);
    /* Export real sender packets. No modifications to F407/converter between
       baseline and -DBLACKBOX_TEST_EXTRA_FIELDS build. */
    reset();if(argc>1){out=fopen(argv[1],"wb");assert(out);}
    for(i=0;i<3000;i++) {
        unsigned before=test_dma_calls;
        g_flight_control_state.roll_rad=.25f;
        seed_bmi();
        Blackbox_SendSnapshot();
        if(test_dma_calls!=before && g_blackbox_tx_stats.state==BLACKBOX_SENDING){
            assert(sbb_valid(test_dma_source,test_dma_size));
            if(out)assert(fwrite(test_dma_source,1,test_dma_size,out)==test_dma_size);
            HAL_UART_TxCpltCallback(&huart7);
        }
        advance(1000);
    }
    assert(g_blackbox_tx_stats.data_completed_count>1200 && g_blackbox_tx_stats.schema_completed_count>10);
    if(out)fclose(out);
    puts("PASS SBB3 schema, DMA ownership/recovery, bandwidth cap, unchanged VOFA, live field extension fixture");return 0;
}
