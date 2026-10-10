#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "../../ThirdParty/Fusion/FusionAhrs.c"
#include "../../App/bmi088_attitude.c"
#include "../../App/rc_sbus.c"
#include "../../App/esc_pwm.c"
#include "../../App/flight_control.c"
#include "../../App/vofa.c"
#include "../../App/blackbox.c"
#include "../../App/optical_flow.c"
#include "../../App/bmi088.c"
void AtomImu_GetData(AtomImuData_t *p){memset(p,0,sizeof(*p));}
static void advance(unsigned us){test_dwt.CYCCNT+=us*480U;test_tick+=us/1000U;}
static unsigned s_test_elapsed_us;
static void timer_step(void){
    unsigned period=htim4.active_autoreload+1U;
    test_dwt.CYCCNT+=period*480U;
    s_test_elapsed_us+=period;test_tick=100U+s_test_elapsed_us/1000U;
    htim4.active_autoreload=htim4.autoreload;
    Blackbox_NotifyFromISR();
}
static void recording_step(void){
    timer_step();Blackbox_ProcessPending();
    if(g_blackbox_tx_stats.busy && g_blackbox_tx_stats.state==BLACKBOX_SENDING)
        HAL_UART_TxCpltCallback(&huart7);
}
static void reset(void){
    test_tick=100;test_dwt.CYCCNT=0;test_dma_calls=test_abort_calls=test_clean_calls=0;
    test_dma_status=test_abort_status=HAL_OK;test_abort_immediate=0;test_clean_hook=NULL;test_scb.CCR=SCB_CCR_DC_Msk;
    test_timer_init_status=test_timer_start_status=test_timer_event_status=HAL_OK;s_test_elapsed_us=0;
    RcSbus_Init();FlightControl_Init();assert(EscPwm_Init()==HAL_OK);OpticalFlow_Init(HAL_OK);Bmi088_Init(HAL_ERROR);Blackbox_Init(HAL_OK);
    assert(Blackbox_StartRecording((TaskHandle_t)0x700)==HAL_OK);
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
    /* Known coherent attitude fixture. Numeric estimator tested separately. */
    g_bmi088_attitude_state.roll_rad=.25f;g_bmi088_attitude_state.pitch_rad=-.125f;g_bmi088_attitude_state.yaw_rad=.5f;
    g_bmi088_attitude_state.sample_count=sample.sample_count;
    g_bmi088_attitude_state.sample_tick_ms=sample.sample_tick_ms;
    g_bmi088_attitude_state.epoch=7;g_bmi088_attitude_state.initialized=g_bmi088_attitude_state.valid=1;
    g_bmi088_attitude_state.accel_used=1;g_bmi088_attitude_state.error=BMI088_ATTITUDE_OK;
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
    assert(sbb_u32(test_dma_source+16)==(uint32_t)Blackbox_NowUs());
    assert(sbb_u32(test_dma_source+20)==(uint32_t)(Blackbox_NowUs()>>32));
    Blackbox_Sample(held);
    assert(sbb_u32(held+148)==UINT32_MAX && !held[152] && !held[153]);
#ifndef BLACKBOX_TEST_EXTRA_FIELDS
    assert(s_payload_size==184 && test_dma_size==212);
#endif
    assert(!memcmp(expected,test_dma_source+24,80) && test_uart_size==96);
    for(i=0;i<4;i++){float motor;memcpy(&motor,test_dma_source+24+64+4*i,4);assert(motor==0);}
    memcpy(held,test_dma_source,512);calls=test_dma_calls;
    Blackbox_SendSnapshot();assert(test_dma_calls==calls && !memcmp(held,test_dma_source,512));
    HAL_UART_TxCpltCallback(&huart1);assert(g_blackbox_tx_stats.busy);
    HAL_UART_TxCpltCallback(&huart7);Blackbox_ProcessPending();assert(test_dma_calls==calls);
    advance(2000);Blackbox_NotifyFromISR();Blackbox_ProcessPending();assert(test_dma_calls==calls+1);
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
    reset();seed_bmi();Blackbox_Sample(held);
    { Bmi088Data_t before=g_bmi088_data;
      Bmi088Data_t after;
      Blackbox_Sample(held);
      after=g_bmi088_data;
      assert(!memcmp(&before,&after,sizeof(before))); }
    { unsigned found=0;
      for(i=0;i<sizeof(s_fields)/sizeof(s_fields[0]);i++) {
          assert(strcmp(s_fields[i].name,"bmi088_gyro_rad_s"));
          if(!strcmp(s_fields[i].name,"bmi088_gyro_deg_s")) {
              assert(!strcmp(s_fields[i].unit,"deg/s") && s_fields[i].offset==112 &&
                  s_fields[i].count==3 && s_fields[i].type==SBB_F32);
              found++;
          }
      }
      assert(found==1); }
    assert(sbb_u32(held+140)==0xfffffffeU && held[153] && held[154]);
    assert(sbb_u32(held+156)==BMI088_CALIBRATION_VERSION && held[164]);
    assert(held[177] && sbb_u32(held+178)==7 && held[182] && held[183]==BMI088_ATTITUDE_OK);
    { float angle;memcpy(&angle,held+165,4);assert(angle==.25f*57.295779513f);
      memcpy(&angle,held+169,4);assert(angle==-.125f*57.295779513f);
      memcpy(&angle,held+173,4);assert(angle==.5f*57.295779513f); }
    for(i=0;i<3;i++){
        float calibrated;memcpy(&calibrated,held+112+4*i,4);
        assert(calibrated==g_bmi088_data.gyro_rad_s[i]*57.295779513f);
        memcpy(&calibrated,held+124+4*i,4);
        assert(calibrated==g_bmi088_data.accel_m_s2[i]);
    }
    for(i=0;i<sizeof(s_fields)/sizeof(s_fields[0]);i++)
        assert(!strstr(s_fields[i].name,"bmi088_gyro_raw") && !strstr(s_fields[i].name,"bmi088_accel_raw") && !strstr(s_fields[i].name,"calibrated") &&
            strcmp(s_fields[i].name,"fc_timestamp_us") && strcmp(s_fields[i].name,"bmi088_temperature_raw"));
    test_tick+=20;Blackbox_Sample(held);
    assert(!held[153] && held[152] && sbb_u32(held+148)==20);
    assert(!held[177]); /* Stale attitude must not masquerade as usable. */
    {float raw_rate;memcpy(&raw_rate,held+112,4);assert(raw_rate==g_bmi088_data.gyro_rad_s[0]*57.295779513f && raw_rate!=-32768*BMI088_GYRO_SCALE*57.295779513f);}
    /* Independent 700 Hz (or field-size-limited) timer; no flight calls. */
    reset();definition();calls=0;
    for(i=0;i<g_blackbox_tx_stats.effective_hz;i++){unsigned before=test_dma_calls;timer_step();Blackbox_ProcessPending();
        if(test_dma_calls!=before && test_dma_source[5]==SBB_DATA)calls++;
        if(g_blackbox_tx_stats.busy)HAL_UART_TxCpltCallback(&huart7);}
    assert(calls==g_blackbox_tx_stats.effective_hz && s_test_elapsed_us==1000000U);
    /* Export real sender packets. No modifications to F407/converter between
       baseline and -DBLACKBOX_TEST_EXTRA_FIELDS build. */
    reset();if(argc>1){out=fopen(argv[1],"wb");assert(out);}
    for(i=0;i<3000;i++) {
        unsigned before=test_dma_calls;
        g_flight_control_state.roll_rad=.25f;
        timer_step();seed_bmi();Blackbox_ProcessPending();
        if(test_dma_calls!=before && g_blackbox_tx_stats.state==BLACKBOX_SENDING){
            assert(sbb_valid(test_dma_source,test_dma_size));
            if(out)assert(fwrite(test_dma_source,1,test_dma_size,out)==test_dma_size);
            HAL_UART_TxCpltCallback(&huart7);
        }
    }
    assert(g_blackbox_tx_stats.data_completed_count>1200 && g_blackbox_tx_stats.schema_completed_count>10);
    if(out)fclose(out);
    puts("PASS SBB3 schema, DMA ownership/recovery, bandwidth cap, unchanged VOFA, live field extension fixture");return 0;
}
