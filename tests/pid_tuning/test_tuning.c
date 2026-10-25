/* Real acquisition, AHRS, RC, controller, ESC and telemetry; only HAL/RTOS are mocked. */
#define BMI_FLIGHT_INTEGRATION_TEST
#define main bmi_sensor_regression_main
#include "test_bmi088.c"
#undef main
#include "../../App/rc_sbus.c"
#include "../../App/optical_flow.c"
#include "../../App/flight_control.c"
#include "../../App/esc_pwm.c"
#include "../../App/vofa.c"

void Blackbox_OnUartError(UART_HandleTypeDef *uart) { (void)uart; }
static void near_value(float a, float b) { assert(fabsf(a-b)<0.0001f); }

static void source(unsigned us)
{
    unsigned i;
    test_tick += us/1000U;
    test_dwt.CYCCNT += us*480U;
    g_bmi088_data.initialized=g_bmi088_data.valid=g_bmi088_data.has_sample=1;
    g_bmi088_data.sample_count++;
    g_bmi088_data.sample_tick_ms=test_tick;
    g_bmi088_data.completion_cycle=test_dwt.CYCCNT;
    g_bmi088_gyro_filter.initialized=g_bmi088_gyro_filter.valid=g_bmi088_gyro_filter.has_sample=1;
    g_bmi088_gyro_filter.sample_count=g_bmi088_data.sample_count;
    g_bmi088_gyro_filter.sample_tick_ms=test_tick;
    g_bmi088_attitude_state.initialized=g_bmi088_attitude_state.valid=1;
    g_bmi088_attitude_state.epoch=1;
    g_bmi088_attitude_state.sample_count=g_bmi088_data.sample_count;
    g_bmi088_attitude_state.sample_tick_ms=test_tick;
    g_rc_sbus_data.last_valid_tick=test_tick;
    for(i=0;i<3;i++) assert(isfinite(g_bmi088_gyro_filter.gyro_rad_s[i]));
}
static void reset_control(void)
{
    test_tick=100; test_dwt.CYCCNT=0;
    Bmi088_Init(HAL_OK); RcSbus_Init(); FlightControl_Init(); EscPwm_Init();
    test_tick += ESC_PWM_STARTUP_DELAY_MS;
    memset((void *)&g_flight_control_pid,0,sizeof(g_flight_control_pid));
    g_flight_control_pid.rate_roll.kp=.2f;
    g_flight_control_pid.rate_pitch.kp=.2f;
    g_flight_control_pid.rate_yaw.kp=.2f;
    g_rc_sbus_data.valid=1;
    g_rc_sbus_data.last_valid_tick=test_tick;
    source(1000);
}
static void arm(void)
{
    RcSbusData_t rc;
    g_rc_sbus_data.throttle=0;
    g_rc_sbus_data.channel[6]=1000;
    RcSbus_GetData(&rc); EscPwm_UpdateFromRc(&rc);
    g_rc_sbus_data.channel[6]=1696;
    RcSbus_GetData(&rc); EscPwm_UpdateFromRc(&rc);
    assert(EscPwm_IsArmed());
    g_rc_sbus_data.throttle=.5f;
}
/* Feed actual SBUS parser including calibrated endpoints and candidate debounce. */
static void frame(unsigned ch1,unsigned ch2,unsigned ch4,unsigned axis,unsigned slider,unsigned flags)
{
    uint8_t b[25]={0};
    uint16_t ch[16]={0};
    unsigned c,k;
    ch[0]=(uint16_t)ch1; ch[1]=(uint16_t)ch2; ch[2]=348;
    ch[3]=(uint16_t)ch4; ch[4]=(uint16_t)axis; ch[6]=1000; ch[9]=(uint16_t)slider;
    b[0]=15; b[23]=(uint8_t)flags;
    for(c=0;c<16;c++) for(k=0;k<11;k++)
        if(ch[c]&(1U<<k)) b[1+(c*11+k)/8]|=(uint8_t)(1U<<((c*11+k)%8));
    test_tick++;
    for(k=0;k<25;k++) RcSbus_ConsumeByte(b[k]);
}
static void test_rc_directions(void)
{
    unsigned i;
    reset_control();
    frame(1696,352,1696,1696,352,0);
    assert(g_rc_sbus_data.pitch_target_rad>0); /* Right stick -> right bank. */
    assert(g_rc_sbus_data.roll_target_rad<0); /* Forward stick -> nose down. */
    assert(g_rc_sbus_data.yaw_rate_target_rad_s<0); /* Right turn = negative Z. */
    near_value(g_rc_sbus_data.pitch_target_rad,RC_SBUS_MAX_TILT_RAD);
    frame(352,1696,352,1696,352,0);
    assert(g_rc_sbus_data.pitch_target_rad<0 && g_rc_sbus_data.roll_target_rad>0 &&
           g_rc_sbus_data.yaw_rate_target_rad_s>0);
    for(i=0;i<3;i++) frame(1024,1040,1024,1696,352,0);
    assert(g_rc_sbus_data.tuning_axis==RC_TUNING_PITCH && g_rc_sbus_data.tuning_rate_rad_s>0);
    for(i=0;i<2;i++) {
        frame(1024,1040,1024,352,352,0);
        assert(g_rc_sbus_data.tuning_axis==RC_TUNING_PITCH && g_rc_sbus_data.tuning_rate_rad_s>0);
    }
    frame(1024,1040,1024,352,352,0);
    assert(g_rc_sbus_data.tuning_axis==RC_TUNING_ROLL && g_rc_sbus_data.tuning_rate_rad_s<0);
    for(i=0;i<3;i++) frame(1024,1040,1024,1024,352,0);
    assert(g_rc_sbus_data.tuning_axis==RC_TUNING_YAW && g_rc_sbus_data.tuning_rate_rad_s<0);
    { uint32_t tick=g_rc_sbus_data.last_valid_tick;
      frame(1696,1696,1696,1696,1696,RC_SBUS_FLAG_FRAME_LOST);
      assert(g_rc_sbus_data.last_valid_tick==tick);
      test_tick=tick+100; RcSbus_CheckTimeout(); assert(!g_rc_sbus_data.valid); }
    frame(1024,1040,1024,1024,1024,RC_SBUS_FLAG_FAILSAFE);
    assert(!g_rc_sbus_data.valid && g_rc_sbus_data.failsafe);
}
/* Independently calculate physical torques from known motor positions/spin.
 * X right, Y forward, thrust +Z; reaction yaw opposes prop spin. */
static void torques(float out[3])
{
    const int x[4]={-1,1,1,-1}, y[4]={-1,-1,1,1}, z[4]={-1,1,-1,1};
    unsigned i;
    memset(out,0,3*sizeof(float));
    for(i=0;i<4;i++) {
        float d=g_esc_pwm_state.command[i]-.5f;
        out[0]+=y[i]*d; out[1]-=x[i]*d; out[2]+=z[i]*d;
    }
}
static void test_restoring(void)
{
    unsigned axis, sign, kind, i;
    for(kind=0;kind<2;kind++) for(axis=0;axis<3;axis++) for(sign=0;sign<2;sign++) {
        float value=sign ? -.2f : .2f, moment[3];
        if(kind && axis==2) continue; /* Heading has no outer angle loop. */
        reset_control(); arm();
        if(kind) {
            g_flight_control_pid.angle_roll.kp=1;
            g_flight_control_pid.angle_pitch.kp=1;
            if(axis==0) g_bmi088_attitude_state.roll_rad=value;
            else g_bmi088_attitude_state.pitch_rad=value;
        } else g_bmi088_gyro_filter.gyro_rad_s[axis]=value;
        FlightControl_Run(); torques(moment);
        assert(g_flight_control_state.running && g_flight_control_state.imu_valid);
        assert(moment[axis]*value<0); /* Always a restoring physical torque. */
        for(i=0;i<3;i++) if(i!=axis) near_value(moment[i],0);
    }
}
static void test_source_and_timing(void)
{
    float held, rates[6]={0}, decoded[20];
    unsigned i, calls;
    reset_control(); arm();
    for(i=0;i<3;i++) { g_bmi088_data.gyro_rad_s[i]=(float)i+4; g_bmi088_gyro_filter.gyro_rad_s[i]=.1f*(i+1); }
    g_bmi088_attitude_state.roll_rad=.2f;
    g_bmi088_attitude_state.pitch_rad=-.3f;
    g_bmi088_attitude_state.yaw_rad=.4f;
    FlightControl_Run();
    near_value(g_flight_control_state.roll_rad,.2f);
    near_value(g_flight_control_state.pitch_rad,-.3f);
    near_value(g_flight_control_state.yaw_rad,.4f);
    near_value(g_flight_control_state.roll_rate_rad_s,.1f);
    near_value(g_flight_control_state.pitch_rate_rad_s,.2f);
    near_value(g_flight_control_state.yaw_rate_rad_s,.3f);
    near_value(g_flight_control_state.roll_rate_raw_rad_s,4);
    near_value(g_flight_control_state.pitch_rate_raw_rad_s,5);
    near_value(g_flight_control_state.yaw_rate_raw_rad_s,6);
    held=s_roll_integral;
    test_dwt.CYCCNT+=480000; FlightControl_Run();
    near_value(s_roll_integral,held); /* Duplicate does not advance PID. */
    source(1250); /* Previous unrelated task delay is deliberately excluded. */
    g_bmi088_data.completion_cycle=s_last_cycle+600000;
    FlightControl_Run(); near_value(g_flight_control_state.dt_s,.00125f);
    s_last_cycle=UINT32_MAX-239999U;
    source(1000); g_bmi088_data.completion_cycle=240000U;
    FlightControl_Run(); near_value(g_flight_control_state.dt_s,.001f);
    FlightControl_ResetController(); near_value(g_bmi088_gyro_filter.gyro_rad_s[0],.1f);
    g_flight_control_filter.q=1000; g_flight_control_filter.r=.000001f;
    source(1000); FlightControl_Run();
    near_value(g_flight_control_state.roll_rate_rad_s,.1f);
    assert(!g_flight_control_notch.active && !g_flight_control_notch.sample_count);
    { EscPwmState_t esc; EscPwm_GetState(&esc);
      assert(VOFA_SendFlightTelemetry(.2f,-.3f,.4f,.5f,0,0,0,rates,4,5,6,esc.command)==HAL_OK); }
    assert(test_uart_size==84);
    memcpy(decoded,test_uart_bytes,sizeof(decoded));
    near_value(decoded[0],.2f); near_value(decoded[1],-.3f);
    near_value(decoded[13],4); near_value(decoded[14],5); near_value(decoded[15],6);
    /* Exercise the actual shared callback router: USART1 error must not
     * restart UART5 or steal its receive-fault handling. */
    test_uart_dma_auto_complete=0;
    assert(VOFA_SendAttitude(0,0,0)==HAL_OK);
    s_restart_pending=0;HAL_UART_ErrorCallback(&huart1);
    assert(!s_restart_pending && s_vofa_tx_state==VOFA_TX_RECOVER);
    assert(VOFA_SendAttitude(0,0,0)==HAL_OK);HAL_UART_TxCpltCallback(&huart1);
    test_uart_dma_auto_complete=1;
    FlightControl_SetTaskHandle((TaskHandle_t)7);
    calls=test_notify_calls;
    FlightControl_OnBmi088Update(1); assert(test_notify_calls==calls);
    FlightControl_OnBmi088Update(1); assert(test_notify_calls==calls+1 && test_notify_task==(TaskHandle_t)7);
    FlightControl_OnBmi088Update(1); FlightControl_OnBmi088Update(0);
    assert(test_notify_calls==calls+2);
    FlightControl_OnBmi088Update(1); assert(test_notify_calls==calls+2);
}
static void test_faults(void)
{
    unsigned kind,i;
    for(kind=0;kind<10;kind++) {
        reset_control(); arm(); FlightControl_Run();
        source(1000);
        switch(kind) {
        case 0:g_bmi088_data.initialized=0; break;
        case 1:g_bmi088_gyro_filter.valid=0; break;
        case 2:g_bmi088_attitude_state.valid=0; break;
        case 3:g_bmi088_gyro_filter.sample_count--; break;
        case 4:g_bmi088_attitude_state.sample_tick_ms--; break;
        case 5:g_bmi088_gyro_filter.gyro_rad_s[1]=NAN; break;
        case 6:g_bmi088_attitude_state.roll_rad=INFINITY; break;
        case 7:g_bmi088_data.gyro_rad_s[2]=NAN; break;
        case 8:test_tick+=10; break;
        case 9:g_bmi088_data.completion_cycle=s_last_cycle; break;
        }
        FlightControl_Run();
        assert(!EscPwm_IsArmed() && !g_flight_control_state.running && !g_flight_control_state.imu_valid);
        for(i=0;i<4;i++) near_value(g_esc_pwm_state.command[i],0);
        assert(g_esc_pwm_state.inhibit_reason==ESC_PWM_INHIBIT_IMU_TIMEOUT);
        /* Data recovery cannot automatically restore arming. */
        g_bmi088_gyro_filter.gyro_rad_s[1]=0;
        g_bmi088_data.gyro_rad_s[2]=0; g_bmi088_attitude_state.roll_rad=0;
        source(1000); FlightControl_Run(); assert(!EscPwm_IsArmed());
    }
    reset_control(); arm(); FlightControl_Run(); test_tick+=10;
    FlightControl_OnTaskTimeout(); assert(!EscPwm_IsArmed());
    reset_control(); arm(); g_rc_sbus_data.failsafe=1; FlightControl_Run(); assert(!EscPwm_IsArmed());
}
static void test_real_pipeline(void)
{
    unsigned i, axis;
    model_reset(100); ready(); measurement(136);
    RcSbus_Init(); FlightControl_Init(); EscPwm_Init();
    FlightControl_SetTaskHandle((TaskHandle_t)7);
    for (axis=0;axis<3;axis++) {
        put_i16(0,2+2*axis,(int)roundf(g_bmi088_calibration.gyro_bias_rad_s[axis]/BMI088_GYRO_SCALE));
        put_i16(1,0x12+2*axis,(int)roundf((g_bmi088_calibration.accel_bias_m_s2[axis]+
            (axis==2 ? 9.80665f/g_bmi088_calibration.accel_scale[axis] : 0))/BMI088_ACCEL_SCALE));
    }
    for(i=0;i<130;i++) {
        test_dwt.CYCCNT+=240000; test_tick+=i&1U;
        g_rc_sbus_data.valid=1; g_rc_sbus_data.last_valid_tick=test_tick;
        event_process();
        if(test_notify_task==(TaskHandle_t)7) {
            FlightControl_Run();
            if(g_flight_control_state.imu_valid) assert(s_sample_count==g_bmi088_data.sample_count);
        }
    }
    assert(g_bmi088_attitude_state.valid && g_flight_control_state.imu_valid);
    assert((uint32_t)(g_bmi088_data.sample_count-s_sample_count)<=1U);
    near_value(g_flight_control_state.roll_rate_rad_s,g_bmi088_gyro_filter.gyro_rad_s[0]);
    near_value(g_flight_control_state.pitch_rate_rad_s,g_bmi088_gyro_filter.gyro_rad_s[1]);
    near_value(g_flight_control_state.yaw_rate_rad_s,g_bmi088_gyro_filter.gyro_rad_s[2]);
    near_value(g_flight_control_state.roll_rad,g_bmi088_attitude_state.roll_rad);
    near_value(g_flight_control_state.dt_s,.001f);
    fail_call=test_spi_calls+1; fail_status=HAL_TIMEOUT; event_process();
    assert(test_notify_task==(TaskHandle_t)7);
    FlightControl_Run();
    assert(!g_flight_control_state.imu_valid && !g_flight_control_state.running);
}
static void test_tuning_gate(void)
{
    unsigned i;
    reset_control(); g_flight_control_tuning.pid_debug=1;
    for(i=0;i<3;i++) frame(1024,1040,1024,1696,352,0);
    source(1000); FlightControl_Run(); assert(g_flight_control_tuning.active);
    arm(); source(1000); FlightControl_Run(); assert(g_flight_control_tuning.waiting_center);
    frame(1024,1040,1024,1696,1024,0); source(1000); FlightControl_Run();
    assert(!g_flight_control_tuning.waiting_center);
    frame(1024,1040,1024,1696,352,0); g_rc_sbus_data.throttle=.5f;
    source(1000); FlightControl_Run();
    assert(g_flight_control_state.pitch_rate_target_rad_s>0);
    near_value(g_flight_control_state.roll_rate_target_rad_s,0);
    near_value(g_flight_control_state.yaw_rate_target_rad_s,0);
    g_flight_control_tuning.pid_debug=0; source(1000); FlightControl_Run();
    assert(g_flight_control_tuning.active); /* Cannot change mode in flight. */
    for(i=0;i<3;i++) frame(1024,1040,1024,352,352,0);
    source(1000); FlightControl_Run(); assert(g_flight_control_tuning.waiting_center);
    near_value(s_pitch_integral,0); near_value(s_yaw_integral,0);
}
#ifndef CONTROL_FIXTURE_ONLY
int main(void)
{
    test_rc_directions(); test_restoring(); test_source_and_timing(); test_faults(); test_tuning_gate();
    test_real_pipeline();
    puts("PASS BMI-only RFU control: physical RC/motor signs, raw/LPF, sample dt/wrap/duplicate, notify divider, safety/rearm, tuning and telemetry");
    return 0;
}
#endif
