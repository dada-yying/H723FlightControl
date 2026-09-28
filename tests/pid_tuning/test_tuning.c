/* Host behavioral tests compile the actual production modules against HAL/RTOS stubs. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../App/rc_sbus.c"
#include "../../App/flight_control.c"
#include "../../App/vofa.c"

static AtomImuData_t test_imu;
static uint8_t test_armed;
static float test_motors[4];
void AtomImu_GetData(AtomImuData_t *p) { *p=test_imu; }
uint8_t EscPwm_IsArmed(void) { return test_armed; }
void EscPwm_ForceMinimum(EscPwmInhibitReason_t reason)
{ (void)reason; test_armed=0; memset(test_motors,0,sizeof(test_motors)); }
void EscPwm_SetMixedNormalized(const float p[4]) { memcpy(test_motors,p,sizeof(test_motors)); }
static void near_value(float actual, float expected) { assert(fabsf(actual-expected)<0.0001f); }
static void frame(uint16_t axis, uint16_t slider, uint8_t flags)
{
    uint8_t bytes[25]={0};
    uint16_t channels[16]={0};
    unsigned c,b;
    channels[0]=1008; channels[1]=1000; channels[2]=1022; channels[3]=992;
    channels[4]=axis; channels[9]=slider;
    bytes[0]=0x0f; bytes[23]=flags;
    for(c=0;c<16;c++) for(b=0;b<11;b++)
        if(channels[c] & (1U<<b)) bytes[1+(c*11+b)/8] |= (uint8_t)(1U<<((c*11+b)%8));
    test_tick++;
    for(b=0;b<25;b++) RcSbus_ConsumeByte(bytes[b]);
}
static void run(void) { test_imu.gyro_rx_count++; test_dwt.CYCCNT+=480000U; FlightControl_Run(); }
static void test_filter(void)
{
    unsigned i;
    float alpha = 0.001f / (1.0f / (FLIGHT_TWO_PI * 50.0f) + 0.001f);
    float held, expected, decoded[13], rates[6];
    RcSbus_Init(); FlightControl_Init();
    memset(&test_imu,0,sizeof(test_imu));
    test_armed=0; test_imu.gyro_valid=1;
    test_imu.gyro_x=-2; test_imu.gyro_y=1; test_imu.gyro_z=3;
    run();
    near_value(g_flight_control_filter.applied_cutoff_hz,50);
    near_value(g_flight_control_state.roll_rate_rad_s,1);
    near_value(g_flight_control_state.pitch_rate_rad_s,2);
    near_value(g_flight_control_state.yaw_rate_rad_s,-3);
    test_imu.gyro_x=0; test_imu.gyro_y=2;
    run(); /* RC invalid/locked must not prevent filtering. */
    near_value(g_flight_control_state.roll_rate_rad_s,1+alpha);
    near_value(g_flight_control_state.pitch_rate_rad_s,2-2*alpha);
    near_value(g_flight_control_state.yaw_rate_rad_s,-3);
    held=g_flight_control_state.roll_rate_rad_s;
    test_dwt.CYCCNT+=480000; FlightControl_Run();
    near_value(g_flight_control_state.roll_rate_rad_s,held);
    test_imu.gyro_rx_count++; test_dwt.CYCCNT+=960000;
    FlightControl_Run();
    expected=held+(0.002f/(1.0f/(FLIGHT_TWO_PI*50)+0.002f))*(2-held);
    near_value(g_flight_control_state.roll_rate_rad_s,expected);
    for(i=0;i<100;i++) run();
    near_value(g_flight_control_state.roll_rate_rad_s,2);
    near_value(g_flight_control_state.pitch_rate_rad_s,0);
    test_imu.gyro_y=-2; run();
    near_value(g_flight_control_state.roll_rate_rad_s,2-4*alpha);
    held=g_flight_control_state.roll_rate_rad_s;
    /* Active control must use the same filtered feedback exported to VOFA. */
    frame(1696,1024,0); test_armed=1;
    g_flight_control_pid.rate_roll.kp=0.1f;
    g_flight_control_filter.cutoff_hz=100;
    run();
    expected=held+(0.001f/(1.0f/(FLIGHT_TWO_PI*100)+0.001f))*(-2-held);
    near_value(g_flight_control_state.roll_rate_rad_s,expected);
    near_value(test_motors[0],g_rc_sbus_data.throttle-0.1f*expected);
    near_value(g_flight_control_filter.applied_cutoff_hz,100);
    { const float invalid[]={0,-1,4,201,NAN,INFINITY,-INFINITY};
      for(i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++) {
        g_flight_control_filter.cutoff_hz=invalid[i]; run();
        near_value(g_flight_control_filter.applied_cutoff_hz,100);
      } }
    g_flight_control_filter.cutoff_hz=5; run();
    near_value(g_flight_control_filter.applied_cutoff_hz,5);
    g_flight_control_filter.cutoff_hz=200; run();
    near_value(g_flight_control_filter.applied_cutoff_hz,200);
    test_tick+=11; FlightControl_OnTaskTimeout();
    assert(!s_filter_initialized && !test_armed);
    held=g_flight_control_state.roll_rate_rad_s;
    FlightControl_Run(); /* Cached valid flag is not a new sample after timeout. */
    assert(!s_filter_initialized);
    near_value(g_flight_control_state.roll_rate_rad_s,held);
    test_imu.gyro_y=4; run();
    near_value(g_flight_control_state.roll_rate_rad_s,4);
    test_imu.gyro_valid=0; run(); assert(!s_filter_initialized);
    test_imu.gyro_valid=1; test_imu.gyro_y=5; run();
    near_value(g_flight_control_state.roll_rate_rad_s,5);
    test_imu.gyro_rx_count=0xfffffffeU; run(); run(); /* Counter wrap. */
    assert(s_filter_rx_count==0);
    rates[0]=0; rates[1]=g_flight_control_state.roll_rate_rad_s*180/3.14159265358979323846f;
    rates[2]=0; rates[3]=g_flight_control_state.pitch_rate_rad_s*180/3.14159265358979323846f;
    rates[4]=0; rates[5]=g_flight_control_state.yaw_rate_rad_s*180/3.14159265358979323846f;
    assert(VOFA_SendFlightTelemetry(0,0,0,0,0,0,0,rates)==HAL_OK);
    memcpy(decoded,test_uart_bytes,sizeof(decoded));
    near_value(decoded[8],rates[1]); near_value(decoded[10],rates[3]); near_value(decoded[12],rates[5]);
    assert(test_uart_size==56);
    puts("PASS: gyro LPF initialization, steps, dt, duplicates, live cutoff, invalid requests, recovery, PID and telemetry");
}
static void test_rate_axis_mapping(void)
{
    unsigned axis, i;
    const float rad_to_deg=180.0f/3.14159265358979323846f;
    float correction, throttle, rates[6], decoded[13];
    FlightControlState_t state;
    for(axis=0;axis<2;axis++) {
        RcSbus_Init(); FlightControl_Init();
        memset(&test_imu,0,sizeof(test_imu));
        test_imu.gyro_valid=1; test_imu.euler_valid=1;
        test_imu.gyro_x=0.3f; test_imu.gyro_y=0.2f; test_imu.gyro_z=-0.4f;
        test_imu.roll=0.1f; test_imu.pitch=-0.2f; test_imu.yaw=0.3f;
        frame(1696,1024,0); test_armed=1;
        g_flight_control_pid.angle_roll.kp=0.5f;
        g_flight_control_pid.angle_pitch.kp=0.5f;
        if(axis==0) g_flight_control_pid.rate_roll.kp=0.1f;
        else g_flight_control_pid.rate_pitch.kp=0.1f;
        run(); FlightControl_GetState(&state);
        near_value(state.roll_rate_rad_s,0.2f);   /* +gyro_y */
        near_value(state.pitch_rate_rad_s,-0.3f); /* -gyro_x */
        near_value(state.yaw_rate_rad_s,0.4f);
        /* Euler mapping and outer-loop targets must not be swapped. */
        near_value(state.roll_rad,-0.1f); near_value(state.pitch_rad,-0.2f);
        near_value(state.yaw_rad,0.3f);
        near_value(state.roll_rate_target_rad_s,0.05f);
        near_value(state.pitch_rate_target_rad_s,0.1f);
        near_value(state.yaw_rate_target_rad_s,0);
        correction=(axis==0) ? 0.1f*(0.05f-0.2f) : 0.1f*(0.1f+0.3f);
        throttle=g_rc_sbus_data.throttle;
        near_value(test_motors[0],throttle+correction);
        near_value(test_motors[1],throttle+((axis==0) ? -correction : correction));
        near_value(test_motors[2],throttle-correction);
        near_value(test_motors[3],throttle+((axis==0) ? correction : -correction));
        rates[0]=state.roll_rate_target_rad_s*rad_to_deg;
        rates[1]=state.roll_rate_rad_s*rad_to_deg;
        rates[2]=state.pitch_rate_target_rad_s*rad_to_deg;
        rates[3]=state.pitch_rate_rad_s*rad_to_deg;
        rates[4]=state.yaw_rate_target_rad_s*rad_to_deg;
        rates[5]=state.yaw_rate_rad_s*rad_to_deg;
        assert(VOFA_SendFlightTelemetry(test_imu.roll*rad_to_deg,
            test_imu.pitch*rad_to_deg,test_imu.yaw*rad_to_deg,throttle,
            g_rc_sbus_data.roll_target_rad*rad_to_deg,
            g_rc_sbus_data.pitch_target_rad*rad_to_deg,
            g_rc_sbus_data.yaw_rate_target_rad_s*rad_to_deg,rates)==HAL_OK);
        memcpy(decoded,test_uart_bytes,sizeof(decoded));
        near_value(decoded[0],0.1f*rad_to_deg);
        near_value(decoded[1],-0.2f*rad_to_deg);
        near_value(decoded[2],0.3f*rad_to_deg);
        near_value(decoded[3],throttle);
        for(i=4;i<7;i++) near_value(decoded[i],0);
        for(i=0;i<6;i++) near_value(decoded[7+i],rates[i]);
        assert(test_uart_size==56 && test_uart_timeout==10);
        near_value(test_imu.gyro_x,0.3f); near_value(test_imu.gyro_y,0.2f);
    }
    puts("PASS: swapped Roll/Pitch feedback, independent PID motor corrections, unchanged Euler/targets and VOFA layout");
}
int main(void)
{
    unsigned i;
    float rates[6]={11,12,13,14,15,16}, decoded[13];
    RcSbus_Init(); FlightControl_Init(); test_imu.gyro_valid=1;
    frame(1696,352,0); frame(1696,352,0);
    assert(!g_rc_sbus_data.tuning_axis_valid);
    frame(1696,352,0);
    assert(g_rc_sbus_data.tuning_axis==RC_TUNING_ROLL);
    near_value(g_rc_sbus_data.tuning_rate_rad_s,1.570796327f);
    frame(1696,1696,0); near_value(g_rc_sbus_data.tuning_rate_rad_s,-1.570796327f);
    frame(1696,1024,0); near_value(g_rc_sbus_data.tuning_rate_rad_s,0);
    frame(1696,1040,0); near_value(g_rc_sbus_data.tuning_rate_rad_s,0);
    frame(1696,0,0); near_value(g_rc_sbus_data.tuning_rate_rad_s,1.570796327f);
    frame(1696,2047,0); near_value(g_rc_sbus_data.tuning_rate_rad_s,-1.570796327f);
    g_flight_control_tuning.pid_debug=1; run();
    assert(g_flight_control_tuning.active && g_flight_control_tuning.waiting_center);
    frame(1696,1024,0); run(); assert(!g_flight_control_tuning.waiting_center);
    test_armed=1; run(); assert(g_flight_control_tuning.waiting_center);
    frame(1696,1024,0); run(); assert(!g_flight_control_tuning.waiting_center);
    g_flight_control_pid.rate_roll.kp=0.1f;
    g_flight_control_pid.rate_pitch.kp=0.1f;
    g_flight_control_pid.rate_yaw.kp=0.1f;
    test_imu.gyro_y=0.8f; test_imu.gyro_z=0.7f;
    frame(1696,352,0); run();
    near_value(g_flight_control_state.roll_rate_target_rad_s,1.570796327f);
    near_value(g_flight_control_state.pitch_rate_target_rad_s,0);
    near_value(g_flight_control_state.yaw_rate_target_rad_s,0);
    near_value(test_motors[0],test_motors[3]); near_value(test_motors[1],test_motors[2]);
    near_value(s_pitch_integral,0); near_value(s_yaw_integral,0);
    g_flight_control_tuning.pid_debug=0; run(); assert(g_flight_control_tuning.active);
    for(i=0;i<3;i++) { frame(1024,352,0); run(); }
    assert(g_flight_control_tuning.selected_axis==RC_TUNING_YAW);
    assert(g_flight_control_tuning.waiting_center);
    for(i=0;i<4;i++) near_value(test_motors[i],g_rc_sbus_data.throttle);
    frame(1024,1024,0); run(); frame(1024,352,0); run();
    near_value(test_motors[0],test_motors[2]); near_value(test_motors[1],test_motors[3]);
    near_value(s_roll_integral,0); near_value(s_pitch_integral,0);
    for(i=0;i<3;i++) { frame(352,352,0); run(); }
    assert(g_flight_control_tuning.selected_axis==RC_TUNING_PITCH);
    frame(352,1024,0); run(); frame(352,352,0); run();
    near_value(test_motors[0],test_motors[1]); near_value(test_motors[2],test_motors[3]);
    near_value(s_roll_integral,0); near_value(s_yaw_integral,0);
    { uint32_t tick=g_rc_sbus_data.last_valid_tick;
      frame(1696,1696,RC_SBUS_FLAG_FRAME_LOST); run();
      assert(g_rc_sbus_data.last_valid_tick==tick);
      near_value(g_rc_sbus_data.tuning_rate_rad_s,1.570796327f);
      test_tick=tick+100; RcSbus_CheckTimeout(); run(); assert(!test_armed); }
    assert(!g_rc_sbus_data.valid); near_value(g_rc_sbus_data.tuning_rate_rad_s,0);
    frame(352,352,0); test_armed=1; frame(352,352,RC_SBUS_FLAG_FAILSAFE); run();
    assert(!test_armed);
    run(); assert(!g_flight_control_tuning.active); /* queued request accepted while locked */
    test_armed=1; test_tick+=11; FlightControl_OnTaskTimeout(); assert(!test_armed);
    near_value(s_roll_integral,0); near_value(s_pitch_integral,0); near_value(s_yaw_integral,0);
    assert(VOFA_SendFlightTelemetry(1,2,3,4,5,6,7,rates)==HAL_OK);
    assert(test_uart_size==56 && test_uart_timeout==10);
    memcpy(decoded,test_uart_bytes,sizeof(decoded));
    for(i=0;i<7;i++) near_value(decoded[i],(float)i+1);
    for(i=0;i<6;i++) near_value(decoded[i+7],rates[i]);
    assert(memcmp(test_uart_bytes+52,"\x00\x00\x80\x7f",4)==0);
    assert(VOFA_SendChannels(rates,14)==HAL_ERROR);
    puts("PASS: mapping, mode gate, fresh-frame recenter, axis isolation, failsafe, timeouts, telemetry");
    test_filter();
    test_rate_axis_mapping();
    return 0;
}
