/* Host behavioral tests compile the actual production modules against HAL/RTOS stubs. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../App/rc_sbus.c"
#include "../../App/optical_flow.c"
#include "../../App/flight_control.c"
#include "../../App/vofa.c"

static AtomImuData_t test_imu;
void Blackbox_OnUartError(UART_HandleTypeDef *uart) { (void)uart; }
static uint8_t test_armed;
static float test_motors[4];
void AtomImu_GetData(AtomImuData_t *p) { *p=test_imu; }
uint8_t EscPwm_IsArmed(void) { return test_armed; }
void EscPwm_ForceMinimum(EscPwmInhibitReason_t reason)
{ (void)reason; test_armed=0; memset(test_motors,0,sizeof(test_motors)); }
void EscPwm_SetMixedNormalized(const float p[4]) { memcpy(test_motors,p,sizeof(test_motors)); }
/* Fix the reference response explicitly, without changing production defaults. */
static void test_flight_init(void)
{
    FlightControl_Init();
    g_flight_control_filter.q=0.07510204f;
    g_flight_control_filter.r=1.0f;
}
static void near_value(float actual, float expected) { assert(fabsf(actual-expected)<0.0001f); }
static void frame(uint16_t axis, uint16_t slider, uint8_t flags)
{
    uint8_t bytes[25]={0};
    uint16_t channels[16]={0};
    unsigned c,b;
    channels[0]=RC_SBUS_ROLL_CENTER; channels[1]=RC_SBUS_PITCH_CENTER;
    channels[2]=1022; channels[3]=RC_SBUS_YAW_CENTER;
    channels[4]=axis; channels[9]=slider;
    bytes[0]=0x0f; bytes[23]=flags;
    for(c=0;c<16;c++) for(b=0;b<11;b++)
        if(channels[c] & (1U<<b)) bytes[1+(c*11+b)/8] |= (uint8_t)(1U<<((c*11+b)%8));
    test_tick++;
    for(b=0;b<25;b++) RcSbus_ConsumeByte(bytes[b]);
}
static void run(void) { test_imu.gyro_rx_count++; test_dwt.CYCCNT+=480000U; FlightControl_Run(); }
/* Independent double-precision reference with the scalar posterior formula. */
static double reference_kalman(double *x, double *p, double z, double q, double r, double dt)
{
    double predicted=*p+q*dt/0.001;
    double gain=predicted/(predicted+r);
    *x+=gain*(z-*x);
    *p=r*predicted/(predicted+r);
    return gain;
}
static void test_filter(void)
{
    unsigned i;
    const float alpha = 0.001f / (1.0f / (6.283185307179586f * 50.0f) + 0.001f);
    float held, covariance, gain, expected, decoded[20], rates[6];
    double ref_x, ref_p, ref_gain;
    RcSbus_Init(); test_flight_init();
    memset(&test_imu,0,sizeof(test_imu));
    test_armed=0; test_imu.gyro_valid=1;
    test_imu.gyro_x=-2; test_imu.gyro_y=-1; test_imu.gyro_z=3;
    run();
    near_value(g_flight_control_filter.applied_q,0.07510204f);
    near_value(g_flight_control_filter.applied_r,1);
    for(i=0;i<3;i++) {
        near_value(s_filter_covariance[i],alpha);
        near_value(g_flight_control_filter.gain[i],1); /* Direct first sample. */
    }
    near_value(g_flight_control_state.roll_rate_rad_s,1);
    near_value(g_flight_control_state.pitch_rate_rad_s,-2);
    near_value(g_flight_control_state.yaw_rate_rad_s,-3);
    test_imu.gyro_x=0; test_imu.gyro_y=-2;
    run(); /* RC invalid/locked must not prevent filtering. */
    near_value(g_flight_control_filter.gain[0],alpha);
    near_value(g_flight_control_state.roll_rate_rad_s,1+alpha);
    near_value(g_flight_control_state.pitch_rate_rad_s,-2+2*alpha);
    near_value(g_flight_control_state.yaw_rate_rad_s,-3);
    held=g_flight_control_state.roll_rate_rad_s;
    covariance=s_filter_covariance[0]; gain=g_flight_control_filter.gain[0];
    g_flight_control_filter.q=0.3f; g_flight_control_filter.r=0.5f;
    test_dwt.CYCCNT+=480000; FlightControl_Run();
    near_value(g_flight_control_state.roll_rate_rad_s,held);
    near_value(s_filter_covariance[0],covariance);
    near_value(g_flight_control_filter.gain[0],gain);
    near_value(g_flight_control_filter.applied_q,0.07510204f); /* No fresh sample. */
    near_value(g_flight_control_filter.applied_r,1);
    g_flight_control_filter.q=0.07510204f; g_flight_control_filter.r=1;
    ref_x=held; ref_p=covariance;
    ref_gain=reference_kalman(&ref_x,&ref_p,2,0.07510204,1,0.002);
    test_imu.gyro_rx_count++; test_dwt.CYCCNT+=960000;
    FlightControl_Run();
    near_value(g_flight_control_state.roll_rate_rad_s,(float)ref_x);
    near_value(s_filter_covariance[0],(float)ref_p);
    near_value(g_flight_control_filter.gain[0],(float)ref_gain);
    for(i=0;i<100;i++) run();
    near_value(g_flight_control_state.roll_rate_rad_s,2);
    near_value(g_flight_control_state.pitch_rate_rad_s,0);
    test_imu.gyro_y=2; run();
    near_value(g_flight_control_state.roll_rate_rad_s,2-4*alpha);
    held=g_flight_control_state.roll_rate_rad_s;
    /* Active control must use the same filtered feedback exported to VOFA. */
    frame(1696,1024,0); test_armed=1;
    g_flight_control_pid.rate_roll.kp=0.1f;
    g_flight_control_pid.rate_roll.ki=0.01f;
    g_flight_control_filter.q=0.3f; g_flight_control_filter.r=0.5f;
    s_roll_integral=0.12f; s_last_roll_error=0.456f; s_have_error=1;
    ref_x=held; ref_p=s_filter_covariance[0];
    ref_gain=reference_kalman(&ref_x,&ref_p,-2,0.3,0.5,0.001);
    run();
    expected=(float)ref_x;
    near_value(g_flight_control_state.roll_rate_rad_s,expected);
    near_value(s_filter_covariance[0],(float)ref_p);
    near_value(g_flight_control_filter.gain[0],(float)ref_gain);
    near_value(s_roll_integral,0.12f-expected*0.001f); /* No reset on live edit. */
    near_value(test_motors[0],g_rc_sbus_data.throttle-0.1f*expected+0.01f*s_roll_integral);
    near_value(g_flight_control_filter.applied_q,0.3f);
    near_value(g_flight_control_filter.applied_r,0.5f);
    /* PID resets must leave both estimate and covariance intact. */
    held=s_filtered_rate[0]; covariance=s_filter_covariance[0];
    FlightControl_ResetController();
    near_value(s_filtered_rate[0],held); near_value(s_filter_covariance[0],covariance);
    { const float invalid[]={0,-1,0.0000001f,1001,NAN,INFINITY,-INFINITY};
      for(i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++) {
        g_flight_control_filter.q=invalid[i]; g_flight_control_filter.r=2; run();
        near_value(g_flight_control_filter.applied_q,0.3f);
        near_value(g_flight_control_filter.applied_r,0.5f);
        g_flight_control_filter.q=2; g_flight_control_filter.r=invalid[i]; run();
        near_value(g_flight_control_filter.applied_q,0.3f);
        near_value(g_flight_control_filter.applied_r,0.5f);
      } }
    g_flight_control_filter.q=0.07510204f; g_flight_control_filter.r=1; run();
    test_tick+=11; FlightControl_OnTaskTimeout();
    assert(!s_filter_initialized && !test_armed);
    held=g_flight_control_state.roll_rate_rad_s;
    FlightControl_Run(); /* Cached valid flag is not a new sample after timeout. */
    assert(!s_filter_initialized);
    near_value(g_flight_control_state.roll_rate_rad_s,held);
    test_imu.gyro_y=-4; run();
    near_value(g_flight_control_state.roll_rate_rad_s,4);
    near_value(s_filter_covariance[0],alpha); near_value(g_flight_control_filter.gain[0],1);
    test_imu.gyro_valid=0; run(); assert(!s_filter_initialized);
    test_imu.gyro_valid=1; test_imu.gyro_y=-5; run();
    near_value(g_flight_control_state.roll_rate_rad_s,5);
    test_imu.gyro_rx_count=0xfffffffeU; run(); run(); /* Counter wrap. */
    assert(s_filter_rx_count==0);
    rates[0]=0; rates[1]=g_flight_control_state.roll_rate_rad_s*180/3.14159265358979323846f;
    rates[2]=0; rates[3]=g_flight_control_state.pitch_rate_rad_s*180/3.14159265358979323846f;
    rates[4]=0; rates[5]=g_flight_control_state.yaw_rate_rad_s*180/3.14159265358979323846f;
    assert(VOFA_SendFlightTelemetry(0,0,0,0,0,0,0,rates,
        g_flight_control_state.roll_rate_raw_rad_s*180/3.14159265358979323846f,
        g_flight_control_state.pitch_rate_raw_rad_s*180/3.14159265358979323846f,
        g_flight_control_state.yaw_rate_raw_rad_s*180/3.14159265358979323846f,
        test_motors)==HAL_OK);
    memcpy(decoded,test_uart_bytes,sizeof(decoded));
    near_value(decoded[8],rates[1]); near_value(decoded[10],rates[3]); near_value(decoded[12],rates[5]);
    near_value(decoded[13],5*180/3.14159265358979323846f);
    assert(test_uart_size==84);
    puts("PASS: Kalman initialization, steady gain, steps, dt, duplicates, live Q/R, invalid pairs, recovery, PID and telemetry");
}
static void test_kalman_extremes(void)
{
    const float params[]={0.000001f,1000.0f};
    unsigned q_index,r_index,step,axis;
    for(q_index=0;q_index<2;q_index++) for(r_index=0;r_index<2;r_index++) {
        RcSbus_Init(); test_flight_init(); test_armed=0;
        memset(&test_imu,0,sizeof(test_imu)); test_imu.gyro_valid=1;
        g_flight_control_filter.q=params[q_index];
        g_flight_control_filter.r=params[r_index];
        for(step=0;step<100;step++) {
            float dt=(step%2==0) ? 0.0002f : 0.005f;
            test_imu.gyro_y=(step%2==0) ? 10.0f : -10.0f;
            /* Exercise initialization and online updates at both dt limits. */
            test_imu.gyro_rx_count++;
            FlightControl_FilterGyro(&test_imu,dt);
            assert(g_flight_control_filter.applied_q==params[q_index]);
            assert(g_flight_control_filter.applied_r==params[r_index]);
            for(axis=0;axis<3;axis++) {
                assert(isfinite(s_filter_covariance[axis]) && s_filter_covariance[axis]>=0);
                assert(isfinite(s_filtered_rate[axis]));
                assert(g_flight_control_filter.gain[axis]>=0 && g_flight_control_filter.gain[axis]<=1);
            }
            assert(fabsf(s_filtered_rate[0])<=10.0001f);
            near_value(s_filtered_rate[1],0); near_value(s_filtered_rate[2],0);
        }
    }
    FlightControl_Init();
    near_value(g_flight_control_filter.q,FLIGHT_FILTER_DEFAULT_Q);
    near_value(g_flight_control_filter.r,1);
    puts("PASS: Kalman extreme Q/R and dt, finite nonnegative covariance, bounded gain, independent axes, boot defaults");
}
static void test_rate_axis_mapping(void)
{
    unsigned axis, i;
    const float rad_to_deg=180.0f/3.14159265358979323846f;
    float correction, throttle, rates[6], decoded[20];
    FlightControlState_t state;
    for(axis=0;axis<2;axis++) {
        RcSbus_Init(); test_flight_init();
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
        near_value(state.roll_rate_rad_s,-0.2f);  /* -gyro_y */
        near_value(state.roll_rate_raw_rad_s,-0.2f);
        near_value(state.pitch_rate_rad_s,0.3f); /* +gyro_x */
        near_value(state.yaw_rate_rad_s,0.4f);
        /* Body attitude uses -native pitch, +native roll, -native yaw. */
        near_value(state.roll_rad,0.2f); near_value(state.pitch_rad,0.1f);
        near_value(state.yaw_rad,-0.3f);
        near_value(state.roll_rate_target_rad_s,-0.1f);
        near_value(state.pitch_rate_target_rad_s,-0.05f);
        near_value(state.yaw_rate_target_rad_s,0);
        correction=(axis==0) ? 0.1f*(-0.1f+0.2f) : 0.1f*(-0.05f-0.3f);
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
        assert(VOFA_SendFlightTelemetry(state.roll_rad*rad_to_deg,
            state.pitch_rad*rad_to_deg,state.yaw_rad*rad_to_deg,throttle,
            g_rc_sbus_data.roll_target_rad*rad_to_deg,
            g_rc_sbus_data.pitch_target_rad*rad_to_deg,
            g_rc_sbus_data.yaw_rate_target_rad_s*rad_to_deg,rates,
            state.roll_rate_raw_rad_s*rad_to_deg,
        g_flight_control_state.pitch_rate_raw_rad_s*180/3.14159265358979323846f,
        g_flight_control_state.yaw_rate_raw_rad_s*180/3.14159265358979323846f,
        test_motors)==HAL_OK);
        memcpy(decoded,test_uart_bytes,sizeof(decoded));
        near_value(decoded[0],0.2f*rad_to_deg);
        near_value(decoded[1],0.1f*rad_to_deg);
        near_value(decoded[2],-0.3f*rad_to_deg);
        near_value(decoded[3],throttle);
        for(i=4;i<7;i++) near_value(decoded[i],0);
        for(i=0;i<6;i++) near_value(decoded[7+i],rates[i]);
        near_value(decoded[13],-0.2f*rad_to_deg);
        assert(test_uart_size==84 && test_uart_timeout==10);
        near_value(test_imu.gyro_x,0.3f); near_value(test_imu.gyro_y,0.2f);
        near_value(test_imu.roll,0.1f); near_value(test_imu.pitch,-0.2f);
        near_value(test_imu.yaw,0.3f);
    }
    puts("PASS: body attitude and rate feedback, independent PID motor corrections, shared VOFA coordinates, native IMU unchanged");
}
static void test_pitch_rate_damping(void)
{
    const float rad_to_deg=180.0f/3.14159265358979323846f;
    const float alpha=0.001f/(1.0f/(6.283185307179586f*50.0f)+0.001f);
    unsigned direction,step;
    float sign,expected,throttle,correction,rates[6],decoded[20];
    FlightControlState_t state;
    /* Latest bench report: +gyro_x is nose-down rotation. Isolate the rate
     * loop (angle gains and rate I/D zero); M1/M2 rear, M3/M4 front. */
    for(direction=0;direction<2;direction++) {
        RcSbus_Init(); test_flight_init();
        memset(&test_imu,0,sizeof(test_imu));
        test_imu.gyro_valid=1; test_imu.euler_valid=1;
        test_imu.roll=0.12f; /* Pitch attitude mapping must not be inverted. */
        frame(352,1024,0); test_armed=1;
        g_flight_control_pid.rate_pitch.kp=0.2f;
        sign=(direction==0) ? 1.0f : -1.0f;
        expected=sign*0.3f;
        for(step=0;step<2;step++) {
            test_imu.gyro_x=sign*((step==0) ? 0.3f : 0.6f);
            if(step!=0) expected+=alpha*(test_imu.gyro_x-expected);
            run(); FlightControl_GetState(&state);
            near_value(state.pitch_rad,0.12f);
            near_value(state.pitch_rate_rad_s,expected);
            near_value(state.pitch_rate_target_rad_s,0);
            near_value(state.roll_rate_rad_s,0);
            near_value(state.yaw_rate_rad_s,0);
            throttle=g_rc_sbus_data.throttle;
            correction=-0.2f*expected;
            near_value(test_motors[0],throttle+correction);
            near_value(test_motors[1],throttle+correction);
            near_value(test_motors[2],throttle-correction);
            near_value(test_motors[3],throttle-correction);
            if(direction==0) assert(test_motors[2]>test_motors[0]);
            else assert(test_motors[0]>test_motors[2]);
            memset(rates,0,sizeof(rates));
            rates[3]=state.pitch_rate_rad_s*rad_to_deg;
            assert(VOFA_SendFlightTelemetry(state.roll_rad*rad_to_deg,
                state.pitch_rad*rad_to_deg,state.yaw_rad*rad_to_deg,
                throttle,0,0,0,rates,state.roll_rate_raw_rad_s*rad_to_deg,
        g_flight_control_state.pitch_rate_raw_rad_s*180/3.14159265358979323846f,
        g_flight_control_state.yaw_rate_raw_rad_s*180/3.14159265358979323846f,
        test_motors)==HAL_OK);
            memcpy(decoded,test_uart_bytes,sizeof(decoded));
            near_value(decoded[1],0.12f*rad_to_deg);
            near_value(decoded[9],0);
            near_value(decoded[10],expected*rad_to_deg);
            assert(test_uart_size==84 && test_uart_timeout==10);
            assert(memcmp(test_uart_bytes+80,"\x00\x00\x80\x7f",4)==0);
        }
    }
    puts("PASS: Pitch +gyro_x damping, nose-down raises front M3/M4, nose-up raises rear M1/M2, filtered CH10 and attitude unchanged");
}
static void test_roll_attitude_restoring_motors(void)
{
    unsigned side;
    float throttle, correction;
    FlightControlState_t state;
    /* Latest bench report supersedes the previous Roll angle sign:
     * native pitch positive = left side down; M1/M4 are the left motors.
     * Hold the tilt still (zero gyro), with zero RC Roll target, positive
     * gains and no I/D, so only the angle-feedback direction is exercised. */
    for(side=0;side<2;side++) {
        RcSbus_Init(); test_flight_init();
        memset(&test_imu,0,sizeof(test_imu));
        test_imu.gyro_valid=1; test_imu.euler_valid=1;
        test_imu.pitch=(side==0) ? 0.2f : -0.2f;
        frame(1696,1024,0); test_armed=1;
        g_flight_control_pid.angle_roll.kp=0.5f;
        g_flight_control_pid.rate_roll.kp=0.2f;
        run(); FlightControl_GetState(&state);
        throttle=g_rc_sbus_data.throttle;
        correction=(side==0) ? 0.02f : -0.02f;
        near_value(state.roll_rad,(side==0) ? -0.2f : 0.2f);
        near_value(state.roll_rate_target_rad_s,(side==0) ? 0.1f : -0.1f);
        near_value(state.roll_rate_rad_s,0);
        near_value(state.pitch_rate_target_rad_s,0);
        near_value(state.yaw_rate_target_rad_s,0);
        near_value(test_motors[0],throttle+correction);
        near_value(test_motors[3],throttle+correction);
        near_value(test_motors[1],throttle-correction);
        near_value(test_motors[2],throttle-correction);
        if(side==0) assert(test_motors[0]>test_motors[1]);
        else assert(test_motors[1]>test_motors[0]);
    }
    puts("PASS: static left tilt raises M1/M4, static right tilt raises M2/M3, zero RC target and positive gains");
}
static void test_attitude_axis_mapping(void)
{
    const float rad_to_deg=180.0f/3.14159265358979323846f;
    unsigned gain_case,axis,sign,i;
    float body_angle, expected[3], target[3], rates[6], decoded[20];
    FlightControlState_t state;

    /* Isolate each physical attitude axis, both signs, below/above rate clamp.
     * Locked motors must not suppress the observable outer-loop target. */
    for(gain_case=0;gain_case<2;gain_case++) for(axis=0;axis<3;axis++)
    for(sign=0;sign<2;sign++) {
        RcSbus_Init(); test_flight_init(); test_armed=0;
        memset(test_motors,0,sizeof(test_motors));
        memset(&test_imu,0,sizeof(test_imu));
        test_imu.gyro_valid=1; test_imu.euler_valid=1;
        body_angle=(sign==0) ? 0.2f : -0.2f;
        memset(expected,0,sizeof(expected)); expected[axis]=body_angle;
        if(axis==0) test_imu.pitch=-body_angle;
        if(axis==1) test_imu.roll=body_angle;
        if(axis==2) test_imu.yaw=-body_angle;
        g_flight_control_pid.angle_roll.kp=(gain_case==0) ? 0.5f : 100.0f;
        g_flight_control_pid.angle_pitch.kp=g_flight_control_pid.angle_roll.kp;
        g_flight_control_pid.rate_roll.kp=0.2f;
        g_flight_control_pid.rate_pitch.kp=0.2f;
        frame(1696,1024,0);
        run(); FlightControl_GetState(&state);

        near_value(state.roll_rad,expected[0]);
        near_value(state.pitch_rad,expected[1]);
        near_value(state.yaw_rad,expected[2]);
        memset(target,0,sizeof(target));
        if(axis<2) target[axis]=(sign==0 ? -1.0f : 1.0f)*
                               (gain_case==0 ? 0.1f : 4.0f);
        near_value(state.roll_rate_target_rad_s,target[0]);
        near_value(state.pitch_rate_target_rad_s,target[1]);
        near_value(state.yaw_rate_target_rad_s,0); /* No Yaw angle loop. */
        near_value(state.roll_rate_rad_s,0);
        near_value(state.pitch_rate_rad_s,0);
        near_value(state.yaw_rate_rad_s,0);
        near_value(state.roll_rate_raw_rad_s,0);
        assert(state.running && !state.armed && !test_armed && !s_have_error);
        for(i=0;i<4;i++) {
            near_value(state.motor_command[i],0);
            near_value(test_motors[i],0);
        }
        near_value(s_roll_integral,0); near_value(s_pitch_integral,0);
        near_value(s_yaw_integral,0);

        rates[0]=state.roll_rate_target_rad_s*rad_to_deg;
        rates[1]=state.roll_rate_rad_s*rad_to_deg;
        rates[2]=state.pitch_rate_target_rad_s*rad_to_deg;
        rates[3]=state.pitch_rate_rad_s*rad_to_deg;
        rates[4]=state.yaw_rate_target_rad_s*rad_to_deg;
        rates[5]=state.yaw_rate_rad_s*rad_to_deg;
        assert(VOFA_SendFlightTelemetry(state.roll_rad*rad_to_deg,
            state.pitch_rad*rad_to_deg,state.yaw_rad*rad_to_deg,
            g_rc_sbus_data.throttle,0,0,0,rates,
            state.roll_rate_raw_rad_s*rad_to_deg,
        g_flight_control_state.pitch_rate_raw_rad_s*180/3.14159265358979323846f,
        g_flight_control_state.yaw_rate_raw_rad_s*180/3.14159265358979323846f,
        test_motors)==HAL_OK);
        memcpy(decoded,test_uart_bytes,sizeof(decoded));
        for(i=0;i<3;i++) near_value(decoded[i],expected[i]*rad_to_deg);
        near_value(decoded[3],g_rc_sbus_data.throttle);
        for(i=4;i<7;i++) near_value(decoded[i],0);
        for(i=0;i<3;i++) {
            near_value(decoded[7+2*i],target[i]*rad_to_deg);
            near_value(decoded[8+2*i],0);
        }
        near_value(decoded[13],0);
        assert(test_uart_size==84 && test_uart_timeout==10);
        assert(memcmp(test_uart_bytes+80,"\x00\x00\x80\x7f",4)==0);
    }
    puts("PASS: positive/negative independent attitude axes, outer-loop restoring signs and limits, locked outputs, CH0..2 snapshot units");
}
static void test_raw_roll_telemetry(void)
{
    const float rad_to_deg=180.0f/3.14159265358979323846f;
    const float alpha=0.001f/(1.0f/(6.283185307179586f*50.0f)+0.001f);
    float expected, rates[6], decoded[20];
    FlightControlState_t state;

    RcSbus_Init(); test_flight_init(); test_armed=0;
    memset(&test_imu,0,sizeof(test_imu));
    FlightControl_GetState(&state);
    near_value(state.roll_rate_raw_rad_s,0);
    test_imu.gyro_y=-99; run(); /* Invalid first sample is not published. */
    near_value(g_flight_control_state.roll_rate_raw_rad_s,0);
    test_imu.gyro_valid=1; test_imu.gyro_y=-1; run();
    near_value(g_flight_control_state.roll_rate_raw_rad_s,1);
    near_value(g_flight_control_state.roll_rate_rad_s,1);

    test_imu.gyro_y=-3; run();
    expected=1+2*alpha;
    near_value(g_flight_control_state.roll_rate_raw_rad_s,3);
    near_value(g_flight_control_state.roll_rate_rad_s,expected);
    test_imu.gyro_y=2; run();
    expected+=alpha*(-2-expected);
    FlightControl_GetState(&state);
    near_value(state.roll_rate_raw_rad_s,-2);
    near_value(state.roll_rate_rad_s,expected);

    /* A changed buffer without a fresh count must not replace either value. */
    test_imu.gyro_y=-99;
    test_dwt.CYCCNT+=480000U; FlightControl_Run();
    FlightControl_GetState(&state);
    near_value(state.roll_rate_raw_rad_s,-2);
    near_value(state.roll_rate_rad_s,expected);
    rates[0]=state.roll_rate_target_rad_s*rad_to_deg;
    rates[1]=state.roll_rate_rad_s*rad_to_deg;
    rates[2]=state.pitch_rate_target_rad_s*rad_to_deg;
    rates[3]=state.pitch_rate_rad_s*rad_to_deg;
    rates[4]=state.yaw_rate_target_rad_s*rad_to_deg;
    rates[5]=state.yaw_rate_rad_s*rad_to_deg;
    assert(VOFA_SendFlightTelemetry(0,0,0,0,0,0,0,rates,
        state.roll_rate_raw_rad_s*rad_to_deg,
        g_flight_control_state.pitch_rate_raw_rad_s*180/3.14159265358979323846f,
        g_flight_control_state.yaw_rate_raw_rad_s*180/3.14159265358979323846f,
        test_motors)==HAL_OK);
    assert(test_uart_size==84 && test_uart_timeout==10);
    memcpy(decoded,test_uart_bytes,sizeof(decoded));
    near_value(decoded[8],expected*rad_to_deg);
    near_value(decoded[13],-2*rad_to_deg);
    assert(fabsf(decoded[8]-decoded[13])>1);
    assert(memcmp(test_uart_bytes+80,"\x00\x00\x80\x7f",4)==0);

    test_imu.gyro_valid=0; run();
    near_value(g_flight_control_state.roll_rate_raw_rad_s,-2);
    near_value(g_flight_control_state.roll_rate_rad_s,expected);
    test_imu.gyro_valid=1; test_imu.gyro_y=-4; run();
    near_value(g_flight_control_state.roll_rate_raw_rad_s,4);
    near_value(g_flight_control_state.roll_rate_rad_s,4);
    test_tick+=11; FlightControl_OnTaskTimeout();
    test_imu.gyro_y=-9; FlightControl_Run(); /* Old count after timeout. */
    assert(!s_filter_initialized);
    near_value(g_flight_control_state.roll_rate_raw_rad_s,4);
    near_value(g_flight_control_state.roll_rate_rad_s,4);
    run(); /* Fresh recovery directly seeds both values. */
    near_value(g_flight_control_state.roll_rate_raw_rad_s,9);
    near_value(g_flight_control_state.roll_rate_rad_s,9);
    test_flight_init();
    near_value(g_flight_control_state.roll_rate_raw_rad_s,0);
    puts("PASS: raw Roll sign, steps, fresh-sample snapshot, invalid/timeout hold and recovery, CH13 units and frame");
}
static void test_raw_pitch_yaw_telemetry(void)
{
    const float degrees=180.0f/3.14159265358979323846f;
    float rates[6]={0}, decoded[20];
    FlightControlState_t state;
    unsigned axis, direction;
    for (axis=0;axis<2;axis++) for (direction=0;direction<2;direction++) {
        const float sign=direction ? -1.0f : 1.0f;
        RcSbus_Init(); test_flight_init(); test_armed=0;
        memset(&test_imu,0,sizeof(test_imu));
        FlightControl_GetState(&state);
        near_value(state.pitch_rate_raw_rad_s,0); near_value(state.yaw_rate_raw_rad_s,0);
        test_imu.gyro_x=axis==0 ? sign : 0;
        test_imu.gyro_z=axis==1 ? -sign : 0;
        run(); /* No valid gyro yet. */
        near_value(g_flight_control_state.pitch_rate_raw_rad_s,0);
        near_value(g_flight_control_state.yaw_rate_raw_rad_s,0);
        test_imu.gyro_valid=1; run();
        test_imu.gyro_x*=3; test_imu.gyro_z*=3; run();
        FlightControl_GetState(&state);
        near_value(state.pitch_rate_raw_rad_s,axis==0 ? sign*3 : 0);
        near_value(state.yaw_rate_raw_rad_s,axis==1 ? sign*3 : 0);
        assert(fabsf((axis==0 ? state.pitch_rate_rad_s : state.yaw_rate_rad_s)-sign*3)>0.1f);
        assert(VOFA_SendFlightTelemetry(0,0,0,0,0,0,0,rates,0,
            state.pitch_rate_raw_rad_s*degrees,state.yaw_rate_raw_rad_s*degrees,test_motors)==HAL_OK);
        memcpy(decoded,test_uart_bytes,sizeof(decoded));
        near_value(decoded[14],state.pitch_rate_raw_rad_s*degrees);
        near_value(decoded[15],state.yaw_rate_raw_rad_s*degrees);
        test_imu.gyro_x=99; test_imu.gyro_z=99;
        FlightControl_Run(); /* Duplicate sample cannot replace raw values. */
        near_value(g_flight_control_state.pitch_rate_raw_rad_s,state.pitch_rate_raw_rad_s);
        near_value(g_flight_control_state.yaw_rate_raw_rad_s,state.yaw_rate_raw_rad_s);
        test_imu.gyro_valid=0; run();
        near_value(g_flight_control_state.pitch_rate_raw_rad_s,state.pitch_rate_raw_rad_s);
        near_value(g_flight_control_state.yaw_rate_raw_rad_s,state.yaw_rate_raw_rad_s);
        test_imu.gyro_valid=1; test_imu.gyro_x=-2; test_imu.gyro_z=4; run();
        near_value(g_flight_control_state.pitch_rate_raw_rad_s,-2);
        near_value(g_flight_control_state.yaw_rate_raw_rad_s,-4);
        near_value(g_flight_control_state.pitch_rate_rad_s,-2);
        near_value(g_flight_control_state.yaw_rate_rad_s,-4);
        test_tick+=11; FlightControl_OnTaskTimeout();
        test_imu.gyro_x=5; test_imu.gyro_z=-6; FlightControl_Run();
        near_value(g_flight_control_state.pitch_rate_raw_rad_s,-2);
        near_value(g_flight_control_state.yaw_rate_raw_rad_s,-4);
        run();
        near_value(g_flight_control_state.pitch_rate_raw_rad_s,5);
        near_value(g_flight_control_state.yaw_rate_raw_rad_s,6);
        near_value(g_flight_control_state.pitch_rate_rad_s,5);
        near_value(g_flight_control_state.yaw_rate_rad_s,6);
    }
    puts("PASS: Pitch/Yaw raw signs, independent axes, steps, units, duplicate/invalid hold and timeout recovery");
}
int main(void)
{
    unsigned i;
    float rates[6]={11,12,13,14,15,16}, decoded[20];
    RcSbus_Init(); test_flight_init(); test_imu.gyro_valid=1;
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
    assert(VOFA_SendFlightTelemetry(1,2,3,4,5,6,7,rates,-17,
        g_flight_control_state.pitch_rate_raw_rad_s*180/3.14159265358979323846f,
        g_flight_control_state.yaw_rate_raw_rad_s*180/3.14159265358979323846f,
        test_motors)==HAL_OK);
    assert(test_uart_size==84 && test_uart_timeout==10);
    memcpy(decoded,test_uart_bytes,sizeof(decoded));
    for(i=0;i<7;i++) near_value(decoded[i],(float)i+1);
    for(i=0;i<6;i++) near_value(decoded[i+7],rates[i]);
    near_value(decoded[13],-17);
    assert(memcmp(test_uart_bytes+80,"\x00\x00\x80\x7f",4)==0);
    assert(VOFA_SendChannels(rates,24)==HAL_ERROR);
    assert(VOFA_SendFlightTelemetry(0,0,0,0,0,0,0,NULL,0,
        g_flight_control_state.pitch_rate_raw_rad_s*180/3.14159265358979323846f,
        g_flight_control_state.yaw_rate_raw_rad_s*180/3.14159265358979323846f,
        test_motors)==HAL_ERROR);
    assert(VOFA_SendAttitude(1,2,3)==HAL_OK);
    assert(test_uart_size==16 && test_uart_timeout==10);
    memcpy(decoded,test_uart_bytes,3U*sizeof(float));
    for(i=0;i<3;i++) near_value(decoded[i],(float)i+1);
    assert(memcmp(test_uart_bytes+12,"\x00\x00\x80\x7f",4)==0);
    puts("PASS: mapping, mode gate, fresh-frame recenter, axis isolation, failsafe, timeouts, telemetry");
    test_filter();
    test_kalman_extremes();
    test_rate_axis_mapping();
    test_pitch_rate_damping();
    test_roll_attitude_restoring_motors();
    test_attitude_axis_mapping();
    test_raw_roll_telemetry();
    test_raw_pitch_yaw_telemetry();
    return 0;
}
