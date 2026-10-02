/* Exercise actual FlightControl_Run/FilterGyro with the existing HAL fixtures. */
#define main tuning_regression_main
#include "test_tuning.c"
#undef main

#define TEST_PI 3.14159265358979323846

typedef struct { double z1,z2; } RefNotch;
static double ref_notch(RefNotch *s,double x,double fs,int seed)
{
    /* Transposed direct form II, independent of production's direct form I. */
    double g=1/(1+tan(TEST_PI*60/fs));
    double b1=-2*g*cos(2*TEST_PI*100/fs),a2=2*g-1,y;
    if(seed) { s->z1=(1-g)*x; s->z2=(g-a2)*x; }
    y=g*x+s->z1;
    s->z1=b1*x-b1*y+s->z2;
    s->z2=g*x-a2*y;
    return y;
}

static void init_notch_test(void)
{
    RcSbus_Init(); FlightControl_Init(); test_armed=0;
    memset(&test_imu,0,sizeof(test_imu)); test_imu.gyro_valid=1;
}

static void test_reference_cascade(void)
{
    RefNotch ref[2]={{0}};
    double x[3]={0},p[3]={0},measurement[3],raw[3];
    unsigned n,axis;
    init_notch_test();
    near_value(s_notch_b0,0.8397996491122632f);
    near_value(s_notch_b1,-1.3588243760038774f);
    near_value(s_notch_a2,0.6795992982245265f);
    for(n=0;n<8000;n++) {
        /* DC, positive/negative steps, impulse and independent sinusoids. */
        raw[0]=2.5+(n>=2000 ? -4.0 : 0)+sin(2*TEST_PI*100*n/1000);
        raw[1]=-1.3+0.3*sin(2*TEST_PI*5*n/1000)+(n==120 ? 3 : 0);
        raw[2]=sin(2*TEST_PI*100*n/1000);
        test_imu.gyro_y=-(float)raw[0]; test_imu.gyro_x=(float)raw[1];
        test_imu.gyro_z=-(float)raw[2];
        for(axis=0;axis<3;axis++) {
            measurement[axis]=axis<2 ? ref_notch(&ref[axis],(float)raw[axis],1000,n==0) : (float)raw[axis];
            if(n==0) { x[axis]=measurement[axis]; p[axis]=(sqrt(.015*.015+4*.015)-.015)/2; }
            else reference_kalman(&x[axis],&p[axis],measurement[axis],.015,1,.001);
        }
        run();
        for(axis=0;axis<3;axis++) assert(fabs(s_filtered_rate[axis]-x[axis])<0.00003);
        near_value(g_flight_control_state.roll_rate_raw_rad_s,(float)raw[0]);
        near_value(g_flight_control_state.pitch_rate_raw_rad_s,(float)raw[1]);
        near_value(g_flight_control_state.yaw_rate_raw_rad_s,(float)raw[2]);
    }
    assert(g_flight_control_notch.coefficient_update_count==2); /* init + first seed */
    puts("PASS: 8000-sample independent double notch/Kalman reference; raw preservation; Yaw unchanged");
}

static void test_frequency_response(void)
{
    const float fs_list[]={500,800,1000,2000};
    const float freq_list[]={5,73.817474f,100,133.817474f,200};
    unsigned s,f,n;
    for(s=0;s<4;s++) for(f=0;f<5;f++) {
        double in_power=0,out_power=0;
        float fs=fs_list[s],freq=freq_list[f];
        init_notch_test();
        for(n=0;n<12000;n++) {
            float input=(float)sin(2*TEST_PI*freq*n/fs);
            test_imu.gyro_y=-input; test_imu.gyro_x=2.0f;
            test_imu.gyro_rx_count++;
            FlightControl_FilterGyro(&test_imu,1/fs);
            if(n>=2000) { in_power+=input*input; out_power+=s_notch_history[0].y1*s_notch_history[0].y1; }
            near_value(s_filtered_rate[1],2); near_value(s_filtered_rate[2],0);
        }
        assert(fabs(g_flight_control_notch.applied_sample_hz-fs)<0.02);
        if(freq==100) assert(sqrt(out_power/in_power)<0.0001);
        if(freq==5) assert(sqrt(out_power/in_power)>0.99);
        if(s==2 && (f==1 || f==3)) assert(fabs(sqrt(out_power/in_power)-sqrt(.5))<.001);
    }
    puts("PASS: notch center rejection >80 dB at 500/800/1000/2000 Hz; 5 Hz passband; exact 60 Hz -3 dB width; axis isolation");
}

static void test_notch_lifecycle(void)
{
    FlightNotchHistory_t held[2];
    uint32_t count,n;
    init_notch_test();
    test_imu.gyro_y=-1; run(); test_imu.gyro_y=-3; run();
    memcpy(held,s_notch_history,sizeof(held)); count=g_flight_control_notch.sample_count;
    FlightControl_ResetController();
    assert(memcmp(held,s_notch_history,sizeof(held))==0);
    test_imu.gyro_y=99; FlightControl_Run();
    assert(memcmp(held,s_notch_history,sizeof(held))==0);
    assert(g_flight_control_notch.sample_count==count);
    test_tick+=11; FlightControl_OnTaskTimeout();
    FlightControl_Run(); assert(!s_filter_initialized);
    test_imu.gyro_y=-4; test_imu.gyro_x=-2; run();
    near_value(s_filtered_rate[0],4); near_value(s_filtered_rate[1],-2);
    near_value(s_notch_history[0].x1,4); near_value(s_notch_history[0].y2,4);
    test_imu.gyro_valid=0; run();
    test_imu.gyro_valid=1; test_imu.gyro_x=7; run();
    near_value(s_filtered_rate[1],7);
    /* Averaged fresh-sample interval includes duplicate calls, not iterations. */
    init_notch_test(); run();
    for(n=0;n<128;n++) {
        FlightControl_FilterGyro(&test_imu,0.001f);
        test_imu.gyro_rx_count++;
        FlightControl_FilterGyro(&test_imu,0.001f);
    }
    assert(g_flight_control_notch.sample_count==129);
    assert(fabs(g_flight_control_notch.sample_hz-500)<0.02);
    /* Sustained undersampling uses Kalman alone and restores notch on recovery. */
    for(n=0;n<128;n++) { test_imu.gyro_rx_count++; FlightControl_FilterGyro(&test_imu,.005f); }
    assert(!g_flight_control_notch.active && g_flight_control_notch.rate_fault_count==1);
    for(n=0;n<128;n++) { test_imu.gyro_rx_count++; FlightControl_FilterGyro(&test_imu,.001f); }
    assert(g_flight_control_notch.active);
    /* Alternating scheduler jitter does not modulate coefficients. */
    init_notch_test(); run(); count=g_flight_control_notch.coefficient_update_count;
    for(n=0;n<1024;n++) {
        test_imu.gyro_rx_count++;
        FlightControl_FilterGyro(&test_imu,n%2 ? .0011f : .0009f);
    }
    assert(g_flight_control_notch.coefficient_update_count==count);
    test_imu.gyro_rx_count=0xffffffffU; FlightControl_FilterGyro(&test_imu,.001f);
    test_imu.gyro_rx_count=0U; FlightControl_FilterGyro(&test_imu,.001f);
    assert(s_filter_rx_count==0U);
    puts("PASS: duplicate samples, PID resets, timeout/invalid recovery, averaged cadence, low-rate fallback, jitter and count wrap");
}

int main(void)
{
    test_reference_cascade();
    test_frequency_response();
    test_notch_lifecycle();
    return 0;
}
