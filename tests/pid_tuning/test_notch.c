/* Regression for the retired ATOM cascade: no second filter may affect BMI. */
#define CONTROL_FIXTURE_ONLY
#include "test_tuning.c"
int main(void)
{
    unsigned i;
    reset_control();
    for(i=0;i<100;i++) {
        float value=(i&1U) ? -.75f : .75f;
        source(1000);
        g_bmi088_gyro_filter.gyro_rad_s[0]=value;
        g_bmi088_gyro_filter.gyro_rad_s[1]=-value;
        g_bmi088_gyro_filter.gyro_rad_s[2]=value*.5f;
        g_flight_control_filter.q=NAN; g_flight_control_filter.r=INFINITY;
        FlightControl_Run();
        near_value(g_flight_control_state.roll_rate_rad_s,value);
        near_value(g_flight_control_state.pitch_rate_rad_s,-value);
        near_value(g_flight_control_state.yaw_rate_rad_s,value*.5f);
        FlightControl_ResetController();
        assert(!g_flight_control_notch.active && !g_flight_control_notch.sample_count);
    }
    puts("PASS BMI LPF output used exactly; legacy notch/Kalman Watch has no effect");
    return 0;
}
