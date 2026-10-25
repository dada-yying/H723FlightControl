/* A1 only: real ESC + FlightControl, modeled inputs/timers/RTOS scheduling.
 * prepare_esc_arming.py inserts one observation into the real ESC source.
 * The timeout task NEVER runs while critical_depth is nonzero. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../App/esc_pwm.h"
#include "../../App/bmi088_attitude.h"
#include "../../App/flight_control.h"
#include "tim.h"

static RcSbusData_t input_rc;
static Bmi088Data_t input_imu;
static Bmi088GyroFilterState_t input_filter;
static Bmi088AttitudeState_t input_attitude;
static unsigned timeout_pending,timeout_executed,timeout_deferred,inject_commit;
static unsigned total_errors;
static uint32_t timeout_tick;
static FILE *trace;
static unsigned trace_index;

static void Test_TimeoutTask(void)
{
    assert(test_critical_depth==0); /* Production task cannot run under BASEPRI. */
    timeout_executed++;
    test_tick=timeout_tick;
    FlightControl_OnTaskTimeout();
}
static void Test_EnterCritical(void) { test_critical_depth++; }
static void Test_ExitCritical(void)
{
    assert(test_critical_depth>0);
    if(--test_critical_depth==0 && timeout_pending) {
        timeout_pending=0;
        Test_TimeoutTask(); /* Pending SysTick/PendSV is serviced at unmask. */
    }
}
#undef taskENTER_CRITICAL
#undef taskEXIT_CRITICAL
#define taskENTER_CRITICAL() Test_EnterCritical()
#define taskEXIT_CRITICAL() Test_ExitCritical()

static void Test_RequestTimeout(void)
{
    if(test_critical_depth) {timeout_pending=1;timeout_deferred++;}
    else Test_TimeoutTask();
}
static void Test_ArmingCommitPoint(void)
{
    if(inject_commit) {inject_commit=0;Test_RequestTimeout();}
}

void RcSbus_GetData(RcSbusData_t *rc) { if(rc)*rc=input_rc; }
void Bmi088_GetSnapshotWithFilter(Bmi088Data_t *imu,
    struct Bmi088GyroFilterState *filter,struct Bmi088AttitudeState *attitude)
{
    input_imu.age_ms=(uint32_t)(test_tick-input_imu.sample_tick_ms);
    if(imu)*imu=input_imu;if(filter)*filter=input_filter;if(attitude)*attitude=input_attitude;
}
#include "../../App/flight_control.c"
#include "esc_arming_actual.inc"

static void check(int condition,const char *name)
{
    if(!condition){total_errors++;printf("FAIL %s\n",name);}
}
static int minimum(void)
{
    unsigned i;
    if(EscPwm_IsArmed())return 0;
    for(i=0;i<4;i++)if(g_esc_pwm_state.command[i]!=0 || g_esc_pwm_state.pulse_us[i]!=1000)return 0;
    return htim1.compare[TIM_CHANNEL_3]==1000 && htim1.compare[TIM_CHANNEL_1]==1000 &&
           htim2.compare[TIM_CHANNEL_3]==1000 && htim2.compare[TIM_CHANNEL_1]==1000;
}
static uint32_t bits(float v) {uint32_t u;memcpy(&u,&v,sizeof(u));return u;}
static void record(void)
{
    EscPwmState_t esc;FlightControlState_t flight;unsigned i;
    EscPwm_GetState(&esc);FlightControl_GetState(&flight);
    if(!trace)return;
    fprintf(trace,"%u,%u,%u,%u,%u,%u,%u,%u",trace_index++,test_tick,esc.armed,
        esc.arm_middle_seen,esc.inhibit_reason,esc.update_count,flight.running,flight.imu_timeout_count);
    for(i=0;i<4;i++)fprintf(trace,",%08x,%u",bits(esc.command[i]),esc.pulse_us[i]);
    fprintf(trace,",%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x\n",
        bits(flight.dt_s),bits(flight.roll_rate_target_rad_s),bits(flight.pitch_rate_target_rad_s),
        bits(flight.yaw_rate_target_rad_s),bits(flight.motor_command[0]),bits(flight.motor_command[1]),
        bits(flight.motor_command[2]),bits(flight.motor_command[3]));
}
static void setup(uint32_t start)
{
    assert(test_critical_depth==0);
    timeout_pending=timeout_executed=timeout_deferred=inject_commit=0;
    memset(&input_rc,0,sizeof(input_rc));memset(&input_imu,0,sizeof(input_imu));
    memset(&input_filter,0,sizeof(input_filter));memset(&input_attitude,0,sizeof(input_attitude));
    test_tick=start;test_dwt.CYCCNT=0;test_pwm_start_status=HAL_OK;
    memset(&htim1,0,sizeof(htim1));memset(&htim2,0,sizeof(htim2));
    FlightControl_Init();assert(EscPwm_Init()==HAL_OK);
    input_rc.valid=1;input_rc.channel[6]=1000;input_rc.last_valid_tick=start;
}
static void rc(uint16_t channel,float throttle)
{
    input_rc.channel[6]=channel;input_rc.throttle=throttle;
    input_rc.last_valid_tick=test_tick;input_rc.valid_frame_count++;
    EscPwm_UpdateFromRc(&input_rc);assert(test_critical_depth==0);
}
static void sample(unsigned us)
{
    test_tick+=us/1000U;test_dwt.CYCCNT+=us*480U;
    input_imu.valid=input_imu.initialized=input_imu.has_sample=1;
    input_filter.valid=input_filter.initialized=input_filter.has_sample=1;
    input_attitude.valid=input_attitude.initialized=1;input_attitude.epoch=1;
    input_imu.sample_count++;input_imu.sample_tick_ms=test_tick;
    input_imu.completion_cycle=test_dwt.CYCCNT;
    input_filter.sample_count=input_attitude.sample_count=input_imu.sample_count;
    input_filter.sample_tick_ms=input_attitude.sample_tick_ms=test_tick;
}

static void normal_inputs(void)
{
    unsigned i;float commands[4]={-.2f,.25f,.75f,1.2f};
    setup(0);record();rc(1000,0);record();test_tick=1999;rc(1696,0);record();
    check(minimum(),"startup delay before 2000ms");test_tick=2000;rc(1696,0);record();
    check(EscPwm_IsArmed(),"startup delay reaches 2000ms");
    EscPwm_SetMixedNormalized(commands);record();
    check(g_esc_pwm_state.pulse_us[0]==1000 && g_esc_pwm_state.pulse_us[1]==1250 &&
          g_esc_pwm_state.pulse_us[2]==1750 && g_esc_pwm_state.pulse_us[3]==2000,"normal mixer pulse map");
    input_rc.frame_lost=1;rc(352,0);record();check(EscPwm_IsArmed(),"lost frame holds armed");
    input_rc.failsafe=1;EscPwm_UpdateFromRc(&input_rc);record();check(minimum(),"failsafe precedence");
    input_rc.failsafe=input_rc.frame_lost=0;
    rc(1696,0);record();check(minimum(),"failsafe consumes middle qualification");
    rc(1000,.051f);rc(1696,0);record();check(minimum(),"high throttle cannot prearm");
    rc(1000,.05f);rc(1696,.05f);record();check(EscPwm_IsArmed(),"low throttle boundary");
    test_tick+=99;EscPwm_UpdateFromRc(&input_rc);record();check(EscPwm_IsArmed(),"silence age99 holds");
    test_tick++;EscPwm_UpdateFromRc(&input_rc);record();check(minimum(),"silence age100 locks");
    rc(352,0);record();rc(1696,0);record();check(minimum(),"low position is not prearm");
    EscPwm_UpdateFromRc(NULL);record();check(minimum(),"null RC locks");
    rc(1000,0);rc(1696,0);EscPwm_Disarm();record();check(minimum(),"explicit disarm");
    setup(UINT32_MAX-2050U);test_tick+=2000;rc(1000,0);rc(1696,0);record();
    test_tick+=99;EscPwm_UpdateFromRc(&input_rc);record();check(EscPwm_IsArmed(),"wrap age99");
    test_tick++;EscPwm_UpdateFromRc(&input_rc);record();check(minimum(),"wrap age100");
    setup(100);test_tick+=2000;sample(1000);input_rc.last_valid_tick=test_tick;
    FlightControl_Run();rc(1000,0);rc(1696,0);
    for(i=0;i<2000;i++) {
        sample(1000);rc(1696,.4f);
        input_rc.roll_target_rad=((int)(i%41)-20)*.005f;
        input_rc.pitch_target_rad=((int)(i%31)-15)*.004f;
        input_rc.yaw_rate_target_rad_s=((int)(i%21)-10)*.01f;
        input_filter.gyro_rad_s[0]=((int)(i%13)-6)*.002f;
        input_filter.gyro_rad_s[1]=((int)(i%11)-5)*.002f;
        input_filter.gyro_rad_s[2]=((int)(i%7)-3)*.002f;
        FlightControl_Run();record();
    }
    check(EscPwm_IsArmed() && g_flight_control_state.running,"normal finite controller stream");
}

static void fault_position(unsigned position)
{
    uint32_t last_control;unsigned i;
    setup(100);test_tick+=2000;sample(1000);input_rc.last_valid_tick=test_tick;
    FlightControl_Run();last_control=test_tick;
    test_tick+=5;FlightControl_OnTaskTimeout();
    test_tick++;rc(1000,0);assert(g_esc_pwm_state.arm_middle_seen);
    test_tick+=3;input_rc.channel[6]=1696;input_rc.last_valid_tick=test_tick;input_rc.valid_frame_count++;
    timeout_tick=last_control+10;
    if(position==0)Test_RequestTimeout();
    if(position==1)inject_commit=1;
    EscPwm_UpdateFromRc(&input_rc);
    if(position==2)Test_RequestTimeout();
    check(timeout_executed==1 && !timeout_pending,"one runnable timeout at depth zero");
    check(minimum() && !g_esc_pwm_state.arm_middle_seen,"fault completion leaves four minimum and consumes middle");
    printf("position=%u executed=%u deferred=%u armed=%u middle=%u reason=%u\n",position,timeout_executed,
        timeout_deferred,EscPwm_IsArmed(),g_esc_pwm_state.arm_middle_seen,g_esc_pwm_state.inhibit_reason);
    for(i=0;i<4;i++)printf("  M%u=%g/%uus\n",i+1,g_esc_pwm_state.command[i],g_esc_pwm_state.pulse_us[i]);
    /* Good sensor recovery, no new middle-to-high sequence; RC remains high. */
    for(i=0;i<3;i++) {
        sample(1000);input_rc.throttle=.5f;FlightControl_Run();
        check(minimum(),"source recovery cannot restore old arm");
    }
    rc(1696,0);check(minimum(),"repeated high cannot rearm");
    rc(1000,.1f);rc(1696,0);check(minimum(),"post-fault high throttle cannot prearm");
    rc(1000,0);check(minimum() && g_esc_pwm_state.arm_middle_seen,"new low-throttle middle stays locked");
    rc(1696,0);check(EscPwm_IsArmed(),"new middle-high rearm");
    sample(1000);input_rc.throttle=.5f;FlightControl_Run();
    check(g_esc_pwm_state.command[0]>.49f,"output resumes only after rearm");
}
static void running_timeout(void)
{
    setup(100);test_tick+=2000;sample(1000);input_rc.last_valid_tick=test_tick;
    FlightControl_Run();rc(1000,0);rc(1696,0);
    input_rc.throttle=.5f;sample(1000);FlightControl_Run();
    assert(g_esc_pwm_state.command[0]>.49f);
    timeout_tick=test_tick+10;Test_RequestTimeout();
    check(minimum() && !g_esc_pwm_state.arm_middle_seen,"running timeout clears all four non-minimum commands");
}
int main(int argc,char **argv)
{
    if(argc>1){trace=fopen(argv[1],"w");assert(trace);}
    normal_inputs();if(trace){fclose(trace);trace=NULL;}
    fault_position(0);fault_position(1);fault_position(2);running_timeout();
    assert(test_critical_depth==0);
    printf("A1 result: errors=%u normal_trace_rows=%u; timeout task always ran at critical depth zero\n",total_errors,trace_index);
    return total_errors?1:0;
}
