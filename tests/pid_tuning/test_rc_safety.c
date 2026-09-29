/* Real RC decoder, UART recovery, flight controller and ESC safety/PWM paths.
 * Only HAL/RTOS, timer registers and IMU acquisition are mocked. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../App/rc_sbus.c"
#include "../../App/esc_pwm.c"
#include "../../App/flight_control.c"

static AtomImuData_t test_imu;
void AtomImu_GetData(AtomImuData_t *data) { *data = test_imu; }
static void near_value(float a, float b) { assert(fabsf(a-b) < 0.0001f); }

static void setup(uint32_t tick)
{
    assert(test_critical_depth == 0);
    test_tick = tick;
    test_check_pwm_critical = 0;
    test_pwm_start_status = HAL_OK;
    test_uart_rx_status = test_uart_abort_status = HAL_OK;
    test_uart_rx_calls = test_uart_abort_calls = 0;
    RcSbus_Init();
    assert(EscPwm_Init() == HAL_OK);
    test_check_pwm_critical = 1;
    FlightControl_Init();
    memset(&test_imu,0,sizeof(test_imu));
    test_imu.gyro_valid = test_imu.euler_valid = 1;
    assert(RcSbus_StartReception() == HAL_OK);
    RcSbus_ProcessPending();
    assert(!s_restart_pending);
}

static void frame_at(uint32_t tick, uint16_t arm, uint16_t throttle,
                     uint16_t axis, uint16_t slider, uint8_t flags)
{
    uint8_t bytes[25] = {0};
    uint16_t channels[16] = {0};
    unsigned c,b;
    channels[0]=1100; channels[1]=900; channels[2]=throttle; channels[3]=1100;
    channels[4]=axis; channels[6]=arm; channels[9]=slider;
    bytes[0]=0x0f; bytes[23]=flags;
    for (c=0;c<16;c++) for (b=0;b<11;b++)
        if (channels[c] & (1U<<b)) bytes[1+(c*11+b)/8] |= (uint8_t)(1U<<((c*11+b)%8));
    test_tick=tick;
    RcSbus_RxBytesFromISR(bytes,sizeof(bytes));
    RcSbus_ProcessPending();
}

static void rc_update(void)
{
    RcSbusData_t data;
    RcSbus_CheckTimeout(); RcSbus_GetData(&data); EscPwm_UpdateFromRc(&data);
    assert(test_critical_depth == 0);
}
static void control_at(uint32_t tick)
{
    test_tick=tick;
    test_dwt.CYCCNT+=480000;
    test_imu.gyro_rx_count++;
    FlightControl_Run();
    assert(test_critical_depth == 0);
}
static void minimum(void)
{
    unsigned i;
    assert(!EscPwm_IsArmed());
    for (i=0;i<4;i++) {
        assert(g_esc_pwm_state.pulse_us[i] == 1000);
        near_value(g_esc_pwm_state.command[i],0);
    }
    assert(htim1.compare[TIM_CHANNEL_1] == 1000 && htim1.compare[TIM_CHANNEL_3] == 1000);
    assert(htim2.compare[TIM_CHANNEL_1] == 1000 && htim2.compare[TIM_CHANNEL_3] == 1000);
}
static uint32_t arm_and_run(uint32_t start)
{
    setup(start);
    frame_at(start+2000,1000,348,1696,1024,0); control_at(test_tick);
    assert(g_esc_pwm_state.arm_middle_seen);
    frame_at(start+2001,1696,348,1696,1024,0); control_at(test_tick);
    assert(EscPwm_IsArmed());
    frame_at(start+2002,1696,1022,1696,352,0); control_at(test_tick);
    assert(g_rc_sbus_data.tuning_axis_valid);
    g_flight_control_pid.rate_roll.kp=0.1f;
    g_flight_control_pid.rate_roll.ki=0.1f;
    s_roll_integral=0.1f;
    control_at(test_tick);
    assert(g_esc_pwm_state.pulse_us[0] > 1000);
    return test_tick;
}
static void same_targets(const RcSbusData_t *good)
{
    near_value(g_rc_sbus_data.throttle,good->throttle);
    near_value(g_rc_sbus_data.roll_target_rad,good->roll_target_rad);
    near_value(g_rc_sbus_data.pitch_target_rad,good->pitch_target_rad);
    near_value(g_rc_sbus_data.yaw_rate_target_rad_s,good->yaw_rate_target_rad_s);
    near_value(g_rc_sbus_data.tuning_rate_rad_s,good->tuning_rate_rad_s);
    assert(g_rc_sbus_data.tuning_axis==good->tuning_axis);
    assert(g_rc_sbus_data.last_good_arm_channel==good->last_good_arm_channel);
    assert(g_rc_sbus_data.last_valid_tick==good->last_valid_tick);
}

static void test_fault_deadlines(void)
{
    unsigned kind,ms;
    for (kind=0;kind<6;kind++) {
        uint32_t last=arm_and_run(0);
        RcSbusData_t good;
        float first_motor=g_esc_pwm_state.command[0];
        RcSbus_GetData(&good);
        for (ms=1;ms<=19;ms++) {
            test_tick=last+ms;
            if (kind<3) {
                uint8_t flags=(kind==0) ? RC_SBUS_FLAG_FAILSAFE :
                    ((kind==1) ? RC_SBUS_FLAG_FRAME_LOST : 12U);
                frame_at(test_tick,352,348,352,1024,flags);
            } else if (kind==3 || kind==4) {
                HAL_UART_ErrorCallback(&huart5);
                /* Flight must see pending error even before RcTask restarts RX. */
                control_at(test_tick);
                assert(g_rc_sbus_data.link_state==RC_LINK_HOLD);
                if (kind==4) test_uart_rx_status=HAL_ERROR;
                RcSbus_ProcessPending();
            }
            test_imu.gyro_y=-0.2f;
            control_at(test_tick);
            assert(EscPwm_IsArmed() && g_flight_control_state.rc_valid);
            same_targets(&good);
            assert(s_roll_integral>0.09f && s_have_error);
        }
        /* Measured motion changes PWM during hold: this is not a PWM freeze. */
        assert(fabsf(g_esc_pwm_state.command[0]-first_motor)>0.01f);
        assert(g_rc_sbus_data.timeout_count==0);
        control_at(last+20); minimum();
        assert(!g_rc_sbus_data.valid && g_rc_sbus_data.link_state==RC_LINK_TIMED_OUT);
        assert(g_rc_sbus_data.timeout_count==1);
        near_value(s_roll_integral,0); assert(!s_have_error);
        near_value(g_rc_sbus_data.throttle,0);
        control_at(last+21); rc_update(); minimum();
        assert(g_rc_sbus_data.timeout_count==1);
        if (kind==3 || kind==4) assert(test_uart_abort_calls>=20);
    }
    puts("PASS: failsafe/lost/both/UART error/restart failure/silence, 19 ms hold, 20 ms lock, PID continues");
}

static void test_recovery(void)
{
    unsigned age;
    for (age=19;age<=21;age++) {
        uint32_t last=arm_and_run(0);
        RcSbusData_t before, recovery;
        RcSbus_GetData(&before);
        frame_at(last+1,352,348,352,1024,RC_SBUS_FLAG_FAILSAFE);
        /* No timeout poll at all before recovery: the decoder must latch expiry. */
        frame_at(last+age,1696,348,1696,352,0);
        RcSbus_GetData(&recovery);
        assert(recovery.valid && recovery.link_state==RC_LINK_ACTIVE);
        assert(recovery.timeout_count==(age>=20 ? 1U : 0U));
        if (age==19) {
            control_at(test_tick);
            assert(EscPwm_IsArmed()); near_value(s_roll_integral,0.1f);
        } else {
            /* Test both consumers and repeated snapshots. */
            EscPwm_UpdateFromRc(&recovery); minimum();
            control_at(test_tick); minimum(); near_value(s_roll_integral,0);
            EscPwm_UpdateFromRc(&before); minimum();
            EscPwm_UpdateFromRc(&recovery); minimum();
            frame_at(test_tick+1,1696,348,1696,1024,0); control_at(test_tick); minimum();
            frame_at(test_tick+1,1000,348,1696,1024,RC_SBUS_FLAG_FAILSAFE);
            control_at(test_tick); assert(!g_esc_pwm_state.arm_middle_seen);
            frame_at(test_tick+1,1696,348,1696,1024,0); control_at(test_tick); minimum();
            frame_at(test_tick+1,1000,348,1696,1024,0); rc_update();
            assert(g_esc_pwm_state.arm_middle_seen);
            frame_at(test_tick+1,1696,348,1696,1024,0); control_at(test_tick);
            assert(EscPwm_IsArmed());
            /* An older RcTask snapshot cannot override newer safety state. */
            EscPwm_UpdateFromRc(&before); EscPwm_UpdateFromRc(&recovery);
            assert(EscPwm_IsArmed());
        }
    }
    /* A middle recovery frame itself cannot satisfy re-arm after an unseen timeout. */
    { uint32_t last=arm_and_run(0);
      frame_at(last+20,1000,348,1696,1024,0); rc_update();
      assert(!g_esc_pwm_state.arm_middle_seen); minimum();
      frame_at(last+21,1696,348,1696,1024,0); control_at(test_tick); minimum(); }
    puts("PASS: recovery at 19/20/21 ms, sticky timeout, fresh middle/high required, stale/replayed snapshots");
}

static void test_uart_retries(void)
{
    uint32_t last=arm_and_run(0);
    RcSbusData_t good;
    RcSbus_GetData(&good);
    test_tick=last+1; HAL_UART_ErrorCallback(&huart5);
    test_uart_abort_status=HAL_ERROR;
    RcSbus_ProcessPending(); control_at(last+10);
    same_targets(&good); assert(EscPwm_IsArmed() && s_restart_pending);
    test_uart_abort_status=HAL_OK; test_uart_rx_status=HAL_ERROR;
    test_tick=last+18; RcSbus_ProcessPending();
    assert(s_restart_pending); same_targets(&good);
    test_uart_rx_status=HAL_OK;
    test_tick=last+19; RcSbus_ProcessPending(); control_at(test_tick);
    assert(!s_restart_pending && EscPwm_IsArmed()); same_targets(&good);
    control_at(last+20); minimum();
    /* Successful restart and a GOOD frame at 19 ms recover with history intact. */
    last=arm_and_run(0);
    test_tick=last+1; HAL_UART_ErrorCallback(&huart5); RcSbus_ProcessPending();
    frame_at(last+19,1696,1022,1696,352,0); control_at(test_tick);
    assert(EscPwm_IsArmed() && g_rc_sbus_data.timeout_count==0);
    near_value(s_roll_integral,0.1f);
    puts("PASS: UART abort/start success and failure cannot renew age; good recovery preserves PID");
}

static void test_protections(void)
{
    unsigned i;
    uint32_t last;
    float motor[4]={1,1,1,1};
    setup(0);
    for (i=0;i<3;i++) {
        test_tick=(i==0)?0:((i==1)?20:5000);
        rc_update(); control_at(test_tick); minimum();
        assert(g_rc_sbus_data.link_state==RC_LINK_NO_DATA && !g_rc_sbus_data.has_valid_frame);
        assert(g_rc_sbus_data.timeout_count==0);
    }
    frame_at(5001,1696,348,1696,1024,RC_SBUS_FLAG_FAILSAFE); rc_update(); minimum();
    frame_at(5002,1696,348,1696,1024,0); rc_update(); minimum();
    frame_at(5003,1000,1022,1696,1024,0); rc_update();
    assert(!g_esc_pwm_state.arm_middle_seen);
    frame_at(5004,1696,348,1696,1024,0); rc_update(); minimum();
    setup(0);
    frame_at(0,1000,348,1696,1024,0); rc_update();
    frame_at(1,1696,348,1696,1024,0); rc_update(); minimum();
    assert(g_esc_pwm_state.inhibit_reason==ESC_PWM_INHIBIT_STARTUP);
    last=arm_and_run(0);
    frame_at(last+1,352,348,1696,352,RC_SBUS_FLAG_FAILSAFE); control_at(test_tick);
    assert(EscPwm_IsArmed());
    frame_at(last+2,1000,348,1696,352,0); control_at(test_tick); minimum();
    EscPwm_SetMixedNormalized(motor); EscPwm_SetAllNormalized(1);
    EscPwm_SetMotorNormalized(ESC_PWM_MOTOR_1,1); minimum();
    last=arm_and_run(0);
    frame_at(last+1,1000,348,1696,1024,0);
    frame_at(last+2,1696,1022,1696,352,RC_SBUS_FLAG_FAILSAFE);
    /* Last good middle locks even when latest diagnostic frame says high. */
    control_at(test_tick); minimum(); assert(!g_esc_pwm_state.arm_middle_seen);
    last=arm_and_run(0);
    frame_at(last+1,1696,348,1696,352,RC_SBUS_FLAG_FAILSAFE);
    test_imu.gyro_valid=0; control_at(test_tick); minimum();
    last=arm_and_run(0);
    frame_at(last+1,1696,348,1696,352,RC_SBUS_FLAG_FAILSAFE);
    test_tick=last+10; FlightControl_OnTaskTimeout(); minimum();
    assert(g_esc_pwm_state.inhibit_reason==ESC_PWM_INHIBIT_IMU_TIMEOUT);
    assert(g_rc_sbus_data.valid && !s_filter_initialized);
    setup(0); test_check_pwm_critical=0; test_pwm_start_status=HAL_ERROR;
    assert(EscPwm_Init()==HAL_ERROR); minimum();
    test_check_pwm_critical=1;
    frame_at(3000,1000,348,1696,1024,0); rc_update();
    frame_at(3001,1696,348,1696,1024,0); rc_update(); minimum();
    puts("PASS: no data, startup, high-on-boot, low throttle, valid switch lock, IMU and init protections");
}

static void test_wrap_and_wait_path(void)
{
    RcSbusData_t data;
    uint32_t last;
    setup(0);
    frame_at(0,1696,348,1696,1024,0);
    RcSbus_GetData(&data);
    assert(RcSbus_IsControlUsable(&data,19));
    assert(!RcSbus_IsControlUsable(&data,20));
    test_tick=20; RcSbus_GetData(&data); assert(data.timeout_count==1);
    last=arm_and_run(UINT32_MAX-2010U);
    control_at(last+19); assert(EscPwm_IsArmed());
    control_at(last+20); minimum(); assert(g_rc_sbus_data.timeout_count==1);
    last=arm_and_run(UINT32_MAX-2010U);
    frame_at(last+20,1696,348,1696,1024,0); control_at(test_tick); minimum();
    assert(g_rc_sbus_data.timeout_count==1);
    last=arm_and_run(0);
    g_rc_sbus_data.valid_frame_count=UINT32_MAX;
    s_rc_frame_seen=UINT32_MAX;
    frame_at(last+1,1696,1022,1696,352,0); control_at(test_tick);
    assert(g_rc_sbus_data.valid_frame_count==0 && EscPwm_IsArmed());
    g_rc_sbus_data.timeout_count=UINT32_MAX;
    s_rc_timeout_seen=s_control_rc_timeout_seen=UINT32_MAX;
    /* Flight's no-notification path must enforce RC deadline too; recent IMU. */
    control_at(last+19);
    test_tick=last+21; FlightControl_OnTaskTimeout(); minimum();
    assert(g_rc_sbus_data.timeout_count==0 && s_filter_initialized);
    assert(g_esc_pwm_state.inhibit_reason==ESC_PWM_INHIBIT_RC_TIMEOUT);
    near_value(s_roll_integral,0);
    /* RcTask consumes expiry, then sees a NEW middle/high sequence before the
     * flight task sees the event. The latter resets PID, not the rearmed ESC. */
    last=arm_and_run(0);
    control_at(last+19);
    frame_at(last+20,1696,348,1696,1024,0); rc_update(); minimum();
    frame_at(last+21,1000,348,1696,1024,0); rc_update();
    frame_at(last+22,1696,348,1696,1024,0); rc_update();
    assert(EscPwm_IsArmed());
    FlightControl_OnTaskTimeout(); assert(EscPwm_IsArmed());
    near_value(s_roll_integral,0);
    puts("PASS: tick zero/wrap, frame/event counter wrap, no-notification RC expiry preserves gyro filter");
}

static void test_tuning_hold(void)
{
    uint32_t last=arm_and_run(0);
    frame_at(last+1,1000,348,1696,1024,0); control_at(test_tick);
    g_flight_control_tuning.pid_debug=1; control_at(test_tick);
    frame_at(test_tick+1,1696,348,1696,352,0); control_at(test_tick);
    assert(EscPwm_IsArmed() && g_flight_control_tuning.waiting_center);
    frame_at(test_tick+1,352,348,352,1024,RC_SBUS_FLAG_FAILSAFE); control_at(test_tick);
    assert(g_flight_control_tuning.waiting_center && s_selected_axis==RC_TUNING_ROLL);
    frame_at(test_tick+1,1696,348,1696,1024,0); control_at(test_tick);
    assert(!g_flight_control_tuning.waiting_center);
    frame_at(test_tick+1,1696,1022,1696,352,0); control_at(test_tick);
    last=test_tick;
    s_roll_integral=0.1f;
    frame_at(last+1,352,348,352,1024,RC_SBUS_FLAG_FRAME_LOST); control_at(test_tick);
    assert(!g_flight_control_tuning.waiting_center);
    assert(s_selected_axis==RC_TUNING_ROLL && s_roll_integral>0.1f);
    near_value(g_flight_control_state.roll_rate_target_rad_s,1.570796327f);
    frame_at(last+2,1696,1022,352,352,0); control_at(test_tick);
    frame_at(last+3,1696,1022,352,352,0); control_at(test_tick);
    assert(s_selected_axis==RC_TUNING_ROLL);
    frame_at(last+4,1696,1022,352,352,0); control_at(test_tick);
    assert(s_selected_axis==RC_TUNING_PITCH && g_flight_control_tuning.waiting_center);
    near_value(s_roll_integral,0);
    puts("PASS: hold preserves tuning target/axis/PID; fault CH10 cannot recenter; three fresh frames switch axis");
}

int main(void)
{
    test_fault_deadlines();
    test_recovery();
    test_uart_retries();
    test_protections();
    test_wrap_and_wait_path();
    test_tuning_hold();
    assert(test_critical_depth==0);
    puts("PASS: real RC + flight + ESC integration, PWM writes protected by balanced critical sections");
    return 0;
}
