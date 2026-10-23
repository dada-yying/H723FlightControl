/* Sequential safety characterization, not a concurrency proof. */
#include <assert.h>
#include <stdio.h>
#include "../../App/rc_sbus.c"
#include "../../App/optical_flow.c"
#include "../../App/esc_pwm.c"
#include "../../App/flight_control.c"
static AtomImuData_t imu;
void AtomImu_GetData(AtomImuData_t *data) { *data = imu; }

static void frame(uint16_t arm, uint16_t throttle, uint8_t flags)
{
    uint8_t bytes[25] = {0};
    uint16_t channels[16] = {1024, 1040, 348, 1024};
    unsigned c, b;
    RcSbusData_t rc;
    channels[2] = throttle;
    channels[6] = arm;
    bytes[0] = 15;
    bytes[23] = flags;
    for (c = 0; c < 16; c++)
    {
        for (b = 0; b < 11; b++)
        {
            if (channels[c] & (1U << b))
                bytes[1 + (c * 11 + b) / 8] |= (uint8_t)(1U << ((c * 11 + b) % 8));
        }
    }
    RcSbus_RxBytesFromISR(bytes, 25);
    RcSbus_ProcessPending();
    RcSbus_CheckTimeout();
    RcSbus_GetData(&rc);
    EscPwm_UpdateFromRc(&rc);
}
static void check(const char *name, unsigned armed, unsigned pulse)
{
    EscPwmState_t s;
    unsigned i;
    EscPwm_GetState(&s);
    assert(s.armed == armed);
    for (i = 0; i < 4; i++) assert(s.pulse_us[i] == pulse);
    assert(htim1.compare[TIM_CHANNEL_3] == pulse);
    assert(htim1.compare[TIM_CHANNEL_1] == pulse);
    assert(htim2.compare[TIM_CHANNEL_3] == pulse);
    assert(htim2.compare[TIM_CHANNEL_1] == pulse);
    printf("%s armed=%u PWM=%u,%u,%u,%u reason=%u valid=%u\n", name,
           s.armed, s.pulse_us[0], s.pulse_us[1], s.pulse_us[2], s.pulse_us[3],
           s.inhibit_reason, g_rc_sbus_data.valid);
}
static void reset(uint32_t tick)
{
    test_tick = tick;
    test_check_pwm_critical = 0;
    test_pwm_start_status = HAL_OK;
    test_rx_status = test_rx_abort_status = HAL_OK;
    RcSbus_Init();
    assert(EscPwm_Init() == HAL_OK);
    FlightControl_Init();
    memset(&imu, 0, sizeof(imu));
    imu.gyro_valid = 1;
    RcSbus_StartReception();
    RcSbus_ProcessPending();
}
int main(void)
{
    const float commands[4] = {0.25f, 0.25f, 0.25f, 0.25f};
    RcSbusData_t rc;
    reset(0);
    frame(1000, 348, 0);
    frame(1696, 348, 0);
    check("startup_guard", 0, 1000);
    test_tick = 2000;
    frame(800, 348, 0);
    frame(1200, 348, 0);
    check("low_high_rejected", 0, 1000);
    frame(1000, 1696, 0);
    frame(1696, 348, 0);
    check("high_throttle_prearm_rejected", 0, 1000);
    frame(1000, 348, 0);
    frame(1200, 348, 0);
    check("middle_high_armed", 1, 1000);
    EscPwm_SetMixedNormalized(commands);
    check("armed_output", 1, 1250);
    frame(800, 1696, RC_SBUS_FLAG_FRAME_LOST);
    check("lost_frame_hold", 1, 1250);
    test_tick = 2099;
    RcSbus_CheckTimeout(); RcSbus_GetData(&rc); EscPwm_UpdateFromRc(&rc);
    check("age99_hold", 1, 1250);
    test_tick = 2100;
    RcSbus_CheckTimeout(); RcSbus_GetData(&rc); EscPwm_UpdateFromRc(&rc);
    check("age100_stop", 0, 1000);
    frame(1696, 348, 0);
    check("recovery_requires_middle", 0, 1000);
    frame(1000, 348, 0); frame(1696, 348, 0);
    EscPwm_SetMixedNormalized(commands);
    frame(1696, 348, RC_SBUS_FLAG_FAILSAFE);
    check("failsafe_stop", 0, 1000);
    frame(1000, 348, 0); frame(1696, 348, 0);
    EscPwm_SetMixedNormalized(commands);
    frame(800, 348, 0);
    check("switch_lock", 0, 1000);
    frame(1000, 348, 0); frame(1696, 348, 0);
    FlightControl_Run();
    test_tick += 10;
    FlightControl_OnTaskTimeout();
    check("imu_timeout", 0, 1000);
    reset(UINT32_MAX - 3000U);
    test_tick = UINT32_MAX - 50U;
    frame(1000, 348, 0); frame(1696, 348, 0);
    EscPwm_SetMixedNormalized(commands);
    test_tick = 48;
    RcSbus_CheckTimeout(); RcSbus_GetData(&rc); EscPwm_UpdateFromRc(&rc);
    check("wrap_age99", 1, 1250);
    test_tick = 49;
    RcSbus_CheckTimeout(); RcSbus_GetData(&rc); EscPwm_UpdateFromRc(&rc);
    check("wrap_age100", 0, 1000);
    assert(test_critical_depth == 0);
    /* The production BSP must never reset a running or previously initialized clock. */
    test_dwt.CYCCNT = 123456U;
    test_scheduler_state = 1;
    BspDwt_Init();
    assert(test_dwt.CYCCNT == 123456U);
    test_scheduler_state = taskSCHEDULER_NOT_STARTED;
    BspDwt_Init();
    assert(test_dwt.CYCCNT == 123456U);
    BspDwt_Enable();
    assert(BspDwt_IsEnabled() && BspDwt_Cycles() == 123456U);
    assert(BspDwt_ElapsedCycles(123000U) == 456U);
    assert(BspDwt_Us(480U) == 1U);
    puts("PASS minimum sequential RC safety characterization");
    return 0;
}
