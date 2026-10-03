/* Reuse the real SPI/register acquisition fixture, then exercise its heater. */
#define main acquisition_regression_main
#include "test_bmi088.c"
#undef main
#include "../../App/bmi088_heater.c"

static void heater_reset(uint32_t tick)
{
    model_reset(tick);
    test_pwm_start_status = HAL_OK;
    test_check_pwm_critical = 0;
    Bmi088Heater_Init(HAL_OK);
    assert(g_bmi088_heater_state.initialized && htim3.compare[TIM_CHANNEL_4] == 0);
    Bmi088Heater_Process(); assert(!g_bmi088_heater_state.heating);
    ready(); Bmi088Heater_Process();
    assert(g_bmi088_heater_state.stop_reason == BMI088_HEATER_WAIT_SAMPLE);
    test_check_pwm_critical = 1;
}

static void temperature(float value, unsigned delay)
{
    test_tick += delay;
    measurement((int)floorf((value - 23.0f) * 8.0f + 0.5f));
    Bmi088_Process();
    Bmi088Heater_Process();
    assert(g_bmi088_heater_state.pwm_ccr <= 500);
    assert(htim3.compare[TIM_CHANNEL_4] == g_bmi088_heater_state.pwm_ccr);
}

int main(void)
{
    uint32_t count, sample;
    unsigned i, status;
    float integral;
    TIM_HandleTypeDef esc1, esc2;
    /* Sensor acquisition remains independent from thermal control. */
    acquisition_regression_main();
    heater_reset(100); temperature(39, 1);
    assert(g_bmi088_heater_state.target_c == 40 && g_bmi088_heater_state.pwm_ccr == 100);
    assert(g_bmi088_heater_state.integral_ccr == 0);
    temperature(39, 20);
    assert(g_bmi088_heater_state.pwm_ccr == 101 && fabs(g_bmi088_heater_state.integral_ccr - 1) < 1e-6);
    count = g_bmi088_heater_state.update_count; integral = s_integral_ccr;
    temperature(38, 1); assert(g_bmi088_heater_state.update_count == count);
    for(i=0;i<18;i++) { test_tick++; Bmi088Heater_Process(); }
    assert(g_bmi088_heater_state.update_count == count && s_integral_ccr == integral);
    /* Independent default-task guard, with NO BMI/heater task calls. */
    test_tick++; Bmi088Heater_CheckTimeout(); assert(g_bmi088_heater_state.heating); /* age 19 ms */
    test_tick++; Bmi088Heater_CheckTimeout(); /* age 20 ms */
    assert(g_bmi088_heater_state.stop_reason == BMI088_HEATER_SENSOR_TIMEOUT);
    assert(!g_bmi088_heater_state.heating && s_integral_ccr == 0);
    temperature(39, 1); assert(g_bmi088_heater_state.pwm_ccr == 100);

    heater_reset(100); temperature(39, 1); temperature(39, 50);
    assert(fabs(s_integral_ccr - 2.5f) < 1e-6 && g_bmi088_heater_state.pwm_ccr == 103);
    temperature(39, 200); /* Scheduler gap resets history, no integration burst. */
    assert(s_integral_ccr == 0 && g_bmi088_heater_state.pwm_ccr == 100);

    /* Maximum output is 5%, saturation cannot keep increasing integral. */
    heater_reset(100); temperature(20, 1);
    for(i=0;i<1000;i++) temperature(20, 20);
    assert(g_bmi088_heater_state.pwm_ccr == 500 && s_integral_ccr == 0);
    assert(fabs(g_bmi088_heater_state.duty - .05f) < 1e-7);
    for(i=0;i<4;i++) { htim1.compare[i]=1100+i; htim2.compare[i]=1500+i; }
    esc1=htim1;esc2=htim2; temperature(39, 20);
    assert(!memcmp(&esc1,&htim1,sizeof(esc1)) && !memcmp(&esc2,&htim2,sizeof(esc2)));
    g_bmi088_heater_force_stop=1; Bmi088Heater_CheckTimeout();
    assert(g_bmi088_heater_state.stop_reason == BMI088_HEATER_MANUAL_STOP && s_integral_ccr == 0);
    g_bmi088_heater_force_stop=0; temperature(41, 1);
    assert(g_bmi088_heater_state.pwm_ccr == 0 && !g_bmi088_heater_state.overtemperature_latched);
    temperature(45, 20);
    assert(g_bmi088_heater_state.overtemperature_latched && g_bmi088_heater_state.overtemperature_count == 1);
    g_bmi088_heater_force_stop=1; temperature(30, 20);
    g_bmi088_heater_force_stop=0; temperature(30, 20);
    assert(g_bmi088_heater_state.stop_reason == BMI088_HEATER_OVERTEMPERATURE && !g_bmi088_heater_state.heating);

    /* SPI failure stops heat on the same task activation, then retry/resume
     * uses zero integral. No extra SPI owner is introduced by the guard. */
    heater_reset(100); temperature(30, 1); sample=g_bmi088_data.sample_count;
    fail_call=test_spi_calls+1;fail_status=HAL_TIMEOUT;
    test_tick++; Bmi088_Process(); Bmi088Heater_Process();
    assert(!g_bmi088_heater_state.heating && g_bmi088_data.sample_count == sample);
    assert(g_bmi088_heater_state.stop_reason == BMI088_HEATER_SENSOR_INVALID);
    fail_call=0;test_tick+=1000;ready();temperature(39,1);
    assert(g_bmi088_heater_state.pwm_ccr == 100 && s_integral_ccr == 0);
    Bmi088_OnTaskCreationFailure();Bmi088Heater_CheckTimeout();
    assert(!g_bmi088_heater_state.heating);

    for(status=HAL_ERROR;status<=HAL_TIMEOUT;status++) {
        test_check_pwm_critical=0;Bmi088Heater_Init((HAL_StatusTypeDef)status);
        Bmi088Heater_Process();assert(!g_bmi088_heater_state.initialized && !g_bmi088_heater_state.heating);
        assert(g_bmi088_heater_state.stop_reason == BMI088_HEATER_PWM_ERROR);
        test_pwm_start_status=(HAL_StatusTypeDef)status;Bmi088Heater_Init(HAL_OK);
        assert(!g_bmi088_heater_state.initialized && htim3.compare[TIM_CHANNEL_4] == 0);
    }
    /* Corrupt non-finite/implausible temperatures cannot request full heat. */
    {
        const float bad[]={NAN,INFINITY,-INFINITY,-41,86};
        heater_reset(100);temperature(30,1);
        for(i=0;i<sizeof(bad)/sizeof(bad[0]);i++) {
            g_bmi088_data.temperature_c=bad[i];Bmi088Heater_CheckTimeout();
            assert(g_bmi088_heater_state.stop_reason == BMI088_HEATER_BAD_TEMPERATURE && !g_bmi088_heater_state.heating);
        }
    }
    heater_reset(UINT32_MAX-240U);temperature(39,1);temperature(39,20);
    assert(g_bmi088_heater_state.pwm_ccr == 101);
    test_tick+=20;Bmi088Heater_CheckTimeout();assert(!g_bmi088_heater_state.heating);

    /* First-order thermal PLANT model, not a hardware temperature guarantee:
     * quantized 0.125 C measurements, ambient 25 C, 50 C equilibrium at 5%. */
    {
        float actual=25, measured=25, max_temperature=actual;
        heater_reset(100);temperature(actual,1);
        for(i=0;i<6000;i++) {
            actual += (25 + 500*g_bmi088_heater_state.duty - actual)*.02f/10;
            if(actual>max_temperature)max_temperature=actual;
            /* Datasheet temperature register refreshes only every 1.28 s. */
            if(i%64==0)measured=actual;
            temperature(measured,20);
        }
        assert(fabs(actual-40)<.25 && max_temperature<45);
        assert(!g_bmi088_heater_state.overtemperature_latched);
        printf("PASS thermal MODEL: final %.3f C, peak %.3f C; actual board response unverified\n",actual,max_temperature);
    }
    assert(test_critical_depth == 0);
    puts("PASS 40 C PI, fresh cadence, anti-windup/5% limit, independent stale guard, faults/manual stop/45 C latch, tick wrap, ESC isolation");
    return 0;
}
