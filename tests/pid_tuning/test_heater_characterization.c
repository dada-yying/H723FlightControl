/* Host characterization of the developer-approved heater baseline. */
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
    Bmi088Heater_Process();
    ready();
    Bmi088Heater_Process();
    assert(g_bmi088_heater_state.stop_reason == BMI088_HEATER_WAIT_SAMPLE);
    test_check_pwm_critical = 1;
}

static void observe(const char *name, uint32_t ccr, float integral)
{
    Bmi088HeaterState_t s = g_bmi088_heater_state;
    assert(s.pwm_ccr == ccr);
    assert(fabsf(s.integral_ccr - integral) < 0.00001f);
    assert(fabsf(s.duty - (float)ccr / 10000.0f) < 0.0000001f);
    assert(htim3.compare[TIM_CHANNEL_4] == ccr);
    assert(s.pwm_ccr <= 1500U);
    printf("%s CCR=%u integral=%.9g duty=%.9g integral_hex=%a duty_hex=%a limit=1500 reason=%u\n",
           name, s.pwm_ccr, s.integral_ccr, s.duty,
           (double)s.integral_ccr, (double)s.duty, (unsigned)s.stop_reason);
}

static void sample(float degrees, unsigned elapsed)
{
    test_tick += elapsed;
    measurement((int)floorf((degrees - 23.0f) * 8.0f + 0.5f));
    event_process();
    Bmi088Heater_Process();
}

int main(void)
{
    unsigned i;
    assert(HEATER_KP_CCR_PER_C == 120.0f);
    assert(HEATER_KI_CCR_PER_C_S == 15.0f);
    assert(BMI088_HEATER_MAX_CCR == 1500U);
    heater_reset(100);
    observe("waiting", 0, 0);
    sample(39, 1);
    observe("39C_first", 120, 0);
    sample(39, 20);
    observe("39C_20ms", 120, 0.3f);
    sample(39, 20);
    observe("39C_40ms", 121, 0.6f);
    sample(38, 1);
    observe("cadence_hold", 121, 0.6f);
    test_tick += 19;
    Bmi088Heater_CheckTimeout();
    observe("age19", 121, 0.6f);
    test_tick++;
    Bmi088Heater_CheckTimeout();
    assert(g_bmi088_heater_state.stop_reason == BMI088_HEATER_SENSOR_TIMEOUT);
    observe("age20_stop", 0, 0);
    sample(39, 1);
    observe("timeout_recovery", 120, 0);
    sample(39, 50);
    observe("50ms_integral", 121, 0.75f);
    sample(39, 200);
    observe("gap_reset", 120, 0);
    heater_reset(100);
    sample(20, 1);
    observe("cold_saturation", 1500, 0);
    for (i = 0; i < 1000; i++)
    {
        sample(20, 20);
        assert(g_bmi088_heater_state.pwm_ccr == 1500 && s_integral_ccr == 0);
    }
    observe("saturation_1000_steps", 1500, 0);
    g_bmi088_heater_force_stop = 1;
    Bmi088Heater_CheckTimeout();
    assert(g_bmi088_heater_state.stop_reason == BMI088_HEATER_MANUAL_STOP);
    observe("manual_stop", 0, 0);
    g_bmi088_heater_force_stop = 0;
    sample(41, 20);
    observe("above_target_lower_saturation", 0, 0);
    sample(45, 20);
    assert(g_bmi088_heater_state.overtemperature_latched);
    observe("overtemperature", 0, 0);
    sample(30, 20);
    assert(g_bmi088_heater_state.stop_reason == BMI088_HEATER_OVERTEMPERATURE);
    observe("latched_after_cooling", 0, 0);
    heater_reset(100);
    sample(30, 1);
    observe("before_spi_fault", 1200, 0);
    fail_call = test_spi_calls + 1;
    fail_status = HAL_TIMEOUT;
    test_tick++;
    event_process();
    Bmi088Heater_Process();
    assert(g_bmi088_heater_state.stop_reason == BMI088_HEATER_SENSOR_INVALID);
    observe("spi_fault", 0, 0);
    heater_reset(UINT32_MAX - 240U);
    sample(39, 1);
    sample(39, 20);
    observe("tick_wrap", 120, 0.3f);
    assert(test_critical_depth == 0);
    puts("PASS approved heater characterization; no thermal plant or hardware claim");
    return 0;
}
