/* Exercise the actual UART queue/parser and task-side filter, not a copy. */
#define main optical_parser_regression_main
#include "test_optical_flow.c"
#undef main
#include <math.h>

static uint32_t sensor_ms;
static uint8_t sensor_seq;
static void start_filter(void)
{
    boot(); sensor_ms = 0; sensor_seq = 0;
}
static void observation(uint32_t range_mm, int x, int y, unsigned tof, unsigned flow)
{
    uint8_t p[27] = {0xef, 1, 1, 0x51, 0, 20};
    unsigned i;
    p[4] = sensor_seq++;
    for (i = 0; i < 4; i++) {
        p[6+i] = (uint8_t)(sensor_ms >> (8*i));
        p[10+i] = (uint8_t)(range_mm >> (8*i));
    }
    p[14] = 255; p[16] = (uint8_t)tof;
    p[18] = (uint8_t)x; p[19] = (uint8_t)((uint16_t)x >> 8);
    p[20] = (uint8_t)y; p[21] = (uint8_t)((uint16_t)y >> 8);
    p[22] = 100; p[23] = (uint8_t)flow;
    checksum(p, 27); deliver(p, 27, 1);
    sensor_ms += 10; test_tick += 10;
}
static OpticalFlowEstimate_t estimate(void)
{
    OpticalFlowEstimate_t e;
    OpticalFlowData_t d;
    OpticalFlow_GetSnapshot(&d, &e);
    assert(e.rx_count == d.rx_count);
    return e;
}
static void close_to(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) < tolerance);
}
static void test_new_frames_and_initialization(void)
{
    OpticalFlowEstimate_t e, before;
    unsigned i;
    start_filter(); e = estimate();
    assert(!e.valid && (e.flags & OF_FILTER_NO_SAMPLE));
    assert(e.age_ms == UINT32_MAX && e.range_age_ms == UINT32_MAX);
    observation(250, 100, -200, 1, 1); e = estimate();
    assert(!e.valid && e.accepted_streak == 1 && e.update_count == 1);
    close_to(e.flow_x_filtered, 100, .001f);
    close_to(e.velocity_x_filtered_m_s, .25f, .00001f);
    close_to(e.velocity_y_filtered_m_s, -.5f, .00001f);
    before = e;
    /* Repeated 1 kHz snapshots and idle task wakes cannot advance the filter. */
    for (i = 0; i < 1000; i++) {
        OpticalFlow_ProcessPending(); e = estimate();
        assert(e.update_count == before.update_count);
        assert(e.flow_x_filtered == before.flow_x_filtered);
    }
    observation(250, 100, -200, 1, 1);
    observation(250, 100, -200, 1, 1); e = estimate();
    assert(e.valid && e.accepted_streak == 3 && e.dt_ms == 10 && !e.flags);
    observation(250, 0, 0, 1, 1); e = estimate();
    assert(e.valid && e.update_count == 4 && e.flow_x_filtered > 90);
    assert(g_optical_flow_data.flow_vel_x == 0); /* Zero is a real new sample. */
    before = e;
    deliver((const uint8_t *)g_optical_flow_data.raw, 27, 1); e = estimate();
    assert(!e.valid && e.flags == OF_FILTER_DUPLICATE);
    assert(e.update_count == before.update_count && e.flow_x_filtered == before.flow_x_filtered);
    observation(250, -300, 0, 1, 1); e = estimate();
    assert(!e.valid && e.accepted_streak == 1);
    close_to(e.flow_x_filtered, -300, .001f);
    observation(250, -300, 0, 1, 1); assert(!estimate().valid);
    observation(250, -300, 0, 1, 1); assert(estimate().valid);
}
static void test_range_spike_and_recovery(void)
{
    OpticalFlowEstimate_t e, before;
    unsigned i;
    const unsigned spikes[] = {1142, 2155, 1155};
    start_filter();
    for (i = 0; i < 3; i++) observation(141, -1000, 200, 1, 1);
    before = estimate(); assert(before.valid);
    for (i = 0; i < 3; i++) {
        observation(spikes[i], -1000, 200, 1, 1); e = estimate();
        assert(!e.valid && (e.flags & OF_FILTER_RANGE_JUMP));
        assert(g_optical_flow_data.distance == spikes[i]);
        assert(e.range_accepted_m == before.range_accepted_m);
        assert(e.update_count == before.update_count);
        assert(e.velocity_x_filtered_m_s == before.velocity_x_filtered_m_s);
    }
    observation(156, -1000, 200, 1, 1); e = estimate();
    assert(!e.valid && e.accepted_streak == 1 && !(e.flags & OF_FILTER_RANGE_JUMP));
    close_to(e.velocity_x_filtered_m_s, -1.56f, .00001f);
    observation(156, -1000, 200, 1, 1);
    observation(156, -1000, 200, 1, 1); assert(estimate().valid);
    observation(0, 0, 0, 1, 1); assert(estimate().flags & OF_FILTER_RANGE_BOUNDS);
    observation(79, 0, 0, 1, 1); assert(!estimate().valid);
    observation(250, 0, 0, 0, 1); assert(estimate().flags & OF_FILTER_TOF_STATUS);
    observation(156, 0, 0, 1, 0); e = estimate();
    assert(!e.valid && (e.flags & OF_FILTER_FLOW_STATUS) && e.range_valid);
    /* The accepted-range clock works even when the first acceptance is tick 0. */
    start_filter(); observation(250, 0, 0, 1, 0); e = estimate();
    assert(e.range_valid && e.range_age_ms == 10 && e.update_count == 0);
    start_filter(); observation(80, 0, 0, 1, 1); assert(estimate().range_valid);
    start_filter(); observation(8000, 0, 0, 1, 1); assert(estimate().range_valid);
    observation(8001, 0, 0, 1, 1); assert(estimate().flags & OF_FILTER_RANGE_BOUNDS);
    /* Range is not artificially clamped to the original 15..40 cm test height. */
    start_filter();
    for (i = 0; i < 200; i++) observation(150 + 10*i, 100, -200, 1, 1);
    e = estimate(); assert(e.valid && e.range_accepted_m > 2.0f);
}
static void test_timeouts_gaps_wrap_and_transport(void)
{
    OpticalFlowEstimate_t e;
    uint8_t p[27];
    unsigned i;
    start_filter();
    for (i = 0; i < 3; i++) observation(250, 100, 200, 1, 1);
    test_tick = 50; assert(estimate().valid); /* Last frame arrived at 20 ms. */
    test_tick = 51; e = estimate(); assert(!e.valid && (e.flags & OF_FILTER_STALE));
    test_tick = 70; assert(!estimate().range_valid); /* 50 ms without good range. */
    observation(250, -500, 0, 1, 1); e = estimate();
    assert(!e.valid && e.accepted_streak == 1 && (e.flags & OF_FILTER_TIMING));
    close_to(e.flow_x_filtered, -500, .001f);
    observation(250, 100, 0, 1, 1); observation(250, 100, 0, 1, 1);
    sensor_ms += 10; sensor_seq++; /* One missing 100 Hz sample: restart, no replay. */
    observation(250, 700, 0, 1, 1); e = estimate();
    assert(e.dt_ms == 20 && !e.valid && e.accepted_streak == 1);
    close_to(e.flow_x_filtered, 700, .001f);
    observation(250, 700, 0, 1, 1); observation(250, 700, 0, 1, 1);
    assert(estimate().valid);
    /* Sensor and MCU clocks wrap independently. */
    start_filter(); sensor_ms = UINT32_MAX - 5; sensor_seq = 255; test_tick = UINT32_MAX - 5;
    observation(250, 100, 0, 1, 1); observation(250, 100, 0, 1, 1);
    observation(250, 100, 0, 1, 1); e = estimate();
    assert(e.valid && e.dt_ms == 10 && !(e.flags & OF_FILTER_TIMING));
    /* Delayed task processing preserves raw bytes but rejects stale work. */
    memcpy(p, (const void *)g_optical_flow_data.raw, 27); p[4]++;
    p[6] += 10; checksum(p, 27); deliver(p, 27, 0);
    test_tick += 31; OpticalFlow_ProcessPending(); e = estimate();
    assert(!e.valid && (e.flags & OF_FILTER_STALE));
    huart10.ErrorCode = 8; OpticalFlow_OnError(&huart10);
    e = estimate(); assert(!e.valid && !e.range_valid && (e.flags & OF_FILTER_TRANSPORT));
    OpticalFlow_ProcessPending(); dma_write = 0;
    e = estimate(); assert(!e.valid && !e.range_valid);
    observation(250, -100, 0, 1, 1); e = estimate();
    assert(!e.valid && e.accepted_streak == 1);
    close_to(e.flow_x_filtered, -100, .001f);
}
static double sine_gain(double hz)
{
    double input_energy = 0, output_energy = 0;
    unsigned i;
    start_filter();
    for (i = 0; i < 1200; i++) {
        int x = (int)(10000.0 * sin(6.283185307179586 * hz * i / 100.0));
        OpticalFlowEstimate_t e;
        observation(250, x, -x, 1, 1); e = estimate();
        if (i >= 200) {
            input_energy += (double)x*x;
            output_energy += (double)e.flow_x_filtered*e.flow_x_filtered;
            close_to(e.flow_y_filtered, -e.flow_x_filtered, .01f);
            close_to(e.velocity_x_filtered_m_s, .0025f*e.flow_x_filtered, .0001f);
        }
    }
    return sqrt(output_energy / input_energy);
}
static int replay(const char *input, const char *output)
{
    FILE *in = fopen(input, "rb"), *out = fopen(output, "w");
    uint8_t p[27]; uint32_t tick;
    assert(in && out); start_filter();
    fputs("rx_count,update_count,flow_x,flow_y,range,innovation,vx,vy,vx_filtered,vy_filtered,age_ms,range_age_ms,dt_ms,flags,valid,range_valid,streak\n", out);
    while (fread(&tick, 4, 1, in) == 1) {
        OpticalFlowEstimate_t e;
        assert(fread(p, 1, 27, in) == 27);
        test_tick = tick; deliver(p, 27, 1); e = estimate();
        fprintf(out, "%u,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%u,%u,%u,%u,%u,%u\n",
            e.rx_count, e.update_count, e.flow_x_filtered, e.flow_y_filtered,
            e.range_accepted_m, e.range_innovation_m, e.velocity_x_m_s, e.velocity_y_m_s,
            e.velocity_x_filtered_m_s, e.velocity_y_filtered_m_s, e.age_ms, e.range_age_ms,
            e.dt_ms, e.flags, e.valid, e.range_valid, e.accepted_streak);
    }
    assert(feof(in)); fclose(in); fclose(out); return 0;
}
int main(int argc, char **argv)
{
    if (argc == 4 && !strcmp(argv[1], "--replay")) return replay(argv[2], argv[3]);
    test_new_frames_and_initialization();
    test_range_spike_and_recovery();
    test_timeouts_gaps_wrap_and_transport();
    assert(sine_gain(1.0) > .998 && sine_gain(1.0) < 1.001);
    assert(sine_gain(23.0) < .04);
    assert(test_critical_depth == 0);
    puts("PASS optical filter: new frames/5Hz response/range spikes/validity/gaps/wrap/UART recovery");
    return 0;
}
