#include "bmi088_gyro_filter.h"
#include <float.h>
#include <math.h>
#include <string.h>

#define BMI_FILTER_PI 3.14159265358979323846f
#define BMI_FILTER_SQRT_TWO 1.4142135623730950488f

typedef struct { float x1, x2, y1, y2; } BmiGyroHistory;
static BmiGyroHistory s_gyro_history[3];
static BmiGyroHistory s_notch_history[3];
static Bmi088GyroFilterState_t s_gyro_filter;
static uint32_t s_gyro_count, s_gyro_cycle, s_gyro_tick, s_rate_intervals;
static uint64_t s_rate_cycles, s_candidate_cycles;
static float s_candidate_hz;
static uint32_t s_candidate_windows;
static uint8_t s_gyro_count_seen, s_gyro_clock_seen, s_rate_supported;
static uint8_t s_stalled_cycle_seen;
static uint32_t s_stalled_cycle;
volatile Bmi088GyroFilterState_t g_bmi088_gyro_filter;

static int BmiFilter_Finite(float v) { return v >= -FLT_MAX && v <= FLT_MAX; }

static int BmiFilter_Parameters(float cutoff, float hz)
{
    const float center=BMI088_GYRO_NOTCH_CENTER_HZ, bw=BMI088_GYRO_NOTCH_BANDWIDTH_HZ;
    return BmiFilter_Finite(cutoff) && BmiFilter_Finite(hz) &&
        BmiFilter_Finite(center) && BmiFilter_Finite(bw) &&
        cutoff>0 && hz>0 && cutoff<0.45f*hz && center>0 && bw>0 &&
        bw<2.0f*center && center+0.5f*bw<0.45f*hz;
}

/* H(z)=b0*(1-2*cos(w0)z^-1+z^-2)/(1+a1*z^-1+a2*z^-2).
 * beta=tan(pi*BW/fs) makes the two half-power frequencies differ by BW
 * exactly in DIGITAL Hz. Their arithmetic midpoint is not generally f0.
 * Q=f0/BW alone does not encode this digital bandwidth. */
static int BmiFilter_NotchDesign(float center, float bw, float hz, float c[5])
{
    float beta;
    unsigned i;
    if(!BmiFilter_Finite(center) || !BmiFilter_Finite(bw) || !BmiFilter_Finite(hz) ||
       !(center>0 && bw>0 && bw<2.0f*center && center+0.5f*bw<0.45f*hz))return 0;
    beta=tanf(BMI_FILTER_PI*bw/hz);
    c[0]=1.0f/(1.0f+beta);c[1]=-2.0f*c[0]*cosf(2.0f*BMI_FILTER_PI*center/hz);
    c[2]=c[0];c[3]=c[1];c[4]=2.0f*c[0]-1.0f;
    for(i=0;i<5;i++)if(!BmiFilter_Finite(c[i]))return 0;
    return fabsf(c[4])<1.0f && 1.0f+c[3]+c[4]>0 && 1.0f-c[3]+c[4]>0;
}

static void BmiFilter_ResetHistory(uint8_t error, uint8_t reset_clock)
{
    if(s_gyro_filter.initialized)s_gyro_filter.reset_count++;
    s_gyro_filter.initialized=s_gyro_filter.valid=0;
    s_gyro_filter.error=error;
    /* Retain last finite output, but never expose it as valid after failure. */
    if(reset_clock) {
        s_gyro_clock_seen=0;s_rate_intervals=0;s_rate_cycles=0;
        s_candidate_windows=0;s_candidate_cycles=0;
    }
}

/* Bilinear transform with prewarping, Q=1/sqrt(2). Commit only a complete,
 * finite, stable coefficient set. Called at init/confirmed rate changes only. */
static int BmiFilter_Design(float cutoff, float hz)
{
    float k, n, b0, b1, b2, a1, a2;
    float notch[5];
    if(!BmiFilter_Parameters(cutoff,hz) ||
       !BmiFilter_NotchDesign(BMI088_GYRO_NOTCH_CENTER_HZ,BMI088_GYRO_NOTCH_BANDWIDTH_HZ,hz,notch))return 0;
    k=tanf(BMI_FILTER_PI*cutoff/hz);
    n=1.0f/(1.0f+BMI_FILTER_SQRT_TWO*k+k*k);
    b0=k*k*n;b1=2.0f*b0;b2=b0;
    a1=2.0f*(k*k-1.0f)*n;a2=(1.0f-BMI_FILTER_SQRT_TWO*k+k*k)*n;
    if(!BmiFilter_Finite(b0) || !BmiFilter_Finite(b1) || !BmiFilter_Finite(b2) ||
       !BmiFilter_Finite(a1) || !BmiFilter_Finite(a2) || !(b0>0) ||
       !(fabsf(a2)<1.0f && 1.0f+a1+a2>0 && 1.0f-a1+a2>0))return 0;
    s_gyro_filter.b0=b0;s_gyro_filter.b1=b1;s_gyro_filter.b2=b2;
    s_gyro_filter.a1=a1;s_gyro_filter.a2=a2;
    s_gyro_filter.cutoff_hz=cutoff;s_gyro_filter.applied_sample_hz=hz;
    s_gyro_filter.notch_center_hz=BMI088_GYRO_NOTCH_CENTER_HZ;
    s_gyro_filter.notch_bandwidth_hz=BMI088_GYRO_NOTCH_BANDWIDTH_HZ;
    s_gyro_filter.notch_b0=notch[0];s_gyro_filter.notch_b1=notch[1];s_gyro_filter.notch_b2=notch[2];
    s_gyro_filter.notch_a1=notch[3];s_gyro_filter.notch_a2=notch[4];
    s_gyro_filter.coefficient_update_count++;
    return 1;
}

void Bmi088GyroFilter_Init(void)
{
    const float nominal_hz=1000000.0f/(float)BMI088_READ_PERIOD_US;
    memset(&s_gyro_filter,0,sizeof(s_gyro_filter));
    memset(s_gyro_history,0,sizeof(s_gyro_history));
    memset(s_notch_history,0,sizeof(s_notch_history));
    s_gyro_count_seen=s_gyro_clock_seen=0;s_rate_intervals=0;
    s_stalled_cycle_seen=0;
    s_rate_cycles=s_candidate_cycles=0;s_candidate_windows=0;
    s_gyro_filter.age_ms=UINT32_MAX;s_gyro_filter.sample_hz=nominal_hz;
    s_rate_supported=(uint8_t)BmiFilter_Design(BMI088_GYRO_FILTER_CUTOFF_HZ,nominal_hz);
    s_gyro_filter.error=s_rate_supported ? BMI088_GYRO_FILTER_WAIT_SAMPLE : BMI088_GYRO_FILTER_BAD_PARAMETERS;
    g_bmi088_gyro_filter=s_gyro_filter;
}

void Bmi088GyroFilter_Invalidate(void)
{
    BmiFilter_ResetHistory(BMI088_GYRO_FILTER_SOURCE_INVALID,1);
    taskENTER_CRITICAL();g_bmi088_gyro_filter=s_gyro_filter;taskEXIT_CRITICAL();
}

static void BmiFilter_MeasureRate(uint32_t cycles)
{
    float hz, deviation;
    s_rate_cycles+=cycles;s_rate_intervals++;
    if(s_rate_intervals<BMI088_GYRO_FILTER_RATE_WINDOW)return;
    hz=(float)((double)SystemCoreClock*(double)s_rate_intervals/(double)s_rate_cycles);
    s_gyro_filter.sample_hz=hz;
    if(!BmiFilter_Parameters(BMI088_GYRO_FILTER_CUTOFF_HZ,hz)) {
        s_rate_supported=0;s_candidate_windows=0;s_candidate_cycles=0;
        BmiFilter_ResetHistory(BMI088_GYRO_FILTER_BAD_PARAMETERS,0);
    } else {
        deviation=hz-s_gyro_filter.applied_sample_hz;
        if(!s_rate_supported || fabsf(deviation)>BMI088_GYRO_FILTER_RATE_TOLERANCE*s_gyro_filter.applied_sample_hz) {
            /* Three mutually consistent windows on the same side of the
             * design rate; a one-window delay/spike is not a new cadence. */
            if(!s_candidate_windows || fabsf(hz-s_candidate_hz)>BMI088_GYRO_FILTER_RATE_TOLERANCE*s_candidate_hz ||
               ((s_candidate_hz-s_gyro_filter.applied_sample_hz)*deviation<0)) {
                s_candidate_windows=0;s_candidate_cycles=0;s_candidate_hz=hz;
            }
            s_candidate_windows++;s_candidate_cycles+=s_rate_cycles;
            if(s_candidate_windows>=BMI088_GYRO_FILTER_RATE_WINDOWS) {
                float design_hz=(float)((double)SystemCoreClock*(double)(BMI088_GYRO_FILTER_RATE_WINDOW*s_candidate_windows)/(double)s_candidate_cycles);
                s_rate_supported=(uint8_t)BmiFilter_Design(BMI088_GYRO_FILTER_CUTOFF_HZ,design_hz);
                if(!s_rate_supported)BmiFilter_ResetHistory(BMI088_GYRO_FILTER_BAD_PARAMETERS,0);
                s_candidate_windows=0;s_candidate_cycles=0;
            }
        } else {s_candidate_windows=0;s_candidate_cycles=0;}
    }
    s_rate_intervals=0;s_rate_cycles=0;
}

const Bmi088GyroFilterState_t *Bmi088GyroFilter_Compute(const Bmi088Data_t *sample,
                                                     uint32_t completion_cycle)
{
    uint32_t start=DWT->CYCCNT,cycles=0,elapsed=0,duration;
    float result[3],notched[3];
    unsigned axis;
    if(!sample || !sample->initialized || !sample->valid || !sample->has_sample) {
        BmiFilter_ResetHistory(BMI088_GYRO_FILTER_SOURCE_INVALID,1);return &s_gyro_filter;
    }
    if(s_gyro_count_seen && sample->sample_count==s_gyro_count)return &s_gyro_filter;
    s_gyro_count=sample->sample_count;s_gyro_count_seen=1;
    for(axis=0;axis<3;axis++) {
        if(!BmiFilter_Finite(sample->gyro_rad_s[axis])) {
            s_gyro_filter.rejected_count++;BmiFilter_ResetHistory(BMI088_GYRO_FILTER_BAD_INPUT,1);return &s_gyro_filter;
        }
        if(fabsf(sample->gyro_rad_s[axis])>=BMI088_GYRO_VALID_LIMIT_RAD_S) {
            s_gyro_filter.rejected_count++;BmiFilter_ResetHistory(BMI088_GYRO_FILTER_OVERRANGE,1);return &s_gyro_filter;
        }
    }
    if(SystemCoreClock<1000000U) {
        s_gyro_filter.rejected_count++;BmiFilter_ResetHistory(BMI088_GYRO_FILTER_TIME_ERROR,1);return &s_gyro_filter;
    }
    if(s_stalled_cycle_seen && completion_cycle==s_stalled_cycle) {
        BmiFilter_ResetHistory(BMI088_GYRO_FILTER_TIME_ERROR,1);return &s_gyro_filter;
    }
    s_stalled_cycle_seen=0;
    if(s_gyro_clock_seen) {
        cycles=(uint32_t)(completion_cycle-s_gyro_cycle);
        elapsed=(uint32_t)(sample->sample_tick_ms-s_gyro_tick);
        if(elapsed>=BMI088_SAMPLE_TIMEOUT_MS || cycles>=SystemCoreClock/50U) {
            BmiFilter_ResetHistory(BMI088_GYRO_FILTER_TIME_ERROR,1);
        } else if(!cycles) {
            s_stalled_cycle=completion_cycle;s_stalled_cycle_seen=1;
            s_gyro_filter.rejected_count++;BmiFilter_ResetHistory(BMI088_GYRO_FILTER_TIME_ERROR,1);return &s_gyro_filter;
        } else BmiFilter_MeasureRate(cycles);
    }
    s_gyro_cycle=completion_cycle;s_gyro_tick=sample->sample_tick_ms;s_gyro_clock_seen=1;
    if(!s_rate_supported)return &s_gyro_filter;
    for(axis=0;axis<3;axis++) {
        const BmiGyroHistory *h=&s_gyro_history[axis];
        const BmiGyroHistory *nh=&s_notch_history[axis];
        const float input=sample->gyro_rad_s[axis];
        notched[axis]=s_gyro_filter.initialized ?
            s_gyro_filter.notch_b0*input+s_gyro_filter.notch_b1*nh->x1+s_gyro_filter.notch_b2*nh->x2-
            s_gyro_filter.notch_a1*nh->y1-s_gyro_filter.notch_a2*nh->y2 : input;
        result[axis]=s_gyro_filter.initialized ?
            s_gyro_filter.b0*notched[axis]+s_gyro_filter.b1*h->x1+s_gyro_filter.b2*h->x2-
            s_gyro_filter.a1*h->y1-s_gyro_filter.a2*h->y2 : input;
        if(!BmiFilter_Finite(notched[axis]) || !BmiFilter_Finite(result[axis])) {
            s_gyro_filter.rejected_count++;BmiFilter_ResetHistory(BMI088_GYRO_FILTER_NUMERICAL_ERROR,1);return &s_gyro_filter;
        }
    }
    /* Validate all axes before committing any history/output. */
    for(axis=0;axis<3;axis++) {
        BmiGyroHistory *h=&s_gyro_history[axis];
        BmiGyroHistory *nh=&s_notch_history[axis];
        if(!s_gyro_filter.initialized){
            h->x1=h->x2=h->y1=h->y2=result[axis];
            nh->x1=nh->x2=nh->y1=nh->y2=sample->gyro_rad_s[axis];
        }
        else {
            nh->x2=nh->x1;nh->x1=sample->gyro_rad_s[axis];nh->y2=nh->y1;nh->y1=notched[axis];
            h->x2=h->x1;h->x1=notched[axis];h->y2=h->y1;h->y1=result[axis];
        }
        s_gyro_filter.gyro_rad_s[axis]=result[axis];
    }
    s_gyro_filter.sample_count=sample->sample_count;s_gyro_filter.sample_tick_ms=sample->sample_tick_ms;
    s_gyro_filter.age_ms=0;s_gyro_filter.update_count++;
    s_gyro_filter.has_sample=s_gyro_filter.initialized=s_gyro_filter.valid=1;
    s_gyro_filter.error=BMI088_GYRO_FILTER_OK;
    duration=(uint32_t)(DWT->CYCCNT-start)/(SystemCoreClock/1000000U);
    if(duration>s_gyro_filter.max_compute_duration_us)s_gyro_filter.max_compute_duration_us=duration;
    return &s_gyro_filter;
}

void Bmi088GyroFilter_GetState(Bmi088GyroFilterState_t *state)
{
    Bmi088_GetSnapshotWithFilter(NULL,state,NULL);
}
