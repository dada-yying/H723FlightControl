/* Real notch/LPF/acquisition/AHRS/control/logger; double reference below. */
#define main blackbox_fixture_main
#include "test_blackbox.c"
#undef main

static Bmi088Data_t filter_input;
static uint64_t filter_us;
static void filter_reset(void)
{
    reset();filter_us=0;test_tick=0;test_dwt.CYCCNT=0;
    memset(&filter_input,0,sizeof(filter_input));
    filter_input.initialized=filter_input.valid=filter_input.has_sample=1;
    filter_input.accel_m_s2[2]=9.80665f;
}
static const Bmi088GyroFilterState_t *filter_feed(unsigned us, float x, float y, float z)
{
    const Bmi088GyroFilterState_t *state;
    Bmi088Data_t before;
    filter_input.sample_count++;filter_us+=us;test_tick=(uint32_t)(filter_us/1000U);
    test_dwt.CYCCNT+=us*480U;filter_input.sample_tick_ms=test_tick;
    filter_input.gyro_rad_s[0]=x;filter_input.gyro_rad_s[1]=y;filter_input.gyro_rad_s[2]=z;
    before=filter_input;state=Bmi088GyroFilter_Compute(&filter_input,test_dwt.CYCCNT);
    assert(!memcmp(&before,&filter_input,sizeof(before)));
    g_bmi088_data=filter_input;g_bmi088_gyro_filter=*state;
    return state;
}

typedef struct {double b[3],a[2],x[3][2],y[3][2];} DoubleFilter;
static void reference_init(DoubleFilter *f,double cutoff,double hz)
{
    const double omega=2.0*3.14159265358979323846*cutoff/hz;
    const double cosine=cos(omega),alpha=sin(omega)/sqrt(2.0),norm=1.0/(1.0+alpha);
    memset(f,0,sizeof(*f));
    f->b[0]=(1.0-cosine)*0.5*norm;f->b[1]=(1.0-cosine)*norm;f->b[2]=f->b[0];
    f->a[0]=-2.0*cosine*norm;f->a[1]=(1.0-alpha)*norm;
}
static void reference_notch_init(DoubleFilter *f,double hz)
{
    const double t=tan(3.14159265358979323846*40.0/hz);
    memset(f,0,sizeof(*f));
    f->a[1]=(1-t)/(1+t);
    f->a[0]=-(1+f->a[1])*cos(2*3.14159265358979323846*120.0/hz);
    f->b[0]=(1+f->a[1])*.5;f->b[1]=f->a[0];f->b[2]=f->b[0];
}
static double notch_gain(double f,double hz)
{
    double w=2*3.14159265358979323846*f/hz;
    double d=cos(w)-cos(2*3.14159265358979323846*120/hz);
    double q=tan(3.14159265358979323846*40/hz)*sin(w);
    return fabs(d)/sqrt(d*d+q*q);
}
static double reference_step(DoubleFilter *f,unsigned axis,double input)
{
    double out=f->b[0]*input+f->b[1]*f->x[axis][0]+f->b[2]*f->x[axis][1]-
               f->a[0]*f->y[axis][0]-f->a[1]*f->y[axis][1];
    f->x[axis][1]=f->x[axis][0];f->x[axis][0]=input;
    f->y[axis][1]=f->y[axis][0];f->y[axis][0]=out;return out;
}

#ifndef BMI_FILTER_FIXTURE_ONLY
int main(void)
{
    unsigned i,axis,updates,resets;
    Bmi088GyroFilterState_t before,state;
    Bmi088Data_t raw;
    const Bmi088GyroFilterState_t *result;
    DoubleFilter reference,notch;
    filter_reset();Bmi088GyroFilter_GetState(&state);
    assert(!state.valid && state.age_ms==UINT32_MAX && state.cutoff_hz==50 && state.applied_sample_hz==2000);
    assert(state.notch_center_hz==120 && state.notch_bandwidth_hz==40);
    result=filter_feed(500,.25f,-.5f,1.0f);
    assert(result->valid && result->update_count==1);
    assert(result->gyro_rad_s[0]==.25f && result->gyro_rad_s[1]==-.5f && result->gyro_rad_s[2]==1.0f);
    for(i=0;i<4000;i++)filter_feed(500,.25f,-.5f,1.0f);
    for(axis=0;axis<3;axis++)assert(fabsf(s_gyro_filter.gyro_rad_s[axis]-filter_input.gyro_rad_s[axis])<2e-5f);
    before=s_gyro_filter;
    for(i=0;i<100;i++)Bmi088GyroFilter_Compute(&filter_input,test_dwt.CYCCNT+500U*480U);
    assert(!memcmp(&before,&s_gyro_filter,sizeof(before)));
    FlightControl_ResetController();
    assert(!memcmp(&before,&s_gyro_filter,sizeof(before)));
    Bmi088_GetSnapshotWithFilter(&raw,&state,NULL);
    assert(state.valid && raw.sample_count==state.sample_count && raw.sample_tick_ms==state.sample_tick_ms);
    assert(!memcmp(raw.gyro_rad_s,filter_input.gyro_rad_s,sizeof(raw.gyro_rad_s)));

    filter_reset();filter_feed(500,0,0,0);reference_init(&reference,50,2000);reference_notch_init(&notch,2000);
    for(i=0;i<8000;i++) {
        const float x=i<3000 ? 1.0f : -.5f;
        const float y=i<4000 ? -.75f : .25f;
        const float z=i&1U ? .5f : -.5f;
        const float input[3]={x,y,z};result=filter_feed(500,x,y,z);
        for(axis=0;axis<3;axis++)assert(fabs(result->gyro_rad_s[axis]-
            reference_step(&reference,axis,reference_step(&notch,axis,input[axis])))<3e-5);
    }
    /* Independent axes + exact analytic digital frequency response. */
    { unsigned rate_index;
      const unsigned rates[]={2000,1000,500};
      for(rate_index=0;rate_index<3;rate_index++) {
      const unsigned hz=rates[rate_index];
      const double frequencies[3]={5,50,200};double sine_sum[3]={0},cosine_sum[3]={0};
      filter_reset();
      for(i=0;i<8000;i++) {
        float in[3];
        for(axis=0;axis<3;axis++)in[axis]=(float)sin(2*3.14159265358979323846*frequencies[axis]*i/hz);
        result=filter_feed(1000000U/hz,in[0],in[1],in[2]);
        if(i>=4000)for(axis=0;axis<3;axis++) {
            const double angle=2*3.14159265358979323846*frequencies[axis]*i/hz;
            sine_sum[axis]+=result->gyro_rad_s[axis]*sin(angle);
            cosine_sum[axis]+=result->gyro_rad_s[axis]*cos(angle);
        }
      }
      for(axis=0;axis<3;axis++) {
        double gain=sqrt(sine_sum[axis]*sine_sum[axis]+cosine_sum[axis]*cosine_sum[axis])*2/4000;
        double ratio=tan(3.14159265358979323846*frequencies[axis]/hz)/tan(3.14159265358979323846*50/hz);
        double expected=notch_gain(frequencies[axis],hz)/sqrt(1+pow(ratio,4));
        assert(fabs(20*log10(gain/expected))<.1);
        printf("Cascade %.0f Hz @ %u Hz: %.5f dB (reference %.5f dB)\n",frequencies[axis],hz,20*log10(gain),20*log10(expected));
      }
      assert(s_gyro_filter.applied_sample_hz==hz);
      }
    }
    filter_reset();filter_feed(500,0,0,0);
    for(i=0;i<300;i++){result=filter_feed(500,1,0,0);assert(result->gyro_rad_s[1]==0 && result->gyro_rad_s[2]==0);}
    for(i=0;i<300;i++){result=filter_feed(500,0,1,0);assert(result->gyro_rad_s[2]==0);}

    /* Ordinary jitter and one changed window cannot modulate coefficients. */
    filter_reset();filter_feed(500,.1f,-.2f,.3f);
    for(i=0;i<2000;i++)filter_feed(i%3==0?250:i%3==1?500:750,.1f,-.2f,.3f);
    assert(s_gyro_filter.coefficient_update_count==1);
    for(i=0;i<128;i++)filter_feed(1000,.1f,-.2f,.3f);
    for(i=0;i<256;i++)filter_feed(500,.1f,-.2f,.3f);
    assert(s_gyro_filter.coefficient_update_count==1);
    filter_reset();filter_feed(500,.1f,-.2f,.3f);resets=s_gyro_filter.reset_count;
    for(i=0;i<383;i++)filter_feed(1000,.1f,-.2f,.3f);
    assert(s_gyro_filter.coefficient_update_count==1);
    filter_feed(1000,.1f,-.2f,.3f);
    assert(s_gyro_filter.coefficient_update_count==2 && s_gyro_filter.applied_sample_hz==1000);
    assert(s_gyro_filter.reset_count==resets && fabsf(s_gyro_filter.gyro_rad_s[0]-.1f)<2e-5f);
    for(i=0;i<384;i++)filter_feed(500,.1f,-.2f,.3f);
    assert(s_gyro_filter.applied_sample_hz==2000);
    /* Unsupported Nyquist regime is invalid, never a silently bypassed LPF. */
    for(i=0;i<128;i++)filter_feed(10000,.1f,-.2f,.3f);
    assert(!s_gyro_filter.valid && s_gyro_filter.error==BMI088_GYRO_FILTER_BAD_PARAMETERS);
    for(i=0;i<384;i++)filter_feed(500,.2f,-.3f,.4f);
    assert(s_gyro_filter.valid && s_gyro_filter.gyro_rad_s[0]==.2f);

    filter_reset();filter_feed(500,1,2,3);test_tick+=20;
    Bmi088GyroFilter_GetState(&state);assert(!state.valid && state.age_ms==20);
    Bmi088_OnSamplingTimeout();assert(!s_gyro_filter.initialized);
    result=filter_feed(20000,-1,-2,-3);assert(result->valid && result->gyro_rad_s[0]==-1);
    updates=result->update_count;Bmi088GyroFilter_Invalidate();
    result=Bmi088GyroFilter_Compute(&filter_input,test_dwt.CYCCNT);
    assert(!result->valid && result->update_count==updates); /* Old cache cannot restore. */
    result=filter_feed(500,.2f,.3f,.4f);assert(result->valid && result->gyro_rad_s[2]==.4f);
    { const float bad[]={NAN,INFINITY,-INFINITY,40};
      for(i=0;i<4;i++)for(axis=0;axis<3;axis++) {
        float in[3]={1,2,3};before=s_gyro_filter;in[axis]=bad[i];
        result=filter_feed(500,in[0],in[1],in[2]);assert(!result->valid);
        assert(!memcmp(before.gyro_rad_s,result->gyro_rad_s,12));
        result=filter_feed(500,.2f,.3f,.4f);assert(result->valid && result->gyro_rad_s[axis]==filter_input.gyro_rad_s[axis]);
      }
    }
    before=s_gyro_filter;
    { const float bad[]={0,-1,NAN,INFINITY};
      for(i=0;i<4;i++) {assert(!BmiFilter_Design(bad[i],2000));assert(!BmiFilter_Design(50,bad[i]));}
      for(i=0;i<4;i++) {float c[5];
        assert(!BmiFilter_NotchDesign(bad[i],40,2000,c));
        assert(!BmiFilter_NotchDesign(120,bad[i],2000,c));
        assert(!BmiFilter_NotchDesign(120,40,bad[i],c));
      }
      assert(!BmiFilter_Design(900,2000) && !BmiFilter_Design(50,100));
      assert(!memcmp(&before,&s_gyro_filter,sizeof(before)));
    }
    { const double cutoffs[]={40,50,60};
      for(i=0;i<3;i++) {
          assert(BmiFilter_Design((float)cutoffs[i],2000));reference_init(&reference,cutoffs[i],2000);
          assert(fabs(s_gyro_filter.b0-reference.b[0])<1e-6 && fabs(s_gyro_filter.a1-reference.a[0])<1e-6 &&
                 fabs(s_gyro_filter.a2-reference.a[1])<1e-6);
      }
      assert(BmiFilter_Design(50,2000));
    }
    result=filter_feed(0,.1f,.2f,.3f);assert(!result->valid);
    result=filter_feed(0,.1f,.2f,.3f);assert(!result->valid); /* Stopped DWT cannot alternate valid/invalid. */
    result=filter_feed(500,.1f,.2f,.3f);assert(result->valid && result->gyro_rad_s[0]==.1f);
    { uint32_t clock=SystemCoreClock;SystemCoreClock=0;result=filter_feed(500,1,2,3);assert(!result->valid);SystemCoreClock=clock; }
    result=filter_feed(500,1,2,3);assert(result->valid && result->gyro_rad_s[0]==1);
    before=s_gyro_filter;s_gyro_history[1].y1=NAN;
    result=filter_feed(500,1,2,3);
    assert(!result->valid && result->error==BMI088_GYRO_FILTER_NUMERICAL_ERROR);
    assert(!memcmp(before.gyro_rad_s,result->gyro_rad_s,12));
    result=filter_feed(500,1,2,3);assert(result->valid && result->gyro_rad_s[1]==2);
    before=s_gyro_filter;s_notch_history[2].y1=INFINITY;
    { BmiGyroHistory lowpass_before[3];memcpy(lowpass_before,s_gyro_history,sizeof(lowpass_before));
      result=filter_feed(500,1,2,3);
      assert(!result->valid && result->error==BMI088_GYRO_FILTER_NUMERICAL_ERROR);
      assert(!memcmp(before.gyro_rad_s,result->gyro_rad_s,12));
      assert(!memcmp(lowpass_before,s_gyro_history,sizeof(lowpass_before)));
    }
    result=filter_feed(500,1,2,3);assert(result->valid && result->gyro_rad_s[2]==3);
    filter_input.sample_count=s_gyro_count=UINT32_MAX;test_dwt.CYCCNT=s_gyro_cycle=UINT32_MAX-100U;
    filter_us=(uint64_t)(UINT32_MAX-1U)*1000U;s_gyro_tick=filter_input.sample_tick_ms=(uint32_t)(filter_us/1000U);
    filter_feed(500,1,2,3);filter_feed(500,1,2,3);filter_feed(500,1,2,3);filter_feed(500,1,2,3);
    assert(s_gyro_filter.valid && filter_input.sample_count==3 && filter_input.sample_tick_ms==0);

    /* AHRS consumes filtered (not raw) rates, but retains raw saturation gate. */
    filter_reset();
    for(i=0;i<120;i++) {
        const Bmi088AttitudeState_t *a;
        result=filter_feed(500,0,0,0);a=Bmi088Attitude_Compute(&filter_input,result,test_dwt.CYCCNT);
        g_bmi088_attitude_state=*a;
    }
    assert(g_bmi088_attitude_state.valid);
    result=filter_feed(500,0,0,-1);
    { const Bmi088AttitudeState_t *a=Bmi088Attitude_Compute(&filter_input,result,test_dwt.CYCCNT);
      double expected=result->gyro_rad_s[2]*.0005; /* RFU +Z, no sign inversion. */
      assert(fabs(a->yaw_rad-expected)<1e-6 && fabs(a->yaw_rad)<.00005f);
      assert(filter_input.gyro_rad_s[2]==-1 && result->sample_count==filter_input.sample_count);
    }
    { Bmi088GyroFilterState_t invalid=*result;invalid.valid=0;
      assert(!Bmi088Attitude_Compute(&filter_input,&invalid,test_dwt.CYCCNT)->valid);
    }
    result=filter_feed(500,0,0,-40);
    assert(!result->valid && !Bmi088Attitude_Compute(&filter_input,result,test_dwt.CYCCNT)->valid);
    /* Real SPI publication: calibrated raw, LPF and AHRS metadata agree. */
    { Bmi088Data_t sensor;
      Bmi088GyroFilterState_t filtered;
      Bmi088AttitudeState_t attitude;
      reset();
      /* The numerical AHRS suite separately emulates registers/IDs. Here an
       * intentionally mismatched filter cannot be promoted by the snapshot. */
      seed_bmi();g_bmi088_gyro_filter.sample_count--;
      Bmi088_GetSnapshotWithFilter(&sensor,&filtered,&attitude);
      assert(sensor.valid && !filtered.valid && !attitude.valid);
    }
    puts("PASS BMI088 notch->Butterworth: startup/steps/double reference/response/axes, fresh-only, jitter/rate, invalid input/parameters/state, recovery/wrap, raw/PID isolation and filtered AHRS");
    return 0;
}
#endif
