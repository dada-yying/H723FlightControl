/* Exercise production single-precision samples; inspect actual designed poles.
 * No recorded log output is represented as a new on-board measurement. */
#define BMI_FILTER_FIXTURE_ONLY
#include "test_bmi088_gyro_filter.c"

typedef struct {double gain,phase;} Response;
static Response response(const double b[3],const double a[2],double f,double hz)
{
    const double w=2*3.14159265358979323846*f/hz;
    const double nr=b[0]+b[1]*cos(w)+b[2]*cos(2*w),ni=-b[1]*sin(w)-b[2]*sin(2*w);
    const double dr=1+a[0]*cos(w)+a[1]*cos(2*w),di=-a[0]*sin(w)-a[1]*sin(2*w);
    Response r;
    r.gain=sqrt((nr*nr+ni*ni)/(dr*dr+di*di));
    r.phase=atan2(ni*dr-nr*di,nr*dr+ni*di);
    return r;
}
static double crossing(const double b[3],const double a[2],double hz,int upper)
{
    double lo=upper?120:0,hi=upper?hz/2:120;
    unsigned i;
    for(i=0;i<60;i++) {
        double mid=(lo+hi)/2,g=response(b,a,mid,hz).gain;
        if((g<sqrt(.5))==upper)lo=mid;else hi=mid;
    }
    return (lo+hi)/2;
}
static void print_response(FILE *out,const char *stage,const double b[3],const double a[2],
                           double f,double hz)
{
    Response r=response(b,a,f,hz),left=response(b,a,f-.001,hz),right=response(b,a,f+.001,hz);
    double slope=atan2(sin(right.phase-left.phase),cos(right.phase-left.phase))/.002;
    if(f==120 && !strcmp(stage,"notch")) {
        /* Phase/delay at the transfer zero is undefined. */
        fprintf(out,"%.0f,%s,%.3f,%.6f,NA,NA,NA\n",hz,stage,f,
            20*log10(fmax(r.gain,1e-15)));
        return;
    }
    fprintf(out,"%.0f,%s,%.3f,%.6f,%.6f,%.6f,%.6f\n",hz,stage,f,
        20*log10(fmax(r.gain,1e-15)),r.phase*180/3.14159265358979323846,
        -r.phase/(2*3.14159265358979323846*f)*1000,
        -slope/(2*3.14159265358979323846)*1000);
}
int main(int argc,char **argv)
{
    const unsigned rates[]={2000,1000,500};
    const double frequencies[]={5,10,20,50,100,110,120,130,140,200};
    unsigned rate_index,j,i,axis;
    FILE *out=argc>1?fopen(argv[1],"w"):stdout;
    assert(out);
    fprintf(out,"sample_hz,stage,frequency_hz,gain_db,phase_deg,phase_delay_ms,group_delay_ms\n");
    for(rate_index=0;rate_index<3;rate_index++) {
        unsigned hz=rates[rate_index],count=hz*4;
        double nb[3],na[2],lb[3],la[2],lower,upper;
        filter_reset();filter_feed(1000000U/hz,0,0,0);
        for(i=0;i<4000;i++)filter_feed(1000000U/hz,0,0,0);
        assert(s_gyro_filter.applied_sample_hz==hz);
        nb[0]=s_gyro_filter.notch_b0;nb[1]=s_gyro_filter.notch_b1;nb[2]=s_gyro_filter.notch_b2;
        na[0]=s_gyro_filter.notch_a1;na[1]=s_gyro_filter.notch_a2;
        lb[0]=s_gyro_filter.b0;lb[1]=s_gyro_filter.b1;lb[2]=s_gyro_filter.b2;
        la[0]=s_gyro_filter.a1;la[1]=s_gyro_filter.a2;
        lower=crossing(nb,na,hz,0);upper=crossing(nb,na,hz,1);
        assert(fabs(upper-lower-40)<.01);
        assert(20*log10(response(nb,na,120,hz).gain)<-85);
        assert(fabs(20*log10(response(lb,la,50,hz).gain)+3.01029995664)<.001);
        printf("%u Hz design: notch half-power %.6f..%.6f Hz, BW %.6f Hz, center %.2f dB; LPF50 %.6f dB\n",
            hz,lower,upper,upper-lower,20*log10(response(nb,na,120,hz).gain),
            20*log10(response(lb,la,50,hz).gain));
        for(j=0;j<sizeof(frequencies)/sizeof(frequencies[0]);j++) {
            const double f=frequencies[j],pi=3.14159265358979323846;
            double ns=0,nc=0,cs=0,cc=0;
            Response nr=response(nb,na,f,hz),lr=response(lb,la,f,hz);
            /* Restart both stages and settle for four seconds at this rate. */
            Bmi088GyroFilter_Invalidate();filter_feed(1000000U/hz,0,0,0);
            for(i=0;i<2*count;i++) {
                double angle=2*pi*f*i/hz;
                float x=(float)sin(angle);
                const Bmi088GyroFilterState_t *r=filter_feed(1000000U/hz,x,-.5f*x,.25f*x);
                assert(r->valid);
                /* Axis scaling must survive both independently stored stages. */
                assert(fabsf(r->gyro_rad_s[1]+.5f*r->gyro_rad_s[0])<1e-6f);
                assert(fabsf(r->gyro_rad_s[2]-.25f*r->gyro_rad_s[0])<1e-6f);
                if(i>=count) {
                    ns+=s_notch_history[0].y1*sin(angle);nc+=s_notch_history[0].y1*cos(angle);
                    cs+=r->gyro_rad_s[0]*sin(angle);cc+=r->gyro_rad_s[0]*cos(angle);
                }
            }
            { double ng=hypot(ns,nc)*2/count,cg=hypot(cs,cc)*2/count;
              double phase=atan2(cc,cs),expected=nr.phase+lr.phase;
              if(f==120) {assert(ng<.0001 && cg<.0001);}
              else {
                double np=atan2(nc,ns),lp=phase-np;
                assert(fabs(20*log10(ng/notch_gain(f,hz)))<.1);
                assert(fabs(20*log10(cg/(nr.gain*lr.gain)))<.1);
                /* The LPF's own response is the cascade / measured notch. */
                assert(fabs(20*log10((cg/ng)/lr.gain))<.1);
                assert(fabs(atan2(sin(phase-expected),cos(phase-expected)))<.002);
                assert(fabs(atan2(sin(np-nr.phase),cos(np-nr.phase)))<.002);
                assert(fabs(atan2(sin(lp-lr.phase),cos(lp-lr.phase)))<.002);
              }
            }
            print_response(out,"notch",nb,na,f,hz);print_response(out,"lowpass",lb,la,f,hz);
            /* Cascade phase/group delays add; report it without phase wrapping. */
            if(f!=120) {
                double df=.001;
                Response nl=response(nb,na,f-df,hz),nh=response(nb,na,f+df,hz);
                Response ll=response(lb,la,f-df,hz),lh=response(lb,la,f+df,hz);
                double slope=atan2(sin(nh.phase-nl.phase),cos(nh.phase-nl.phase))+
                    atan2(sin(lh.phase-ll.phase),cos(lh.phase-ll.phase));
                fprintf(out,"%u,cascade,%.3f,%.6f,%.6f,%.6f,%.6f\n",hz,f,
                    20*log10(nr.gain*lr.gain),(nr.phase+lr.phase)*180/pi,
                    -(nr.phase+lr.phase)/(2*pi*f)*1000,-slope/(2*df*2*pi)*1000);
            } else fprintf(out,"%u,cascade,120,%.6f,NA,NA,NA\n",hz,
                20*log10(fmax(nr.gain*lr.gain,1e-15)));
        }
        /* All stages' constant histories remain initialized across a confirmed
         * rate change; no coefficient update from ordinary scheduling jitter. */
        for(axis=0;axis<3;axis++)assert(isfinite(s_gyro_history[axis].y1));
    }
    if(out!=stdout)fclose(out);
    puts("PASS measured notch/LPF/cascade amplitude and phase; exact digital bandwidth at 2k/1k/500Hz");
    return 0;
}
