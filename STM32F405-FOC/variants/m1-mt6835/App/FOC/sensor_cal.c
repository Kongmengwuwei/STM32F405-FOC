#include "sensor_cal.h"
#include <math.h>
#define TURN 6.2831853072f
#define START (15.0f * 0.01745329252f)
sensor_cal_status_t sensor_cal;
static uint32_t hz, tick;
static float origin, end_forward, sum, mean, minimum, maximum;
typedef struct { unsigned n; float x, y, xx, xy, yy; } fit_t;
static fit_t fits[2];
bool sensor_cal_active(void) { return sensor_cal.phase>=SENSOR_CAL_HOLD && sensor_cal.phase<=SENSOR_CAL_SAMPLE; }
bool sensor_cal_start(uint32_t sample_hz)
{
    if (sensor_cal_active() || sample_hz<1000u || sample_hz>40000u || sample_hz%1000u) return false;
    /* This entry is IRQ-locked. Explicit typed volatile stores avoid the
     * byte-wise nano-libc memset delaying the PWM update interrupt. */
    volatile sensor_cal_status_t *st=&sensor_cal;
    st->phase=st->error=0;st->valid=false;
    st->candidate.zero=0;st->candidate.direction=0;st->candidate.pole_pairs=0;
    st->forward_pairs=st->reverse_pairs=st->residual_deg=st->return_deg=0;
    for(unsigned i=0;i<2u;++i){
        volatile fit_t *f=&fits[i];f->n=0;
        f->x=f->y=f->xx=f->xy=f->yy=0;
    }
    hz=sample_hz;tick=0;sum=mean=0; sensor_cal.phase=SENSOR_CAL_HOLD;return true;
}
void sensor_cal_cancel(void) { if(sensor_cal_active()){sensor_cal.valid=false;sensor_cal.phase=SENSOR_CAL_CANCELLED;} }
void sensor_cal_abort(void) { sensor_cal.valid=false;sensor_cal.error=SENSOR_CAL_ABORT;sensor_cal.phase=SENSOR_CAL_FAILED; }
static bool fail(uint32_t error) { sensor_cal.valid=false;sensor_cal.error=error;sensor_cal.phase=SENSOR_CAL_FAILED;return false; }
static void add(fit_t *f,float x,float y)
{
    /* Welford covariance avoids cancellation for multi-turn absolute angles. */
    float dx=x-f->x,dy=y-f->y; ++f->n;
    f->x+=dx/(float)f->n; f->y+=dy/(float)f->n;
    f->xx+=dx*(x-f->x);f->xy+=dx*(y-f->y);f->yy+=dy*(y-f->y);
}
static float slope(const fit_t *f) { return f->xx>0 ? f->xy/f->xx : 0; }
static float residual(const fit_t *f) { return sqrtf(fmaxf(0,f->yy-slope(f)*f->xy)/(float)f->n); }
bool sensor_cal_step(float angle,float *field)
{
    if (!sensor_cal_active()) return false;
    if (!isfinite(angle)) return fail(SENSOR_CAL_INPUT);
    ++tick;float t=(float)tick/(float)hz;
    *field=START;
    if(t<=3.5f) { sensor_cal.phase=SENSOR_CAL_HOLD;origin=angle;return true; }
    if(t<=15.5f) {
        sensor_cal.phase=SENSOR_CAL_FORWARD;*field+=TURN*(t-3.5f)/4.0f;
        if(t>=7.5f && tick%(hz/1000u)==0u) add(&fits[0],90.0f*(t-7.5f),angle-origin);
        end_forward=angle;return true;
    }
    if(t<=27.5f) {
        sensor_cal.phase=SENSOR_CAL_REVERSE;*field+=TURN*(27.5f-t)/4.0f;
        if(t>=19.5f && tick%(hz/1000u)==0u) add(&fits[1],90.0f*(t-19.5f),angle-end_forward);
        return true;
    }
    if(t<=27.8f) { sensor_cal.phase=SENSOR_CAL_SETTLE;minimum=maximum=angle;return true; }
    sensor_cal.phase=SENSOR_CAL_SAMPLE;
    minimum=fminf(minimum,angle);maximum=fmaxf(maximum,angle);
    ++sum;mean+=(angle-mean)/sum;
    if(t<28.0f) return true;
    float sf=slope(&fits[0]),sr=slope(&fits[1]);
    if(fits[0].n<1000u || fits[1].n<1000u || fabsf(sf)<1.0f/65.0f || fabsf(sr)<1.0f/65.0f)
        return fail(SENSOR_CAL_NO_MOTION);
    if(sf*sr>=0) return fail(SENSOR_CAL_DIRECTION);
    sensor_cal.forward_pairs=1.0f/fabsf(sf);sensor_cal.reverse_pairs=1.0f/fabsf(sr);
    float estimate=.5f*(sensor_cal.forward_pairs+sensor_cal.reverse_pairs);
    unsigned poles=(unsigned)(estimate+.5f);
    if(poles<1u || poles>64u || fabsf(sensor_cal.forward_pairs-(float)poles)>.15f ||
       fabsf(sensor_cal.reverse_pairs-(float)poles)>.15f) return fail(SENSOR_CAL_INTEGER);
    sensor_cal.residual_deg=fmaxf(residual(&fits[0]),residual(&fits[1]));
    sensor_cal.return_deg=mean-origin;
    /* Bound electrical RMS to 10 degrees: allow cogging at low excitation,
     * while integer agreement and return/stationary checks stay independent. */
    if(sensor_cal.residual_deg*(float)poles>10.0f) return fail(SENSOR_CAL_FOLLOWING);
    if(fabsf(sensor_cal.return_deg)*(float)poles>7.0f || (maximum-minimum)*(float)poles>3.0f)
        return fail(SENSOR_CAL_RETURN);
    sensor_cal.candidate.pole_pairs=poles;
    sensor_cal.candidate.direction=sf>0 ? 1 : -1;
    /* Mean electrical offsets over two turns in each direction. Opposite
     * scan lag cancels; a single static hold can be biased by cogging/stiction.
     * x is electrical degrees after the discarded first turn, y is relative
     * mechanical degrees; reconstruct each segment's absolute field angle. */
    float scale=(float)sensor_cal.candidate.direction*(float)poles*0.01745329252f;
    float zf=scale*(origin+fits[0].y)-START-TURN-fits[0].x*0.01745329252f;
    float zr=scale*(end_forward+fits[1].y)-START-2.0f*TURN+fits[1].x*0.01745329252f;
    float difference=zr-zf;
    difference-=floorf((difference+TURN*.5f)/TURN)*TURN;
    if(fabsf(difference)>TURN/6.0f)return fail(SENSOR_CAL_FOLLOWING);
    float zero=zf+.5f*difference;
    sensor_cal.candidate.zero=zero-floorf(zero/TURN)*TURN;
    sensor_cal.phase=SENSOR_CAL_READY;sensor_cal.valid=true;return false;
}
