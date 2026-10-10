#include "sensor_cal.h"
#include "control.h"
#include "bsp_motor_record.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
volatile uint32_t motor_sample_us;
volatile float motor_duty[3];
uint32_t bsp_uart_millis(void) {return motor_sample_us/1000u;}
static float wrapdeg(float x) {return x-floorf(x/360.0f)*360.0f;}
static void scan(float poles,int direction,int mode)
{
    assert(sensor_cal_start(10000u));float field=0;
    for(unsigned i=1;sensor_cal_active();++i) {
        float t=(float)i/10000.0f;
        float turn=t<3.5f?0:t<15.5f?(t-3.5f)/4:t<27.5f?(27.5f-t)/4:0;
        float lag=t>3.5f && t<15.5f?-.05f:t>15.5f && t<27.5f?.05f:0;
        float angle=359.5f+(float)direction*(turn*360.0f/poles+lag)+.001f*sinf(123.0f*t);
        if(mode==1)angle=359.5f; /* Stuck shaft. */
        if(mode==2 && t>22)angle+=15; /* Slip during reverse. */
        if(mode==3 && t>27.5f)angle+=2; /* Does not return. */
        if(mode==4)angle+=.6f*cosf(turn*6.2831853072f); /* Cogging biases the static hold, zero mean over scan turns. */
        (void)sensor_cal_step(angle,&field);
        assert(i<=280000u);
    }
    if((mode && mode!=4) || poles==7.5f){assert(!sensor_cal.valid);return;}
    assert(sensor_cal.valid && sensor_cal.phase==SENSOR_CAL_READY);
    assert(sensor_cal.candidate.pole_pairs==(unsigned)poles && sensor_cal.candidate.direction==direction);
    float expected=foc_wrap((float)direction*poles*359.5f*.01745329252f-15*.01745329252f);
    float error=foc_wrap(sensor_cal.candidate.zero-expected+3.1415926536f)-3.1415926536f;
    if(fabsf(error)>=.002f)fprintf(stderr,"poles=%g direction=%d zero error=%g\n",poles,direction,error);
    assert(fabsf(error)<.002f);
}
static void sample(float angle,float bus,float b,float c)
{
    motor_sample_us+=100;foc_step(wrapdeg(angle),bus,b,c,0);
}
int main(void)
{
    for(unsigned p=1;p<=64;p+=1){scan((float)p,1,0);scan((float)p,-1,0);}
    scan(7.5f,1,0);scan(7,1,1);scan(7,1,2);scan(7,1,3);
    scan(7,1,4);scan(7,-1,4);
    assert(sensor_cal_start(10000));sensor_cal_cancel();assert(!sensor_cal.valid);
    assert(sensor_cal_start(10000));sensor_cal_abort();assert(!sensor_cal.valid);
    assert(!sensor_cal_start(0));assert(sensor_cal_start(10000));
    float field;assert(!sensor_cal_step(NAN,&field));
    foc_calibration_t old={1.2f,1,7};foc_init(&old);
    for(unsigned i=0;i<FOC_OFFSET_WAIT_SAMPLES+2048u;++i)sample(359.5f,12,1.65f,1.65f);
    assert(foc_autocalibrate(1.5f,12));
    while(foc.state==FOC_PRECHARGE)sample(359.5f,12,1.65f,1.65f);
    for(unsigned i=2;i<=280000u;++i){
        float t=(float)i/10000,turn=t<3.5f?0:t<15.5f?(t-3.5f)/4:t<27.5f?(27.5f-t)/4:0;
        sample(359.5f-turn*360/11,12,1.65f,1.65f);
    }
    assert(foc.state==FOC_IDLE && sensor_cal.valid);
    assert(foc.calibration.pole_pairs==7 && foc.calibration.zero==old.zero);
    assert(foc_apply_calibration());assert(foc.calibration.pole_pairs==11 && foc.calibration.direction==-1);
    /* Runtime Park conversion must use the detected count, not the profile. */
    sample(12,12,1.65f,1.65f);
    float expected=foc_wrap(-11*12*.01745329252f-foc.calibration.zero)*57.29577951f;
    assert(fabsf(foc.electrical_deg-expected)<.002f);
    foc.rpm=0;assert(foc_autocalibrate(1.5f,12));sample(12,25,1.65f,1.65f);
    assert(!sensor_cal.valid && foc.fault==FOC_BUS);
    assert(foc.calibration.pole_pairs==11); /* Failed scan cannot replace calibration. */
    puts("PASS: 1..64 pole pairs, both directions, wrap, slip/stuck/noninteger/return rejection, runtime mapping and bus abort");
}
