#include "anticogging.h"
#include "control.h"
#include "bsp_motor_record.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static uint32_t ticks;
uint32_t bsp_uart_millis(void)
{
    static uint32_t last, elapsed;
    elapsed+=(ticks-last)&0xffffff;last=ticks;
    return elapsed/1000; /* Independent monotonic clock across sample wrap. */
}
static float disturbance(float deg) { return .30f * sinf(deg * .01745329252f * 84); }
static float run(bool enabled, int direction)
{
    foc.state=FOC_IDLE; control_stop(); assert(control_zero());
    assert(cog_enable(enabled));
    float theta=359.9f, rpm=0, sum=0;
    control_step(ticks,theta); assert(control_speed(10)); foc.state=FOC_RUN;
    for(unsigned n=0;n<40000;++n) {
        ticks=(ticks+100)&0xffffff;
        float iq=control_iq_ref();
        rpm += 1000 * (direction * (iq-disturbance(theta)) - .002f*rpm) * .0001f;
        theta += 6 * rpm * .0001f;
        foc.iq=iq; control_step(ticks,theta);
        assert(fabsf(control_iq_ref())<=FOC_MOTOR_OUTER_CURRENT_MAX);
        if(n>=10000) sum+=(rpm-10)*(rpm-10);
    }
    control_stop(); foc.state=FOC_IDLE;
    assert(cog_applied()==0);
    return sqrtf(sum/30000);
}
static void light_shaft_capture(void)
{
    foc.state=FOC_IDLE; control_stop(); assert(control_zero());
    float theta=2, velocity=0, iq=0;
    control_step(ticks,theta); assert(control_cog_learn());
    for(unsigned point=1;point<=6;++point) {
        float target=.5f*point;
        assert(control_position(target));foc.state=FOC_RUN;
        float sum=0, square=0;
        for(unsigned n=0;n<18000;++n) {
            if(n%1000==0)assert(control_position(target));
            ticks=(ticks+100)&0xffffff;
            /* Independent light mechanical plant, 200 Hz current actuator,
             * 14-bit encoder quantisation; not an ideal Iq assignment. */
            iq += .11808862f*(control_iq_ref()-iq);
            velocity += 25000*(iq-disturbance(theta)-.0001f*velocity)*.0001f;
            theta += 6*velocity*.0001f;foc.iq=iq;
            float sample=roundf(theta/0.02197265625f)*0.02197265625f;
            control_step(ticks,sample);
            assert(control_fault()==0 && fabsf(control_iq_ref())<=2);
            if(n>=16000){float e=theta-2-target;sum+=e;square+=e*e;}
        }
        float error=sum/2000, rms=sqrtf(square/2000);
        printf("static capture %.1f deg: mean error %.4f, rms %.4f deg\n",target,error,rms);
        assert(fabsf(error)<.1f && rms<.1f);
    }
    control_stop();foc.state=FOC_IDLE;
}
int main(void)
{
    foc=(foc_t){.state=FOC_IDLE,.zero_ready=true,.calibrated=true,
        .calibration={1,1,7},.voltage_scale=1};
    cog_reset(); assert(!cog_enable(true)); assert(cog_begin());
    assert(!cog_point(1,.1f) && !cog_point(0,NAN) && !cog_point(0,2));
    assert(!cog_commit());
    for(unsigned i=0;i<COG_POINTS;++i) assert(cog_point(i,disturbance(i*.5f)+.2f));
    assert(cog_commit() && cog_valid()); assert(cog_gain(1));
    assert(cog_enable(true));
    assert(fabsf(cog_output(-.1f,0,.1f)-cog_output(359.9f,0,.1f))<1e-5f);
    assert(fabsf(cog_output(0,120,.1f))<1e-5f);
    assert(cog_enable(false));
    /* A failed partial replacement preserves the previous valid map. */
    float old=cog_table.map[1]; assert(cog_begin() && cog_point(0,.1f));
    cog_cancel(); assert(!cog_commit() && cog_valid() && cog_table.map[1]==old);
    static cog_record_t r;
    r=(cog_record_t){.version=1,.profile=(FOC_CALIBRATION_ID<<16)|7,.points=COG_POINTS,.table=cog_table,.magic=0x434f4731};
    r.checksum=cog_checksum(&r); assert(cog_record_valid(&r,&foc.calibration));
    r.table.map[10]+=.01f; assert(!cog_record_valid(&r,&foc.calibration));
    for(int dir=-1;dir<=1;dir+=2) {
        foc.calibration.direction=cog_table.calibration.direction=dir;
        float baseline=run(false,dir), compensated=run(true,dir);
        printf("direction %d: speed error rms %.4f -> %.4f RPM\n",dir,baseline,compensated);
        assert(compensated<baseline*.30f);
    }
    assert(cog_enable(true)); assert(control_torque(.1f)); foc.state=FOC_RUN;
    for(unsigned i=0;i<20;++i){ticks+=100;control_step(ticks,0);}
    assert(cog_applied()==0); control_stop();
    foc.state=FOC_IDLE; assert(cog_enable(false));
    assert(control_cog_learn() && !control_speed(10) && !control_torque(.1f));
    assert(control_position(100)); foc.state=FOC_RUN;
    uint32_t start=ticks;
    while(((ticks-start)&0xffffff)<210000) {
        ticks=(ticks+100)&0xffffff; control_step(ticks,0);
        assert(fabsf(control_iq_ref())<=2);
    }
    assert(control_fault()==FOC_UART); control_stop(); assert(!control_cog_learning());
    light_shaft_capture();
    assert(cog_enable(true)); assert(control_zero());control_step(ticks,2.5f);
    assert(control_speed(300));foc.state=FOC_RUN;
    for(unsigned n=0;n<20;++n){ticks+=100;control_step(ticks,2.5f);}
    assert(control_speed_reference()<60 && cog_applied()==0); /* High target, low transient reference. */
    control_stop();foc.state=FOC_IDLE;
    foc.calibration.zero+=.01f; assert(!cog_valid());
    assert(cog_output(20,10,.001f)==0);
    puts("PASS: wrap, binding, complete upload, cancel, CRC, mode exclusion and disturbed mechanical plant");
}
