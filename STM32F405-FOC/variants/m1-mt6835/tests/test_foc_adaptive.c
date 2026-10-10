#include "foc.h"
#include "control.h"
#include "bsp_motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
volatile float motor_duty[3];
volatile uint32_t motor_sample_us;
uint32_t bsp_uart_millis(void) { return motor_sample_us/1000u; }
static float iq;
static void sample(float bus,bool model)
{
    /* Stationary theta=0. q current is beta; voltage-driven physical plant. */
    if(model)iq=iq*expf(-.2f/(.0005f*FOC_SAMPLE_HZ))+
        foc.uq/.2f*(1-expf(-.2f/(.0005f*FOC_SAMPLE_HZ)));
    foc_step(0,bus,1.65f+.8660254038f*iq/FOC_CURRENT_A_PER_V,
                     1.65f-.8660254038f*iq/FOC_CURRENT_A_PER_V,0);
    for(unsigned n=0;n<3;++n)motor_duty[n]=foc.duty[n];
    motor_sample_us+=1000000u/FOC_SAMPLE_HZ;foc_outer_step();
}
int main(void)
{
    foc_calibration_t cal={0,1,FOC_POLE_PAIRS};foc_init(&cal);
    for(unsigned n=0;n<5000;++n)sample(12,false);
    assert(foc.state==FOC_IDLE && foc.zero_ready);
    motor_parameters_t p={.2f,.0003f,.0005f,0,0,0,200};
    p.kp_d=6.2831853072f*p.bandwidth_hz*p.ld;p.kp_q=6.2831853072f*p.bandwidth_hz*p.lq;
    p.ki=6.2831853072f*p.bandwidth_hz*p.resistance;
    assert(foc_apply_parameters(&p) && foc_parameters_active);
    assert(foc_rollback_parameters() && !foc_parameters_active);
    assert(!foc_rollback_parameters());
    assert(foc_apply_parameters(&p));
    motor_parameters_t next=p;next.bandwidth_hz=100;next.kp_d*=.5f;next.kp_q*=.5f;next.ki*=.5f;
    assert(foc_apply_parameters(&next));
    assert(foc_rollback_parameters() && foc_parameters_active && foc_parameters.ki==p.ki);
    assert(control_torque(.3f));
    assert(!foc_rollback_parameters());
    for(unsigned n=0;n<4000;++n)sample(12,true);
    assert(!foc_default_parameters());
    const float buses[]={12,18,24,12};
    for(unsigned b=0;b<4;++b) {
        for(unsigned n=0;n<3000;++n) {
            sample(buses[b],true);
            assert(foc.state==FOC_RUN && isfinite(foc.ud) && isfinite(foc.uq) && foc_window(foc.duty));
        }
        assert(fabsf(iq-.3f)<.003f);
    }
    assert(control_torque(-.3f));for(unsigned n=0;n<4000;++n)sample(18,true);assert(fabsf(iq+.3f)<.003f);
    foc_stop();assert(foc_parameters_active && foc_default_parameters() && !foc_parameters_active);
    assert(!foc_identify(NAN,12) && !foc_identify(1.5f,30));
    assert(foc_identify(1.5f,12));assert(!foc_current(.1f) && !foc_apply_parameters(&p));
    sample(12,false);float probe_angle=foc.electrical_deg;
    foc_step(.2f,12,1.65f,1.65f,0);
    assert(fabsf(foc.electrical_deg-probe_angle)<.001f); /* Probe does not chase the moving encoder. */
    foc_stop();assert(!foc_identifying() && motor_ident.phase==IDENT_CANCELLED);
    /* Across the full stator circle, choose the nearest balanced phase
     * direction. A small q injection must not reverse any phase current. */
    for(unsigned deg=0;deg<360;deg+=5u){
        foc.rpm=0;foc.calibration.zero=foc_wrap(-(float)deg*.01745329252f);
        assert(foc_identify(1.5f,12));sample(12,false);
        float selected=foc.electrical_deg,theta=selected*.01745329252f;
        assert(fabsf(remainderf(selected-(float)deg,360.0f))<=30.001f);
        assert(fabsf(selected/60.0f-roundf(selected/60.0f))<.0001f);
        for(int sign=-1;sign<=1;sign+=2){
            float a=cosf(theta)-.2f*(float)sign*sinf(theta);
            float b=sinf(theta)+.2f*(float)sign*cosf(theta);
            assert(fabsf(a)>.32f);
            assert(fabsf(-.5f*a+.8660254038f*b)>.32f);
            assert(fabsf(-.5f*a-.8660254038f*b)>.32f);
        }
        foc_step(.2f,12,1.65f,1.65f,0);
        assert(fabsf(foc.electrical_deg-selected)<.001f);
        foc_stop();
    }
    foc.calibration.zero=0;
    assert(foc_apply_parameters(&p));assert(foc_calibrate());assert(!foc_parameters_active && !motor_ident.valid);
    foc_stop();
    assert(foc_apply_parameters(&p));assert(foc_parameters_can_rollback());
    foc_trip(FOC_TIMING);assert(foc_rollback_parameters() && !foc_parameters_can_rollback());
    assert(foc.state==FOC_FAULT && foc.fault==FOC_TIMING); /* Never clear a fault to roll back. */
    motor_ident.phase=IDENT_READY;motor_ident.valid=true;
    foc_trip(FOC_TIMING);assert(!motor_ident.valid);
    assert(!foc_apply_parameters(&p));
    puts("PASS: real FOC runtime gains, 12/18/24 V adaptation, both current signs, stop/default/calibration guards");
}
