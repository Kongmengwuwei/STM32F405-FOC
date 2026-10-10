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
    foc_calibration_t cal={0,1};foc_init(&cal);
    for(unsigned n=0;n<5000;++n)sample(12,false);
    assert(foc.state==FOC_IDLE && foc.zero_ready);
    motor_parameters_t p={.2f,.0003f,.0005f,0,0,0,200};
    p.kp_d=6.2831853072f*p.bandwidth_hz*p.ld;p.kp_q=6.2831853072f*p.bandwidth_hz*p.lq;
    p.ki=6.2831853072f*p.bandwidth_hz*p.resistance;
    assert(foc_apply_parameters(&p) && foc_parameters_active);
    assert(control_torque(.3f));
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
    foc_stop();assert(!foc_identifying() && motor_ident.phase==IDENT_CANCELLED);
    assert(foc_apply_parameters(&p));assert(foc_calibrate());assert(!foc_parameters_active && !motor_ident.valid);
    foc_stop();
    motor_ident.phase=IDENT_READY;motor_ident.valid=true;
    foc_trip(FOC_TIMING);assert(!motor_ident.valid);
    assert(!foc_apply_parameters(&p));
    puts("PASS: real FOC runtime gains, 12/18/24 V adaptation, both current signs, stop/default/calibration guards");
}
