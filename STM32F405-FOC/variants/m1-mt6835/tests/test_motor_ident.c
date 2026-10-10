#include "motor_ident.h"
#include "bsp_motor_record.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static motor_ident_config_t config={10000,1.5f,1.5f,11.1f,24.0f,200};
static void plant(float r,float ld,float lq,float bus)
{
    assert(motor_ident_start(&config,100,bus));
    float id=0,iq=0,ud=0,uq=0;
    unsigned n=0;
    while(motor_ident_active() && n++<(unsigned)(config.sample_hz*4)) {
        /* Exact continuous RL solution, sampled after one held voltage.
         * A constant inverter drop under positive d-axis bias cancels in
         * differential R and the AC amplitude. No controller-gain oracle. */
        id=id*expf(-r/(config.sample_hz*ld))+(ud-.08f)/r*(1-expf(-r/(config.sample_hz*ld)));
        iq=iq*expf(-r/(config.sample_hz*lq))+uq/r*(1-expf(-r/(config.sample_hz*lq)));
        (void)motor_ident_step(id+.001f*sinf((float)n),iq,ud,uq,100,0,bus,&ud,&uq);
    }
    printf("R=%g Ld=%g Lq=%g: state=%u err=%u -> R=%g Ld=%g Lq=%g\n",r,ld,lq,
        motor_ident.phase,motor_ident.error,motor_ident.candidate.resistance,motor_ident.candidate.ld,motor_ident.candidate.lq);
    assert(motor_ident.valid && motor_ident.phase==IDENT_READY && ud==0 && uq==0);
    motor_parameters_t *p=&motor_ident.candidate;
    assert(fabsf(p->resistance/r-1)<.05f);
    assert(fabsf(p->ld/ld-1)<.08f && fabsf(p->lq/lq-1)<.08f);
    assert(motor_parameters_valid(p,config.sample_hz));
    foc_calibration_t cal={1.23f,1};
    tuning_record_t rec={.version=1,.profile=(FOC_CALIBRATION_ID<<16)|FOC_POLE_PAIRS,
        .cal=cal,.parameters=*p,.magic=0x524c5049u};
    rec.checksum=tuning_checksum(&rec);assert(tuning_record_valid(&rec,&cal));
    rec.parameters.ld*=2;assert(!tuning_record_valid(&rec,&cal));
    rec.parameters=*p;rec.checksum=tuning_checksum(&rec);cal.zero+=.01f;assert(!tuning_record_valid(&rec,&cal));
    rec.parameters.ld=NAN;rec.checksum=tuning_checksum(&rec);assert(!tuning_record_valid(&rec,&rec.cal));
}
int main(void)
{
    plant(.1f,.0001f,.00015f,12);
    plant(.7f,.0006f,.0004f,24);
    plant(.2f,.0003f,.0003f,18);
    config.sample_hz=20000;plant(.3f,.0002f,.0003f,12);config.sample_hz=10000;
    assert(!motor_ident_start(&config,100,7));
    float ud,uq;
    assert(motor_ident_start(&config,100,12));
    assert(!motor_ident_step(0,0,0,0,100,0,15,&ud,&uq));assert(motor_ident.error==IDENT_BUS);
    assert(motor_ident_start(&config,100,12));
    assert(!motor_ident_step(3,0,0,0,100,0,12,&ud,&uq));assert(motor_ident.error==IDENT_CURRENT);
    assert(motor_ident_start(&config,100,12));
    assert(!motor_ident_step(0,0,0,0,100.6f,0,12,&ud,&uq));assert(motor_ident.error==IDENT_MOVING);
    assert(motor_ident_start(&config,100,12));
    assert(!motor_ident_step(NAN,0,0,0,100,0,12,&ud,&uq));assert(motor_ident.error==IDENT_INPUT);
    assert(motor_ident_start(&config,100,12));
    for(unsigned i=0;i<7000 && motor_ident_active();++i)(void)motor_ident_step(0,0,0,0,100,0,12,&ud,&uq);
    assert(motor_ident.phase==IDENT_FAILED && motor_ident.error==IDENT_EXCITATION);
    assert(motor_ident_start(&config,100,12));motor_ident_cancel();assert(!motor_ident.valid && !motor_ident_active());
    assert(motor_ident_start(&config,100,12));motor_ident_abort(IDENT_TIMING);
    assert(motor_ident.phase==IDENT_FAILED && motor_ident.error==IDENT_TIMING && !motor_ident.valid);
    puts("PASS: RL plants, inverter offset, 12/18/24 V, 10/20 kHz, CRC/binding and rejection boundaries");
}
