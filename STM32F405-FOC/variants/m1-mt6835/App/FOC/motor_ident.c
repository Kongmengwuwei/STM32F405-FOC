#include "motor_ident.h"
#include <math.h>
#define TAU 6.283185307179586f
motor_ident_status_t motor_ident;
static motor_ident_config_t cfg;
static uint32_t tick, samples;
static float origin, start_bus, dc, ac, osc_s, osc_c, rot_s, rot_c;
static float sum_i, sum_u, sum_ii, si, ci, su, cu;
static float low_i, low_u, impedance[2];
static unsigned pass;
static motor_parameters_t first_pass;

bool motor_parameters_valid(const motor_parameters_t *p, float hz)
{
    if (!p || !isfinite(hz) || hz < 10000.0f) return false;
    if (!isfinite(p->resistance) || !isfinite(p->ld) || !isfinite(p->lq) ||
        !isfinite(p->kp_d) || !isfinite(p->kp_q) || !isfinite(p->ki) || !isfinite(p->bandwidth_hz)) return false;
    if (p->resistance < .02f || p->resistance > 10.0f || p->ld < 2e-6f || p->ld > .05f ||
        p->lq < 2e-6f || p->lq > .05f || p->bandwidth_hz < 20.0f || p->bandwidth_hz > hz/40.0f ||
        p->kp_d <= 0.0f || p->kp_d > 10.0f || p->kp_q <= 0.0f || p->kp_q > 10.0f ||
        p->ki <= 0.0f || p->ki > 5000.0f) return false;
    float w = TAU * p->bandwidth_hz;
    return fabsf(p->kp_d-w*p->ld) <= .001f*p->kp_d &&
           fabsf(p->kp_q-w*p->lq) <= .001f*p->kp_q && fabsf(p->ki-w*p->resistance) <= .001f*p->ki;
}
bool motor_ident_active(void) { return motor_ident.phase >= IDENT_R_LOW && motor_ident.phase <= IDENT_LQ_HIGH; }
void motor_ident_cancel(void)
{
    if (motor_ident_active()) { motor_ident.phase=IDENT_CANCELLED; motor_ident.valid=false; }
}
void motor_ident_abort(uint32_t error)
{
    if(motor_ident_active()) { motor_ident.phase=IDENT_FAILED;motor_ident.error=error;motor_ident.valid=false; }
}
static void begin_phase(uint32_t phase)
{
    motor_ident.phase=phase; tick=samples=0;
    sum_i=sum_u=sum_ii=si=ci=su=cu=0.0f;
    bool high=(phase == IDENT_LD_HIGH || phase == IDENT_LQ_HIGH);
    /* Fixed samples/period: avoid libm calls inside the command IRQ lock and
     * the phase-change ISR. Frequencies scale with the acquisition clock. */
    rot_s=high ? .3090169944f : .1564344650f;
    rot_c=high ? .9510565163f : .9876883406f;
    osc_s=0.0f; osc_c=1.0f;
}
bool motor_ident_start(const motor_ident_config_t *config, float angle, float bus)
{
    if (!config || motor_ident_active()) return false;
    const float values[]={config->sample_hz,config->test_current,config->voltage_limit,
        config->bus_min,config->bus_max,config->bandwidth_hz,angle,bus};
    for (unsigned i=0;i<sizeof values/sizeof values[0];++i) if (!isfinite(values[i])) return false;
    if (config->sample_hz < 10000.0f || config->sample_hz > 20000.0f || config->test_current < .2f ||
        config->test_current > 3.0f || config->voltage_limit <= 0.0f || config->voltage_limit > 1.5f ||
        config->bus_min <= 0.0f || config->bus_min >= config->bus_max || bus < config->bus_min || bus > config->bus_max ||
        config->bandwidth_hz < 20.0f || config->bandwidth_hz > config->sample_hz/40.0f) return false;
    cfg=*config; origin=angle; start_bus=bus; dc=ac=low_i=low_u=0.0f; pass=0;
    /* Started with IRQs locked: typed stores avoid nano-libc's byte-wise
     * memset extending the critical section past the PWM update deadline. */
    volatile motor_ident_status_t *st=&motor_ident;
    st->phase=st->error=0;st->valid=st->repeat_valid=false;
    st->candidate.resistance=st->candidate.ld=st->candidate.lq=0;
    st->candidate.kp_d=st->candidate.kp_q=st->candidate.ki=st->candidate.bandwidth_hz=0;
    begin_phase(IDENT_R_LOW); return true;
}
static bool fail(uint32_t error, float *ud, float *uq)
{
    motor_ident.error=error; motor_ident.phase=IDENT_FAILED; motor_ident.valid=false;
    *ud=*uq=0.0f; return false;
}
static bool inductance(float *l)
{
    /* Exact zero-order-held RL impedance, rather than a continuous-time
     * approximation at the ADC rate. |Z|^2 = Rac^2 + B*(1-cos(Omega)),
     * B=2*Rac^2*a/(1-a)^2, a=exp(-Rac*Ts/L). Magnitudes cancel a fixed
     * acquisition delay; voltages still MUST come from the active PWM. */
    const float c1=.9876883406f, c2=.9510565163f;
    float z1=impedance[0],z2=impedance[1],r=motor_ident.candidate.resistance;
    float b=(z2*z2-z1*z1)/(c1-c2);
    float rac2=z1*z1-b*(1.0f-c1);
    if (!isfinite(b) || b<=0.0f || rac2<=0.0f) return false;
    float rac=sqrtf(rac2),m=b/(2.0f*rac2);
    float a=2.0f*m/(2.0f*m+1.0f+sqrtf(4.0f*m+1.0f));
    if (!(a>0.0f && a<1.0f)) return false;
    *l=-rac/(cfg.sample_hz*logf(a));
    /* Reject unresolved reactive response, inverter distortion or motion;
     * never turn a negative radicand into a made-up inductance. */
    return *l >= 2e-6f && *l <= .05f && rac2 > 0.0f &&
        fabsf(rac-r) <= .30f*r && TAU*cfg.sample_hz/20.0f*(*l)/z2 >= .15f;
}
static bool repeat_agrees(float a,float b,float tolerance)
{
    return fabsf(a-b)<=tolerance*.5f*(a+b);
}
bool motor_ident_step(float id, float iq, float applied_ud, float applied_uq,
                      float angle, float rpm, float bus, float *ud, float *uq)
{
    *ud=*uq=0.0f;
    if (!motor_ident_active()) return false;
    if (!isfinite(id)||!isfinite(iq)||!isfinite(applied_ud)||!isfinite(applied_uq)||
        !isfinite(angle)||!isfinite(rpm)||!isfinite(bus)) return fail(IDENT_INPUT,ud,uq);
    if (bus < cfg.bus_min || bus > cfg.bus_max || fabsf(bus-start_bus) > .1f*start_bus) return fail(IDENT_BUS,ud,uq);
    if (hypotf(id,iq) > 1.8f*cfg.test_current) return fail(IDENT_CURRENT,ud,uq);
    uint32_t phase=motor_ident.phase;
    bool is_ac=phase >= IDENT_LD_LOW, q_axis=phase >= IDENT_LQ_LOW;
    uint32_t settle_ticks=(uint32_t)(cfg.sample_hz*.2f);
    bool settling=!is_ac && tick<settle_ticks;
    if(!is_ac && tick==settle_ticks)origin=angle;
    /* DC steps may shift the held-field equilibrium. Only the existing first
     * .2 s can settle; collected samples and all AC stages stay within .5 deg.
     * Settling itself remains bounded and contributes no R fit samples. */
    if(fabsf(angle-origin)>(settling ? 30.0f : .5f) || fabsf(rpm)>(settling ? 120.0f : 20.0f))
        return fail(IDENT_MOVING,ud,uq);
    float target=cfg.test_current * (phase==IDENT_R_LOW ? .5f : 1.0f);
    /* Slow voltage integrator bootstraps without unknown current-loop gains.
     * Freeze it during AC collection so injected-frequency plant stays open. */
    if (!is_ac || tick < (uint32_t)(cfg.sample_hz*.2f))
        dc=fminf(cfg.voltage_limit*.8f,fmaxf(0.0f,dc + 20.0f*(target-id)/cfg.sample_hz));
    if (tick >= (uint32_t)(cfg.sample_hz*.2f)) {
        float current=q_axis ? iq : id, voltage=q_axis ? applied_uq : applied_ud;
        ++samples; sum_i+=current; sum_u+=voltage; sum_ii+=current*current;
        if (is_ac) { si+=current*osc_s;ci+=current*osc_c;su+=voltage*osc_s;cu+=voltage*osc_c; }
    }
    *ud=dc;
    if (is_ac) { if(q_axis)*uq=ac*osc_s;else *ud+=ac*osc_s; }
    float next_s=osc_s*rot_c+osc_c*rot_s;
    osc_c=osc_c*rot_c-osc_s*rot_s;osc_s=next_s;
    /* Normalize once per excitation period; no unbounded oscillator drift. */
    if (tick%40u==39u) { float inv=1.0f/hypotf(osc_s,osc_c);osc_s*=inv;osc_c*=inv; }
    if (++tick < (uint32_t)(cfg.sample_hz*.6f)) return true;
    float mean=sum_i/(float)samples;
    if (!is_ac) {
        float variance=fmaxf(0.0f,sum_ii/(float)samples-mean*mean);
        if (fabsf(mean-target) > .1f*target || sqrtf(variance) > .05f*target+.02f)
            return fail(IDENT_EXCITATION,ud,uq);
        if (phase==IDENT_R_LOW) { low_i=mean;low_u=sum_u/(float)samples; }
        else {
            float r=(sum_u/(float)samples-low_u)/(mean-low_i);
            if (!isfinite(r)||r<.02f||r>10.0f) return fail(IDENT_FIT,ud,uq);
            motor_ident.candidate.resistance=r;
            ac=fminf((pass ? .15f : .2f)*r*cfg.test_current,cfg.voltage_limit*.15f);
            if (ac < .01f || dc+ac > cfg.voltage_limit) return fail(IDENT_EXCITATION,ud,uq);
        }
    } else {
        float i2=si*si+ci*ci,u2=su*su+cu*cu;
        float power=sum_ii-sum_i*mean;
        float fundamental=2.0f*i2/(float)samples;
        if (sqrtf(i2)*2.0f/(float)samples < .025f || power <= 0.0f || fundamental < .75f*power)
            return fail(IDENT_EXCITATION,ud,uq);
        unsigned high=(phase==IDENT_LD_HIGH||phase==IDENT_LQ_HIGH);
        impedance[high]=sqrtf(u2/i2);
        if (high && !inductance(q_axis ? &motor_ident.candidate.lq : &motor_ident.candidate.ld))
            return fail(IDENT_FIT,ud,uq);
    }
    if (phase==IDENT_LQ_HIGH) {
        motor_parameters_t *p=&motor_ident.candidate;
        if (!pass) {
            first_pass=*p;pass=1;
            /* Independent DC and both-axis AC measurements at 75% AC
             * amplitude expose drift and amplitude-dependent inverter bias.
             * Both passes retain the same current/voltage/motion bounds. */
            begin_phase(IDENT_R_LOW);return true;
        }
        if (!repeat_agrees(p->resistance,first_pass.resistance,.15f) ||
            !repeat_agrees(p->ld,first_pass.ld,.20f) ||
            !repeat_agrees(p->lq,first_pass.lq,.20f)) return fail(IDENT_REPEAT,ud,uq);
        p->resistance=.5f*(p->resistance+first_pass.resistance);
        p->ld=.5f*(p->ld+first_pass.ld);p->lq=.5f*(p->lq+first_pass.lq);
        float bw=fminf(cfg.bandwidth_hz,fminf(5000.0f/(TAU*p->resistance),10.0f/(TAU*fmaxf(p->ld,p->lq))));
        p->bandwidth_hz=bw;p->kp_d=TAU*bw*p->ld;p->kp_q=TAU*bw*p->lq;p->ki=TAU*bw*p->resistance;
        if (!motor_parameters_valid(p,cfg.sample_hz)) return fail(IDENT_FIT,ud,uq);
        motor_ident.repeat_valid=motor_ident.valid=true;motor_ident.phase=IDENT_READY;*ud=*uq=0.0f;return false;
    }
    begin_phase(phase+1u);return true;
}
