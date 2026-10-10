#include "foc.h"
#include "bsp_motor.h" /* motor_sample_us: the sample-phase timestamp. */
#include "control.h"
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846f
#define TURN (2.0f * PI)
#define ALIGN_START (FOC_MOTOR_ALIGNMENT_START_DEG * (PI / 180.0f))
#define ALIGN_HOLD_END FOC_CAL_TICKS(30000u)
#define ALIGN_FORWARD_END FOC_CAL_TICKS(30000u + FOC_ALIGNMENT_SWEEP_TICKS_20KHZ)
#define ALIGN_REVERSE_END FOC_CAL_TICKS(30000u + 2u * FOC_ALIGNMENT_SWEEP_TICKS_20KHZ)
#define ALIGN_SETTLE_END FOC_CAL_TICKS(36000u + 2u * FOC_ALIGNMENT_SWEEP_TICKS_20KHZ)
#define ALIGN_SAMPLE_END FOC_CAL_TICKS(40000u + 2u * FOC_ALIGNMENT_SWEEP_TICKS_20KHZ)
foc_t foc;
motor_parameters_t foc_parameters;
bool foc_parameters_active;
static float previous, position, origin, forward, sum_sin, sum_cos, low, high;
static uint32_t ticks;
static float integral_d, integral_q, variance_b, variance_c, previous_command;
static float pwm_offset_sum_b, pwm_offset_sum_c;
static bool tracking, aligning, identifying, resuming;
static float identification_current_limit;

bool foc_identifying(void) { return identifying; }
bool foc_default_parameters(void)
{
    if (foc.state != FOC_IDLE) return false;
    foc_parameters_active=false; integral_d=integral_q=0.0f;
    return true;
}
bool foc_apply_parameters(const motor_parameters_t *p)
{
    if (foc.state != FOC_IDLE || !foc.calibrated || !foc.zero_ready ||
        !motor_parameters_valid(p,(float)FOC_SAMPLE_HZ)) return false;
    foc_parameters=*p;foc_parameters_active=true;integral_d=integral_q=0.0f;
    return true;
}
bool foc_identify(float current, float bus)
{
    if (foc.state != FOC_IDLE || !foc.calibrated || !foc.zero_ready || fabsf(foc.rpm)>=5.0f ||
        current > fminf(3.0f,FOC_PHASE_TRIP/2.0f)) return false;
    motor_ident_config_t config={(float)FOC_SAMPLE_HZ,current,1.5f,FOC_BUS_MIN,FOC_BUS_MAX,
        fminf(200.0f,(float)FOC_SAMPLE_HZ/40.0f)};
    if (!motor_ident_start(&config,position,bus)) return false;
    control_stop();identifying=true;aligning=false;
    identification_current_limit=1.8f*current;
    foc.command=foc.iq_ref=0.0f;integral_d=integral_q=0.0f;ticks=0u;
    foc.state=FOC_PRECHARGE;return true;
}

static void sincos_fast(float theta, float *s, float *c)
{
    static const float table[513] = {
#include "sine_table.inc"
    };
    float x = theta * (512.0f / TURN);
    unsigned index = (unsigned)x;
    float fraction = x - (float)index;
    unsigned i = index & 511u, j = (i + 128u) & 511u;
    *s = table[i] + fraction * (table[i + 1u] - table[i]);
    *c = table[j] + fraction * (table[j + 1u] - table[j]);
}

float foc_wrap(float radians)
{
    return radians - floorf(radians / TURN) * TURN;
}

void foc_integrators(float *d, float *q)
{
    *d = integral_d;
    *q = integral_q;
}

float foc_modulate(float a, float beta, float bus_voltage, float duty[3])
{
    float b = -0.5f * a + 0.8660254038f * beta;
    float c = -0.5f * a - 0.8660254038f * beta;
    float maximum = a > b ? a : b, minimum = a < b ? a : b;
    maximum = maximum > c ? maximum : c;
    minimum = minimum < c ? minimum : c;
    /* Keep centred SVPWM where possible. Shift all phases equally to extend
       the low-side window; scale only when the phase span no longer fits. */
    float ceiling = (float)FOC_EDGE_LIMIT / (float)FOC_PWM_ARR;
    float span = maximum - minimum, available = ceiling * bus_voltage;
    float scale = span > available ? available / span : 1.0f;
    float inverse_bus = 1.0f / bus_voltage;
    float width = span * scale * inverse_bus;
    float offset = 0.5f - 0.5f * width;
    if (offset > ceiling - width) offset = ceiling - width;
    if (offset < 0.0f) offset = 0.0f; /* Roundoff at full phase span. */
    inverse_bus *= scale;
    duty[0] = offset + (a - minimum) * inverse_bus;
    duty[1] = offset + (b - minimum) * inverse_bus;
    duty[2] = offset + (c - minimum) * inverse_bus;
    return scale;
}

bool foc_window(const float duty[3])
{
    /* All phases must be quiet across the aperture, B/C low sides conducting.
       Include quantized CCR rounding and the provisional trigger allowance. */
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(duty[i]) || duty[i] < 0.0f || duty[i] > 1.0f) return false;
        uint32_t edge = (uint32_t)(duty[i] * (float)FOC_PWM_ARR + 0.5f);
        if (edge + FOC_SETTLE_TICKS >= FOC_TRIGGER_TICKS ||
            FOC_HOLD_TICKS + FOC_SETTLE_TICKS >= FOC_PWM_PERIOD_TICKS - edge) return false;
    }
    return true;
}

void foc_stop(void)
{
    motor_ident_cancel();identifying=false;
    foc.command = foc.iq_ref = foc.ud = foc.uq = 0.0f;
    foc.voltage_scale = 0.0f;
    integral_d = integral_q = 0.0f;
    pwm_offset_sum_b = pwm_offset_sum_c = 0.0f;
    aligning = false;
    previous_command = 0.0f; /* Cancel any in-flight command ramp. */
    foc.duty[0] = foc.duty[1] = foc.duty[2] = 0.0f;
    control_stop();
    if (foc.state != FOC_FAULT) foc.state = foc.zero_ready ? FOC_IDLE : FOC_OFFSET;
}

void foc_trip(uint32_t fault)
{
    /* Priority-0 faults can preempt the estimator. Repeated fault handling
     * also clears a READY candidate published after that preemption. */
    motor_ident.valid=false;
    if(identifying || motor_ident_active())motor_ident_abort(fault==FOC_TIMING ? IDENT_TIMING :
        fault==FOC_CURRENT ? IDENT_CURRENT : fault==FOC_BUS ? IDENT_BUS : IDENT_INPUT);
    foc_warn(fault);
    if (foc.state != FOC_FAULT) foc.fault = fault;
    foc.state = FOC_FAULT;
    foc_stop();
}

void foc_warn(uint32_t warning)
{
    if (!warning || warning > FOC_VOLTAGE) return;
    uint32_t bit = 1u << warning;
    if (!(__atomic_fetch_or(&foc.warnings, bit, __ATOMIC_RELAXED) & bit)) {
        /* IRQ0 can interrupt IRQ1 or the foreground parser. */
        __atomic_add_fetch(&foc.warning_count, 1u, __ATOMIC_RELAXED);
        __atomic_store_n(&foc.last_warning, warning, __ATOMIC_RELAXED);
    }
}

void foc_clear_warnings(void)
{
    foc.warnings = foc.last_warning = foc.warning_count = 0u;
}

bool foc_check(uint32_t warning)
{
    foc_warn(warning);
    if (!FOC_PROTECTION_TRIP) return false;
    foc_trip(warning);
    return true;
}

bool foc_calibrate(void)
{
    if (foc.state != FOC_IDLE || !foc.zero_ready) return false;
    if (fabsf(foc.rpm) >= 5.0f) {
        foc_warn(FOC_SPEED);
        if (FOC_PROTECTION_TRIP) return false;
    }
    foc_parameters_active=false;motor_ident.valid=false;
    aligning = true;
    integral_d = integral_q = 0.0f;
    position = previous; /* Keep alignment deltas precise after many revolutions. */
    ticks = 0u;
    foc.state = FOC_PRECHARGE;
    return true;
}

void foc_init(const foc_calibration_t *calibration)
{
    memset(&foc, 0, sizeof foc);
    memset(&motor_ident,0,sizeof motor_ident);
    memset(&foc_parameters,0,sizeof foc_parameters);
    foc_parameters_active=identifying=false;
    tracking = false;
    resuming = false;
    aligning = FOC_AUTOCALIBRATE && calibration == NULL;
    ticks = 0u;
    integral_d = integral_q = variance_b = variance_c = 0.0f;
    pwm_offset_sum_b = pwm_offset_sum_c = 0.0f;
    if (calibration) { foc.calibration = *calibration; foc.calibrated = true; }
    foc.state = FOC_OFFSET; /* Gate-off current offsets precede any alignment. */
}

void foc_acquisition_resume(void)
{
    /* Flash save pauses acquisition with gates off. The first new delta
       covers that pause, not one sample period; preserve position but do not
       interpret it as instantaneous speed or a sensor discontinuity. */
    resuming = true;
    foc.rpm = 0.0f;
}

bool foc_current(float amps)
{
    if (!isfinite(amps)) return false;
    if (fabsf(amps) > FOC_CURRENT_MAX) {
        foc_warn(FOC_CURRENT);
        if (FOC_PROTECTION_TRIP) return false;
    }
    if (!foc.calibrated || !foc.zero_ready || (foc.state != FOC_IDLE && foc.state != FOC_RUN)) return false;
    foc.command = amps;
    if (foc.state == FOC_IDLE && amps != 0.0f) {
        aligning = false;
        integral_d = integral_q = foc.iq_ref = 0.0f;
        ticks = 0u;
        foc.state = FOC_PRECHARGE;
    }
    return true;
}

void foc_step(float mechanical_deg, float bus_voltage, float b_voltage, float c_voltage, float encoder_delay)
{
    if (!isfinite(mechanical_deg)) { foc_trip(FOC_SENSOR); return; }
    if (!isfinite(b_voltage) || !isfinite(c_voltage)) { foc_trip(FOC_ADC); return; }
    if (!isfinite(encoder_delay)) { foc_trip(FOC_TIMING); return; }
    if (encoder_delay < 0.0f || encoder_delay > 50e-6f) {
        if (foc_check(FOC_TIMING)) return;
        encoder_delay = 0.0f; /* Ignore unusable timestamp compensation. */
    }
    if (foc.state == FOC_PRECHARGE || foc.state == FOC_RUN || foc.state == FOC_CALIBRATE || foc.state == FOC_PWM_ZERO) {
        if (!isfinite(bus_voltage) || bus_voltage <= 0.0f) { foc_trip(FOC_BUS); return; }
        if (identifying && (bus_voltage < FOC_BUS_MIN || bus_voltage > FOC_BUS_MAX)) { foc_trip(FOC_BUS);return; }
        if ((bus_voltage < FOC_BUS_MIN || bus_voltage > FOC_BUS_MAX) && foc_check(FOC_BUS)) return;
    }
    /* Noncomputable inputs stop output, but WARN never locks out a later
       valid command. Resume to idle, never silently restore old torque. */
    if (!FOC_PROTECTION_TRIP && foc.state == FOC_FAULT) {
        if (!isfinite(bus_voltage) || bus_voltage <= 0.0f) return;
        foc.fault = FOC_OK;
        foc.state = foc.zero_ready ? FOC_IDLE : FOC_OFFSET;
        tracking = false;
        ticks = 0u;
        foc.rpm = 0.0f;
        if (!foc.zero_ready) foc.b_offset = foc.c_offset = variance_b = variance_c = 0.0f;
    }
    float s, c;
    if (mechanical_deg < 0.0f || mechanical_deg >= 360.0f) {
        mechanical_deg = fmodf(mechanical_deg, 360.0f);
        if (mechanical_deg < 0.0f) mechanical_deg += 360.0f;
    }
    /* Optional correction measured for one encoder/magnet installation. */
    sincos_fast(mechanical_deg * (PI / 90.0f), &s, &c);
    mechanical_deg += FOC_ENCODER_HARMONIC_DEG * c;
    float delta = mechanical_deg - previous;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    if (!tracking) { delta = 0.0f; position = mechanical_deg; tracking = true; }
    else if (!resuming && fabsf(delta) > FOC_MAX_STEP_DEG && foc_check(FOC_SENSOR)) return;
    previous = mechanical_deg;
    position += delta;
    if (!resuming) foc.rpm += FOC_RPM_FILTER_ALPHA * (delta * ((float)FOC_SAMPLE_HZ / 6.0f) - foc.rpm);
    resuming = false;
    if (foc.state == FOC_OFFSET) {
        if (fabsf(foc.rpm) >= 5.0f || fabsf(delta) >= FOC_STATIONARY_STEP_DEG) {
            foc_warn(FOC_ZERO);
            if (FOC_PROTECTION_TRIP) {
                ticks = 0u;
                foc.b_offset = foc.c_offset = variance_b = variance_c = 0.0f;
                return;
            }
        }
        if (++ticks <= FOC_OFFSET_WAIT_SAMPLES) return; /* 200 ms stationary. */
        float n = (float)(ticks - FOC_OFFSET_WAIT_SAMPLES);
        float db = b_voltage - foc.b_offset, dc = c_voltage - foc.c_offset;
        foc.b_offset += db / n; foc.c_offset += dc / n;
        variance_b += db * (b_voltage - foc.b_offset);
        variance_c += dc * (c_voltage - foc.c_offset);
        if (ticks == FOC_OFFSET_WAIT_SAMPLES + 2048u) {
            if (foc.b_offset < 1.4f || foc.b_offset > 1.9f || foc.c_offset < 1.4f || foc.c_offset > 1.9f ||
                variance_b > 2048.0f * 4e-6f || variance_c > 2048.0f * 4e-6f) {
                if (foc_check(FOC_ZERO)) return;
            }
            foc.zero_ready = true;
            foc.state = FOC_IDLE;
            if (aligning && !foc.calibrated) (void)foc_calibrate();
        }
        return;
    }
    if (!foc.zero_ready) { foc.id = foc.iq = 0.0f; return; }
    float omega = (float)foc.calibration.direction * (float)FOC_POLE_PAIRS * foc.rpm * (TURN / 60.0f);
    float theta = foc_wrap((float)foc.calibration.direction * (float)FOC_POLE_PAIRS * mechanical_deg * (PI / 180.0f) -
                          foc.calibration.zero - omega * encoder_delay);
    if (!isfinite(omega) || !isfinite(theta)) { foc_trip(FOC_NUMERIC); return; }
    foc.electrical_deg = theta * (180.0f / PI);
    sincos_fast(theta, &s, &c);
    float ib = (b_voltage - foc.b_offset) * FOC_CURRENT_A_PER_V, ic = (c_voltage - foc.c_offset) * FOC_CURRENT_A_PER_V;
    float ia = -ib - ic, beta = (ib - ic) * 0.5773502692f;
    if (!isfinite(ia) || !isfinite(beta)) {
        foc_trip(FOC_NUMERIC); return;
    }
    foc.id = ia * c + beta * s;
    foc.iq = -ia * s + beta * c;
    if (identifying && hypotf(foc.id,foc.iq) > identification_current_limit) { foc_trip(FOC_CURRENT);return; }
    if (foc.state == FOC_PRECHARGE || foc.state == FOC_RUN || foc.state == FOC_CALIBRATE || foc.state == FOC_PWM_ZERO) {
        float trip = FOC_PHASE_TRIP;
        if ((fabsf(ia) >= trip || fabsf(ib) >= trip || fabsf(ic) >= trip) && foc_check(FOC_CURRENT)) return;
        float speed_trip = control_mode() == CONTROL_TORQUE ? FOC_TORQUE_SPEED_TRIP_RPM : FOC_SPEED_MAX;
        if (!aligning && fabsf(foc.rpm) >= speed_trip && foc_check(FOC_SPEED)) return;
    }
    if (foc.state == FOC_PRECHARGE) {
        if (++ticks < FOC_PRECHARGE_SAMPLES) return; /* 2 ms, low sides on. */
        ticks = 0u;
        if (FOC_PWM_ZERO_SAMPLES != 0u) {
            pwm_offset_sum_b = pwm_offset_sum_c = 0.0f;
            foc.ud = foc.uq = 0.0f;
            foc.duty[0] = foc.duty[1] = foc.duty[2] = 0.5f;
            foc.state = FOC_PWM_ZERO;
            return;
        }
        foc.state = (aligning || identifying) ? FOC_CALIBRATE : FOC_RUN;
    }
#if FOC_PWM_ZERO_SAMPLES > 0u
    if (foc.state == FOC_PWM_ZERO) {
        /* Equal PWM duty creates zero line-to-line voltage. Measure the ADC
           zero with the power stage switching, after its first millisecond. */
        if (fabsf(foc.rpm) >= 5.0f && foc_check(FOC_SPEED)) return;
        ++ticks;
        foc.duty[0] = foc.duty[1] = foc.duty[2] = 0.5f;
        if (ticks > FOC_PWM_ZERO_SETTLE_SAMPLES) {
            pwm_offset_sum_b += b_voltage;
            pwm_offset_sum_c += c_voltage;
        }
        if (ticks == FOC_PWM_ZERO_SETTLE_SAMPLES + FOC_PWM_ZERO_SAMPLES) {
            float next_b = pwm_offset_sum_b / (float)FOC_PWM_ZERO_SAMPLES;
            float next_c = pwm_offset_sum_c / (float)FOC_PWM_ZERO_SAMPLES;
            if (fabsf(next_b - foc.b_offset) > 0.01f || fabsf(next_c - foc.c_offset) > 0.01f) {
                if (foc_check(FOC_ZERO)) return;
            }
            foc.b_offset = next_b; foc.c_offset = next_c;
            integral_d = integral_q = 0.0f;
            ticks = 0u;
            foc.state = (aligning || identifying) ? FOC_CALIBRATE : FOC_RUN;
        }
        return;
    }
#endif
    if (foc.state == FOC_CALIBRATE && identifying) {
        /* Stationary d/q probing; active CCRs describe the period just sampled.
           Common-mode duty cancels in the reconstructed alpha/beta voltage. */
        float va=bus_voltage*(2.0f*motor_duty[0]-motor_duty[1]-motor_duty[2])/3.0f;
        float vb=bus_voltage*(motor_duty[1]-motor_duty[2])*0.5773502692f;
        float ud,uq;
        if (!motor_ident_step(foc.id,foc.iq,va*c+vb*s,-va*s+vb*c,
                              position,foc.rpm,bus_voltage,&ud,&uq)) {
            foc_stop();return;
        }
        foc.ud=ud;foc.uq=uq;
        foc.voltage_scale=foc_modulate(ud*c-uq*s,ud*s+uq*c,bus_voltage,foc.duty);
        foc.ud*=foc.voltage_scale;foc.uq*=foc.voltage_scale;
        return;
    }
    if (foc.state == FOC_RUN) {
        /* Optional command shaping; WARN defaults to direct current steps. */
        float step = foc.command - previous_command;
        const float ramp_step = FOC_CURRENT_SLEW_A_PER_S / (float)FOC_SAMPLE_HZ;
        if (ramp_step > 0.0f) {
            if (step > ramp_step) step = ramp_step;
            else if (step < -ramp_step) step = -ramp_step;
        }
        previous_command += step;
        foc.iq_ref = previous_command;
        /* The 1 kHz outer loop, when scheduled, replaces the reference. It only
           produces a value on its own millisecond, so hold the torque reference
           between outer-loop updates. */
        uint32_t outer_fault = control_fault();
        if (outer_fault) { foc_trip(outer_fault); return; }
        if (control_mode() != CONTROL_TORQUE && control_scheduled())
            foc.iq_ref = control_iq_ref();
        /* PI/feedforward and back calculation have independent tuning units. */
        float ed = -foc.id, eq = foc.iq_ref - foc.iq;
        float kpd=foc_parameters_active ? foc_parameters.kp_d : FOC_CURRENT_KP;
        float kpq=foc_parameters_active ? foc_parameters.kp_q : FOC_CURRENT_KP;
        float ki_step=foc_parameters_active ? foc_parameters.ki/(float)FOC_SAMPLE_HZ : FOC_CURRENT_KI_STEP;
        float aw_step=foc_parameters_active ? 6.2831853072f*foc_parameters.bandwidth_hz/(float)FOC_SAMPLE_HZ : FOC_CURRENT_AW_STEP;
        float ld=foc_parameters_active ? foc_parameters.ld : FOC_MOTOR_INDUCTANCE_H;
        float lq=foc_parameters_active ? foc_parameters.lq : FOC_MOTOR_INDUCTANCE_H;
        float ud = kpd * ed + integral_d - omega * lq * foc.iq;
        float uq = kpq * eq + integral_q + omega * (ld * foc.id + FOC_MOTOR_FLUX_WB);
        /* Linear SVPWM ceiling; modulation also accounts for the ADC window. */
        float fraction = FOC_PROTECTION_TRIP ? FOC_VOLTAGE_FRACTION : 0.5773502692f;
        /* New-motor commissioning ceiling also applies under WARN; the
           existing current/timeout warning policy remains independent. */
        float limit = bus_voltage * fminf(fraction, FOC_MOTOR_RUN_VOLTAGE_FRACTION);
        float norm2 = ud * ud + uq * uq;
        if (!isfinite(norm2) || !isfinite(integral_d) || !isfinite(integral_q)) {
            foc_trip(FOC_NUMERIC); return;
        }
        float scale = norm2 > limit * limit ? limit / sqrtf(norm2) : 1.0f;
        foc.voltage_scale = scale;
        if (scale < 1.0f) foc_warn(FOC_VOLTAGE);
        foc.ud = ud * scale; foc.uq = uq * scale;
        /* Predict to the next PWM centre using the selected timer period. */
        float advance = omega * (((float)(3u * FOC_PWM_ARR - FOC_HOLD_TICKS)) / 168e6f);
        float sa, ca;
        sincos_fast(foc_wrap(advance), &sa, &ca);
        float so = s * ca + c * sa, co = c * ca - s * sa;
        scale = foc_modulate(foc.ud * co - foc.uq * so, foc.ud * so + foc.uq * co, bus_voltage, foc.duty);
        foc.voltage_scale *= scale;
        if (scale < 1.0f) foc_warn(FOC_VOLTAGE);
        foc.ud *= scale; foc.uq *= scale;
        integral_d += ki_step * ed + aw_step * (foc.ud - ud);
        integral_q += ki_step * eq + aw_step * (foc.uq - uq);
    } else if (foc.state == FOC_CALIBRATE) {
        ++ticks;
        float theta = ALIGN_START, ud = FOC_ALIGNMENT_VOLTS;
        if (FOC_ALIGNMENT_CURRENT_A == 0.0f && ticks <= FOC_CAL_TICKS(10000u))
            ud *= (float)ticks / (float)FOC_CAL_TICKS(10000u);
        if (ticks == ALIGN_HOLD_END) origin = position;
        if (ticks > ALIGN_HOLD_END && ticks <= ALIGN_FORWARD_END)
            theta = ALIGN_START + TURN * (float)(ticks - ALIGN_HOLD_END) / (float)FOC_CAL_TICKS(FOC_ALIGNMENT_SWEEP_TICKS_20KHZ);
        if (ticks == ALIGN_FORWARD_END) {
            forward = position - origin;
            if (fabsf(forward) < (360.0f / (float)FOC_POLE_PAIRS) * 0.8f ||
                fabsf(forward) > (360.0f / (float)FOC_POLE_PAIRS) * 1.2f) {
                /* A failed experiment is not a usable calibration record. */
                if (!foc_check(FOC_ALIGNMENT)) foc_stop();
                return;
            }
        }
        if (ticks > ALIGN_FORWARD_END && ticks <= ALIGN_REVERSE_END)
            theta = ALIGN_START + TURN * (1.0f - (float)(ticks - ALIGN_FORWARD_END) / (float)FOC_CAL_TICKS(FOC_ALIGNMENT_SWEEP_TICKS_20KHZ));
        if (ticks == ALIGN_SETTLE_END) { sum_sin = sum_cos = 0.0f; low = high = position; }
        if (ticks > ALIGN_SETTLE_END) {
            float angle = mechanical_deg * (PI / 180.0f);
            sincos_fast(angle, &s, &c);
            sum_sin += s; sum_cos += c;
            low = fminf(low, position); high = fmaxf(high, position);
        }
        if (ticks == ALIGN_SAMPLE_END) {
            if (fabsf(position - origin) > 1.0f || high - low > 1.0f) {
                if (!foc_check(FOC_ALIGNMENT)) foc_stop();
                return;
            }
            foc.calibration.direction = forward > 0.0f ? 1 : -1;
            foc.calibration.zero = foc_wrap((float)foc.calibration.direction * (float)FOC_POLE_PAIRS * atan2f(sum_sin, sum_cos) - ALIGN_START);
            foc_stop();
            foc.state = FOC_SAVE;
            return;
        }
        sincos_fast(theta, &s, &c);
        if (FOC_ALIGNMENT_CURRENT_A > 0.0f) {
            /* Electrical zero is unknown; regulate current along the
               commanded stator field rather than the encoder d-axis. */
            float target = FOC_ALIGNMENT_CURRENT_A;
            if (ticks <= FOC_CAL_TICKS(10000u))
                target *= (float)ticks / (float)FOC_CAL_TICKS(10000u);
            float measured = ia * c + beta * s;
            float error = target - measured;
            integral_d += FOC_CURRENT_KI_STEP * error;
            float requested = FOC_CURRENT_KP * error + integral_d;
            if (!isfinite(requested)) { foc_trip(FOC_NUMERIC); return; }
            ud = fminf(fmaxf(requested, 0.0f), FOC_ALIGNMENT_VOLTS);
            integral_d += 0.1f * (ud - requested);
        }
        foc.ud = ud; foc.uq = 0.0f;
        foc_modulate(ud * c, ud * s, bus_voltage, foc.duty);
    }
}

void foc_outer_step(void)
{
    /* Track mechanical position while stopped too; only control_step's RUN
       branch produces torque. Holding starts from the actual shaft position. */
    if (foc.state != FOC_FAULT) control_step(motor_sample_us, position);
}
