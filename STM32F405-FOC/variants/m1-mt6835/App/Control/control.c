#if defined(FOC_DUAL_LEG) && (defined(__arm__) || defined(__thumb__))
#define FOC_REALTIME __attribute__((section(".RamFunc")))
#else
#define FOC_REALTIME
#endif
#include "control.h"
#include "anticogging.h"
#include "bsp_uart.h"
#include "foc.h"
#include <math.h>
#ifdef FOC_DUAL_LEG
#include "finite_math.h"
#endif
#ifdef FOC_DUAL_LEG
#include "leg_guard.h"
#endif

/* Basic cascade: position P -> speed PI -> Iq_ref, above the selected port's
   current loop. TRIP limits match the manual current command range; WARN
   records the same thresholds. A motor-specific actuator bound may also
   limit WARN output, as used by the Longqiu commissioning profile.
   Tuning order: speed Kp first until it tracks without oscillating, then speed
   Ki to remove the steady-state error, then position Kp. */
#ifndef FOC_OUTER_SPEED_KP

#define FOC_OUTER_SPEED_KP FOC_SPEED_KP
#endif
#ifndef FOC_OUTER_SPEED_KI
#define FOC_OUTER_SPEED_KI FOC_SPEED_KI
#endif
#ifndef FOC_OUTER_POSITION_KP
#define FOC_OUTER_POSITION_KP FOC_POSITION_KP
#endif
#define SPEED_KP FOC_OUTER_SPEED_KP
#define SPEED_KI FOC_OUTER_SPEED_KI
#define POSITION_KP FOC_OUTER_POSITION_KP
#ifndef FOC_MOTOR_SPEED_FILTER_S
#define FOC_MOTOR_SPEED_FILTER_S 0.001f
#endif
#ifndef FOC_OUTER_POSITION_SPEED_KP
#ifdef FOC_MOTOR_POSITION_SPEED_KP
#define FOC_OUTER_POSITION_SPEED_KP FOC_MOTOR_POSITION_SPEED_KP
#else
#define FOC_OUTER_POSITION_SPEED_KP SPEED_KP
#endif
#endif
#define CONTROL_BANDWIDTH 0.1f  /* Integrator back-calculation gain. */
#define CONTROL_PERIOD_US 1000u /* Outer-loop period; runs once per millisecond. */
#define CONTROL_JUMP_US 4000u   /* Gap above this is a discontinuity, not a dt. */
#ifdef FOC_DUAL_LEG
#define CONTROL_COMMAND_MS 500u
#else
#define CONTROL_COMMAND_MS 200u
#endif
static uint32_t mode, fault;
static float position, reference, speed, speed_target, position_target;
static float integral_speed, last_deg, speed_reference;
#ifdef FOC_DUAL_LEG
static float leg_position_reference, leg_reference_speed;
static bool leg_reference_external;
#endif
static float velocity_position, last_velocity_position;
static float absolute_deg;
static float position_origin;
static uint32_t observer_tick;
static uint32_t previous_tick, command_ms;
static bool tracking, commanded;
static volatile bool cog_learning;
static float cog_velocity, cog_velocity_position;
bool FOC_REALTIME control_cog_learning(void) { return cog_learning; }
bool control_cog_learn(void)
{
    if(foc.state!=FOC_IDLE || !foc.calibrated || !foc.zero_ready || (cog_flags()&6u))return false;
    cog_velocity=speed;cog_velocity_position=velocity_position;
    cog_learning=true;return true;
}

/* The free-shaft Longqiu needs stronger low-speed damping but less reaction
   to fast velocity ripple at higher speed. Interpolate on the shaped reference
   (not noisy measured speed); integral state is preserved across the range. */
#ifdef FOC_MOTOR_SPEED_HIGH_KP
static float speed_gain_blend(void)
{
    float blend = (fabsf(speed_reference) - FOC_MOTOR_SPEED_GAIN_LOW_RPM) /
        (FOC_MOTOR_SPEED_GAIN_HIGH_RPM - FOC_MOTOR_SPEED_GAIN_LOW_RPM);
    return fminf(1.0f, fmaxf(0.0f, blend));
}
#endif

float FOC_REALTIME control_iq_ref(void) { return reference; }
uint32_t FOC_REALTIME control_mode(void) { return mode; }
uint32_t FOC_REALTIME control_fault(void) { return fault; }
float FOC_REALTIME control_speed_rpm(void) { return speed; }
float FOC_REALTIME control_speed_target(void) { return speed_target; }
float FOC_REALTIME control_speed_reference(void) { return speed_reference; }
float FOC_REALTIME control_position_deg(void) { return position; }
float FOC_REALTIME control_position_target(void) { return position_target; }
float FOC_REALTIME control_absolute_deg(void) { return absolute_deg; }

/* WARN holds a target until the next command or explicit stop. */
bool FOC_REALTIME control_scheduled(void)
{
    return commanded && (!FOC_PROTECTION_TRIP ||
        (uint32_t)(bsp_uart_millis() - command_ms) < CONTROL_COMMAND_MS);
}

void control_stop(void)
{
    cog_learning=false;
    cog_stop();
    cog_cancel();
    mode = CONTROL_TORQUE;
    fault = 0u;
    commanded = false;
    reference = speed_target = position_target = 0.0f;
    integral_speed = 0.0f;
    speed_reference = 0.0f;
#ifdef FOC_DUAL_LEG
    leg_reference_external=false;leg_reference_speed=0;
#endif
}

static void FOC_REALTIME accept(void)
{
    command_ms = bsp_uart_millis();
    commanded = true;
    fault = 0u;
}

/* Shared enable test. Idle with a zero torque target must not start, matching
   the existing `Iq 0` semantics; speed and position mode always start, because
   holding zero speed or holding the present position is a real request to run.
   `stop` remains the only way down. */
static bool arm(uint32_t wanted, float target)
{
    if (wanted != CONTROL_TORQUE) return true;
    return foc.state != FOC_IDLE || target != 0.0f;
}

/* Start from idle through the existing PRECHARGE interlock, whose two
   milliseconds of all-low-side conduction are also what foc_calibrate uses.
   The target is already set by the caller; foc_stop() cleared the previous one. */
static void go(void)
{
    if (foc.state == FOC_IDLE) foc.state = FOC_PRECHARGE;
}

/* A resend must not disturb the loops. WARN also treats a new target after
   silence as a continuous run; only a new run or mode drops integrators. */
static bool FOC_REALTIME continuous(uint32_t wanted)
{
    return commanded && mode == wanted && (!FOC_PROTECTION_TRIP ||
        (uint32_t)(bsp_uart_millis() - command_ms) < CONTROL_COMMAND_MS);
}

static bool FOC_REALTIME start(uint32_t wanted, float target)
{
    if (!arm(wanted, target)) return false;
    if (!foc.calibrated || !foc.zero_ready) return false;
    if (foc.state != FOC_IDLE && foc.state != FOC_RUN &&
        foc.state != FOC_PRECHARGE && foc.state != FOC_PWM_ZERO) return false;
    if (!continuous(wanted)) {
        integral_speed = 0.0f;
        reference = 0.0f; /* Never reuse an old mode's outer-loop output. */
        speed_reference = speed;
#ifdef FOC_DUAL_LEG
        leg_position_reference = position;

        speed_reference = 0;
#endif
    }
    go();
    mode = wanted;
    accept();
    return true;
}

bool control_torque(float amps)
{
#ifdef FOC_DUAL_LEG
    (void)amps;return false;
#endif
    if(cog_learning)return false;
    if (!isfinite(amps)) return false;
    if (fabsf(amps) > FOC_CURRENT_MAX) {
        foc_warn(FOC_CURRENT);
        if (FOC_PROTECTION_TRIP) return false;
    }
    if (!start(CONTROL_TORQUE, amps)) return false;
    foc.command = amps; /* Rejected commands must not change the live target. */
    speed_target = 0.0f;
    return true;
}

bool control_speed(float rpm)
{
#ifdef FOC_DUAL_LEG
    (void)rpm;return false;
#endif
    if(cog_learning)return false;
    if (!isfinite(rpm)) return false;
    if (fabsf(rpm) > FOC_SPEED_MAX) {
        foc_warn(FOC_SPEED);
        if (FOC_PROTECTION_TRIP) return false;
    }
    if (!start(CONTROL_SPEED, rpm)) return false;
    speed_target = rpm;
    return true;
}

bool FOC_REALTIME control_position(float deg)
{
#ifdef FOC_DUAL_LEG
    if(!leg_target_valid(FOC_LEG_AXIS,deg,position))return false;
#endif
    if (!isfinite(deg)) return false;
    if (fabsf(deg) > 1e6f) {
        foc_warn(FOC_POSITION);
        if (FOC_PROTECTION_TRIP) return false;
    }
    if (!start(CONTROL_POSITION, deg)) return false;
    position_target = deg;
#ifdef FOC_DUAL_LEG
    leg_reference_external = false;
#endif
    return true;
}

#ifdef FOC_DUAL_LEG
bool FOC_REALTIME control_position_tracking(float deg, float rpm)
{
    if(!isfinite(rpm)||fabsf(rpm)>leg_motion_rate()||!isfinite(deg)||
       fabsf(deg)>1e6f||!leg_target_valid(FOC_LEG_AXIS,deg,position))return false;
    uint32_t now=bsp_uart_millis();
    if(mode==CONTROL_POSITION&&commanded&&!fault&&foc.state==FOC_RUN&&
       (uint32_t)(now-command_ms)<CONTROL_COMMAND_MS) {
        /* Continuous preflighted references need no enable/mode transition.
         * Retain range and expiry checks, refresh once, preserve integrators. */
        position_target=deg;command_ms=now;
    } else if(!control_position(deg))return false;
    leg_reference_speed=rpm;
    leg_reference_external=true;
    return true;
}
#endif

bool control_hold_position(void)
{
#ifdef FOC_DUAL_LEG
    return control_position(position);
#endif
    if (!start(CONTROL_POSITION, 0.0f)) return false;
    position_target = position; /* Stop where we are, but keep holding it. */
    return true;
}

bool control_zero(void)
{
    if (foc.state != FOC_IDLE || (FOC_PROTECTION_TRIP && fabsf(foc.rpm) >= 5.0f)) return false;
    position = 0.0f;
    tracking = false;
    return true;
}

static float cog_hold_output(float dt, float live_position, float velocity)
{
    /* Static capture needs stiffness against negative cogging slope,
     * without the normal position->speed cascade's extra phase lag.
     * All terms are physical Iq; direction is applied exactly once. */
    float error=(float)foc.calibration.direction*(position_target-live_position);
    float wanted=1.2f*error-.020f*(float)foc.calibration.direction*velocity+integral_speed;
    float limited=fminf(2.0f,fmaxf(-2.0f,wanted));
    if(foc.voltage_scale>=.999f || error*(wanted-foc.iq)<=0)
        integral_speed+=error*dt;
    integral_speed+=CONTROL_BANDWIDTH*(limited-wanted);
    integral_speed=fminf(1.5f,fmaxf(-1.5f,integral_speed));
    speed_target=speed_reference=0;
    return limited;
}
/* Position P -> speed trajectory -> speed PI using the outer velocity estimate. */
static FOC_REALTIME float outer_output(float dt)
{
    float acceleration = mode == CONTROL_POSITION ?
        FOC_MOTOR_POSITION_ACCEL_RPM_PER_S : FOC_MOTOR_SPEED_SLEW_RPM_PER_S;
#ifdef FOC_DUAL_LEG
    if(mode==CONTROL_POSITION) acceleration=fminf(150,leg_motion_acceleration());
#endif
#ifdef FOC_DUAL_LEG
    if (mode==CONTROL_POSITION && leg_reference_external) {
        /* The foot planner already supplies a bounded C2 trajectory. Avoid
         * shaping it a second time; use its mechanical velocity as damping
         * feed-forward. Current, range, effort slew and timeout stay active. */
        leg_position_reference=position_target;
        speed_reference=speed_target=leg_reference_speed;
    } else {
#endif
    if (mode == CONTROL_POSITION) {
#ifdef FOC_DUAL_LEG
        float distance = position_target - leg_position_reference;
#else
        float distance = position_target - position;
#endif
        float omega = POSITION_KP * distance;
        float ceiling = FOC_POSITION_SPEED_MAX;
#ifdef FOC_DUAL_LEG
        ceiling = fminf(ceiling, fminf(45,leg_motion_rate()));
#endif
        /* Degrees = 3 * RPM^2 / acceleration(RPM/s). Approach fast while
           distant; reserve enough remaining distance to brake. The linear P
           branch takes over near the target instead of a sharp sqrt cusp. */
#ifdef FOC_DUAL_LEG
        float brake_speed = sqrtf(fminf(150,leg_motion_acceleration()) * fabsf(distance) / 3.0f);
#else
        float brake_speed = sqrtf(FOC_MOTOR_POSITION_BRAKE_RPM_PER_S * fabsf(distance) / 3.0f);
#endif
        if (brake_speed < ceiling) ceiling = brake_speed;
        /* Position approach speed is a trajectory parameter, not a fault or
           target rejection. Always approach a distant target at this speed. */
        if (omega > ceiling) omega = ceiling;
        if (omega < -ceiling) omega = -ceiling;
        speed_target = omega;
    }
    /* Calibration.direction maps mechanical rotation to electrical rotation.
       Positive Iq follows the electrical direction, so invert both P and I
       for a motor whose calibrated mechanical direction is negative. */
    float change = speed_target - speed_reference;
    float slew = acceleration * dt;
    if (change > slew) change = slew;
    if (change < -slew) change = -slew;
    speed_reference += change;
#ifdef FOC_DUAL_LEG
    float distance=position_target-leg_position_reference;
    float next=leg_position_reference+6.0f*speed_reference*dt;
    if(distance*(position_target-next)<=0){next=position_target;speed_reference=0;}
    leg_position_reference=next;
    }
    /* Limited joint workspace: position stiffness supplies breakaway torque
     * without a long speed-integrator buildup. Measured motor velocity adds
     * damping; the reference trajectory remains bounded in motor units. */
    float error=(float)foc.calibration.direction*(leg_position_reference-position);
    float ki=FOC_LEG_POSITION_CURRENT_KI;
    /* Extra stiffness outside 0.3 joint degrees provides correction through
     * static gear friction; the quiet near-target slope remains unchanged. */
    float correction=copysignf(fmaxf(0, fabsf(error)-
        FOC_LEG_BREAKAWAY_DEADBAND_MOTOR_DEG),error);
    /* Schedule from the smooth trajectory, never from measured velocity.
     * A hard moving/quiet switch can toggle during mechanical ringing. */
    float motion_blend=fminf(1, fmaxf(0, (fabsf(speed_reference)-5)/15));
    float damping=FOC_LEG_POSITION_DAMPING_A_RPM+motion_blend*
        (FOC_LEG_MOVING_DAMPING_A_RPM-FOC_LEG_POSITION_DAMPING_A_RPM);
    float position_effort=FOC_LEG_POSITION_CURRENT_KP*error+
        FOC_LEG_BREAKAWAY_KP*correction;
    float catchup_effort=damping*FOC_LEG_POSITION_CATCHUP_RPM;
    position_effort=fminf(catchup_effort,fmaxf(-catchup_effort,position_effort));
    float wanted=position_effort-
        damping*(float)foc.calibration.direction*(speed-speed_reference)+integral_speed;
    bool speed_braking;
    /* Use the same fast mechanical observer as the damping term. The FOC
     * angle-prediction RPM has a separate 5 ms filter; that delay can keep
     * applying a brake after a geared joint has already reversed. */
    wanted=leg_speed_effort(wanted,speed,speed_reference,
        foc.calibration.direction,&speed_braking);
#else
    float error = (float)foc.calibration.direction * (speed_reference - speed);
    float kp = mode == CONTROL_POSITION ? FOC_OUTER_POSITION_SPEED_KP : SPEED_KP;
    float ki = SPEED_KI;
    /* Current Kp = bandwidth*L is not a bandwidth measurement. A different
     * inductance must not silently retune the mechanical PI. Outer gains
     * belong to the motor/load profile and need their own motion acceptance. */
#ifdef FOC_MOTOR_SPEED_HIGH_KP
    float blend = speed_gain_blend();
    if (mode == CONTROL_SPEED) kp += blend * (FOC_MOTOR_SPEED_HIGH_KP - kp);
#ifdef FOC_MOTOR_POSITION_HIGH_SPEED_KP
    else kp += blend * (FOC_MOTOR_POSITION_HIGH_SPEED_KP - kp);
#endif
    ki += blend * (FOC_MOTOR_SPEED_HIGH_KI - ki);
#endif
    /* During a fast reversal the shaped reference crosses zero while the
     * shaft is still moving quickly. Do not re-enable the low-speed map at
     * that instant, or add it to an already large acceleration demand. */
    float compensation_rpm=fmaxf(fabsf(speed_target),
        fmaxf(fabsf(speed_reference),fabsf(speed)));
    float wanted = kp * error + integral_speed + cog_output(absolute_deg, compensation_rpm, dt);
#endif
    float limited = wanted;
    if (fabsf(wanted) > FOC_CURRENT_MAX) foc_warn(FOC_CURRENT);
    if (FOC_PROTECTION_TRIP) {
        if (limited > FOC_CURRENT_MAX) limited = FOC_CURRENT_MAX;
        if (limited < -FOC_CURRENT_MAX) limited = -FOC_CURRENT_MAX;
    }
#ifdef FOC_MOTOR_OUTER_CURRENT_MAX
    /* New-motor actuator ceiling is independent of diagnostic policy. */
    if (limited > FOC_MOTOR_OUTER_CURRENT_MAX) limited = FOC_MOTOR_OUTER_CURRENT_MAX;
    if (limited < -FOC_MOTOR_OUTER_CURRENT_MAX) limited = -FOC_MOTOR_OUTER_CURRENT_MAX;
#endif
    #ifdef FOC_DUAL_LEG
    limited=fminf(leg_current_limit(),fmaxf(-leg_current_limit(),limited));
    float actuator_limited=limited;
    float change_limit=(FOC_LEG_EFFORT_SLEW_A_S+motion_blend*
        (FOC_LEG_MOVING_EFFORT_SLEW_A_S-FOC_LEG_EFFORT_SLEW_A_S))*dt;
    limited=reference+fminf(change_limit,fmaxf(-change_limit,limited-reference));
    limited=leg_limit_effort(FOC_LEG_AXIS,limited,foc.calibration.direction);
    #endif
    /* Do not wind up against unavailable inverter voltage. Preserve WARN's
       direct-current commands and do not add a current threshold lockout. */
    bool pushing_voltage = foc.voltage_scale < 0.999f &&
        error * (wanted - foc.iq) > 0.0f;
#ifdef FOC_DUAL_LEG
    /* Velocity-driven saturation must not create an opposite gravity bias.
     * Integrate only when the error unwinds a limit, or no limit is active. */
    bool integral_brakes=error*(float)foc.calibration.direction*speed<=0;
    if (!pushing_voltage && (!speed_braking || integral_brakes) &&
        (fabsf(wanted-actuator_limited)<.001f || error*(wanted-actuator_limited)<=0))
        integral_speed += ki*copysignf(
            fmaxf(0, fabsf(error)-FOC_LEG_INTEGRAL_DEADBAND_MOTOR_DEG), error)*dt;
    float bias_limit=fminf(FOC_LEG_INTEGRAL_MAX_A,leg_current_limit());
    integral_speed=fminf(bias_limit,fmaxf(-bias_limit,integral_speed));
#else
    if (!pushing_voltage) integral_speed += ki * error * dt;
    if (limited != wanted) integral_speed += CONTROL_BANDWIDTH * (limited - wanted);
#endif
    if (FOC_PROTECTION_TRIP) {
        if (integral_speed > FOC_CURRENT_MAX) integral_speed = FOC_CURRENT_MAX;
        if (integral_speed < -FOC_CURRENT_MAX) integral_speed = -FOC_CURRENT_MAX;
    }
#ifdef FOC_MOTOR_OUTER_CURRENT_MAX
    if (integral_speed > FOC_MOTOR_OUTER_CURRENT_MAX) integral_speed = FOC_MOTOR_OUTER_CURRENT_MAX;
    if (integral_speed < -FOC_MOTOR_OUTER_CURRENT_MAX) integral_speed = -FOC_MOTOR_OUTER_CURRENT_MAX;
#endif
    return limited;
}

FOC_REALTIME void control_step_with_angle(uint32_t sample_us, float mechanical_deg, float encoder_deg)
{
    if (foc.fault && !fault) fault = foc.fault; /* A trip latches this loop too. */
    if (!isfinite(mechanical_deg) || !isfinite(encoder_deg)) return;
    absolute_deg = encoder_deg;
    if (!tracking) {
        last_deg = velocity_position = last_velocity_position = mechanical_deg;
        position_origin=mechanical_deg-position;
        previous_tick = observer_tick = sample_us;
#if defined(FOC_DUAL_LEG) && FOC_LEG_AXIS == 1
        /* Half-period phase offset spreads the two 1 kHz math bursts; both
         * observers and current loops still consume every 10 kHz sample. */
        previous_tick = (sample_us-500u)&0xffffffu;
#endif
        speed = 0.0f;
        tracking = true;
        return;
    }
    uint32_t observation_us = (sample_us - observer_tick) & 0xffffffu;
    observer_tick = sample_us;
    if (observation_us > CONTROL_JUMP_US) {
        velocity_position = last_velocity_position = mechanical_deg;
        speed = 0.0f;
    } else {
        /* 0.2 ms angle prefilter for velocity only: use all encoder samples
           rather than noisy end points decimated to 1 kHz. No change to the
           position feedback or the current loop's electrical angle. */
        float observation_dt = (float)observation_us * 1e-6f;
        velocity_position += observation_dt / (0.0002f + observation_dt) *
            (mechanical_deg - velocity_position);
    }
    /* Static capture runs at the current-sample cadence. The normal 1 kHz
     * cascade is retained for user motion; its sampling delay is unsuitable
     * for a stiff hold on this very light free shaft. */
    if(cog_learning) {
        if(observation_us>CONTROL_JUMP_US) { fault=FOC_TIMING;return; }
        float dt=(float)observation_us*1e-6f;
        if(dt>0) {
            float measured=(velocity_position-cog_velocity_position)/(6*dt);
            cog_velocity += dt/(.0005f+dt)*(measured-cog_velocity);
        }
        cog_velocity_position=velocity_position;
        if(foc.state==FOC_RUN && commanded && dt>0) {
            if((uint32_t)(bsp_uart_millis()-command_ms)>=200u) { fault=FOC_UART;return; }
            reference=cog_hold_output(dt,mechanical_deg-position_origin,cog_velocity);
        }
    }
#ifdef FOC_DUAL_LEG
    /* Target conversion uses the latest leg observation at 10 kHz. Publish
     * position at the same cadence to avoid adding stale 1 kHz position to it. */
    position = mechanical_deg-position_origin;
#endif
    uint32_t elapsed = (sample_us - previous_tick) & 0xffffffu;
    if (elapsed < CONTROL_PERIOD_US) return;
    /* At the 9400 RPM limit, one millisecond moves less than 57 degrees. */
    last_deg = mechanical_deg;
    position = mechanical_deg-position_origin;
    if (elapsed > CONTROL_JUMP_US) {
        previous_tick = sample_us; /* Resynchronise without integrating the gap. */
#if defined(FOC_DUAL_LEG) && FOC_LEG_AXIS == 1
        previous_tick = (sample_us-500u)&0xffffffu;
#endif
        last_velocity_position = velocity_position;
        speed = 0.0f;
        return;
    }
    float dt = (float)elapsed * 1e-6f;
    previous_tick = sample_us;
    /* Estimate at the outer-loop cadence with a profile-selected time filter.
       This avoids differentiating encoder quantisation at 10/20 kHz and
       leaves the current loop's angle prediction unchanged. */
    float measured = (velocity_position - last_velocity_position) / (6.0f * dt);
    last_velocity_position = velocity_position;
    float filter_s = FOC_MOTOR_SPEED_FILTER_S;
#ifdef FOC_DUAL_LEG
    float motion_blend=fminf(1, fmaxf(0, (fabsf(speed_reference)-5)/15));
    filter_s+=motion_blend*(FOC_LEG_MOVING_FILTER_S-filter_s);
#endif
#ifdef FOC_MOTOR_SPEED_HIGH_KP
    filter_s += speed_gain_blend() * (FOC_MOTOR_SPEED_HIGH_FILTER_S - filter_s);
#endif
    if(cog_learning)filter_s=.0005f;
    speed += (dt / (filter_s + dt)) * (measured - speed);
    if (foc.state != FOC_RUN) { reference = 0.0f; cog_stop(); return; }
    if (!commanded) return;
    /* Learning has a hard keepalive even under WARN. A vanished host cannot
     * leave a calibration target energized indefinitely. */
    if(cog_learning && (uint32_t)(bsp_uart_millis()-command_ms)>=200u) {
        fault=FOC_UART;return;
    }
    /* Persistent WARN targets need no keepalive, including a position hold. */
    if (FOC_PROTECTION_TRIP && (uint32_t)(bsp_uart_millis() - command_ms) >= CONTROL_COMMAND_MS) {
        foc_warn(FOC_UART);
        fault = FOC_UART; return;
    }
    if (fault == FOC_UART) fault = 0u;
    if(cog_learning)return;
    if (mode == CONTROL_TORQUE) { cog_stop(); return; } /* Direct Iq excludes compensation. */
    reference = outer_output(dt);
}
void control_step(uint32_t sample_us, float mechanical_deg)
{
    /* Compatibility entry for host outer-loop fixtures. Hardware supplies
     * the instantaneous encoder phase separately from its multi-turn value. */
    control_step_with_angle(sample_us,mechanical_deg,
        mechanical_deg-360.0f*floorf(mechanical_deg/360.0f));
}
