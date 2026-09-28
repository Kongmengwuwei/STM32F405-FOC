#include "control.h"
#include "bsp_uart.h"
#include "foc.h"
#include <math.h>

/* Basic cascade: position P -> speed PI -> Iq_ref, above the selected port's
   current loop. TRIP limits match the manual current command range; WARN
   records the same thresholds and leaves the calculated reference intact.
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
#define CONTROL_COMMAND_MS 200u /* TRIP only: require refreshed host targets. */
static uint32_t mode, fault;
static float position, reference, speed, speed_target, position_target;
static float integral_speed, last_deg, speed_reference;
static float velocity_position, last_velocity_position;
static uint32_t observer_tick;
static uint32_t previous_tick, command_ms;
static bool tracking, commanded;

float control_iq_ref(void) { return reference; }
uint32_t control_mode(void) { return mode; }
uint32_t control_fault(void) { return fault; }
float control_speed_rpm(void) { return speed; }
float control_speed_target(void) { return speed_target; }
float control_speed_reference(void) { return speed_reference; }
float control_position_deg(void) { return position; }
float control_position_target(void) { return position_target; }

/* WARN holds a target until the next command or explicit stop. */
bool control_scheduled(void)
{
    return commanded && (!FOC_PROTECTION_TRIP ||
        (uint32_t)(bsp_uart_millis() - command_ms) < CONTROL_COMMAND_MS);
}

void control_stop(void)
{
    mode = CONTROL_TORQUE;
    fault = 0u;
    commanded = false;
    reference = speed_target = position_target = 0.0f;
    integral_speed = 0.0f;
    speed_reference = 0.0f;
}

static void accept(void)
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
static bool go(void)
{
    if (foc.state != FOC_IDLE) return true;
    foc.state = FOC_PRECHARGE;
    return true;
}

/* A resend must not disturb the loops. WARN also treats a new target after
   silence as a continuous run; only a new run or mode drops integrators. */
static bool continuous(uint32_t wanted)
{
    return commanded && mode == wanted && (!FOC_PROTECTION_TRIP ||
        (uint32_t)(bsp_uart_millis() - command_ms) < CONTROL_COMMAND_MS);
}

static bool start(uint32_t wanted, float target)
{
    if (!arm(wanted, target)) return false;
    if (!foc.calibrated || !foc.zero_ready) return false;
    if (foc.state != FOC_IDLE && foc.state != FOC_RUN &&
        foc.state != FOC_PRECHARGE && foc.state != FOC_PWM_ZERO) return false;
    if (!continuous(wanted)) {
        integral_speed = 0.0f;
        reference = 0.0f; /* Never reuse an old mode's outer-loop output. */
        speed_reference = speed;
    }
    if (!go()) return false;
    mode = wanted;
    accept();
    return true;
}

bool control_torque(float amps)
{
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
    if (!isfinite(rpm)) return false;
    if (fabsf(rpm) > FOC_SPEED_MAX) {
        foc_warn(FOC_SPEED);
        if (FOC_PROTECTION_TRIP) return false;
    }
    if (!start(CONTROL_SPEED, rpm)) return false;
    speed_target = rpm;
    return true;
}

bool control_position(float deg)
{
    if (!isfinite(deg)) return false;
    if (fabsf(deg) > 1e6f) {
        foc_warn(FOC_POSITION);
        if (FOC_PROTECTION_TRIP) return false;
    }
    if (!start(CONTROL_POSITION, deg)) return false;
    position_target = deg;
    return true;
}

bool control_hold_position(void)
{
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

/* Position P -> speed trajectory -> speed PI using the outer velocity estimate. */
static float outer_output(float dt)
{
    float acceleration = mode == CONTROL_POSITION ?
        FOC_MOTOR_POSITION_ACCEL_RPM_PER_S : FOC_MOTOR_SPEED_SLEW_RPM_PER_S;
    if (mode == CONTROL_POSITION) {
        float distance = position_target - position;
        float omega = POSITION_KP * distance;
        float ceiling = FOC_POSITION_SPEED_MAX;
        /* Degrees = 3 * RPM^2 / acceleration(RPM/s). Approach fast while
           distant; reserve enough remaining distance to brake. The linear P
           branch takes over near the target instead of a sharp sqrt cusp. */
        float brake_speed = sqrtf(FOC_MOTOR_POSITION_BRAKE_RPM_PER_S * fabsf(distance) / 3.0f);
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
    float error = (float)foc.calibration.direction * (speed_reference - speed);
    float kp = mode == CONTROL_POSITION ? FOC_OUTER_POSITION_SPEED_KP : SPEED_KP;
    float wanted = kp * error + integral_speed;
    float limited = wanted;
    if (fabsf(wanted) > FOC_CURRENT_MAX) foc_warn(FOC_CURRENT);
    if (FOC_PROTECTION_TRIP) {
        if (limited > FOC_CURRENT_MAX) limited = FOC_CURRENT_MAX;
        if (limited < -FOC_CURRENT_MAX) limited = -FOC_CURRENT_MAX;
    }
    /* Do not wind up against unavailable inverter voltage. Preserve WARN's
       direct-current commands and do not add a current threshold lockout. */
    bool pushing_voltage = foc.voltage_scale < 0.999f &&
        error * (wanted - foc.iq) > 0.0f;
    if (!pushing_voltage) integral_speed += SPEED_KI * error * dt;
    if (limited != wanted) integral_speed += CONTROL_BANDWIDTH * (limited - wanted);
    if (FOC_PROTECTION_TRIP) {
        if (integral_speed > FOC_CURRENT_MAX) integral_speed = FOC_CURRENT_MAX;
        if (integral_speed < -FOC_CURRENT_MAX) integral_speed = -FOC_CURRENT_MAX;
    }
    return limited;
}

void control_step(uint32_t sample_us, float mechanical_deg)
{
    if (foc.fault && !fault) fault = foc.fault; /* A trip latches this loop too. */
    if (!isfinite(mechanical_deg)) return;
    if (!tracking) {
        last_deg = velocity_position = last_velocity_position = mechanical_deg;
        previous_tick = observer_tick = sample_us;
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
    uint32_t elapsed = (sample_us - previous_tick) & 0xffffffu;
    if (elapsed < CONTROL_PERIOD_US) return;
    /* At the 9400 RPM limit, one millisecond moves less than 57 degrees. */
    float step_deg = mechanical_deg - last_deg; /* foc_step already unwraps. */
    last_deg = mechanical_deg;
    position += step_deg;
    if (elapsed > CONTROL_JUMP_US) {
        previous_tick = sample_us; /* Resynchronise without integrating the gap. */
        last_velocity_position = velocity_position;
        speed = 0.0f;
        return;
    }
    float dt = (float)elapsed * 1e-6f;
    previous_tick = sample_us;
    /* Estimate at the outer-loop cadence with a time-based 1 ms filter.
       This avoids differentiating encoder quantisation at 10/20 kHz and
       leaves the current loop's angle prediction unchanged. */
    float measured = (velocity_position - last_velocity_position) / (6.0f * dt);
    last_velocity_position = velocity_position;
    speed += (dt / (0.001f + dt)) * (measured - speed);
    if (foc.state != FOC_RUN) { reference = 0.0f; return; }
    if (!commanded) return;
    /* Persistent WARN targets need no keepalive, including a position hold. */
    if (FOC_PROTECTION_TRIP && (uint32_t)(bsp_uart_millis() - command_ms) >= CONTROL_COMMAND_MS) {
        foc_warn(FOC_UART);
        fault = FOC_UART; return;
    }
    if (fault == FOC_UART) fault = 0u;
    if (mode == CONTROL_TORQUE) return; /* Its Iq ramp lives in foc_step(). */
    reference = outer_output(dt);
}
