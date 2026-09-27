#include "control.h"
#include "bsp_uart.h"
#include "foc.h"
#include <math.h>

/* Basic cascade: position P -> speed PI -> Iq_ref, above the selected port's
   current loop. TRIP limits match the manual current command range; WARN
   records the same thresholds and leaves the calculated reference intact.
   Tuning order: speed Kp first until it tracks without oscillating, then speed
   Ki to remove the steady-state error, then position Kp. */
#define SPEED_KP FOC_SPEED_KP
#define SPEED_KI FOC_SPEED_KI
#define POSITION_KP FOC_POSITION_KP
#define CONTROL_BANDWIDTH 0.1f  /* Integrator back-calculation gain. */
#define CONTROL_PERIOD_US 1000u /* Outer-loop period; runs once per millisecond. */
#define CONTROL_JUMP_US 4000u   /* Gap above this is a discontinuity, not a dt. */
#define CONTROL_COMMAND_MS 200u /* Stop if the host stops sending mode targets. */
static uint32_t mode, fault;
static float position, reference, speed, speed_target, position_target;
static float integral_speed, last_deg;
static uint32_t previous_tick, command_ms;
static bool tracking, commanded;

float control_iq_ref(void) { return reference; }
uint32_t control_mode(void) { return mode; }
uint32_t control_fault(void) { return fault; }
float control_speed_rpm(void) { return speed; }
float control_speed_target(void) { return speed_target; }
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
    if (foc.state != FOC_IDLE && foc.state != FOC_RUN && foc.state != FOC_PRECHARGE) return false;
    if (!continuous(wanted)) integral_speed = 0.0f;
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

/* Speed PI with the position P above it. foc.rpm is already encoder-filtered. */
static float outer_output(float dt)
{
    if (mode == CONTROL_POSITION) {
        float omega = POSITION_KP * (position_target - position);
        float ceiling = FOC_POSITION_SPEED_MAX;
        if (fabsf(omega) > ceiling) foc_warn(FOC_SPEED);
        if (FOC_PROTECTION_TRIP) {
            if (omega > ceiling) omega = ceiling;
            if (omega < -ceiling) omega = -ceiling;
        }
        speed_target = omega;
    }
    /* Calibration.direction maps mechanical rotation to electrical rotation.
       Positive Iq follows the electrical direction, so invert both P and I
       for a motor whose calibrated mechanical direction is negative. */
    float error = (float)foc.calibration.direction * (speed_target - speed);
    float wanted = SPEED_KP * error + integral_speed;
    float limited = wanted;
    if (fabsf(wanted) > FOC_CURRENT_MAX) foc_warn(FOC_CURRENT);
    if (FOC_PROTECTION_TRIP) {
        if (limited > FOC_CURRENT_MAX) limited = FOC_CURRENT_MAX;
        if (limited < -FOC_CURRENT_MAX) limited = -FOC_CURRENT_MAX;
    }
    integral_speed += SPEED_KI * error * dt;
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
    uint32_t elapsed = (sample_us - previous_tick) & 0xffffffu;
    if (elapsed < CONTROL_PERIOD_US) return;
    /* At the 9400 RPM limit, one millisecond moves less than 57 degrees. */
    if (!tracking) { last_deg = mechanical_deg; tracking = true; }
    float step_deg = mechanical_deg - last_deg; /* foc_step already unwraps. */
    last_deg = mechanical_deg;
    position += step_deg;
    if (elapsed > CONTROL_JUMP_US) {
        previous_tick = sample_us; /* Resynchronise without integrating the gap. */
        return;
    }
    float dt = (float)elapsed * 1e-6f;
    previous_tick = sample_us;
    speed = foc.rpm;
    if (!commanded) return;
    /* TRIP needs a live host. WARN reports silence and retains all targets. */
    if ((uint32_t)(bsp_uart_millis() - command_ms) >= CONTROL_COMMAND_MS) {
        foc_warn(FOC_UART);
        if (FOC_PROTECTION_TRIP) { fault = FOC_UART; return; }
    }
    if (fault == FOC_UART) fault = 0u;
    if (mode == CONTROL_TORQUE) return; /* Its Iq ramp lives in foc_step(). */
    reference = outer_output(dt);
}
