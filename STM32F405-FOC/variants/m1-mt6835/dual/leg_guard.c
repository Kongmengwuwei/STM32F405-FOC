#include "leg_guard.h"
#include "foc.h"
#include "realtime.h"
#include <math.h>
#include <string.h>
typedef struct {
  bool ready;
  float start, previous, travel, speed_squared, speed_angle;
  uint32_t speed_tick;
  int32_t turns;
  uint32_t tick;
} leg_axis_t;
static leg_axis_t axes[2];
static volatile unsigned teaching;
static volatile bool homed;
static bool driven;
void leg_set_driven(bool on) { driven = on; }
static float home_motor[2];
static unsigned excursion;
int leg_direction(unsigned i) {
  return i ? LEG_DIRECTION_M1 : LEG_DIRECTION_M0;
}
bool leg_homed(void) { return homed; }
float leg_speed_rms(unsigned i) {
  return i < 2 && axes[i].ready ? sqrtf(axes[i].speed_squared) : NAN;
}
void leg_unhome(void) { homed = false; }
void leg_clear_tracking_fault(void) {
  teaching = 0;
  homed = false;
}
static float motor_angle(unsigned i) {
  return (float)axes[i].turns * 360 + axes[i].previous;
}
float DUAL_REALTIME leg_joint(unsigned i) {
  return homed && i < 2 ? leg_direction(i) * (motor_angle(i) - home_motor[i]) /
                              LEG_GEAR_RATIO
                        : 0;
}
float DUAL_REALTIME leg_joint_target(unsigned i, float t, float p) {
  return leg_joint(i) + leg_direction(i) * (t - p) / LEG_GEAR_RATIO;
}
float DUAL_REALTIME leg_motor_target(unsigned i, float q, float p) {
  return p + leg_direction(i) * LEG_GEAR_RATIO * (q - leg_joint(i));
}
uint32_t DUAL_REALTIME leg_motion_fault(bool driven) {
  if (!driven) {
    excursion = 0;
    return 0;
  }
  if (!homed)
    return FOC_POSITION;
  if (axes[0].speed_squared >
          LEG_SPEED_TRIP_MOTOR_RPM * LEG_SPEED_TRIP_MOTOR_RPM ||
      axes[1].speed_squared >
          LEG_SPEED_TRIP_MOTOR_RPM * LEG_SPEED_TRIP_MOTOR_RPM)
    return FOC_SPEED;
  float q0 = leg_joint(0), q1 = leg_joint(1);
  if (q0 < -2 || q0 > 112 || q1 < -2 || q1 > 112)
    return FOC_POSITION;
  if (q0 < -1 || q0 > 111 || q1 < -1 || q1 > 111) {
    if (++excursion >= FOC_SAMPLE_HZ / 50)
      return FOC_POSITION;
  } else
    excursion = 0;
  return 0;
}
float DUAL_REALTIME leg_limit_effort(unsigned i, float iq,
                                     int electrical_direction) {
  if (!homed)
    return 0;
  float q = leg_joint(i), inward = leg_direction(i) * electrical_direction * iq;
  if ((q <= 0 && inward < 0) || (q >= 110 && inward > 0))
    return 0;
  return iq;
}
void leg_teach_begin(void) {
  memset(axes, 0, sizeof axes);
  teaching = 1;
  homed = false;
}
void leg_teach_invalidate(void) {
  if (teaching)
    teaching = 2;
}
unsigned leg_teach_state(void) { return teaching; }
float leg_reference(unsigned i) { return leg_ready(i) ? axes[i].start : NAN; }
static volatile float current_limit = 10.0f;
static volatile float motion_rate = 30.0f;
float DUAL_REALTIME leg_motion_rate(void) { return motion_rate; }
float DUAL_REALTIME leg_motion_acceleration(void) {
  /* Preserve the qualified <=45 RPM behavior; stage acceleration with speed.
   * Units: motor RPM/s. Existing current/effort/overspeed guards still apply.
   */
  return motion_rate <= 45 ? 150 : 150 + 10 * (motion_rate - 45);
}
bool leg_set_motion_rate(float rpm) {
  if (!isfinite(rpm) || rpm < 10 || rpm > 120)
    return false;
  motion_rate = rpm;
  return true;
}
bool DUAL_REALTIME leg_ready(unsigned i) { return i < 2 && axes[i].ready; }
float leg_travel(unsigned i) {
  return homed ? leg_joint(i) : (i < 2 ? axes[i].travel : 0);
}
float DUAL_REALTIME leg_speed_effort(float wanted, float rpm, float reference,
                                     int direction, bool *braking) {
  *braking = false;
  if (!isfinite(wanted) || !isfinite(rpm) || !isfinite(reference) ||
      (direction != 1 && direction != -1)) {
    *braking = true;
    return 0;
  }
  float ceiling = fabsf(reference);
  if (ceiling > motion_rate)
    ceiling = motion_rate;
  ceiling += LEG_SPEED_MARGIN_RPM;
  float excess = fabsf(rpm) - ceiling;
  if (excess <= 0)
    return wanted;
  *braking = true;
  float sign = (float)direction * (rpm < 0 ? -1.0f : 1.0f);
  /* Add damping continuously from zero at the threshold. Forcing total Iq
   * across zero here discards the gravity bias and creates a torque step on
   * every geared reversal. Position catch-up is bounded by the caller. */
  return wanted - sign * LEG_SPEED_BRAKE_A_RPM * excess;
}
float DUAL_REALTIME leg_current_limit(void) { return current_limit; }
bool leg_set_current_limit(float a) {
  if (!isfinite(a) || a < .2f || a > LEG_CURRENT_MAX)
    return false;
  current_limit = a;
  return true;
}
static float difference(float a, float b) {
  float d = a - b;
  if (d > 180)
    d -= 360;
  if (d < -180)
    d += 360;
  return d;
}
unsigned leg_home_check(void) {
  if (!leg_ready(0) || !leg_ready(1) || teaching == 2)
    return LEG_HOME_TRACKING;
  unsigned failure = 0;
  if (fabsf(difference(axes[0].previous, LEG_REFERENCE_RAW_M0)) >
      LEG_HOME_RAW_TOLERANCE_DEG)
    failure |= LEG_HOME_M0;
  if (fabsf(difference(axes[1].previous, LEG_REFERENCE_RAW_M1)) >
      LEG_HOME_RAW_TOLERANCE_DEG)
    failure |= LEG_HOME_M1;
  return failure;
}
unsigned leg_restore_prepare(leg_restore_t *plan) {
  if (!plan || !leg_ready(0) || !leg_ready(1) || teaching == 2)
    return LEG_HOME_TRACKING;
  const volatile leg_axis_t *view = axes;
  float raw[2], motor[2];
  bool coherent = false;
  for (unsigned attempt = 0; attempt < 4; ++attempt) {
    uint32_t tick0 = view[0].tick, tick1 = view[1].tick;
    for (unsigned i = 0; i < 2; ++i) {
      raw[i] = view[i].previous;
      motor[i] = (float)view[i].turns * 360 + raw[i];
    }
    /* A sampling interrupt can update the pair, but never interleave with
     * itself. Retry rather than mask ADC while solving possible turns. */
    if (tick0 == tick1 && tick0 == view[0].tick && tick1 == view[1].tick) {
      coherent = true;
      break;
    }
  }
  if (!coherent)
    return LEG_HOME_TRACKING;
  const float reference[2] = {LEG_REFERENCE_RAW_M0, LEG_REFERENCE_RAW_M1};
  const float period = 360.0f / LEG_GEAR_RATIO;
  if (!(LEG_RESTORE_MAX_DEG - LEG_RESTORE_MIN_DEG < period))
    return LEG_HOME_TRACKING;
  leg_restore_t candidate;
  unsigned failure = 0;
  for (unsigned i = 0; i < 2; ++i) {
    float phase =
        leg_direction(i) * difference(raw[i], reference[i]) / LEG_GEAR_RATIO;
    unsigned count = 0;
    float joint = 0;
    for (int turn = -1; turn <= 1; ++turn) {
      float q = phase + turn * period;
      if (q >= LEG_RESTORE_MIN_DEG && q <= LEG_RESTORE_MAX_DEG) {
        joint = q;
        ++count;
      }
    }
    if (count != 1)
      failure |= i ? LEG_HOME_M1 : LEG_HOME_M0;
    candidate.motor_zero[i] =
        motor[i] - leg_direction(i) * LEG_GEAR_RATIO * joint;
  }
  if (!failure)
    *plan = candidate; /* Rejection never changes the caller's candidate. */
  return failure;
}
bool leg_restore_apply(const leg_restore_t *plan) {
  if (!plan || homed || !leg_ready(0) || !leg_ready(1) || teaching == 2 ||
      !isfinite(plan->motor_zero[0]) || !isfinite(plan->motor_zero[1]))
    return false;
  home_motor[0] = plan->motor_zero[0];
  home_motor[1] = plan->motor_zero[1];
  axes[0].speed_squared = axes[1].speed_squared = 0;
  teaching = 0;
  excursion = 0;
  homed = true;
  return true;
}
static bool capture_home(void) {
  if (!leg_ready(0) || !leg_ready(1) || teaching == 2)
    return false;
  home_motor[0] = motor_angle(0);
  home_motor[1] = motor_angle(1);
  axes[0].speed_squared = axes[1].speed_squared = 0;
  teaching = 0;
  homed = true;
  excursion = 0;
  return true;
}
bool leg_home(void) { return !leg_home_check() && capture_home(); }
/* Explicit human confirmation of the original physical stop; never infer it
 * from an arbitrary single-turn encoder angle. Caller enforces idle/freshness.
 */
bool leg_home_confirmed_stop(void) { return capture_home(); }
uint32_t DUAL_REALTIME leg_observe(unsigned i, float angle, uint32_t tick) {
  if (i > 1 || !isfinite(angle) || angle < 0 || angle >= 360)
    return FOC_SENSOR;
  if (!axes[i].ready) {
    axes[i].start = axes[i].previous = angle;
    axes[i].tick = tick;
    axes[i].speed_tick = tick;
    axes[i].speed_angle = angle;
    axes[i].ready = true;
    return 0;
  }
  uint32_t elapsed = (tick - axes[i].tick) & 0xffffffu;
  if ((teaching || homed) && (!elapsed || elapsed > 300u)) {
    leg_teach_invalidate();
    leg_unhome();
    return FOC_SENSOR;
  }
  float step = difference(angle, axes[i].previous);
  uint32_t window = (tick - axes[i].speed_tick) & 0xffffffu;
  if (window >= 1000u) {
    /* TLE angle noise differentiated over 100 us can dominate the squared
     * speed, even at standstill. Use 1 ms net displacement for this independent
     * RMS guard, with the same 5 ms envelope and unchanged 180 RPM threshold.
     * Position, CRC, sample-gap and per-sample jump checks stay at 10 kHz. */
    if (window <= 1300u) {
      float rpm = difference(angle, axes[i].speed_angle) *
                  (1000000.0f / (6.0f * (float)window));
      float alpha = (float)window / (5000.0f + (float)window);
      axes[i].speed_squared += alpha * (rpm * rpm - axes[i].speed_squared);
    } else {
      axes[i].speed_squared = 0;
    }
    axes[i].speed_tick = tick;
    axes[i].speed_angle = angle;
  }
  float wrapped = angle - axes[i].previous;
  if (wrapped > 180)
    --axes[i].turns;
  if (wrapped < -180)
    ++axes[i].turns;
  axes[i].travel =
      ((float)axes[i].turns * 360.0f + angle - axes[i].start) / LEG_GEAR_RATIO;
  axes[i].previous = angle;
  __asm__ __volatile__("" ::: "memory"); /* Publish this axis snapshot last. */
  axes[i].tick = tick;
  /* Off-state motion is tracked without freezing the encoder acquisition. */
  if (elapsed && elapsed <= 300u &&
      fabsf(step) > (driven ? FOC_MAX_STEP_DEG : LEG_PASSIVE_STEP_DEG))
    return FOC_SENSOR;
  return 0;
}
bool DUAL_REALTIME leg_target_valid(unsigned i, float target, float position) {
  if (teaching || !homed || !leg_ready(i) || !isfinite(target) ||
      !isfinite(position))
    return false;
  float q = leg_joint_target(i, target, position);
  return q >= LEG_JOINT_MIN - .0001f && q <= LEG_JOINT_MAX + .0001f;
}
