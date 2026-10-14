#include "leg_guard.h"
#include "foc.h"
#include <math.h>
#include <string.h>
static struct {
  bool ready;
  float start, previous, travel, speed_squared;
  int32_t turns;
  uint32_t tick;
} axes[2];
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
void leg_unhome(void) { homed = false; }
void leg_clear_tracking_fault(void) {
  teaching = 0;
  homed = false;
}
static float motor_angle(unsigned i) {
  return (float)axes[i].turns * 360 + axes[i].previous;
}
float leg_joint(unsigned i) {
  return homed && i < 2 ? leg_direction(i) * (motor_angle(i) - home_motor[i]) /
                              LEG_GEAR_RATIO
                        : 0;
}
float leg_joint_target(unsigned i, float t, float p) {
  return leg_joint(i) + leg_direction(i) * (t - p) / LEG_GEAR_RATIO;
}
float leg_motor_target(unsigned i, float q, float p) {
  return p + leg_direction(i) * LEG_GEAR_RATIO * (q - leg_joint(i));
}
uint32_t leg_motion_fault(bool driven) {
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
float leg_limit_effort(unsigned i, float iq, int electrical_direction) {
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
float leg_motion_rate(void) { return motion_rate; }
bool leg_set_motion_rate(float rpm) {
  if (!isfinite(rpm) || rpm < 10 || rpm > 60)
    return false;
  motion_rate = rpm;
  return true;
}
bool leg_ready(unsigned i) { return i < 2 && axes[i].ready; }
float leg_travel(unsigned i) {
  return homed ? leg_joint(i) : (i < 2 ? axes[i].travel : 0);
}
float leg_current_limit(void) { return current_limit; }
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
bool leg_home(void) {
  if (!leg_ready(0) || !leg_ready(1) || teaching == 2)
    return false;
  if (fabsf(difference(axes[0].previous, LEG_REFERENCE_RAW_M0)) > LEG_HOME_RAW_TOLERANCE_DEG ||
      fabsf(difference(axes[1].previous, LEG_REFERENCE_RAW_M1)) > LEG_HOME_RAW_TOLERANCE_DEG)
    return false;
  home_motor[0] = motor_angle(0);
  home_motor[1] = motor_angle(1);
  axes[0].speed_squared = axes[1].speed_squared = 0;
  teaching = 0;
  homed = true;
  excursion = 0;
  return true;
}
uint32_t leg_observe(unsigned i, float angle, uint32_t tick) {
  if (i > 1 || !isfinite(angle) || angle < 0 || angle >= 360)
    return FOC_SENSOR;
  if (!axes[i].ready) {
    axes[i].start = axes[i].previous = angle;
    axes[i].tick = tick;
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
  if (elapsed && elapsed <= 300u) {
    float rpm = step * (1000000.0f / (6.0f * (float)elapsed));
    axes[i].speed_squared += .02f * (rpm * rpm - axes[i].speed_squared);
  }
  float wrapped = angle - axes[i].previous;
  if (wrapped > 180)
    --axes[i].turns;
  if (wrapped < -180)
    ++axes[i].turns;
  axes[i].travel =
      ((float)axes[i].turns * 360.0f + angle - axes[i].start) / LEG_GEAR_RATIO;
  axes[i].previous = angle;
  axes[i].tick = tick;
  /* Off-state motion is tracked without freezing the encoder acquisition. */
  if (elapsed && elapsed <= 300u &&
      fabsf(step) > (driven ? FOC_MAX_STEP_DEG : LEG_PASSIVE_STEP_DEG))
    return FOC_SENSOR;
  return 0;
}
bool leg_target_valid(unsigned i, float target, float position) {
  if (teaching || !homed || !leg_ready(i) || !isfinite(target) ||
      !isfinite(position))
    return false;
  float q = leg_joint_target(i, target, position);
  return q >= LEG_JOINT_MIN - .0001f && q <= LEG_JOINT_MAX + .0001f;
}
