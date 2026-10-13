#include "leg_guard.h"
#include "foc.h"
#include <math.h>
static struct {
  bool ready;
  float start, previous, travel;
  int32_t turns;
  uint32_t tick;
} axes[2];
static volatile float current_limit = 5.0f;
bool leg_ready(unsigned i) { return i < 2 && axes[i].ready; }
float leg_travel(unsigned i) { return i < 2 ? axes[i].travel : 0; }
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
  float step = difference(angle, axes[i].previous);
  float wrapped = angle - axes[i].previous;
  if (wrapped > 180)
    --axes[i].turns;
  if (wrapped < -180)
    ++axes[i].turns;
  axes[i].travel =
      ((float)axes[i].turns * 360.0f + angle - axes[i].start) / LEG_GEAR_RATIO;
  axes[i].previous = angle;
  axes[i].tick = tick;
  if (fabsf(axes[i].travel) > LEG_TRAVEL_DEG)
    return FOC_POSITION;
  if (elapsed && elapsed <= 300u && fabsf(step) > FOC_MAX_STEP_DEG)
    return FOC_SENSOR;
  return 0;
}
bool leg_target_valid(unsigned i, float target, float position) {
  return leg_ready(i) && isfinite(target) && isfinite(position) &&
         fabsf(axes[i].travel + (target - position) / LEG_GEAR_RATIO) <=
             LEG_TARGET_DEG;
}
