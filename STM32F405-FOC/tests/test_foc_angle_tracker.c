#include "foc_angle_tracker.h"

#include <assert.h>
#include <math.h>

int main(void)
{
  foc_angle_tracker_t tracker;
  float speed = -1.0f;
  assert(foc_angle_tracker_init(&tracker, 100.0f, 0.01f, 1000u));
  assert(foc_angle_tracker_update(&tracker, true, 6.282f, 100u, &speed) ==
         FOC_ANGLE_PRIMING);
  assert(foc_angle_tracker_update(&tracker, true, 0.005f, 200u, &speed) ==
         FOC_ANGLE_VALID);
  assert(speed > 0.0f && speed < 100.0f);

  assert(foc_angle_tracker_update(&tracker, true, 2.0f, 300u, &speed) ==
         FOC_ANGLE_FAULT);
  assert(foc_angle_tracker_update(&tracker, true, 0.02f, 400u, &speed) ==
         FOC_ANGLE_FAULT);

  assert(foc_angle_tracker_init(&tracker, 100.0f, 0.01f, 1000u));
  assert(foc_angle_tracker_update(&tracker, true, 0.0f, 0u, &speed) ==
         FOC_ANGLE_PRIMING);
  assert(foc_angle_tracker_update(&tracker, false, 0.0f, 100u, &speed) ==
         FOC_ANGLE_FAULT);

  assert(foc_angle_tracker_init(&tracker, 100.0f, 0.01f, 1000u));
  assert(foc_angle_tracker_update(&tracker, true, 0.0f, 0u, &speed) ==
         FOC_ANGLE_PRIMING);
  assert(foc_angle_tracker_update(&tracker, true, 0.0f, 1001u, &speed) ==
         FOC_ANGLE_FAULT);

  assert(!foc_angle_tracker_init(&tracker, NAN, 0.01f, 1000u));
  assert(!foc_angle_tracker_init(&tracker, 1000.0f, 0.01f, 10000u));
  return 0;
}
