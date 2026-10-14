#include "foc.h"
#include "leg_guard.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static uint32_t tick;
static float q;
static void observe(float joint) {
  float a = fmodf(18 + 7 * joint, 360), b = fmodf(350 - 7 * joint, 360);
  if (a < 0)
    a += 360;
  if (b < 0)
    b += 360;
  tick += 100;
  assert(!leg_observe(0, a, tick) && !leg_observe(1, b, tick));
}
static void move(float wanted) {
  while (fabsf(wanted - q) > .002f) {
    q += copysignf(.002f, wanted - q);
    observe(q);
  }
  q = wanted;
  observe(q);
}
int main(void) {
  observe(0);
  assert(!leg_homed() && !leg_target_valid(0, 0, 0));
  /* Explicit operator home tolerates gear slack, but rejects readings outside
     the stop neighborhood. Sample both shafts while manually placing them. */
  for (unsigned n = 0; n < 20; ++n) {
    tick += 100;
    assert(!leg_observe(0, 18, tick));
    assert(!leg_observe(1, 350 - .5f * (n + 1), tick));
  }
  assert(leg_home());
  leg_unhome();
  for (unsigned n = 0; n < 10; ++n) {
    tick += 100;
    assert(!leg_observe(0, 18, tick));
    assert(!leg_observe(1, 340 - .5f * (n + 1), tick));
  }
  assert(!leg_home());
  for (unsigned n = 0; n < 30; ++n) {
    tick += 100;
    assert(!leg_observe(0, 18, tick));
    assert(!leg_observe(1, 335 + .5f * (n + 1), tick));
  }
  assert(leg_home());
  assert(leg_limit_effort(0, -1, 1) == 0 && leg_limit_effort(1, 1, 1) == 0);
  assert(leg_limit_effort(0, 1, 1) == 1 && leg_limit_effort(1, -1, 1) == -1);
  /* Gravity can back-drive an unpowered gear motor faster than its run limit.
   */
  tick += 100;
  assert(!leg_observe(0, 18.8f, tick) && leg_homed());
  tick += 100;
  assert(!leg_observe(0, 18, tick));
  leg_set_driven(true);
  move(110);
  assert(fabsf(leg_joint(0) - 110) < .001f &&
         fabsf(leg_joint(1) - 110) < .001f);
  assert(leg_limit_effort(0, 1, 1) == 0 && leg_limit_effort(1, -1, 1) == 0);
  float m0 = leg_motor_target(0, 25, 123), m1 = leg_motor_target(1, 25, -456);
  assert(fabsf(leg_joint_target(0, m0, 123) - 25) < .001f);
  assert(fabsf(leg_joint_target(1, m1, -456) - 25) < .001f);
  assert(leg_target_valid(0, m0, 123) && leg_target_valid(1, m1, -456));
  assert(!leg_target_valid(0, leg_motor_target(0, 111, 123), 123));
  move(111.5f);
  for (unsigned n = 0; n < 199; ++n)
    assert(!leg_motion_fault(true));
  assert(leg_motion_fault(true) == FOC_POSITION);
  assert(!leg_motion_fault(false)); /* Passive motion causes no drive trip. */
  move(110);
  assert(!leg_motion_fault(true));
  move(112.2f);
  assert(leg_motion_fault(true) == FOC_POSITION);
  tick += 500;
  assert(leg_observe(0, 18, tick) == FOC_SENSOR && !leg_homed());
  puts("PASS: 770 motor degrees, mirrored coordinates, outward effort blocked, "
       "20ms transient allowance, 2deg hard excursion, gate-off tracking and "
       "gap invalidation");
}
