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
static void restore_tests(void) {
  const float samples[] = {-1.5f, 0,     .1f, 3,  5,     12,
                           20,    25.7f, 26,  36, 39.9f, 40};
  for (unsigned k = 0; k < sizeof samples / sizeof samples[0]; ++k) {
    leg_teach_begin();
    observe(samples[k]); /* Cold-start single turn, no prior motor turns. */
    leg_restore_t plan;
    assert(!leg_restore_prepare(&plan) && leg_restore_apply(&plan));
    assert(fabsf(leg_joint(0) - samples[k]) < .0001f &&
           fabsf(leg_joint(1) - samples[k]) < .0001f);
    assert(!leg_restore_apply(&plan)); /* Cannot reinterpret valid tracking. */
    q = samples[k];
    move(60); /* Subsequent wrap tracking crosses the recovery interval. */
    assert(fabsf(leg_joint(0) - 60) < .001f &&
           fabsf(leg_joint(1) - 60) < .001f);
  }
  for (unsigned k = 0; k < 2; ++k) {
    leg_teach_begin();
    observe(k ? -3 : 45);
    leg_restore_t plan = {{123, 456}};
    assert(leg_restore_prepare(&plan) == (LEG_HOME_M0 | LEG_HOME_M1));
    assert(plan.motor_zero[0] == 123 && plan.motor_zero[1] == 456 &&
           !leg_homed());
  }
  leg_teach_begin();
  observe(5);
  leg_teach_invalidate();
  leg_restore_t plan;
  assert(leg_restore_prepare(&plan) == LEG_HOME_TRACKING);
  assert(!leg_restore_apply(NULL));
  leg_teach_begin();
  observe(0);
  assert(leg_home());
  q = 0;
  leg_unhome();
}
static void speed_brake_tests(void) {
  assert(leg_set_motion_rate(120));
  for (int direction = -1; direction <= 1; direction += 2) {
    for (int sign = -1; sign <= 1; sign += 2) {
      bool braking;
      float effort = leg_speed_effort(10 * sign * direction, 150 * sign,
                                      96 * sign, direction, &braking);
      assert(braking && fabsf(effort - direction * sign * 9.32f) < .001f);
      effort = leg_speed_effort(-8 * sign * direction, 150 * sign, -96 * sign,
                                direction, &braking);
      assert(braking && fabsf(effort + 8.68f * sign * direction)<.001f);
      effort = leg_speed_effort(4 * sign * direction, 100 * sign, 96 * sign,
                                direction, &braking);
      assert(!braking && effort == 4 * sign * direction);
      effort = leg_speed_effort(10 * sign * direction, 35 * sign, 0, direction,
                                &braking);
      assert(braking && fabsf(effort - direction * sign * 9.7f) < .001f);
      /* A load-supporting bias must not jump when velocity crosses the
       * soft threshold. Test all electrical/shaft direction combinations. */
      float at=leg_speed_effort(2*sign*direction,20*sign,0,direction,&braking);
      float above=leg_speed_effort(2*sign*direction,20.01f*sign,0,direction,&braking);
      assert(fabsf(at-above)<.001f);
    }
  }
  bool braking;
  assert(leg_speed_effort(1, NAN, 0, 1, &braking) == 0 && braking);
  assert(leg_set_motion_rate(30));
}
static void speed_noise_tests(void) {
  /* Alternating 0.08 degree angle noise yields 267 RPM raw differences at
   * 100 us, despite zero net rotation. A motor-speed trip must not square
   * those differences before estimating mechanical velocity. */
  leg_teach_begin();
  observe(0);
  assert(leg_home());
  for (unsigned n = 0; n < 400; ++n) {
    observe((n & 1u ? .08f : -.08f) / LEG_GEAR_RATIO);
    assert(!leg_motion_fault(true));
  }
  assert(leg_speed_rms(0) < 5 && leg_speed_rms(1) < 5);
  /* Real overspeed still trips in either shaft direction within 20 ms;
   * the test remains far from the joint and per-sample jump limits. */
  for (int sign = -1; sign <= 1; sign += 2) {
    leg_teach_begin();
    observe(10);
    leg_restore_t plan;
    assert(!leg_restore_prepare(&plan) && leg_restore_apply(&plan));
    bool tripped = false;
    for (unsigned n = 1; n <= 200; ++n) {
      observe(10 + sign * 220.0f * 6 * .0001f * n / LEG_GEAR_RATIO);
      if (leg_motion_fault(true) == FOC_SPEED) {
        tripped = true;
        assert(leg_speed_rms(0) > LEG_SPEED_TRIP_MOTOR_RPM);
        break;
      }
    }
    assert(tripped);
  }
  leg_teach_begin();
  observe(0);
}
int main(void) {
  speed_noise_tests();
  speed_brake_tests();
  restore_tests();
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
