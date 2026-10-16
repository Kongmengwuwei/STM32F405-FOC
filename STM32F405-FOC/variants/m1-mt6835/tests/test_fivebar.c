#include "fivebar_config.h"
#include "fivebar_motion.h"
#include "fivebar_point.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static fb_geometry_t geometry = FIVEBAR_GEOMETRY_INITIALIZER;
static void near(float a, float b, float tolerance) {
  assert(fabsf(a - b) < tolerance);
}
static void check_motion(const fb_motion_t *m, const fb_limits_t *limits) {
  float previous[2] = {0};
  for (unsigned n = 0; n <= 10000; n++) {
    fb_reference_t r;
    fb_pose_t p;
    fb_evaluate(m, m->duration * n / 10000, &r);
    assert(fb_inverse(&geometry, r.xy, &p) == FB_OK);
    assert(hypotf(r.velocity[0], r.velocity[1]) <= limits->speed_mm_s);
    assert(hypotf(r.acceleration[0], r.acceleration[1]) <= limits->accel_mm_s2);
    for (unsigned i = 0; i < 2; i++) {
      float v =
          p.inverse[i][0] * r.velocity[0] + p.inverse[i][1] * r.velocity[1];
      assert(fabsf(v) <= limits->joint_speed_deg_s);
      if (n)
        assert(fabsf(v - previous[i]) * 10000 / m->duration <=
               limits->joint_accel_deg_s2);
      previous[i] = v;
    }
  }
}
static void check_point(const fb_point_motion_t *m, const fb_limits_t *l) {
  for (unsigned n = 0; n <= 5000; n++) {
    float t = m->duration * n / 5000;
    fb_reference_t r;
    fb_pose_t p;
    assert(fb_point_evaluate(&geometry, m, t, &r, &p) == FB_OK);
    assert(hypotf(r.velocity[0], r.velocity[1]) <= l->speed_mm_s);
    assert(hypotf(r.acceleration[0], r.acceleration[1]) <= l->accel_mm_s2);
    for (unsigned i = 0; i < 2; i++) {
      float v =
          p.inverse[i][0] * r.velocity[0] + p.inverse[i][1] * r.velocity[1];
      assert(fabsf(v) <= l->joint_speed_deg_s);
    }
    /* Independent finite difference checks the J-dot acceleration term. */
    if (n % 50 == 0 && t > .002f && t < m->duration - .002f &&
        !(m->count == 2 && fabsf(t - m->segment[0].duration) < .002f)) {
      fb_reference_t a, b;
      fb_pose_t pa, pb;
      assert(fb_point_evaluate(&geometry, m, t - .001f, &a, &pa) == FB_OK);
      assert(fb_point_evaluate(&geometry, m, t + .001f, &b, &pb) == FB_OK);
      for (unsigned i = 0; i < 2; i++)
        near((b.velocity[i] - a.velocity[i]) / .002f, r.acceleration[i], 1.2f);
    }
  }
}
static bool service_tick(void *p) { ++*(unsigned*)p;return true; }
static bool service_cancel(void *p) { ++*(unsigned*)p;return false; }
int main(void) {
  assert(fb_valid(&geometry));
  unsigned checked = 0, rejected = 0;
  float max_roundtrip = 0, max_jacobian = 0;
  for (unsigned i = 2; i <= 108; i++)
    for (unsigned j = 2; j <= 108; j++) {
      float q[2] = {(float)i, (float)j};
      fb_pose_t p, inverse;
      fb_status_t status = fb_forward(&geometry, q, &p);
      if (status != FB_OK) {
        rejected++;
        continue;
      }
      assert(fb_inverse(&geometry, p.xy, &inverse) == FB_OK);
      for (unsigned k = 0; k < 2; k++) {
        max_roundtrip = fmaxf(max_roundtrip, fabsf(q[k] - inverse.q[k]));
        near(q[k], inverse.q[k], .003f);
        float perturbed[2] = {q[0], q[1]};
        perturbed[k] += .02f;
        fb_pose_t plus;
        if (fb_forward(&geometry, perturbed, &plus) != FB_OK)
          continue;
        perturbed[k] -= .04f;
        fb_pose_t minus;
        if (fb_forward(&geometry, perturbed, &minus) != FB_OK)
          continue;
        for (unsigned axis = 0; axis < 2; axis++) {
          float derivative = (plus.xy[axis] - minus.xy[axis]) / .04f;
          float error = fabsf(derivative - p.jacobian[axis][k]);
          max_jacobian = fmaxf(max_jacobian, error);
          near(derivative, p.jacobian[axis][k], .015f);
        }
      }
      for (unsigned row = 0; row < 2; row++)
        for (unsigned col = 0; col < 2; col++)
          near(p.inverse[row][0] * p.jacobian[0][col] +
                   p.inverse[row][1] * p.jacobian[1][col],
               row == col ? 1 : 0, .0001f);
      checked++;
    }
  assert(checked > 5000);
  fb_pose_t untouched;
  memset(&untouched, 0x5a, sizeof untouched);
  fb_pose_t output = untouched;
  float bad[2] = {NAN, 160};
  assert(fb_inverse(&geometry, bad, &output) == FB_NONFINITE);
  assert(!memcmp(&output, &untouched, sizeof output));
  bad[0] = 0;
  bad[1] = 1000;
  assert(fb_inverse(&geometry, bad, &output) == FB_UNREACHABLE);
  bad[0] = -1;
  bad[1] = 2;
  assert(fb_forward(&geometry, bad, &output) == FB_LIMIT);
  fb_geometry_t g = geometry;
  g.q_sign[0] = 0;
  assert(!fb_valid(&g));
  g = geometry;
  g.min_sine = .5f;
  float top[2] = {2, 2};
  assert(fb_forward(&g, top, &output) == FB_SERIAL_SINGULAR);
  g = geometry;
  g.min_sine = .5f;
  g.active[0] = g.active[1] = 1;
  g.zero_ccw[0] = 0;
  g.zero_ccw[1] = 180;
  g.q_min[0] = g.q_min[1] = 0;
  float bottom[2] = {0, 0};
  assert(fb_forward(&g, bottom, &output) == FB_PARALLEL_SINGULAR);
  /* Zero offsets on other periodic turns must preserve physical q. */
  g = geometry;
  g.zero_ccw[0] += 360;
  g.zero_ccw[1] -= 360;
  float middle[2] = {60, 60};
  fb_pose_t p;
  assert(fb_forward(&g, middle, &p) == FB_OK);
  assert(fb_inverse(&g, p.xy, &output) == FB_OK);
  near(output.q[0], 60, .001f);
  near(output.q[1], 60, .001f);
  g = geometry;
  g.elbow_sign[0] = -g.elbow_sign[0];
  assert(fb_forward(&g, middle, &p) == FB_BRANCH);
  fb_limits_t limits = {50, 100, 30, 90};
  fb_reference_t initial = {.xy = {0, 160}};
  float goal[2] = {10, 180};
  fb_motion_t motion;
  assert(fb_plan(&geometry, &initial, goal, &limits, &motion) == FB_OK);
  check_motion(&motion, &limits);
  fb_reference_t finish;
  fb_evaluate(&motion, motion.duration + 1, &finish);
  near(finish.xy[0], 10, .0001f);
  near(finish.xy[1], 180, .0001f);
  assert(finish.velocity[0] == 0 && finish.acceleration[1] == 0);
  fb_reference_t moving;
  fb_evaluate(&motion, .3f * motion.duration, &moving);
  float reverse[2] = {-10, 150};
  fb_motion_t new_motion;
  assert(fb_plan(&geometry, &moving, reverse, &limits, &new_motion) == FB_OK);
  fb_reference_t start;
  fb_evaluate(&new_motion, 0, &start);
  for (unsigned k = 0; k < 2; k++) {
    near(start.xy[k], moving.xy[k], .0001f);
    near(start.velocity[k], moving.velocity[k], .0001f);
    near(start.acceleration[k], moving.acceleration[k], .0001f);
  }
  check_motion(&new_motion, &limits);
  fb_motion_t saved = new_motion;
  float unreachable[2] = {0, 1000};
  assert(fb_plan(&geometry, &moving, unreachable, &limits, &new_motion) !=
         FB_OK);
  assert(!memcmp(&saved, &new_motion, sizeof saved));
  /* Valid endpoints do not make the retarget braking excursion valid. */
  float edge_q[2] = {107.9f, 107.9f}, inside_q[2] = {100, 100};
  fb_pose_t edge, inside;
  assert(fb_forward(&geometry, edge_q, &edge) == FB_OK);
  assert(fb_forward(&geometry, inside_q, &inside) == FB_OK);
  fb_reference_t excursion = {.xy = {edge.xy[0], edge.xy[1]},
                              .velocity = {0, 40}};
  fb_status_t path_status =
      fb_plan(&geometry, &excursion, inside.xy, &limits, &new_motion);
  assert(path_status == FB_LIMIT || path_status == FB_UNREACHABLE ||
         path_status == FB_BRANCH);
  assert(!memcmp(&saved, &new_motion, sizeof saved));
  /* Far endpoints and reversal during motion must not require a Cartesian
   * chord. Both polynomial pieces and their braking junction are bounded. */
  fb_limits_t fast = {150, 600, 30, 90};
  float start_q[2] = {5, 5}, far_q[2] = {105, 105}, reverse_q[2] = {10, 10};
  fb_pose_t start_p, far_p, reverse_p;
  assert(fb_forward(&geometry, start_q, &start_p) == FB_OK);
  assert(fb_forward(&geometry, far_q, &far_p) == FB_OK);
  assert(fb_forward(&geometry, reverse_q, &reverse_p) == FB_OK);
  fb_reference_t initial_point = {.xy = {start_p.xy[0], start_p.xy[1]}};
  fb_point_motion_t point, reversed;
  assert(fb_point_plan(&geometry, &initial_point, far_p.xy, &fast, &point) ==
         FB_OK);
  check_point(&point, &fast);
  fb_reference_t during;
  fb_pose_t during_p;
  assert(fb_point_evaluate(&geometry, &point, .4f * point.duration, &during,
                           &during_p) == FB_OK);
  assert(fb_point_plan(&geometry, &during, reverse_p.xy, &fast, &reversed) ==
         FB_OK);
  fb_reference_t beginning;
  fb_pose_t beginning_p;
  assert(fb_point_evaluate(&geometry, &reversed, 0, &beginning, &beginning_p) ==
         FB_OK);
  for (unsigned i = 0; i < 2; i++) {
    near(beginning.xy[i], during.xy[i], .001f);
    near(beginning.velocity[i], during.velocity[i], .002f);
    near(beginning.acceleration[i], during.acceleration[i], .01f);
  }
  check_point(&reversed, &fast);
  assert(reversed.count == 2);
  /* Retarget during acceleration, peak velocity and deceleration. Include
   * both directions; these cases used to scan many single-curve candidates. */
  for (unsigned n = 1; n < 10; n += 2) {
    fb_reference_t live, joined;
    fb_pose_t live_pose, joined_pose;
    fb_point_motion_t changed, again;
    assert(fb_point_evaluate(&geometry, &point, point.duration * n / 10, &live,
                             &live_pose) == FB_OK);
    assert(fb_point_plan(&geometry, &live, reverse_p.xy, &fast, &changed) ==
           FB_OK);
    assert(changed.count == 2);
    assert(fb_point_evaluate(&geometry, &changed, 0, &joined, &joined_pose) ==
           FB_OK);
    for (unsigned i = 0; i < 2; i++) {
      near(joined.xy[i], live.xy[i], .001f);
      near(joined.velocity[i], live.velocity[i], .002f);
      near(joined.acceleration[i], live.acceleration[i], .01f);
    }
    check_point(&changed, &fast);
    assert(fb_point_evaluate(&geometry, &changed, .5f * changed.duration, &live,
                             &live_pose) == FB_OK);
    assert(fb_point_plan(&geometry, &live, far_p.xy, &fast, &again) == FB_OK);
    check_point(&again, &fast);
  }
  if (reversed.count == 2) {
    fb_reference_t left, right;
    fb_pose_t pl, pr;
    assert(fb_point_evaluate(&geometry, &reversed,
                             reversed.segment[0].duration - .0001f, &left,
                             &pl) == FB_OK);
    assert(fb_point_evaluate(&geometry, &reversed, reversed.segment[0].duration,
                             &right, &pr) == FB_OK);
    for (unsigned i = 0; i < 2; i++) {
      near(left.xy[i], right.xy[i], .001f);
      near(left.velocity[i], right.velocity[i], .002f);
      near(left.acceleration[i], right.acceleration[i], .2f);
    }
  }
  /* A feasible brake close to the upper boundary requires two pieces. */
  fb_point_motion_t seed = {.count = 1, .duration = 1};
  seed.segment[0].duration = 1;
  for (unsigned i = 0; i < 2; i++) {
    seed.segment[0].c[i][0] = 100;
    seed.segment[0].c[i][1] = 25;
  }
  assert(fb_point_evaluate(&geometry, &seed, 0, &during, &during_p) == FB_OK);
  assert(fb_point_plan(&geometry, &during, reverse_p.xy, &fast, &reversed) ==
         FB_OK);
  assert(reversed.count == 2);
  check_point(&reversed, &fast);
  /* Fast profiles and serviced preflight use the same checked path. */
  float previous_duration=point.duration;
  for(unsigned rpm=60;rpm<=120;rpm+=30) {
    float acceleration=150+10*((float)rpm-45);
    fb_limits_t quicker={400,1800,.8f*rpm*6/7,.7f*acceleration*6/7};
    fb_point_motion_t accelerated;
    unsigned calls=0;fb_plan_service_t service={service_tick,&calls};
    assert(fb_point_plan_serviced(&geometry,&initial_point,far_p.xy,&quicker,&accelerated,&service)==FB_OK);
    assert(calls>0&&accelerated.duration<previous_duration);
    previous_duration=accelerated.duration;check_point(&accelerated,&quicker);
    fb_reference_t jq,xy;fb_pose_t check;
    float elapsed=accelerated.duration*.4f;
    assert(fb_point_joint_evaluate(&accelerated,elapsed,&jq)==FB_OK);
    assert(fb_point_evaluate(&geometry,&accelerated,elapsed,&xy,&check)==FB_OK);
    for(unsigned i=0;i<2;i++) {
      near(jq.xy[i],check.q[i],.0001f);
      near(jq.velocity[i],check.inverse[i][0]*xy.velocity[0]+check.inverse[i][1]*xy.velocity[1],.001f);
    }
    fb_point_motion_t saved=accelerated;service.poll=service_cancel;
    assert(fb_point_plan_serviced(&geometry,&initial_point,far_p.xy,&quicker,&accelerated,&service)!=FB_OK);
    assert(!memcmp(&saved,&accelerated,sizeof saved));
  }
  fb_point_motion_t kept = reversed;
  assert(fb_point_plan(&geometry, &during, unreachable, &fast, &reversed) !=
         FB_OK);
  assert(!memcmp(&kept, &reversed, sizeof kept));
  printf("point full-stroke duration=%.3fs reverse=%.3fs segments=%u\n",
         point.duration, reversed.duration, reversed.count);
  printf("PASS: %u FK/IK poses (%u excluded), max joint roundtrip %.6f deg, "
         "Jacobian diff %.6f mm/deg; C2 retarget, rate bounds, full-path "
         "rejection\n",
         checked, rejected, max_roundtrip, max_jacobian);
}
