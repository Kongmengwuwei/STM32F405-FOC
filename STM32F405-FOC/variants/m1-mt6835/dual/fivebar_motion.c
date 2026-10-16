#include "fivebar_motion.h"
#include <math.h>
#include <stddef.h>
static void polynomial(fb_motion_t *m, const fb_reference_t *r,
                       const float goal[2], float duration) {
  m->duration = duration;
  for (unsigned k = 0; k < 2; k++) {
    float *c = m->coefficient[k];
    c[0] = r->xy[k];
    c[1] = r->velocity[k] * duration;
    c[2] = .5f * r->acceleration[k] * duration * duration;
    float delta = goal[k] - c[0] - c[1] - c[2], v = -c[1] - 2 * c[2],
          a = -2 * c[2];
    c[3] = 10 * delta - 4 * v + .5f * a;
    c[4] = -15 * delta + 7 * v - a;
    c[5] = 6 * delta - 3 * v + .5f * a;
    m->goal[k] = goal[k];
  }
}
void fb_evaluate(const fb_motion_t *m, float seconds, fb_reference_t *r) {
  float u = fminf(1, fmaxf(0, seconds / m->duration)), t = m->duration;
  for (unsigned k = 0; k < 2; k++) {
    const float *c = m->coefficient[k];
    r->xy[k] =
        c[0] + u * (c[1] + u * (c[2] + u * (c[3] + u * (c[4] + u * c[5]))));
    r->velocity[k] =
        (c[1] +
         u * (2 * c[2] + u * (3 * c[3] + u * (4 * c[4] + u * 5 * c[5])))) /
        t;
    r->acceleration[k] =
        (2 * c[2] + u * (6 * c[3] + u * (12 * c[4] + u * 20 * c[5]))) / (t * t);
    if (seconds >= t) {
      r->xy[k] = m->goal[k];
      r->velocity[k] = r->acceleration[k] = 0;
    }
  }
}
fb_status_t fb_plan(const fb_geometry_t *g, const fb_reference_t *r,
                    const float goal[2], const fb_limits_t *l,
                    fb_motion_t *out) {
  if (!g || !r || !goal || !l || !out || !fb_valid(g))
    return FB_CONFIG;
  const float limits[] = {l->speed_mm_s, l->accel_mm_s2, l->joint_speed_deg_s,
                          l->joint_accel_deg_s2};
  for (unsigned k = 0; k < 4; k++)
    if (!isfinite(limits[k]) || limits[k] < .01f || limits[k] > 10000)
      return FB_RATE;
  for (unsigned k = 0; k < 2; k++)
    if (!isfinite(r->xy[k]) || !isfinite(r->velocity[k]) ||
        !isfinite(r->acceleration[k]) || !isfinite(goal[k]))
      return FB_NONFINITE;
  fb_pose_t pose;
  fb_status_t status = fb_inverse(g, r->xy, &pose);
  if (status != FB_OK)
    return status;
  status = fb_inverse(g, goal, &pose);
  if (status != FB_OK)
    return status;
  float distance = hypotf(goal[0] - r->xy[0], goal[1] - r->xy[1]);
  float duration = fmaxf(.2f, fmaxf(1.9f * distance / l->speed_mm_s,
                                    sqrtf(6 * distance / l->accel_mm_s2)));
  for (unsigned trial = 0; trial < 18 && duration <= 30;
       trial++, duration *= 1.2f) {
    fb_motion_t candidate;
    polynomial(&candidate, r, goal, duration);
    float previous_q[2] = {0}, previous_v[2] = {0}, previous_xy[2] = {0};
    bool rate_ok = true, resolution_ok = true;
    /* At most 512 subdivisions, then refuse rather than silently skip a
     * narrow workspace defect. 1 mm / 1 degree between samples. */
    unsigned count = 128;
    for (;;) {
      rate_ok = resolution_ok = true;
      for (unsigned n = 0; n <= count; n++) {
        fb_reference_t ref;
        fb_evaluate(&candidate, duration * n / count, &ref);
        status = fb_inverse(g, ref.xy, &pose);
        if (status != FB_OK)
          return status;
        if (hypotf(ref.velocity[0], ref.velocity[1]) > l->speed_mm_s * .98f ||
            hypotf(ref.acceleration[0], ref.acceleration[1]) >
                l->accel_mm_s2 * .98f)
          rate_ok = false;
        if (n &&
            hypotf(ref.xy[0] - previous_xy[0], ref.xy[1] - previous_xy[1]) > 1)
          resolution_ok = false;
        for (unsigned i = 0; i < 2; i++) {
          float v = pose.inverse[i][0] * ref.velocity[0] +
                    pose.inverse[i][1] * ref.velocity[1];
          if (!isfinite(v) || fabsf(v) > l->joint_speed_deg_s * .98f ||
              (n && fabsf(v - previous_v[i]) * count / duration >
                        l->joint_accel_deg_s2 * .95f))
            rate_ok = false;
          if (n && fabsf(pose.q[i] - previous_q[i]) > 1)
            resolution_ok = false;
          previous_q[i] = pose.q[i];
          previous_v[i] = v;
          previous_xy[i] = ref.xy[i];
        }
      }
      if (resolution_ok || count == 512)
        break;
      count *= 2;
    }
    if (!resolution_ok)
      return FB_PATH;
    if (rate_ok) {
      *out = candidate;
      return FB_OK;
    }
  }
  return FB_RATE;
}
