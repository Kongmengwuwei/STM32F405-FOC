#include "fivebar_point.h"
#include "finite_math.h"
#include "fivebar_internal.h"
#include <math.h>
#include <string.h>
#define RAD .0174532925199433f
static float norm(float x, float y) { return sqrtf(x * x + y * y); }
static void curve(fb_joint_curve_t *m, const fb_reference_t *r,
                  const float end[2], float t) {
  m->duration = t;
  for (unsigned i = 0; i < 2; i++) {
    float *c = m->c[i];
    c[0] = r->xy[i];
    c[1] = r->velocity[i] * t;
    c[2] = .5f * r->acceleration[i] * t * t;
    float d = end[i] - c[0] - c[1] - c[2], v = -c[1] - 2 * c[2], a = -2 * c[2];
    c[3] = 10 * d - 4 * v + .5f * a;
    c[4] = -15 * d + 7 * v - a;
    c[5] = 6 * d - 3 * v + .5f * a;
  }
}
static void evaluate(const fb_joint_curve_t *m, float seconds,
                     fb_reference_t *q) {
  float t = m->duration, u = fminf(1, fmaxf(0, seconds / t));
  for (unsigned i = 0; i < 2; i++) {
    const float *c = m->c[i];
    q->xy[i] =
        c[0] + u * (c[1] + u * (c[2] + u * (c[3] + u * (c[4] + u * c[5]))));
    q->velocity[i] =
        (c[1] +
         u * (2 * c[2] + u * (3 * c[3] + u * (4 * c[4] + u * 5 * c[5])))) /
        t;
    q->acceleration[i] =
        (2 * c[2] + u * (6 * c[3] + u * (12 * c[4] + u * 20 * c[5]))) / (t * t);
    if (seconds >= t)
      q->velocity[i] = q->acceleration[i] = 0;
  }
}
/* Differentiate the two passive-link length constraints twice. This includes
 * the J-dot term; J*q-double-dot alone underestimates foot acceleration. */
static fb_status_t cartesian(const fb_geometry_t *g, const fb_reference_t *q,
                             fb_reference_t *r, fb_pose_t *p) {
  fb_status_t status = fb_forward_validated(g, q->xy, p);
  if (status != FB_OK)
    return status;
  memcpy(r->xy, p->xy, sizeof r->xy);
  for (unsigned i = 0; i < 2; i++)
    r->velocity[i] =
        p->jacobian[i][0] * q->velocity[0] + p->jacobian[i][1] * q->velocity[1];
  float rows[2][2], rhs[2];
  for (unsigned i = 0; i < 2; i++) {
    /* FK already computed the active-link vector; reuse it instead of four
     * library trig calls per preflight sample. Body y points down. */
    float ex = p->elbow[i][0] - g->pivot[i][0],
          ey = p->elbow[i][1] - g->pivot[i][1];
    float w = g->q_sign[i] * RAD * q->velocity[i];
    float a = g->q_sign[i] * RAD * q->acceleration[i];
    float vb[2] = {ey * w, -ex * w};
    float ab[2] = {-ex * w * w + ey * a, -ey * w * w - ex * a};
    float vx = r->velocity[0] - vb[0], vy = r->velocity[1] - vb[1];
    rows[i][0] = p->xy[0] - p->elbow[i][0];
    rows[i][1] = p->xy[1] - p->elbow[i][1];
    rhs[i] = rows[i][0] * ab[0] + rows[i][1] * ab[1] - vx * vx - vy * vy;
  }
  float det = rows[0][0] * rows[1][1] - rows[0][1] * rows[1][0];
  r->acceleration[0] = (rhs[0] * rows[1][1] - rows[0][1] * rhs[1]) / det;
  r->acceleration[1] = (rows[0][0] * rhs[1] - rhs[0] * rows[1][0]) / det;
  return isfinite(r->acceleration[0]) && isfinite(r->acceleration[1])
             ? FB_OK
             : FB_NONFINITE;
}
static fb_status_t inspect(const fb_geometry_t *g, const fb_joint_curve_t *m,
                           const fb_limits_t *l, float *scale,
                           const fb_plan_service_t *service) {
  /* Select the first grid from endpoint span, then still refine against
   * actual sample spacing. This avoids redundant scans of long motions. */
  fb_reference_t first, last;
  fb_reference_t rf, rl;
  fb_pose_t pf, pl;
  evaluate(m, 0, &first);
  evaluate(m, m->duration, &last);
  fb_status_t status = cartesian(g, &first, &rf, &pf);
  if (status != FB_OK)
    return status;
  status = cartesian(g, &last, &rl, &pl);
  if (status != FB_OK)
    return status;
  float span = fmaxf(
      norm(rl.xy[0] - rf.xy[0], rl.xy[1] - rf.xy[1]),
      fmaxf(fabsf(last.xy[0] - first.xy[0]), fabsf(last.xy[1] - first.xy[1])));
  unsigned count = 128;
  while (count < 512 && count < 2 * span)
    count *= 2;
  for (;;) {
    float previous_q[2] = {0}, previous_xy[2] = {0};
    bool resolution = true, rate = true;
    *scale = 1;
    for (unsigned n = 0; n <= count; n++) {
      if (service && service->poll && (n & 31u) == 0 &&
          !service->poll(service->context))
        return FB_CONFIG;
      fb_reference_t q, r;
      fb_pose_t p;
      evaluate(m, m->duration * n / count, &q);
      fb_status_t status = cartesian(g, &q, &r, &p);
      if (status != FB_OK)
        return status;
      float v = norm(r.velocity[0], r.velocity[1]) / l->speed_mm_s;
      float a = norm(r.acceleration[0], r.acceleration[1]) / l->accel_mm_s2;
      if (v > .98f || a > .97f)
        rate = false;
      *scale = fmaxf(*scale, fmaxf(v / .96f, sqrtf(a / .95f)));
      if (n && norm(r.xy[0] - previous_xy[0], r.xy[1] - previous_xy[1]) > 1)
        resolution = false;
      for (unsigned i = 0; i < 2; i++) {
        v = fabsf(q.velocity[i]) / l->joint_speed_deg_s;
        a = fabsf(q.acceleration[i]) / l->joint_accel_deg_s2;
        if (v > .98f || a > .97f)
          rate = false;
        *scale = fmaxf(*scale, fmaxf(v / .96f, sqrtf(a / .95f)));
        if (n && fabsf(q.xy[i] - previous_q[i]) > 1)
          resolution = false;
        previous_q[i] = q.xy[i];
        previous_xy[i] = r.xy[i];
      }
    }
    if (resolution)
      return rate ? FB_OK : FB_RATE;
    if (count == 512)
      return FB_PATH;
    count *= 2;
  }
}
static fb_status_t plan_joint(const fb_geometry_t *g,
                              const fb_reference_t *initial, const float end[2],
                              const fb_limits_t *l, fb_joint_curve_t *out,
                              const fb_plan_service_t *service) {
  float delta =
      fmaxf(fabsf(end[0] - initial->xy[0]), fabsf(end[1] - initial->xy[1]));
  /* Rest-to-rest quintic peaks at 1.875*delta/T. Start below the
   * inspector margin so ordinary moves do not require two full scans. */
  float duration =
      fmaxf(.15f, fmaxf(2.0f * delta / l->joint_speed_deg_s,
                        sqrtf(6.5f * delta / l->joint_accel_deg_s2)));
  for (unsigned trial = 0; trial < 12 && duration <= 30; trial++) {
    fb_joint_curve_t candidate;
    curve(&candidate, initial, end, duration);
    float scale;
    fb_status_t status = inspect(g, &candidate, l, &scale, service);
    if (status == FB_OK) {
      *out = candidate;
      return FB_OK;
    }
    if (status != FB_RATE)
      return status;
    duration *= fmaxf(1.08f, scale * 1.02f);
  }
  return FB_RATE;
}
fb_status_t fb_point_plan_serviced(const fb_geometry_t *g,
                                   const fb_reference_t *r, const float goal[2],
                                   const fb_limits_t *l, fb_point_motion_t *out,
                                   const fb_plan_service_t *service) {
  if (!g || !r || !goal || !l || !out || !fb_valid(g))
    return FB_CONFIG;
  const float limits[] = {l->speed_mm_s, l->accel_mm_s2, l->joint_speed_deg_s,
                          l->joint_accel_deg_s2};
  for (unsigned i = 0; i < 4; i++)
    if (!isfinite(limits[i]) || limits[i] < .01f || limits[i] > 10000)
      return FB_RATE;
  for (unsigned i = 0; i < 2; i++)
    if (!isfinite(r->xy[i]) || !isfinite(r->velocity[i]) ||
        !isfinite(r->acceleration[i]) || !isfinite(goal[i]))
      return FB_NONFINITE;
  fb_pose_t start, end;
  fb_status_t status = fb_inverse(g, r->xy, &start);
  if (status != FB_OK)
    return status;
  status = fb_inverse(g, goal, &end);
  if (status != FB_OK)
    return status;
  fb_reference_t q = {0};
  memcpy(q.xy, start.q, sizeof q.xy);
  for (unsigned i = 0; i < 2; i++)
    q.velocity[i] = start.inverse[i][0] * r->velocity[0] +
                    start.inverse[i][1] * r->velocity[1];
  fb_reference_t bias;
  fb_pose_t pose;
  status = cartesian(g, &q, &bias, &pose);
  if (status != FB_OK)
    return status;
  for (unsigned i = 0; i < 2; i++)
    q.acceleration[i] =
        start.inverse[i][0] * (r->acceleration[0] - bias.acceleration[0]) +
        start.inverse[i][1] * (r->acceleration[1] - bias.acceleration[1]);
  fb_point_motion_t candidate = {0};
  memcpy(candidate.goal, goal, sizeof candidate.goal);
  bool moving = norm(q.velocity[0], q.velocity[1]) >= .001f ||
                norm(q.acceleration[0], q.acceleration[1]) >= .001f;
  if (!moving) {
    status = plan_joint(g, &q, end.q, l, &candidate.segment[0], service);
    if (status != FB_OK)
      return status;
    candidate.count = 1;
    candidate.duration = candidate.segment[0].duration;
    *out = candidate;
    return FB_OK;
  }
  /* Moving retargets brake from the existing C2 reference first. Repeatedly
   * lengthening a single quintic with nonzero initial velocity can increase
   * its excursion and exhaust foreground time before reaching this brake.
   * Preflight both pieces; publish neither unless both are valid. */
  float t = fmaxf(
      .04f,
      fmaxf(1.8f * fmaxf(fabsf(q.velocity[0]), fabsf(q.velocity[1])) /
                l->joint_accel_deg_s2,
            1.8f * norm(r->velocity[0], r->velocity[1]) / l->accel_mm_s2));
  for (unsigned trial = 0; trial < 12 && t < 10; trial++) {
    float stop[2];
    for (unsigned i = 0; i < 2; i++)
      stop[i] =
          q.xy[i] + .5f * q.velocity[i] * t + q.acceleration[i] * t * t / 12;
    curve(&candidate.segment[0], &q, stop, t);
    float scale;
    status = inspect(g, &candidate.segment[0], l, &scale, service);
    if (status != FB_OK) {
      if (status != FB_RATE)
        return status;
      t *= fmaxf(1.08f, scale * 1.02f);
      continue;
    }
    fb_reference_t rest = {.xy = {stop[0], stop[1]}};
    status = plan_joint(g, &rest, end.q, l, &candidate.segment[1], service);
    if (status != FB_OK)
      return status;
    candidate.count = 2;
    candidate.duration = t + candidate.segment[1].duration;
    if (candidate.duration > 30)
      return FB_RATE;
    *out = candidate;
    return FB_OK;
  }
  return FB_RATE;
}
fb_status_t fb_point_plan(const fb_geometry_t *g, const fb_reference_t *r,
                          const float goal[2], const fb_limits_t *l,
                          fb_point_motion_t *out) {
  return fb_point_plan_serviced(g, r, goal, l, out, NULL);
}
fb_status_t fb_point_joint_evaluate(const fb_point_motion_t *m, float seconds,
                                    fb_reference_t *q) {
  if (!m || !q || !isfinite(seconds) || m->count < 1 || m->count > 2)
    return FB_CONFIG;
  unsigned part = 0;
  for (unsigned i = 0; i < m->count; i++)
    if (!isfinite(m->segment[i].duration) || m->segment[i].duration <= 0)
      return FB_CONFIG;
  if (m->count == 2 && seconds >= m->segment[0].duration) {
    seconds -= m->segment[0].duration;
    part = 1;
  }
  evaluate(&m->segment[part], seconds, q);
  for (unsigned i = 0; i < 2; i++)
    if (!isfinite(q->xy[i]) || !isfinite(q->velocity[i]) ||
        !isfinite(q->acceleration[i]))
      return FB_NONFINITE;
  return FB_OK;
}
fb_status_t fb_point_evaluate(const fb_geometry_t *g,
                              const fb_point_motion_t *m, float seconds,
                              fb_reference_t *r, fb_pose_t *p) {
  if (!g || !m || !r || !p || !fb_valid(g) || !isfinite(seconds) ||
      m->count < 1 || m->count > 2)
    return FB_CONFIG;
  fb_reference_t q;
  fb_status_t status = fb_point_joint_evaluate(m, seconds, &q);
  return status == FB_OK ? cartesian(g, &q, r, p) : status;
}
