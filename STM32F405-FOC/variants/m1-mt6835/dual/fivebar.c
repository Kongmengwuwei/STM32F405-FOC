#include "fivebar.h"
#include "fivebar_internal.h"
#include <math.h>
#include <stddef.h>
#define RAD .017453292519943295f
#define DEG 57.29577951308232f
/* Geometry is bounded to 10 m links/pivots; M4 hardware sqrt avoids the
 * general-purpose hypot scaling routine in every path sample. */
static float norm(float x, float y) { return sqrtf(x * x + y * y); }
static bool sign_ok(int s) { return s == 1 || s == -1; }
static float cross(float ax, float ay, float bx, float by) {
  return ax * by - ay * bx;
}
bool fb_valid(const fb_geometry_t *g) {
  if (!g || !sign_ok(g->assembly_sign) || !isfinite(g->min_sine) ||
      g->min_sine < .01f || g->min_sine > .5f)
    return false;
  for (unsigned i = 0; i < 2; i++) {
    if (!sign_ok(g->q_sign[i]) || !sign_ok(g->elbow_sign[i]) ||
        !isfinite(g->active[i]) || !isfinite(g->passive[i]) ||
        g->active[i] < 1 || g->passive[i] < 1 || g->active[i] > 10000 ||
        g->passive[i] > 10000 || !isfinite(g->zero_ccw[i]) ||
        fabsf(g->zero_ccw[i]) > 720 || !isfinite(g->q_min[i]) ||
        !isfinite(g->q_max[i]) || g->q_max[i] <= g->q_min[i] ||
        g->q_max[i] - g->q_min[i] >= 360 || fabsf(g->q_min[i]) > 720 ||
        fabsf(g->q_max[i]) > 720)
      return false;
    for (unsigned k = 0; k < 2; k++)
      if (!isfinite(g->pivot[i][k]) || fabsf(g->pivot[i][k]) > 10000)
        return false;
  }
  return true;
}
/* Reconstruct a periodic inverse solution in its physical joint interval.
 * Inspect neighboring turns so float roundoff at q=0 cannot become q=360. */
static bool joint_angle(const fb_geometry_t *g, unsigned i, float theta,
                        float *q) {
  float raw = (theta * DEG - g->zero_ccw[i]) / (float)g->q_sign[i];
  float first = floorf((g->q_min[i] - raw) / 360);
  for (unsigned n = 0; n < 3; n++) {
    float candidate = raw + 360 * (first + (float)n);
    if (candidate >= g->q_min[i] - .001f && candidate <= g->q_max[i] + .001f) {
      *q = fminf(g->q_max[i], fmaxf(g->q_min[i], candidate));
      return true;
    }
  }
  return false;
}
fb_status_t fb_forward_validated(const fb_geometry_t *g, const float q[2],
                                 fb_pose_t *out) {
  if (!q || !out)
    return FB_CONFIG;
  fb_pose_t p = {0};
  float tangent[2][2];
  for (unsigned i = 0; i < 2; i++) {
    if (!isfinite(q[i]))
      return FB_NONFINITE;
    if (q[i] < g->q_min[i] - .001f || q[i] > g->q_max[i] + .001f)
      return FB_LIMIT;
    p.q[i] = q[i];
    float t = (g->zero_ccw[i] + g->q_sign[i] * q[i]) * RAD;
    float c = cosf(t), s = sinf(t);
    p.elbow[i][0] = g->pivot[i][0] + g->active[i] * c;
    p.elbow[i][1] = g->pivot[i][1] - g->active[i] * s;
    tangent[i][0] = -g->active[i] * s;
    tangent[i][1] = -g->active[i] * c;
  }
  float dx = p.elbow[1][0] - p.elbow[0][0], dy = p.elbow[1][1] - p.elbow[0][1];
  float d = norm(dx, dy), l0 = g->passive[0], l1 = g->passive[1];
  if (d < .0001f)
    return FB_PARALLEL_SINGULAR;
  if (d > l0 + l1 + .001f || d < fabsf(l0 - l1) - .001f)
    return FB_UNREACHABLE;
  float a = (l0 * l0 - l1 * l1 + d * d) / (2 * d), h2 = l0 * l0 - a * a;
  if (h2 < -.001f)
    return FB_UNREACHABLE;
  float h = sqrtf(fmaxf(0, h2)), ux = dx / d, uy = dy / d;
  p.xy[0] = p.elbow[0][0] + a * ux - g->assembly_sign * h * uy;
  p.xy[1] = p.elbow[0][1] + a * uy + g->assembly_sign * h * ux;
  float arm[2][2], b[2];
  for (unsigned i = 0; i < 2; i++) {
    for (unsigned k = 0; k < 2; k++)
      arm[i][k] = p.xy[k] - p.elbow[i][k];
    float dot = arm[i][0] * tangent[i][0] + arm[i][1] * tangent[i][1];
    p.serial_sine[i] = fabsf(dot) / (g->active[i] * g->passive[i]);
    if (p.serial_sine[i] < g->min_sine)
      return FB_SERIAL_SINGULAR;
    /* Body y points down: a positive CCW elbow offset has negative cross. */
    float rx = p.xy[0] - g->pivot[i][0], ry = p.xy[1] - g->pivot[i][1];
    float ex = p.elbow[i][0] - g->pivot[i][0],
          ey = p.elbow[i][1] - g->pivot[i][1];
    if (cross(rx, ry, ex, ey) * (float)g->elbow_sign[i] >= 0)
      return FB_BRANCH;
    b[i] = dot * (float)g->q_sign[i] * RAD;
    p.inverse[i][0] = arm[i][0] / b[i];
    p.inverse[i][1] = arm[i][1] / b[i];
  }
  float determinant = cross(arm[0][0], arm[0][1], arm[1][0], arm[1][1]);
  p.parallel_sine = fabsf(determinant) / (l0 * l1);
  if (p.parallel_sine < g->min_sine)
    return FB_PARALLEL_SINGULAR;
  p.jacobian[0][0] = arm[1][1] * b[0] / determinant;
  p.jacobian[0][1] = -arm[0][1] * b[1] / determinant;
  p.jacobian[1][0] = -arm[1][0] * b[0] / determinant;
  p.jacobian[1][1] = arm[0][0] * b[1] / determinant;
  for (unsigned i = 0; i < 2; i++)
    for (unsigned k = 0; k < 2; k++)
      if (!isfinite(p.jacobian[i][k]) || !isfinite(p.inverse[i][k]))
        return FB_NONFINITE;
  *out = p;
  return FB_OK;
}
fb_status_t fb_forward(const fb_geometry_t *g, const float q[2],
                       fb_pose_t *out) {
  if (!fb_valid(g))
    return FB_CONFIG;
  return fb_forward_validated(g, q, out);
}
fb_status_t fb_inverse(const fb_geometry_t *g, const float xy[2],
                       fb_pose_t *out) {
  if (!fb_valid(g) || !xy || !out)
    return FB_CONFIG;
  if (!isfinite(xy[0]) || !isfinite(xy[1]))
    return FB_NONFINITE;
  if (fabsf(xy[0]) > 100000 || fabsf(xy[1]) > 100000)
    return FB_UNREACHABLE;
  float q[2];
  for (unsigned i = 0; i < 2; i++) {
    float dx = xy[0] - g->pivot[i][0], dy = xy[1] - g->pivot[i][1];
    float r = norm(dx, dy), a = g->active[i], b = g->passive[i];
    if (r < .0001f || r > 100000 || r > a + b + .001f ||
        r < fabsf(a - b) - .001f)
      return FB_UNREACHABLE;
    float c = (r * r + a * a - b * b) / (2 * a * r);
    if (c < -1.00001f || c > 1.00001f)
      return FB_UNREACHABLE;
    float theta = atan2f(-dy, dx) +
                  (float)g->elbow_sign[i] * acosf(fminf(1, fmaxf(-1, c)));
    if (!joint_angle(g, i, theta, &q[i]))
      return FB_LIMIT;
  }
  fb_pose_t p;
  fb_status_t s = fb_forward(g, q, &p);
  if (s != FB_OK)
    return s;
  /* FK verification rejects the other circle-intersection/assembly mode. */
  if (norm(p.xy[0] - xy[0], p.xy[1] - xy[1]) > .02f)
    return FB_BRANCH;
  *out = p;
  return FB_OK;
}
const char *fb_status_name(fb_status_t s) {
  static const char *const names[] = {"ok",
                                      "geometry",
                                      "nonfinite",
                                      "unreachable",
                                      "joint_limit",
                                      "serial_singularity",
                                      "parallel_singularity",
                                      "assembly_branch",
                                      "path",
                                      "rate"};
  return (unsigned)s < sizeof names / sizeof names[0] ? names[s] : "unknown";
}
