#ifndef FIVEBAR_H
#define FIVEBAR_H
#include <stdbool.h>
/* Body frame: x right, y down, millimetres. Geometric angles are CCW from
 * +x, degrees. Array index always means M0/M1, never implicitly left/right. */
typedef enum {
  FB_OK,
  FB_CONFIG,
  FB_NONFINITE,
  FB_UNREACHABLE,
  FB_LIMIT,
  FB_SERIAL_SINGULAR,
  FB_PARALLEL_SINGULAR,
  FB_BRANCH,
  FB_PATH,
  FB_RATE
} fb_status_t;
typedef struct {
  float pivot[2][2], active[2], passive[2];
  float zero_ccw[2], q_min[2], q_max[2];
  int q_sign[2], elbow_sign[2], assembly_sign;
  float min_sine;
} fb_geometry_t;
typedef struct {
  float xy[2], q[2], elbow[2][2];
  float jacobian[2][2]; /* mm / logical joint degree. */
  float inverse[2][2];  /* logical joint degree / mm. */
  float serial_sine[2], parallel_sine;
} fb_pose_t;
bool fb_valid(const fb_geometry_t *g);
fb_status_t fb_forward(const fb_geometry_t *g, const float q[2],
                       fb_pose_t *out);
fb_status_t fb_inverse(const fb_geometry_t *g, const float xy[2],
                       fb_pose_t *out);
const char *fb_status_name(fb_status_t s);
#endif
