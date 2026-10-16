#ifndef FIVEBAR_MOTION_H
#define FIVEBAR_MOTION_H
#include "fivebar.h"
typedef struct {
  float xy[2], velocity[2], acceleration[2];
} fb_reference_t;
typedef struct {
  float coefficient[2][6], duration, goal[2];
} fb_motion_t;
typedef struct {
  float speed_mm_s, accel_mm_s2, joint_speed_deg_s, joint_accel_deg_s2;
} fb_limits_t;
/* Quintic Cartesian motion. Failed planning leaves the old plan untouched.
 * Validation samples the entire curve, including retarget braking excursions.
 */
fb_status_t fb_plan(const fb_geometry_t *, const fb_reference_t *,
                    const float goal[2], const fb_limits_t *, fb_motion_t *);
void fb_evaluate(const fb_motion_t *, float seconds, fb_reference_t *);
#endif
