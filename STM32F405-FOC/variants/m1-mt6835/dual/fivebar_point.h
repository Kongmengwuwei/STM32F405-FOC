#ifndef FIVEBAR_POINT_H
#define FIVEBAR_POINT_H
#include "fivebar_motion.h"
typedef struct {
  float c[2][6], duration;
} fb_joint_curve_t;
typedef struct {
  fb_joint_curve_t segment[2];
  unsigned count;
  float duration, goal[2];
} fb_point_motion_t;
/* End-point motion, not a Cartesian straight line. Full FK/branch/rate checks.
 * Moving retargets brake first, preserving position, velocity and acceleration.
 * Rejection never modifies the accepted plan. */
fb_status_t fb_point_plan(const fb_geometry_t *, const fb_reference_t *,
                          const float goal[2], const fb_limits_t *,
                          fb_point_motion_t *);
/* Optional foreground service: never re-enter planning or mutate its inputs.
 * Return false to cancel. Called between bounded batches of path samples. */
typedef struct {
  bool (*poll)(void *);
  void *context;
} fb_plan_service_t;
fb_status_t fb_point_plan_serviced(const fb_geometry_t *,
                                   const fb_reference_t *, const float goal[2],
                                   const fb_limits_t *, fb_point_motion_t *,
                                   const fb_plan_service_t *);
/* Lightweight runtime reference for an already preflighted joint path. */
fb_status_t fb_point_joint_evaluate(const fb_point_motion_t *, float,
                                    fb_reference_t *);
fb_status_t fb_point_evaluate(const fb_geometry_t *, const fb_point_motion_t *,
                              float seconds, fb_reference_t *, fb_pose_t *);
#endif
