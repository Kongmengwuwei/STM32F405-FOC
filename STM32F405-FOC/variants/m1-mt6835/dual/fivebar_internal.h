#ifndef FIVEBAR_INTERNAL_H
#define FIVEBAR_INTERNAL_H
#include "fivebar.h"
/* Internal bulk-planning entry: caller must have passed fb_valid once for
 * this immutable geometry. Joint, branch, singularity and numeric checks
 * still run for every pose. Public fb_forward always validates geometry. */
fb_status_t fb_forward_validated(const fb_geometry_t *g, const float q[2],
                                 fb_pose_t *out);
#endif
