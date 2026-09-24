#include "foc_angle_tracker.h"

#include <math.h>

#define FOC_TWO_PI 6.2831853071795865f
#define FOC_PI 3.1415926535897932f

bool foc_angle_tracker_init(foc_angle_tracker_t *tracker,
                            float max_speed_rad_s,
                            float step_tolerance_rad,
                            uint32_t max_gap_us)
{
  if (tracker == 0 || !isfinite(max_speed_rad_s) ||
      !isfinite(step_tolerance_rad) || max_speed_rad_s <= 0.0f ||
      step_tolerance_rad < 0.0f || max_gap_us == 0u ||
      max_speed_rad_s * (float)max_gap_us * 0.000001f +
          step_tolerance_rad >= FOC_PI)
  {
    return false;
  }

  *tracker = (foc_angle_tracker_t){
    .max_speed_rad_s = max_speed_rad_s,
    .step_tolerance_rad = step_tolerance_rad,
    .max_gap_us = max_gap_us,
  };
  return true;
}

foc_angle_status_t foc_angle_tracker_update(foc_angle_tracker_t *tracker,
                                             bool frame_valid,
                                             float angle_rad,
                                             uint32_t sample_time_us,
                                             float *speed_rad_s)
{
  if (tracker == 0 || speed_rad_s == 0)
  {
    return FOC_ANGLE_FAULT;
  }
  *speed_rad_s = 0.0f;

  if (tracker->fault_latched)
  {
    return FOC_ANGLE_FAULT;
  }
  if (!frame_valid || !isfinite(angle_rad) ||
      angle_rad < 0.0f || angle_rad >= FOC_TWO_PI)
  {
    tracker->fault_latched = true;
    return FOC_ANGLE_FAULT;
  }

  if (!tracker->has_previous)
  {
    tracker->previous_angle_rad = angle_rad;
    tracker->previous_time_us = sample_time_us;
    tracker->has_previous = true;
    return FOC_ANGLE_PRIMING;
  }

  const uint32_t elapsed_us = sample_time_us - tracker->previous_time_us;
  if (elapsed_us == 0u || elapsed_us > tracker->max_gap_us)
  {
    tracker->fault_latched = true;
    return FOC_ANGLE_FAULT;
  }

  float delta = angle_rad - tracker->previous_angle_rad;
  if (delta > FOC_PI)
  {
    delta -= FOC_TWO_PI;
  }
  else if (delta < -FOC_PI)
  {
    delta += FOC_TWO_PI;
  }

  const float elapsed_s = (float)elapsed_us * 0.000001f;
  const float max_step =
      tracker->max_speed_rad_s * elapsed_s + tracker->step_tolerance_rad;
  if (fabsf(delta) > max_step)
  {
    tracker->fault_latched = true;
    return FOC_ANGLE_FAULT;
  }

  tracker->previous_angle_rad = angle_rad;
  tracker->previous_time_us = sample_time_us;
  *speed_rad_s = delta / elapsed_s;
  return FOC_ANGLE_VALID;
}
