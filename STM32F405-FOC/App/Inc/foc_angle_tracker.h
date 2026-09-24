#ifndef FOC_ANGLE_TRACKER_H
#define FOC_ANGLE_TRACKER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  FOC_ANGLE_PRIMING,
  FOC_ANGLE_VALID,
  FOC_ANGLE_FAULT
} foc_angle_status_t;

typedef struct
{
  float max_speed_rad_s;
  float step_tolerance_rad;
  uint32_t max_gap_us;
  float previous_angle_rad;
  uint32_t previous_time_us;
  bool has_previous;
  bool fault_latched;
} foc_angle_tracker_t;

bool foc_angle_tracker_init(foc_angle_tracker_t *tracker,
                            float max_speed_rad_s,
                            float step_tolerance_rad,
                            uint32_t max_gap_us);

/* Only FOC_ANGLE_VALID may be used for closed-loop torque control. */
foc_angle_status_t foc_angle_tracker_update(foc_angle_tracker_t *tracker,
                                             bool frame_valid,
                                             float angle_rad,
                                             uint32_t sample_time_us,
                                             float *speed_rad_s);

#ifdef __cplusplus
}
#endif

#endif /* FOC_ANGLE_TRACKER_H */
