#ifndef FOC_SPEED_LOOP_H
#define FOC_SPEED_LOOP_H

#include <stdbool.h>

typedef struct
{
  float kp_amps_per_rad_s;
  float ki_amps_per_rad;
  float period_seconds;
  float target_rad_s;
  float max_iq_amps;
  float max_forward_rad_s;
  float max_reverse_rad_s;
  float filter_alpha;
} foc_speed_loop_config_t;

typedef struct
{
  foc_speed_loop_config_t config;
  float filtered_rad_s;
  float integral_amps;
  float iq_target_amps;
  bool ready;
} foc_speed_loop_t;

bool foc_speed_loop_init(foc_speed_loop_t *loop,
                         const foc_speed_loop_config_t *config);
bool foc_speed_loop_step(foc_speed_loop_t *loop,
                         float measured_forward_rad_s,
                         float *iq_target_amps);

#endif
