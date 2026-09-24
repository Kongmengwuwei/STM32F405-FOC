#include "foc_speed_loop.h"

#include <math.h>
#include <stddef.h>

static float clampf(float value, float low, float high)
{
  return value < low ? low : (value > high ? high : value);
}

bool foc_speed_loop_init(foc_speed_loop_t *loop,
                         const foc_speed_loop_config_t *config)
{
  if (loop == NULL || config == NULL ||
      !isfinite(config->kp_amps_per_rad_s) ||
      !isfinite(config->ki_amps_per_rad) ||
      !isfinite(config->period_seconds) ||
      !isfinite(config->target_rad_s) ||
      !isfinite(config->max_iq_amps) ||
      !isfinite(config->max_forward_rad_s) ||
      !isfinite(config->max_reverse_rad_s) ||
      !isfinite(config->filter_alpha) ||
      config->kp_amps_per_rad_s < 0.0f ||
      config->ki_amps_per_rad < 0.0f ||
      config->period_seconds <= 0.0f ||
      config->target_rad_s <= 0.0f ||
      config->max_iq_amps <= 0.0f ||
      config->max_forward_rad_s <= config->target_rad_s ||
      config->max_reverse_rad_s <= 0.0f ||
      config->filter_alpha <= 0.0f ||
      config->filter_alpha > 1.0f)
  {
    return false;
  }
  *loop = (foc_speed_loop_t){.config = *config, .ready = true};
  return true;
}

bool foc_speed_loop_step(foc_speed_loop_t *loop,
                         float measured_forward_rad_s,
                         float *iq_target_amps)
{
  if (loop == NULL || !loop->ready || iq_target_amps == NULL ||
      !isfinite(measured_forward_rad_s) ||
      fabsf(measured_forward_rad_s) > 100.0f)
  {
    return false;
  }
  loop->filtered_rad_s += loop->config.filter_alpha *
      (measured_forward_rad_s - loop->filtered_rad_s);
  if (!isfinite(loop->filtered_rad_s) ||
      loop->filtered_rad_s > loop->config.max_forward_rad_s ||
      loop->filtered_rad_s < -loop->config.max_reverse_rad_s)
  {
    return false;
  }
  const float error = loop->config.target_rad_s - loop->filtered_rad_s;
  const float trial = loop->config.kp_amps_per_rad_s * error +
                      loop->integral_amps;
  const float output = clampf(trial, 0.0f, loop->config.max_iq_amps);
  if (trial == output ||
      (trial > output && error < 0.0f) ||
      (trial < output && error > 0.0f))
  {
    loop->integral_amps = clampf(
        loop->integral_amps + loop->config.ki_amps_per_rad *
            loop->config.period_seconds * error,
        0.0f, loop->config.max_iq_amps);
  }
  loop->iq_target_amps = output;
  *iq_target_amps = output;
  return true;
}
