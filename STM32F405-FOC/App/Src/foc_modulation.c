#include "foc_modulation.h"

#include <math.h>

static float maximum(float a, float b)
{
  return a > b ? a : b;
}

static float minimum(float a, float b)
{
  return a < b ? a : b;
}

static float clamp(float value, float low, float high)
{
  return maximum(low, minimum(value, high));
}

bool foc_svpwm_compute(float alpha_volts, float beta_volts,
                       float bus_volts, float min_duty,
                       foc_pwm_duty_t *duty)
{
  if (duty == 0 || !isfinite(alpha_volts) || !isfinite(beta_volts) ||
      !isfinite(bus_volts) || !isfinite(min_duty) ||
      bus_volts <= 0.0f || min_duty < 0.0f || min_duty >= 0.5f)
  {
    return false;
  }

  const float half_sqrt_three = 0.8660254037844386f;
  const float va = alpha_volts;
  const float vb = -0.5f * alpha_volts + half_sqrt_three * beta_volts;
  const float vc = -0.5f * alpha_volts - half_sqrt_three * beta_volts;
  const float vmax = maximum(va, maximum(vb, vc));
  const float vmin = minimum(va, minimum(vb, vc));
  const float span = vmax - vmin;
  const float available_span = bus_volts * (1.0f - 2.0f * min_duty);
  if (!isfinite(span) || !isfinite(available_span) || available_span <= 0.0f)
  {
    return false;
  }

  const bool saturated = span > available_span;
  const float scale = saturated ? available_span / span : 1.0f;
  const float common_mode = -0.5f * (vmax + vmin);
  const float inverse_bus = scale / bus_volts;
  const float max_duty = 1.0f - min_duty;
  if (!isfinite(common_mode) || !isfinite(inverse_bus))
  {
    return false;
  }

  *duty = (foc_pwm_duty_t){
    .a = clamp(0.5f + (va + common_mode) * inverse_bus,
               min_duty, max_duty),
    .b = clamp(0.5f + (vb + common_mode) * inverse_bus,
               min_duty, max_duty),
    .c = clamp(0.5f + (vc + common_mode) * inverse_bus,
               min_duty, max_duty),
    .saturated = saturated,
  };
  return true;
}
