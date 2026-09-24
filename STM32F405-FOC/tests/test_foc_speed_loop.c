#include "foc_speed_loop.h"

#include <assert.h>
#include <math.h>

int main(void)
{
  foc_speed_loop_t loop;
  const foc_speed_loop_config_t config = {
      .kp_amps_per_rad_s = 0.035f,
      .ki_amps_per_rad = 0.1f,
      .period_seconds = 0.001f,
      .target_rad_s = 6.28318531f,
      .max_iq_amps = 0.4f,
      .max_forward_rad_s = 12.5663706f,
      .max_reverse_rad_s = 3.14159265f,
      .filter_alpha = 0.05f};
  assert(foc_speed_loop_init(&loop, &config));
  float iq = 0.0f;
  assert(foc_speed_loop_step(&loop, 0.0f, &iq));
  assert(iq > 0.2f && iq < 0.4f);
  for (unsigned i = 0u; i < 1000u; ++i)
  {
    assert(foc_speed_loop_step(&loop, 0.0f, &iq));
    assert(iq >= 0.0f && iq <= 0.4f);
  }
  assert(iq == 0.4f);
  assert(loop.integral_amps <= 0.4f);
  for (unsigned i = 0u; i < 200u; ++i)
    assert(foc_speed_loop_step(&loop, 6.28318531f, &iq));
  assert(fabsf(loop.filtered_rad_s - config.target_rad_s) < 0.001f);
  assert(!foc_speed_loop_step(&loop, NAN, &iq));
  assert(!foc_speed_loop_step(&loop, 101.0f, &iq));

  assert(foc_speed_loop_init(&loop, &config));
  for (unsigned i = 0u; i < 200u; ++i)
  {
    if (!foc_speed_loop_step(&loop, 20.0f, &iq)) break;
  }
  assert(loop.filtered_rad_s > config.max_forward_rad_s);

  assert(foc_speed_loop_init(&loop, &config));
  for (unsigned i = 0u; i < 200u; ++i)
  {
    if (!foc_speed_loop_step(&loop, -5.0f, &iq)) break;
  }
  assert(loop.filtered_rad_s < -config.max_reverse_rad_s);
  return 0;
}
