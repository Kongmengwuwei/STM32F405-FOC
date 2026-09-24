#include "foc_current_loop.h"

#include <assert.h>
#include <math.h>

static bool near(float a, float b, float tolerance)
{
  return fabsf(a - b) < tolerance;
}

int main(void)
{
  foc_current_loop_t loop;
  const foc_current_loop_config_t config = {
      .kp_volts_per_amp = 1.0f,
      .ki_volts_per_amp_second = 100.0f,
      .period_seconds = 0.0001f,
      .max_phase_current_amps = 1.0f,
      .max_target_amps = 0.5f,
      .max_voltage_fraction = 0.3f,
      .minimum_duty = 0.2f};
  assert(foc_current_loop_init(&loop, &config));
  foc_pwm_duty_t duty;
  assert(foc_current_loop_step(&loop, 0.0f, 0.0f, 0.0f, 12.0f,
                                0.0f, 0.3f, &duty));
  assert(near(loop.measured_amps.d, 0.0f, 0.00001f));
  assert(near(loop.measured_amps.q, 0.0f, 0.00001f));
  assert(near(loop.commanded_volts.q, 0.3f, 0.00001f));
  assert(near(duty.a, 0.5f, 0.00001f));
  assert(duty.b > duty.a && duty.c < duty.a);
  assert(loop.integral_q_volts > 0.0f);

  foc_current_loop_reset(&loop);
  assert(near(loop.integral_q_volts, 0.0f, 0.00001f));
  assert(!foc_current_loop_step(&loop, 1.1f, 0.0f, 0.0f, 12.0f,
                                 0.0f, 0.3f, &duty));
  assert(!foc_current_loop_step(&loop, 0.0f, 0.0f, 0.0f, 12.0f,
                                 0.0f, 0.6f, &duty));
  assert(!foc_current_loop_step(&loop, 0.0f, 0.0f, 6.3f, 12.0f,
                                 0.0f, 0.3f, &duty));

  foc_current_loop_config_t saturating = config;
  saturating.kp_volts_per_amp = 100.0f;
  assert(foc_current_loop_init(&loop, &saturating));
  assert(foc_current_loop_step(&loop, 0.0f, 0.0f, 0.0f, 12.0f,
                                0.0f, 0.5f, &duty));
  assert(loop.voltage_limited);
  assert(near(hypotf(loop.commanded_volts.d, loop.commanded_volts.q),
              3.6f, 0.0001f));
  assert(near(loop.integral_q_volts, 0.0f, 0.00001f));
  assert(duty.a >= 0.2f && duty.a <= 0.8f);
  assert(duty.b >= 0.2f && duty.b <= 0.8f);
  assert(duty.c >= 0.2f && duty.c <= 0.8f);
  return 0;
}
