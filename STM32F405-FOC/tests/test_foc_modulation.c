#include "foc_modulation.h"

#include <assert.h>
#include <math.h>

static void near(float actual, float expected)
{
  assert(fabsf(actual - expected) < 0.00001f);
}

int main(void)
{
  foc_pwm_duty_t duty;
  assert(!foc_svpwm_compute(0.0f, 0.0f, 0.0f, 0.05f, &duty));
  assert(!foc_svpwm_compute(0.0f, 0.0f, 12.0f, 0.5f, &duty));

  assert(foc_svpwm_compute(0.0f, 0.0f, 12.0f, 0.05f, &duty));
  near(duty.a, 0.5f);
  near(duty.b, 0.5f);
  near(duty.c, 0.5f);
  assert(!duty.saturated);

  assert(foc_svpwm_compute(1.0f, 0.0f, 12.0f, 0.05f, &duty));
  near(duty.a, 0.5625f);
  near(duty.b, 0.4375f);
  near(duty.c, 0.4375f);
  assert(!duty.saturated);

  assert(foc_svpwm_compute(20.0f, 0.0f, 12.0f, 0.05f, &duty));
  near(duty.a, 0.95f);
  near(duty.b, 0.05f);
  near(duty.c, 0.05f);
  assert(duty.saturated);
  return 0;
}
