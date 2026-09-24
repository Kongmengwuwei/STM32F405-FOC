#include "foc_measurements.h"

#include <assert.h>
#include <math.h>

static void near(float actual, float expected, float tolerance)
{
  assert(fabsf(actual - expected) < tolerance);
}

int main(void)
{
  foc_measurement_scale_t scale = {0};
  assert(!foc_measurement_scale_init(&scale, 0.0f, 39000.0f, 2200.0f,
                                     0.001f, 20.0f));
  assert(foc_measurement_scale_init(&scale, 3.3f, 39000.0f, 2200.0f,
                                    0.001f, 20.0f));

  near(foc_bus_voltage_from_adc(0, &scale), 0.0f, 0.00001f);
  near(foc_bus_voltage_from_adc(4095, &scale),
       3.3f * 41200.0f / 2200.0f, 0.0001f);

  const float zero_count = 2048.0f;
  near(foc_phase_current_from_adc(2048, zero_count, 1.0f, &scale),
       0.0f, 0.00001f);
  near(foc_phase_current_from_adc(2073, zero_count, 1.0f, &scale),
       25.0f * 3.3f / 4095.0f / (0.001f * 20.0f), 0.0001f);
  near(foc_phase_current_from_adc(2073, zero_count, -1.0f, &scale),
       -25.0f * 3.3f / 4095.0f / (0.001f * 20.0f), 0.0001f);
  return 0;
}
