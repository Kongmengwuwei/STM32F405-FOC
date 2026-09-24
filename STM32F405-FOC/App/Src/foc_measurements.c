#include "foc_measurements.h"

#include <math.h>

bool foc_measurement_scale_init(foc_measurement_scale_t *scale,
                                float vdda_volts,
                                float divider_high_ohms,
                                float divider_low_ohms,
                                float shunt_ohms,
                                float amplifier_gain)
{
  if (scale == 0 || !isfinite(vdda_volts) || !isfinite(divider_high_ohms) ||
      !isfinite(divider_low_ohms) || !isfinite(shunt_ohms) ||
      !isfinite(amplifier_gain) || vdda_volts <= 0.0f ||
      divider_high_ohms <= 0.0f || divider_low_ohms <= 0.0f ||
      shunt_ohms <= 0.0f || amplifier_gain <= 0.0f)
  {
    return false;
  }

  const float adc_volts_per_count = vdda_volts / 4095.0f;
  const float bus_factor =
      (divider_high_ohms + divider_low_ohms) / divider_low_ohms;
  const float bus_scale = adc_volts_per_count * bus_factor;
  const float current_scale =
      adc_volts_per_count / (shunt_ohms * amplifier_gain);
  if (!isfinite(bus_scale) || !isfinite(current_scale) ||
      bus_scale <= 0.0f || current_scale <= 0.0f)
  {
    return false;
  }

  scale->bus_volts_per_count = bus_scale;
  scale->phase_amps_per_count = current_scale;
  return true;
}

float foc_bus_voltage_from_adc(uint16_t raw,
                               const foc_measurement_scale_t *scale)
{
  return (float)raw * scale->bus_volts_per_count;
}

float foc_phase_current_from_adc(uint16_t raw, float zero_count,
                                 float polarity,
                                 const foc_measurement_scale_t *scale)
{
  return ((float)raw - zero_count) * scale->phase_amps_per_count * polarity;
}
