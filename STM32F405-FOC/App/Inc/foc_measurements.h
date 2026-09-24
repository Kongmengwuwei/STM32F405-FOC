#ifndef FOC_MEASUREMENTS_H
#define FOC_MEASUREMENTS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float bus_volts_per_count;
  float phase_amps_per_count;
} foc_measurement_scale_t;

/* For a 12-bit ADC. All electrical parameters must be positive and finite. */
bool foc_measurement_scale_init(foc_measurement_scale_t *scale,
                                float vdda_volts,
                                float divider_high_ohms,
                                float divider_low_ohms,
                                float shunt_ohms,
                                float amplifier_gain);

float foc_bus_voltage_from_adc(uint16_t raw,
                               const foc_measurement_scale_t *scale);

/* polarity is +1 or -1 after physical current-direction calibration. */
float foc_phase_current_from_adc(uint16_t raw, float zero_count,
                                 float polarity,
                                 const foc_measurement_scale_t *scale);

#ifdef __cplusplus
}
#endif

#endif /* FOC_MEASUREMENTS_H */
