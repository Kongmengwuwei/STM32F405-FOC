#ifndef FOC_MODULATION_H
#define FOC_MODULATION_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float a;
  float b;
  float c;
  bool saturated;
} foc_pwm_duty_t;

/* Centered space-vector modulation, with room reserved at both duty edges. */
bool foc_svpwm_compute(float alpha_volts, float beta_volts,
                       float bus_volts, float min_duty,
                       foc_pwm_duty_t *duty);

#ifdef __cplusplus
}
#endif

#endif /* FOC_MODULATION_H */
