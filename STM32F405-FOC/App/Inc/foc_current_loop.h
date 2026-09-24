#ifndef FOC_CURRENT_LOOP_H
#define FOC_CURRENT_LOOP_H

#include <stdbool.h>

#include "foc_modulation.h"
#include "foc_transforms.h"

typedef struct
{
  float kp_volts_per_amp;
  float ki_volts_per_amp_second;
  float period_seconds;
  float max_phase_current_amps;
  float max_target_amps;
  float max_voltage_fraction;
  float minimum_duty;
} foc_current_loop_config_t;

typedef struct
{
  foc_current_loop_config_t config;
  float integral_d_volts;
  float integral_q_volts;
  foc_dq_t measured_amps;
  foc_dq_t commanded_volts;
  bool voltage_limited;
  bool ready;
} foc_current_loop_t;

bool foc_current_loop_init(foc_current_loop_t *loop,
                           const foc_current_loop_config_t *config);
void foc_current_loop_reset(foc_current_loop_t *loop);

/* ib/ic and the electrical angle must already have a verified polarity,
 * phase order, and zero offset. A false return is a latched drive fault. */
bool foc_current_loop_step(foc_current_loop_t *loop,
                            float phase_b_amps, float phase_c_amps,
                            float electrical_angle_rad, float bus_volts,
                            float id_target_amps, float iq_target_amps,
                            foc_pwm_duty_t *duty);

#endif
