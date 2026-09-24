#include "foc_current_loop.h"

#include <math.h>
#include <stddef.h>

static float clampf(float value, float low, float high)
{
  return value < low ? low : (value > high ? high : value);
}

bool foc_current_loop_init(foc_current_loop_t *loop,
                           const foc_current_loop_config_t *config)
{
  if (loop == NULL || config == NULL ||
      !isfinite(config->kp_volts_per_amp) ||
      !isfinite(config->ki_volts_per_amp_second) ||
      !isfinite(config->period_seconds) ||
      !isfinite(config->max_phase_current_amps) ||
      !isfinite(config->max_target_amps) ||
      !isfinite(config->max_voltage_fraction) ||
      !isfinite(config->minimum_duty) ||
      config->kp_volts_per_amp <= 0.0f ||
      config->ki_volts_per_amp_second < 0.0f ||
      config->period_seconds <= 0.0f ||
      config->max_phase_current_amps <= config->max_target_amps ||
      config->max_target_amps <= 0.0f ||
      config->max_voltage_fraction <= 0.0f ||
      config->max_voltage_fraction >= 0.5f ||
      config->minimum_duty < 0.0f ||
      config->minimum_duty >= 0.5f)
  {
    return false;
  }

  *loop = (foc_current_loop_t){.config = *config, .ready = true};
  return true;
}

void foc_current_loop_reset(foc_current_loop_t *loop)
{
  if (loop != NULL)
  {
    loop->integral_d_volts = 0.0f;
    loop->integral_q_volts = 0.0f;
    loop->measured_amps = (foc_dq_t){0};
    loop->commanded_volts = (foc_dq_t){0};
    loop->voltage_limited = false;
  }
}

bool foc_current_loop_step(foc_current_loop_t *loop,
                            float phase_b_amps, float phase_c_amps,
                            float electrical_angle_rad, float bus_volts,
                            float id_target_amps, float iq_target_amps,
                            foc_pwm_duty_t *duty)
{
  if (loop == NULL || !loop->ready || duty == NULL ||
      !isfinite(phase_b_amps) || !isfinite(phase_c_amps) ||
      !isfinite(electrical_angle_rad) || !isfinite(bus_volts) ||
      !isfinite(id_target_amps) || !isfinite(iq_target_amps) ||
      electrical_angle_rad < 0.0f ||
      electrical_angle_rad >= 6.2831853071795865f ||
      bus_volts < 6.0f || bus_volts > 20.0f ||
      fabsf(phase_b_amps) > loop->config.max_phase_current_amps ||
      fabsf(phase_c_amps) > loop->config.max_phase_current_amps ||
      fabsf(phase_b_amps + phase_c_amps) >
          loop->config.max_phase_current_amps ||
      hypotf(id_target_amps, iq_target_amps) >
          loop->config.max_target_amps)
  {
    return false;
  }

  const float sine = sinf(electrical_angle_rad);
  const float cosine = cosf(electrical_angle_rad);
  const foc_alpha_beta_t currents =
      foc_clarke_from_bc(phase_b_amps, phase_c_amps);
  const foc_dq_t measured = foc_park(currents, sine, cosine);
  if (!isfinite(measured.d) || !isfinite(measured.q))
  {
    return false;
  }

  const float error_d = id_target_amps - measured.d;
  const float error_q = iq_target_amps - measured.q;
  const float voltage_limit = bus_volts * loop->config.max_voltage_fraction;
  const float trial_d = loop->config.kp_volts_per_amp * error_d +
                        loop->integral_d_volts;
  const float trial_q = loop->config.kp_volts_per_amp * error_q +
                        loop->integral_q_volts;
  const float trial_magnitude = hypotf(trial_d, trial_q);
  if (!isfinite(trial_magnitude))
  {
    return false;
  }
  const float scale = trial_magnitude > voltage_limit
                          ? voltage_limit / trial_magnitude : 1.0f;
  const foc_dq_t command = {.d = trial_d * scale,
                            .q = trial_q * scale};
  const foc_alpha_beta_t voltage =
      foc_inverse_park(command, sine, cosine);
  foc_pwm_duty_t next_duty;
  if (!foc_svpwm_compute(voltage.alpha, voltage.beta, bus_volts,
                         loop->config.minimum_duty, &next_duty))
  {
    return false;
  }

  const float integration_step =
      loop->config.ki_volts_per_amp_second * loop->config.period_seconds;
  /* Integrate only when unsaturated or when error points back into the
   * voltage circle. This prevents windup at the intentionally low limit. */
  const bool can_integrate = scale == 1.0f ||
      trial_d * error_d + trial_q * error_q < 0.0f;
  if (can_integrate)
  {
    loop->integral_d_volts = clampf(
        loop->integral_d_volts + integration_step * error_d,
        -voltage_limit, voltage_limit);
    loop->integral_q_volts = clampf(
        loop->integral_q_volts + integration_step * error_q,
        -voltage_limit, voltage_limit);
  }

  loop->measured_amps = measured;
  loop->commanded_volts = command;
  loop->voltage_limited = scale < 1.0f || next_duty.saturated;
  *duty = next_duty;
  return true;
}
